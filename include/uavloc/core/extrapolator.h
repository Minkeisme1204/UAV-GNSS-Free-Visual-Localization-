#pragma once

// Extrapolator — the time-buffer that turns three independently-timed telemetry
// channels into one sample AT THE IMAGE TIMESTAMP (S6b). See
// .docs/designs/system_manager_design.md §3.3 and §3.4; shape borrowed from the
// kcb ExtrapolatorInterface (GetAHRSState / GetGimbalState / GetGNSSState).
//
// A source pushes attitude / gimbal / GNSS as they arrive (add*); when an image
// arrives, SystemManager (S6c) asks for the state at the image timestamp
// (get*). Nothing here triggers processing — this is a buffer, not a pipeline
// stage.
//
// ── The four rules, all of them load-bearing ─────────────────────────────────
//
// ① EXACT TIMESTAMP ⇒ THE STORED SAMPLE IS RETURNED VERBATIM, NEVER
//   INTERPOLATED (design §3.4). This is what keeps the bit-identical gate of
//   S6c alive: today VideoReader looks telemetry up by frame_id, tomorrow the
//   path goes through this buffer by TIME, and every dataset we own has exactly
//   one telemetry row per frame — so as long as an exact hit gives back that
//   very row, the numbers cannot move.
//
//   The comparison is `==` on double, ON PURPOSE, with NO epsilon. The value
//   travels FrameData::timestamp_msec → source adapter → add*() → get*() with
//   no arithmetic applied to it, so bit equality is exactly the right question.
//   A tolerance would silently merge two nearby samples into one and would hide
//   the day someone starts transforming timestamps.
//
// ② SLERP FOR ANGLES, LINEAR FOR EVERYTHING ELSE. Attitude (roll/pitch/yaw)
//   and gimbal (pan/tilt) are converted to a quaternion, interpolated with
//   slerp along the shortest arc, and converted back; GNSS (lat/lon/alt/speed)
//   is interpolated component-wise. Linear interpolation of Euler angles is
//   wrong across the 0/360° seam — the project already has that exact case
//   (pan 359° → 3°, see the DeltaYawFactor wrap handling).
//
// ③ NO EXTRAPOLATION. A query outside the buffered interval returns false, and
//   the caller decides what to do. Extrapolating attitude through a turn is a
//   large error, not a small one: 50°/s between two keyframes was measured on
//   MUN-FRL. The optional `max_gap_sec` (S6c) extends the same refusal to a
//   query that falls INSIDE a hole in the buffer.
//
// ④ THREAD-SAFE. Each of the three buffers has its OWN std::mutex, so the
//   channels never contend with each other. A plain mutex, not a shared_mutex:
//   the load is ~one add and ~one get per sample per channel (a ~1:1
//   read/write ratio, not the many-readers case shared_mutex pays off for) and
//   each critical section is a binary search over a few hundred entries plus a
//   copy — shorter than the extra cost of taking a shared_mutex.
//
// ── Angle conventions ────────────────────────────────────────────────────────
// Attitude uses the ZYX (yaw→pitch→roll) chain, gimbal the ZY (pan→tilt) chain,
// i.e. the same conventions sensor::TelemetryData documents. An INTERPOLATED
// result is re-extracted from the rotation, so its angles come back wrapped:
// roll/yaw/pan/tilt in (-180, 180], pitch in [-90, 90]. An EXACT hit is a
// verbatim copy and keeps whatever range the producer used (e.g. yaw = 359).
//
// Two documented limits of that round trip, neither reachable in normal flight:
//   * attitude at pitch = ±90° is the ZYX gimbal lock — roll and yaw are not
//     separable there, so an interpolated result near it is unreliable;
//   * an input pitch outside [-90, 90] does not round-trip.
//
// ⚠ GNSS longitude is interpolated LINEARLY, so a pair of samples straddling
// the ±180° meridian would interpolate the long way round. The design (§3.3)
// prescribes linear interpolation for position and no dataset of this project
// goes near the antimeridian; fixing it is left to whoever first needs it.

#include "uavloc/sensor/stream_types.h"

#include <cstddef>
#include <memory>

namespace uavloc {
namespace core {

//! How a get*() query was answered. The caller NEEDS this: rule ① hands an
//! exact hit back VERBATIM, in whatever range the producer used, while an
//! interpolated sample comes back wrapped by the quaternion round trip. A
//! consumer that wants to normalise angle ranges must therefore leave the exact
//! hit alone — normalising it would move the numbers of the datasets whose
//! producer uses [0, 360) (MUN-FRL yaw) and break the S6c bit-identical gate.
enum class SampleOrigin {
    EXACT,        //!< a stored sample with exactly that timestamp (rule ①)
    INTERPOLATED  //!< slerp / lerp between the two neighbours (rule ②)
};

class Extrapolator {
public:
    Extrapolator() = delete;

    //! `buffer_span_sec` bounds the buffers IN TIME: a sample older than the
    //! newest sample of its own channel by more than this span is dropped (and
    //! one arriving already that old is refused). <= 0 means unbounded, which
    //! grows without limit on a long flight and is therefore logged as a
    //! warning.
    //!
    //! `max_gap_sec` bounds INTERPOLATION, not the buffer: when the two samples
    //! bracketing a query are further apart than this, the query fails instead
    //! of interpolating across the hole. <= 0 (the default, and the S6b
    //! behaviour) means "no limit". Rule ③ already refuses to extrapolate past
    //! the ends of the buffer; this is the same argument applied INSIDE it — a
    //! channel that went silent for seconds leaves two samples that still
    //! bracket the query, and slerping between them invents an attitude that
    //! was never measured.
    explicit Extrapolator(double buffer_span_sec, double max_gap_sec = 0.0);

    ~Extrapolator();

    Extrapolator(const Extrapolator&)            = delete;
    Extrapolator& operator=(const Extrapolator&) = delete;

    // ── filling the buffers ──────────────────────────────────────────────────

    //! Buffer one sample at `t_msec`. Callable from any thread.
    //!
    //! Out-of-order arrival is ACCEPTED and inserted at its sorted position
    //! (the common in-order case stays an O(1) push_back); a sample whose
    //! timestamp is already present REPLACES the stored one (last write wins,
    //! logged) so that rule ① never has two candidates to choose from. A
    //! non-finite timestamp, or one older than the buffer span allows, is
    //! refused and logged.
    void addAttitude(double t_msec, const sensor::AttitudeData& att);
    void addGimbal(double t_msec, const sensor::GimbalData& gim);
    void addGnss(double t_msec, const sensor::GnssData& gnss);

    // ── querying at an image timestamp ───────────────────────────────────────

    //! State at `t_msec`, written to `out`. Returns true only when `t_msec`
    //! lies inside the buffered interval: an exact hit yields the stored sample
    //! verbatim (rule ①), otherwise the two neighbours are interpolated (rule
    //! ②). Returns false — leaving `out` untouched — when the buffer is empty,
    //! `t_msec` is outside it (NOTHING is extrapolated, rule ③), or the two
    //! neighbours are further apart than `max_gap_sec`.
    //!
    //! `origin`, when not null, is written ONLY on success and says which of
    //! the two rules produced `out` — see SampleOrigin.
    bool getAttitude(double t_msec, sensor::AttitudeData& out,
                     SampleOrigin* origin = nullptr) const;
    bool getGimbal(double t_msec, sensor::GimbalData& out,
                   SampleOrigin* origin = nullptr) const;
    bool getGnss(double t_msec, sensor::GnssData& out,
                 SampleOrigin* origin = nullptr) const;

    // ── housekeeping ─────────────────────────────────────────────────────────

    //! Drop every buffered sample of all three channels.
    void clear();

    //! Number of buffered samples of one channel (for SystemStats and tests).
    std::size_t attitudeCount() const;
    std::size_t gimbalCount() const;
    std::size_t gnssCount() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace core
} // namespace uavloc
