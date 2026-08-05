// test_extrapolator — unit test for core::Extrapolator (S6b,
// .docs/designs/system_manager_design.md §3.3 / §3.4).
//
// Checks:
//   E1  EXACT timestamp ⇒ the stored sample comes back with error EXACTLY 0
//       (compared with ==, not "close enough") on all three channels. This is
//       the condition the bit-identical gate of S6c rests on (§3.4), so a
//       stored yaw = 359.0 must come back as 359.0, unwrapped and untouched.
//   E2  interpolation between two samples equals the hand-computed weighted
//       average: GNSS component-wise linear, and single-axis rotations (yaw,
//       pitch, roll, pan, tilt) where slerp has one exact expected answer.
//   E3  THE 0/360° SEAM: 359° → 3° at alpha = 0.5 gives ~1°, NOT the ~181° a
//       linear interpolation of Euler angles would give. Checked on attitude
//       yaw AND gimbal pan.
//   E4  the -179° → 179° seam gives ~180° (either sign — same angle), not 0°.
//   E5  a query BEFORE the first sample returns false and leaves `out` alone.
//   E6  a query AFTER the last sample returns false — no extrapolation (§3.3).
//   E7  an empty buffer answers false on every channel and does not crash.
//   E8  with a SINGLE sample: an exact hit succeeds, anything else fails.
//   E9  buffer_span_sec trims: samples older than the span are dropped, the
//       counts are exact, a query into the trimmed region fails, and a sample
//       that arrives already too old is refused. span <= 0 keeps everything.
//   E10 THREAD SAFETY: 4 writer threads and 4 reader threads hammering the same
//       object concurrently — no crash, no deadlock (hard deadline: a hang is a
//       FAIL, not a wait).
//   E11 OUT-OF-ORDER arrival: a late sample is INSERTED at its sorted position
//       (proved by interpolating across it), and a repeated timestamp REPLACES
//       the stored sample (last write wins) instead of piling up a second
//       candidate for the exact-match rule.
//
// Pure unit test: no data files, no config, headless, exits non-zero on failure.

#include "uavloc/core/extrapolator.h"
#include "uavloc/sensor/stream_types.h"

#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <future>
#include <string>
#include <thread>
#include <vector>

namespace {

using uavloc::core::Extrapolator;
using uavloc::sensor::AttitudeData;
using uavloc::sensor::GimbalData;
using uavloc::sensor::GnssData;

using Ms = std::chrono::milliseconds;

// ── Test tunables (no magic numbers inline) ──────────────────────────────────

//! Buffer span used by every check that does not test trimming itself. Far
//! wider than any timestamp these checks use, so nothing is ever trimmed.
constexpr double WIDE_SPAN_SEC = 3600.0;

//! Span of the trimming check E9, and the sample spacing it uses [ms].
constexpr double TRIM_SPAN_SEC = 2.0;
constexpr double TRIM_STEP_MS  = 1000.0;

//! Tolerance for a value that went through a quaternion round trip [deg].
//! Only floating-point noise lives below this; every real error this test can
//! catch (a linear-Euler interpolation across the seam, a wrong axis order) is
//! orders of magnitude larger.
constexpr double ANGLE_EPS_DEG = 1e-9;

//! Tolerance for a linearly interpolated GNSS component. Degrees / metres /
//! metres-per-second all share it; the arithmetic is a single lerp.
constexpr double LINEAR_EPS = 1e-12;

//! E12: the interpolation-gap limit and the hole it is tested against. The hole
//! is deliberately WIDER than the limit (10 s vs 1 s) and the narrow gap
//! deliberately below it, so neither verdict depends on a boundary case.
constexpr double MAX_GAP_SEC       = 1.0;
constexpr double MAX_GAP_HOLE_MS   = 10000.0;
constexpr double MAX_GAP_NARROW_MS = 500.0;

//! E10: threads, iterations and deadline of the concurrency check.
constexpr int WRITER_THREADS   = 4;
constexpr int READER_THREADS   = 4;
constexpr int SAMPLES_PER_WRITER = 20000;
constexpr Ms  THREAD_DEADLINE{10000};

bool g_ok = true;

bool check(bool cond, const std::string& name) {
    if (cond) {
        spdlog::info("PASS: {}", name);
    } else {
        spdlog::error("FAIL: {}", name);
        g_ok = false;
    }
    return cond;
}

bool near_eq(double a, double b, double eps) {
    return std::fabs(a - b) <= eps;
}

//! Difference between two angles, wrapped to (-180, 180] [deg]. Used so that
//! -180 and +180 compare equal (they are the same angle).
double angle_diff_deg(double a, double b) {
    double d = std::fmod(a - b + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

bool angle_eq(double a, double b, double eps = ANGLE_EPS_DEG) {
    return std::fabs(angle_diff_deg(a, b)) <= eps;
}

//! Runs `fn` on its own thread and hard-fails the process if it does not return
//! within `deadline` — a deadlocked Extrapolator must be reported, not waited
//! on.
void run_with_deadline(const std::string& name, const std::function<void()>& fn, Ms deadline) {
    std::promise<void> done;
    std::future<void>  fut = done.get_future();
    std::thread        th([&] {
        fn();
        done.set_value();
    });
    if (fut.wait_for(deadline) != std::future_status::ready) {
        spdlog::error("FAIL: {} did not finish within {} ms — HANG", name, deadline.count());
        spdlog::default_logger()->flush();
        std::_Exit(1);
    }
    th.join();
    check(true, name + " finished within its deadline");
}

AttitudeData make_attitude(double roll, double pitch, double yaw) {
    AttitudeData a;
    a.roll_deg  = roll;
    a.pitch_deg = pitch;
    a.yaw_deg   = yaw;
    return a;
}

GimbalData make_gimbal(double pan, double tilt) {
    GimbalData g;
    g.pan_deg  = pan;
    g.tilt_deg = tilt;
    return g;
}

GnssData make_gnss(double lat, double lon, double alt, double speed) {
    GnssData g;
    g.latitude_deg  = lat;
    g.longitude_deg = lon;
    g.altitude_m    = alt;
    g.speed_mps     = speed;
    return g;
}

// ── E1: exact timestamp ⇒ exactly the stored sample ──────────────────────────

void test_exact_match_is_bit_exact() {
    Extrapolator ex(WIDE_SPAN_SEC);

    // Two samples per channel, so the exact query has a neighbour it could
    // wrongly interpolate with. yaw/pan are stored UNWRAPPED (359) — an exact
    // hit must not normalise them either.
    const AttitudeData a0 = make_attitude(1.25, -2.5, 359.0);
    const AttitudeData a1 = make_attitude(-3.75, 4.5, 3.0);
    const GimbalData   m0 = make_gimbal(359.0, 90.0);
    const GimbalData   m1 = make_gimbal(3.0, 80.0);
    const GnssData     g0 = make_gnss(47.5711234567, -52.7012345678, 123.456, 12.5);
    const GnssData     g1 = make_gnss(47.5721234567, -52.7002345678, 124.567, 13.5);

    const double t0 = 1234567.891;
    const double t1 = 1234667.891;

    ex.addAttitude(t0, a0);
    ex.addAttitude(t1, a1);
    ex.addGimbal(t0, m0);
    ex.addGimbal(t1, m1);
    ex.addGnss(t0, g0);
    ex.addGnss(t1, g1);

    AttitudeData a_out;
    GimbalData   m_out;
    GnssData     g_out;

    const bool got = ex.getAttitude(t0, a_out) && ex.getGimbal(t0, m_out) && ex.getGnss(t0, g_out);

    // == on purpose: the error must be EXACTLY zero, not small (design §3.4).
    const bool att_exact = a_out.roll_deg == a0.roll_deg && a_out.pitch_deg == a0.pitch_deg &&
                           a_out.yaw_deg == a0.yaw_deg;
    const bool gim_exact = m_out.pan_deg == m0.pan_deg && m_out.tilt_deg == m0.tilt_deg;
    const bool gnss_exact = g_out.latitude_deg == g0.latitude_deg &&
                            g_out.longitude_deg == g0.longitude_deg &&
                            g_out.altitude_m == g0.altitude_m && g_out.speed_mps == g0.speed_mps;

    check(got && att_exact && gim_exact && gnss_exact,
          "E1 exact timestamp returns the stored sample with error EXACTLY 0 (all 3 channels)");

    // The LAST sample is an exact hit too (the upper bound of the interval).
    AttitudeData a_last;
    const bool   last_ok = ex.getAttitude(t1, a_last) && a_last.yaw_deg == a1.yaw_deg &&
                         a_last.pitch_deg == a1.pitch_deg && a_last.roll_deg == a1.roll_deg;
    check(last_ok, "E1b the newest sample is an exact hit as well");
}

// ── E2: interpolation equals the hand-computed value ─────────────────────────

void test_interpolation_values() {
    // GNSS: component-wise linear at alpha = 0.25.
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addGnss(1000.0, make_gnss(10.0, 20.0, 100.0, 4.0));
        ex.addGnss(2000.0, make_gnss(14.0, 24.0, 200.0, 8.0));

        GnssData   out;
        const bool ok = ex.getGnss(1250.0, out);
        check(ok && near_eq(out.latitude_deg, 11.0, LINEAR_EPS) &&
                  near_eq(out.longitude_deg, 21.0, LINEAR_EPS) &&
                  near_eq(out.altitude_m, 125.0, LINEAR_EPS) &&
                  near_eq(out.speed_mps, 5.0, LINEAR_EPS),
              "E2a GNSS interpolates linearly (alpha = 0.25 ⇒ 11 / 21 / 125 / 5)");
    }

    // Attitude, one axis at a time: a rotation about a fixed axis has exactly
    // one slerp answer, so the expectation is hand-computable.
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addAttitude(0.0, make_attitude(0.0, 0.0, 10.0));
        ex.addAttitude(1000.0, make_attitude(0.0, 0.0, 50.0));
        AttitudeData out;
        const bool   ok = ex.getAttitude(250.0, out);
        check(ok && angle_eq(out.yaw_deg, 20.0) && angle_eq(out.pitch_deg, 0.0) &&
                  angle_eq(out.roll_deg, 0.0),
              "E2b attitude yaw 10 → 50 at alpha 0.25 ⇒ 20 deg");
    }
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addAttitude(0.0, make_attitude(0.0, -10.0, 0.0));
        ex.addAttitude(1000.0, make_attitude(0.0, 30.0, 0.0));
        AttitudeData out;
        const bool   ok = ex.getAttitude(500.0, out);
        check(ok && angle_eq(out.pitch_deg, 10.0) && angle_eq(out.yaw_deg, 0.0) &&
                  angle_eq(out.roll_deg, 0.0),
              "E2c attitude pitch -10 → 30 at alpha 0.5 ⇒ 10 deg");
    }
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addAttitude(0.0, make_attitude(-40.0, 0.0, 0.0));
        ex.addAttitude(1000.0, make_attitude(40.0, 0.0, 0.0));
        AttitudeData out;
        const bool   ok = ex.getAttitude(750.0, out);
        check(ok && angle_eq(out.roll_deg, 20.0) && angle_eq(out.pitch_deg, 0.0) &&
                  angle_eq(out.yaw_deg, 0.0),
              "E2d attitude roll -40 → 40 at alpha 0.75 ⇒ 20 deg");
    }

    // Gimbal, one axis at a time. The tilt case sits at 90 deg = nadir, the
    // singular direction of a naive pan/tilt extraction.
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addGimbal(0.0, make_gimbal(10.0, 90.0));
        ex.addGimbal(1000.0, make_gimbal(50.0, 90.0));
        GimbalData out;
        const bool ok = ex.getGimbal(250.0, out);
        check(ok && angle_eq(out.pan_deg, 20.0) && angle_eq(out.tilt_deg, 90.0),
              "E2e gimbal pan 10 → 50 at alpha 0.25 ⇒ 20 deg, tilt stays 90 (nadir)");
    }
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addGimbal(0.0, make_gimbal(120.0, 80.0));
        ex.addGimbal(1000.0, make_gimbal(120.0, 100.0));
        GimbalData out;
        const bool ok = ex.getGimbal(500.0, out);
        check(ok && angle_eq(out.tilt_deg, 90.0) && angle_eq(out.pan_deg, 120.0),
              "E2f gimbal tilt 80 → 100 at alpha 0.5 ⇒ 90 deg");
    }
}

// ── E3 / E4: the wrap-around seams ───────────────────────────────────────────

void test_wrap_around_seam() {
    // The case the project already has: pan 359 → 3.
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addAttitude(0.0, make_attitude(0.0, 0.0, 359.0));
        ex.addAttitude(1000.0, make_attitude(0.0, 0.0, 3.0));
        AttitudeData out;
        const bool   ok = ex.getAttitude(500.0, out);
        spdlog::info("E3 yaw 359 -> 3 at alpha 0.5 gives {:.9f} deg "
                     "(linear Euler would give 181)",
                     out.yaw_deg);
        check(ok && angle_eq(out.yaw_deg, 1.0) && std::fabs(angle_diff_deg(out.yaw_deg, 181.0)) > 90.0,
              "E3a attitude yaw 359 → 3 SLERPs to ~1 deg, not ~181");
    }
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addGimbal(0.0, make_gimbal(359.0, 90.0));
        ex.addGimbal(1000.0, make_gimbal(3.0, 90.0));
        GimbalData out;
        const bool ok = ex.getGimbal(500.0, out);
        spdlog::info("E3 pan 359 -> 3 at alpha 0.5 gives {:.9f} deg", out.pan_deg);
        check(ok && angle_eq(out.pan_deg, 1.0) && angle_eq(out.tilt_deg, 90.0),
              "E3b gimbal pan 359 → 3 SLERPs to ~1 deg");
    }

    // Same seam approached from the other side: -179 → 179 is 2 deg apart, so
    // the midpoint is ±180 (the same angle), never 0.
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addAttitude(0.0, make_attitude(0.0, 0.0, -179.0));
        ex.addAttitude(1000.0, make_attitude(0.0, 0.0, 179.0));
        AttitudeData out;
        const bool   ok = ex.getAttitude(500.0, out);
        spdlog::info("E4 yaw -179 -> 179 at alpha 0.5 gives {:.9f} deg", out.yaw_deg);
        check(ok && angle_eq(out.yaw_deg, 180.0) && !angle_eq(out.yaw_deg, 0.0, 1.0),
              "E4a attitude yaw -179 → 179 SLERPs to ~180 deg, not 0");
    }
    {
        Extrapolator ex(WIDE_SPAN_SEC);
        ex.addGimbal(0.0, make_gimbal(-179.0, 90.0));
        ex.addGimbal(1000.0, make_gimbal(179.0, 90.0));
        GimbalData out;
        const bool ok = ex.getGimbal(500.0, out);
        spdlog::info("E4 pan -179 -> 179 at alpha 0.5 gives {:.9f} deg", out.pan_deg);
        check(ok && angle_eq(out.pan_deg, 180.0) && !angle_eq(out.pan_deg, 0.0, 1.0),
              "E4b gimbal pan -179 → 179 SLERPs to ~180 deg, not 0");
    }
}

// ── E5 / E6: no extrapolation on either side ─────────────────────────────────

void test_no_extrapolation() {
    Extrapolator ex(WIDE_SPAN_SEC);
    ex.addAttitude(1000.0, make_attitude(1.0, 2.0, 3.0));
    ex.addAttitude(2000.0, make_attitude(4.0, 5.0, 6.0));
    ex.addGimbal(1000.0, make_gimbal(10.0, 90.0));
    ex.addGimbal(2000.0, make_gimbal(20.0, 90.0));
    ex.addGnss(1000.0, make_gnss(1.0, 2.0, 3.0, 4.0));
    ex.addGnss(2000.0, make_gnss(5.0, 6.0, 7.0, 8.0));

    const AttitudeData a_sentinel = make_attitude(-99.0, -99.0, -99.0);
    AttitudeData       a_out      = a_sentinel;
    GimbalData         m_out;
    GnssData           g_out;

    const bool before = !ex.getAttitude(999.999, a_out) && !ex.getGimbal(999.999, m_out) &&
                        !ex.getGnss(999.999, g_out);
    const bool untouched = a_out.roll_deg == a_sentinel.roll_deg &&
                           a_out.pitch_deg == a_sentinel.pitch_deg &&
                           a_out.yaw_deg == a_sentinel.yaw_deg;
    check(before && untouched, "E5 query before the first sample returns false, out untouched");

    const bool after = !ex.getAttitude(2000.001, a_out) && !ex.getGimbal(2000.001, m_out) &&
                       !ex.getGnss(2000.001, g_out) && !ex.getAttitude(1e9, a_out);
    check(after, "E6 query after the last sample returns false (no extrapolation)");
}

// ── E7: empty buffers ────────────────────────────────────────────────────────

void test_empty_buffer() {
    Extrapolator ex(WIDE_SPAN_SEC);
    AttitudeData a_out;
    GimbalData   m_out;
    GnssData     g_out;

    const bool all_false = !ex.getAttitude(0.0, a_out) && !ex.getGimbal(0.0, m_out) &&
                           !ex.getGnss(0.0, g_out) && !ex.getAttitude(1234.5, a_out);
    const bool zero_counts =
        ex.attitudeCount() == 0 && ex.gimbalCount() == 0 && ex.gnssCount() == 0;
    check(all_false && zero_counts, "E7 empty buffers answer false and count 0");

    // clear() on an empty object is a no-op, not a crash.
    ex.clear();
    check(ex.attitudeCount() == 0, "E7b clear() on an empty Extrapolator is safe");

    // And clear() really empties a filled one.
    ex.addAttitude(1.0, make_attitude(1.0, 2.0, 3.0));
    ex.addGimbal(1.0, make_gimbal(1.0, 2.0));
    ex.addGnss(1.0, make_gnss(1.0, 2.0, 3.0, 4.0));
    const bool filled = ex.attitudeCount() == 1 && ex.gimbalCount() == 1 && ex.gnssCount() == 1;
    ex.clear();
    const bool emptied = ex.attitudeCount() == 0 && ex.gimbalCount() == 0 &&
                         ex.gnssCount() == 0 && !ex.getAttitude(1.0, a_out);
    check(filled && emptied, "E7c clear() drops every buffered sample of all three channels");
}

// ── E8: a single sample ──────────────────────────────────────────────────────

void test_single_sample() {
    Extrapolator ex(WIDE_SPAN_SEC);
    const double t = 4242.125;
    ex.addAttitude(t, make_attitude(1.5, -2.5, 350.0));
    ex.addGnss(t, make_gnss(1.0, 2.0, 3.0, 4.0));

    AttitudeData a_out;
    GnssData     g_out;
    const bool   hit = ex.getAttitude(t, a_out) && a_out.yaw_deg == 350.0 &&
                     a_out.roll_deg == 1.5 && a_out.pitch_deg == -2.5 && ex.getGnss(t, g_out) &&
                     g_out.altitude_m == 3.0;
    const bool miss = !ex.getAttitude(t - 0.001, a_out) && !ex.getAttitude(t + 0.001, a_out) &&
                      !ex.getGnss(t - 0.001, g_out) && !ex.getGnss(t + 0.001, g_out);
    check(hit && miss, "E8 single sample: exact hit succeeds, any other instant fails");
}

// ── E9: buffer span trimming ─────────────────────────────────────────────────

void test_buffer_span_trim() {
    Extrapolator ex(TRIM_SPAN_SEC);

    // Samples at 0, 1000, 2000, 3000 ms with a 2000 ms span: once 3000 is in,
    // the horizon is 1000, so only 0 falls out.
    for (int k = 0; k <= 3; ++k) {
        const double t = k * TRIM_STEP_MS;
        ex.addAttitude(t, make_attitude(0.0, 0.0, static_cast<double>(k)));
        ex.addGimbal(t, make_gimbal(static_cast<double>(k), 90.0));
        ex.addGnss(t, make_gnss(static_cast<double>(k), 0.0, 0.0, 0.0));
    }

    const bool counts = ex.attitudeCount() == 3 && ex.gimbalCount() == 3 && ex.gnssCount() == 3;
    check(counts, "E9a samples older than buffer_span_sec are dropped, counts exact");

    AttitudeData a_out;
    const bool   trimmed_gone = !ex.getAttitude(0.0, a_out) && !ex.getAttitude(500.0, a_out);
    const bool   kept_ok      = ex.getAttitude(1000.0, a_out) && a_out.yaw_deg == 1.0 &&
                       ex.getAttitude(3000.0, a_out) && a_out.yaw_deg == 3.0;
    check(trimmed_gone && kept_ok, "E9b a query into the trimmed region fails, the rest still hits");

    // A sample that arrives ALREADY older than the span is refused outright.
    ex.addAttitude(0.0, make_attitude(0.0, 0.0, 99.0));
    check(ex.attitudeCount() == 3 && !ex.getAttitude(0.0, a_out),
          "E9c a sample arriving older than the span is refused");

    // span <= 0 means unbounded: nothing is ever trimmed.
    Extrapolator unbounded(0.0);
    for (int k = 0; k < 50; ++k) {
        unbounded.addGnss(k * TRIM_STEP_MS, make_gnss(static_cast<double>(k), 0.0, 0.0, 0.0));
    }
    GnssData g_out;
    check(unbounded.gnssCount() == 50 && unbounded.getGnss(0.0, g_out) && g_out.latitude_deg == 0.0,
          "E9d buffer_span_sec = 0 keeps every sample (unbounded)");
}

// ── E10: concurrent add / get ────────────────────────────────────────────────

void test_thread_safety() {
    Extrapolator     ex(WIDE_SPAN_SEC);
    std::atomic<bool> stop_readers{false};
    std::atomic<long long> reads_ok{0};

    run_with_deadline(
        "E10 concurrent add/get",
        [&] {
            std::vector<std::thread> workers;
            workers.reserve(WRITER_THREADS + READER_THREADS);

            // Writers interleave their timestamps (thread w owns t ≡ w mod
            // WRITER_THREADS), so every insertion races the others AND lands
            // out of order relative to them — exercising the sorted-insert path
            // under contention, not just push_back.
            for (int w = 0; w < WRITER_THREADS; ++w) {
                workers.emplace_back([&, w] {
                    for (int k = 0; k < SAMPLES_PER_WRITER; ++k) {
                        const double t = static_cast<double>(k * WRITER_THREADS + w);
                        ex.addAttitude(t, make_attitude(0.0, 0.0, static_cast<double>(k % 360)));
                        ex.addGimbal(t, make_gimbal(static_cast<double>(k % 360), 90.0));
                        ex.addGnss(t, make_gnss(static_cast<double>(k), 0.0, 0.0, 0.0));
                    }
                });
            }

            for (int r = 0; r < READER_THREADS; ++r) {
                workers.emplace_back([&, r] {
                    AttitudeData a;
                    GimbalData   m;
                    GnssData     g;
                    // Deterministic sweep (no shared RNG state) over the whole
                    // timestamp range, each reader starting at its own offset
                    // and stepping by a half instant so both the exact-hit and
                    // the interpolating path are exercised.
                    long long i = r;
                    while (!stop_readers.load()) {
                        const double t = 0.5 * static_cast<double>(
                                                   i++ % (2 * SAMPLES_PER_WRITER * WRITER_THREADS));
                        if (ex.getAttitude(t, a)) reads_ok.fetch_add(1);
                        (void)ex.getGimbal(t, m);
                        (void)ex.getGnss(t, g);
                        (void)ex.attitudeCount();
                    }
                });
            }

            for (int w = 0; w < WRITER_THREADS; ++w) {
                workers[static_cast<std::size_t>(w)].join();
            }
            stop_readers.store(true);
            for (std::size_t i = WRITER_THREADS; i < workers.size(); ++i) {
                workers[i].join();
            }
        },
        THREAD_DEADLINE);

    const std::size_t expected = static_cast<std::size_t>(SAMPLES_PER_WRITER) * WRITER_THREADS;
    check(ex.attitudeCount() == expected && ex.gimbalCount() == expected &&
              ex.gnssCount() == expected,
          "E10b every concurrently added sample is present exactly once");
    spdlog::info("E10 readers completed {} successful attitude queries", reads_ok.load());
}

// ── E11: out-of-order and duplicate timestamps ───────────────────────────────

void test_out_of_order_and_duplicates() {
    // Documented behaviour: a late sample is INSERTED at its sorted position
    // (not rejected), so the buffer stays time-sorted and the query path is
    // unaffected by arrival order.
    Extrapolator ex(WIDE_SPAN_SEC);
    ex.addGnss(100.0, make_gnss(0.0, 0.0, 0.0, 0.0));
    ex.addGnss(300.0, make_gnss(200.0, 0.0, 0.0, 0.0));
    ex.addGnss(200.0, make_gnss(100.0, 0.0, 0.0, 0.0));  // late arrival

    GnssData   out;
    const bool inserted = ex.gnssCount() == 3 && ex.getGnss(200.0, out) && out.latitude_deg == 100.0;
    // t = 250 tells the two behaviours apart: with the late sample in its
    // sorted place it is bracketed by (200 → 100) and (300 → 200) and gives
    // 150; had it simply been appended, the bracket would have been (100 → 0)
    // and (300 → 200) and given 125.
    const bool ordered = ex.getGnss(250.0, out) && near_eq(out.latitude_deg, 150.0, LINEAR_EPS);
    check(inserted && ordered, "E11a an out-of-order sample is inserted at its sorted position");

    // A repeated timestamp REPLACES the stored sample (last write wins): the
    // exact-match rule must never have two candidates.
    ex.addGnss(200.0, make_gnss(999.0, 0.0, 0.0, 0.0));
    const bool replaced = ex.gnssCount() == 3 && ex.getGnss(200.0, out) && out.latitude_deg == 999.0;
    check(replaced, "E11b a repeated timestamp replaces the stored sample (last write wins)");

    // Same rule for the newest timestamp (the push_back fast path).
    Extrapolator ex2(WIDE_SPAN_SEC);
    ex2.addAttitude(10.0, make_attitude(0.0, 0.0, 1.0));
    ex2.addAttitude(10.0, make_attitude(0.0, 0.0, 2.0));
    AttitudeData a_out;
    check(ex2.attitudeCount() == 1 && ex2.getAttitude(10.0, a_out) && a_out.yaw_deg == 2.0,
          "E11c a duplicate of the newest timestamp replaces it too");
}

// ── E12: max_gap_sec (S6c) ───────────────────────────────────────────────────

void test_max_gap() {
    // OFF (the default, and the S6b behaviour): a query in the middle of a
    // 10 s hole still interpolates. This half of the check is what pins
    // "extrapolator_max_gap_sec = 0 changes nothing".
    {
        Extrapolator off(WIDE_SPAN_SEC);  // max_gap defaulted to 0
        off.addGnss(0.0, make_gnss(0.0, 0.0, 0.0, 0.0));
        off.addGnss(MAX_GAP_HOLE_MS, make_gnss(100.0, 0.0, 0.0, 0.0));
        GnssData out;
        const bool interpolated = off.getGnss(0.5 * MAX_GAP_HOLE_MS, out) &&
                                  near_eq(out.latitude_deg, 50.0, LINEAR_EPS);
        check(interpolated, "E12a max_gap off (0): a query inside a 10 s hole "
                            "still interpolates");
    }

    // ON: the same buffer, the same query, now refused — and `out` untouched.
    Extrapolator ex(WIDE_SPAN_SEC, MAX_GAP_SEC);
    ex.addAttitude(0.0, make_attitude(0.0, 0.0, 0.0));
    ex.addAttitude(MAX_GAP_HOLE_MS, make_attitude(0.0, 0.0, 40.0));
    ex.addGimbal(0.0, make_gimbal(0.0, 90.0));
    ex.addGimbal(MAX_GAP_HOLE_MS, make_gimbal(40.0, 90.0));
    ex.addGnss(0.0, make_gnss(0.0, 0.0, 0.0, 0.0));
    ex.addGnss(MAX_GAP_HOLE_MS, make_gnss(100.0, 0.0, 0.0, 0.0));

    AttitudeData a_out = make_attitude(-1.0, -1.0, -1.0);
    GimbalData   m_out = make_gimbal(-1.0, -1.0);
    GnssData     g_out = make_gnss(-1.0, -1.0, -1.0, -1.0);
    const double mid   = 0.5 * MAX_GAP_HOLE_MS;
    const bool refused = !ex.getAttitude(mid, a_out) && !ex.getGimbal(mid, m_out) &&
                         !ex.getGnss(mid, g_out);
    const bool untouched = a_out.yaw_deg == -1.0 && m_out.pan_deg == -1.0 &&
                           g_out.latitude_deg == -1.0;
    check(refused && untouched,
          "E12b max_gap on: a query inside a too-wide gap fails on all three "
          "channels and leaves `out` alone");

    // The gap rule must NOT touch the exact-match rule: the two samples that
    // bound the hole are still returned verbatim.
    const bool ends_still_hit = ex.getAttitude(0.0, a_out) && a_out.yaw_deg == 0.0 &&
                                ex.getAttitude(MAX_GAP_HOLE_MS, a_out) &&
                                a_out.yaw_deg == 40.0;
    check(ends_still_hit, "E12c max_gap does not affect exact hits");

    // And a gap NARROWER than the limit still interpolates: the rule fires on
    // the bracket width, not on the mere presence of a limit.
    ex.addGnss(MAX_GAP_HOLE_MS + MAX_GAP_NARROW_MS,
               make_gnss(200.0, 0.0, 0.0, 0.0));
    const bool narrow_ok =
        ex.getGnss(MAX_GAP_HOLE_MS + 0.5 * MAX_GAP_NARROW_MS, g_out) &&
        near_eq(g_out.latitude_deg, 150.0, LINEAR_EPS);
    check(narrow_ok, "E12d a gap narrower than max_gap_sec still interpolates");
}

}  // namespace

int main() {
    spdlog::set_level(spdlog::level::info);

    test_exact_match_is_bit_exact();
    test_interpolation_values();
    test_wrap_around_seam();
    test_no_extrapolation();
    test_empty_buffer();
    test_single_sample();
    test_buffer_span_trim();
    test_thread_safety();
    test_out_of_order_and_duplicates();
    test_max_gap();

    if (!g_ok) {
        spdlog::error("test_extrapolator: FAIL");
        return 1;
    }
    spdlog::info("test_extrapolator: PASS");
    return 0;
}
