// test_anchor — unit test for the M1 absolute-position producer
// (anchor::FakeAnchor behind anchor::AnchorInterface).
//
// Seven checks, all on a groundtruth CSV this test WRITES into the current
// working directory (no dataset needed, nothing under config/ or data/ is
// touched):
//   1. setup() fails when no groundtruth CSV is configured;
//   2. NO fix is produced before setEnuOrigin() — a producer that has not been
//      told which ENU frame to answer in must stay silent, not guess;
//   3. the duty cycle holds: one fix every `every_kf` requests, and the first
//      one lands on request `every_kf`;
//   4. a request whose telemetry is unusable (agl_m <= 0) produces nothing;
//   5. the fix lands where the groundtruth is, to within the declared sigma,
//      and carries the query's timestamp verbatim (that timestamp is the ONLY
//      key the back-end has to attach the fix to a state);
//   6. same seed ⇒ same fixes, bit for bit; a different seed ⇒ different ones;
//   7. SYNCHRONOUS mode delivers the result BEFORE requestFix() returns, and
//      ASYNC mode delivers it on another thread after start();
//   8. RE-ANCHOR (AnchorRequestReason::REINIT): it bypasses the duty cycle, it
//      still obeys the three DATA gates, reinit_min_kf_gap throttles the bypass
//      without dropping the request, and one request produces one fix.
//
// Headless; exits non-zero on failure.

#include "uavloc/anchor/fake_anchor.h"

#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using uavloc::anchor::AnchorQuery;
using uavloc::anchor::AnchorRequestReason;
using uavloc::anchor::FakeAnchor;
using uavloc::anchor::FakeAnchorConfig;
using uavloc::anchor::FakeAnchorMode;

bool g_ok = true;

bool check(bool cond, const char* name) {
    if (cond) {
        spdlog::info("PASS: {}", name);
    } else {
        spdlog::error("FAIL: {}", name);
        g_ok = false;
    }
    return cond;
}

// ── the synthetic groundtruth log ────────────────────────────────────────────
// A straight eastward leg at a constant latitude: frame k sits at
// (lat0, lon0 + k * LON_STEP_DEG). Small enough that the flat-earth mapping is
// exact for the purpose of this test.
const char* const CSV_NAME     = "test_anchor_groundtruth.csv";
constexpr int     CSV_ROWS     = 400;
constexpr double  LAT0_DEG     = 45.0;
constexpr double  LON0_DEG     = -75.0;
constexpr double  LON_STEP_DEG = 1e-4;
//! Flat-earth constants of sensor::GeoReferencer, restated here so the expected
//! values are computed independently of the code under test.
constexpr double METERS_PER_DEG_LAT = 111319.5;

double meters_per_deg_lon() {
    return METERS_PER_DEG_LAT * std::cos(LAT0_DEG * M_PI / 180.0);
}

//! Groundtruth East [m] of frame `k` relative to the origin of frame 0.
double truth_east_m(int k) {
    return static_cast<double>(k) * LON_STEP_DEG * meters_per_deg_lon();
}

bool write_csv() {
    std::ofstream f(CSV_NAME);
    if (!f.is_open()) {
        return false;
    }
    f << "frame_id,roll_deg,pitch_deg,yaw_deg,gimbal_pan_deg,gimbal_tilt_deg,"
         "altitude_m,latitude_deg,longitude_deg,ground_speed_mps\n";
    f.precision(12);
    for (int k = 0; k < CSV_ROWS; ++k) {
        f << k << ",0,0,0,0,90,100.0," << LAT0_DEG << ','
          << (LON0_DEG + k * LON_STEP_DEG) << ",10\n";
    }
    return true;
}

FakeAnchorConfig base_config() {
    FakeAnchorConfig c;
    c.telemetry.csv_path   = CSV_NAME;
    c.telemetry.has_header = true;
    c.telemetry.columns    = {
        {uavloc::sensor::DroneTelemetryField::FRAME_ID, "frame_id", -1},
        {uavloc::sensor::DroneTelemetryField::ROLL_DEG, "roll_deg", -1},
        {uavloc::sensor::DroneTelemetryField::PITCH_DEG, "pitch_deg", -1},
        {uavloc::sensor::DroneTelemetryField::YAW_DEG, "yaw_deg", -1},
        {uavloc::sensor::DroneTelemetryField::GIMBAL_PAN_DEG, "gimbal_pan_deg", -1},
        {uavloc::sensor::DroneTelemetryField::GIMBAL_TILT_DEG, "gimbal_tilt_deg", -1},
        {uavloc::sensor::DroneTelemetryField::SENSOR_ALTITUDE_M, "altitude_m", -1},
        {uavloc::sensor::DroneTelemetryField::LATITUDE_DEG, "latitude_deg", -1},
        {uavloc::sensor::DroneTelemetryField::LONGITUDE_DEG, "longitude_deg", -1},
        {uavloc::sensor::DroneTelemetryField::GROUND_SPEED_MPS, "ground_speed_mps", -1},
    };
    c.sigma_m  = 5.0;
    c.every_kf = 10;
    c.seed     = 1234u;
    c.mode     = FakeAnchorMode::SYNCHRONOUS;
    return c;
}

AnchorQuery query_at(unsigned int frame_id) {
    AnchorQuery q;
    q.frame_id       = frame_id;
    q.timestamp_msec = 1000.0 + 40.0 * frame_id;
    q.agl_m          = 100.0;   // usable telemetry
    return q;
}

//! Drive `n` requests on frames 0..n-1 and return every fix that came out.
std::vector<uavloc::anchor::AbsoluteFix> drive(FakeAnchor& a, int n) {
    std::vector<uavloc::anchor::AbsoluteFix> out;
    a.setResultCallback([&out](const uavloc::anchor::AbsoluteFix& f) {
        out.push_back(f);
    });
    for (int k = 0; k < n; ++k) {
        a.requestFix(query_at(static_cast<unsigned int>(k)));
    }
    return out;
}

// ── 1. no groundtruth ⇒ setup() fails ────────────────────────────────────────
void test_setup_requires_groundtruth() {
    FakeAnchorConfig c = base_config();
    c.telemetry.csv_path.clear();
    FakeAnchor a(c);
    check(!a.setup(), "setup() fails without a groundtruth CSV");

    FakeAnchorConfig missing = base_config();
    missing.telemetry.csv_path = "test_anchor_this_file_does_not_exist.csv";
    FakeAnchor b(missing);
    check(!b.setup(), "setup() fails when the groundtruth CSV cannot be read");
}

// ── 2. silence until the owner supplies the ENU origin ───────────────────────
void test_no_origin_no_fix() {
    FakeAnchor a(base_config());
    if (!check(a.setup() && a.start(), "setup+start (no-origin case)")) {
        return;
    }
    const auto fixes = drive(a, 100);
    check(fixes.empty(), "no fix is produced before setEnuOrigin()");
    check(a.stats().skipped_no_origin > 0,
          "the missing origin is COUNTED, not silently ignored");
    a.stop();
}

// ── 3./4./5. cadence, telemetry gate, position + timestamp ───────────────────
void test_cadence_and_position() {
    FakeAnchorConfig c = base_config();
    FakeAnchor a(c);
    if (!check(a.setup() && a.start(), "setup+start (cadence case)")) {
        return;
    }
    a.setEnuOrigin(LAT0_DEG, LON0_DEG);

    std::vector<uavloc::anchor::AbsoluteFix> fixes;
    std::vector<double>       truth_e;
    std::vector<unsigned int> fix_frames;
    a.setResultCallback([&fixes](const uavloc::anchor::AbsoluteFix& f) {
        fixes.push_back(f);
    });
    for (int k = 0; k < 100; ++k) {
        AnchorQuery q = query_at(static_cast<unsigned int>(k));
        // Frames 19 and 20 carry no usable telemetry — and they are exactly the
        // ones the duty cycle would otherwise have fired on (requests 20/21),
        // so the gate is genuinely exercised instead of hiding behind cadence.
        if (k == 19 || k == 20) {
            q.agl_m = 0.0;
        }
        const std::size_t before = fixes.size();
        a.requestFix(q);
        if (fixes.size() > before) {
            truth_e.push_back(truth_east_m(k));
            fix_frames.push_back(static_cast<unsigned int>(k));
        }
    }

    // Requests are 1-based inside the producer: request 10 is frame 9. With
    // requests 20 and 21 refused for lack of telemetry the cadence slips to
    // 22, so the fixes land on requests 10, 22, 32, 42, ..., 92 — nine of them.
    check(fixes.size() == 9, "duty cycle + telemetry gate: 9 fixes out of 100 "
                             "requests");
    check(a.stats().skipped_no_telemetry == 2,
          "a request with agl_m <= 0 is refused and counted");

    bool all_close    = true;
    bool all_spd      = true;
    for (std::size_t i = 0; i < fixes.size() && i < truth_e.size(); ++i) {
        // 5 sigma on a 2-D Gaussian: a real failure (wrong frame, wrong row)
        // is orders of magnitude larger, so this is a detector, not a tuning.
        const double dx = fixes[i].xy_enu.x() - truth_e[i];
        const double dy = fixes[i].xy_enu.y();  // the leg is due east
        if (std::hypot(dx, dy) > 5.0 * c.sigma_m) all_close = false;
        if (!fixes[i].valid) all_spd = false;
        if (fixes[i].cov(0, 0) <= 0.0 || fixes[i].cov(1, 1) <= 0.0) all_spd = false;
    }
    check(all_close, "every fix sits within 5 sigma of the groundtruth");
    check(all_spd, "every fix is valid and declares a positive-definite cov");

    // Timestamp verbatim — the ONE key the back-end uses to attach a fix to a
    // state. Compared against the frame each fix was actually produced on, so
    // the check does not depend on predicting the duty cycle.
    bool all_stamped = fixes.size() == fix_frames.size();
    for (std::size_t i = 0; all_stamped && i < fixes.size(); ++i) {
        if (std::abs(fixes[i].timestamp_msec -
                     (1000.0 + 40.0 * fix_frames[i])) > 1e-9) {
            all_stamped = false;
        }
    }
    check(all_stamped, "every fix carries the QUERY's timestamp verbatim");
    a.stop();
}

// ── 6. reproducibility ───────────────────────────────────────────────────────
void test_seed_reproducibility() {
    auto run = [](unsigned int seed) {
        FakeAnchorConfig c = base_config();
        c.seed = seed;
        FakeAnchor a(c);
        a.setup();
        a.start();
        a.setEnuOrigin(LAT0_DEG, LON0_DEG);
        const auto fixes = drive(a, 100);
        a.stop();
        return fixes;
    };
    const auto x = run(1234u);
    const auto y = run(1234u);
    const auto z = run(4321u);

    bool same = x.size() == y.size();
    for (std::size_t i = 0; same && i < x.size(); ++i) {
        same = (x[i].xy_enu == y[i].xy_enu);
    }
    check(same && !x.empty(), "same seed reproduces the fixes bit for bit");

    bool differs = x.size() == z.size() && !x.empty();
    bool any_diff = false;
    for (std::size_t i = 0; differs && i < x.size(); ++i) {
        if (x[i].xy_enu != z[i].xy_enu) any_diff = true;
    }
    check(any_diff, "a different seed produces different fixes");
}

// ── 7. delivery modes ────────────────────────────────────────────────────────
void test_delivery_modes() {
    // SYNCHRONOUS: the callback must have fired before requestFix() returns.
    {
        FakeAnchor a(base_config());
        a.setup();
        a.start();
        a.setEnuOrigin(LAT0_DEG, LON0_DEG);
        int  delivered = 0;
        bool inside    = false;
        bool in_call   = false;
        a.setResultCallback([&](const uavloc::anchor::AbsoluteFix&) {
            ++delivered;
            inside = in_call;
        });
        for (int k = 0; k < 10; ++k) {
            in_call = true;
            a.requestFix(query_at(static_cast<unsigned int>(k)));
            in_call = false;
        }
        check(delivered == 1 && inside,
              "SYNCHRONOUS delivers the fix before requestFix() returns");
        a.stop();
    }

    // ASYNC: the callback fires on the producer's own thread.
    {
        FakeAnchorConfig c = base_config();
        c.mode     = FakeAnchorMode::ASYNC;
        c.every_kf = 1;   // one fix per request, so the queue actually works
        FakeAnchor a(c);
        a.setup();
        std::atomic<int>            delivered{0};
        std::atomic<bool>           other_thread{true};
        const std::thread::id       main_id = std::this_thread::get_id();
        a.setResultCallback([&](const uavloc::anchor::AbsoluteFix&) {
            if (std::this_thread::get_id() == main_id) other_thread = false;
            ++delivered;
        });
        a.start();
        a.setEnuOrigin(LAT0_DEG, LON0_DEG);
        for (int k = 0; k < 20; ++k) {
            a.requestFix(query_at(static_cast<unsigned int>(k)));
        }
        // stop() drains and joins, so no polling loop is needed.
        a.stop();
        check(delivered.load() > 0, "ASYNC delivers fixes");
        check(other_thread.load(), "ASYNC delivers them on the producer's thread");
    }
}

// ── 8. re-anchor (AnchorRequestReason::REINIT) ───────────────────────────────

//! Build a producer that is set up, started and given the ENU origin, so each
//! re-anchor case below starts from the same clean state.
struct Running {
    FakeAnchor                               anchor;
    std::vector<uavloc::anchor::AbsoluteFix> fixes;
    std::vector<bool>                        from_reinit;

    explicit Running(const FakeAnchorConfig& c) : anchor(c) {
        anchor.setup();
        anchor.start();
        anchor.setEnuOrigin(LAT0_DEG, LON0_DEG);
        anchor.setResultCallback([this](const uavloc::anchor::AbsoluteFix& f) {
            fixes.push_back(f);
        });
        anchor.setGenerationCallback(
            [this](const uavloc::anchor::FakeFixRecord& r) {
                from_reinit.push_back(r.from_reinit);
            });
    }
    ~Running() { anchor.stop(); }
};

//! A REINIT request must produce a fix even in the middle of the duty cycle —
//! the instant the VO chain restarts is the one an absolute measurement is
//! worth the most, and a cadence slot must not be what decides it.
void test_reinit_bypasses_cadence() {
    FakeAnchorConfig c = base_config();   // every_kf = 10
    Running r(c);

    // Requests 1..5 are ordinary and land between two cadence slots.
    for (unsigned int k = 0; k < 5; ++k) {
        r.anchor.requestFix(query_at(k));
    }
    check(r.fixes.empty(), "no cadence fix yet at request 5 of every_kf=10");

    AnchorQuery q = query_at(5);
    q.reason      = AnchorRequestReason::REINIT;
    r.anchor.requestFix(q);
    check(r.fixes.size() == 1, "a REINIT request bypasses the duty cycle");
    check(!r.from_reinit.empty() && r.from_reinit.back(),
          "the generated record is labelled from_reinit");

    const uavloc::anchor::FakeAnchorStats st = r.anchor.stats();
    check(st.reinit_requested == 1 && st.reinit_generated == 1,
          "the re-anchor request and fix are counted APART from the cadence ones");
    check(st.generated == 1, "and are also part of the total `generated`");

    // The cadence restarts from the re-anchor fix (request 6): requests 7..15
    // must stay silent, request 16 fires.
    for (unsigned int k = 6; k < 15; ++k) {
        r.anchor.requestFix(query_at(k));
    }
    check(r.fixes.size() == 1,
          "the duty-cycle counter is reset by a re-anchor fix (no fix right "
          "after it)");
    r.anchor.requestFix(query_at(15));
    check(r.fixes.size() == 2, "the ordinary cadence resumes 10 requests later");
}

//! The bypass covers the CADENCE only. Telemetry / origin / groundtruth are
//! DATA conditions: without them there is nothing to manufacture a fix from,
//! whatever the reason of the request.
void test_reinit_still_obeys_data_gates() {
    {   // no usable telemetry
        Running r(base_config());
        AnchorQuery q = query_at(3);
        q.reason      = AnchorRequestReason::REINIT;
        q.agl_m       = 0.0;
        r.anchor.requestFix(q);
        check(r.fixes.empty() && r.anchor.stats().skipped_no_telemetry == 1,
              "a REINIT with agl_m <= 0 still produces nothing");
    }
    {   // no groundtruth row for that frame
        Running r(base_config());
        AnchorQuery q = query_at(CSV_ROWS + 50);
        q.reason      = AnchorRequestReason::REINIT;
        r.anchor.requestFix(q);
        check(r.fixes.empty() && r.anchor.stats().skipped_no_groundtruth == 1,
              "a REINIT with no groundtruth row still produces nothing");
    }
    {   // no ENU origin — Running sets one, so build this producer by hand
        FakeAnchor a(base_config());
        a.setup();
        a.start();
        int delivered = 0;
        a.setResultCallback(
            [&delivered](const uavloc::anchor::AbsoluteFix&) { ++delivered; });
        AnchorQuery q = query_at(3);
        q.reason      = AnchorRequestReason::REINIT;
        a.requestFix(q);
        check(delivered == 0 && a.stats().skipped_no_origin == 1,
              "a REINIT before setEnuOrigin() still produces nothing");
        a.stop();
    }
}

//! reinit_min_kf_gap limits how often the bypass may be used. A throttled
//! REINIT is DEMOTED to an ordinary cadence request, never dropped.
void test_reinit_min_gap() {
    // gap (5) deliberately SHORTER than the duty cycle (10), so the last check
    // below can only pass through the bypass and not through the cadence.
    FakeAnchorConfig c  = base_config();   // every_kf = 10
    c.reinit_min_kf_gap = 5;
    Running r(c);

    AnchorQuery q1 = query_at(0);          // request 1
    q1.reason      = AnchorRequestReason::REINIT;
    r.anchor.requestFix(q1);
    check(r.fixes.size() == 1, "the FIRST re-anchor is never throttled");

    AnchorQuery q2 = query_at(1);          // request 2 — inside the gap
    q2.reason      = AnchorRequestReason::REINIT;
    r.anchor.requestFix(q2);
    check(r.fixes.size() == 1, "a second re-anchor inside the gap is throttled");
    check(r.anchor.stats().reinit_throttled == 1,
          "the throttled re-anchor is COUNTED, not silently dropped");
    check(r.anchor.stats().reinit_requested == 2,
          "a throttled re-anchor is still counted as a re-anchor REQUEST");

    // ... and it was demoted, not dropped: the ordinary cadence still applies,
    // so request 11 (ten after the last fix) produces the next one.
    for (unsigned int k = 2; k < 11; ++k) {   // requests 3..11
        r.anchor.requestFix(query_at(k));
    }
    check(r.fixes.size() == 2,
          "a throttled re-anchor falls back to the ordinary duty cycle");
    check(r.from_reinit.size() == 2 && !r.from_reinit.back(),
          "the fallback fix is NOT labelled from_reinit");

    // Request 12: one after a cadence fix, so the duty cycle would refuse it —
    // but 11 requests have passed since the last bypass, so the bypass is free.
    AnchorQuery q3 = query_at(11);
    q3.reason      = AnchorRequestReason::REINIT;
    r.anchor.requestFix(q3);
    check(r.fixes.size() == 3 && r.from_reinit.back(),
          "the bypass is available again once the gap has passed");
}

//! core::SystemManager sends ONE request for a frame that is both a keyframe
//! and the re-init frame, labelled REINIT. Modelled here at the producer seam:
//! one request must never manufacture two fixes.
void test_reinit_single_request_single_fix() {
    FakeAnchorConfig c = base_config();
    Running r(c);
    AnchorQuery q = query_at(4);
    q.reason      = AnchorRequestReason::REINIT;
    r.anchor.requestFix(q);
    check(r.fixes.size() == 1 && r.anchor.stats().requested == 1 &&
              r.anchor.stats().generated == 1,
          "one re-anchor request yields exactly one fix");
}

}  // namespace

int main() {
    spdlog::set_level(spdlog::level::info);

    if (!write_csv()) {
        spdlog::error("test_anchor: cannot write the synthetic groundtruth CSV");
        return 1;
    }

    test_setup_requires_groundtruth();
    test_no_origin_no_fix();
    test_cadence_and_position();
    test_seed_reproducibility();
    test_delivery_modes();
    test_reinit_bypasses_cadence();
    test_reinit_still_obeys_data_gates();
    test_reinit_min_gap();
    test_reinit_single_request_single_fix();

    std::remove(CSV_NAME);

    if (!g_ok) {
        spdlog::error("test_anchor: FAIL");
        return 1;
    }
    spdlog::info("test_anchor: PASS");
    return 0;
}
