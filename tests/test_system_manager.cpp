// test_system_manager — unit test for core::SystemManager and for
// sensor::GeoReferencer::latlon_from_enu().
//
// S3 checks (synchronous mode), 1..7 below, plus the S5 checks 8..15 of the
// ASYNCHRONOUS input path (queue + processing thread + input_policy + monitor
// thread + the onFrame()/stop() concurrency contract):
//   8.  async + BLOCK: pushing far more frames than the queue holds loses
//       nothing — processed == pushed, dropped == 0;
//   9.  async + DROP_OLDEST: a producer faster than the consumer does lose
//       frames, and frames_dropped equals the number of false onFrame()
//       returns exactly (received == processed + dropped);
//  10.  stop() CONCURRENT with a producer thread hammering onFrame():
//       terminates within its deadline, and every frame onFrame() accepted was
//       processed;
//  11.  stop() with frames still queued drains them all (no discard);
//  12.  stop() while a producer is BLOCKED waiting for queue space: the
//       producer is woken and leaves, nothing deadlocks;
//  13.  the monitor thread publishes at its configured cadence even while the
//       processing thread is stuck in a slow subscriber, and stops publishing
//       after stop();
//  14.  onFrame() after stop() returns false in async mode too, and nothing is
//       processed after teardown;
//  15.  async and sync produce the SAME LocalizationOutput sequence on the same
//       real frames (see the note at that check for what "same" covers).
//
// plus the S6c checks 16..22 of the FOUR-CHANNEL input path (onAttitude /
// onGimbal / onGnss / onImage + core::Extrapolator + attachSource):
//  16.  four channels pumped BY HAND (no source at all): one onImage() produces
//       exactly one processed frame, and the telemetry the pipeline saw is the
//       one that was pushed, field by field;
//  17.  a MISSING channel (attitude never published) ⇒ the frame reaches the
//       pipeline with has_telemetry == false and default (zero) telemetry —
//       nothing is invented — and SystemStats::frames_without_telemetry counts
//       it;
//  18.  WRONG ORDER (image published before the other three) ⇒ defined
//       behaviour: that frame gets no telemetry, and the very next image at the
//       same timestamp — now that the buffers are filled — gets it. This is why
//       S6a requires the image channel to fire LAST;
//  19.  attachSource(VideoDataSource) + start(): frames flow with nobody calling
//       read(), on_localization fires and at least one output is valid;
//  20.  stop() in the middle of a running source: returns within its deadline
//       and every frame the system accepted was processed;
//  21.  pauseSource() / resumeSource() (the viewer's Start/Stop button): while
//       paused NO frame is processed and the system stays RUNNING, and after
//       the resume the frame-id sequence is CONTIGUOUS across the pause — the
//       video is neither rewound (repeated ids) nor cut (a hole);
//  22.  ANGLE RANGE at the point of consumption: an INTERPOLATED heading lands
//       in (-180, 180] (the range core::Extrapolator documents for it), while
//       an EXACT hit is passed through VERBATIM — a stored yaw = 340 stays 340
//       and is NOT rewritten to -20. That second half is the bit-identical
//       guard: rewriting exact hits would move every fused number.
//
// Checks 19, 20 and 21 soft-skip when the gitignored dataset video is absent.
//
// EVERY wait in the S5 checks has a deadline: a hang is a failure, and it is
// reported as one instead of blocking forever (see run_with_deadline()).
//
// The S3 checks:
//   1. onFrame() BEFORE start()          ⇒ false, no crash;
//   2. start() without a successful setup() ⇒ false;
//   3. setup → start → stop moves state() correctly, and stop() is idempotent;
//   4. the six channels of callbacks() accept subscribers, report the right
//      size() and honour remove()/clear();
//   5. latlon_from_enu() is the exact inverse of enu() (< 1e-6 deg round trip)
//      AND differs from latlon() on the same input — i.e. it really does NOT
//      apply R_enu_w, which is the whole reason it exists;
//  5b. the vertical datum: alt0 is added EXACTLY ONCE, the synthetic case in
//      which an AGL double-count would be visible (see test_altitude_datum);
//   6. pushAbsoluteFix() before RUNNING  ⇒ false, no crash;
//   7. a REAL run over ~120 frames of a mission dataset: the output callback
//      fires, at least one LocalizationOutput is valid, its lat/lon sits within
//      0.5 deg of the telemetry of that frame, its altitude_m stays within the
//      configured AGL bound of the telemetry AGL, on_vo_data sees exactly
//      one payload per processed frame, and on_fusion_result publishes the raw
//      back-end result once per frame, BEFORE the LocalizationOutput built from
//      it (the ordering test_full_flight relies on since S4). Since S7 it also
//      checks accuracy_m on that real run: it is filled, it is NEVER 0 (0 would
//      claim perfect certainty) and it is never NaN once the fusion graph has
//      been anchored.
//
// Check 7 soft-skips (still PASS) when the gitignored dataset video is absent,
// the same contract as test_full_flight. The mission config is only READ; every
// other check builds its YAML in memory. Nothing under config/ is created or
// modified.
//
// Headless; exits non-zero on failure.

#include "uavloc/core/system_config.h"
#include "uavloc/core/system_manager.h"
#include "uavloc/core/system_types.h"
#include "uavloc/sensor/frame_data.h"
#include "uavloc/sensor/geo_reference.h"
#include "uavloc/sensor/stream_types.h"
#include "uavloc/sensor/video_data_source.h"
#include "uavloc/sensor/video_reader.h"

#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using uavloc::core::LocalizationOutput;
using uavloc::core::SystemConfig;
using uavloc::core::SystemManager;
using uavloc::core::SystemState;

//! Path of the mission config used by check 7. Provided by CMake so the test
//! source carries no hard-coded path; argv[1] overrides it.
#ifndef UAVLOC_MISSION_CONFIG_PATH
#define UAVLOC_MISSION_CONFIG_PATH ""
#endif

//! Frames fed in check 7. Large enough for VO to initialize and the fusion
//! graph to anchor on the MUN-FRL datasets (initialization lands around frame
//! 36), small enough to keep the unit test in the tens of seconds.
constexpr int REAL_RUN_FRAMES = 120;

//! A cut container can advertise more frames than are decodable; treat a run of
//! consecutive non-OK reads as end of stream (same guard as test_full_flight).
constexpr int MAX_CONSECUTIVE_BAD_READS = 100;

//! Round-trip tolerance of check 5 [deg]. The flat-earth forward and inverse
//! use the SAME constants, so only floating-point noise separates them.
constexpr double LATLON_ROUNDTRIP_TOL_DEG = 1e-6;

//! How far latlon() must sit from latlon_from_enu() for check 5 to have proven
//! anything [deg]. The two differ by the R_enu_w rotation, worth hundreds of
//! metres on the test point — orders of magnitude above the tolerance above.
constexpr double LATLON_ROTATION_MIN_DIFF_DEG = 1e-4;

//! Maximum accepted gap between a fused fix and the telemetry of its frame
//! [deg]. Deliberately loose: this checks the geo-referencing WIRING (right
//! anchor, no double rotation, plausible magnitude), not VO accuracy.
constexpr double REAL_RUN_MAX_LATLON_ERR_DEG = 0.5;

// ── S5 constants ──────────────────────────────────────────────────────────

//! Synthetic frame size for the async checks [px]. Small enough that the VO
//! front-end costs a few milliseconds, large enough for its scale pyramid.
constexpr int SYN_W = 320;
constexpr int SYN_H = 240;

//! Deadline for any single blocking operation of the async checks [ms]. Two
//! orders of magnitude above the delays those checks program, so exceeding it
//! means "stuck", not "slow machine".
constexpr int OP_DEADLINE_MS = 10000;

//! Deadline for waiting on a counter to reach a value [ms].
constexpr int POLL_DEADLINE_MS = 10000;

//! Push deadline handed to BLOCK-policy checks [ms]. Deliberately huge: those
//! checks must fail because a frame was LOST, never because a push timed out.
constexpr unsigned int LONG_PUSH_TIMEOUT_MS = 600000;

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

//! Minimal in-memory mission config: SystemConfig::fromYaml() throws without a
//! "Camera:" section, and the intrinsics must be non-degenerate or setup()
//! refuses them.
SystemConfig minimal_config() {
    return SystemConfig::fromYaml(YAML::Load(
        "Camera:\n"
        "  fx: 1000.0\n"
        "  fy: 1000.0\n"
        "  cx: 640.0\n"
        "  cy: 360.0\n"
        "  width: 1280\n"
        "  height: 720\n"));
}

// ── S5 helpers ────────────────────────────────────────────────────────────

//! Config for the async checks: a camera matching the synthetic frames, no
//! fusion (the queue behaviour under test is independent of the back-end, and
//! leaving it out keeps the consumer cost dominated by the programmed delay).
SystemConfig async_config(uavloc::core::InputPolicy policy, unsigned int capacity,
                          unsigned int push_timeout_ms, unsigned int stats_period_ms,
                          bool async_input = true) {
    SystemConfig cfg = SystemConfig::fromYaml(YAML::Load(
        "Camera:\n"
        "  fx: 250.0\n"
        "  fy: 250.0\n"
        "  cx: 160.0\n"
        "  cy: 120.0\n"
        "  width: " + std::to_string(SYN_W) + "\n"
        "  height: " + std::to_string(SYN_H) + "\n"));
    cfg.async_input          = async_input;
    cfg.input_policy         = policy;
    cfg.input_queue_capacity = capacity;
    cfg.push_timeout_ms      = push_timeout_ms;
    cfg.stats_period_ms      = stats_period_ms;
    cfg.enable_fusion        = false;
    cfg.enable_geo           = false;
    return cfg;
}

//! One synthetic frame. The content is a deterministic function of `id` so a
//! run is reproducible, and textured enough for the ORB front-end to do real
//! work rather than bail out on an empty image.
uavloc::sensor::FrameData make_frame(unsigned long long id) {
    uavloc::sensor::FrameData fd;
    fd.frame_id       = id;
    fd.timestamp_msec = static_cast<double>(id) * 100.0;
    fd.image          = cv::Mat(SYN_H, SYN_W, CV_8UC1, cv::Scalar(32));
    for (int k = 0; k < 24; ++k) {
        const int x = static_cast<int>((37 * k + 11 * id) % SYN_W);
        const int y = static_cast<int>((53 * k + 7 * id) % SYN_H);
        cv::rectangle(fd.image, cv::Rect(x, y, 12, 9), cv::Scalar(220), cv::FILLED);
    }
    fd.status = uavloc::sensor::FrameStatus::OK;
    fd.valid  = true;
    return fd;
}

//! Polls `pred` until it holds or the deadline expires. Returns its final value
//! — the caller turns that into a PASS/FAIL, so a missed condition is a
//! reported failure and never an endless wait.
template <typename Pred>
bool wait_until(Pred pred, std::chrono::milliseconds deadline) {
    const auto t_end = std::chrono::steady_clock::now() + deadline;
    while (std::chrono::steady_clock::now() < t_end) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
}

//! Runs `fn` on a helper thread and waits at most `deadline`.
//! A hang is a FAILURE and must not be an infinite one — but a stuck thread
//! cannot be killed, and unwinding the stack while it still references the
//! objects under test would turn the failure into undefined behaviour. So the
//! process reports and hard-exits instead.
void run_with_deadline(const std::string& name, const std::function<void()>& fn,
                       std::chrono::milliseconds deadline) {
    std::promise<void> done;
    std::future<void>  fut = done.get_future();
    std::thread        th([&] {
        fn();
        done.set_value();
    });
    if (fut.wait_for(deadline) != std::future_status::ready) {
        spdlog::error("FAIL: {} did not finish within {} ms — HANG", name,
                      deadline.count());
        spdlog::default_logger()->flush();
        std::_Exit(1);
    }
    th.join();
    check(true, name + " finished within its deadline");
}

//! Counts processed frames and, optionally, slows the consumer down. Subscribed
//! to callbacks().on_frame_processed, which SystemManager emits exactly once per
//! frame that went through the pipeline — so the counter is an exact
//! processed-frame count independent of what VO made of the image.
class ProcessedProbe {
public:
    ProcessedProbe(SystemManager& sys, std::chrono::milliseconds delay)
        : delay_(delay) {
        sys.callbacks().on_frame_processed.add(
            [this](double /*timestamp_msec*/, const uavloc::core::FrameProcessed& fp) {
                if (delay_.count() > 0) {
                    std::this_thread::sleep_for(delay_);
                }
                std::lock_guard<std::mutex> lock(mutex_);
                ids_.push_back(fp.frame_id);
                count_.store(ids_.size());
            });
    }

    std::size_t count() const { return count_.load(); }

    std::vector<unsigned int> ids() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return ids_;
    }

private:
    const std::chrono::milliseconds delay_;
    mutable std::mutex              mutex_;
    std::vector<unsigned int>       ids_;
    std::atomic<std::size_t>        count_{0};
};

// ── 1 + 6. Input refused before start() ───────────────────────────────────
void test_input_refused_before_start() {
    SystemManager sys(minimal_config());
    check(sys.state() == SystemState::CREATED, "fresh manager is CREATED");

    uavloc::sensor::FrameData fd;  // deliberately empty: must never be touched
    check(!sys.onFrame(fd), "onFrame() before setup()/start() returns false");
    check(!sys.pushAbsoluteFix(uavloc::anchor::AbsoluteFix{}),
          "pushAbsoluteFix() before start() returns false");

    check(sys.setup(), "setup() succeeds on a valid minimal config");
    check(sys.state() == SystemState::CREATED, "setup() does not change the state");
    check(!sys.onFrame(fd), "onFrame() after setup() but before start() returns false");
    check(!sys.pushAbsoluteFix(uavloc::anchor::AbsoluteFix{}),
          "pushAbsoluteFix() after setup() but before start() returns false");

    check(!sys.latest().valid, "latest() is invalid before anything is processed");
    check(sys.stats().frames_processed == 0, "no frame was processed");
    // Both onFrame() calls were counted as received and as dropped;
    // pushAbsoluteFix() is not a frame and must not touch these counters.
    check(sys.stats().frames_received == 2 && sys.stats().frames_dropped == 2,
          "every refused onFrame() was counted as received + dropped");
}

// ── 2. start() without setup() ────────────────────────────────────────────
void test_start_without_setup() {
    SystemManager sys(minimal_config());
    check(!sys.start(), "start() without setup() returns false");
    check(sys.state() == SystemState::CREATED, "a refused start() leaves the state alone");

    // Degenerate intrinsics must be refused by setup(), so start() stays refused.
    SystemManager bad(SystemConfig::fromYaml(YAML::Load(
        "Camera:\n  fx: 0.0\n  fy: 0.0\n  cx: 0.0\n  cy: 0.0\n")));
    check(!bad.setup(), "setup() refuses degenerate camera intrinsics");
    check(!bad.start(), "start() refuses a manager whose setup() failed");
}

// ── 3. Lifecycle + idempotent stop() ──────────────────────────────────────
void test_lifecycle() {
    SystemManager sys(minimal_config());
    check(sys.setup(), "setup()");
    check(!sys.setup(), "a second setup() is refused");
    check(sys.start(), "start()");
    check(sys.state() == SystemState::RUNNING, "state() is RUNNING after start()");
    check(!sys.start(), "a second start() is refused");

    sys.stop();
    check(sys.state() == SystemState::STOPPED, "state() is STOPPED after stop()");
    sys.stop();
    check(sys.state() == SystemState::STOPPED, "a second stop() is a safe no-op");

    uavloc::sensor::FrameData fd;
    check(!sys.onFrame(fd), "onFrame() after stop() returns false");
    check(!sys.start(), "start() after stop() is refused");
}

// ── 4. Debug channels + output callback ───────────────────────────────────
void test_channels() {
    SystemManager sys(minimal_config());

    auto& cbs = sys.callbacks();

    const auto id_vo = cbs.on_vo_data.add([](double, const uavloc::vo::VOData&) {});
    const auto id_frame =
        cbs.on_frame_processed.add([](double, const uavloc::core::FrameProcessed&) {});
    const auto id_fres =
        cbs.on_fusion_result.add([](double, const uavloc::fusion::FusionResult&) {});
    const auto id_lag =
        cbs.on_lag_window.add([](const std::vector<uavloc::fusion::FusionLagPose>&) {});
    const auto id_stats = cbs.on_stats.add([](const uavloc::core::SystemStats&) {});
    const auto id_out   = cbs.on_localization.add([](const LocalizationOutput&) {});

    check(cbs.on_vo_data.size() == 1, "on_vo_data has 1 subscriber");
    check(cbs.on_frame_processed.size() == 1, "on_frame_processed has 1 subscriber");
    check(cbs.on_fusion_result.size() == 1, "on_fusion_result has 1 subscriber");
    check(cbs.on_lag_window.size() == 1, "on_lag_window has 1 subscriber");
    check(cbs.on_stats.size() == 1, "on_stats has 1 subscriber");
    check(cbs.on_localization.size() == 1, "on_localization has 1 subscriber");

    // callbacks() must hand out the SAME struct every time — the slots are
    // members of the manager, not a per-call copy and not a static bus.
    check(&sys.callbacks() == &cbs, "callbacks() returns the same member struct");

    // A second subscriber on the same channel, then removal of the first: the
    // counts must move independently per channel.
    cbs.on_frame_processed.add([](double, const uavloc::core::FrameProcessed&) {});
    check(cbs.on_frame_processed.size() == 2, "on_frame_processed accepts a 2nd subscriber");
    check(cbs.on_vo_data.size() == 1, "the other channels are unaffected by it");

    check(cbs.on_vo_data.remove(id_vo), "on_vo_data remove");
    check(cbs.on_frame_processed.remove(id_frame), "on_frame_processed remove");
    check(cbs.on_fusion_result.remove(id_fres), "on_fusion_result remove");
    check(cbs.on_lag_window.remove(id_lag), "on_lag_window remove");
    check(cbs.on_stats.remove(id_stats), "on_stats remove");
    check(cbs.on_localization.remove(id_out), "on_localization remove");
    check(cbs.on_vo_data.size() == 0 && cbs.on_frame_processed.size() == 1 &&
              cbs.on_fusion_result.size() == 0 && cbs.on_lag_window.size() == 0 &&
              cbs.on_stats.size() == 0 && cbs.on_localization.size() == 0,
          "subscriber counts after remove");

    // Removing every output subscriber must be possible and must not crash.
    cbs.on_frame_processed.clear();
    check(cbs.on_frame_processed.size() == 0, "clear() empties a channel");
}

// ── 5. latlon_from_enu is the pure inverse of enu ─────────────────────────
void test_latlon_from_enu() {
    uavloc::sensor::GeoReferencer geo;
    // A non-zero view azimuth is essential: with psi = 0 the R_enu_w rotation
    // would still be non-identity, but a 30 deg anchor makes the difference
    // between latlon() and latlon_from_enu() unmistakable.
    constexpr double LAT0 = 21.0, LON0 = 105.0, ALT0 = 100.0, PSI_DEG = 30.0;
    geo.init(LAT0, LON0, ALT0, PSI_DEG);

    // A handful of points within a few kilometres of the anchor.
    const std::vector<uavloc::sensor::LatLonAlt> points = {
        {LAT0 + 0.01, LON0 + 0.02, ALT0 + 50.0},
        {LAT0 - 0.03, LON0 + 0.01, ALT0 - 20.0},
        {LAT0 + 0.00, LON0 - 0.04, ALT0 + 0.0},
        {LAT0, LON0, ALT0},
    };

    bool round_trip_ok = true;
    bool differs_ok    = true;
    for (const auto& p : points) {
        const Eigen::Vector3d p_enu = geo.enu(p.lat, p.lon, p.alt);
        const auto            rt    = geo.latlon_from_enu(p_enu);
        if (std::abs(rt.lat - p.lat) > LATLON_ROUNDTRIP_TOL_DEG ||
            std::abs(rt.lon - p.lon) > LATLON_ROUNDTRIP_TOL_DEG ||
            std::abs(rt.alt - p.alt) > 1e-6) {
            round_trip_ok = false;
            spdlog::error("  round trip failed: ({:.7f}, {:.7f}, {:.3f}) -> "
                          "({:.7f}, {:.7f}, {:.3f})",
                          p.lat, p.lon, p.alt, rt.lat, rt.lon, rt.alt);
        }
        // latlon() rotates its argument by R_enu_w first, so on the SAME input
        // it must land somewhere else — except at the anchor, where every
        // rotation maps 0 to 0.
        const auto rotated = geo.latlon(p_enu);
        const bool at_anchor = p_enu.norm() < 1.0;
        const double diff = std::abs(rotated.lat - rt.lat) + std::abs(rotated.lon - rt.lon);
        if (!at_anchor && diff < LATLON_ROTATION_MIN_DIFF_DEG) {
            differs_ok = false;
            spdlog::error("  latlon() and latlon_from_enu() agree at ({:.7f}, {:.7f}) "
                          "— the rotation was not applied by one of them",
                          p.lat, p.lon);
        }
    }
    check(round_trip_ok, "latlon_from_enu(enu(x)) == x within 1e-6 deg");
    check(differs_ok,
          "latlon_from_enu() differs from latlon() (it does NOT apply R_enu_w)");
}

// ── 5b. The vertical datum: alt0 is added exactly once ────────────────────
// THIS is the case where the AGL double-count would show. SystemManager
// anchors the GeoReferencer with alt0 = 0 precisely because the fused Z is
// already an AGL; if alt0 were the telemetry altitude instead, the output
// altitude would come out as alt0 + agl and the second assertion below would
// read 246.8 instead of 223.4. A dataset check cannot catch this whenever the
// anchor AGL happens to be near zero (dataset3 anchors at 0.02 m), so it is
// pinned here, synthetically, with no data at all.
void test_altitude_datum() {
    constexpr double Z_AGL   = 123.4;  //!< a fused ENU Up coordinate [m]
    constexpr double ALT_TOL = 1e-9;

    uavloc::sensor::GeoReferencer zero_anchor;
    zero_anchor.init(21.0, 105.0, 0.0, 0.0);  // what SystemManager does
    const double alt_zero =
        zero_anchor.latlon_from_enu(Eigen::Vector3d(10.0, -20.0, Z_AGL)).alt;
    check(std::abs(alt_zero - Z_AGL) < ALT_TOL,
          "alt0 = 0 -> altitude_m IS the AGL (123.4), no double count");

    uavloc::sensor::GeoReferencer offset_anchor;
    offset_anchor.init(21.0, 105.0, 100.0, 0.0);
    const double alt_offset =
        offset_anchor.latlon_from_enu(Eigen::Vector3d(10.0, -20.0, Z_AGL)).alt;
    check(std::abs(alt_offset - (100.0 + Z_AGL)) < ALT_TOL,
          "alt0 = 100 -> 223.4, i.e. alt0 is added exactly once");

    // The gap between the two anchors is the anchor altitude itself — that is
    // the quantity a double-counting SystemManager would add on top of an AGL.
    check(std::abs((alt_offset - alt_zero) - 100.0) < ALT_TOL,
          "the two anchors differ by exactly alt0 (the double-count amount)");
}

// ── 7. Real run over a mission dataset ────────────────────────────────────
void test_real_run(const std::string& config_path) {
    if (config_path.empty()) {
        spdlog::error("FAIL: real-run check has no mission config path");
        g_ok = false;
        return;
    }
    YAML::Node root;
    SystemConfig sys_cfg;
    uavloc::sensor::VideoReaderConfig reader_cfg;
    try {
        root       = YAML::LoadFile(config_path);  // READ-ONLY
        sys_cfg    = SystemConfig::fromYaml(root);
        reader_cfg = uavloc::sensor::VideoReaderConfig::fromYaml(root);
    } catch (const std::exception& ex) {
        spdlog::error("FAIL: cannot parse mission config '{}': {}", config_path, ex.what());
        g_ok = false;
        return;
    }
    // Forced in code, not in the YAML: with a synchronous back-end the fusion
    // result callback fires inside onFrame(), so the assertions below can read
    // the telemetry of the very frame that produced the output.
    sys_cfg.fusion.async_enabled = false;

    uavloc::sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::warn("test_system_manager: cannot open video '{}' — real-run check "
                     "SKIPPED (dataset absent)", reader_cfg.video_path);
        return;  // soft-skip: still PASS
    }

    SystemManager sys(sys_cfg);

    int  vo_data_count   = 0;
    int  output_count    = 0;
    int  valid_count     = 0;
    int  latlon_checked  = 0;
    bool latlon_ok       = true;
    // Telemetry of the frame currently in flight; the output callback runs
    // inline inside onFrame(), so this is always the matching record.
    double cur_lat = 0.0, cur_lon = 0.0, cur_agl = 0.0;
    bool   cur_telem_ok = false;
    std::vector<double> agl_err_m;  //!< |output altitude − telemetry AGL| [m]

    // S7 accuracy_m accounting. `accuracy_zero` is the failure the S7 spec
    // singles out: 0 means "perfectly certain", the most dangerous value to
    // publish. `accuracy_nan_after_graph` counts outputs that carried a fused
    // pose but no uncertainty — allowed only before the graph is anchored,
    // which is exactly what `graph_anchored` tracks (it is set from the raw
    // FusionResult, which is published BEFORE its LocalizationOutput).
    std::vector<double> accuracy_m;
    int  accuracy_zero            = 0;
    int  accuracy_nan_after_graph = 0;
    bool graph_anchored           = false;

    // on_fusion_result must fire once per frame and BEFORE the output built
    // from it, so a driver can pair the raw result with its LocalizationOutput.
    int  fres_count      = 0;
    bool fres_before_out = true;
    bool fres_pending    = false;

    sys.callbacks().on_vo_data.add(
        [&](double, const uavloc::vo::VOData&) { ++vo_data_count; });
    sys.callbacks().on_fusion_result.add(
        [&](double, const uavloc::fusion::FusionResult& fr) {
            ++fres_count;
            fres_pending = true;
            if (fr.covariance_valid) {
                graph_anchored = true;
            }
        });
    sys.callbacks().on_localization.add([&](const LocalizationOutput& out) {
        ++output_count;
        if (!fres_pending) fres_before_out = false;
        fres_pending = false;
        if (std::isnan(out.accuracy_m)) {
            if (graph_anchored) ++accuracy_nan_after_graph;
        } else {
            accuracy_m.push_back(out.accuracy_m);
            if (out.accuracy_m == 0.0f) ++accuracy_zero;
        }
        if (!out.valid) {
            return;
        }
        ++valid_count;
        if (!cur_telem_ok) {
            return;
        }
        ++latlon_checked;
        // altitude_m is an AGL, directly comparable to telemetry AGL.
        agl_err_m.push_back(std::abs(out.altitude_m - cur_agl));
        if (std::abs(out.latitude - cur_lat) > REAL_RUN_MAX_LATLON_ERR_DEG ||
            std::abs(out.longitude - cur_lon) > REAL_RUN_MAX_LATLON_ERR_DEG) {
            latlon_ok = false;
            spdlog::error("  frame {}: fused ({:.6f}, {:.6f}) vs telemetry "
                          "({:.6f}, {:.6f})",
                          out.frame_id, out.latitude, out.longitude, cur_lat, cur_lon);
        }
    });

    if (!check(sys.setup(), "real run: setup()")) return;
    if (!check(sys.start(), "real run: start()")) return;

    int frames_fed           = 0;
    int consecutive_bad_reads = 0;
    uavloc::sensor::FrameData fd;
    while (frames_fed < REAL_RUN_FRAMES) {
        const uavloc::sensor::FrameStatus status = reader.read(fd);
        if (status == uavloc::sensor::FrameStatus::END_OF_STREAM ||
            status == uavloc::sensor::FrameStatus::ERROR ||
            status == uavloc::sensor::FrameStatus::CAMERA_DISCONNECTED) {
            break;
        }
        if (status != uavloc::sensor::FrameStatus::OK || !fd.valid || !fd.HasImage()) {
            if (++consecutive_bad_reads >= MAX_CONSECUTIVE_BAD_READS) break;
            continue;
        }
        consecutive_bad_reads = 0;

        cur_telem_ok = fd.has_telemetry && fd.telemetry.altitude_m > 0.0;
        cur_lat      = fd.telemetry.latitude_deg;
        cur_lon      = fd.telemetry.longitude_deg;
        cur_agl      = fd.telemetry.altitude_m;

        if (!sys.onFrame(fd)) {
            spdlog::error("FAIL: onFrame() refused frame {} while RUNNING", fd.frame_id);
            g_ok = false;
            break;
        }
        ++frames_fed;
    }
    sys.stop();
    reader.close();

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("real run: fed={} processed={} vo_data={} fusion_res={} outputs={} "
                 "valid={} checked_latlon={} lost={} reinit={} fps={:.2f}",
                 frames_fed, st.frames_processed, vo_data_count, fres_count,
                 output_count, valid_count, latlon_checked, st.lost_events,
                 st.reinit_events, st.fps_processed);

    if (frames_fed == 0) {
        spdlog::warn("test_system_manager: no decodable frame — real-run check SKIPPED");
        return;
    }
    check(static_cast<unsigned long long>(frames_fed) == st.frames_processed,
          "real run: every accepted frame was processed");
    check(vo_data_count == frames_fed,
          "real run: on_vo_data published once per processed frame");
    check(fres_count == frames_fed,
          "real run: on_fusion_result published once per processed frame");
    check(fres_before_out,
          "real run: the raw FusionResult is published BEFORE its LocalizationOutput");
    check(output_count >= 1, "real run: the output callback fired");
    check(valid_count >= 1, "real run: at least one LocalizationOutput is valid");
    check(latlon_checked >= 1, "real run: at least one fix was compared to telemetry");
    check(latlon_ok, "real run: fused lat/lon within 0.5 deg of the frame's telemetry");

    // Vertical: AglFactor pins the fused Z to telemetry AGL, so the gap is a
    // few sigma at most. The bound comes from the CONFIG (agl_sigma_m plus the
    // AGL-bias prior), never from a hard-coded number, and it is applied to the
    // MEDIAN because a re-init transient can throw individual frames far off.
    // NOTE: this cannot distinguish alt0 = 0 from alt0 = telemetry on this
    // dataset — the anchor AGL is ~0.02 m. test_altitude_datum() is the check
    // that does.
    const double agl_bound_m =
        sys_cfg.fusion.agl_sigma_m + sys_cfg.fusion.agl_bias_prior_sigma_m;
    if (!agl_err_m.empty()) {
        std::sort(agl_err_m.begin(), agl_err_m.end());
        const double median = agl_err_m[agl_err_m.size() / 2];
        const std::size_t p95_rank = static_cast<std::size_t>(
            std::ceil(0.95 * static_cast<double>(agl_err_m.size())));
        const double p95 = agl_err_m[std::min(p95_rank, agl_err_m.size()) - 1];
        spdlog::info("real run: |altitude_m - telemetry AGL| median={:.2f} m "
                     "p95={:.2f} m max={:.2f} m (bound {:.2f} m = agl_sigma_m "
                     "+ agl_bias_prior_sigma_m)",
                     median, p95, agl_err_m.back(), agl_bound_m);
        check(median <= agl_bound_m,
              "real run: median |altitude_m - telemetry AGL| within the config bound");
    } else {
        spdlog::error("FAIL: real run collected no altitude sample");
        g_ok = false;
    }

    // ── S7: accuracy_m on a real run ─────────────────────────────────────────
    if (!accuracy_m.empty()) {
        std::vector<double> sorted = accuracy_m;
        std::sort(sorted.begin(), sorted.end());
        const std::size_t p95_rank = static_cast<std::size_t>(
            std::ceil(0.95 * static_cast<double>(sorted.size())));
        spdlog::info("real run: accuracy_m over {} outputs — median={:.2f} m "
                     "p95={:.2f} m max={:.2f} m ({} outputs had no covariance)",
                     sorted.size(), sorted[sorted.size() / 2],
                     sorted[std::min(p95_rank, sorted.size()) - 1], sorted.back(),
                     output_count - static_cast<int>(sorted.size()));
    }
    check(!accuracy_m.empty(), "real run: at least one output carries accuracy_m");
    check(accuracy_zero == 0,
          "real run: accuracy_m is never 0 (0 would claim perfect certainty)");
    check(accuracy_nan_after_graph == 0,
          "real run: accuracy_m is not NaN once the fusion graph is anchored");

    check(sys.latest().valid, "real run: latest() carries the last valid fix");
    check(st.frames_dropped == 0, "real run: no frame was refused");
}

// ── 8. async + BLOCK loses nothing ────────────────────────────────────────
void test_async_block_no_loss() {
    constexpr unsigned long long N_FRAMES = 24;
    constexpr unsigned int       CAPACITY = 2;  // far smaller than N_FRAMES

    SystemManager sys(async_config(uavloc::core::InputPolicy::BLOCK, CAPACITY,
                                   LONG_PUSH_TIMEOUT_MS, 0));
    ProcessedProbe probe(sys, std::chrono::milliseconds(2));
    if (!check(sys.setup() && sys.start(), "async BLOCK: setup + start")) return;

    std::size_t accepted = 0;
    run_with_deadline(
        "async BLOCK: pushing 24 frames through a 2-deep queue",
        [&] {
            for (unsigned long long i = 0; i < N_FRAMES; ++i) {
                if (sys.onFrame(make_frame(i))) {
                    ++accepted;
                }
            }
        },
        std::chrono::milliseconds(OP_DEADLINE_MS));

    run_with_deadline("async BLOCK: stop()", [&] { sys.stop(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("async BLOCK: accepted={} processed={} dropped={} queue_depth={}",
                 accepted, st.frames_processed, st.frames_dropped, st.queue_depth);
    check(accepted == N_FRAMES, "async BLOCK: every onFrame() was accepted");
    check(st.frames_processed == N_FRAMES,
          "async BLOCK: every pushed frame was processed (no loss)");
    check(st.frames_dropped == 0, "async BLOCK: nothing was dropped");
    check(probe.count() == N_FRAMES,
          "async BLOCK: the pipeline ran once per pushed frame");

    // Order is a property of the queue, not an accident: a FIFO must preserve it.
    const std::vector<unsigned int> ids = probe.ids();
    bool in_order = ids.size() == N_FRAMES;
    for (std::size_t i = 0; in_order && i < ids.size(); ++i) {
        in_order = ids[i] == static_cast<unsigned int>(i);
    }
    check(in_order, "async BLOCK: frames were processed in push order");
}

// ── 9. async + DROP_OLDEST drops, and says so ─────────────────────────────
void test_async_drop_oldest() {
    constexpr unsigned long long N_FRAMES = 60;
    constexpr unsigned int       CAPACITY = 2;

    SystemManager sys(async_config(uavloc::core::InputPolicy::DROP_OLDEST, CAPACITY,
                                   LONG_PUSH_TIMEOUT_MS, 0));
    // 5 ms per frame against a producer that never waits: the queue must
    // overflow, which is the whole point of this check.
    ProcessedProbe probe(sys, std::chrono::milliseconds(5));
    if (!check(sys.setup() && sys.start(), "DROP_OLDEST: setup + start")) return;

    std::size_t refused = 0;
    run_with_deadline(
        "DROP_OLDEST: pushing 60 frames as fast as possible",
        [&] {
            for (unsigned long long i = 0; i < N_FRAMES; ++i) {
                if (!sys.onFrame(make_frame(i))) {
                    ++refused;
                }
            }
        },
        std::chrono::milliseconds(OP_DEADLINE_MS));

    run_with_deadline("DROP_OLDEST: stop()", [&] { sys.stop(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("DROP_OLDEST: received={} processed={} dropped={} refused_returns={}",
                 st.frames_received, st.frames_processed, st.frames_dropped, refused);
    check(refused > 0, "DROP_OLDEST: the producer was told frames are being lost");
    check(st.frames_dropped == refused,
          "DROP_OLDEST: frames_dropped == number of false onFrame() returns");
    check(st.frames_received == N_FRAMES, "DROP_OLDEST: every push was counted");
    // A false return means one OLDER frame was discarded while the new one was
    // still queued, so the three counters must close exactly.
    check(st.frames_processed + st.frames_dropped == st.frames_received,
          "DROP_OLDEST: processed + dropped == received");
    check(probe.count() == st.frames_processed,
          "DROP_OLDEST: the pipeline ran once per processed frame");
}

// ── 10. stop() concurrent with a producer ─────────────────────────────────
void test_stop_races_producer() {
    SystemManager sys(async_config(uavloc::core::InputPolicy::BLOCK, 4,
                                   LONG_PUSH_TIMEOUT_MS, 0));
    ProcessedProbe probe(sys, std::chrono::milliseconds(1));
    if (!check(sys.setup() && sys.start(), "concurrent stop: setup + start")) return;

    std::atomic<bool>        producer_done{false};
    std::atomic<std::size_t> accepted{0};
    std::atomic<std::size_t> refused{0};
    std::atomic<bool>        accepted_after_stop{false};
    std::atomic<bool>        stop_returned{false};

    std::thread producer([&] {
        for (unsigned long long i = 0; i < 5000 && !producer_done.load(); ++i) {
            const bool ok = sys.onFrame(make_frame(i));
            if (ok) {
                ++accepted;
                // The moment stop() has returned, no push may still be taken:
                // that would be a frame handed to a torn-down pipeline.
                if (stop_returned.load()) {
                    accepted_after_stop.store(true);
                }
            } else {
                ++refused;
            }
        }
    });

    // Let the producer get going, then tear down underneath it.
    check(wait_until([&] { return accepted.load() >= 4; },
                     std::chrono::milliseconds(POLL_DEADLINE_MS)),
          "concurrent stop: the producer is running before stop() is called");

    run_with_deadline("concurrent stop: stop() while onFrame() is being hammered",
                      [&] {
                          sys.stop();
                          stop_returned.store(true);
                      },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    // Keep pushing for a moment AFTER stop() so the "refused from now on"
    // guarantee is actually exercised, then wind the producer down.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    producer_done.store(true);
    producer.join();

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("concurrent stop: accepted={} refused={} processed={} dropped={} state={}",
                 accepted.load(), refused.load(), st.frames_processed,
                 st.frames_dropped, static_cast<int>(st.state));
    check(sys.state() == SystemState::STOPPED, "concurrent stop: state is STOPPED");
    check(refused.load() > 0, "concurrent stop: pushes after stop() were refused");
    check(!accepted_after_stop.load(),
          "concurrent stop: no frame was accepted after stop() returned");
    // The core guarantee: acceptance means processing, even across a teardown.
    check(st.frames_processed == accepted.load(),
          "concurrent stop: every accepted frame was processed");
    check(probe.count() == accepted.load(),
          "concurrent stop: the pipeline ran once per accepted frame");
}

// ── 11. stop() drains the queue ───────────────────────────────────────────
void test_stop_drains_queue() {
    constexpr unsigned long long N_FRAMES = 8;

    // Capacity 16 > N_FRAMES and a slow consumer: by the time stop() is called
    // almost every frame is still sitting in the queue, unprocessed.
    SystemManager sys(async_config(uavloc::core::InputPolicy::BLOCK, 16,
                                   LONG_PUSH_TIMEOUT_MS, 0));
    ProcessedProbe probe(sys, std::chrono::milliseconds(20));
    if (!check(sys.setup() && sys.start(), "drain: setup + start")) return;

    std::size_t accepted = 0;
    for (unsigned long long i = 0; i < N_FRAMES; ++i) {
        if (sys.onFrame(make_frame(i))) {
            ++accepted;
        }
    }
    const std::size_t processed_before_stop = probe.count();
    check(accepted == N_FRAMES, "drain: all 8 frames were accepted");
    check(processed_before_stop < N_FRAMES,
          "drain: frames were still queued when stop() was called");

    run_with_deadline("drain: stop() with a non-empty queue", [&] { sys.stop(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("drain: accepted={} processed_before_stop={} processed={} dropped={} "
                 "queue_depth={}",
                 accepted, processed_before_stop, st.frames_processed,
                 st.frames_dropped, st.queue_depth);
    check(st.frames_processed == N_FRAMES, "drain: stop() processed every queued frame");
    check(st.frames_dropped == 0, "drain: stop() discarded nothing");
    check(st.queue_depth == 0, "drain: the queue is empty afterwards");
}

// ── 12. stop() while a producer waits for space ───────────────────────────
void test_stop_wakes_blocked_producer() {
    // Capacity 1 + a 300 ms consumer + a 10 minute push deadline: the producer
    // WILL be parked inside onFrame(). If stop() did not shut the queue down,
    // this check would sit there for ten minutes — which is exactly why the
    // deadline below turns it into a reported failure instead.
    SystemManager sys(async_config(uavloc::core::InputPolicy::BLOCK, 1,
                                   LONG_PUSH_TIMEOUT_MS, 0));
    ProcessedProbe probe(sys, std::chrono::milliseconds(300));
    if (!check(sys.setup() && sys.start(), "blocked producer: setup + start")) return;

    std::atomic<std::size_t> pushes{0};
    std::atomic<std::size_t> accepted{0};
    std::atomic<bool>        producer_exited{false};

    std::thread producer([&] {
        for (unsigned long long i = 0; i < 8; ++i) {
            ++pushes;
            if (sys.onFrame(make_frame(i))) {
                ++accepted;
            }
        }
        producer_exited.store(true);
    });

    // Wait until the producer is demonstrably stuck: it has issued more pushes
    // than the queue and the consumer can have absorbed.
    check(wait_until([&] { return pushes.load() >= 3 && accepted.load() < pushes.load(); },
                     std::chrono::milliseconds(POLL_DEADLINE_MS)),
          "blocked producer: the producer is parked on a full queue");

    run_with_deadline("blocked producer: stop() with a producer waiting for space",
                      [&] { sys.stop(); }, std::chrono::milliseconds(OP_DEADLINE_MS));

    run_with_deadline("blocked producer: the parked producer left onFrame()",
                      [&] { producer.join(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));
    check(producer_exited.load(), "blocked producer: the producer thread ran to the end");

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("blocked producer: pushes={} accepted={} processed={} dropped={}",
                 pushes.load(), accepted.load(), st.frames_processed, st.frames_dropped);
    check(st.frames_processed == accepted.load(),
          "blocked producer: every accepted frame was still processed");
}

// ── 13. The monitor thread ────────────────────────────────────────────────
void test_monitor_thread() {
    constexpr unsigned int STATS_PERIOD_MS = 50;
    //! Long enough for several periods, and far longer than one period so the
    //! "at least two samples" bound cannot be met by luck [ms].
    constexpr int OBSERVE_MS = 400;
    //! How long the single frame keeps the processing thread busy [ms] —
    //! deliberately longer than OBSERVE_MS, so any stats sample seen during the
    //! observation window PROVES the cadence does not depend on the load.
    constexpr int CONSUMER_BLOCK_MS = 3000;

    SystemManager sys(async_config(uavloc::core::InputPolicy::DROP_OLDEST, 2,
                                   LONG_PUSH_TIMEOUT_MS, STATS_PERIOD_MS));
    ProcessedProbe probe(sys, std::chrono::milliseconds(CONSUMER_BLOCK_MS));

    std::atomic<std::size_t> samples{0};
    std::atomic<bool>        state_ok{true};
    std::atomic<int>         last_received{-1};
    sys.callbacks().on_stats.add([&](const uavloc::core::SystemStats& st) {
        if (st.state != SystemState::RUNNING) {
            state_ok.store(false);
        }
        last_received.store(static_cast<int>(st.frames_received));
        ++samples;
    });

    if (!check(sys.setup() && sys.start(), "monitor: setup + start")) return;
    check(sys.onFrame(make_frame(0)), "monitor: one frame accepted");

    check(wait_until([&] { return samples.load() >= 2; },
                     std::chrono::milliseconds(OBSERVE_MS)),
          "monitor: at least 2 SystemStats samples within 400 ms (period 50 ms)");
    check(probe.count() == 0,
          "monitor: the processing thread was still busy — the cadence is "
          "independent of the load");
    check(state_ok.load(), "monitor: every sample reported state == RUNNING");
    check(last_received.load() >= 1, "monitor: the samples carry the live counters");

    const std::size_t before_stop = samples.load();
    run_with_deadline("monitor: stop()", [&] { sys.stop(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));
    const std::size_t at_stop = samples.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(4 * STATS_PERIOD_MS));
    spdlog::info("monitor: samples before_stop={} at_stop={} after_stop={}",
                 before_stop, at_stop, samples.load());
    check(samples.load() == at_stop, "monitor: no sample is published after stop()");
}

// ── 14. onFrame() after stop(), async mode ────────────────────────────────
void test_async_input_after_stop() {
    SystemManager sys(async_config(uavloc::core::InputPolicy::BLOCK, 4,
                                   LONG_PUSH_TIMEOUT_MS, 0));
    ProcessedProbe probe(sys, std::chrono::milliseconds(0));
    if (!check(sys.setup() && sys.start(), "after stop: setup + start")) return;

    check(sys.onFrame(make_frame(0)), "after stop: a frame is accepted while RUNNING");
    run_with_deadline("after stop: stop()", [&] { sys.stop(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const unsigned long long processed_at_stop = sys.stats().frames_processed;
    check(!sys.onFrame(make_frame(1)), "after stop: onFrame() returns false");
    check(!sys.onFrame(make_frame(2)), "after stop: it keeps returning false");
    check(!sys.pushAbsoluteFix(uavloc::anchor::AbsoluteFix{}),
          "after stop: pushAbsoluteFix() returns false");

    // Give a hypothetical surviving worker time to pick anything up.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const uavloc::core::SystemStats st = sys.stats();
    check(st.frames_processed == processed_at_stop,
          "after stop: nothing was processed after teardown");
    check(probe.count() == processed_at_stop,
          "after stop: the pipeline did not run again");
    check(st.frames_dropped == 2, "after stop: both refused frames were counted");
}

// ── 15. async and sync agree on the same real frames ──────────────────────
// Runs the SAME dataset frames through a synchronous manager and through an
// asynchronous one (BLOCK, so nothing is dropped), and compares the resulting
// LocalizationOutput sequences field by field.
//
// WHAT THIS DOES AND DOES NOT PROVE. The comparison is exact, and it covers
// the full real pipeline (VO front-end, fusion back-end, geo-referencing).
// It is NOT a proof that async is deterministic in general:
//   * the fusion back-end is forced synchronous in both runs, so the fusion
//     thread's scheduling is out of the picture by construction;
//   * VO keeps its own mapping thread in both runs — the equality observed
//     here says that thread's timing did not change the result on this data,
//     not that it never can;
//   * the two runs use two SystemManager instances in one process, so the
//     check also (deliberately) covers instance-to-instance reproducibility.
// The honest claim is therefore: on this dataset, moving the pipeline onto a
// processing thread behind a BLOCK queue changed no output value and no output
// ORDER — which is what S5 had to demonstrate.
void test_async_matches_sync(const std::string& config_path) {
    //! Frames per run. Past VO initialization on the MUN-FRL datasets (~36) so
    //! the comparison includes valid, geo-referenced fixes.
    constexpr int EQUIV_FRAMES = 100;

    if (config_path.empty()) {
        spdlog::error("FAIL: async/sync equivalence check has no mission config path");
        g_ok = false;
        return;
    }
    YAML::Node root;
    SystemConfig base_cfg;
    uavloc::sensor::VideoReaderConfig reader_cfg;
    try {
        root       = YAML::LoadFile(config_path);  // READ-ONLY
        base_cfg   = SystemConfig::fromYaml(root);
        reader_cfg = uavloc::sensor::VideoReaderConfig::fromYaml(root);
    } catch (const std::exception& ex) {
        spdlog::error("FAIL: cannot parse mission config '{}': {}", config_path, ex.what());
        g_ok = false;
        return;
    }
    base_cfg.fusion.async_enabled = false;

    // Collects the output sequence of one run. The manager is stopped before
    // the vector is read, and stop() joins every thread that could write it, so
    // no synchronisation is needed at the read.
    auto run = [&](bool async, std::vector<LocalizationOutput>& out) -> int {
        SystemConfig cfg = base_cfg;
        cfg.async_input          = async;
        cfg.input_policy         = uavloc::core::InputPolicy::BLOCK;
        cfg.input_queue_capacity = 8;
        cfg.push_timeout_ms      = LONG_PUSH_TIMEOUT_MS;  // must never lose a frame
        cfg.stats_period_ms      = 50;                    // monitor thread running too

        uavloc::sensor::VideoReader reader(reader_cfg);
        if (!reader.open()) {
            return -1;
        }
        SystemManager sys(cfg);
        sys.callbacks().on_localization.add(
            [&out](const LocalizationOutput& o) { out.push_back(o); });
        if (!sys.setup() || !sys.start()) {
            return -1;
        }

        int frames_fed            = 0;
        int consecutive_bad_reads = 0;
        uavloc::sensor::FrameData fd;
        while (frames_fed < EQUIV_FRAMES) {
            const uavloc::sensor::FrameStatus status = reader.read(fd);
            if (status == uavloc::sensor::FrameStatus::END_OF_STREAM ||
                status == uavloc::sensor::FrameStatus::ERROR ||
                status == uavloc::sensor::FrameStatus::CAMERA_DISCONNECTED) {
                break;
            }
            if (status != uavloc::sensor::FrameStatus::OK || !fd.valid || !fd.HasImage()) {
                if (++consecutive_bad_reads >= MAX_CONSECUTIVE_BAD_READS) break;
                continue;
            }
            consecutive_bad_reads = 0;
            if (!sys.onFrame(fd)) {
                spdlog::error("FAIL: equivalence run (async={}) lost frame {}", async,
                              fd.frame_id);
                g_ok = false;
                break;
            }
            ++frames_fed;
        }
        sys.stop();
        reader.close();
        return frames_fed;
    };

    std::vector<LocalizationOutput> sync_out;
    std::vector<LocalizationOutput> async_out;
    int fed_sync = 0, fed_async = 0;
    run_with_deadline("equivalence: synchronous run",
                      [&] { fed_sync = run(false, sync_out); },
                      std::chrono::milliseconds(10 * OP_DEADLINE_MS));
    if (fed_sync < 0) {
        spdlog::warn("test_system_manager: cannot open video '{}' — async/sync "
                     "equivalence check SKIPPED (dataset absent)",
                     reader_cfg.video_path);
        return;  // soft-skip: still PASS
    }
    run_with_deadline("equivalence: asynchronous run",
                      [&] { fed_async = run(true, async_out); },
                      std::chrono::milliseconds(10 * OP_DEADLINE_MS));

    spdlog::info("equivalence: fed sync={} async={} | outputs sync={} async={}",
                 fed_sync, fed_async, sync_out.size(), async_out.size());
    if (!check(fed_sync > 0 && fed_sync == fed_async,
               "equivalence: both runs consumed the same frames")) {
        return;
    }
    if (!check(sync_out.size() == async_out.size() && !sync_out.empty(),
               "equivalence: both runs produced the same number of outputs")) {
        return;
    }

    std::size_t first_diff = sync_out.size();
    for (std::size_t i = 0; i < sync_out.size(); ++i) {
        const LocalizationOutput& a = sync_out[i];
        const LocalizationOutput& b = async_out[i];
        // Bitwise on purpose: this is a reproducibility check, not an accuracy
        // one, so "close enough" would hide exactly what it must catch.
        // accuracy_m is NaN when no covariance exists, and NaN != NaN, so the
        // two are compared with a NaN-aware equality: both NaN counts as equal,
        // one NaN does not.
        const bool same_accuracy =
            (std::isnan(a.accuracy_m) && std::isnan(b.accuracy_m)) ||
            a.accuracy_m == b.accuracy_m;
        const bool same = a.frame_id == b.frame_id &&
                          a.timestamp_msec == b.timestamp_msec && a.valid == b.valid &&
                          a.latitude == b.latitude && a.longitude == b.longitude &&
                          a.altitude_m == b.altitude_m && a.roll == b.roll &&
                          a.pitch == b.pitch && a.yaw == b.yaw && a.health == b.health &&
                          same_accuracy;
        if (!same) {
            first_diff = i;
            spdlog::error("  output {} differs: sync(frame {}, valid {}, "
                          "{:.9f}, {:.9f}, {:.6f}) vs async(frame {}, valid {}, "
                          "{:.9f}, {:.9f}, {:.6f})",
                          i, a.frame_id, a.valid, a.latitude, a.longitude, a.altitude_m,
                          b.frame_id, b.valid, b.latitude, b.longitude, b.altitude_m);
            break;
        }
    }
    check(first_diff == sync_out.size(),
          "equivalence: async and sync LocalizationOutput sequences are identical");

    const std::size_t valid_count = static_cast<std::size_t>(
        std::count_if(sync_out.begin(), sync_out.end(),
                      [](const LocalizationOutput& o) { return o.valid; }));
    spdlog::info("equivalence: {} of {} compared outputs carry a valid fix",
                 valid_count, sync_out.size());
    check(valid_count > 0,
          "equivalence: the comparison covered geo-referenced fixes, not just "
          "empty outputs");
}

// ═══ S6c — the four typed channels ════════════════════════════════════════

//! Records every FrameProcessed, telemetry included. That payload is how the
//! ASSEMBLY STEP becomes observable: on the four-channel path its `telemetry`
//! is what SystemManager rebuilt from the extrapolator, not what any test
//! handed it directly.
class FrameProbe {
public:
    explicit FrameProbe(SystemManager& sys) {
        sys.callbacks().on_frame_processed.add(
            [this](double, const uavloc::core::FrameProcessed& fp) {
                std::lock_guard<std::mutex> lock(mutex_);
                frames_.push_back(fp);
                count_.store(frames_.size());
            });
    }

    std::size_t count() const { return count_.load(); }

    uavloc::core::FrameProcessed at(std::size_t i) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_.at(i);
    }

private:
    mutable std::mutex                        mutex_;
    std::vector<uavloc::core::FrameProcessed> frames_;
    std::atomic<std::size_t>                  count_{0};
};

//! Synchronous manager for the channel checks: no fusion (the assembly step is
//! independent of the back-end, and leaving it out keeps the check fast and
//! deterministic), monitor thread off.
SystemConfig channel_config() {
    SystemConfig cfg = async_config(uavloc::core::InputPolicy::BLOCK, 4,
                                    LONG_PUSH_TIMEOUT_MS, 0, /*async_input=*/false);
    return cfg;
}

//! One synthetic telemetry sample, pushed as the three non-image channels.
//! Values are arbitrary but distinct per field, so a mis-wired assembly step
//! (pan into tilt, lat into lon, …) shows up as a mismatch rather than as a
//! coincidence.
constexpr double CH_ROLL   = -3.5;
constexpr double CH_PITCH  = 2.25;
constexpr double CH_YAW    = 137.75;
constexpr double CH_PAN    = 180.0;
constexpr double CH_TILT   = 90.0;
constexpr double CH_LAT    = 47.5712345;
constexpr double CH_LON    = -52.7123456;
constexpr double CH_ALT    = 123.5;
constexpr double CH_SPEED  = 17.25;

void push_full_state(SystemManager& sys, double t_msec) {
    uavloc::sensor::AttitudeData att;
    att.roll_deg  = CH_ROLL;
    att.pitch_deg = CH_PITCH;
    att.yaw_deg   = CH_YAW;
    check(sys.onAttitude(t_msec, att), "channels: onAttitude accepted");

    uavloc::sensor::GimbalData gim;
    gim.pan_deg  = CH_PAN;
    gim.tilt_deg = CH_TILT;
    check(sys.onGimbal(t_msec, gim), "channels: onGimbal accepted");

    uavloc::sensor::GnssData gnss;
    gnss.latitude_deg  = CH_LAT;
    gnss.longitude_deg = CH_LON;
    gnss.altitude_m    = CH_ALT;
    gnss.speed_mps     = CH_SPEED;
    check(sys.onGnss(t_msec, gnss), "channels: onGnss accepted");
}

//! Image channel payload built from the same synthetic image the async checks
//! use, so the VO front-end does real work.
uavloc::sensor::ImageData make_image(unsigned long long id) {
    const uavloc::sensor::FrameData fd = make_frame(id);
    uavloc::sensor::ImageData img;
    img.frame_id  = id;
    img.image     = fd.image;
    img.camera_id = "syn";
    return img;
}

// ── 16. Four channels pumped by hand ──────────────────────────────────────
void test_channels_manual_feed() {
    SystemManager sys(channel_config());
    FrameProbe    probe(sys);
    int           vo_data_count = 0;
    sys.callbacks().on_vo_data.add(
        [&](double, const uavloc::vo::VOData&) { ++vo_data_count; });
    if (!check(sys.setup() && sys.start(), "manual channels: setup + start")) return;

    constexpr double T_MSEC = 1000.0;
    push_full_state(sys, T_MSEC);
    // The three channels above must not have run the pipeline: only the image
    // triggers a cycle (design §3.1).
    check(probe.count() == 0 && sys.stats().frames_received == 0,
          "manual channels: the 3 buffering channels trigger no processing");

    check(sys.onImage(T_MSEC, make_image(0)), "manual channels: onImage accepted");
    sys.stop();

    const uavloc::core::SystemStats st = sys.stats();
    if (!check(probe.count() == 1 && st.frames_processed == 1,
               "manual channels: one onImage() produced exactly one processed frame")) {
        return;
    }
    check(vo_data_count == 1, "manual channels: on_vo_data fired exactly once");
    check(st.frames_without_telemetry == 0,
          "manual channels: the frame was NOT counted as telemetry-less");

    const uavloc::core::FrameProcessed fp = probe.at(0);
    const uavloc::sensor::TelemetryData& t = fp.telemetry;
    check(fp.has_telemetry, "manual channels: the assembled frame carries telemetry");
    // Exact equality: an exact timestamp hit is a VERBATIM copy (Extrapolator
    // rule ①), so anything but == would hide a stray transformation.
    check(t.roll_deg == CH_ROLL && t.pitch_deg == CH_PITCH &&
              t.heading_deg == CH_YAW && t.gimbal_pan_deg == CH_PAN &&
              t.gimbal_tilt_deg == CH_TILT && t.latitude_deg == CH_LAT &&
              t.longitude_deg == CH_LON && t.altitude_m == CH_ALT &&
              t.speed_mps == CH_SPEED,
          "manual channels: every telemetry field was rebuilt EXACTLY");
    check(t.timestamp_msec == T_MSEC && t.frame_id == 0 && t.valid,
          "manual channels: timestamp / frame_id / valid are set on the rebuilt record");
}

// ── 17. A missing channel ⇒ no telemetry, nothing invented ────────────────
void test_channels_missing_attitude() {
    SystemManager sys(channel_config());
    FrameProbe    probe(sys);
    if (!check(sys.setup() && sys.start(), "missing channel: setup + start")) return;

    constexpr double T_MSEC = 2000.0;
    // Gimbal + GNSS only: the attitude channel never fires for this instant.
    uavloc::sensor::GimbalData gim;
    gim.pan_deg  = CH_PAN;
    gim.tilt_deg = CH_TILT;
    sys.onGimbal(T_MSEC, gim);
    uavloc::sensor::GnssData gnss;
    gnss.latitude_deg  = CH_LAT;
    gnss.longitude_deg = CH_LON;
    gnss.altitude_m    = CH_ALT;
    sys.onGnss(T_MSEC, gnss);

    check(sys.onImage(T_MSEC, make_image(0)),
          "missing channel: the frame is still accepted");
    sys.stop();

    const uavloc::core::SystemStats st = sys.stats();
    if (!check(probe.count() == 1 && st.frames_processed == 1,
               "missing channel: the frame went through the pipeline")) {
        return;
    }
    const uavloc::core::FrameProcessed fp = probe.at(0);
    check(!fp.has_telemetry,
          "missing channel: has_telemetry == false (VideoReader's own behaviour)");
    // NOT "close to zero": a default-constructed record, i.e. the partial
    // gimbal/GNSS state was NOT smuggled in alongside a fabricated attitude.
    const uavloc::sensor::TelemetryData& t = fp.telemetry;
    check(t.roll_deg == 0.0 && t.pitch_deg == 0.0 && t.heading_deg == 0.0 &&
              t.gimbal_pan_deg == 0.0 && t.gimbal_tilt_deg == 0.0 &&
              t.latitude_deg == 0.0 && t.longitude_deg == 0.0 &&
              t.altitude_m == 0.0 && !t.valid,
          "missing channel: the telemetry record is default, nothing was invented");
    check(st.frames_without_telemetry == 1,
          "missing channel: SystemStats::frames_without_telemetry counted it");
}

// ── 18. Wrong order: the image fires FIRST ────────────────────────────────
void test_channels_wrong_order() {
    SystemManager sys(channel_config());
    FrameProbe    probe(sys);
    if (!check(sys.setup() && sys.start(), "wrong order: setup + start")) return;

    constexpr double T_MSEC = 3000.0;

    // (a) Image first, buffers still empty.
    check(sys.onImage(T_MSEC, make_image(0)), "wrong order: the early image is accepted");
    // (b) The three states arrive AFTERWARDS, at the same instant…
    push_full_state(sys, T_MSEC);
    // (c) …and the next image at that instant does get them.
    check(sys.onImage(T_MSEC, make_image(1)), "wrong order: the later image is accepted");
    sys.stop();

    if (!check(probe.count() == 2, "wrong order: both frames were processed")) return;
    const uavloc::core::FrameProcessed first  = probe.at(0);
    const uavloc::core::FrameProcessed second = probe.at(1);
    check(!first.has_telemetry,
          "wrong order: the frame published before its state has NO telemetry");
    check(second.has_telemetry && second.telemetry.heading_deg == CH_YAW,
          "wrong order: the next frame at the same instant DOES have it — which "
          "is exactly why a source must publish the image LAST");
    check(sys.stats().frames_without_telemetry == 1,
          "wrong order: exactly one frame was counted as telemetry-less");
}

// ── 19 + 20. attachSource: the system is driven, it does not drive ────────
//! Frames to observe before the mid-stream stop() of check 20.
constexpr std::size_t SOURCE_RUN_FRAMES  = 100;
constexpr std::size_t SOURCE_STOP_FRAMES = 30;
//! Deadline for the streamed part of checks 19/20 [ms]. Generous: the MUN-FRL
//! front end runs at ~15 fps, so 100 frames need seconds, not milliseconds.
constexpr int SOURCE_DEADLINE_MS = 120000;

//! Builds a VideoDataSource over the mission config's VideoReader, or nullptr
//! when the gitignored dataset video is absent (⇒ soft-skip).
std::unique_ptr<uavloc::sensor::VideoDataSource> make_dataset_source(
    const uavloc::sensor::VideoReaderConfig& reader_cfg) {
    auto reader = std::make_unique<uavloc::sensor::VideoReader>(reader_cfg);
    if (!reader->open()) {
        return nullptr;
    }
    reader->close();  // VideoDataSource::startStreaming() re-opens it
    return std::make_unique<uavloc::sensor::VideoDataSource>(std::move(reader));
}

//! Shared setup of checks 19 and 20. Returns false when the config cannot be
//! read (a real failure) — the dataset-absent case is reported by `skipped`.
bool load_dataset_config(const std::string& config_path, SystemConfig& sys_cfg,
                         uavloc::sensor::VideoReaderConfig& reader_cfg) {
    if (config_path.empty()) {
        spdlog::error("FAIL: source check has no mission config path");
        g_ok = false;
        return false;
    }
    try {
        const YAML::Node root = YAML::LoadFile(config_path);  // READ-ONLY
        sys_cfg    = SystemConfig::fromYaml(root);
        reader_cfg = uavloc::sensor::VideoReaderConfig::fromYaml(root);
    } catch (const std::exception& ex) {
        spdlog::error("FAIL: cannot parse mission config '{}': {}", config_path, ex.what());
        g_ok = false;
        return false;
    }
    // Synchronous fusion + inline input: the source thread runs the pipeline
    // itself, which is the configuration a live rig would use with a slow
    // enough camera and the one that keeps this check deterministic.
    sys_cfg.fusion.async_enabled = false;
    sys_cfg.async_input          = false;
    sys_cfg.stats_period_ms      = 0;
    return true;
}

void test_source_runs_the_system(const std::string& config_path) {
    SystemConfig                      sys_cfg;
    uavloc::sensor::VideoReaderConfig reader_cfg;
    if (!load_dataset_config(config_path, sys_cfg, reader_cfg)) return;

    auto source = make_dataset_source(reader_cfg);
    if (!source) {
        spdlog::warn("test_system_manager: cannot open video '{}' — attachSource "
                     "check SKIPPED (dataset absent)", reader_cfg.video_path);
        return;  // soft-skip: still PASS
    }

    SystemManager sys(sys_cfg);
    std::atomic<std::size_t> outputs{0};
    std::atomic<std::size_t> valid{0};
    sys.callbacks().on_localization.add([&](const LocalizationOutput& o) {
        if (o.valid) ++valid;
        ++outputs;
    });

    sys.attachSource(std::move(source));
    if (!check(sys.setup() && sys.start(), "source run: setup + start")) return;
    // sourceCompletion() must answer while the source is attached; the exact
    // value is a progress figure, so only its domain is asserted.
    const double completion = sys.sourceCompletion();
    check(std::isnan(completion) || (completion >= 0.0 && completion <= 1.0),
          "source run: sourceCompletion() is NaN or within [0, 1]");

    // NOBODY calls read() here — that is the point of the push model.
    const bool reached = wait_until(
        [&] { return outputs.load() >= SOURCE_RUN_FRAMES; },
        std::chrono::milliseconds(SOURCE_DEADLINE_MS));
    run_with_deadline("source run: stop()", [&] { sys.stop(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("source run: outputs={} valid={} received={} processed={} "
                 "dropped={} without_telemetry={}",
                 outputs.load(), valid.load(), st.frames_received,
                 st.frames_processed, st.frames_dropped,
                 st.frames_without_telemetry);
    check(reached, "source run: 100 frames flowed with nobody calling read()");
    check(outputs.load() >= SOURCE_RUN_FRAMES, "source run: on_localization fired");
    check(valid.load() >= 1, "source run: at least one LocalizationOutput is valid");
    check(st.frames_dropped == 0, "source run: no frame was refused");
}

void test_source_stop_midstream(const std::string& config_path) {
    SystemConfig                      sys_cfg;
    uavloc::sensor::VideoReaderConfig reader_cfg;
    if (!load_dataset_config(config_path, sys_cfg, reader_cfg)) return;

    auto source = make_dataset_source(reader_cfg);
    if (!source) {
        spdlog::warn("test_system_manager: cannot open video '{}' — mid-stream stop "
                     "check SKIPPED (dataset absent)", reader_cfg.video_path);
        return;  // soft-skip: still PASS
    }

    SystemManager sys(sys_cfg);
    ProcessedProbe probe(sys, std::chrono::milliseconds(0));
    sys.attachSource(std::move(source));
    if (!check(sys.setup() && sys.start(), "mid-stream stop: setup + start")) return;

    // Let the stream get going, then tear down underneath it.
    check(wait_until([&] { return probe.count() >= SOURCE_STOP_FRAMES; },
                     std::chrono::milliseconds(SOURCE_DEADLINE_MS)),
          "mid-stream stop: the source is streaming before stop() is called");

    run_with_deadline("mid-stream stop: stop() while the source is streaming",
                      [&] { sys.stop(); }, std::chrono::milliseconds(OP_DEADLINE_MS));

    const uavloc::core::SystemStats st = sys.stats();
    spdlog::info("mid-stream stop: received={} processed={} dropped={} probe={}",
                 st.frames_received, st.frames_processed, st.frames_dropped,
                 probe.count());
    check(sys.state() == SystemState::STOPPED, "mid-stream stop: state is STOPPED");
    // The books must close: nothing accepted was thrown away.
    check(st.frames_processed + st.frames_dropped == st.frames_received,
          "mid-stream stop: processed + dropped == received");
    check(probe.count() == st.frames_processed,
          "mid-stream stop: the pipeline ran once per processed frame");
    check(st.frames_dropped == 0,
          "mid-stream stop: no frame accepted before the teardown was lost");

    // Nothing may still reach the system afterwards, and a second stop() is a
    // no-op even with a source attached.
    const unsigned long long processed_at_stop = st.frames_processed;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sys.stop();
    check(sys.stats().frames_processed == processed_at_stop,
          "mid-stream stop: nothing was processed after stop() returned");
}

// ── 21. pauseSource() / resumeSource(): no frame lost, no frame replayed ──
//! Frames to see before pausing, and after resuming.
constexpr std::size_t PAUSE_AFTER_FRAMES  = 20;
constexpr std::size_t RESUME_AFTER_FRAMES = 20;
//! How long to sit still while paused [ms]. Long enough that a reader that did
//! NOT stop would have published several more frames (the front end runs at a
//! few frames per second here).
constexpr int PAUSE_HOLD_MS = 1000;

void test_source_pause_resume(const std::string& config_path) {
    SystemConfig                      sys_cfg;
    uavloc::sensor::VideoReaderConfig reader_cfg;
    if (!load_dataset_config(config_path, sys_cfg, reader_cfg)) return;

    auto source = make_dataset_source(reader_cfg);
    if (!source) {
        spdlog::warn("test_system_manager: cannot open video '{}' — pause/resume "
                     "check SKIPPED (dataset absent)", reader_cfg.video_path);
        return;  // soft-skip: still PASS
    }

    SystemManager sys(sys_cfg);
    ProcessedProbe probe(sys, std::chrono::milliseconds(0));
    sys.attachSource(std::move(source));
    if (!check(sys.setup() && sys.start(), "pause/resume: setup + start")) return;
    check(!sys.isSourcePaused(), "pause/resume: a fresh run is not paused");

    if (!check(wait_until([&] { return probe.count() >= PAUSE_AFTER_FRAMES; },
                          std::chrono::milliseconds(SOURCE_DEADLINE_MS)),
               "pause/resume: the source is streaming before the pause")) {
        sys.stop();
        return;
    }

    check(sys.pauseSource(), "pause/resume: pauseSource() accepted");
    check(sys.isSourcePaused(), "pause/resume: isSourcePaused() reports the pause");
    check(sys.state() == SystemState::RUNNING,
          "pause/resume: the SYSTEM stays RUNNING — a pause is not a stop()");
    // pauseSource() joins the reading thread, so the count is final the moment
    // it returns; anything appearing later means the reader kept going.
    const std::size_t at_pause = probe.count();
    std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_HOLD_MS));
    check(probe.count() == at_pause,
          "pause/resume: NO frame is processed while paused");
    check(sys.pauseSource(), "pause/resume: pausing twice is a no-op that succeeds");

    check(sys.resumeSource(), "pause/resume: resumeSource() accepted");
    check(!sys.isSourcePaused(), "pause/resume: isSourcePaused() clears on resume");
    check(wait_until([&] { return probe.count() >= at_pause + RESUME_AFTER_FRAMES; },
                     std::chrono::milliseconds(SOURCE_DEADLINE_MS)),
          "pause/resume: frames flow again after the resume");
    run_with_deadline("pause/resume: stop() after a resume", [&] { sys.stop(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    // ── the actual gate: the frame-id sequence ───────────────────────────────
    // A pause that rewound the video would repeat ids; one that dropped the
    // buffered frames would leave a hole. Both are fatal for every number this
    // pipeline produces afterwards, so they are asserted, not eyeballed.
    const std::vector<unsigned int> ids = probe.ids();
    if (!check(ids.size() > at_pause, "pause/resume: frames exist on both sides")) {
        return;
    }
    bool strictly_increasing = true;
    bool contiguous          = true;
    for (std::size_t i = 1; i < ids.size(); ++i) {
        if (ids[i] <= ids[i - 1])      strictly_increasing = false;
        if (ids[i] != ids[i - 1] + 1)  contiguous = false;
    }
    const unsigned int boundary_step = ids[at_pause] - ids[at_pause - 1];
    spdlog::info("pause/resume: {} frames before the pause, {} after; last id "
                 "before = {}, first id after = {} (step {})",
                 at_pause, ids.size() - at_pause, ids[at_pause - 1],
                 ids[at_pause], boundary_step);
    check(strictly_increasing,
          "pause/resume: frame ids strictly increase — the video was NOT rewound");
    check(boundary_step == 1,
          "pause/resume: the id right after the resume is the one right after "
          "the pause — no frame lost, none replayed");
    check(contiguous,
          "pause/resume: the whole id sequence is contiguous across the pause");
}

// ── 22. Angle range at the point of consumption ───────────────────────────
// See the wrap180_deg() comment in src/core/system_manager.cpp for the two
// measurements this rule rests on: MUN-FRL yaw is in [0, 360), YenBai yaw is in
// (-180, 180], and src/fusion/factors.h imposes NO range (heading only ever
// reaches sin/cos or a wrapped difference).
void test_channels_angle_range() {
    //! Attitude samples bracketing the 0/360 seam, and the instant between them
    //! [ms]. slerp at alpha = 0.25 of 340 → 20 (a +40° arc) is 350, which the
    //! quaternion round trip returns as -10.
    constexpr double T_LO = 0.0, T_HI = 100.0, T_MID = 25.0;
    constexpr double YAW_LO = 340.0, YAW_HI = 20.0;
    constexpr double EXPECTED_INTERP_YAW = -10.0;
    constexpr double ANGLE_TOL_DEG       = 1e-9;

    SystemManager sys(channel_config());
    FrameProbe    probe(sys);
    if (!check(sys.setup() && sys.start(), "angle range: setup + start")) return;

    uavloc::sensor::AttitudeData a_lo;
    a_lo.yaw_deg = YAW_LO;
    uavloc::sensor::AttitudeData a_hi;
    a_hi.yaw_deg = YAW_HI;
    sys.onAttitude(T_LO, a_lo);
    sys.onAttitude(T_HI, a_hi);

    uavloc::sensor::GimbalData gim;
    gim.pan_deg  = CH_PAN;
    gim.tilt_deg = CH_TILT;
    uavloc::sensor::GnssData gnss;
    gnss.latitude_deg = CH_LAT;
    gnss.longitude_deg = CH_LON;
    gnss.altitude_m    = CH_ALT;
    for (double t : {T_LO, T_HI}) {
        sys.onGimbal(t, gim);
        sys.onGnss(t, gnss);
    }

    // Frame 0 at T_MID: attitude is INTERPOLATED (gimbal/GNSS are too).
    check(sys.onImage(T_MID, make_image(0)), "angle range: interpolated frame accepted");
    // Frame 1 at T_LO: attitude is an EXACT hit.
    check(sys.onImage(T_LO, make_image(1)), "angle range: exact-hit frame accepted");
    sys.stop();

    if (!check(probe.count() == 2, "angle range: both frames were processed")) return;
    const uavloc::sensor::TelemetryData interp = probe.at(0).telemetry;
    const uavloc::sensor::TelemetryData exact  = probe.at(1).telemetry;

    check(probe.at(0).has_telemetry && probe.at(1).has_telemetry,
          "angle range: both frames carry telemetry");
    check(interp.heading_deg > -180.0 && interp.heading_deg <= 180.0,
          "angle range: an INTERPOLATED heading lands in (-180, 180]");
    check(std::abs(interp.heading_deg - EXPECTED_INTERP_YAW) < ANGLE_TOL_DEG,
          "angle range: the interpolated heading is the shortest-arc value "
          "(-10), not the 260 a linear Euler blend would give");
    // The half that guards the bit-identical gate: an exact hit keeps the
    // producer's range. 340 must NOT come back as -20.
    check(exact.heading_deg == YAW_LO,
          "angle range: an EXACT hit is passed through VERBATIM (340 stays 340)");
}

} // namespace

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);

    const std::string config_path =
        (argc > 1) ? argv[1] : std::string(UAVLOC_MISSION_CONFIG_PATH);

    test_input_refused_before_start();
    test_start_without_setup();
    test_lifecycle();
    test_channels();
    test_latlon_from_enu();
    test_altitude_datum();
    test_real_run(config_path);

    // S5 — asynchronous input path.
    test_async_block_no_loss();
    test_async_drop_oldest();
    test_stop_races_producer();
    test_stop_drains_queue();
    test_stop_wakes_blocked_producer();
    test_monitor_thread();
    test_async_input_after_stop();
    test_async_matches_sync(config_path);

    // S6c — the four typed channels + attachSource.
    test_channels_manual_feed();
    test_channels_missing_attitude();
    test_channels_wrong_order();
    test_channels_angle_range();
    test_source_runs_the_system(config_path);
    test_source_stop_midstream(config_path);
    test_source_pause_resume(config_path);

    if (!g_ok) {
        spdlog::error("test_system_manager: FAIL");
        return 1;
    }
    spdlog::info("test_system_manager: PASS");
    return 0;
}
