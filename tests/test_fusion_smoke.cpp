// test_fusion_smoke — Stage F1 skeleton smoke test for the fusion module.
//
// No video/data needed: feeds synthetic VOResults (straight-line motion,
// every 10th frame a keyframe) + synthetic telemetry through FusionModule in
// both async and sync mode, and checks:
//   * the result callback fires once per frame, in frame order;
//   * latest() reports a valid, finite pose;
//   * double-stop does not crash;
//   * two synchronous runs produce bit-identical pose sequences (determinism).
// Headless; exits non-zero on failure.

#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/fusion/fusion_module.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>

namespace {

constexpr int    NUM_FRAMES        = 60;
constexpr int    KEYFRAME_STRIDE   = 10;   // every 10th frame is a keyframe
constexpr double FRAME_DT_MSEC     = 40.0;
constexpr double STEP_X_M          = 1.0;  // straight-line motion, 1 m/frame in X
constexpr double TELEM_ALTITUDE_M  = 800.0;
constexpr double TELEM_HEADING_DEG = 90.0;
// With the F1 factors the ENU anchor sits at (0, 0, agl_0) and the AGL factor
// pins Z, so every fused pose must stay near the telemetry altitude.
constexpr double Z_ANCHOR_TOL_M = 50.0;

uavloc::vo::VOResult make_vo_result(int i) {
    uavloc::vo::VOResult res;
    res.frame_id       = static_cast<unsigned int>(i);
    res.timestamp_msec = i * FRAME_DT_MSEC;
    res.state          = uavloc::vo::VOTrackingState::TRACKING;
    res.T_wc           = uavloc::Mat44_t::Identity();
    res.T_wc(0, 3)     = i * STEP_X_M;
    res.T_prev_curr    = uavloc::Mat44_t::Identity();
    res.T_prev_curr(0, 3) = STEP_X_M;
    res.has_pose    = true;
    res.is_keyframe = (i % KEYFRAME_STRIDE == 0);
    return res;
}

uavloc::sensor::TelemetryData make_telemetry(int i) {
    uavloc::sensor::TelemetryData telem;
    telem.frame_id       = static_cast<uint64_t>(i);
    telem.timestamp_msec = i * FRAME_DT_MSEC;
    telem.heading_deg    = TELEM_HEADING_DEG;
    telem.altitude_m     = TELEM_ALTITUDE_M;
    telem.valid          = true;
    return telem;
}

struct FeedOutcome {
    std::vector<uavloc::fusion::FusionResult> results;
    uavloc::fusion::FusionResult              latest;
};

FeedOutcome run_feed(bool async_enabled) {
    uavloc::fusion::FusionConfig cfg;  // defaults
    cfg.async_enabled = async_enabled;

    uavloc::fusion::FusionModule module(cfg);

    FeedOutcome outcome;
    // Callback fires on the fusion thread (async) / caller thread (sync); the
    // vector is only read after stop(), so no extra synchronization is needed.
    module.add_result_callback([&outcome](const uavloc::fusion::FusionResult& r) {
        outcome.results.push_back(r);
    });

    module.start();
    for (int i = 0; i < NUM_FRAMES; ++i) {
        module.push(make_vo_result(i), make_telemetry(i));
    }
    module.stop();
    module.stop();  // double-stop must be a safe no-op

    outcome.latest = module.latest();
    return outcome;
}

bool check_outcome(const FeedOutcome& outcome, const char* label) {
    bool ok = true;

    if (outcome.results.size() != static_cast<size_t>(NUM_FRAMES)) {
        spdlog::error("[{}] callback fired {} times, expected {}",
                      label, outcome.results.size(), NUM_FRAMES);
        ok = false;
    }
    for (size_t i = 0; i < outcome.results.size(); ++i) {
        const auto& r = outcome.results[i];
        if (r.frame_id != i) {
            spdlog::error("[{}] result {} out of order: frame_id={}", label, i, r.frame_id);
            ok = false;
            break;
        }
        if (!r.T_enu_c.matrix().allFinite()) {
            spdlog::error("[{}] non-finite pose at frame {}", label, r.frame_id);
            ok = false;
            break;
        }
    }
    if (!outcome.latest.has_pose) {
        spdlog::error("[{}] latest().has_pose is false", label);
        ok = false;
    }
    if (!outcome.latest.T_enu_c.matrix().allFinite()) {
        spdlog::error("[{}] latest() pose is not finite", label);
        ok = false;
    }
    const double z_last = outcome.latest.T_enu_c.translation().z();
    if (std::abs(z_last - TELEM_ALTITUDE_M) > Z_ANCHOR_TOL_M) {
        spdlog::error("[{}] latest() z={:.1f} m not anchored near AGL {:.1f} m",
                      label, z_last, TELEM_ALTITUDE_M);
        ok = false;
    }
    return ok;
}

bool sequences_identical(const FeedOutcome& a, const FeedOutcome& b,
                         const char* label_a, const char* label_b) {
    if (a.results.size() != b.results.size()) {
        spdlog::error("size mismatch: {}={} vs {}={}",
                      label_a, a.results.size(), label_b, b.results.size());
        return false;
    }
    for (size_t i = 0; i < a.results.size(); ++i) {
        const auto& ra = a.results[i];
        const auto& rb = b.results[i];
        const bool same_pose =
            (ra.T_enu_c.matrix().array() == rb.T_enu_c.matrix().array()).all();
        if (ra.frame_id != rb.frame_id || ra.has_pose != rb.has_pose || !same_pose) {
            spdlog::error("frame {}: {} vs {} results differ (has_pose {}/{})",
                          i, label_a, label_b, ra.has_pose, rb.has_pose);
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    spdlog::set_level(spdlog::level::info);

    // ── Async leg ──────────────────────────────────────────────────────────
    spdlog::info("test_fusion_smoke: async leg ({} frames, keyframe every {})",
                 NUM_FRAMES, KEYFRAME_STRIDE);
    const FeedOutcome async_run = run_feed(/*async_enabled=*/true);
    if (!check_outcome(async_run, "async")) {
        return 1;
    }

    // ── Sync legs (determinism) ────────────────────────────────────────────
    spdlog::info("test_fusion_smoke: sync legs (determinism check)");
    const FeedOutcome sync_run_1 = run_feed(/*async_enabled=*/false);
    const FeedOutcome sync_run_2 = run_feed(/*async_enabled=*/false);
    if (!check_outcome(sync_run_1, "sync#1") || !check_outcome(sync_run_2, "sync#2")) {
        return 1;
    }
    if (!sequences_identical(sync_run_1, sync_run_2, "sync#1", "sync#2")) {
        spdlog::error("test_fusion_smoke: sync runs are not deterministic");
        return 1;
    }

    const auto& fused_last = sync_run_1.results.back();
    spdlog::info("test_fusion_smoke: last fused pose t=({:.3f}, {:.3f}, {:.3f}) "
                 "health={} base_kf={}",
                 fused_last.T_enu_c.translation().x(),
                 fused_last.T_enu_c.translation().y(),
                 fused_last.T_enu_c.translation().z(),
                 static_cast<int>(fused_last.health), fused_last.base_keyframe_id);
    spdlog::info("test_fusion_smoke: PASS (async + sync deterministic)");
    return 0;
}
