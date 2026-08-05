#pragma once

// FusionModule — GTSAM fixed-lag-smoother back-end (Stage F1 skeleton).
//
// Consumes per-frame VO results + synced telemetry and produces fused ENU
// camera poses (FusionResult). Keyframes trigger a smoother update; other
// frames only propagate the output pose from the last optimized state —
// cheap, no graph touch.
//
// Two modes, selected by FusionConfig::async_enabled (mirrors MappingModule):
//   * asynchronous (default): push() enqueues (non-blocking, never drops —
//     keyframes are required by the graph) and a dedicated fusion thread runs
//     the smoother; callbacks fire on the fusion thread.
//   * synchronous: push() processes inline on the caller's thread —
//     deterministic baseline; callbacks fire on the caller's thread.
//
// F1 graph (see .docs/theory/heading_agl_prior_tactics.md §5.3/§6/§7):
// states x(k) Pose3 T_enu_camera, s(k) VO-scale multiplier, b(k) AGL-vs-ENU-Z
// bias. Per-keyframe factors: ScaledVOFactor (Huber), scale/bias random walks,
// offset-immune
// DeltaYawFactor (heading deltas, slew-rate gated), rotation-only roll/pitch
// attitude prior (yaw loose), AGL Z factor. X(0) is anchored at ENU
// (0, 0, agl_0) with the telemetry-measured rotation.
//
// This header is gtsam-free; all GTSAM usage stays in src/fusion/.

#include "uavloc/anchor/absolute_fix.h"
#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/sensor/telemetry_data.h"

#include <functional>
#include <memory>
#include <vector>

namespace uavloc::fusion {

class FusionModule {
public:
    FusionModule() = delete;

    explicit FusionModule(const FusionConfig& config);

    ~FusionModule();

    FusionModule(const FusionModule&) = delete;
    FusionModule& operator=(const FusionModule&) = delete;

    //! Spawn the fusion thread when async_enabled (no-op otherwise).
    void start();

    //! Drain the remaining queue, then join the thread; idempotent.
    void stop();

    //! Called every frame from the tracking thread. Non-blocking in async mode
    //! (enqueues); runs inline in sync mode. Keyframes trigger a graph update,
    //! other frames only propagate the output pose.
    void push(const vo::VOResult& res, const sensor::TelemetryData& telem);

    //! Hand an absolute horizontal position measurement to the back-end
    //! (M1). Thread-safe and non-blocking in both modes: the fix is queued
    //! (drop-oldest — a stale fix is worthless and must never stall the
    //! pipeline) and consumed at the next keyframe graph update, which
    //! attaches it to the keyframe state X(k) closest in time (within
    //! FusionConfig::fix_match_tolerance_sec). A fix whose keyframe has already
    //! left the fixed-lag window is dropped (FusionFixStats::marginalized), as
    //! is one older than the age budget (FusionFixStats::age_expired); one that
    //! is still inside the window corrects the PAST as well as the present.
    //!
    //! Fixes with valid == false, a non-finite position or a non-SPD
    //! covariance are rejected outright (not counted as injected).
    //!
    //! ⚠ Quality filtering is the PRODUCER's job: the consumer only checks
    //! AbsoluteFix::confidence against FusionConfig::fix_min_confidence (0 by
    //! default). The self-consistency Mahalanobis gate is off by default —
    //! with the M1 fake anchor (confidence always 1.0) nothing rejects a wrong
    //! fix, so a VPR producer must supply a real confidence and covariance.
    void push_absolute_fix(const anchor::AbsoluteFix& fix);

    //! Thread-safe snapshot of the cumulative absolute-fix accounting.
    FusionFixStats fix_stats() const;

    //! Register a subscriber invoked once per processed frame, in registration
    //! order, on the processing thread (fusion thread in async mode).
    void add_result_callback(std::function<void(const FusionResult&)> cb);

    //! Thread-safe snapshot of the newest result.
    FusionResult latest() const;

    //! Thread-safe copy of the corrected lag window: every keyframe pose still
    //! alive in the fixed-lag smoother, re-optimized by the latest update, in
    //! ascending frame_id. Rebuilt after every keyframe smoother update —
    //! snapshot only, never touches the graph.
    std::vector<FusionLagPose> getLagWindow() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::fusion
