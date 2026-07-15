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
// Skeleton scope (F1 steps 1+2): plumbing + threading + a minimal graph
// (PriorFactor anchor on X(0), BetweenFactor<Pose3> from the raw VO relative
// pose between keyframes) proving GTSAM linkage. The real factors
// (ScaledVOFactor, DeltaYaw, AGL, scale/bias walks, θ) land in a later step —
// see .docs/theory/heading_agl_prior_tactics.md §7.
//
// This header is gtsam-free; all GTSAM usage stays in src/fusion/.

#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/sensor/telemetry_data.h"

#include <functional>
#include <memory>

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

    //! Register a subscriber invoked once per processed frame, in registration
    //! order, on the processing thread (fusion thread in async mode).
    void add_result_callback(std::function<void(const FusionResult&)> cb);

    //! Thread-safe snapshot of the newest result.
    FusionResult latest() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::fusion
