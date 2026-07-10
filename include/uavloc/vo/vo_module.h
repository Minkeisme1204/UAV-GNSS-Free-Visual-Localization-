#pragma once

// VOModule — the single representative object that owns the whole visual-odometry
// pipeline (feature detector, projection matcher, pose estimator, initializer,
// tracker) plus its state machine, and exposes one SYNCHRONOUS entry point:
//
//     VOResult processFrame(const sensor::FrameData&);
//
// This mirrors the stella_vslam (system::feed_monocular_frame) / SVO
// (FrameHandlerMono::addImage) architecture: feed a frame in, get a pose out.
// Orchestration that previously lived in the test driver (NOT_INITIALIZED ->
// INITIALIZED -> TRACKING -> LOST welding via a global anchor) now lives here.
//
// SE3 convention (CLAUDE.md): T_wc is camera->world. VOResult::T_wc is the GLOBAL
// pose (T_world_anchor * T_wc_local), continuous across LOST / re-initialisation.

#include "uavloc/sensor/camera_model.h"
#include "uavloc/sensor/frame_data.h"
#include "uavloc/vo/feature_detector.h"
#include "uavloc/vo/pose_estimator.h"
#include "uavloc/vo/projection_matcher.h"
#include "uavloc/vo/vo_data.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace uavloc::vo {

// Aggregate configuration for the whole VO pipeline. The public sub-stage configs
// are held by value; the internal Initializer / Tracker configs reuse the same
// "VO:" YAML keys, so instead of leaking those private types into this public
// header the raw VO subtree is retained and re-parsed inside the module.
struct VOConfig {
    FeatureDetectorConfig   detector;
    ProjectionMatcherConfig matcher;
    PoseEstimatorConfig     estimator;

    // While (re-)initialising, the held reference frame is re-latched to the
    // current frame after this many consecutive NOT_READY attempts so a stale
    // reference cannot pin initialisation forever. YAML key: VO.reinit_refresh_frames.
    int reinit_refresh_frames = 20;

    // Source of the camera off-nadir angle used to promote monocular translation
    // to metric scale (slant range = AGL / cos(off_nadir)):
    //   "auto"     — gimbal tilt when it is in a plausible range, else airframe
    //   "gimbal"   — always gimbal tilt (|90 - tilt|) when in range
    //   "airframe" — always airframe attitude (cos(roll)·cos(pitch))
    // YAML key: VO.off_nadir_source.
    std::string off_nadir_source = "auto";

    // Threading mode. When false (default) the pipeline is fully synchronous and
    // DETERMINISTIC: processFrame() runs tracking + local mapping inline. When
    // true, start()/pushFrame()/stop() drive a Tracking thread + Local Mapping
    // thread; this is intentionally NOT bit-for-bit deterministic. YAML key:
    // VO.async_enabled.
    bool async_enabled = false;

    // FrameQueue (async mode) capacity — buffered frames before drop-oldest.
    // YAML key: VO.frame_queue_capacity.
    int frame_queue_capacity = 5;

    // FrameQueue (async mode) pop timeout in milliseconds — how long the Tracking
    // thread waits for a frame before re-checking its running flag.
    // YAML key: VO.frame_queue_pop_timeout_ms.
    int frame_queue_pop_timeout_ms = 50;

    // Raw "VO:" subtree, used internally to build the Initializer / Tracker
    // configs (which reuse the same VO keys). Populated by fromYaml; when left
    // null the internal stages fall back to their own defaults.
    YAML::Node vo_node;

    // Build from the mission "VO:" node; missing keys fall back to defaults.
    static VOConfig fromYaml(const YAML::Node& vo_node);
};

// Per-frame result of processFrame(). T_wc is the GLOBAL camera->world pose;
// consume it only when has_pose is true.
struct VOResult {
    int             frame_id       = 0;
    double          timestamp_msec = 0.0;
    VOTrackingState state          = VOTrackingState::NOT_INITIALIZED;
    Eigen::Matrix4d T_wc           = Eigen::Matrix4d::Identity();  // GLOBAL (welded)
    Eigen::Matrix4d T_prev_curr    = Eigen::Matrix4d::Identity();  // GLOBAL increment
    bool            has_pose       = false;
    bool            is_keyframe    = false;
    int             num_keypoints  = 0;
    int             num_matches    = 0;
    int             num_inliers    = 0;
    int             num_landmarks  = 0;
    double          inlier_ratio   = 0.0;
    // 2D pixel observations of the local-map landmarks tracked (PnP/BA inliers) in
    // THIS frame. Populated only on TRACKING frames; empty otherwise. Read-only
    // diagnostic snapshot for overlaying "landmark keypoints" in the debug viewer.
    std::vector<Eigen::Vector2d> tracked_observations;
};

class VOModule {
public:
    using StatusCallback    = std::function<void(VOTrackingState, const std::string&)>;
    using KeyframeCallback  = std::function<void(const VOResult&)>;
    // Fired for EVERY processed frame in async mode (on the Tracking thread, in
    // frame order). The single registered consumer must own all order-dependent
    // downstream state so it stays consistent on that one thread.
    using VOResultCallback  = std::function<void(const VOResult&)>;

    VOModule(const VOConfig& config, const sensor::CameraModel& camera);
    ~VOModule();

    VOModule(const VOModule&)            = delete;
    VOModule& operator=(const VOModule&) = delete;

    // Synchronous feed-frame -> pose (DEFAULT, deterministic). Returns the
    // per-frame result; the caller gates on VOResult::has_pose to decide whether
    // to consume the pose. Do not mix with the async start()/pushFrame() API.
    VOResult processFrame(const sensor::FrameData& frame);

    // ── Asynchronous API (opt-in; NOT deterministic) ──────────────────────────
    // start() spawns the Tracking + Local Mapping threads; pushFrame() enqueues a
    // frame non-blocking (drop-oldest; returns false when an older frame had to be
    // dropped); stop() joins both threads (draining pending keyframes). Per-frame
    // results are delivered through the VOResultCallback on the Tracking thread.
    void start();
    bool pushFrame(const sensor::FrameData& frame);
    void stop();

    VOTrackingState state() const;
    int             numLandmarks() const;

    // Optional observers. setStatusCallback fires on every tracking-state
    // transition; setKeyframeCallback fires whenever a keyframe is selected
    // (initialisation success or a tracking keyframe insertion);
    // setVOResultCallback fires on every processed frame in async mode.
    void setStatusCallback(StatusCallback cb);
    void setKeyframeCallback(KeyframeCallback cb);
    void setVOResultCallback(VOResultCallback cb);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace uavloc::vo
