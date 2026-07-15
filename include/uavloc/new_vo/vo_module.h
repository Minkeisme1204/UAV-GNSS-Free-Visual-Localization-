#pragma once

// VOModule — pure visual-odometry front-end orchestrator for the new_vo pipeline.
//
// GLUE layer: drives the state machine (NOT_INITIALIZED -> TRACKING -> LOST)
// and the per-frame loop over the already-ported components (VoFrameLoader,
// Initializer, FrameTracker, LocalMapUpdater, KeyframeInserter, LocalMapper,
// PoseOptimizer). Synchronous / deterministic — everything runs inline on the
// caller's thread. Pure VO: no relocalization, no loop closing, no BoW. See
// .docs/designs/vo_design_v3.md.

#include "uavloc/common/type.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/sensor/frame_data.h"

#include <functional>
#include <memory>
#include <vector>

namespace uavloc {
namespace vo {

namespace data {
class MapDatabase;
} // namespace data

//! VO tracking state (pure VO; no relocalization).
enum class VOTrackingState {
    NOT_INITIALIZED,
    TRACKING,
    LOST
};

//! Per-frame VO output.
struct VOResult {
    unsigned int    frame_id       = 0;
    double          timestamp_msec = 0.0;
    VOTrackingState state          = VOTrackingState::NOT_INITIALIZED;

    //! GLOBAL camera-to-world pose (last valid pose); identity until a pose exists.
    Mat44_t T_wc = Mat44_t::Identity();
    //! GLOBAL world-frame increment from the previous valid pose to this one.
    Mat44_t T_prev_curr = Mat44_t::Identity();

    bool has_pose    = false;
    bool is_keyframe = false;

    int    num_keypoints = 0;
    int    num_matches   = 0;
    int    num_inliers   = 0;
    int    num_landmarks = 0;
    double inlier_ratio  = 0.0;

    //! Undistorted pixel positions of the landmarks tracked (non-outlier,
    //! valid-landmark keypoints) in this frame. Filled on TRACKING frames after
    //! the local-map pose optimization; empty otherwise. For visualization
    //! (e.g. drawing the tracked keypoints on the frame).
    std::vector<Eigen::Vector2d> tracked_observations;
};

//! Publish payload delivered to every registered data-out callback once per
//! processed frame. Carries only Eigen/std types (no debug_viewer dependency)
//! so any consumer (fusion, anchor, a viewer bridge) can read it directly.
struct VOData {
    //! Per-frame VO result (pose, state, stats).
    VOResult result;
    //! World positions of the current map landmarks. Filled only on keyframes
    //! (when the map changed); empty otherwise.
    std::vector<Vec3_t> map_points;
    //! True when map_points was refreshed this frame (i.e. this is a keyframe).
    bool map_updated = false;
};

class VOModule {
public:
    VOModule() = delete;

    //! Construct from an assembled VOConfig.
    explicit VOModule(const VOConfig& config);

    ~VOModule();

    VOModule(const VOModule&) = delete;
    VOModule& operator=(const VOModule&) = delete;

    //! Process a single frame synchronously and return the per-frame result.
    VOResult process_frame(const sensor::FrameData& fd);

    //! Register a subscriber invoked once per processed frame with a VOData
    //! payload (pose + map points). Multiple subscribers are supported; they
    //! are called in registration order at the end of process_frame().
    void add_data_out_callback(std::function<void(const VOData&)> cb);

    //! Remove all registered data-out callbacks.
    void clear_data_out_callbacks();

    //! Current tracking state.
    VOTrackingState get_state() const;

    //! Borrowed pointer to the underlying map database (owned by VOModule).
    data::MapDatabase* get_map_database() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace vo
} // namespace uavloc
