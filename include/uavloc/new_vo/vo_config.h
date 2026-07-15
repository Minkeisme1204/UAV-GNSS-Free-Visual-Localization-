#pragma once

// VOConfig — one assembled configuration for the new_vo orchestration layer.
//
// Holds the camera intrinsics (source for camera::PerspectiveCamera), the raw
// "VO" YAML sub-tree (used verbatim to build Initializer / KeyframeInserter /
// LocalMapCleaner / LocalBundleAdjuster / PoseOptimizer — each reads with the
// node["key"].as<T>(default) pattern so missing keys never throw), plus the
// scalar parameters consumed directly by VOModule (ORB, FrameTracker,
// LocalMapUpdater, local-map tracking gate, MapDatabase, LocalMapper).

#include "uavloc/sensor/camera_model.h"

#include <yaml-cpp/yaml.h>

#include <string>

namespace uavloc {
namespace vo {

struct VOConfig {
    //! Camera intrinsics + distortion — used to build camera::PerspectiveCamera.
    sensor::CameraIntrinsics camera_intrinsics;

    //! The whole "VO" YAML sub-tree (may be empty; sub-components fall back to
    //! their own defaults). Passed to Initializer / KeyframeInserter /
    //! LocalMapCleaner / the optimizer factories.
    YAML::Node vo_node;

    //-----------------------------------------
    // ORB feature extraction (VoFrameLoader / OrbParams)
    std::string  orb_name          = "orb";
    float        orb_scale_factor  = 1.2f;
    unsigned int orb_num_levels    = 8;
    unsigned int orb_ini_fast_thr  = 20;
    unsigned int orb_min_fast_thr  = 7;
    unsigned int orb_min_area      = 800;
    unsigned int num_grid_cols     = 64;
    unsigned int num_grid_rows     = 48;

    //-----------------------------------------
    // FrameTracker
    unsigned int frame_tracker_num_matches_thr = 10;
    float        frame_tracker_margin          = 20.0f;
    bool         use_fixed_seed                = false;

    //-----------------------------------------
    // LocalMapUpdater
    unsigned int max_num_local_keyfrms = 60;

    //-----------------------------------------
    // Local-map tracking gate (mirrors stella num_tracked_lms_thr) and the
    // projection-matcher margin for local landmarks.
    unsigned int min_inliers_to_track        = 20;
    float        local_map_projection_margin = 5.0f;
    float        projection_lowe_ratio       = 0.8f;

    //-----------------------------------------
    // MapDatabase covisibility threshold.
    unsigned int min_num_shared_lms = 15;

    //-----------------------------------------
    // LocalMapper (synchronous local mapping).
    unsigned int num_covisibilities_for_landmark_generation = 10;
    unsigned int num_covisibilities_for_landmark_fusion     = 10;
    float        triangulation_parallax_deg_thr             = 1.0f;
    float        triangulation_residual_deg_thr             = 0.2f;
    double       baseline_dist_thr_ratio                    = 0.02;

    //-----------------------------------------
    // Temporal mapping (bounded-VO sliding window). Mirrors stella_vslam
    // system::enable_temporal_mapping() + mapping_module num_temporal_keyframes
    // / erase_temporal_keyframes. When enabled, VOModule arms
    // MapDatabase::set_fixed_keyframe_id_threshold() at startup (and re-arms it
    // after a LOST reset): keyframes created past the threshold are "temporal".
    // NOTE (faithful to stella): the threshold is a one-shot snapshot of
    // next_keyframe_id_ — with an empty map at startup it is 0, so the
    // tracking-side temporal logic (local-map budget, landmark temporal-ratio
    // filter, non-temporal re-tracking pass) only fires when a map pre-exists;
    // for pure VO the mapping-side erase block (erase_temporal_keyframes) is
    // what bounds the keyframe window.
    bool         temporal_mapping_enabled = false;
    unsigned int num_temporal_keyframes   = 15;
    bool         erase_temporal_keyframes = false;
    //! Mirror of stella enable_temporal_keyframe_only_tracking: treat a failed
    //! non-temporal re-search as success (keep tracking on temporal keyframes).
    bool enable_temporal_keyframe_only_tracking = false;

    //-----------------------------------------
    // Asynchronous local mapping. When disabled (default) the mapping step runs
    // inline on the tracking thread (deterministic baseline). When enabled a
    // dedicated mapping thread consumes a bounded keyframe queue.
    bool         async_enabled          = false;
    unsigned int mapping_queue_threshold = 0;
    //! Mirror of stella keyframe_inserter.cc wait_for_local_bundle_adjustment:
    //! in async mode, after enqueuing a keyframe the tracking thread BLOCKS
    //! until the mapping thread has fully processed it (per-keyframe
    //! handshake). No effect in sync mode.
    bool wait_for_local_bundle_adjustment = false;

    //-----------------------------------------
    // Telemetry metric-scale seed after two-view initialization. When the
    // altitude/AGL prior is available, the seeded map is rescaled so the median
    // landmark depth equals (altitude / cos(off-nadir)). Disabled when the
    // median depth or altitude is non-positive.
    bool enable_metric_scale = true;

    //-----------------------------------------
    // Weld-on-reinit. After LOST -> re-initialization the new map segment is
    // anchored at the world origin internally; when this flag is enabled the
    // OUTPUT side (VOResult poses + published VOData map points) is
    // premultiplied by the accumulated segment-to-global transform so the
    // reported trajectory continues from the last valid pose instead of
    // jumping back to the origin. Internal (segment-local) state is untouched.
    // Interim measure: LOST will later be handled by VPR relocalization /
    // fusion, which will replace welding.
    bool weld_on_reinit = true;

    //! Build a VOConfig from a mission config root node (reads "Camera" and "VO").
    static VOConfig fromYaml(const YAML::Node& root);
};

} // namespace vo
} // namespace uavloc
