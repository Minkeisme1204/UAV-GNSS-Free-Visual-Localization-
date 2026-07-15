#include "uavloc/new_vo/vo_config.h"

namespace uavloc {
namespace vo {

VOConfig VOConfig::fromYaml(const YAML::Node& root) {
    VOConfig cfg;

    // Camera intrinsics (reads root["Camera"] internally).
    cfg.camera_intrinsics = sensor::CameraIntrinsics::fromYaml(root);

    // The whole "VO" sub-tree (may be empty; components default their own keys).
    cfg.vo_node = root["VO"] ? root["VO"] : YAML::Node();
    const YAML::Node& vo = cfg.vo_node;

    // ORB feature extraction.
    cfg.orb_name         = vo["orb_name"].as<std::string>("orb");
    cfg.orb_scale_factor = vo["scale_factor"].as<float>(1.2f);
    cfg.orb_num_levels   = vo["scale_levels"].as<unsigned int>(8);
    cfg.orb_ini_fast_thr = vo["ini_fast_threshold"].as<unsigned int>(20);
    cfg.orb_min_fast_thr = vo["min_fast_threshold"].as<unsigned int>(7);
    cfg.orb_min_area     = vo["orb_min_area"].as<unsigned int>(800);
    cfg.num_grid_cols    = vo["num_grid_cols"].as<unsigned int>(64);
    cfg.num_grid_rows    = vo["num_grid_rows"].as<unsigned int>(48);

    // FrameTracker.
    cfg.frame_tracker_num_matches_thr = vo["frame_tracker_num_matches_thr"].as<unsigned int>(20);
    cfg.frame_tracker_margin          = vo["search_radius"].as<float>(20.0f);
    cfg.use_fixed_seed                = vo["use_fixed_seed"].as<bool>(false);

    // LocalMapUpdater.
    cfg.max_num_local_keyfrms = vo["max_num_local_keyfrms"].as<unsigned int>(60);

    // Local-map tracking gate.
    cfg.min_inliers_to_track        = vo["min_inliers_to_track"].as<unsigned int>(20);
    cfg.local_map_projection_margin = vo["local_map_projection_margin"].as<float>(5.0f);
    cfg.projection_lowe_ratio       = vo["lowe_ratio_threshold"].as<float>(0.8f);

    // MapDatabase covisibility threshold.
    cfg.min_num_shared_lms = vo["min_num_shared_lms"].as<unsigned int>(15);

    // LocalMapper.
    cfg.num_covisibilities_for_landmark_generation =
        vo["num_covisibilities_for_landmark_generation"].as<unsigned int>(10);
    cfg.num_covisibilities_for_landmark_fusion =
        vo["num_covisibilities_for_landmark_fusion"].as<unsigned int>(10);
    cfg.triangulation_parallax_deg_thr = vo["triangulation_parallax_deg_thr"].as<float>(1.0f);
    cfg.triangulation_residual_deg_thr = vo["residual_deg_thr"].as<float>(0.2f);
    cfg.baseline_dist_thr_ratio        = vo["baseline_dist_thr_ratio"].as<double>(0.02);

    // Temporal mapping (bounded-VO sliding window; stella temporal mapping).
    cfg.temporal_mapping_enabled = vo["temporal_mapping_enabled"].as<bool>(false);
    cfg.num_temporal_keyframes   = vo["num_temporal_keyframes"].as<unsigned int>(15);
    cfg.erase_temporal_keyframes = vo["erase_temporal_keyframes"].as<bool>(false);
    cfg.enable_temporal_keyframe_only_tracking =
        vo["enable_temporal_keyframe_only_tracking"].as<bool>(false);

    // Asynchronous local mapping.
    cfg.async_enabled           = vo["async_enabled"].as<bool>(false);
    cfg.mapping_queue_threshold = vo["mapping_queue_threshold"].as<unsigned int>(0);
    cfg.wait_for_local_bundle_adjustment =
        vo["wait_for_local_bundle_adjustment"].as<bool>(false);

    // Telemetry metric-scale seed.
    cfg.enable_metric_scale = vo["enable_metric_scale"].as<bool>(true);

    // Weld-on-reinit (output-side pose/map continuity after LOST -> re-init).
    cfg.weld_on_reinit = vo["weld_on_reinit"].as<bool>(true);

    return cfg;
}

} // namespace vo
} // namespace uavloc
