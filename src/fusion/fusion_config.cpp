#include "uavloc/fusion/fusion_config.h"

namespace uavloc::fusion {

FusionConfig FusionConfig::fromYaml(const YAML::Node& root) {
    FusionConfig cfg;

    const YAML::Node node = root["Fusion"] ? root["Fusion"] : YAML::Node();

    cfg.async_enabled = node["async_enabled"].as<bool>(cfg.async_enabled);
    cfg.lag_seconds   = node["lag_seconds"].as<double>(cfg.lag_seconds);

    cfg.vo_rot_sigma_deg    = node["vo_rot_sigma_deg"].as<double>(cfg.vo_rot_sigma_deg);
    cfg.vo_trans_sigma_m    = node["vo_trans_sigma_m"].as<double>(cfg.vo_trans_sigma_m);
    cfg.delta_yaw_sigma_rad = node["delta_yaw_sigma_rad"].as<double>(cfg.delta_yaw_sigma_rad);
    cfg.delta_yaw_max_step_deg =
        node["delta_yaw_max_step_deg"].as<double>(cfg.delta_yaw_max_step_deg);
    cfg.rollpitch_sigma_rad = node["rollpitch_sigma_rad"].as<double>(cfg.rollpitch_sigma_rad);
    cfg.agl_sigma_m         = node["agl_sigma_m"].as<double>(cfg.agl_sigma_m);
    cfg.agl_bias_walk_m     = node["agl_bias_walk_m"].as<double>(cfg.agl_bias_walk_m);
    cfg.agl_bias_prior_sigma_m =
        node["agl_bias_prior_sigma_m"].as<double>(cfg.agl_bias_prior_sigma_m);
    cfg.scale_walk_sigma    = node["scale_walk_sigma"].as<double>(cfg.scale_walk_sigma);
    cfg.scale_prior_sigma   = node["scale_prior_sigma"].as<double>(cfg.scale_prior_sigma);
    // anchor_yaw_sigma_deg — the X(0) anchor yaw sigma. Backward compatible:
    // the removed mount-azimuth θ state reused theta_sigma_deg for this same
    // sigma, so if the new key is absent fall back to the old one (existing
    // mission configs carrying only theta_sigma_deg keep parsing unchanged).
    cfg.anchor_yaw_sigma_deg =
        node["anchor_yaw_sigma_deg"]
            ? node["anchor_yaw_sigma_deg"].as<double>(cfg.anchor_yaw_sigma_deg)
            : node["theta_sigma_deg"].as<double>(cfg.anchor_yaw_sigma_deg);
    cfg.anchor_xy_sigma_m   = node["anchor_xy_sigma_m"].as<double>(cfg.anchor_xy_sigma_m);
    cfg.reinit_trans_inflation =
        node["reinit_trans_inflation"].as<double>(cfg.reinit_trans_inflation);

    cfg.map_depth_enabled   = node["map_depth_enabled"].as<bool>(cfg.map_depth_enabled);
    cfg.map_depth_sigma     = node["map_depth_sigma"].as<double>(cfg.map_depth_sigma);
    cfg.map_depth_min_lms   = node["map_depth_min_lms"].as<int>(cfg.map_depth_min_lms);
    cfg.map_depth_min_agl_m = node["map_depth_min_agl_m"].as<double>(cfg.map_depth_min_agl_m);

    cfg.huber_k = node["huber_k"].as<double>(cfg.huber_k);

    return cfg;
}

} // namespace uavloc::fusion
