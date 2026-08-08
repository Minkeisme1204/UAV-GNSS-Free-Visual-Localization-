#include "uavloc/fusion/fusion_config.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <string>

namespace uavloc::fusion {

namespace {

//! Parse the "fix_robust_kernel" string (case-insensitive). An unrecognised
//! value keeps the default and warns rather than throwing — same
//! never-throw-on-config contract as the .as<T>(default) reads below.
FixRobustKernel parse_fix_robust_kernel(const std::string& name,
                                        FixRobustKernel fallback) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower == "none")  return FixRobustKernel::NONE;
    if (lower == "huber") return FixRobustKernel::HUBER;
    if (lower == "tukey") return FixRobustKernel::TUKEY;
    spdlog::warn("FusionConfig: unknown fix_robust_kernel '{}' — "
                 "expected none|huber|tukey; keeping the default", name);
    return fallback;
}

} // namespace

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

    cfg.fix_gate_enabled  = node["fix_gate_enabled"].as<bool>(cfg.fix_gate_enabled);
    cfg.fix_min_confidence =
        node["fix_min_confidence"].as<double>(cfg.fix_min_confidence);
    cfg.fix_gate_chi2     = node["fix_gate_chi2"].as<double>(cfg.fix_gate_chi2);
    cfg.fix_robust_kernel = parse_fix_robust_kernel(
        node["fix_robust_kernel"].as<std::string>(std::string("huber")),
        cfg.fix_robust_kernel);
    cfg.fix_tukey_c       = node["fix_tukey_c"].as<double>(cfg.fix_tukey_c);
    cfg.fix_match_tolerance_sec =
        node["fix_match_tolerance_sec"].as<double>(cfg.fix_match_tolerance_sec);
    cfg.fix_age_margin_sec =
        node["fix_age_margin_sec"].as<double>(cfg.fix_age_margin_sec);
    cfg.fix_drift_rate_m_per_m =
        node["fix_drift_rate_m_per_m"].as<double>(cfg.fix_drift_rate_m_per_m);
    // Retired key. It used to mean two unrelated things at once (fix age budget
    // AND keyframe-match tolerance) and its value was published in
    // .docs/reports/m1_fake_anchor.md, so a config still carrying it must be
    // told that its number is now ignored — silently changing the meaning of a
    // published key would invalidate every result reported against it.
    if (node["fix_max_age_sec"]) {
        spdlog::warn("FusionConfig: 'fix_max_age_sec' is RETIRED and IGNORED "
                     "(value in config: {}). It has been split into "
                     "'fix_match_tolerance_sec' (keyframe-match tolerance, now "
                     "{} s) and 'fix_age_margin_sec' (age budget = lag_seconds "
                     "- margin = {} s). Remove the key from the config.",
                     node["fix_max_age_sec"].as<double>(0.0),
                     cfg.fix_match_tolerance_sec,
                     cfg.lag_seconds - cfg.fix_age_margin_sec);
    }
    // Guard rails: neither knob may silently produce a nonsensical gate. Same
    // never-throw-on-config contract as everywhere else — warn and clamp.
    if (cfg.fix_match_tolerance_sec <= 0.0) {
        spdlog::warn("FusionConfig: fix_match_tolerance_sec ({}) must be > 0 — "
                     "no fix could ever match a keyframe; keeping the default {}",
                     cfg.fix_match_tolerance_sec, FusionConfig{}.fix_match_tolerance_sec);
        cfg.fix_match_tolerance_sec = FusionConfig{}.fix_match_tolerance_sec;
    }
    if (cfg.fix_age_margin_sec >= cfg.lag_seconds) {
        spdlog::warn("FusionConfig: fix_age_margin_sec ({}) >= lag_seconds ({}) "
                     "— the fix age budget would be zero or negative and EVERY "
                     "fix would expire; keeping the default {}",
                     cfg.fix_age_margin_sec, cfg.lag_seconds,
                     FusionConfig{}.fix_age_margin_sec);
        cfg.fix_age_margin_sec = FusionConfig{}.fix_age_margin_sec;
    }
    // anchor::AbsoluteFix::confidence is defined on [0, 1]; a threshold outside
    // that range is a config mistake with a silent, total effect (> 1 rejects
    // EVERY fix, < 0 is a no-op written as if it did something).
    if (cfg.fix_min_confidence < 0.0 || cfg.fix_min_confidence > 1.0) {
        spdlog::warn("FusionConfig: fix_min_confidence ({}) is outside [0, 1] — "
                     "AbsoluteFix::confidence lives on that interval, so this "
                     "would either reject every fix or do nothing; keeping the "
                     "default {}",
                     cfg.fix_min_confidence, FusionConfig{}.fix_min_confidence);
        cfg.fix_min_confidence = FusionConfig{}.fix_min_confidence;
    }
    if (cfg.fix_drift_rate_m_per_m < 0.0) {
        spdlog::warn("FusionConfig: fix_drift_rate_m_per_m ({}) is negative — "
                     "the drift term is a variance inflation and cannot shrink "
                     "the gate; disabling it (0.0)",
                     cfg.fix_drift_rate_m_per_m);
        cfg.fix_drift_rate_m_per_m = 0.0;
    }

    cfg.huber_k = node["huber_k"].as<double>(cfg.huber_k);

    cfg.health_converged_sigma_m =
        node["health_converged_sigma_m"].as<double>(cfg.health_converged_sigma_m);
    cfg.health_drifting_sigma_m =
        node["health_drifting_sigma_m"].as<double>(cfg.health_drifting_sigma_m);
    // An inverted pair would turn the hysteresis band into a region where both
    // transitions fire; collapse it to a single threshold instead (degenerate
    // but still monotone) and say so, rather than throwing on config.
    if (cfg.health_drifting_sigma_m < cfg.health_converged_sigma_m) {
        spdlog::warn("FusionConfig: health_drifting_sigma_m ({}) < "
                     "health_converged_sigma_m ({}) — clamping the drifting "
                     "threshold up; hysteresis is disabled",
                     cfg.health_drifting_sigma_m, cfg.health_converged_sigma_m);
        cfg.health_drifting_sigma_m = cfg.health_converged_sigma_m;
    }

    return cfg;
}

} // namespace uavloc::fusion
