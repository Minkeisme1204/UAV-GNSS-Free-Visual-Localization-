#pragma once

// FusionConfig — runtime parameters for the GTSAM fusion back-end (Stage F1).
//
// Defaults follow the F1 factor table in
// .docs/theory/heading_agl_prior_tactics.md §7. Loaded from the mission YAML
// under the root key "Fusion:"; every field has an inline default so missing
// keys never throw.

#include <yaml-cpp/yaml.h>

namespace uavloc::fusion {

struct FusionConfig {
    //! Run the smoother on its own thread (push() enqueues, non-blocking).
    //! false = inline deterministic processing on the caller's thread.
    bool async_enabled = true;

    //! Fixed-lag smoother window [s].
    double lag_seconds = 20.0;

    // ── Noise sigmas (F1 factor table) ────────────────────────────────────
    double vo_rot_sigma_deg    = 0.3;    //!< VO relative-pose rotation
    double vo_trans_sigma_m    = 1.0;    //!< VO relative-pose translation
    double delta_yaw_sigma_rad = 0.025;  //!< delta-yaw factor
    //! Slew-rate outlier gate: skip the delta-yaw factor when the measured
    //! per-keyframe heading step exceeds this (datalink glitch guard).
    double delta_yaw_max_step_deg = 45.0;
    double rollpitch_sigma_rad = 0.017;  //!< telemetry roll/pitch prior
    double agl_sigma_m         = 7.0;    //!< AGL Z measurement
    double agl_bias_walk_m     = 0.5;    //!< AGL bias random walk per keyframe
    double agl_bias_prior_sigma_m = 5.0; //!< AGL bias prior on b(0)
    double scale_walk_sigma    = 0.01;   //!< scale random walk per keyframe
    double scale_prior_sigma   = 0.2;    //!< scale init prior on S(0)
    double theta_init_deg      = 46.0;   //!< mount-azimuth prior mean
    double theta_sigma_deg     = 10.0;   //!< mount-azimuth prior sigma
    double anchor_xy_sigma_m   = 100.0;  //!< weak X/Y anchor on X(0)

    //! Huber robust-kernel parameter.
    double huber_k = 1.345;

    //! Build from the root of the mission YAML (reads root["Fusion"]).
    static FusionConfig fromYaml(const YAML::Node& root);
};

} // namespace uavloc::fusion
