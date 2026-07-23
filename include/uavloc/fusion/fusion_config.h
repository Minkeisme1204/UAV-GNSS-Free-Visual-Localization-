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
    //! Scale random walk per keyframe. Scale is nearly unobservable from
    //! level flight (constant altitude, no VPR X/Y fixes), so the walk must
    //! be tight: a loose walk lets VO Z-drift alias into s(k) (observed:
    //! s slid 1.0 → 0.09 over 193 KFs at 0.01). s(k) only needs freedom once
    //! VPR X/Y or altitude maneuvers make it observable (F2).
    double scale_walk_sigma    = 0.001;
    //! Scale init prior on S(0) (and the fresh restart prior at re-init
    //! boundaries). Tight for the same reason as scale_walk_sigma: VO is
    //! metric-seeded from altitude, so s ≈ 1 is a strong measurement.
    double scale_prior_sigma   = 0.02;
    double theta_init_deg      = 46.0;   //!< mount-azimuth prior mean
    double theta_sigma_deg     = 10.0;   //!< mount-azimuth prior sigma
    double anchor_xy_sigma_m   = 100.0;  //!< weak X/Y anchor on X(0)
    //! Multiplier on vo_trans_sigma_m for the ScaledVOFactor at a post-reinit
    //! boundary keyframe: the welded pose across a LOST gap is an assumption
    //! (VOModule pose-continuity weld), not a measurement.
    double reinit_trans_inflation = 10.0;

    //! Huber robust-kernel parameter.
    double huber_k = 1.345;

    //! Build from the root of the mission YAML (reads root["Fusion"]).
    static FusionConfig fromYaml(const YAML::Node& root);
};

} // namespace uavloc::fusion
