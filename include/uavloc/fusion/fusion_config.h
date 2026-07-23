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

    //! Fixed-lag smoother window [s]. 40 s (was 20): the HoaLac sharp-turn
    //! episode showed a ~13 s VO-vs-compass disagreement; with a 20 s lag,
    //! early-turn keyframes were marginalized (frozen) mid-conflict, leaving
    //! permanent kinks — 40 s lets such an episode fully resolve before any
    //! pose freezes.
    double lag_seconds = 40.0;

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
    //! Yaw-component sigma of the X(0) anchor PriorFactor<Pose3> (Pose3 tangent
    //! index 2 — the near-vertical camera-z rotation for a near-nadir camera).
    //! Loaded from the "anchor_yaw_sigma_deg" YAML key, with a backward-
    //! compatible fallback to the deprecated "theta_sigma_deg" key (which used
    //! to double as the mount-azimuth θ prior sigma before θ was removed).
    double anchor_yaw_sigma_deg = 10.0;
    double anchor_xy_sigma_m   = 100.0;  //!< weak X/Y anchor on X(0)
    //! Multiplier on vo_trans_sigma_m for the ScaledVOFactor at a post-reinit
    //! boundary keyframe: the welded pose across a LOST gap is an assumption
    //! (VOModule pose-continuity weld), not a measurement.
    double reinit_trans_inflation = 10.0;

    // ── Map-depth scale measurement (MapDepthFactor) ──────────────────────
    //! Master switch for the map-depth unary on s(k). OFF by default so every
    //! pre-existing config and result stays bit-for-bit reproducible.
    bool   map_depth_enabled = false;
    //! Log-relative sigma of s*d_vo*cos(beta) vs AGL. 0.085 is measured: over
    //! 265 keyframes q = d_vo/AGL tracks the true map-scale drift with
    //! corr = 0.915, and the residual scatter is 8.5 % RMS. That residual is
    //! structured, not white (smoothing over 1→80 keyframes leaves it at
    //! 0.085→0.087), so no filtering can beat this value.
    double map_depth_sigma = 0.085;
    //! Reject a median taken over too few landmarks (noisy median).
    int    map_depth_min_lms = 200;
    //! Reject near-ground samples: below a few metres AGL the ratio d_vo/AGL
    //! explodes (measured 2.25 and 7.9 on such segments).
    double map_depth_min_agl_m = 5.0;

    //! Huber robust-kernel parameter.
    double huber_k = 1.345;

    //! Build from the root of the mission YAML (reads root["Fusion"]).
    static FusionConfig fromYaml(const YAML::Node& root);
};

} // namespace uavloc::fusion
