#pragma once

// FusionConfig — runtime parameters for the GTSAM fusion back-end (Stage F1).
//
// Defaults follow the F1 factor table in
// .docs/theory/heading_agl_prior_tactics.md §7. Loaded from the mission YAML
// under the root key "Fusion:"; every field has an inline default so missing
// keys never throw.

#include <yaml-cpp/yaml.h>

namespace uavloc::fusion {

//! Robust kernel wrapped around the absolute-fix noise model (M1 ablation
//! knob). Parsed from the "fix_robust_kernel" YAML string: none|huber|tukey.
enum class FixRobustKernel {
    NONE,
    HUBER,
    TUKEY
};

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

    // ── Absolute-fix intake (anchor::AbsoluteFix, M1) ─────────────────────
    //
    // ⚠ OUTSTANDING DEBT (2026-08-02). With fix_gate_enabled = false (the
    // default, see below) and fix_min_confidence = 0.0 there is NO outlier
    // rejection left on this path at all. The M1 fake anchor always reports
    // confidence = 1.0 with a constant cov, so today nothing would stop a
    // single wrong fix from dragging the whole trajectory. The VPR producer
    // (M3–M5) MUST supply a real per-match confidence and a real covariance —
    // that is where the rejection belongs (kcb_slam filters on the producer's
    // match count / uncertainty, never on a self-consistency test). Until then,
    // treat every absolute fix as trusted by construction.

    //! Master switch for the Mahalanobis gate below. Default FALSE.
    //!
    //! The gate compares a fix against the graph's OWN estimate, so it is
    //! self-referential: once the estimate drifts, correct fixes look like
    //! outliers, get rejected, and the drift grows — a self-feeding loop.
    //! Measured on YenBai 800 m (fake fix every 20 KF, sigma 20 m, seed 42,
    //! 0 outliers, rho = 0): 43 applied / 163 gated, with a longest rejection
    //! streak of 150 consecutive fixes spanning 184.5 s. Turning the gate off
    //! is therefore the default; the code is kept (and fix_gate_chi2 with it)
    //! so the M1 configuration remains reproducible by setting this to true.
    bool fix_gate_enabled = false;
    //! Producer-side quality floor: a fix whose anchor::AbsoluteFix::confidence
    //! is below this is dropped before it reaches the graph
    //! (FusionFixStats::low_confidence). This is the kcb_slam model — reject on
    //! what the MEASUREMENT PRODUCER knows about its own match, not on whether
    //! the measurement agrees with the estimate it is supposed to correct.
    //! Default 0.0 = accept everything, because the M1 fake anchor reports a
    //! constant confidence = 1.0 and a threshold would be meaningless; it
    //! becomes the real filter once VPR fills the field (kcb precedent:
    //! min_keyframe_confidence = 0.35).
    double fix_min_confidence = 0.0;
    //! Mahalanobis gate on the fix residual, evaluated with the COMBINED
    //! covariance S = cov_fix + cov_pred(X(k)): chi-square with 2 dof at 99 %.
    //! Only consulted when fix_gate_enabled is true.
    //! The combined form is essential — once VO has drifted 200 m, a CORRECT
    //! fix with sigma = 20 m sits at 10 sigma, so a gate built on cov_fix
    //! alone would reject exactly the measurement the estimator needs most.
    //! cov_pred grows with the drift, so a true fix stays admissible while a
    //! 1 km outlier is still rejected.
    double fix_gate_chi2 = 9.21;
    //! Robust kernel wrapped around the fix noise model. When the gate is
    //! enabled it runs FIRST, so this only shapes the influence of
    //! measurements that already passed; "none" isolates the kernel from the
    //! gate for ablation.
    //!
    //! Default HUBER, changed from TUKEY on 2026-08-02 [đo]. Tukey redescends
    //! to exactly zero weight for large residuals — and after a long stretch of
    //! drift a CORRECT fix also carries a large residual, so Tukey silently
    //! discards the measurement that was supposed to end the drift. Huber keeps
    //! a linear (down-weighted but non-zero) influence instead. Measured, same
    //! run as fix_gate_enabled above: tukey 43 applied / 163 gated vs huber
    //! 193 / 15; on MUN-FRL ds6 (full flight, rho = 0) the error slope went
    //! +33.13 m/km (tukey) → +4.30 m/km (huber).
    FixRobustKernel fix_robust_kernel = FixRobustKernel::HUBER;
    //! Tukey kernel parameter (kcb precedent, .docs/theory/05_khoi_fusion.md).
    double fix_tukey_c = 4.685;
    //! Largest |t_fix − t_keyframe| accepted when a fix is matched to a live
    //! keyframe state. This is a SYNCHRONISATION tolerance — "is this the
    //! keyframe the fix was measured on?" — so it must be of the order of the
    //! keyframe interval, NOT of the lag window. (kcb_slam looks the state up
    //! by exact timestamp; 0.15 s ≈ half a keyframe interval at 3 KF/s is the
    //! nearest-neighbour equivalent.)
    double fix_match_tolerance_sec = 0.15;
    //! Safety margin subtracted from lag_seconds to obtain the AGE budget of a
    //! fix: a fix is dropped when (t_keyframe − t_fix) exceeds
    //! (lag_seconds − fix_age_margin_sec). The budget is the whole lag window
    //! minus this margin because that window is exactly how far back the
    //! smoother can still correct — the kcb_slam rule
    //! (unsafe_time_threshold = last_stamp − lag_window + 0.5).
    //!
    //! ⚠ The retired key `fix_max_age_sec` (default 5.0) used to serve BOTH
    //! roles at once; it was published in .docs/reports/m1_fake_anchor.md, so
    //! it is rejected loudly (a warning) rather than silently reinterpreted.
    double fix_age_margin_sec = 0.5;
    //! Drift-rate inflation of the gate covariance:
    //!   S = cov_fix + cov_pred + (fix_drift_rate_m_per_m · s)² · I
    //! with s = distance flown since the last APPLIED fix [m]. It models the
    //! part of the VO error the graph does NOT model: the graph treats VO
    //! error as independent noise (growing as √n) whereas the real drift is
    //! systematic (growing as n), which made the gate over-confident by ~118×
    //! on YenBai (.docs/reports/m1_fake_anchor.md §6.2).
    //!
    //! rho is a MEASURED quantity — read it off the RPE p95 @1000 m row of
    //! .docs/reports/Phase2_roadmap.md §2.5 for the dataset at hand
    //! (ds6 ≈ 0.15, ds3 ≈ 0.18, YenBai 800 m ≈ 0.68 m per m flown). It is
    //! therefore a mission-config value and never hard-coded in the source.
    //!
    //! Default 0.0 = OFF: the term vanishes and every pre-existing result
    //! stays bit-for-bit reproducible. Turning it on must be explicit.
    double fix_drift_rate_m_per_m = 0.0;

    //! Huber robust-kernel parameter.
    double huber_k = 1.345;

    // ── Health state machine (S7) ─────────────────────────────────────────
    // Thresholds on the horizontal 1-sigma radius of the marginal covariance
    // of X(k) (fusion::horizontal_accuracy_m). They must satisfy
    // health_converged_sigma_m <= health_drifting_sigma_m; the band between
    // them is the hysteresis dead zone (see fusion::next_fusion_health).
    //! Below this the estimate is reported CONVERGED. Default 25 m: with the
    //! default anchor_xy_sigma_m = 100 the graph starts far above it, and 25 m
    //! is the order of the fix sigma the M1 absolute-fix work uses (20 m), so
    //! CONVERGED means "no worse than one absolute fix".
    double health_converged_sigma_m = 25.0;
    //! Above this a CONVERGED estimate is reported DRIFTING. Default 50 m =
    //! 2x the converged threshold, i.e. the model-uncertainty has doubled.
    //! ⚠ Both thresholds compare against the estimator's SELF-REPORTED sigma,
    //! which is known to be optimistic (m1_fake_anchor.md §6.2) — CONVERGED
    //! means "the graph is well constrained", not "the position is accurate".
    double health_drifting_sigma_m = 50.0;

    //! Build from the root of the mission YAML (reads root["Fusion"]).
    static FusionConfig fromYaml(const YAML::Node& root);
};

} // namespace uavloc::fusion
