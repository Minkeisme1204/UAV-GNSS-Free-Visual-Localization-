#pragma once

// Fusion module public data types (Stage F1).
//
// FusionResult is the per-frame output of the GTSAM fixed-lag smoother
// back-end: the fused camera pose in ENU plus the auxiliary states the F1
// graph estimates (scale S(k), AGL bias b(k)). See
// .docs/theory/heading_agl_prior_tactics.md §7 for the factor table.
//
// This header is intentionally gtsam-free: only Eigen/std types cross the
// public API boundary.

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uavloc::fusion {

//! Health of the fusion estimate. Driven by the horizontal 1-sigma radius of
//! the marginal covariance of X(k) (S7), with hysteresis between the two
//! FusionConfig thresholds — see next_fusion_health().
enum class FusionHealth {
    //! No usable marginal covariance yet (graph not anchored, or the estimate
    //! is still wider than health_converged_sigma_m).
    INITIALIZING,
    //! Self-reported horizontal uncertainty is inside the converged band.
    CONVERGED,
    //! Self-reported horizontal uncertainty grew past health_drifting_sigma_m.
    DRIFTING
};

//! Per-frame fused output.
struct FusionResult {
    unsigned int frame_id       = 0;
    double       timestamp_msec = 0.0;

    //! Fused camera pose in the ENU frame (camera → ENU).
    Eigen::Isometry3d T_enu_c = Eigen::Isometry3d::Identity();

    double scale      = 1.0;   //!< current metric-scale estimate S(k)
    double agl_bias_m = 0.0;   //!< AGL-vs-ENU-Z bias b(k)

    FusionHealth health   = FusionHealth::INITIALIZING;
    bool         has_pose = false;

    //! Which optimized keyframe state X(k) this pose is propagated from.
    unsigned int base_keyframe_id = 0;
    //! True right after a smoother update (i.e. this frame was a keyframe).
    bool graph_updated = false;

    //! HORIZONTAL position covariance in ENU [m²] — the East/North block of
    //! marginalCovariance(X(k_base)) after rotating the Pose3 translation
    //! block out of the local (camera) frame into ENU (Pose3::retract
    //! translates by R·v, so the raw block is NOT in ENU).
    //!
    //! ⚠ This is the covariance of the BASE keyframe X(base_keyframe_id), not
    //! of this frame: on a non-keyframe the pose is dead-reckoned from that
    //! state and the extra uncertainty of the propagation is NOT added.
    //!
    //! ⚠ MODEL uncertainty, NOT a calibrated confidence radius — see the
    //! warning on horizontal_accuracy_m().
    Eigen::Matrix2d xy_covariance = Eigen::Matrix2d::Zero();
    //! false = no covariance could be computed (graph not anchored yet, or
    //! marginalCovariance() threw). xy_covariance is then Zero and must NOT be
    //! interpreted as "perfectly certain".
    bool covariance_valid = false;
};

//! Horizontal 1-sigma accuracy radius [m] of `result`, defined as the LARGEST
//! semi-axis of the 1-sigma error ellipse: sqrt(max eigenvalue of
//! xy_covariance). The largest axis (not sqrt(trace/2), which is an RMS over
//! the two axes) is used so an elongated ellipse — the normal shape here,
//! since along-track and cross-track uncertainty differ — is never reported
//! smaller than its worst direction.
//!
//! Returns NaN when result.covariance_valid is false. It deliberately never
//! returns 0: 0 means "perfectly certain", which is the most dangerous value
//! to hand an autopilot.
//!
//! ⚠⚠ CALIBRATION WARNING — this number is the estimator's uncertainty UNDER
//! ITS OWN MODEL, not a validated confidence radius. Measured on YenBai
//! (.docs/reports/m1_fake_anchor.md §6.2): the graph reported ~17 m horizontal
//! sigma while the true error was ~2 000 m — over-confident by ~118×. Cause:
//! the graph models VO error as independent noise (accumulating as √n) whereas
//! the real drift is systematic (accumulating as n). Until a drift term enters
//! the model, treat this value as a relative indicator (it grows when the
//! estimate weakens) and NOT as an absolute metre-level bound.
double horizontal_accuracy_m(const Eigen::Matrix2d& xy_covariance,
                             bool                   covariance_valid);

//! Convenience overload: horizontal_accuracy_m(r.xy_covariance,
//! r.covariance_valid). Same definition and the same warnings as above.
double horizontal_accuracy_m(const FusionResult& result);

//! Health state machine (kcb OptimizationStatus pattern), driven by the
//! horizontal 1-sigma radius `sigma_xy_m`:
//!
//!   any state  → INITIALIZING  when covariance_valid == false
//!   INITIALIZING → CONVERGED   when sigma_xy_m <  converged_sigma_m
//!   CONVERGED    → DRIFTING    when sigma_xy_m >  drifting_sigma_m
//!   DRIFTING     → CONVERGED   when sigma_xy_m <  converged_sigma_m
//!
//! Note the DRIFTING → CONVERGED edge uses the CONVERGED threshold, not the
//! DRIFTING one (kcb uses the latter): with converged_sigma_m <
//! drifting_sigma_m the band between the two thresholds is a hysteresis dead
//! zone, so a sigma hovering there cannot make the state chatter.
//!
//! Pure function of its arguments — no hidden state — so the transition table
//! is unit-testable without a factor graph.
FusionHealth next_fusion_health(FusionHealth current,
                                double       sigma_xy_m,
                                bool         covariance_valid,
                                double       converged_sigma_m,
                                double       drifting_sigma_m);

//! Cumulative accounting of absolute-fix handling (M1). Counters increase
//! monotonically over the module's lifetime; a fix handed to
//! FusionModule::push_absolute_fix() ends up in exactly one terminal counter
//! (or is still in flight inside the fix queue), so
//!   injected >= applied + gated + low_confidence + age_expired + unmatched
//!               + marginalized + queue_dropped + no_graph.
//!
//! The drop reasons are counted SEPARATELY on purpose. M1 lumped three of them
//! into one "too_late" counter and the report never noticed that 84 % of the
//! fixes were being thrown away (.docs/reports/m1_fake_anchor.md §6.4): a
//! rejected measurement is only diagnosable if the reason survives.
struct FusionFixStats {
    //! Fixes accepted by push_absolute_fix() (valid flag + finite SPD cov).
    unsigned long long injected = 0;
    //! Passed the Mahalanobis gate and entered the factor graph.
    unsigned long long applied  = 0;
    //! Rejected by the Mahalanobis gate (residual too large for S). Stays 0
    //! while FusionConfig::fix_gate_enabled is false — which is the default.
    unsigned long long gated    = 0;
    //! Rejected by the producer-side quality floor: the fix reported a
    //! confidence below FusionConfig::fix_min_confidence. This is the counter
    //! that will do the work once VPR supplies real confidences; with the M1
    //! fake anchor (constant confidence 1.0) it stays 0.
    unsigned long long low_confidence = 0;
    //! Older than the age budget (lag_seconds − fix_age_margin_sec) measured
    //! against the keyframe consuming it — outside what the smoother can still
    //! correct.
    unsigned long long age_expired = 0;
    //! No live keyframe state within fix_match_tolerance_sec of the fix
    //! timestamp: the fix cannot be attributed to a pose the graph holds.
    unsigned long long unmatched = 0;
    //! The matched keyframe state has already been marginalized out of the
    //! fixed-lag window (calculateEstimate() no longer knows the key).
    unsigned long long marginalized = 0;
    //! Dropped from the intake queue by drop-oldest (the consumer could not
    //! keep up); never happens at the cadence a VPR produces.
    unsigned long long queue_dropped = 0;
    //! Arrived before the graph was anchored (no X(k) to attach to).
    unsigned long long no_graph = 0;

    // ── Gate observability (not counters — instantaneous values) ───────────
    //! Distance flown [m] since the last APPLIED fix (before any fix: since
    //! graph initialization), accumulated over the optimized keyframe
    //! positions. This is the `s` of the drift term
    //! FusionConfig::fix_drift_rate_m_per_m.
    double distance_since_fix_m = 0.0;
    //! Horizontal residual [m] of the most recently GATED fix, and the gate
    //! radius [m] along that residual direction
    //! (|r|·sqrt(fix_gate_chi2 / d²)). Both are 0 until a fix is gated — and
    //! stay 0 for the whole run when the gate is disabled. They
    //! answer "how far off was it, and how wide was the gate" — the pair the
    //! M1 post-mortem had to reconstruct by hand.
    double last_gated_residual_m    = 0.0;
    double last_gated_gate_radius_m = 0.0;
};

//! One keyframe pose still alive inside the fixed-lag smoother window, as
//! re-optimized by the latest smoother update. FusionModule::getLagWindow()
//! returns these in ascending frame_id so a viewer can redraw the recent past
//! of the fused trajectory with the corrected (smoothed) poses.
struct FusionLagPose {
    unsigned int frame_id = 0;
    Eigen::Isometry3d T_enu_c = Eigen::Isometry3d::Identity();
};

} // namespace uavloc::fusion
