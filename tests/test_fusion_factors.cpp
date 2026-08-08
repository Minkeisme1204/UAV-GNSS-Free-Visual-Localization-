// test_fusion_factors — F1 drift-recovery test for the fusion factor set
// (ScaledVOFactor, DeltaYawFactor, AglFactor, scale/bias walks, θ prior).
//
// Fully synthetic, sync mode (deterministic), no data files:
//   GT: constant AGL 800 m, 20 m/s eastward for 30 KFs (heading 90°), then a
//   90° left turn and 30 KFs northward (heading 0°); KF every 1 s; camera
//   nadir (tilt = 90°), pan = 0, roll = pitch = 0. The turn makes yaw errors
//   visible in position.
//
//   VO simulation in the VO WORLD frame consistent with the module's ENU
//   chain: T_wc(k) = T_enu_w⁻¹ · T_enu_c_GT(k), then the relative increments
//   are corrupted with (a) +0.1°/KF extra yaw about the ENU vertical,
//   (b) +2 m/KF drift along the ENU vertical (mapped into the camera frame),
//   (c) translation scale ×1.05 — and re-chained.
//
// Checks:
//   0. attitude-chain conventions on known synthetic cases;
//   1. perfect (uncorrupted) VO reproduces the GT ENU trajectory;
//   2. raw corrupted VO endpoint error ≈ 6° yaw / ≈ +126 m Z (corruption sanity);
//   3. fused endpoint: |yaw err| < 1.5°, |Z err| < 3·agl_sigma, horizontal
//      error < half the raw VO horizontal error;
//   4. a constant +46° offset on ALL heading measurements only rotates the
//      output rigidly by the (known) anchor-yaw gauge — after removing that
//      rotation the two fused trajectories agree to < 0.5 m RMS (delta-yaw
//      offset immunity);
//   5. two identical sync runs are bit-identical (determinism);
//   6. the absolute-fix Mahalanobis gate (M1), incl. the residual recorded on
//      a rejection; 6b the age budget and the keyframe-match tolerance as two
//      independent thresholds with one counter each; 6c the drift inflation
//      (rho·s)² of the gate covariance and the bookkeeping of s;
//   7. (S7) every fused result carries the ENU horizontal marginal covariance
//      of its keyframe state, horizontal_accuracy_m() is exactly the largest
//      1-sigma semi-axis, and an unavailable/degenerate covariance reports NaN
//      — never 0, which would mean "perfectly certain";
//   8. (S7) the health state machine moves both ways on synthetic sigmas, has
//      hysteresis between the two thresholds, and falls back to INITIALIZING
//      whenever the covariance is unusable.
//
// Headless; exits non-zero on failure.

#include "fusion/factors.h"  // private src/fusion header: attitude chain + yaw helper

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "uavloc/anchor/absolute_fix.h"
#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/fusion/fusion_module.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;

// ── Scenario ──────────────────────────────────────────────────────────────
constexpr int    LEG_KFS          = 30;             // KFs per leg
constexpr int    NUM_KFS          = 2 * LEG_KFS + 1; // k = 0 … 60
constexpr double KF_DT_MSEC       = 1000.0;         // KF every 1 s
constexpr double SPEED_MPS        = 20.0;           // → 20 m per KF
constexpr double AGL_M            = 800.0;
constexpr double HEADING_EAST_DEG  = 90.0;
constexpr double HEADING_NORTH_DEG = 0.0;
constexpr double TILT_NADIR_DEG    = 90.0;

// ── Injected VO corruption ────────────────────────────────────────────────
constexpr double YAW_DRIFT_DEG_PER_KF = 0.1;
constexpr double Z_DRIFT_M_PER_KF     = 2.0;
constexpr double SCALE_CORRUPTION     = 1.05;

// Constant heading offset for the offset-immunity leg (the empirical mount
// azimuth magnitude — any constant works).
constexpr double HEADING_OFFSET_DEG = 46.0;

// ── Assertion tolerances ──────────────────────────────────────────────────
constexpr double CHAIN_TOL             = 1e-9;  // exact-math convention checks
constexpr double PERFECT_POS_TOL_M     = 1.0;   // perfect-VO leg vs GT
constexpr double PERFECT_ROT_TOL_DEG   = 0.5;
constexpr double RAW_YAW_ERR_MIN_DEG   = 5.0;   // corruption sanity band
constexpr double RAW_YAW_ERR_MAX_DEG   = 7.0;
constexpr double RAW_Z_ERR_MIN_M       = 100.0;
constexpr double RAW_Z_ERR_MAX_M       = 150.0;
constexpr double FUSED_YAW_ERR_MAX_DEG = 1.5;
constexpr double FUSED_Z_ERR_SIGMAS    = 3.0;   // < 3·agl_sigma_m
constexpr double FUSED_HORIZ_FRACTION  = 0.5;   // < half the raw error
constexpr double OFFSET_RMS_TOL_M      = 0.5;
constexpr double JACOBIAN_TOL          = 1e-5;  // numerical-derivative grade

// MapDepthFactor unit check: the measured end-of-segment map shrinkage
// (median map depth = 0.549 × AGL ⇒ true scale ≈ 1.82).
constexpr double MAP_DEPTH_RATIO = 0.549;
constexpr double MAP_DEPTH_SIGMA = 0.085;

// ── AbsoluteXYFactor / absolute-fix intake (M1) ───────────────────────────
constexpr double FIX_SIGMA_M          = 20.0;    // FoundLoc-grade VPR sigma
constexpr double FIX_TEST_OFFSET_M    = 100.0;   // injected residual (check 0c)
constexpr double FIX_OUTLIER_OFFSET_M = 1000.0;  // gross outlier
//! Inlier offset used by the gate test: 1 sigma. NOTE the 2-dof chi-square at
//! 99 % is 9.21, i.e. a gate radius of 3.03 sigma — a "3 sigma" probe would sit
//! ON the threshold, so the accept case is probed well inside it instead.
constexpr double FIX_INLIER_OFFSET_M  = 20.0;
constexpr double FIX_ROBUST_RESIDUAL_M = 10.0;   // "still trusted" residual
constexpr double TUKEY_OUTLIER_W_MAX   = 1e-9;   // Tukey redescends to exactly 0
constexpr double TUKEY_INLIER_W_MIN    = 0.95;
constexpr double FD_STEP               = 1e-6;   // independent finite-difference
constexpr double FD_TOL                = 1e-4;   // step-dependent agreement band
// Injection instants for the gate test (keyframe indices on the perfect-VO run).
constexpr int    FIX_OUTLIER_KF = 20;
constexpr int    FIX_INLIER_KF  = 40;

// ── Time-threshold split probes (check 6b) ────────────────────────────────
//! 10 s old: past the retired 5 s fix_max_age_sec but far inside the age
//! budget (lag_seconds 40 − fix_age_margin_sec 0.5 = 39.5 s). A whole number
//! of keyframe intervals, so it still lands EXACTLY on a live keyframe.
constexpr int    FIX_AGED_OK_KF        = 40;
constexpr double FIX_AGED_OK_SHIFT_SEC = -10.0;
//! 0.4 s off any keyframe (KFs are 1 s apart): inside the retired 5 s window
//! but outside fix_match_tolerance_sec = 0.15 s ⇒ must NOT be attached.
constexpr int    FIX_MISMATCH_KF        = 20;
constexpr double FIX_MISMATCH_SHIFT_SEC = 0.4;
//! 45 s old ⇒ beyond the 39.5 s budget.
constexpr int    FIX_AGE_EXPIRED_KF        = 50;
constexpr double FIX_AGE_EXPIRED_SHIFT_SEC = -45.0;

// ── Drift-inflation probes (check 6c) ─────────────────────────────────────
//! Injection keyframe, residual and drift rate chosen so the SAME fix falls
//! outside the gate at rho = 0 and inside it at rho = FIX_DRIFT_RATE.
//! Calibration [đo, this test's own log]: with rho = 0 the module reports a
//! gate radius of 310.8 m at k = 20 (sigma_pred here is dominated by the weak
//! 100 m anchor_xy prior — check 7 measures 100…179 m). An 800 m residual is
//! therefore rejected with 2.6x margin. At k = 20 the flown path is s = 400 m,
//! so rho = 2.0 adds an 800 m drift sigma and pushes the radius past 2 400 m —
//! the same fix is then admitted with 3x margin. Both margins are large, so
//! the probe tests the mechanism and not a threshold.
constexpr int    FIX_DRIFT_KF       = 20;
constexpr double FIX_DRIFT_OFFSET_M = 800.0;
constexpr double FIX_DRIFT_RATE     = 2.0;
//! Tolerance when re-deriving the flown path from the scenario geometry: the
//! fused positions track GT to well under a metre per keyframe (check 1).
constexpr double FIX_DISTANCE_TOL_M = 10.0;

// ── Confidence-floor probes (check 6e) ────────────────────────────────────
//! Floor applied by the consumer, and a producer confidence just under it.
//! The value mirrors kcb_slam's min_keyframe_confidence (0.35); only the
//! ORDER of the two numbers matters to the probe.
constexpr double FIX_MIN_CONFIDENCE = 0.35;
constexpr double FIX_LOW_CONFIDENCE = 0.30;

// ── Covariance / accuracy / health (S7) ───────────────────────────────────
//! Synthetic health thresholds — chosen so 30 m lands INSIDE the hysteresis
//! band, which is what the no-chatter probe needs.
constexpr double HEALTH_CONV_SIGMA_M = 25.0;
constexpr double HEALTH_DRIFT_SIGMA_M = 50.0;
constexpr double HEALTH_BAND_SIGMA_M  = 30.0;  // between the two thresholds
constexpr double HEALTH_LOW_SIGMA_M   = 10.0;  // below the converged threshold
constexpr double HEALTH_HIGH_SIGMA_M  = 60.0;  // above the drifting threshold
constexpr int    HEALTH_CHATTER_STEPS = 20;    // sweeps inside the dead zone
//! Tolerance when re-deriving sqrt(max eigenvalue) by hand [m].
constexpr double ACCURACY_TOL_M = 1e-9;

double heading_gt_deg(int k) {
    return (k <= LEG_KFS) ? HEADING_EAST_DEG : HEADING_NORTH_DEG;
}

Eigen::Vector3d position_gt_enu(int k) {
    const double step = SPEED_MPS * KF_DT_MSEC * 1e-3;
    if (k <= LEG_KFS) {
        return {step * k, 0.0, AGL_M};  // eastward leg
    }
    return {step * LEG_KFS, step * (k - LEG_KFS), AGL_M};  // northward leg
}

Eigen::Isometry3d pose_gt_enu(int k) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear()      = uavloc::fusion::rotation_enu_camera(
        0.0, 0.0, heading_gt_deg(k), 0.0, TILT_NADIR_DEG);
    T.translation() = position_gt_enu(k);
    return T;
}

//! GT VO-world poses: world = first camera frame, T_wc(k) = T_enu_w⁻¹·T_enu_c(k).
std::vector<Eigen::Isometry3d> build_gt_vo(const Eigen::Isometry3d& T_enu_w) {
    std::vector<Eigen::Isometry3d> T_wc(NUM_KFS);
    for (int k = 0; k < NUM_KFS; ++k) {
        T_wc[k] = T_enu_w.inverse() * pose_gt_enu(k);
    }
    return T_wc;
}

//! Corrupt the GT relative increments and re-chain. The yaw drift is applied
//! about the ENU vertical and the Z drift along the ENU vertical, both mapped
//! into the GT camera(k-1) frame (world Z is NOT vertical — the camera is
//! nadir, so the vertical is the optical axis direction).
std::vector<Eigen::Isometry3d> build_corrupted_vo(
    const std::vector<Eigen::Isometry3d>& T_wc_gt,
    const Eigen::Isometry3d&              T_enu_w) {
    std::vector<Eigen::Isometry3d> T_wc(NUM_KFS);
    T_wc[0] = T_wc_gt[0];
    for (int k = 1; k < NUM_KFS; ++k) {
        const Eigen::Isometry3d T_rel = T_wc_gt[k - 1].inverse() * T_wc_gt[k];
        // ENU up expressed in the GT camera(k-1) frame.
        const Eigen::Vector3d up_c =
            (T_enu_w.linear() * T_wc_gt[k - 1].linear()).transpose() *
            Eigen::Vector3d::UnitZ();
        Eigen::Isometry3d bad = Eigen::Isometry3d::Identity();
        bad.linear() =
            Eigen::AngleAxisd(YAW_DRIFT_DEG_PER_KF * D2R, up_c).toRotationMatrix() *
            T_rel.linear();
        bad.translation() =
            SCALE_CORRUPTION * (T_rel.translation() + Z_DRIFT_M_PER_KF * up_c);
        T_wc[k] = T_wc[k - 1] * bad;
    }
    return T_wc;
}

uavloc::fusion::FusionConfig make_config() {
    uavloc::fusion::FusionConfig cfg;  // defaults, then scenario overrides:
    cfg.async_enabled = false;         // deterministic sync mode

    // The synthetic telemetry heading is exact while the corrupted VO carries
    // a systematic +0.1°/KF yaw bias — trust heading deltas more than VO
    // rotation here (per-link information ratio decides how much of the VO
    // yaw bias survives).
    cfg.vo_rot_sigma_deg    = 1.0;
    cfg.delta_yaw_sigma_rad = 0.0035;  // ≈ 0.2°

    // VO is metric-seeded (s ≈ 1 by construction) and in F1 nothing but the
    // prior makes s strongly observable (design doc §6) — keep the scale
    // state pinned near the seed so the unmodelled additive vertical drift
    // cannot alias into s.
    cfg.scale_prior_sigma = 0.02;
    cfg.scale_walk_sigma  = 0.001;

    // Flat terrain in this scenario: a slow bias walk keeps the AGL factor
    // from ceding Z to the injected vertical drift.
    cfg.agl_bias_walk_m = 0.1;
    return cfg;
}

struct RunOutcome {
    std::vector<uavloc::fusion::FusionResult> results;
};

RunOutcome run_fusion(const std::vector<Eigen::Isometry3d>& T_wc,
                      double heading_offset_deg) {
    uavloc::fusion::FusionModule module(make_config());

    RunOutcome outcome;
    module.add_result_callback([&outcome](const uavloc::fusion::FusionResult& r) {
        outcome.results.push_back(r);
    });

    module.start();  // no-op in sync mode
    for (int k = 0; k < NUM_KFS; ++k) {
        uavloc::vo::VOResult res;
        res.frame_id       = static_cast<unsigned int>(k);
        res.timestamp_msec = k * KF_DT_MSEC;
        res.state          = uavloc::vo::VOTrackingState::TRACKING;
        res.T_wc           = T_wc[static_cast<size_t>(k)].matrix();
        res.has_pose       = true;
        res.is_keyframe    = true;  // KF cadence == feed cadence in this test

        uavloc::sensor::TelemetryData telem;
        telem.frame_id        = static_cast<uint64_t>(k);
        telem.timestamp_msec  = k * KF_DT_MSEC;
        telem.heading_deg     = heading_gt_deg(k) + heading_offset_deg;
        telem.gimbal_tilt_deg = TILT_NADIR_DEG;
        telem.altitude_m      = AGL_M;
        telem.valid           = true;

        module.push(res, telem);
    }
    module.stop();
    return outcome;
}

double yaw_deg_of(const Eigen::Matrix3d& R_enu_c) {
    return uavloc::fusion::yaw_of_enu_rotation(gtsam::Rot3(R_enu_c)) * R2D;
}

double wrap_deg(double a) {
    return uavloc::fusion::wrap_angle(a * D2R) * R2D;
}

// ── Check 0: attitude-chain conventions on known synthetic cases ───────────
bool check_attitude_chain() {
    bool ok = true;

    // heading = 90° (East), level, no gimbal: body-forward (= camera optical
    // axis z at pan = tilt = 0) must map to +x East; body-right → South.
    {
        const Eigen::Matrix3d R =
            uavloc::fusion::rotation_enu_camera(0.0, 0.0, 90.0, 0.0, 0.0);
        if ((R * Eigen::Vector3d::UnitZ() - Eigen::Vector3d(1, 0, 0)).norm() > CHAIN_TOL) {
            spdlog::error("chain: heading=90 body-forward != +x East");
            ok = false;
        }
        if ((R * Eigen::Vector3d::UnitX() - Eigen::Vector3d(0, -1, 0)).norm() > CHAIN_TOL) {
            spdlog::error("chain: heading=90 body-right != South");
            ok = false;
        }
    }

    // Nadir (tilt = 90°), heading = 90°: optical axis straight down; camera x
    // (image right) = right of an east-flying aircraft = South.
    {
        const Eigen::Matrix3d R =
            uavloc::fusion::rotation_enu_camera(0.0, 0.0, 90.0, 0.0, 90.0);
        if ((R * Eigen::Vector3d::UnitZ() - Eigen::Vector3d(0, 0, -1)).norm() > CHAIN_TOL) {
            spdlog::error("chain: nadir optical axis != straight down");
            ok = false;
        }
        if ((R * Eigen::Vector3d::UnitX() - Eigen::Vector3d(0, -1, 0)).norm() > CHAIN_TOL) {
            spdlog::error("chain: nadir camera-x != South");
            ok = false;
        }
    }

    // Pan adds to heading: heading = 0 + pan = 90 must equal heading = 90 + pan = 0.
    {
        const Eigen::Matrix3d Ra =
            uavloc::fusion::rotation_enu_camera(0.0, 0.0, 0.0, 90.0, 0.0);
        const Eigen::Matrix3d Rb =
            uavloc::fusion::rotation_enu_camera(0.0, 0.0, 90.0, 0.0, 0.0);
        if ((Ra - Rb).norm() > CHAIN_TOL) {
            spdlog::error("chain: pan does not add to heading");
            ok = false;
        }
    }
    return ok;
}

// ── Check 0b: MapDepthFactor sign, zero and log-relativity ─────────────────
// The map shrinks relative to metric, so d_vo < AGL: at s = 1 the residual
// must be NEGATIVE (the factor pushes s UP) and it must vanish at the true
// scale s = AGL / (d_vo · cos β), independently of the absolute AGL.
bool check_map_depth_factor() {
    using uavloc::fusion::MapDepthFactor;
    bool ok = true;

    const auto   noise  = gtsam::noiseModel::Isotropic::Sigma(1, MAP_DEPTH_SIGMA);
    const auto   key    = gtsam::Symbol('s', 0);
    const double agl    = 100.0;
    const double d_vo   = MAP_DEPTH_RATIO * agl;
    const double s_true = 1.0 / MAP_DEPTH_RATIO;
    const MapDepthFactor f(key, d_vo, agl, 1.0, noise);

    const double e_unit = f.error_function(1.0)(0);
    if (!(e_unit < 0.0)) {
        spdlog::error("map-depth: residual at s=1 is {:+.4f}, expected negative "
                      "(the factor must push s UP)", e_unit);
        ok = false;
    }
    if (std::abs(e_unit - std::log(MAP_DEPTH_RATIO)) > CHAIN_TOL) {
        spdlog::error("map-depth: residual at s=1 is {:+.6f}, expected log(ratio)",
                      e_unit);
        ok = false;
    }
    if (std::abs(f.error_function(s_true)(0)) > CHAIN_TOL) {
        spdlog::error("map-depth: residual at the true scale is {:+.6f}, expected 0",
                      f.error_function(s_true)(0));
        ok = false;
    }
    // Log form ⇒ the residual depends only on the RATIO, not on the altitude.
    {
        const double         agl_hi = 10.0 * agl;
        const MapDepthFactor f_hi(key, MAP_DEPTH_RATIO * agl_hi, agl_hi, 1.0, noise);
        if (std::abs(f_hi.error_function(1.0)(0) - e_unit) > CHAIN_TOL) {
            spdlog::error("map-depth: residual is altitude-dependent");
            ok = false;
        }
    }
    // Degenerate input ⇒ inert (zero residual), never NaN.
    {
        const MapDepthFactor f_bad(key, 0.0, agl, 1.0, noise);
        if (f_bad.error_function(1.0)(0) != 0.0) {
            spdlog::error("map-depth: non-positive depth did not give a zero residual");
            ok = false;
        }
    }
    // Analytic Jacobian of log(s·d·cos β) is 1/s.
    {
        gtsam::Matrix H;
        f.evaluateError(s_true, &H);
        if (std::abs(H(0, 0) - 1.0 / s_true) > JACOBIAN_TOL) {
            spdlog::error("map-depth: dE/ds = {:.6f}, expected {:.6f}",
                          H(0, 0), 1.0 / s_true);
            ok = false;
        }
    }
    return ok;
}

// ── Check 0c: AbsoluteXYFactor residual, Jacobian and robust weighting ─────
bool check_absolute_xy_factor() {
    using uavloc::fusion::AbsoluteXYFactor;
    bool ok = true;

    const auto     noise = gtsam::noiseModel::Isotropic::Sigma(2, FIX_SIGMA_M);
    const auto     key   = gtsam::Symbol('x', 0);
    // A non-trivial pose: nadir camera, heading 30°, well off the ENU origin —
    // so a wrong frame or a wrong tangent index cannot pass unnoticed.
    const Eigen::Vector3d t_enu(10.0, -5.0, AGL_M);
    const gtsam::Pose3    p(
        gtsam::Rot3(uavloc::fusion::rotation_enu_camera(0.0, 0.0, 30.0, 0.0,
                                                        TILT_NADIR_DEG)),
        gtsam::Point3(t_enu));

    // (a) Measurement FIX_TEST_OFFSET_M west of the pose ⇒ residual is
    //     (pose − measurement) = (+offset, 0): positive East, zero North.
    const Eigen::Vector2d meas(t_enu.x() - FIX_TEST_OFFSET_M, t_enu.y());
    const AbsoluteXYFactor f(key, meas, noise);
    const gtsam::Vector    e = f.error_function(p);
    if (e.size() != 2) {
        spdlog::error("absolute-xy: residual has {} rows, expected 2", e.size());
        return false;
    }
    if (std::abs(e(0) - FIX_TEST_OFFSET_M) > CHAIN_TOL || std::abs(e(1)) > CHAIN_TOL) {
        spdlog::error("absolute-xy: residual ({:+.4f}, {:+.4f}), expected ({:+.1f}, 0)",
                      e(0), e(1), FIX_TEST_OFFSET_M);
        ok = false;
    }

    // (b) Jacobian. Pose3 retract translates by R·v, so analytically
    //     d(t_enu)/d(tangent) = [0_{2x3} | R.topRows(2)].
    gtsam::Matrix H;
    f.evaluateError(p, &H);
    if (H.rows() != 2 || H.cols() != 6) {
        spdlog::error("absolute-xy: Jacobian is {}x{}, expected 2x6", H.rows(), H.cols());
        return false;
    }
    const Eigen::Matrix3d R = p.rotation().matrix();
    if (H.block<2, 3>(0, 0).norm() > JACOBIAN_TOL) {
        spdlog::error("absolute-xy: rotation block of the Jacobian is not zero "
                      "(norm {:.2e})", H.block<2, 3>(0, 0).norm());
        ok = false;
    }
    if ((H.block<2, 3>(0, 3) - R.topRows<2>()).norm() > JACOBIAN_TOL) {
        spdlog::error("absolute-xy: translation block of the Jacobian != R.topRows(2) "
                      "(diff {:.2e})", (H.block<2, 3>(0, 3) - R.topRows<2>()).norm());
        ok = false;
    }
    // Independent central difference (different step than the factor's own):
    // catches a wrong variable/ordering, not just a wrong closed form.
    for (int j = 0; j < 6; ++j) {
        gtsam::Vector6 d = gtsam::Vector6::Zero();
        d(j) = FD_STEP;
        const gtsam::Vector fd =
            (f.error_function(p.retract(d)) - f.error_function(p.retract(-d))) /
            (2.0 * FD_STEP);
        if ((fd - H.col(j)).norm() > FD_TOL) {
            spdlog::error("absolute-xy: Jacobian column {} disagrees with the "
                          "finite difference (diff {:.2e})", j, (fd - H.col(j)).norm());
            ok = false;
        }
    }

    // (c) Tukey robust wrapping: a 1 km outlier must lose essentially all of
    //     its weight while a FIX_ROBUST_RESIDUAL_M residual keeps essentially
    //     all of it (the gate runs first; this only shapes what got through).
    uavloc::fusion::FusionConfig cfg;  // defaults carry fix_tukey_c
    const auto tukey =
        gtsam::noiseModel::mEstimator::Tukey::Create(cfg.fix_tukey_c);
    const auto robust = gtsam::noiseModel::Robust::Create(tukey, noise);
    const double w_out = robust->robust()->weight(FIX_OUTLIER_OFFSET_M / FIX_SIGMA_M);
    const double w_in  = robust->robust()->weight(FIX_ROBUST_RESIDUAL_M / FIX_SIGMA_M);
    spdlog::info("absolute-xy: Tukey(c={:.3f}) weight — {:.0f} m outlier {:.3e}, "
                 "{:.0f} m residual {:.4f}",
                 cfg.fix_tukey_c, FIX_OUTLIER_OFFSET_M, w_out,
                 FIX_ROBUST_RESIDUAL_M, w_in);
    if (w_out > TUKEY_OUTLIER_W_MAX) {
        spdlog::error("absolute-xy: outlier weight {:.3e} > {:.0e}", w_out,
                      TUKEY_OUTLIER_W_MAX);
        ok = false;
    }
    if (w_in < TUKEY_INLIER_W_MIN) {
        spdlog::error("absolute-xy: inlier weight {:.4f} < {:.2f}", w_in,
                      TUKEY_INLIER_W_MIN);
        ok = false;
    }
    return ok;
}

//! One scripted absolute-fix injection: pushed right AFTER the keyframe `k`
//! has been fed, carrying keyframe k's timestamp + `time_shift_sec` and GT
//! position + `offset`.
struct FixInjection {
    int             k;
    Eigen::Vector2d offset;
    //! Deliberate timestamp error, used to probe the two independent time
    //! thresholds (match tolerance vs age budget).
    double          time_shift_sec = 0.0;
    //! Producer-side confidence carried by the fix. The default matches
    //! anchor::AbsoluteFix's own default, so probes that do not care about the
    //! confidence floor behave exactly as before it existed.
    double          confidence = 0.0;
};

//! make_config() with the self-consistency Mahalanobis gate switched ON. The
//! gate is OFF by default since 2026-08-02 (FusionConfig::fix_gate_enabled), so
//! every probe that is ABOUT the gate has to ask for it explicitly.
uavloc::fusion::FusionConfig make_gate_config() {
    uavloc::fusion::FusionConfig cfg = make_config();
    cfg.fix_gate_enabled = true;
    return cfg;
}

//! Perfect-VO run with scripted fix injections; returns the intake accounting.
uavloc::fusion::FusionFixStats run_fusion_with_fixes(
    const std::vector<Eigen::Isometry3d>&  T_wc,
    const std::vector<FixInjection>&       injections,
    bool                                   push_before_init,
    const uavloc::fusion::FusionConfig&    cfg = make_config()) {
    uavloc::fusion::FusionModule module(cfg);
    module.start();  // no-op in sync mode

    if (push_before_init) {
        uavloc::anchor::AbsoluteFix early;
        early.timestamp_msec = 0.0;
        early.cov            = FIX_SIGMA_M * FIX_SIGMA_M * Eigen::Matrix2d::Identity();
        early.valid          = true;
        module.push_absolute_fix(early);
    }

    for (int k = 0; k < NUM_KFS; ++k) {
        uavloc::vo::VOResult res;
        res.frame_id       = static_cast<unsigned int>(k);
        res.timestamp_msec = k * KF_DT_MSEC;
        res.state          = uavloc::vo::VOTrackingState::TRACKING;
        res.T_wc           = T_wc[static_cast<size_t>(k)].matrix();
        res.has_pose       = true;
        res.is_keyframe    = true;

        uavloc::sensor::TelemetryData telem;
        telem.frame_id        = static_cast<uint64_t>(k);
        telem.timestamp_msec  = k * KF_DT_MSEC;
        telem.heading_deg     = heading_gt_deg(k);
        telem.gimbal_tilt_deg = TILT_NADIR_DEG;
        telem.altitude_m      = AGL_M;
        telem.valid           = true;

        module.push(res, telem);

        // Pushed after the keyframe update, so the fix attaches to a keyframe
        // that is already inside the smoother and therefore has a marginal
        // covariance for the gate (it is consumed by the NEXT keyframe).
        for (const auto& inj : injections) {
            if (inj.k != k) continue;
            uavloc::anchor::AbsoluteFix fix;
            fix.timestamp_msec = k * KF_DT_MSEC + inj.time_shift_sec * 1000.0;
            fix.xy_enu = position_gt_enu(k).head<2>() + inj.offset;
            fix.cov        = FIX_SIGMA_M * FIX_SIGMA_M * Eigen::Matrix2d::Identity();
            fix.confidence = inj.confidence;
            fix.valid      = true;
            module.push_absolute_fix(fix);
        }
    }
    module.stop();
    return module.fix_stats();
}

// ── Check 6: Mahalanobis gate + malformed-input rejection ──────────────────
bool check_absolute_fix_gate(const std::vector<Eigen::Isometry3d>& T_wc_gt) {
    bool ok = true;

    const std::vector<FixInjection> injections = {
        {FIX_OUTLIER_KF, Eigen::Vector2d(FIX_OUTLIER_OFFSET_M, 0.0)},
        {FIX_INLIER_KF,  Eigen::Vector2d(FIX_INLIER_OFFSET_M, 0.0)},
    };
    const uavloc::fusion::FusionFixStats st =
        run_fusion_with_fixes(T_wc_gt, injections, /*push_before_init=*/true,
                              make_gate_config());
    spdlog::info("fix intake: injected={} applied={} gated={} low_confidence={} "
                 "age_expired={} unmatched={} marginalized={} queue_dropped={} "
                 "no_graph={}",
                 st.injected, st.applied, st.gated, st.low_confidence,
                 st.age_expired, st.unmatched, st.marginalized, st.queue_dropped,
                 st.no_graph);
    if (st.injected != 3 || st.applied != 1 || st.gated != 1 ||
        st.age_expired != 0 || st.unmatched != 0 || st.marginalized != 0 ||
        st.queue_dropped != 0 || st.no_graph != 1) {
        spdlog::error("check 6 FAIL: expected injected=3 applied=1 gated=1 "
                      "age_expired=0 unmatched=0 marginalized=0 "
                      "queue_dropped=0 no_graph=1");
        ok = false;
    }
    // The gated fix must have LEFT ITS RESIDUAL BEHIND: a rejection with no
    // recorded magnitude is what made the M1 post-mortem guesswork.
    if (!(st.last_gated_residual_m > 0.0) ||
        !(st.last_gated_gate_radius_m > 0.0) ||
        !(st.last_gated_residual_m > st.last_gated_gate_radius_m)) {
        spdlog::error("check 6 FAIL: gated fix recorded residual {:.2f} m / gate "
                      "radius {:.2f} m — expected both > 0 with residual OUTSIDE "
                      "the radius", st.last_gated_residual_m,
                      st.last_gated_gate_radius_m);
        ok = false;
    }

    // Malformed fixes must be refused outright — not even counted as injected.
    {
        uavloc::fusion::FusionModule module(make_config());
        uavloc::anchor::AbsoluteFix bad;
        bad.valid = false;                       // not a measurement
        module.push_absolute_fix(bad);
        bad.valid = true;
        bad.cov   = Eigen::Matrix2d::Zero();     // not positive definite
        module.push_absolute_fix(bad);
        bad.cov            = Eigen::Matrix2d::Identity();
        bad.xy_enu.x()     = std::numeric_limits<double>::quiet_NaN();
        module.push_absolute_fix(bad);
        if (module.fix_stats().injected != 0) {
            spdlog::error("check 6 FAIL: malformed fixes were accepted ({} injected)",
                          module.fix_stats().injected);
            ok = false;
        }
    }
    return ok;
}

// ── Check 6b: the two time thresholds are INDEPENDENT ──────────────────────
//! The retired fix_max_age_sec (5 s) served as both the age budget and the
//! keyframe-match tolerance, two quantities that need opposite values. The
//! probes below can only all pass if they are separate knobs:
//!   * a fix 10 s old but landing exactly on a live keyframe is APPLIED
//!     (the old 5 s limit would have dropped it);
//!   * a fix 0.4 s off any keyframe is UNMATCHED
//!     (the old 5 s limit would have attached it to the wrong pose);
//!   * a fix 45 s old — past lag_seconds − fix_age_margin_sec = 39.5 s — is
//!     AGE_EXPIRED, and counted as such rather than merged with the above.
bool check_fix_time_thresholds(const std::vector<Eigen::Isometry3d>& T_wc_gt) {
    bool ok = true;

    const std::vector<FixInjection> injections = {
        {FIX_AGED_OK_KF,      Eigen::Vector2d(FIX_INLIER_OFFSET_M, 0.0), FIX_AGED_OK_SHIFT_SEC},
        {FIX_MISMATCH_KF,     Eigen::Vector2d(FIX_INLIER_OFFSET_M, 0.0), FIX_MISMATCH_SHIFT_SEC},
        {FIX_AGE_EXPIRED_KF,  Eigen::Vector2d(FIX_INLIER_OFFSET_M, 0.0), FIX_AGE_EXPIRED_SHIFT_SEC},
    };
    const uavloc::fusion::FusionFixStats st =
        run_fusion_with_fixes(T_wc_gt, injections, /*push_before_init=*/false,
                              make_gate_config());
    spdlog::info("time thresholds: injected={} applied={} gated={} age_expired={} "
                 "unmatched={} marginalized={}",
                 st.injected, st.applied, st.gated, st.age_expired,
                 st.unmatched, st.marginalized);
    if (st.injected != 3 || st.applied != 1 || st.unmatched != 1 ||
        st.age_expired != 1 || st.gated != 0 || st.marginalized != 0) {
        spdlog::error("check 6b FAIL: expected injected=3 applied=1 unmatched=1 "
                      "age_expired=1 gated=0 marginalized=0");
        ok = false;
    }
    return ok;
}

// ── Check 6c: drift inflation of the gate covariance ───────────────────────
//! S = cov_fix + cov_pred + (rho·s)²·I. One and the same fix, one and the same
//! trajectory: rejected with rho = 0 and admitted with rho > 0, because after
//! s metres of flight the gate must be wide enough for a CORRECT fix seen from
//! a drifted estimate (m1_fake_anchor.md §6). Also checks that `s` itself is
//! accumulated over the flown path and reset by an APPLIED fix.
bool check_fix_drift_inflation(const std::vector<Eigen::Isometry3d>& T_wc_gt) {
    bool ok = true;

    const std::vector<FixInjection> injections = {
        {FIX_DRIFT_KF, Eigen::Vector2d(FIX_DRIFT_OFFSET_M, 0.0), 0.0},
    };

    uavloc::fusion::FusionConfig cfg_off = make_gate_config();  // rho = 0
    uavloc::fusion::FusionConfig cfg_on  = make_gate_config();
    cfg_on.fix_drift_rate_m_per_m = FIX_DRIFT_RATE;

    const uavloc::fusion::FusionFixStats off =
        run_fusion_with_fixes(T_wc_gt, injections, false, cfg_off);
    const uavloc::fusion::FusionFixStats on =
        run_fusion_with_fixes(T_wc_gt, injections, false, cfg_on);

    spdlog::info("drift gate: rho=0 → applied={} gated={} (residual {:.1f} m vs "
                 "radius {:.1f} m); rho={} → applied={} gated={}",
                 off.applied, off.gated, off.last_gated_residual_m,
                 off.last_gated_gate_radius_m, FIX_DRIFT_RATE, on.applied,
                 on.gated);
    if (off.applied != 0 || off.gated != 1) {
        spdlog::error("check 6c FAIL: with rho=0 the {:.0f} m fix should be GATED",
                      FIX_DRIFT_OFFSET_M);
        ok = false;
    }
    if (on.applied != 1 || on.gated != 0) {
        spdlog::error("check 6c FAIL: with rho={} the same fix should be APPLIED "
                      "(gate widened by the drift term)", FIX_DRIFT_RATE);
        ok = false;
    }
    // The gate radius the rho=0 run reported must match the closed form
    // 3.03·sqrt(sigma_fix² + sigma_pred²) to within the cov_pred it also
    // reports — here only the ORDER is checkable without the internal
    // covariance, so assert the inequality chain that made the fix admissible:
    // rho·s must exceed the recorded (too small) gate radius.
    const double s_expected_m = SPEED_MPS * KF_DT_MSEC * 1e-3 * FIX_DRIFT_KF;
    if (!(FIX_DRIFT_RATE * s_expected_m > off.last_gated_gate_radius_m)) {
        spdlog::error("check 6c FAIL: drift sigma {:.1f} m does not exceed the "
                      "rho=0 gate radius {:.1f} m — the probe proves nothing",
                      FIX_DRIFT_RATE * s_expected_m, off.last_gated_gate_radius_m);
        ok = false;
    }

    // `s` accounting: with no fix ever applied it is the whole flown path; the
    // applied fix restarts it at the keyframe that consumed the fix.
    //
    // The probe uses a ZERO-residual fix on purpose. s is measured on the
    // graph's OPTIMIZED positions, so an applied fix with a large residual also
    // MOVES the estimate and that displacement legitimately counts as flown
    // path — which is exactly what happens with the (default) Huber kernel:
    // the 800 m fix above ends at s = 1370.8 m instead of 800 m [đo]. Under the
    // former Tukey default the same fix was redescended to ~zero weight and the
    // discrepancy was invisible. Separating the two probes keeps this one about
    // the accounting and nothing else.
    const uavloc::fusion::FusionFixStats none =
        run_fusion_with_fixes(T_wc_gt, {}, false, cfg_on);
    const uavloc::fusion::FusionFixStats reset = run_fusion_with_fixes(
        T_wc_gt, {{FIX_DRIFT_KF, Eigen::Vector2d::Zero(), 0.0, 0.0}}, false,
        make_gate_config());
    const double leg_m       = SPEED_MPS * KF_DT_MSEC * 1e-3;
    const double full_path_m = leg_m * (NUM_KFS - 1);
    // The fix pushed after KF FIX_DRIFT_KF is consumed by KF FIX_DRIFT_KF+1,
    // which resets s BEFORE adding its own leg.
    const double after_fix_m = leg_m * (NUM_KFS - 1 - FIX_DRIFT_KF);
    spdlog::info("drift distance: no fix → s={:.1f} m (expect {:.1f}); after an "
                 "applied zero-residual fix → s={:.1f} m (expect {:.1f}); after "
                 "the applied {:.0f} m fix → s={:.1f} m (estimate displacement "
                 "included)",
                 none.distance_since_fix_m, full_path_m,
                 reset.distance_since_fix_m, after_fix_m,
                 FIX_DRIFT_OFFSET_M, on.distance_since_fix_m);
    if (std::abs(none.distance_since_fix_m - full_path_m) > FIX_DISTANCE_TOL_M) {
        spdlog::error("check 6c FAIL: distance_since_fix_m {:.2f} m != flown path "
                      "{:.2f} m", none.distance_since_fix_m, full_path_m);
        ok = false;
    }
    if (reset.applied != 1) {
        spdlog::error("check 6c FAIL: the zero-residual probe fix was not applied "
                      "(applied={}) — the reset assertion below proves nothing",
                      reset.applied);
        ok = false;
    }
    if (std::abs(reset.distance_since_fix_m - after_fix_m) > FIX_DISTANCE_TOL_M) {
        spdlog::error("check 6c FAIL: distance_since_fix_m {:.2f} m after an "
                      "applied fix != {:.2f} m", reset.distance_since_fix_m,
                      after_fix_m);
        ok = false;
    }
    return ok;
}

// ── Check 6d: fix_gate_enabled is a real switch ────────────────────────────
//! The SAME fix, on the SAME trajectory, with the gate as the only difference:
//! the 1 km outlier of check 6 is rejected when the gate runs and reaches the
//! graph when it does not. This is the escape hatch out of the self-feeding
//! rejection loop measured on YenBai (FusionConfig::fix_gate_enabled) — and,
//! read the other way round, the proof that with the default configuration
//! NOTHING rejects a gross outlier any more.
bool check_fix_gate_switch(const std::vector<Eigen::Isometry3d>& T_wc_gt) {
    bool ok = true;

    const std::vector<FixInjection> injections = {
        {FIX_OUTLIER_KF, Eigen::Vector2d(FIX_OUTLIER_OFFSET_M, 0.0)},
    };

    const uavloc::fusion::FusionFixStats on =
        run_fusion_with_fixes(T_wc_gt, injections, false, make_gate_config());
    // make_config() carries the shipped default: gate OFF.
    const uavloc::fusion::FusionFixStats off =
        run_fusion_with_fixes(T_wc_gt, injections, false, make_config());

    spdlog::info("gate switch: {:.0f} m fix — gate ON → applied={} gated={}; "
                 "gate OFF (default) → applied={} gated={}",
                 FIX_OUTLIER_OFFSET_M, on.applied, on.gated, off.applied,
                 off.gated);
    if (on.applied != 0 || on.gated != 1) {
        spdlog::error("check 6d FAIL: with fix_gate_enabled=true the {:.0f} m "
                      "fix must be GATED", FIX_OUTLIER_OFFSET_M);
        ok = false;
    }
    if (off.applied != 1 || off.gated != 0) {
        spdlog::error("check 6d FAIL: with fix_gate_enabled=false the {:.0f} m "
                      "fix must reach the graph (applied=1, gated=0)",
                      FIX_OUTLIER_OFFSET_M);
        ok = false;
    }
    // A disabled gate must also leave its observability fields untouched
    // rather than reporting a stale/half-computed rejection.
    if (off.last_gated_residual_m != 0.0 || off.last_gated_gate_radius_m != 0.0) {
        spdlog::error("check 6d FAIL: gate disabled but residual/radius were "
                      "written ({:.2f} / {:.2f})", off.last_gated_residual_m,
                      off.last_gated_gate_radius_m);
        ok = false;
    }
    return ok;
}

// ── Check 6e: producer-side confidence floor ───────────────────────────────
//! fix_min_confidence filters on what the PRODUCER reports about its own match
//! — never on agreement with the estimate — so it cannot feed a drift loop.
//! Two fixes of the SAME (small) residual, differing only in confidence: the
//! one below the floor is counted as low_confidence and never reaches the
//! graph, the one at the floor is applied.
bool check_fix_confidence_floor(const std::vector<Eigen::Isometry3d>& T_wc_gt) {
    bool ok = true;

    uavloc::fusion::FusionConfig cfg = make_config();  // gate off (default)
    cfg.fix_min_confidence = FIX_MIN_CONFIDENCE;

    const std::vector<FixInjection> injections = {
        {FIX_OUTLIER_KF, Eigen::Vector2d(FIX_INLIER_OFFSET_M, 0.0), 0.0,
         FIX_LOW_CONFIDENCE},
        {FIX_INLIER_KF,  Eigen::Vector2d(FIX_INLIER_OFFSET_M, 0.0), 0.0,
         FIX_MIN_CONFIDENCE},
    };
    const uavloc::fusion::FusionFixStats st =
        run_fusion_with_fixes(T_wc_gt, injections, false, cfg);

    spdlog::info("confidence floor ({:.2f}): injected={} applied={} "
                 "low_confidence={} gated={}",
                 FIX_MIN_CONFIDENCE, st.injected, st.applied, st.low_confidence,
                 st.gated);
    if (st.injected != 2 || st.applied != 1 || st.low_confidence != 1 ||
        st.gated != 0) {
        spdlog::error("check 6e FAIL: expected injected=2 applied=1 "
                      "low_confidence=1 gated=0");
        ok = false;
    }

    // Default floor (0.0) must be a true no-op: the same two fixes both land.
    const uavloc::fusion::FusionFixStats none =
        run_fusion_with_fixes(T_wc_gt, injections, false, make_config());
    if (none.applied != 2 || none.low_confidence != 0) {
        spdlog::error("check 6e FAIL: with the default fix_min_confidence=0 both "
                      "fixes must be applied (applied={} low_confidence={})",
                      none.applied, none.low_confidence);
        ok = false;
    }
    return ok;
}

// ── Check 7: marginal covariance → accuracy_m (S7) ─────────────────────────
//! Every fused result of a healthy run must carry a usable horizontal
//! covariance, and the accuracy derived from it must be the largest 1-sigma
//! semi-axis — NaN, never 0, when no covariance exists.
bool check_covariance_and_accuracy(const std::vector<Eigen::Isometry3d>& T_wc_gt) {
    using uavloc::fusion::horizontal_accuracy_m;
    bool ok = true;

    const RunOutcome run = run_fusion(T_wc_gt, 0.0);
    if (run.results.size() != static_cast<size_t>(NUM_KFS)) {
        spdlog::error("check 7 FAIL: expected {} results, got {}",
                      NUM_KFS, run.results.size());
        return false;
    }

    double acc_min = std::numeric_limits<double>::infinity();
    double acc_max = 0.0;
    for (int k = 0; k < NUM_KFS; ++k) {
        const auto&  r   = run.results[static_cast<size_t>(k)];
        const double acc = horizontal_accuracy_m(r);
        if (!r.covariance_valid) {
            spdlog::error("check 7 FAIL: no covariance at k={}", k);
            ok = false;
            break;
        }
        // Symmetric (it is R·Σ·Rᵀ of a symmetric block) and positive definite.
        const Eigen::Matrix2d& C = r.xy_covariance;
        if (!C.allFinite() || std::abs(C(0, 1) - C(1, 0)) > ACCURACY_TOL_M ||
            C(0, 0) <= 0.0 || C(1, 1) <= 0.0 ||
            C.determinant() <= 0.0) {
            spdlog::error("check 7 FAIL: xy_covariance at k={} is not SPD "
                          "([{:.4f} {:.4f}; {:.4f} {:.4f}])",
                          k, C(0, 0), C(0, 1), C(1, 0), C(1, 1));
            ok = false;
            break;
        }
        // accuracy == sqrt of the larger eigenvalue, re-derived in closed form
        // from the 2x2 characteristic polynomial (independent of Eigen's
        // solver, which the implementation uses).
        const double tr   = C(0, 0) + C(1, 1);
        const double det  = C.determinant();
        const double disc = std::sqrt(std::max(0.0, tr * tr - 4.0 * det));
        const double expect = std::sqrt(0.5 * (tr + disc));
        if (!std::isfinite(acc) || std::abs(acc - expect) > ACCURACY_TOL_M) {
            spdlog::error("check 7 FAIL: accuracy at k={} is {:.9f} m, expected "
                          "{:.9f} m", k, acc, expect);
            ok = false;
            break;
        }
        acc_min = std::min(acc_min, acc);
        acc_max = std::max(acc_max, acc);
    }
    if (!ok) {
        return false;
    }
    spdlog::info("covariance: accuracy over {} KFs in [{:.2f}, {:.2f}] m",
                 NUM_KFS, acc_min, acc_max);

    // The failure contract: no covariance ⇒ NaN, and specifically NOT 0.
    uavloc::fusion::FusionResult none;  // default: covariance_valid == false
    const double acc_none = horizontal_accuracy_m(none);
    if (!std::isnan(acc_none)) {
        spdlog::error("check 7 FAIL: missing covariance reported {:.3f} m "
                      "instead of NaN", acc_none);
        ok = false;
    }
    // A degenerate (non-positive-definite) covariance must not be reported as
    // "perfectly certain" either.
    uavloc::fusion::FusionResult degenerate;
    degenerate.covariance_valid = true;
    degenerate.xy_covariance    = Eigen::Matrix2d::Zero();
    if (!std::isnan(horizontal_accuracy_m(degenerate))) {
        spdlog::error("check 7 FAIL: zero covariance did not report NaN");
        ok = false;
    }
    uavloc::fusion::FusionResult nan_cov;
    nan_cov.covariance_valid = true;
    nan_cov.xy_covariance.setConstant(std::numeric_limits<double>::quiet_NaN());
    if (!std::isnan(horizontal_accuracy_m(nan_cov))) {
        spdlog::error("check 7 FAIL: non-finite covariance did not report NaN");
        ok = false;
    }
    return ok;
}

// ── Check 8: health state machine (S7) ─────────────────────────────────────
//! Pure transition table + hysteresis, driven by synthetic sigmas.
bool check_health_state_machine() {
    using uavloc::fusion::FusionHealth;
    using uavloc::fusion::next_fusion_health;
    bool ok = true;

    auto step = [](FusionHealth s, double sigma, bool valid = true) {
        return next_fusion_health(s, sigma, valid, HEALTH_CONV_SIGMA_M,
                                  HEALTH_DRIFT_SIGMA_M);
    };
    auto expect = [&ok](FusionHealth got, FusionHealth want, const char* what) {
        if (got != want) {
            spdlog::error("check 8 FAIL: {} — got {}, want {}", what,
                          static_cast<int>(got), static_cast<int>(want));
            ok = false;
        }
    };

    // Rising edge: only a sigma BELOW the converged threshold leaves start-up.
    expect(step(FusionHealth::INITIALIZING, HEALTH_HIGH_SIGMA_M),
           FusionHealth::INITIALIZING, "INITIALIZING stays at a high sigma");
    expect(step(FusionHealth::INITIALIZING, HEALTH_BAND_SIGMA_M),
           FusionHealth::INITIALIZING, "INITIALIZING stays inside the band");
    expect(step(FusionHealth::INITIALIZING, HEALTH_LOW_SIGMA_M),
           FusionHealth::CONVERGED, "INITIALIZING → CONVERGED below threshold");

    // Falling edge: only a sigma ABOVE the drifting threshold leaves CONVERGED.
    expect(step(FusionHealth::CONVERGED, HEALTH_BAND_SIGMA_M),
           FusionHealth::CONVERGED, "CONVERGED holds inside the band");
    expect(step(FusionHealth::CONVERGED, HEALTH_HIGH_SIGMA_M),
           FusionHealth::DRIFTING, "CONVERGED → DRIFTING above threshold");

    // Recovery must cross the WHOLE band (this is the hysteresis).
    expect(step(FusionHealth::DRIFTING, HEALTH_BAND_SIGMA_M),
           FusionHealth::DRIFTING, "DRIFTING holds inside the band");
    expect(step(FusionHealth::DRIFTING, HEALTH_LOW_SIGMA_M),
           FusionHealth::CONVERGED, "DRIFTING → CONVERGED below threshold");

    // No covariance / NaN sigma ⇒ back to INITIALIZING from ANY state.
    for (auto s : {FusionHealth::INITIALIZING, FusionHealth::CONVERGED,
                   FusionHealth::DRIFTING}) {
        expect(step(s, HEALTH_LOW_SIGMA_M, /*valid=*/false),
               FusionHealth::INITIALIZING, "invalid covariance → INITIALIZING");
        expect(step(s, std::numeric_limits<double>::quiet_NaN()),
               FusionHealth::INITIALIZING, "NaN sigma → INITIALIZING");
    }

    // No chatter: a sigma sweeping the dead zone leaves the state untouched,
    // from both sides of the band.
    for (auto start : {FusionHealth::CONVERGED, FusionHealth::DRIFTING}) {
        FusionHealth s = start;
        for (int i = 0; i < HEALTH_CHATTER_STEPS; ++i) {
            const double frac = static_cast<double>(i % 2);
            const double sigma = HEALTH_CONV_SIGMA_M + 1.0 +
                frac * (HEALTH_DRIFT_SIGMA_M - HEALTH_CONV_SIGMA_M - 2.0);
            s = step(s, sigma);
            if (s != start) {
                spdlog::error("check 8 FAIL: state changed at sigma {:.1f} m "
                              "inside the hysteresis band", sigma);
                ok = false;
                break;
            }
        }
    }
    return ok;
}

} // namespace

int main() {
    spdlog::set_level(spdlog::level::info);
    bool ok = true;

    // ── 0. Conventions ──────────────────────────────────────────────────────
    if (!check_attitude_chain()) {
        return 1;
    }
    spdlog::info("check 0 PASS: attitude-chain conventions");

    if (!check_map_depth_factor()) {
        return 1;
    }
    spdlog::info("check 0b PASS: MapDepthFactor sign/zero/log-relativity");

    if (!check_absolute_xy_factor()) {
        return 1;
    }
    spdlog::info("check 0c PASS: AbsoluteXYFactor residual/Jacobian/robust weight");

    const Eigen::Isometry3d T_enu_w = pose_gt_enu(0);  // world = first camera frame
    const std::vector<Eigen::Isometry3d> T_wc_gt      = build_gt_vo(T_enu_w);
    const std::vector<Eigen::Isometry3d> T_wc_corrupt = build_corrupted_vo(T_wc_gt, T_enu_w);

    // ── 1. Perfect-VO leg: fused output must reproduce GT ENU ──────────────
    {
        const RunOutcome perfect = run_fusion(T_wc_gt, 0.0);
        for (int k = 0; k < NUM_KFS; ++k) {
            const auto& r = perfect.results[static_cast<size_t>(k)];
            const double pos_err = (r.T_enu_c.translation() - position_gt_enu(k)).norm();
            const Eigen::AngleAxisd rot_err(
                r.T_enu_c.linear().transpose() * pose_gt_enu(k).linear());
            if (pos_err > PERFECT_POS_TOL_M ||
                std::abs(rot_err.angle()) * R2D > PERFECT_ROT_TOL_DEG) {
                spdlog::error("check 1 FAIL: perfect VO diverges from GT at k={} "
                              "(pos {:.3f} m, rot {:.3f}°)",
                              k, pos_err, rot_err.angle() * R2D);
                ok = false;
                break;
            }
        }
        if (ok) {
            spdlog::info("check 1 PASS: perfect VO reproduces GT ENU trajectory");
        }
    }

    // ── 2. Raw corrupted VO sanity ──────────────────────────────────────────
    const Eigen::Isometry3d raw_end = T_enu_w * T_wc_corrupt[NUM_KFS - 1];
    const Eigen::Isometry3d gt_end  = pose_gt_enu(NUM_KFS - 1);
    const double raw_yaw_err_deg =
        wrap_deg(yaw_deg_of(raw_end.linear()) - yaw_deg_of(gt_end.linear()));
    const double raw_z_err_m = raw_end.translation().z() - gt_end.translation().z();
    const double raw_horiz_err_m =
        (raw_end.translation() - gt_end.translation()).head<2>().norm();
    spdlog::info("raw corrupted VO endpoint: yaw err {:+.2f}°, Z err {:+.1f} m, "
                 "horizontal err {:.1f} m",
                 raw_yaw_err_deg, raw_z_err_m, raw_horiz_err_m);
    if (std::abs(raw_yaw_err_deg) < RAW_YAW_ERR_MIN_DEG ||
        std::abs(raw_yaw_err_deg) > RAW_YAW_ERR_MAX_DEG ||
        raw_z_err_m < RAW_Z_ERR_MIN_M || raw_z_err_m > RAW_Z_ERR_MAX_M) {
        spdlog::error("check 2 FAIL: corruption outside the expected sanity band");
        ok = false;
    } else {
        spdlog::info("check 2 PASS: corruption sanity");
    }

    // ── 3. Fused drift recovery ─────────────────────────────────────────────
    const RunOutcome fused = run_fusion(T_wc_corrupt, 0.0);
    {
        const auto& r = fused.results.back();
        const double yaw_err_deg =
            wrap_deg(yaw_deg_of(r.T_enu_c.linear()) - yaw_deg_of(gt_end.linear()));
        const double z_err_m = r.T_enu_c.translation().z() - gt_end.translation().z();
        const double horiz_err_m =
            (r.T_enu_c.translation() - gt_end.translation()).head<2>().norm();
        const double z_bound = FUSED_Z_ERR_SIGMAS * make_config().agl_sigma_m;
        spdlog::info("fused endpoint: yaw err {:+.3f}°, Z err {:+.2f} m, "
                     "horizontal err {:.1f} m (scale {:.4f}, bias {:.2f} m)",
                     yaw_err_deg, z_err_m, horiz_err_m, r.scale, r.agl_bias_m);
        if (std::abs(yaw_err_deg) >= FUSED_YAW_ERR_MAX_DEG) {
            spdlog::error("check 3 FAIL: fused yaw err {:.3f}° >= {}°",
                          yaw_err_deg, FUSED_YAW_ERR_MAX_DEG);
            ok = false;
        }
        if (std::abs(z_err_m) >= z_bound) {
            spdlog::error("check 3 FAIL: fused Z err {:.2f} m >= {:.1f} m",
                          z_err_m, z_bound);
            ok = false;
        }
        if (horiz_err_m >= FUSED_HORIZ_FRACTION * raw_horiz_err_m) {
            spdlog::error("check 3 FAIL: fused horizontal err {:.1f} m >= {:.1f} m "
                          "(half the raw error)",
                          horiz_err_m, FUSED_HORIZ_FRACTION * raw_horiz_err_m);
            ok = false;
        }
        if (ok) {
            spdlog::info("check 3 PASS: drift recovery");
        }
    }

    // ── 4. Heading-offset immunity ──────────────────────────────────────────
    // A constant +δ on every heading only changes the absolute-yaw gauge (set
    // by the X(0) anchor rotation): the fused trajectory rotates rigidly by
    // Rz(-δ) about the anchor vertical (heading is CW-from-North, ENU yaw is
    // CCW). The delta-yaw factors are untouched (deltas cancel the offset), so
    // after removing the known rigid rotation the runs must coincide.
    {
        const RunOutcome offset = run_fusion(T_wc_corrupt, HEADING_OFFSET_DEG);
        const Eigen::Matrix3d R_undo =
            Eigen::AngleAxisd(HEADING_OFFSET_DEG * D2R, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        double sum_sq = 0.0;
        for (int k = 0; k < NUM_KFS; ++k) {
            const Eigen::Vector3d p_base =
                fused.results[static_cast<size_t>(k)].T_enu_c.translation();
            const Eigen::Vector3d p_off =
                offset.results[static_cast<size_t>(k)].T_enu_c.translation();
            sum_sq += (R_undo * p_off - p_base).squaredNorm();
        }
        const double rms = std::sqrt(sum_sq / NUM_KFS);
        spdlog::info("offset-immunity: de-rotated trajectory RMS {:.4f} m", rms);
        if (rms >= OFFSET_RMS_TOL_M) {
            spdlog::error("check 4 FAIL: offset run differs by {:.3f} m RMS >= {} m",
                          rms, OFFSET_RMS_TOL_M);
            ok = false;
        } else {
            spdlog::info("check 4 PASS: heading-offset immunity");
        }
    }

    // ── 5. Determinism ──────────────────────────────────────────────────────
    {
        const RunOutcome rerun = run_fusion(T_wc_corrupt, 0.0);
        bool identical = rerun.results.size() == fused.results.size();
        for (size_t i = 0; identical && i < fused.results.size(); ++i) {
            identical = (fused.results[i].T_enu_c.matrix().array() ==
                         rerun.results[i].T_enu_c.matrix().array()).all() &&
                        fused.results[i].scale == rerun.results[i].scale &&
                        fused.results[i].agl_bias_m == rerun.results[i].agl_bias_m;
        }
        if (!identical) {
            spdlog::error("check 5 FAIL: two identical sync runs are not bit-identical");
            ok = false;
        } else {
            spdlog::info("check 5 PASS: determinism");
        }
    }

    // ── 6. Absolute-fix gate ────────────────────────────────────────────────
    // Perfect VO (fused ≈ GT, check 1), so a fix at GT + offset probes the
    // gate directly: a 1 km outlier must be rejected while a 1σ fix is taken.
    {
        if (!check_absolute_fix_gate(T_wc_gt)) {
            ok = false;
        } else {
            spdlog::info("check 6 PASS: absolute-fix Mahalanobis gate");
        }
        if (!check_fix_time_thresholds(T_wc_gt)) {
            ok = false;
        } else {
            spdlog::info("check 6b PASS: age budget and keyframe-match tolerance "
                         "are independent thresholds with separate counters");
        }
        if (!check_fix_drift_inflation(T_wc_gt)) {
            ok = false;
        } else {
            spdlog::info("check 6c PASS: drift term widens the gate, s tracks the "
                         "flown path and resets on an applied fix");
        }
        if (!check_fix_gate_switch(T_wc_gt)) {
            ok = false;
        } else {
            spdlog::info("check 6d PASS: fix_gate_enabled switches the "
                         "self-consistency gate (off by default ⇒ a large "
                         "residual reaches the graph)");
        }
        if (!check_fix_confidence_floor(T_wc_gt)) {
            ok = false;
        } else {
            spdlog::info("check 6e PASS: producer-side confidence floor drops a "
                         "fix into low_confidence; default 0 is a no-op");
        }
    }

    // ── 7. Marginal covariance → accuracy_m ─────────────────────────────────
    if (!check_covariance_and_accuracy(T_wc_gt)) {
        ok = false;
    } else {
        spdlog::info("check 7 PASS: covariance filled, accuracy = max 1σ semi-axis, "
                     "NaN (never 0) when unavailable");
    }

    // ── 8. Health state machine ─────────────────────────────────────────────
    if (!check_health_state_machine()) {
        ok = false;
    } else {
        spdlog::info("check 8 PASS: health transitions both ways, hysteresis holds");
    }

    if (!ok) {
        spdlog::error("test_fusion_factors: FAIL");
        return 1;
    }
    spdlog::info("test_fusion_factors: PASS");
    return 0;
}
