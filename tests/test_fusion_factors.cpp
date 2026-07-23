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
//   5. two identical sync runs are bit-identical (determinism).
//
// Headless; exits non-zero on failure.

#include "fusion/factors.h"  // private src/fusion header: attitude chain + yaw helper

#include <gtsam/inference/Symbol.h>

#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/fusion/fusion_module.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
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

    if (!ok) {
        spdlog::error("test_fusion_factors: FAIL");
        return 1;
    }
    spdlog::info("test_fusion_factors: PASS");
    return 0;
}
