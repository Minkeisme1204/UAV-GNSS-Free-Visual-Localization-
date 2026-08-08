#pragma once

// Implementation-only GTSAM factors + telemetry attitude chain for the fusion
// module (Stage F1). Private header (src/fusion/) — never installed; gtsam
// must not leak into the public uavloc API.
//
// Formulations follow .docs/theory/heading_agl_prior_tactics.md §5.3/§6 and
// the kcb_slam ScaledVOFactor precedent
// (.docs/related_work/kcb_slam/05_khoi_fusion.md §3.1):
// pure error functions + gtsam::numericalDerivative Jacobians.

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace uavloc::fusion {

//! Step for gtsam::numericalDerivative (kcb precedent). A solver-internal
//! discretization constant, not a mission tunable — deliberately not config.
constexpr double NUMERICAL_JACOBIAN_STEP = 1e-5;

//! Wrap an angle to (-pi, pi].
inline double wrap_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

// ── Telemetry attitude chain ─────────────────────────────────────────────────
//
// Frame conventions (SINGLE source of truth — the X(0) anchor, the per-KF
// roll/pitch attitude prior and the VO-world→ENU init all use this helper;
// verified against known synthetic cases in tests/test_fusion_factors.cpp):
//
//   ENU    : x = East, y = North, z = Up (fusion state frame).
//   NED    : x = North, y = East, z = Down (aviation attitude reference).
//   body   : x = forward, y = right, z = down (NED-style airframe axes).
//            R_ned_body = Rz(heading) * Ry(pitch) * Rx(roll)
//            heading: 0 = North, clockwise positive (telemetry convention;
//            rotation about NED z = down is clockwise seen from above);
//            pitch: nose-up positive; roll: right-wing-down positive.
//            Known case: heading = 90°, level → body-forward maps to +x East.
//   camera : z = optical axis (line of sight), x = image right, y = image down.
//
// Gimbal chain: pan rotates about body z (down) — positive pan turns the view
// clockwise seen from above, i.e. it adds to the airframe heading. Tilt is
// measured from the horizontal plane, 90° = nadir; tilting the line of sight
// down from body-forward toward body-down is a rotation about body y by -tilt.
// With pan = tilt = 0 the camera looks forward along body x.

//! Camera→body axis alignment at pan = 0, tilt = 0 (camera looking forward
//! along body x): camera x (image right) = body y (right), camera y (image
//! down) = body z (down), camera z (optical axis) = body x (forward).
//! Columns are the camera axes expressed in body coordinates. Fixed by the
//! mount/axes definition above — a convention, not a config value.
inline const Eigen::Matrix3d& r_cam_alignment() {
    static const Eigen::Matrix3d R_CAM_ALIGNMENT =
        (Eigen::Matrix3d() << 0.0, 0.0, 1.0,
                              1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0).finished();
    return R_CAM_ALIGNMENT;
}

//! NED → ENU axis swap (involutive: it is its own inverse).
inline const Eigen::Matrix3d& c_enu_ned() {
    static const Eigen::Matrix3d C_ENU_NED =
        (Eigen::Matrix3d() << 0.0, 1.0,  0.0,
                              1.0, 0.0,  0.0,
                              0.0, 0.0, -1.0).finished();
    return C_ENU_NED;
}

//! Measured camera orientation in ENU from telemetry attitude + gimbal angles
//! (all inputs in degrees):
//!   R_enu_cam = C_enu_ned * R_ned_body(heading, pitch, roll)
//!             * Rz(pan) * Ry(-tilt) * R_CAM_ALIGNMENT
inline Eigen::Matrix3d rotation_enu_camera(double roll_deg, double pitch_deg,
                                           double heading_deg,
                                           double gimbal_pan_deg,
                                           double gimbal_tilt_deg) {
    constexpr double D2R = M_PI / 180.0;
    const Eigen::Matrix3d R_ned_body =
        (Eigen::AngleAxisd(heading_deg * D2R, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll_deg * D2R, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Matrix3d R_body_cam =
        (Eigen::AngleAxisd(gimbal_pan_deg * D2R, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-gimbal_tilt_deg * D2R, Eigen::Vector3d::UnitY()))
            .toRotationMatrix() *
        r_cam_alignment();
    return c_enu_ned() * R_ned_body * R_body_cam;
}

//! Yaw of an ENU camera pose: azimuth (CCW from East) of the camera x-axis
//! (image right) projected onto the E-N plane. Any fixed axis choice works
//! for the delta-yaw factor — constant offsets cancel in the delta — but the
//! camera x-axis is chosen because it stays near-horizontal for a near-nadir
//! camera (the optical z-axis is near-vertical there, so its E-N projection
//! would be degenerate). NOTE the sign relation to telemetry heading:
//! heading is clockwise-from-North, this yaw is CCW-from-East, hence
//! d(yaw_enu) = -d(heading).
inline double yaw_of_enu_rotation(const gtsam::Rot3& R_enu_c) {
    const gtsam::Matrix3 R = R_enu_c.matrix();
    return std::atan2(R(1, 0), R(0, 0));
}

// ── ScaledVOFactor ───────────────────────────────────────────────────────────

//! VO relative-pose constraint with a scale multiplier on the measured
//! translation (kcb_slam formulation). Measurement: the VO relative pose
//! T_rel = T_wc_prevKF^-1 * T_wc_k (rotation R_vo, translation t_vo).
//!   predicted X_k = X_{k-1} ∘ Pose3(R_vo, s * t_vo)
//!   error(6)      = predicted.localCoordinates(X_k)
class ScaledVOFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double> {
public:
    ScaledVOFactor(gtsam::Key pose_i, gtsam::Key pose_j, gtsam::Key scale,
                   const gtsam::Pose3& measured,
                   const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double>(
              model, pose_i, pose_j, scale),
          measured_(measured) {}

    //! Pure error function (separate so numericalDerivative can use it).
    gtsam::Vector error_function(const gtsam::Pose3& p_i, const gtsam::Pose3& p_j,
                                 const double& s) const {
        const gtsam::Pose3 predicted = p_i.compose(
            gtsam::Pose3(measured_.rotation(), measured_.translation() * s));
        return predicted.localCoordinates(p_j);
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& p_i, const gtsam::Pose3& p_j,
                                const double& s,
                                gtsam::Matrix* H1 = nullptr,
                                gtsam::Matrix* H2 = nullptr,
                                gtsam::Matrix* H3 = nullptr) const override {
        const std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&,
                                          const double&)>
            fn = [this](const gtsam::Pose3& a, const gtsam::Pose3& b,
                        const double& c) { return error_function(a, b, c); };
        if (H1) {
            *H1 = gtsam::numericalDerivative31<gtsam::Vector, gtsam::Pose3,
                                               gtsam::Pose3, double>(
                fn, p_i, p_j, s, NUMERICAL_JACOBIAN_STEP);
        }
        if (H2) {
            *H2 = gtsam::numericalDerivative32<gtsam::Vector, gtsam::Pose3,
                                               gtsam::Pose3, double>(
                fn, p_i, p_j, s, NUMERICAL_JACOBIAN_STEP);
        }
        if (H3) {
            *H3 = gtsam::numericalDerivative33<gtsam::Vector, gtsam::Pose3,
                                               gtsam::Pose3, double>(
                fn, p_i, p_j, s, NUMERICAL_JACOBIAN_STEP);
        }
        return error_function(p_i, p_j, s);
    }

private:
    gtsam::Pose3 measured_;
};

// ── DeltaYawFactor ───────────────────────────────────────────────────────────

//! Relative-yaw constraint from telemetry heading (+ gimbal pan) deltas —
//! offset-immune: any constant heading offset (unknown mount azimuth,
//! magnetic declination) cancels exactly in the delta (§5.2). The measured
//! delta is baked at construction, ALREADY converted by the caller to the
//! ENU-yaw sign convention of yaw_of_enu_rotation() — i.e.
//! measured = wrap(-Δ(heading + pan)) in radians, because heading is
//! clockwise-from-North while yaw_enu is CCW-from-East.
//!   error(1) = wrap( yaw(X_k) - yaw(X_{k-1}) - measured )
class DeltaYawFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
public:
    DeltaYawFactor(gtsam::Key pose_i, gtsam::Key pose_j,
                   double measured_delta_yaw_rad,
                   const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, pose_i,
                                                               pose_j),
          measured_delta_yaw_rad_(measured_delta_yaw_rad) {}

    gtsam::Vector error_function(const gtsam::Pose3& p_i,
                                 const gtsam::Pose3& p_j) const {
        const double yaw_i = yaw_of_enu_rotation(p_i.rotation());
        const double yaw_j = yaw_of_enu_rotation(p_j.rotation());
        return gtsam::Vector1(
            wrap_angle(yaw_j - yaw_i - measured_delta_yaw_rad_));
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& p_i, const gtsam::Pose3& p_j,
                                gtsam::Matrix* H1 = nullptr,
                                gtsam::Matrix* H2 = nullptr) const override {
        const std::function<gtsam::Vector(const gtsam::Pose3&,
                                          const gtsam::Pose3&)>
            fn = [this](const gtsam::Pose3& a, const gtsam::Pose3& b) {
                return error_function(a, b);
            };
        if (H1) {
            *H1 = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3,
                                               gtsam::Pose3>(
                fn, p_i, p_j, NUMERICAL_JACOBIAN_STEP);
        }
        if (H2) {
            *H2 = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3,
                                               gtsam::Pose3>(
                fn, p_i, p_j, NUMERICAL_JACOBIAN_STEP);
        }
        return error_function(p_i, p_j);
    }

private:
    double measured_delta_yaw_rad_;
};

// ── AglFactor ────────────────────────────────────────────────────────────────

//! AGL altitude unary on (pose X(k), bias b(k)): the telemetry AGL measures
//! the ENU height up to a slowly-varying terrain/baro bias b(k), so
//!   error(1) = z(X_k) - (agl_k - b_k)
class AglFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, double> {
public:
    AglFactor(gtsam::Key pose, gtsam::Key bias, double measured_agl_m,
              const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, double>(model, pose, bias),
          measured_agl_m_(measured_agl_m) {}

    gtsam::Vector error_function(const gtsam::Pose3& p, const double& b) const {
        return gtsam::Vector1(p.translation().z() - (measured_agl_m_ - b));
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& p, const double& b,
                                gtsam::Matrix* H1 = nullptr,
                                gtsam::Matrix* H2 = nullptr) const override {
        const std::function<gtsam::Vector(const gtsam::Pose3&, const double&)>
            fn = [this](const gtsam::Pose3& a, const double& c) {
                return error_function(a, c);
            };
        if (H1) {
            *H1 = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3,
                                               double>(fn, p, b,
                                                       NUMERICAL_JACOBIAN_STEP);
        }
        if (H2) {
            *H2 = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3,
                                               double>(fn, p, b,
                                                       NUMERICAL_JACOBIAN_STEP);
        }
        return error_function(p, b);
    }

private:
    double measured_agl_m_;
};

// ── AbsoluteXYFactor ─────────────────────────────────────────────────────────

//! Absolute horizontal position unary on X(k) — the back-end's entry point for
//! anchor::AbsoluteFix (M1: fake fixes from groundtruth, later: VPR).
//!   error(2) = [ x(X_k) - meas_x , y(X_k) - meas_y ]
//!
//! Two dimensions ONLY: Z is already measured by AglFactor (stacking a third
//! row here would double-count the altitude evidence and make the two sources
//! impossible to ablate separately), and no rotation row is added — absolute
//! yaw stays unconstrained in M1 by design, so any yaw self-correction seen in
//! the results is genuinely produced by the position measurements alone.
//!
//! gtsam::GPSFactor is deliberately not reused: it constrains all three axes,
//! and neutralising its Z row with a huge sigma would silently overlap
//! AglFactor.
class AbsoluteXYFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    AbsoluteXYFactor(gtsam::Key pose, const Eigen::Vector2d& measured_xy_enu,
                     const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose),
          measured_xy_enu_(measured_xy_enu) {}

    //! Pure error function (separate so numericalDerivative can use it).
    gtsam::Vector error_function(const gtsam::Pose3& p) const {
        return gtsam::Vector2(p.translation().x() - measured_xy_enu_.x(),
                              p.translation().y() - measured_xy_enu_.y());
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& p,
                                gtsam::Matrix* H1 = nullptr) const override {
        const std::function<gtsam::Vector(const gtsam::Pose3&)> fn =
            [this](const gtsam::Pose3& a) { return error_function(a); };
        if (H1) {
            *H1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                fn, p, NUMERICAL_JACOBIAN_STEP);
        }
        return error_function(p);
    }

    const Eigen::Vector2d& measured() const { return measured_xy_enu_; }

private:
    Eigen::Vector2d measured_xy_enu_;  //!< (East, North) [m]
};

// ── MapDepthFactor ───────────────────────────────────────────────────────────

//! Map-scale unary on s(k): for a near-nadir camera over ground the median
//! optical-axis depth of the reference keyframe's landmarks and the telemetry
//! AGL are THE SAME physical distance expressed in two different units —
//!   s(k) * d_vo(k) * cos(beta) = AGL(k)
//! with d_vo in MAP UNITS (vo::VOResult::median_map_depth) and beta the
//! off-nadir angle (cos beta = 1 for a nadir view).
//!
//! This is the SECOND factor touching s(k). ScaledVOFactor alone is invariant
//! under s -> alpha*s with the pose chain rebuilt, so the map-scale drift
//! (measured 0.295 %/keyframe) is invisible to the graph; this factor breaks
//! that invariance with an absolute, per-keyframe measurement of the map unit.
//!
//! The residual is taken in LOG space so the noise model is RELATIVE: the
//! measured spread of d_vo/AGL is ~8.5 % of the value, i.e. 1.7 m at 20 m AGL
//! and 7.2 m at 85 m — one dimensionless sigma is then correct at every
//! altitude.
//!   error(1) = log(s * d_vo * cos beta) - log(agl)
//! The map shrinks relative to metric, so d_vo < agl and the residual is
//! negative at s = 1: the factor pushes s UP, as intended.
//!
//! Deliberately NO bias state: a free bias absorbs exactly the evidence this
//! factor exists to deliver (that is how agl_bias neutralised AglFactor).
//!
//! Degenerate inputs (non-positive d_vo, cos beta or agl) would make log()
//! undefined; the error is then defined to be ZERO (the factor becomes inert
//! rather than poisoning the graph with a NaN). The caller gates on positive
//! depth and a minimum AGL, so this never triggers in practice.
class MapDepthFactor : public gtsam::NoiseModelFactor1<double> {
public:
    MapDepthFactor(gtsam::Key scale, double map_depth, double measured_agl_m,
                   double cos_off_nadir, const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor1<double>(model, scale),
          map_depth_(map_depth),
          measured_agl_m_(measured_agl_m),
          cos_off_nadir_(cos_off_nadir) {}

    gtsam::Vector error_function(const double& s) const {
        const double predicted_agl = s * map_depth_ * cos_off_nadir_;
        if (!(predicted_agl > 0.0) || !(measured_agl_m_ > 0.0)) {
            return gtsam::Vector1(0.0);  // inert on degenerate input
        }
        return gtsam::Vector1(std::log(predicted_agl) -
                              std::log(measured_agl_m_));
    }

    gtsam::Vector evaluateError(const double& s,
                                gtsam::Matrix* H1 = nullptr) const override {
        const std::function<gtsam::Vector(const double&)> fn =
            [this](const double& a) { return error_function(a); };
        if (H1) {
            *H1 = gtsam::numericalDerivative11<gtsam::Vector, double>(
                fn, s, NUMERICAL_JACOBIAN_STEP);
        }
        return error_function(s);
    }

private:
    double map_depth_;      //!< d_vo — median map depth [map units]
    double measured_agl_m_; //!< AGL [m]
    double cos_off_nadir_;  //!< cos(beta)
};

} // namespace uavloc::fusion
