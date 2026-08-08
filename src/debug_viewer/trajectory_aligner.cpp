// trajectory_aligner.cpp — the 4-DoF VO-world -> groundtruth-ENU fit.
//
// Moved VERBATIM out of tests/test_vo_viewer.cpp: the arithmetic, its order and
// its types are reproduced exactly, because the driver's pose dump is a
// byte-for-byte regression gate. Do not "simplify" an expression here.

#include "uavloc/debug_viewer/trajectory_aligner.h"

#include "gps_to_enu.h"

#include <spdlog/spdlog.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>

namespace uavloc::debug_viewer {
namespace {

// Radians -> degrees, for the diagnostic tilt/yaw logs.
constexpr double DEG_PER_RAD = 180.0 / M_PI;

// Minimum points for a determined plane fit (a plane has 3 DoF).
constexpr std::size_t MIN_PLANE_POINTS = 3;

} // namespace

TrajectoryAligner::TrajectoryAligner(const Config& cfg) : config_(cfg) {}

// ── Groundtruth side ────────────────────────────────────────────────────────

void TrajectoryAligner::setEnuOrigin(double lat0, double lon0, double alt0) {
    if (has_origin_) {
        return;  // first call wins: re-anchoring would move every past point
    }
    lat0_       = lat0;
    lon0_       = lon0;
    alt0_       = alt0;
    has_origin_ = true;
}

Eigen::Vector3d TrajectoryAligner::gpsToEnu(double lat, double lon,
                                            double alt) const {
    if (!has_origin_) {
        return Eigen::Vector3d::Zero();
    }
    const ENUPoint e = gps_to_enu(lat, lon, alt, lat0_, lon0_, alt0_);
    return Eigen::Vector3d(e.e, e.n, e.u);
}

// ── Fitting ─────────────────────────────────────────────────────────────────

void TrajectoryAligner::addCorrespondence(const Eigen::Vector3d& src_vo,
                                          const Eigen::Vector3d& dst_enu) {
    if (frozen_) {
        return;
    }
    src_.push_back(src_vo);
    dst_.push_back(dst_enu);
}

void TrajectoryAligner::setPlanePoints(
    const std::vector<Eigen::Vector3d>& pts_vo) {
    plane_pts_ = pts_vo;
}

bool TrajectoryAligner::readyToFreeze() const {
    return static_cast<int>(src_.size()) >= config_.window_frames &&
           (config_.min_spread_m <= 0.0 ||
            horizontalSpreadM(dst_) >= config_.min_spread_m);
}

bool TrajectoryAligner::tryFreeze() {
    if (frozen_ || !readyToFreeze()) {
        return false;
    }

    const std::size_t n_corr = src_.size();
    Eigen::Matrix3Xd S(3, n_corr);
    Eigen::Matrix3Xd D(3, n_corr);
    for (std::size_t i = 0; i < n_corr; ++i) {
        S.col(static_cast<Eigen::Index>(i)) = src_[i];
        D.col(static_cast<Eigen::Index>(i)) = dst_[i];
    }
    // Diagnostic reference ONLY: the previous free-3D-rotation Umeyama (scale
    // fixed = 1). On this near-collinear camera window its roll about the
    // flight axis is degenerate — it is computed solely for the before/after
    // tilt log below (and used as the fallback when no plane can be fitted).
    const Eigen::Matrix4d T_umeyama3d =
        Eigen::umeyama(S, D, /*with_scaling=*/false);

    // The latest keyframe map-point cloud (VO world, metres) — the landmark
    // carpet defines the ground plane.
    const std::vector<Eigen::Vector3d>& plane_pts = plane_pts_;

    Eigen::Vector3d plane_c, plane_n;
    if (fitPlane(plane_pts, plane_c, plane_n)) {
        // 1) VO-side "up": landmark-plane normal, signed from the plane
        //    centroid TOWARD the mean camera position of the window (the
        //    camera flies above the ground).
        Eigen::Vector3d cam_mean = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d& s : src_) {
            cam_mean += s;
        }
        cam_mean /= static_cast<double>(n_corr);
        Eigen::Vector3d up_vo = plane_n;
        if (up_vo.dot(cam_mean - plane_c) < 0.0) {
            up_vo = -up_vo;
        }

        // 2) R0: rotate up_vo onto ENU Up — kills roll/pitch.
        const Eigen::Matrix3d R0 =
            Eigen::Quaterniond::FromTwoVectors(up_vo, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();

        // 3) Yaw about Up + translation: 2D Procrustes (no scale) on the
        //    horizontal components of the R0-levelled window. With
        //    C = sum(d_c * s'_c^T) the optimal yaw is
        //    atan2(C10 - C01, C00 + C11).
        Eigen::Vector3d s_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d d_mean = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d> s_lev(n_corr);
        for (std::size_t i = 0; i < n_corr; ++i) {
            s_lev[i] = R0 * src_[i];
            s_mean += s_lev[i];
            d_mean += dst_[i];
        }
        s_mean /= static_cast<double>(n_corr);
        d_mean /= static_cast<double>(n_corr);
        Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
        for (std::size_t i = 0; i < n_corr; ++i) {
            const Eigen::Vector2d sc = (s_lev[i] - s_mean).head<2>();
            const Eigen::Vector2d dc = (dst_[i] - d_mean).head<2>();
            C += dc * sc.transpose();
        }
        const double yaw = std::atan2(C(1, 0) - C(0, 1), C(0, 0) + C(1, 1));
        const Eigen::Matrix3d R_yaw =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // 4) T_align = [R_yaw*R0 | t]; t from the centroids (E,N via the 2D
        //    fit; U = mean altitude offset — yaw does not change Up).
        T_align_.setIdentity();
        T_align_.block<3, 3>(0, 0) = R_yaw * R0;
        T_align_.block<3, 1>(0, 3) = d_mean - R_yaw * s_mean;

        // 5) Quantitative tilt check (headless-verifiable): re-fit the plane on
        //    the ALIGNED landmarks.
        spdlog::info("alignment: landmark-plane tilt after "
                     "align = {:.2f} deg (4-DoF gravity-aligned, "
                     "yaw {:.2f} deg, {} plane points)",
                     alignedPlaneTiltDeg(plane_pts, T_align_),
                     yaw * DEG_PER_RAD, plane_pts.size());
        spdlog::info("alignment: OLD free-3D umeyama would give "
                     "landmark-plane tilt = {:.2f} deg "
                     "(degenerate roll, for comparison only)",
                     alignedPlaneTiltDeg(plane_pts, T_umeyama3d));
    } else {
        spdlog::warn("alignment: no landmark snapshot to fit "
                     "the ground plane ({} pts) — falling back "
                     "to free 3D umeyama (roll unconstrained)",
                     plane_pts.size());
        T_align_ = T_umeyama3d;
    }
    frozen_ = true;
    return true;
}

// ── Applying ────────────────────────────────────────────────────────────────

Eigen::Vector3d TrajectoryAligner::applyPoint(
    const Eigen::Vector3d& p_vo) const {
    return (T_align_ * p_vo.homogeneous()).hnormalized();
}

Eigen::Matrix3d TrajectoryAligner::applyRotation(
    const Eigen::Matrix3d& R_vo) const {
    return T_align_.block<3, 3>(0, 0) * R_vo;
}

// ── Diagnostics ─────────────────────────────────────────────────────────────

bool TrajectoryAligner::fitPlane(const std::vector<Eigen::Vector3d>& pts,
                                 Eigen::Vector3d& centroid,
                                 Eigen::Vector3d& normal) {
    if (pts.size() < MIN_PLANE_POINTS) {
        return false;
    }
    centroid = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& p : pts) {
        centroid += p;
    }
    centroid /= static_cast<double>(pts.size());
    Eigen::Matrix3Xd M(3, static_cast<Eigen::Index>(pts.size()));
    for (std::size_t i = 0; i < pts.size(); ++i) {
        M.col(static_cast<Eigen::Index>(i)) = pts[i] - centroid;
    }
    Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(M, Eigen::ComputeFullU);
    normal = svd.matrixU().col(2).normalized();
    return true;
}

double TrajectoryAligner::alignedPlaneTiltDeg(
    const std::vector<Eigen::Vector3d>& pts, const Eigen::Matrix4d& T) {
    std::vector<Eigen::Vector3d> transformed;
    transformed.reserve(pts.size());
    for (const Eigen::Vector3d& p : pts) {
        transformed.push_back((T * p.homogeneous()).hnormalized());
    }
    Eigen::Vector3d c, n;
    if (!fitPlane(transformed, c, n)) {
        return -1.0;
    }
    const double cos_ang =
        std::min(1.0, std::abs(n.dot(Eigen::Vector3d::UnitZ())));
    return std::acos(cos_ang) * DEG_PER_RAD;
}

double TrajectoryAligner::horizontalSpreadM(
    const std::vector<Eigen::Vector3d>& pts) {
    if (pts.empty()) {
        return 0.0;
    }
    double min_e = pts.front().x(), max_e = min_e;
    double min_n = pts.front().y(), max_n = min_n;
    for (const Eigen::Vector3d& p : pts) {
        min_e = std::min(min_e, p.x());
        max_e = std::max(max_e, p.x());
        min_n = std::min(min_n, p.y());
        max_n = std::max(max_n, p.y());
    }
    return std::hypot(max_e - min_e, max_n - min_n);
}

} // namespace uavloc::debug_viewer
