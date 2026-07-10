#include "uavloc/vo/local_optimizer.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>  // ldlt, inverse
#include <spdlog/spdlog.h>

namespace uavloc::vo {

namespace {

// Lower bound on a point's camera-frame depth (m) for it to contribute. Points
// at/behind the camera have a singular projection Jacobian; skip them. This is a
// numerical guard, not a tunable pipeline threshold, so it stays in source.
constexpr double MIN_DEPTH_M = 1e-3;

// Floor on the per-frame MAD scale (px). When residuals are already tiny the MAD
// can collapse to ~0 and over-weight noise; clamp it. Numerical guard.
constexpr double MIN_SIGMA_PX = 1.0;

// Scale factor converting MAD to a Gaussian-consistent standard deviation
// (sigma ≈ 1.4826 * MAD for normally distributed residuals).
constexpr double MAD_TO_SIGMA = 1.4826;

// se3 skew-symmetric (hat) of a 3-vector.
inline Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d S;
    S <<     0.0, -w.z(),  w.y(),
          w.z(),    0.0, -w.x(),
         -w.y(),  w.x(),    0.0;
    return S;
}

// SE3 exponential map for a 6-vector twist xi = (rotation | translation), i.e.
// the first three components are the rotation part (the convention used in
// §D.1: J = -dpi/dPc * [ -[Pc]x | I ], so dx[0:3] is rotation, dx[3:6] is
// translation). Returns the 4x4 transform exp(xi^).
Eigen::Matrix4d se3Exp(const Eigen::Matrix<double, 6, 1>& xi) {
    const Eigen::Vector3d omega = xi.head<3>();
    const Eigen::Vector3d upsilon = xi.tail<3>();
    const double theta = omega.norm();

    const Eigen::Matrix3d Omega = skew(omega);
    Eigen::Matrix3d R;
    Eigen::Matrix3d V;
    if (theta < 1e-10) {
        // First-order expansion near zero rotation.
        R = Eigen::Matrix3d::Identity() + Omega;
        V = Eigen::Matrix3d::Identity() + 0.5 * Omega;
    } else {
        const double theta2 = theta * theta;
        const Eigen::Matrix3d Omega2 = Omega * Omega;
        const double s = std::sin(theta);
        const double c = std::cos(theta);
        R = Eigen::Matrix3d::Identity() + (s / theta) * Omega +
            ((1.0 - c) / theta2) * Omega2;
        V = Eigen::Matrix3d::Identity() + ((1.0 - c) / theta2) * Omega +
            ((theta - s) / (theta2 * theta)) * Omega2;
    }

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = V * upsilon;
    return T;
}

}  // namespace

// ---------------------------------------------------------------------------
// PoseOptimizerConfig
// ---------------------------------------------------------------------------
PoseOptimizerConfig PoseOptimizerConfig::fromYaml(const YAML::Node& vo_node) {
    PoseOptimizerConfig cfg;  // start from defaults
    if (!vo_node) {
        return cfg;
    }
    cfg.max_iters = vo_node["pose_opt_iterations"].as<int>(cfg.max_iters);
    cfg.huber_px  = vo_node["pose_opt_huber_px"].as<double>(cfg.huber_px);
    cfg.max_reproj_error_px =
        vo_node["pose_opt_max_reproj_error_px"].as<double>(cfg.max_reproj_error_px);
    return cfg;
}

// ---------------------------------------------------------------------------
// PoseOptimizer
// ---------------------------------------------------------------------------
PoseOptimizer::PoseOptimizer(const PoseOptimizerConfig& config,
                             const sensor::CameraModel& camera)
    : config_(config) {
    const Eigen::Matrix3d& K = camera.K();
    fx_ = K(0, 0);
    fy_ = K(1, 1);
    cx_ = K(0, 2);
    cy_ = K(1, 2);
}

int PoseOptimizer::optimize(const std::vector<Eigen::Vector3d>& pts_w,
                            const std::vector<Eigen::Vector2d>& obs_px,
                            Eigen::Matrix4d&                    T_wc_inout,
                            std::vector<bool>&                  inlier_out) {
    const size_t n = pts_w.size();
    inlier_out.assign(n, true);
    if (n != obs_px.size() || n == 0) {
        spdlog::warn("PoseOptimizer: empty/size-mismatched input ({} pts, {} obs)",
                     n, obs_px.size());
        return 0;
    }

    // Work INTERNALLY on T_cw (world->camera): the §D.1 Jacobian and the
    // exp(dx)*T left-update are derived for T_cw. Convert back to T_wc at the end.
    Eigen::Matrix4d T_cw = T_wc_inout.inverse();

    // Mean reprojection error of all points BEFORE refinement (evidence the BA
    // actually reduces residuals — see the matching log at the end).
    auto meanReproj = [&](const Eigen::Matrix4d& T) {
        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        const Eigen::Vector3d t = T.block<3, 1>(0, 3);
        double sum = 0.0;
        int cnt = 0;
        for (size_t i = 0; i < n; ++i) {
            const Eigen::Vector3d Pc = R * pts_w[i] + t;
            if (Pc.z() <= MIN_DEPTH_M) {
                continue;
            }
            const double u = fx_ * Pc.x() / Pc.z() + cx_;
            const double v = fy_ * Pc.y() / Pc.z() + cy_;
            sum += std::hypot(obs_px[i].x() - u, obs_px[i].y() - v);
            ++cnt;
        }
        return cnt > 0 ? sum / cnt : 0.0;
    };
    const double mean_before = meanReproj(T_cw);

    // Per-point active flag for the GN accumulation (separate from the final
    // inlier_out gate). Starts all-true; tightened by the robust kernel + the
    // depth guard inside the loop.
    std::vector<bool> active(n, true);

    const double huber = config_.huber_px;

    for (int iter = 0; iter < config_.max_iters; ++iter) {
        const Eigen::Matrix3d R_cw = T_cw.block<3, 3>(0, 0);
        const Eigen::Vector3d t_cw = T_cw.block<3, 1>(0, 3);

        // --- 1. residual norms (for the per-frame MAD scale) ----------------
        std::vector<double> res_norms;
        res_norms.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const Eigen::Vector3d Pc = R_cw * pts_w[i] + t_cw;
            if (Pc.z() <= MIN_DEPTH_M) {
                continue;
            }
            const double u = fx_ * Pc.x() / Pc.z() + cx_;
            const double v = fy_ * Pc.y() / Pc.z() + cy_;
            const double ex = obs_px[i].x() - u;
            const double ey = obs_px[i].y() - v;
            res_norms.push_back(std::hypot(ex, ey));
        }
        if (res_norms.empty()) {
            // A degenerate PnP pose (all landmarks behind the camera) — the
            // Tracker falls back to the raw pose, so this is recoverable, not a
            // fatal error; keep it at debug to avoid flooding the log.
            spdlog::debug("PoseOptimizer: all points behind camera at iter {} — abort",
                          iter);
            break;
        }

        // MAD scale: median of |r - median(r)|, scaled to a Gaussian sigma.
        std::vector<double> tmp = res_norms;
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        const double med = tmp[tmp.size() / 2];
        std::vector<double> abs_dev;
        abs_dev.reserve(res_norms.size());
        for (double r : res_norms) {
            abs_dev.push_back(std::abs(r - med));
        }
        std::nth_element(abs_dev.begin(), abs_dev.begin() + abs_dev.size() / 2,
                         abs_dev.end());
        const double mad = abs_dev[abs_dev.size() / 2];
        const double sigma = std::max(MAD_TO_SIGMA * mad, MIN_SIGMA_PX);

        // --- 2. accumulate normal equations with Huber weighting ------------
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
        int n_used = 0;
        for (size_t i = 0; i < n; ++i) {
            if (!active[i]) {
                continue;
            }
            const Eigen::Vector3d Pc = R_cw * pts_w[i] + t_cw;
            const double Z = Pc.z();
            if (Z <= MIN_DEPTH_M) {
                continue;
            }
            const double X = Pc.x(), Y = Pc.y();
            const double u = fx_ * X / Z + cx_;
            const double v = fy_ * Y / Z + cy_;

            // residual e = obs - projection (2x1)
            Eigen::Vector2d e(obs_px[i].x() - u, obs_px[i].y() - v);
            const double r = e.norm();

            // dpi/dPc (2x3)
            Eigen::Matrix<double, 2, 3> dpi;
            const double invZ = 1.0 / Z;
            const double invZ2 = invZ * invZ;
            dpi << fx_ * invZ,        0.0, -fx_ * X * invZ2,
                          0.0, fy_ * invZ, -fy_ * Y * invZ2;

            // J = -dpi * [ -[Pc]x | I ]   (2x6, rotation | translation)
            Eigen::Matrix<double, 3, 6> dPc;  // d(Pc)/d(xi)
            dPc.block<3, 3>(0, 0) = -skew(Pc);
            dPc.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
            const Eigen::Matrix<double, 2, 6> J = -dpi * dPc;

            // Huber weight on whitened residual ||e||/sigma.
            const double rw = r / sigma;
            double w = 1.0;
            if (rw > huber) {
                w = huber / rw;
            }

            H.noalias() += w * J.transpose() * J;
            g.noalias() += w * J.transpose() * e;
            ++n_used;
        }

        if (n_used < 3) {
            spdlog::debug("PoseOptimizer: only {} active points at iter {} — stop",
                          n_used, iter);
            break;
        }

        // --- 3. solve H dx = -g  ------------------------------------------
        // Residual e = obs - proj, linearised as e(x+dx) ≈ e + J dx with
        // J = de/dx = -dproj/dx. Minimising ||e + J dx||² gives the normal
        // equations H dx = -g, H = ΣwJᵀJ, g = ΣwJᵀe (§D.1). Update is the
        // left-multiply T_cw <- exp(dx) * T_cw.
        const Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-g);
        if (!dx.allFinite()) {
            spdlog::warn("PoseOptimizer: non-finite update at iter {} — stop", iter);
            break;
        }

        T_cw = se3Exp(dx) * T_cw;

        if (dx.norm() < 1e-9) {
            break;  // converged
        }
    }

    // --- 4. final outlier gate on the refined pose -------------------------
    const Eigen::Matrix3d R_cw = T_cw.block<3, 3>(0, 0);
    const Eigen::Vector3d t_cw = T_cw.block<3, 1>(0, 3);
    int n_inliers = 0;
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d Pc = R_cw * pts_w[i] + t_cw;
        if (Pc.z() <= MIN_DEPTH_M) {
            inlier_out[i] = false;
            continue;
        }
        const double u = fx_ * Pc.x() / Pc.z() + cx_;
        const double v = fy_ * Pc.y() / Pc.z() + cy_;
        const double err = std::hypot(obs_px[i].x() - u, obs_px[i].y() - v);
        if (err <= config_.max_reproj_error_px) {
            inlier_out[i] = true;
            ++n_inliers;
        } else {
            inlier_out[i] = false;
        }
    }

    const double mean_after = meanReproj(T_cw);
    spdlog::debug("PoseOptimizer: mean reproj {:.3f}px -> {:.3f}px, inliers {}/{}",
                  mean_before, mean_after, n_inliers, n);

    T_wc_inout = T_cw.inverse();
    return n_inliers;
}

}  // namespace uavloc::vo
