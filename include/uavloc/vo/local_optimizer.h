#pragma once

// PoseOptimizer — Chặng 1C: motion-only bundle adjustment.
//
// Refines a single camera pose (6-DoF) against a set of FIXED 3D landmarks by
// minimising the robust (Huber) reprojection error with Gauss-Newton. This is
// the classic ORB-SLAM `Optimizer::PoseOptimization` / SVO `PoseOptimizer`
// step: landmarks are held constant, only the pose moves.
//
// Self-contained (Eigen only, no g2o/ceres) so it stays light on Jetson.
//
// SE3 convention (CLAUDE.md): the codebase stores T_wc (camera->world). The
// textbook reprojection Jacobian and the exp(dx)*T left-update are derived for
// T_cw (world->camera). This optimizer therefore works INTERNALLY on
// T_cw = T_wc.inverse() and only converts back to T_wc at the boundary — see
// .docs/theory/svo/03_svo_vs_uavloc_assessment.md §D.1 (convention caveat).

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "uavloc/sensor/camera_model.h"

namespace uavloc::vo {

// Tunable parameters for the motion-only BA. All fields default so the struct
// can be built with PoseOptimizerConfig{} and overridden from the mission
// "VO:" YAML block. No magic numbers belong in the implementation.
struct PoseOptimizerConfig {
    // Gauss-Newton iteration cap. YAML: VO.pose_opt_iterations.
    int max_iters = 10;

    // Huber kernel threshold (px) on the whitened reprojection residual.
    // YAML: VO.pose_opt_huber_px.
    double huber_px = 2.0;

    // Final outlier gate (px): after convergence a point whose reprojection
    // error exceeds this is flagged as an outlier in inlier_out.
    // YAML: VO.pose_opt_max_reproj_error_px (reused).
    double max_reproj_error_px = 5.99;

    // Build from the mission "VO:" node; missing keys fall back to defaults.
    static PoseOptimizerConfig fromYaml(const YAML::Node& vo_node);
};

// Motion-only bundle adjustment. Holds the landmarks fixed and refines the pose.
class PoseOptimizer {
public:
    PoseOptimizer(const PoseOptimizerConfig& config,
                  const sensor::CameraModel& camera);

    // Refine T_wc_inout (camera->world) against fixed 3D-2D correspondences.
    //  - pts_w     : landmark positions in the world frame (metres)
    //  - obs_px    : matching observed pixel coordinates (undistorted)
    //  - T_wc_inout: in = initial pose (e.g. from PnP); out = refined pose
    //  - inlier_out: resized to pts_w.size(); true for points kept as inliers
    //                (reprojection error <= max_reproj_error_px after refinement)
    // Returns the number of inliers. On invalid/degenerate input the pose is
    // left unchanged and 0 is returned.
    int optimize(const std::vector<Eigen::Vector3d>& pts_w,
                 const std::vector<Eigen::Vector2d>& obs_px,
                 Eigen::Matrix4d&                    T_wc_inout,
                 std::vector<bool>&                  inlier_out);

    const PoseOptimizerConfig& config() const { return config_; }

private:
    PoseOptimizerConfig config_;
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
};

}  // namespace uavloc::vo
