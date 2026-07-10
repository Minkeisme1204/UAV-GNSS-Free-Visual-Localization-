#pragma once

#include "uavloc/sensor/camera_model.h"
#include "uavloc/vo/vo_data.h"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace uavloc::vo {

// Outcome of a single PoseEstimator::estimate call. Mirrors the status-enum
// pattern used by IFeatureDetector::detect and ProjectionMatcher::match so call
// sites can branch on the return value instead of inspecting the output struct.
enum class PoseStatus {
    OK,
    NOT_ENOUGH_MATCHES,    // matches.num_matches < min_matches_for_homography
    HOMOGRAPHY_FAILED,     // cv::findHomography returned an empty / non-finite H
    DECOMPOSITION_FAILED,  // no physically valid (R, t, n) among the decompositions
    NOT_ENOUGH_INLIERS,    // RANSAC inliers < min_inliers, or ratio < min_inlier_ratio
    LOW_PARALLAX,          // median inlier disparity < min_parallax_px (motion too small)
    ERROR                  // bad input (invalid intrinsics, size mismatch, ...)
};

// Tunable parameters for PoseEstimator. All fields have defaults so the estimator
// can be built with PoseEstimatorConfig{} and overridden from YAML (key
// "PoseEstimator:" nested under the mission "VO:" block). No magic numbers should
// appear in the estimation logic.
struct PoseEstimatorConfig {
    // --- Homography RANSAC (cv::findHomography, method = cv::RANSAC) ------------
    double ransac_reproj_threshold_px = 3.0;    // max reprojection error for an inlier
    double ransac_confidence          = 0.995;  // RANSAC confidence level
    int    ransac_max_iters           = 2000;   // RANSAC iteration cap

    // --- Acceptance gates ------------------------------------------------------
    int    min_matches_for_homography = 20;     // findHomography needs >= 4; RANSAC wants more
    int    min_inliers                = 30;     // below this -> NOT_ENOUGH_INLIERS
    double min_inlier_ratio           = 0.5;    // inliers / matches gate

    // --- Nadir-aware homography decomposition ----------------------------------
    // The UAV camera looks straight down, so the ground-plane normal expressed in
    // the camera frame is close to the optical axis (0, 0, 1). cv::decomposeHomographyMat
    // returns up to four (R, t, n) candidates; the selector keeps the one whose
    // normal lies within nadir_normal_max_angle_deg of the expected nadir
    // direction and whose triangulated points stay in front of both cameras.
    double nadir_normal_max_angle_deg = 35.0;

    // --- Parallax gate ---------------------------------------------------------
    // Two-view pose recovery (homography decomposition) is ill-conditioned when
    // the camera barely moves between frames: the median pixel disparity of the
    // inlier correspondences collapses and the recovered translation direction
    // becomes meaningless. When the median inlier disparity is below this many
    // pixels the estimator returns LOW_PARALLAX and leaves out_pose unchanged so
    // the caller can hold the previous pose instead of chaining a noisy step.
    double min_parallax_px = 2.0;

    // --- Optional essential-matrix fallback ------------------------------------
    // findHomography is preferred for the planar nadir ground plane. When the
    // homography is ill-conditioned (significant parallax over non-planar
    // terrain), optionally fall back to cv::findEssentialMat + recoverPose.
    // Leave false to keep the estimator homography-only.
    bool   use_essential_fallback     = true;

    // Build a config from a YAML node; missing keys fall back to defaults.
    static PoseEstimatorConfig fromYaml(const YAML::Node& node);
};

// Estimates the relative camera pose between two temporally adjacent frames from
// the 2D-2D correspondences produced by ProjectionMatcher.
//
// Homography-first: fits a ground-plane homography with cv::findHomography
// (RANSAC) over the matched keypoints, then recovers (R, t, n) via
// cv::decomposeHomographyMat using a nadir-aware solution selector. An optional
// essential-matrix path covers the non-planar / high-parallax case.
//
// Monocular note: the recovered translation t_L_C is up-to-scale (unit-norm
// direction). Metric scale must be supplied downstream from telemetry altitude
// or the local map; this class does not invent a scale.
//
// Stateless across calls. Geometric inliers are written back into `matches`
// (inliers / inlier_mask / num_inliers / inlier_ratio), which ProjectionMatcher
// leaves at their defaults.
class PoseEstimator {
public:
    // `camera` supplies the intrinsics K used for findHomography / decomposition.
    // Keypoint coordinates are assumed already undistorted (Preprocessor stage).
    PoseEstimator(const PoseEstimatorConfig& config,
                  const sensor::CameraModel& camera);

    // Estimate the relative pose ref(prev) -> curr.
    //  - ref, curr     : the matched FeatureSets; keypoints supply pixel coords
    //  - matches       : in  = matches;
    //                    out = inliers / inlier_mask / num_inliers / inlier_ratio
    //  - out_pose      : R_L_C, t_L_C and T_prev_curr on success; unchanged on failure
    //  - metric_scale  : ground-plane distance (slant range) in metres supplied by
    //                    the caller, typically altitude_m / cos(off_nadir) from
    //                    telemetry. The homography decomposition returns t as
    //                    baseline/d_plane, so the metric baseline is t * d_plane;
    //                    when metric_scale > 0 the output translation is in metres,
    //                    when <= 0 it stays unit-norm (monocular up-to-scale).
    // Returns a PoseStatus mirroring the outcome for convenient call-site branching.
    PoseStatus estimate(const FeatureSet& ref,
                        const FeatureSet& curr,
                        MatchesData& matches,
                        VOPoseData& out_pose,
                        double metric_scale = -1.0);

    // The configuration this estimator was built with.
    const PoseEstimatorConfig& config() const;

private:
    PoseEstimatorConfig config_;
    cv::Matx33d         K_;  // intrinsics in OpenCV form for findHomography / decompose
};

}  // namespace uavloc::vo
