#ifndef PROJECTION_MATCHER_H
#define PROJECTION_MATCHER_H

#include "uavloc/vo/vo_data.h"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <yaml-cpp/yaml.h>

namespace uavloc::vo {

// Tunable parameters for ProjectionMatcher. All fields have defaults so the
// matcher can be constructed with ProjectionMatcherConfig{} and overridden from
// YAML (key "ProjectionMatcher:" nested under the mission "VO:" block). No magic
// numbers should appear in the matching logic.
struct ProjectionMatcherConfig {
    float nn_ratio               = 0.75f;  // Lowe ratio threshold (d1/d2 < nn_ratio)
    int   max_hamming_distance   = 64;     // hard cap on best-match Hamming distance (0..256)
    int   min_matches            = 30;     // below this -> NOT_ENOUGH_MATCHES
    float search_radius_px       = 30.0f;  // guided mode: window radius around projected point
    bool  use_orientation_filter = true;
    int   orientation_hist_bins  = 30;     // 360deg / bins
    int   orientation_top_bins   = 3;      // keep matches in the N most-populated bins
    bool  use_gms                = false;  // optional GMS filter (needs many features)
    float gms_threshold_factor   = 6.0f;   // GMS support threshold scale
    bool  use_cuda               = false;  // use cv::cuda BF matcher when available

    // Build a config from a YAML node; missing keys fall back to defaults.
    static ProjectionMatcherConfig fromYaml(const YAML::Node& node);
};

// Produces 2D-2D point correspondences between two temporally adjacent frames
// (ref = previous frame, curr = current frame) for the downstream
// MotionEstimator. Stateless: the caller supplies both FeatureSets.
//
// Geometric verification (RANSAC / homography / essential matrix) is owned by
// MotionEstimator; this class leaves inliers / inlier_mask / num_inliers /
// inlier_ratio at their defaults.
class ProjectionMatcher {
public:
    explicit ProjectionMatcher(const ProjectionMatcherConfig& config);

    // Match two temporally adjacent frames. The result is written into `out`;
    // the returned MatchStatus mirrors the outcome for convenient call-site
    // branching (same pattern as IFeatureDetector::detect).
    //
    // H_ref_curr_prior (optional): homography with p_curr ~= H * p_ref, e.g. fed
    // back from the previous MotionEstimator step. When nullptr, an identity
    // prior is assumed (small inter-frame motion in nadir video).
    //
    // Pipeline: windowed search-by-projection using the prior -> ratio /
    // orientation / optional GMS filters. If fewer than min_matches survive, the
    // matcher automatically retries with a global brute-force kNN + ratio match
    // (covers the first pair / fast motion).
    MatchStatus match(const FeatureSet& ref,
                      const FeatureSet& curr,
                      MatchesData& out,
                      const Eigen::Matrix3d* H_ref_curr_prior = nullptr);

    const ProjectionMatcherConfig& config() const;

private:
    ProjectionMatcherConfig config_;
    cv::Ptr<cv::BFMatcher>  matcher_;   // NORM_HAMMING
};

}  // namespace uavloc::vo

#endif  // PROJECTION_MATCHER_H
