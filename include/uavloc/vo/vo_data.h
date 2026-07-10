#ifndef VO_DATA_H
#define VO_DATA_H

#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Core>

namespace uavloc::vo {

enum class VOTrackingState {
    NOT_INITIALIZED,
    INITIALIZED,
    TRACKING,
    LOST, 
    FAILED
};

enum class VOFailureReason {
    NONE, 
    NOT_ENOUGH_FEATURES,
    NOT_ENOUGH_MATCHES, 
    NOT_ENOUGH_INLIERS, 
    POSE_ESTIMATION_FAILED,
};

// Descriptor encoding carried by a FeatureSet. NONE marks a keypoint-only
// (FAST) result with no descriptor payload.
enum class DescriptorType {
    NONE,
    ORB        // 32-byte binary (extend later: BRISK, etc.)
};

// Outcome of a single feature-detection call.
enum class FeatureStatus {
    OK,
    NOT_ENOUGH_FEATURES,
    EMPTY_IMAGE,
    ERROR
};

// Outcome of a single ProjectionMatcher::match call.
enum class MatchStatus {
    OK,
    EMPTY_INPUT,        // a FeatureSet has no keypoints
    NO_DESCRIPTORS,     // descriptor_type != ORB or empty descriptors
    NOT_ENOUGH_MATCHES  // surviving matches < min_matches
};

// Provenance of a FeatureSet, copied from the source frame so the result is
// self-describing once it leaves the detector.
struct FeatureFrameMeta {
    uint64_t frame_id       = 0;
    double   timestamp_msec = 0.0;
};

struct FeatureSet {
    FeatureFrameMeta meta;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    DescriptorType descriptor_type = DescriptorType::NONE;
    FeatureStatus  status = FeatureStatus::OK;
    double processing_time_ms = 0.0;

    // True only when a non-empty descriptor matrix is present and tagged.
    bool hasDescriptors() const {
        return !descriptors.empty() && descriptor_type != DescriptorType::NONE;
    }

    // Resets keypoints, descriptors, meta, type and status to defaults.
    void clear();
};

struct MatchesData {
    uint64_t ref_frame_id = 0; 
    uint64_t curr_frame_id = 0;

    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> inliers;

    std::vector<uint8_t> inlier_mask;

    int num_matches = 0;
    int num_inliers = 0;
    double inlier_ratio = 0.0;
};

struct VOPoseData {
    Eigen::Matrix3d R_L_C = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_L_C = Eigen::Vector3d::Zero();
    Eigen::Matrix4d T_prev_curr = Eigen::Matrix4d::Identity();
};



}

#endif 