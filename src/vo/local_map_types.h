#pragma once

// Internal local-map data types for the VO module: the 3D Landmark, the
// observation link, and the Keyframe. These are implementation details of the
// local map (not a public API consumed by VPR/fusion), so they live under
// src/vo/ rather than include/uavloc/vo/ (see
// .docs/designs/vo_keyframe_landmark_design.md §C.3).
//
// Cross-references between Landmark and Keyframe are by id only (never by raw
// pointer) to avoid shared_ptr cycles and dangling links during culling. The
// LocalMap owns both via shared_ptr.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "uavloc/sensor/telemetry_data.h"
#include "uavloc/vo/vo_data.h"  // FeatureSet

namespace uavloc::vo {

// Lifecycle of a landmark in the local map. CANDIDATE = freshly triangulated;
// CONVERGED = enough observations / parallax / small reprojection error;
// BAD = marked for the LocalMap to drop.
enum class LandmarkState {
    CANDIDATE,
    CONVERGED,
    BAD
};

// A single observation of a landmark: which keyframe saw it, at which keypoint.
// id-based (keyframe_id, not a pointer) to avoid reference cycles.
struct Observation {
    uint64_t keyframe_id    = 0;
    size_t   keypoint_index = 0;
};

// A 3D point on the surface of the scene, observed by one or more keyframes.
// Position is expressed in the world frame, in metres (scale from telemetry
// altitude — see vo_keyframe_landmark_design.md §A.5).
struct Landmark {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t        id    = 0;
    Eigen::Vector3d pos_w = Eigen::Vector3d::Zero();  // world frame, metres
    cv::Mat         descriptor;                        // 1x32 CV_8U (ORB), to re-match

    std::vector<Observation> observations;             // (keyframe_id, keypoint_index)
    int obs_count       = 0;                            // = observations.size()
    // num_visible / num_found are bumped from the Tracking thread (per-frame
    // visibility / match bookkeeping) while the Local Mapping thread reads them in
    // cullLandmarks(); they are std::atomic so those cross-thread increments/reads
    // are race-free without holding the map lock on the hot path (see Phase A of
    // the multi-thread refactor). A std::atomic member also makes Landmark
    // non-copyable, which is fine: it is only ever owned through a shared_ptr.
    std::atomic<int> num_visible{0};                    // times it should have been seen
    std::atomic<int> num_found{0};                      // times it was matched
    int n_failed_reproj = 0;                            // times reprojection failed

    LandmarkState state = LandmarkState::CANDIDATE;

    bool isBad() const { return state == LandmarkState::BAD; }
};

// A promoted frame kept in the local map: the pose variable for BA and the
// source for new feature detection / landmark triangulation. Packages the
// processed result of one FrameData; references the source only by id.
struct Keyframe {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t id             = 0;    // local-map keyframe id
    uint64_t frame_id       = 0;    // originating FrameData::frame_id
    double   timestamp_msec = 0.0;

    Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();  // camera -> world

    FeatureSet features;            // keypoints + descriptors (vo_data.h)

    // keypoint_index -> landmark_id ; -1 if unassigned. Same length as keypoints.
    std::vector<int64_t> landmark_refs;

    sensor::TelemetryData telemetry;  // yaw / altitude: prior + scale

    int    num_inliers  = 0;
    double inlier_ratio = 0.0;
    bool   is_keyframe  = true;
};

}  // namespace uavloc::vo
