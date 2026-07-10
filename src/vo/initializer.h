#pragma once

// Initializer — bootstraps the VO local map from a fixed reference frame.
//
// Unlike a per-frame two-view tracker (Phase 0), the Initializer holds the
// FIRST frame as a fixed reference and waits, frame after frame, until the
// camera has translated far enough to accumulate sufficient parallax. Only
// then does it run homography decomposition (PoseEstimator) and triangulate a
// metric seed map. This is the ORB-SLAM "initialise once, accumulate parallax"
// strategy and is the root fix for the per-frame DECOMPOSITION_FAILED spam:
// decomposition is never attempted on a near-stationary baseline.
//
// Scale: the caller passes metric_scale (slant range to ground, metres) from
// telemetry; PoseEstimator promotes the translation to metres, so the
// triangulated landmarks are metric.

#include <memory>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "uavloc/sensor/camera_model.h"
#include "uavloc/sensor/telemetry_data.h"
#include "uavloc/vo/pose_estimator.h"
#include "uavloc/vo/projection_matcher.h"
#include "uavloc/vo/vo_data.h"

#include "local_map.h"

namespace uavloc::vo {

// Tunable parameters for the Initializer. Reuses existing VO YAML keys; no new
// config struct or YAML key is introduced. All fields default so the struct can
// be built with InitializerConfig{} and overridden from YAML.
struct InitializerConfig {
    // Minimum median inlier pixel disparity before a decomposition is attempted.
    // YAML key: VO.parallax_pixel_threshold.
    double parallax_pixel_threshold = 5.0;

    // Minimum number of well-triangulated points to accept initialization.
    // YAML key: VO.min_triangulated_pts.
    int min_triangulated_pts = 80;

    // Maximum per-point reprojection error (px) for a triangulated point to be
    // counted as good. YAML key: VO.max_reproj_error_px.
    double max_reproj_error_px = 4.0;

    // Build from the mission "VO:" node; missing keys fall back to defaults.
    static InitializerConfig fromYaml(const YAML::Node& vo_node);
};

// Outcome of a single tryInitialize() call.
enum class InitStatus {
    NOT_READY,  // not enough parallax / good points yet — keep the reference
    SUCCESS,    // seed map built; localMap() is now populated
    FAILED      // unrecoverable (bad input / pose estimation error)
};

// Stateful across calls within one initialization attempt: holds the fixed
// reference FeatureSet + its telemetry. Reuses ProjectionMatcher / PoseEstimator
// (passed by reference) and CameraModel for triangulation.
class Initializer {
public:
    Initializer(const InitializerConfig& config,
                ProjectionMatcher&        matcher,
                PoseEstimator&            pose_estimator,
                const sensor::CameraModel& camera);

    // Latch the first frame as the fixed reference. Clears any prior attempt.
    void reset(const FeatureSet&            first_ref,
               const sensor::TelemetryData& ref_telemetry);

    // Attempt to initialise against the held reference using the current frame.
    //  - curr         : current FeatureSet (undistorted keypoints expected)
    //  - curr_tel     : telemetry of the current frame (for the KF1 record)
    //  - metric_scale : slant range to ground in metres (caller-supplied, e.g.
    //                   altitude_m / cos(off_nadir)); promotes translation +
    //                   landmarks to metres. <= 0 keeps them up-to-scale.
    // On SUCCESS the local map holds KF0 (T_wc = I) and KF1 (T_wc = T_prev_curr)
    // plus the triangulated landmarks; isInitialized() becomes true.
    InitStatus tryInitialize(const FeatureSet&            curr,
                             const sensor::TelemetryData& curr_tel,
                             double                       metric_scale);

    bool            isInitialized() const { return initialized_; }
    const LocalMap& localMap()      const { return *local_map_; }

    // Mutable access to the seeded local map, so a downstream tracker can extend
    // it (add keyframes/landmarks, cull). The map persists across reset()
    // (cleared, not reallocated), so a reference taken once stays valid.
    LocalMap&       localMap()            { return *local_map_; }

    // The relative pose ref -> curr recovered on SUCCESS (KF1 T_wc). Identity
    // until a successful initialization.
    const Eigen::Matrix4d& T_prev_curr() const { return T_prev_curr_; }

    // Number of landmarks seeded on the last SUCCESS (0 otherwise).
    int numSeeded() const { return num_seeded_; }

    // Number of ref->curr feature matches on the last SUCCESS (0 otherwise). This
    // is the "matches" denominator for the seeded inlier ratio; report-only.
    int numMatches() const { return num_matches_; }

    // Median landmark depth (Z in the KF0 camera frame, metres) on SUCCESS.
    double medianSeedDepth() const { return median_seed_depth_; }

private:
    InitializerConfig          config_;
    ProjectionMatcher&         matcher_;
    PoseEstimator&             pose_estimator_;
    const sensor::CameraModel& camera_;

    bool                       has_ref_ = false;
    FeatureSet                 ref_features_;
    sensor::TelemetryData      ref_telemetry_;

    bool                       initialized_ = false;
    std::unique_ptr<LocalMap>  local_map_;
    Eigen::Matrix4d            T_prev_curr_      = Eigen::Matrix4d::Identity();
    int                        num_seeded_       = 0;
    int                        num_matches_      = 0;
    double                     median_seed_depth_ = 0.0;
};

}  // namespace uavloc::vo
