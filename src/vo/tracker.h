#pragma once

// Tracker — Chặng 1B: track the camera pose against the local map instead of
// chaining adjacent two-view homographies (Phase 0).
//
// Once the Initializer has seeded a metric local map, this class takes over per
// frame:
//   1. Predict the pose with a constant-velocity motion model.
//   2. Project the local-map landmarks into the predicted frame and keep the
//      visible ones (cheirality + inside the image).
//   3. Re-match the visible landmark descriptors against the current keypoints,
//      REUSING ProjectionMatcher: the visible landmarks are packaged as a
//      "fake" FeatureSet whose keypoints are the projected pixels and whose
//      descriptors are the landmark descriptors; an identity homography prior
//      makes the matcher search a window around each projection.
//   4. Solve the 2D-3D pose with cv::solvePnPRansac.
//   5. Decide whether to insert a keyframe and, if so, triangulate new
//      landmarks against the nearest covisible keyframe and cull stale ones.
//
// SE3 convention (CLAUDE.md): T_wc is camera->world; T_cw = T_wc.inverse().
// The constant-velocity increment is stored as delta_ = T_wc_prev^-1 * T_wc,
// so the prediction is T_wc_pred = T_wc_last * delta_.

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "uavloc/sensor/camera_model.h"
#include "uavloc/sensor/telemetry_data.h"
#include "uavloc/vo/local_optimizer.h"
#include "uavloc/vo/projection_matcher.h"
#include "uavloc/vo/vo_data.h"

#include "local_map.h"
#include "local_mapper.h"

namespace uavloc::vo {

// Tunable parameters for the Tracker. Reuses existing VO YAML keys (no new keys
// are introduced); every field defaults so the struct can be built with
// TrackerConfig{} and overridden from YAML.
struct TrackerConfig {
    // Window radius (px) for re-matching projected landmarks. YAML: VO.search_radius.
    double search_radius_px = 20.0;

    // cv::solvePnPRansac iteration cap. YAML: VO.pnp_iterations.
    int pnp_iterations = 100;

    // cv::solvePnPRansac confidence. YAML: VO.pnp_confidence.
    double pnp_confidence = 0.99;

    // Minimum PnP inliers to accept a tracked pose. YAML: VO.min_inliers_to_track.
    int min_inliers_to_track = 30;

    // PnP RANSAC reprojection-error threshold (px). YAML: VO.pose_opt_max_reproj_error_px.
    double pose_opt_max_reproj_error_px = 5.99;

    // Keyframe insertion: minimum frame gap since the last keyframe. Acts as a
    // FLOOR (AND condition) to stop back-to-back keyframes, NOT a standalone
    // trigger. YAML: VO.min_frame_gap.
    int min_frame_gap = 5;

    // Keyframe insertion: insert when the inlier ratio drops below this.
    // YAML: VO.min_inlier_ratio. (Logged for diagnostics; no longer a trigger.)
    double min_inlier_ratio = 0.20;

    // Keyframe insertion: insert when tracked inliers fall below this count.
    // YAML: VO.min_tracked_landmarks.
    int min_tracked_landmarks = 50;

    // Keyframe insertion (SVO DOWNLOOKING, depth-normalized): insert when the
    // camera has translated by at least this fraction of the median scene depth
    // since the last keyframe (d / depth >= ratio). Larger => sparser keyframes
    // with a longer triangulation baseline. YAML: VO.kfselect_min_depth_ratio.
    double kfselect_min_depth_ratio = 0.10;

    // Max per-point reprojection error (px) for a newly triangulated landmark to
    // be kept. YAML: VO.max_reproj_error_px.
    double max_reproj_error_px = 4.0;

    // Motion-only BA iteration cap (Chặng 1C). YAML: VO.pose_opt_iterations.
    int pose_opt_iterations = 10;

    // Motion-only BA Huber threshold (px). YAML: VO.pose_opt_huber_px.
    double pose_opt_huber_px = 2.0;

    // Altitude re-anchor (Fix #1): master enable switch. Disabled by default —
    // the per-keyframe similarity rescale about KF0 destabilises long segments
    // (trajectory blow-up). Scaffolding below is kept for a future re-design.
    // YAML: VO.enable_scale_reanchor.
    bool enable_scale_reanchor = false;

    // Altitude re-anchor (Fix #1): lower/upper clamp on the per-keyframe rescale
    // scalar s = d_target/depth_med, applied around the world origin to pin the
    // map scale to the AGL prior. YAML: VO.rescale_s_min / VO.rescale_s_max.
    double rescale_s_min = 0.5;
    double rescale_s_max = 2.0;

    // Altitude re-anchor (Fix #1): minimum number of visible landmarks before a
    // rescale is trusted (guards a noisy / biased median depth).
    // YAML: VO.min_landmarks_for_rescale.
    int min_landmarks_for_rescale = 30;

    // Build from the mission "VO:" node; missing keys fall back to defaults.
    static TrackerConfig fromYaml(const YAML::Node& vo_node);
};

// Outcome of a single track() call.
enum class TrackStatus {
    OK,                 // tracked; pose updated, no keyframe inserted
    KEYFRAME_INSERTED,  // tracked AND a new keyframe was promoted this frame
    LOST                // too few PnP inliers; pose left at the motion prediction
};

// Tracks the camera against a shared LocalMap (built by the Initializer). Reuses
// ProjectionMatcher (passed by reference) and CameraModel; operates in place on
// the LocalMap it is given.
class Tracker {
public:
    Tracker(const TrackerConfig&        config,
            ProjectionMatcher&          matcher,
            const sensor::CameraModel&  camera,
            LocalMap&                   local_map,
            LocalMapper&                mapper);

    // Initialise the tracker from the post-init state: the pose of the latest
    // keyframe (KF1) and its keyframe id. Resets the motion model to identity.
    void start(const Eigen::Matrix4d& T_wc_init, uint64_t last_kf_id);

    // Select synchronous (default) or asynchronous keyframe handling. In async
    // mode the tracker hands keyframes to the LocalMapper thread and skips the
    // tracking-coupled scale re-anchor (which is not deterministic across
    // threads). Must be toggled while no frame is in flight.
    void setAsync(bool async) { async_ = async; }

    // Track the current frame against the local map.
    //  - curr           : current FeatureSet (undistorted keypoints expected)
    //  - frame_id       : originating FrameData::frame_id
    //  - timestamp_msec : current frame timestamp
    //  - tel            : current telemetry (stored on any inserted keyframe)
    //  - T_wc_out       : the tracked (or, on LOST, predicted) camera->world pose
    TrackStatus track(const FeatureSet&            curr,
                      uint64_t                     frame_id,
                      double                       timestamp_msec,
                      const sensor::TelemetryData& tel,
                      Eigen::Matrix4d&             T_wc_out);

    int lastTrackedInliers() const { return last_tracked_inliers_; }

    // Number of 2D-3D correspondences fed to PnP in the most recent track() call
    // (before inlier filtering). This is the "matches" denominator for the tracked
    // inlier ratio; 0 on a frame that went LOST before PnP. Report-only.
    int lastTrackedMatches() const { return last_tracked_matches_; }

    // 2D pixel observations of the landmarks tracked as inliers in the most recent
    // track() call. Empty on a LOST frame. Read-only diagnostic snapshot; does not
    // affect tracking logic.
    const std::vector<Eigen::Vector2d>& lastObservations() const {
        return last_observations_;
    }

private:
    TrackerConfig              config_;
    ProjectionMatcher&         matcher_;
    const sensor::CameraModel& camera_;
    LocalMap&                  local_map_;
    LocalMapper&               mapper_;     // grows the map from keyframe packets

    // Keyframe handling mode: false = synchronous inline (deterministic default),
    // true = hand off to the LocalMapper thread.
    bool async_ = false;

    // Motion-only BA (Chặng 1C): refines the PnP pose with the landmarks fixed.
    PoseOptimizer              pose_optimizer_;

    Eigen::Matrix4d T_wc_last_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d delta_     = Eigen::Matrix4d::Identity();  // T_wc_prev^-1 * T_wc

    // World position of the last inserted keyframe; the depth-normalized keyframe
    // gate measures translation since this point. Updated on start() and on every
    // keyframe insertion.
    Eigen::Vector3d t_wc_last_kf_ = Eigen::Vector3d::Zero();

    uint64_t last_kf_id_      = 0;
    uint64_t last_kf_frame_id_ = 0;
    int      last_tracked_inliers_ = 0;
    int      last_tracked_matches_ = 0;  // 2D-3D correspondences fed to PnP

    // Pixel observations of the inlier-tracked landmarks from the most recent
    // track() call (diagnostic only; see lastObservations()).
    std::vector<Eigen::Vector2d> last_observations_;
};

}  // namespace uavloc::vo
