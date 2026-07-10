#include "tracker.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>  // MatrixBase::inverse() (Eigen/LU)
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>

namespace uavloc::vo {

namespace {

// Fixed RANSAC seed for reproducible PnP (CLAUDE.md "PnP: 0xDEADBEEF").
constexpr uint64_t PNP_RNG_SEED = 0xDEADBEEFULL;

// Degrees -> radians for the off-nadir angle in the altitude re-anchor (pure
// unit conversion, not tunable).
const double RAD_PER_DEG = M_PI / 180.0;

// Lower clamp on cos(off_nadir) so a near-90 deg airframe tilt cannot blow up
// the slant-range d_target (mirrors metricScaleFromTelemetry in vo_module.cpp).
constexpr double MIN_COS_TILT = 1e-3;

}  // namespace

// ---------------------------------------------------------------------------
// TrackerConfig
// ---------------------------------------------------------------------------
TrackerConfig TrackerConfig::fromYaml(const YAML::Node& vo_node) {
    TrackerConfig cfg;  // start from defaults
    if (!vo_node) {
        return cfg;
    }
    cfg.search_radius_px =
        vo_node["search_radius"].as<double>(cfg.search_radius_px);
    cfg.pnp_iterations =
        vo_node["pnp_iterations"].as<int>(cfg.pnp_iterations);
    cfg.pnp_confidence =
        vo_node["pnp_confidence"].as<double>(cfg.pnp_confidence);
    cfg.min_inliers_to_track =
        vo_node["min_inliers_to_track"].as<int>(cfg.min_inliers_to_track);
    cfg.pose_opt_max_reproj_error_px =
        vo_node["pose_opt_max_reproj_error_px"].as<double>(cfg.pose_opt_max_reproj_error_px);
    cfg.min_frame_gap =
        vo_node["min_frame_gap"].as<int>(cfg.min_frame_gap);
    cfg.min_inlier_ratio =
        vo_node["min_inlier_ratio"].as<double>(cfg.min_inlier_ratio);
    cfg.min_tracked_landmarks =
        vo_node["min_tracked_landmarks"].as<int>(cfg.min_tracked_landmarks);
    cfg.kfselect_min_depth_ratio =
        vo_node["kfselect_min_depth_ratio"].as<double>(cfg.kfselect_min_depth_ratio);
    cfg.max_reproj_error_px =
        vo_node["max_reproj_error_px"].as<double>(cfg.max_reproj_error_px);
    cfg.pose_opt_iterations =
        vo_node["pose_opt_iterations"].as<int>(cfg.pose_opt_iterations);
    cfg.pose_opt_huber_px =
        vo_node["pose_opt_huber_px"].as<double>(cfg.pose_opt_huber_px);
    cfg.enable_scale_reanchor =
        vo_node["enable_scale_reanchor"].as<bool>(cfg.enable_scale_reanchor);
    cfg.rescale_s_min =
        vo_node["rescale_s_min"].as<double>(cfg.rescale_s_min);
    cfg.rescale_s_max =
        vo_node["rescale_s_max"].as<double>(cfg.rescale_s_max);
    cfg.min_landmarks_for_rescale =
        vo_node["min_landmarks_for_rescale"].as<int>(cfg.min_landmarks_for_rescale);
    return cfg;
}

// ---------------------------------------------------------------------------
// Tracker
// ---------------------------------------------------------------------------
namespace {

// Build the motion-only BA config from the tracker config (no separate YAML
// pass — the keys already live under the same "VO:" block).
PoseOptimizerConfig makePoseOptimizerConfig(const TrackerConfig& cfg) {
    PoseOptimizerConfig pc;
    pc.max_iters           = cfg.pose_opt_iterations;
    pc.huber_px            = cfg.pose_opt_huber_px;
    pc.max_reproj_error_px = cfg.pose_opt_max_reproj_error_px;
    return pc;
}

}  // namespace

Tracker::Tracker(const TrackerConfig&       config,
                 ProjectionMatcher&         matcher,
                 const sensor::CameraModel& camera,
                 LocalMap&                  local_map,
                 LocalMapper&               mapper)
    : config_(config),
      matcher_(matcher),
      camera_(camera),
      local_map_(local_map),
      mapper_(mapper),
      pose_optimizer_(makePoseOptimizerConfig(config), camera) {}

void Tracker::start(const Eigen::Matrix4d& T_wc_init, uint64_t last_kf_id) {
    T_wc_last_            = T_wc_init;
    delta_               = Eigen::Matrix4d::Identity();
    last_kf_id_          = last_kf_id;
    last_tracked_inliers_ = 0;
    t_wc_last_kf_         = T_wc_init.block<3, 1>(0, 3);
    auto kf = local_map_.getKeyframe(last_kf_id);
    last_kf_frame_id_ = kf ? kf->frame_id : 0;
}

TrackStatus Tracker::track(const FeatureSet&            curr,
                           uint64_t                     frame_id,
                           double                       timestamp_msec,
                           const sensor::TelemetryData& tel,
                           Eigen::Matrix4d&             T_wc_out) {
    const Eigen::Matrix3d& K = camera_.K();
    const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
    const int img_w = camera_.width();
    const int img_h = camera_.height();

    // Reset the per-frame observation snapshot; only inlier-tracked pixels below
    // are recorded, so a LOST frame (early return) leaves this empty.
    last_observations_.clear();
    // Reset the report-only 2D-3D match count; set once the correspondences are
    // built below, so a frame that goes LOST before PnP reports 0.
    last_tracked_matches_ = 0;

    // ---- 1. constant-velocity prediction ------------------------------------
    const Eigen::Matrix4d T_wc_pred = T_wc_last_ * delta_;
    const Eigen::Matrix4d T_cw_pred = T_wc_pred.inverse();
    const Eigen::Matrix3d R_cw      = T_cw_pred.block<3, 3>(0, 0);
    const Eigen::Vector3d t_cw      = T_cw_pred.block<3, 1>(0, 3);

    // ---- 2. project landmarks + visibility filter ---------------------------
    std::vector<uint64_t>    vis_ids;       // landmark id per fake-keypoint
    std::vector<cv::KeyPoint> vis_kps;       // projected pixels
    cv::Mat                  vis_desc;       // N x 32 CV_8U, vconcat of descriptors
    std::vector<double>      vis_depths;     // camera-frame z of each visible landmark

    // Value snapshot taken under the map lock: the Tracking thread never iterates
    // the live containers (which the Local Mapping thread may mutate). Native
    // container order is preserved, so this is bit-for-bit identical to the old
    // in-place iteration in single-thread (sync) mode.
    const std::vector<LandmarkSnapshot> lm_snapshot = local_map_.snapshotLandmarks();
    vis_ids.reserve(lm_snapshot.size());
    vis_kps.reserve(lm_snapshot.size());
    vis_depths.reserve(lm_snapshot.size());

    for (const LandmarkSnapshot& s : lm_snapshot) {
        if (!s.lm || s.is_bad || s.descriptor.empty()) {
            continue;
        }
        const Eigen::Vector3d p_c = R_cw * s.pos_w + t_cw;
        if (p_c.z() <= 0.0) {
            continue;  // behind the camera
        }
        const double u = fx * p_c.x() / p_c.z() + cx;
        const double v = fy * p_c.y() / p_c.z() + cy;
        if (u < 0.0 || u >= img_w || v < 0.0 || v >= img_h) {
            continue;  // outside the image
        }
        ++s.lm->num_visible;  // atomic; safe against the Local Mapping thread
        vis_ids.push_back(s.id);
        vis_kps.emplace_back(cv::Point2f(static_cast<float>(u),
                                         static_cast<float>(v)), 1.0f);
        vis_desc.push_back(s.descriptor.row(0));
        vis_depths.push_back(p_c.z());
    }

    if (static_cast<int>(vis_ids.size()) < config_.min_inliers_to_track) {
        spdlog::warn("Tracker: only {} visible landmarks (< {}) on frame {} — LOST",
                     vis_ids.size(), config_.min_inliers_to_track, frame_id);
        T_wc_out = T_wc_pred;
        return TrackStatus::LOST;
    }

    // ---- 3. re-match (reuse ProjectionMatcher with identity prior) ----------
    FeatureSet lm_set;            // "fake" reference: projections + descriptors
    lm_set.keypoints       = vis_kps;
    lm_set.descriptors     = vis_desc;
    lm_set.descriptor_type = DescriptorType::ORB;
    lm_set.meta.frame_id   = last_kf_id_;

    MatchesData matches;
    const Eigen::Matrix3d H_identity = Eigen::Matrix3d::Identity();
    const MatchStatus mstat = matcher_.match(lm_set, curr, matches, &H_identity);
    if (mstat != MatchStatus::OK) {
        spdlog::warn("Tracker: landmark match not OK (status={}, matches={}) on "
                     "frame {} — LOST",
                     static_cast<int>(mstat), matches.num_matches, frame_id);
        T_wc_out = T_wc_pred;
        return TrackStatus::LOST;
    }

    // ---- 4. PnP -------------------------------------------------------------
    std::vector<cv::Point3f> object_pts;
    std::vector<cv::Point2f> image_pts;
    std::vector<uint64_t>    match_lm_ids;     // landmark id per PnP correspondence
    std::vector<int>         match_curr_kp;    // curr keypoint index per correspondence
    object_pts.reserve(matches.matches.size());
    image_pts.reserve(matches.matches.size());
    match_lm_ids.reserve(matches.matches.size());
    match_curr_kp.reserve(matches.matches.size());

    const int n_curr = static_cast<int>(curr.keypoints.size());
    for (const cv::DMatch& m : matches.matches) {
        if (m.queryIdx < 0 || m.queryIdx >= static_cast<int>(vis_ids.size()) ||
            m.trainIdx < 0 || m.trainIdx >= n_curr) {
            continue;
        }
        const std::shared_ptr<Landmark> lm =
            local_map_.getLandmark(vis_ids[static_cast<size_t>(m.queryIdx)]);
        if (!lm) {
            continue;
        }
        object_pts.emplace_back(static_cast<float>(lm->pos_w.x()),
                                static_cast<float>(lm->pos_w.y()),
                                static_cast<float>(lm->pos_w.z()));
        image_pts.push_back(curr.keypoints[m.trainIdx].pt);
        match_lm_ids.push_back(lm->id);
        match_curr_kp.push_back(m.trainIdx);
    }

    // Number of 2D-3D correspondences fed to PnP; report-only denominator for the
    // tracked inlier ratio (does not influence the pose estimation below).
    last_tracked_matches_ = static_cast<int>(object_pts.size());

    if (static_cast<int>(object_pts.size()) < config_.min_inliers_to_track) {
        spdlog::warn("Tracker: only {} 2D-3D matches (< {}) on frame {} — LOST",
                     object_pts.size(), config_.min_inliers_to_track, frame_id);
        T_wc_out = T_wc_pred;
        return TrackStatus::LOST;
    }

    cv::Mat K_cv;
    cv::eigen2cv(K, K_cv);
    const cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);  // keypoints already undistorted

    // PnP. NOTE (Chặng 1C experiment): seeding RANSAC with the constant-velocity
    // prediction via useExtrinsicGuess was measured to INCREASE tracking-LOST on
    // this nadir dataset (90 -> ~110 over the full run; isolated to the guess,
    // not the BA), because a drifting motion prediction biases the iterative PnP
    // toward a poor local optimum. Cold-start PnP is kept; the predicted pose is
    // still used downstream as the LOST fallback and the BA already corrects the
    // recovered pose. Revisit a guarded guess once the motion model is sturdier.
    cv::Mat rvec, tvec;
    std::vector<int> inlier_idx;
    cv::setRNGSeed(static_cast<int>(PNP_RNG_SEED));  // reproducible PnP RANSAC
    const bool pnp_ok = cv::solvePnPRansac(
        object_pts, image_pts, K_cv, dist, rvec, tvec,
        /*useExtrinsicGuess=*/false, config_.pnp_iterations,
        static_cast<float>(config_.pose_opt_max_reproj_error_px),
        config_.pnp_confidence, inlier_idx, cv::SOLVEPNP_ITERATIVE);

    const int n_inliers = pnp_ok ? static_cast<int>(inlier_idx.size()) : 0;
    if (!pnp_ok || n_inliers < config_.min_inliers_to_track) {
        spdlog::warn("Tracker: PnP {} ({} inliers < {}) on frame {} — LOST",
                     pnp_ok ? "weak" : "failed", n_inliers,
                     config_.min_inliers_to_track, frame_id);
        T_wc_out = T_wc_pred;
        return TrackStatus::LOST;
    }

    // rvec/tvec describe T_cw (world -> camera); invert to camera -> world.
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    Eigen::Matrix3d R_cw_est;
    Eigen::Vector3d t_cw_est;
    cv::cv2eigen(R_cv, R_cw_est);
    cv::cv2eigen(tvec, t_cw_est);

    Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity();
    T_cw.block<3, 3>(0, 0) = R_cw_est;
    T_cw.block<3, 1>(0, 3) = t_cw_est;
    Eigen::Matrix4d T_wc = T_cw.inverse();

    // ---- 5. motion-only BA refinement (Chặng 1C) ---------------------------
    // Refine T_wc against the PnP inlier 2D-3D set with landmarks fixed. The
    // robust GN typically reduces the mean reprojection error and corrects the
    // raw-PnP under-scale/jitter. inlier_idx indexes into object_pts/image_pts.
    std::vector<bool> is_inlier(object_pts.size(), false);
    {
        std::vector<Eigen::Vector3d> ba_pts;
        std::vector<Eigen::Vector2d> ba_obs;
        std::vector<int>             ba_src;  // -> index in object_pts
        ba_pts.reserve(inlier_idx.size());
        ba_obs.reserve(inlier_idx.size());
        ba_src.reserve(inlier_idx.size());
        for (int idx : inlier_idx) {
            if (idx < 0 || idx >= static_cast<int>(object_pts.size())) {
                continue;
            }
            ba_pts.emplace_back(object_pts[idx].x, object_pts[idx].y,
                                object_pts[idx].z);
            ba_obs.emplace_back(image_pts[idx].x, image_pts[idx].y);
            ba_src.push_back(idx);
        }

        std::vector<bool> ba_inlier;
        const int n_ba = pose_optimizer_.optimize(ba_pts, ba_obs, T_wc, ba_inlier);
        if (n_ba >= config_.min_inliers_to_track) {
            // BA succeeded: adopt the refined pose and its inlier set.
            for (size_t k = 0; k < ba_src.size(); ++k) {
                if (k < ba_inlier.size() && ba_inlier[k]) {
                    is_inlier[static_cast<size_t>(ba_src[k])] = true;
                }
            }
        } else {
            // BA degenerated below the tracking floor; keep the raw PnP pose and
            // its inliers (do not let a weak refinement drop the frame).
            spdlog::debug("Tracker: BA kept only {} inliers (< {}) on frame {} — "
                          "falling back to raw PnP pose",
                          n_ba, config_.min_inliers_to_track, frame_id);
            T_wc = T_cw.inverse();
            for (int idx : inlier_idx) {
                if (idx >= 0 && idx < static_cast<int>(is_inlier.size())) {
                    is_inlier[static_cast<size_t>(idx)] = true;
                }
            }
        }
    }
    // Post-BA inlier count drives the bookkeeping, ratio, and keyframe gates.
    int n_tracked = 0;
    for (size_t i = 0; i < match_lm_ids.size(); ++i) {
        if (!is_inlier[i]) {
            continue;
        }
        ++n_tracked;
        // Snapshot the inlier observation pixel for the debug-viewer overlay
        // (read-only; does not influence the pose/keyframe logic below).
        last_observations_.emplace_back(image_pts[i].x, image_pts[i].y);
        if (auto lm = local_map_.getLandmark(match_lm_ids[i])) {
            ++lm->num_found;
        }
    }

    const double inlier_ratio =
        object_pts.empty() ? 0.0
                           : static_cast<double>(n_tracked) /
                                 static_cast<double>(object_pts.size());

    delta_                = T_wc_last_.inverse() * T_wc;
    T_wc_last_            = T_wc;
    last_tracked_inliers_ = n_tracked;
    T_wc_out              = T_wc;

    // ---- 7. keyframe decision (SVO DOWNLOOKING, depth-normalized) -----------
    // Insert when the camera has moved a meaningful fraction of the scene depth
    // (so the triangulation baseline is healthy), or the tracked count is low —
    // but only once a minimum frame gap (FLOOR) has elapsed to avoid back-to-
    // back keyframes. The inlier-ratio gate is no longer a trigger (it forced
    // dense, short-baseline keyframes); it is kept only for diagnostics.
    const int frame_gap = static_cast<int>(frame_id) -
                          static_cast<int>(last_kf_frame_id_);

    // Median scene depth = median camera-frame z of the visible landmarks; fall
    // back to the post-init median seed depth proxy via the keyframe baseline
    // when none are visible (rare; vis count already passed the LOST floor).
    double depth = 0.0;
    if (!vis_depths.empty()) {
        std::vector<double> sorted_depths = vis_depths;
        const size_t mid = sorted_depths.size() / 2;
        std::nth_element(sorted_depths.begin(),
                         sorted_depths.begin() + static_cast<long>(mid),
                         sorted_depths.end());
        depth = sorted_depths[mid];
    }

    // Metric translation since the last keyframe.
    const double d = (T_wc.block<3, 1>(0, 3) - t_wc_last_kf_).norm();
    const double depth_ratio = (depth > 0.0) ? d / depth : 0.0;

    const bool need_kf =
        (config_.kfselect_min_depth_ratio < 0.0)
            ? ((frame_gap >= config_.min_frame_gap) ||
               (n_tracked < config_.min_tracked_landmarks) ||
               (inlier_ratio < config_.min_inlier_ratio))
            : (((depth_ratio >= config_.kfselect_min_depth_ratio) ||
                (n_tracked < config_.min_tracked_landmarks)) &&
               (frame_gap >= config_.min_frame_gap));

    if (!need_kf) {
        return TrackStatus::OK;
    }

    spdlog::debug("Tracker: KF gate fired at frame {} "
                  "(d={:.2f} m, depth={:.2f} m, ratio={:.3f}>= {:.3f}, "
                  "n_tracked={}, inlier_ratio={:.3f}, frame_gap={})",
                  frame_id, d, depth, depth_ratio,
                  config_.kfselect_min_depth_ratio, n_tracked, inlier_ratio,
                  frame_gap);

    // -- 7a. build a keyframe packet and advance the tracker's keyframe state --
    // The Tracking thread allocates the keyframe id and advances its own motion /
    // keyframe state immediately (ORB-SLAM style: last_kf_id_ is current at once),
    // then hands the heavy map work — attach observations, triangulate against the
    // reference keyframe, insert, cull — to the LocalMapper. In sync mode the
    // mapper runs this inline (deterministic, bit-for-bit with the old code); in
    // async mode it runs on the Local Mapping thread.
    KeyframePacket pkt;
    pkt.kf_id          = local_map_.nextKeyframeId();
    pkt.ref_kf_id      = last_kf_id_;   // reference keyframe for triangulation (old KF)
    pkt.frame_id       = frame_id;
    pkt.timestamp_msec = timestamp_msec;
    pkt.T_wc           = T_wc;
    pkt.features       = curr;
    pkt.telemetry      = tel;
    pkt.num_inliers    = n_tracked;
    pkt.inlier_ratio   = inlier_ratio;
    pkt.match_lm_ids   = match_lm_ids;
    pkt.match_curr_kp  = match_curr_kp;
    pkt.is_inlier.assign(match_lm_ids.size(), 0);
    for (size_t i = 0; i < match_lm_ids.size(); ++i) {
        pkt.is_inlier[i] = is_inlier[i] ? 1 : 0;
    }

    const uint64_t new_kf_id = pkt.kf_id;
    last_kf_id_       = new_kf_id;
    last_kf_frame_id_ = frame_id;
    t_wc_last_kf_     = T_wc.block<3, 1>(0, 3);

    mapper_.submitKeyframe(std::move(pkt));

    // -- 7b. altitude re-anchor (Fix #1) --------------------------------------
    // Pin the map scale to the AGL prior so monocular scale/Z drift does not
    // accumulate. The slant range d_target = AGL / cos(off_nadir) is the expected
    // median camera-frame depth for a near-nadir camera; rescale the local map
    // (landmarks + keyframe translations) by s = d_target / depth_med around the
    // world origin (KF0). A uniform similarity preserves reprojection, so the
    // inlier set is untouched — only the absolute scale is re-anchored. Reuses the
    // median visible-landmark depth `depth` computed for the keyframe gate above.
    //
    // DISABLED BY DEFAULT (VO.enable_scale_reanchor=false): the similarity about
    // KF0 blows the trajectory up on long segments. Scaffolding kept for re-design.
    // Sync-only: the rescale mutates BOTH the map and the tracker's motion state,
    // which cannot be done consistently while the Local Mapping thread is running.
    if (!async_ && config_.enable_scale_reanchor) {
        if (depth > 0.0 && tel.altitude_m > 0.0 &&
            static_cast<int>(vis_depths.size()) >= config_.min_landmarks_for_rescale) {
            const double roll  = tel.roll_deg  * RAD_PER_DEG;
            const double pitch = tel.pitch_deg * RAD_PER_DEG;
            const double cos_tilt = std::max(std::cos(roll) * std::cos(pitch),
                                             MIN_COS_TILT);
            const double d_target = tel.altitude_m / cos_tilt;
            double s = d_target / depth;
            s = std::max(config_.rescale_s_min,
                         std::min(config_.rescale_s_max, s));
            if (s != 1.0) {
                local_map_.rescale(s);
                // Re-anchor the tracker pose state to match the rescaled map. The
                // rescale is a similarity about the origin, so only translations and
                // the motion-twist translation scale.
                T_wc_last_.block<3, 1>(0, 3) *= s;
                t_wc_last_kf_                *= s;
                delta_.block<3, 1>(0, 3)     *= s;
                T_wc_out.block<3, 1>(0, 3)   *= s;
            }
            spdlog::debug("Tracker: altitude re-anchor at frame {} "
                          "(d_target={:.2f} m, depth_med={:.2f} m, s={:.4f})",
                          frame_id, d_target, depth, s);
        }
    }

    spdlog::debug("Tracker: keyframe {} promoted at frame {} (inliers {}) — "
                  "map growth handed to LocalMapper",
                  new_kf_id, frame_id, n_tracked);
    return TrackStatus::KEYFRAME_INSERTED;
}

}  // namespace uavloc::vo
