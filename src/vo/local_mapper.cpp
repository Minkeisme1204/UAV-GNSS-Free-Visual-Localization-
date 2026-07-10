#include "local_mapper.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>  // MatrixBase::inverse() (Eigen/LU)
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>

namespace uavloc::vo {

namespace {

// Minimum ratio num_found/num_visible below which a (sufficiently observed)
// landmark is culled — same spirit as the SVO/ORB-SLAM 0.25 gate. Not a tunable
// pipeline threshold (it is a map-hygiene constant), so it stays in source.
constexpr double MIN_FOUND_RATIO = 0.25;

// A landmark must have been projected at least this many times before its
// found/visible ratio is trusted enough to cull it (avoids dropping fresh
// landmarks that simply have not been seen yet).
constexpr int MIN_VISIBLE_BEFORE_CULL = 5;

}  // namespace

// ---------------------------------------------------------------------------
// LocalMapperConfig
// ---------------------------------------------------------------------------
LocalMapperConfig LocalMapperConfig::fromYaml(const YAML::Node& vo_node) {
    LocalMapperConfig cfg;  // start from defaults
    if (!vo_node) {
        return cfg;
    }
    cfg.max_reproj_error_px =
        vo_node["max_reproj_error_px"].as<double>(cfg.max_reproj_error_px);
    return cfg;
}

// ---------------------------------------------------------------------------
// LocalMapper
// ---------------------------------------------------------------------------
LocalMapper::LocalMapper(const LocalMapperConfig&       config,
                         const ProjectionMatcherConfig& matcher_config,
                         const sensor::CameraModel&     camera,
                         LocalMap&                      local_map)
    : config_(config),
      matcher_(matcher_config),
      camera_(camera),
      local_map_(local_map) {}

LocalMapper::~LocalMapper() {
    stop();
}

void LocalMapper::submitKeyframe(KeyframePacket packet) {
    if (running_.load()) {
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_.push_back(std::move(packet));
        }
        queue_cv_.notify_one();
    } else {
        // Synchronous: run inline so the pipeline stays deterministic.
        processKeyframe(packet);
    }
}

void LocalMapper::start() {
    if (running_.exchange(true)) {
        return;  // already running
    }
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        stop_flag_ = false;
    }
    thread_ = std::thread([this] { run(); });
}

void LocalMapper::stop() {
    if (!running_.load()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        stop_flag_ = true;
    }
    queue_cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false);

    // Drain any packets still queued so the local map is complete.
    while (!queue_.empty()) {
        processKeyframe(queue_.front());
        queue_.pop_front();
    }
}

void LocalMapper::run() {
    while (true) {
        KeyframePacket pkt;
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait(lk, [this] { return !queue_.empty() || stop_flag_; });
            if (stop_flag_ && queue_.empty()) {
                break;
            }
            pkt = std::move(queue_.front());
            queue_.pop_front();
        }
        processKeyframe(pkt);
    }
}

void LocalMapper::processKeyframe(const KeyframePacket& pkt) {
    // -- build the keyframe from the packet -----------------------------------
    auto kf = std::make_shared<Keyframe>();
    kf->id             = pkt.kf_id;
    kf->frame_id       = pkt.frame_id;
    kf->timestamp_msec = pkt.timestamp_msec;
    kf->T_wc           = pkt.T_wc;
    kf->features       = pkt.features;
    kf->landmark_refs.assign(pkt.features.keypoints.size(), -1);
    kf->telemetry      = pkt.telemetry;
    kf->num_inliers    = pkt.num_inliers;
    kf->inlier_ratio   = pkt.inlier_ratio;

    // -- attach observations of the tracked (inlier) landmarks ----------------
    for (size_t i = 0; i < pkt.match_lm_ids.size(); ++i) {
        if (i >= pkt.is_inlier.size() || !pkt.is_inlier[i]) {
            continue;
        }
        const int kp = pkt.match_curr_kp[i];
        if (kp < 0 || kp >= static_cast<int>(kf->landmark_refs.size())) {
            continue;
        }
        if (auto lm = local_map_.getLandmark(pkt.match_lm_ids[i])) {
            lm->observations.push_back({kf->id, static_cast<size_t>(kp)});
            lm->obs_count = static_cast<int>(lm->observations.size());
            kf->landmark_refs[static_cast<size_t>(kp)] =
                static_cast<int64_t>(lm->id);
        }
    }

    // -- triangulate new landmarks against the reference keyframe -------------
    int n_new = 0;
    if (auto ref_kf = local_map_.getKeyframe(pkt.ref_kf_id)) {
        n_new = triangulateNewLandmarks(*ref_kf, pkt.features, *kf, pkt.T_wc);
    }

    local_map_.addKeyframe(kf);

    // -- cull stale landmarks -------------------------------------------------
    const int n_culled = cullLandmarks();

    spdlog::debug("LocalMapper: keyframe {} at frame {} "
                  "(inliers {}, +{} new landmarks, -{} culled, map={})",
                  kf->id, pkt.frame_id, pkt.num_inliers, n_new, n_culled,
                  local_map_.numLandmarks());
}

int LocalMapper::triangulateNewLandmarks(const Keyframe&    ref_kf,
                                         const FeatureSet&  curr,
                                         Keyframe&          new_kf,
                                         const Eigen::Matrix4d& T_wc_curr) {
    const Eigen::Matrix3d& K = camera_.K();
    const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);

    // Match the reference keyframe against the current frame (identity prior).
    MatchesData matches;
    const Eigen::Matrix3d H_identity = Eigen::Matrix3d::Identity();
    const MatchStatus mstat =
        matcher_.match(ref_kf.features, curr, matches, &H_identity);
    if (mstat != MatchStatus::OK) {
        return 0;
    }

    // Projection matrices: P = K [R_cw | t_cw] for each view.
    cv::Mat K_cv;
    cv::eigen2cv(K, K_cv);

    auto projection = [&](const Eigen::Matrix4d& T_wc) {
        const Eigen::Matrix4d T_cw = T_wc.inverse();
        Eigen::Matrix<double, 3, 4> Rt = T_cw.block<3, 4>(0, 0);
        cv::Mat Rt_cv;
        cv::eigen2cv(Rt, Rt_cv);
        return cv::Mat(K_cv * Rt_cv);
    };
    const cv::Mat P_ref  = projection(ref_kf.T_wc);
    const cv::Mat P_curr = projection(T_wc_curr);

    const Eigen::Matrix4d T_cw_ref  = ref_kf.T_wc.inverse();
    const Eigen::Matrix4d T_cw_curr = T_wc_curr.inverse();

    std::vector<cv::Point2f> pts_ref, pts_curr;
    std::vector<int>         ref_kp, curr_kp;
    const int n_ref  = static_cast<int>(ref_kf.features.keypoints.size());
    const int n_curr = static_cast<int>(curr.keypoints.size());
    for (const cv::DMatch& m : matches.matches) {
        if (m.queryIdx < 0 || m.queryIdx >= n_ref ||
            m.trainIdx < 0 || m.trainIdx >= n_curr) {
            continue;
        }
        // Skip keypoints already tied to a landmark in either view.
        if (ref_kf.landmark_refs[static_cast<size_t>(m.queryIdx)] >= 0) {
            continue;
        }
        if (new_kf.landmark_refs[static_cast<size_t>(m.trainIdx)] >= 0) {
            continue;
        }
        pts_ref.push_back(ref_kf.features.keypoints[m.queryIdx].pt);
        pts_curr.push_back(curr.keypoints[m.trainIdx].pt);
        ref_kp.push_back(m.queryIdx);
        curr_kp.push_back(m.trainIdx);
    }
    if (pts_ref.empty()) {
        return 0;
    }

    cv::Mat pts4d_f;
    cv::triangulatePoints(P_ref, P_curr, pts_ref, pts_curr, pts4d_f);
    cv::Mat pts4d;
    pts4d_f.convertTo(pts4d, CV_64F);  // triangulatePoints returns CV_32F (1A bug)

    int n_new = 0;
    for (int i = 0; i < pts4d.cols; ++i) {
        const double w = pts4d.at<double>(3, i);
        if (std::abs(w) < 1e-9) {
            continue;
        }
        const Eigen::Vector3d X(pts4d.at<double>(0, i) / w,
                                pts4d.at<double>(1, i) / w,
                                pts4d.at<double>(2, i) / w);

        // Cheirality in both keyframes.
        const Eigen::Vector3d Xr =
            T_cw_ref.block<3, 3>(0, 0) * X + T_cw_ref.block<3, 1>(0, 3);
        const Eigen::Vector3d Xc =
            T_cw_curr.block<3, 3>(0, 0) * X + T_cw_curr.block<3, 1>(0, 3);
        if (Xr.z() <= 0.0 || Xc.z() <= 0.0) {
            continue;
        }
        // Reprojection gate in both views.
        const double er = std::hypot(fx * Xr.x() / Xr.z() + cx -
                                         pts_ref[static_cast<size_t>(i)].x,
                                     fy * Xr.y() / Xr.z() + cy -
                                         pts_ref[static_cast<size_t>(i)].y);
        const double ec = std::hypot(fx * Xc.x() / Xc.z() + cx -
                                         pts_curr[static_cast<size_t>(i)].x,
                                     fy * Xc.y() / Xc.z() + cy -
                                         pts_curr[static_cast<size_t>(i)].y);
        if (er > config_.max_reproj_error_px || ec > config_.max_reproj_error_px) {
            continue;
        }

        const int rkp = ref_kp[static_cast<size_t>(i)];
        const int ckp = curr_kp[static_cast<size_t>(i)];

        auto lm = std::make_shared<Landmark>();
        lm->id    = local_map_.nextLandmarkId();
        lm->pos_w = X;
        if (ckp >= 0 && ckp < curr.descriptors.rows) {
            lm->descriptor = curr.descriptors.row(ckp).clone();
        }
        lm->observations.push_back({ref_kf.id, static_cast<size_t>(rkp)});
        lm->observations.push_back({new_kf.id, static_cast<size_t>(ckp)});
        lm->obs_count   = 2;
        lm->num_visible = 2;
        lm->num_found   = 2;
        lm->state       = LandmarkState::CONVERGED;

        new_kf.landmark_refs[static_cast<size_t>(ckp)] =
            static_cast<int64_t>(lm->id);
        // ref_kf is owned by the LocalMap; update its ref too.
        if (auto ref_owned = local_map_.getKeyframe(ref_kf.id)) {
            ref_owned->landmark_refs[static_cast<size_t>(rkp)] =
                static_cast<int64_t>(lm->id);
        }
        local_map_.addLandmark(lm);
        ++n_new;
    }
    return n_new;
}

int LocalMapper::cullLandmarks() {
    int n_culled = 0;
    // Only the Local Mapping thread mutates the map, so iterating the live
    // container here (marking state, never erasing) is safe against the Tracking
    // thread, which reads through the locked snapshotLandmarks().
    for (const auto& kv : local_map_.landmarks()) {
        const std::shared_ptr<Landmark>& lm = kv.second;
        if (!lm || lm->isBad()) {
            continue;
        }
        if (lm->num_visible.load() < MIN_VISIBLE_BEFORE_CULL) {
            continue;  // too young to judge
        }
        const double found_ratio =
            static_cast<double>(lm->num_found.load()) /
            static_cast<double>(std::max(1, lm->num_visible.load()));
        if (found_ratio < MIN_FOUND_RATIO) {
            lm->state = LandmarkState::BAD;
            ++n_culled;
        }
    }
    return n_culled;
}

}  // namespace uavloc::vo
