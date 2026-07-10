#include "initializer.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>

namespace uavloc::vo {

// ---------------------------------------------------------------------------
// InitializerConfig
// ---------------------------------------------------------------------------
InitializerConfig InitializerConfig::fromYaml(const YAML::Node& vo_node) {
    InitializerConfig cfg;  // start from defaults
    if (!vo_node) {
        return cfg;
    }
    cfg.parallax_pixel_threshold =
        vo_node["parallax_pixel_threshold"].as<double>(cfg.parallax_pixel_threshold);
    cfg.min_triangulated_pts =
        vo_node["min_triangulated_pts"].as<int>(cfg.min_triangulated_pts);
    cfg.max_reproj_error_px =
        vo_node["max_reproj_error_px"].as<double>(cfg.max_reproj_error_px);
    return cfg;
}

// ---------------------------------------------------------------------------
// Initializer
// ---------------------------------------------------------------------------
Initializer::Initializer(const InitializerConfig& config,
                         ProjectionMatcher&        matcher,
                         PoseEstimator&            pose_estimator,
                         const sensor::CameraModel& camera)
    : config_(config),
      matcher_(matcher),
      pose_estimator_(pose_estimator),
      camera_(camera),
      local_map_(std::make_unique<LocalMap>()) {}

void Initializer::reset(const FeatureSet&            first_ref,
                        const sensor::TelemetryData& ref_telemetry) {
    ref_features_      = first_ref;
    ref_telemetry_     = ref_telemetry;
    has_ref_           = true;
    initialized_       = false;
    local_map_->clear();
    T_prev_curr_       = Eigen::Matrix4d::Identity();
    num_seeded_        = 0;
    num_matches_       = 0;
    median_seed_depth_ = 0.0;
}

namespace {

// Median pixel disparity over the given matches (all of them — used as the
// pre-decomposition parallax gate before inliers exist). queryIdx -> ref,
// trainIdx -> curr.
double medianMatchDisparity(const FeatureSet&             ref,
                            const FeatureSet&             curr,
                            const std::vector<cv::DMatch>& matches) {
    std::vector<double> disp;
    disp.reserve(matches.size());
    const int n_ref  = static_cast<int>(ref.keypoints.size());
    const int n_curr = static_cast<int>(curr.keypoints.size());
    for (const cv::DMatch& m : matches) {
        if (m.queryIdx < 0 || m.queryIdx >= n_ref ||
            m.trainIdx < 0 || m.trainIdx >= n_curr) {
            continue;
        }
        const cv::Point2f d = curr.keypoints[m.trainIdx].pt -
                              ref.keypoints[m.queryIdx].pt;
        disp.push_back(std::sqrt(static_cast<double>(d.x) * d.x +
                                 static_cast<double>(d.y) * d.y));
    }
    if (disp.empty()) {
        return 0.0;
    }
    const size_t mid = disp.size() / 2;
    std::nth_element(disp.begin(), disp.begin() + mid, disp.end());
    return disp[mid];
}

}  // namespace

InitStatus Initializer::tryInitialize(const FeatureSet&            curr,
                                      const sensor::TelemetryData& curr_tel,
                                      double                       metric_scale) {
    if (!has_ref_) {
        spdlog::warn("Initializer::tryInitialize called before reset()");
        return InitStatus::FAILED;
    }

    // ---- 1. match reference -> current (identity prior) ---------------------
    MatchesData matches;
    const MatchStatus mstat = matcher_.match(ref_features_, curr, matches, nullptr);
    if (mstat != MatchStatus::OK) {
        spdlog::debug("Initializer: match not ready (status={}, num_matches={})",
                      static_cast<int>(mstat), matches.num_matches);
        return InitStatus::NOT_READY;  // keep reference, wait for a later frame
    }

    // ---- 2. parallax gate (median disparity over matches) -------------------
    const double median_disp =
        medianMatchDisparity(ref_features_, curr, matches.matches);
    if (median_disp < config_.parallax_pixel_threshold) {
        spdlog::debug("Initializer: accumulating parallax "
                      "(median disparity {:.2f}px < {:.2f}px) — holding reference",
                      median_disp, config_.parallax_pixel_threshold);
        return InitStatus::NOT_READY;  // keep reference — root fix for spam
    }

    // ---- 3. recover relative pose ref -> curr (metric) ----------------------
    VOPoseData pose;
    const PoseStatus pstat =
        pose_estimator_.estimate(ref_features_, curr, matches, pose, metric_scale);
    if (pstat == PoseStatus::LOW_PARALLAX) {
        // Estimator's own (stricter) parallax gate — keep waiting.
        return InitStatus::NOT_READY;
    }
    if (pstat != PoseStatus::OK) {
        spdlog::debug("Initializer: pose estimation not ready (status={})",
                      static_cast<int>(pstat));
        return InitStatus::NOT_READY;
    }

    // ---- 4. triangulate inliers (P0 = K[I|0], P1 = K[R|t]) ------------------
    const Eigen::Matrix3d& K = camera_.K();
    cv::Mat K_cv;
    cv::eigen2cv(K, K_cv);

    cv::Mat R_cv, t_cv;
    cv::eigen2cv(pose.R_L_C, R_cv);
    cv::eigen2cv(pose.t_L_C, t_cv);

    cv::Mat P0 = cv::Mat::zeros(3, 4, CV_64F);
    K_cv.copyTo(P0(cv::Rect(0, 0, 3, 3)));  // K [I | 0]

    cv::Mat Rt1(3, 4, CV_64F);
    R_cv.copyTo(Rt1(cv::Rect(0, 0, 3, 3)));
    t_cv.reshape(1, 3).copyTo(Rt1(cv::Rect(3, 0, 1, 3)));
    cv::Mat P1 = K_cv * Rt1;                  // K [R | t]

    // Inlier correspondence pixels (from the estimator's inlier writeback).
    std::vector<cv::Point2f> in_ref, in_curr;
    std::vector<int> in_ref_kp, in_curr_kp;  // keypoint indices in ref / curr
    in_ref.reserve(matches.inliers.size());
    in_curr.reserve(matches.inliers.size());
    in_ref_kp.reserve(matches.inliers.size());
    in_curr_kp.reserve(matches.inliers.size());
    for (const cv::DMatch& m : matches.inliers) {
        in_ref.push_back(ref_features_.keypoints[m.queryIdx].pt);
        in_curr.push_back(curr.keypoints[m.trainIdx].pt);
        in_ref_kp.push_back(m.queryIdx);
        in_curr_kp.push_back(m.trainIdx);
    }
    if (static_cast<int>(in_ref.size()) < config_.min_triangulated_pts) {
        spdlog::debug("Initializer: too few inliers to triangulate ({} < {})",
                      in_ref.size(), config_.min_triangulated_pts);
        return InitStatus::NOT_READY;
    }

    cv::Mat pts4d_f;  // 4 x N homogeneous, world == KF0 camera frame
    cv::triangulatePoints(P0, P1, in_ref, in_curr, pts4d_f);
    // cv::triangulatePoints always returns CV_32F; convert so the at<double>
    // reads below are valid (reading float storage as double yields garbage).
    cv::Mat pts4d;
    pts4d_f.convertTo(pts4d, CV_64F);

    // ---- 5. count good points (cheirality in both cameras + reproj error) ---
    const Eigen::Matrix3d R = pose.R_L_C;
    const Eigen::Vector3d t = pose.t_L_C;
    const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);

    struct GoodPoint {
        Eigen::Vector3d pos_w;
        int ref_kp;
        int curr_kp;
    };
    std::vector<GoodPoint> good;
    good.reserve(in_ref.size());
    std::vector<double> depths;
    depths.reserve(in_ref.size());

    const int n = pts4d.cols;
    for (int i = 0; i < n; ++i) {
        const double w = pts4d.at<double>(3, i);
        if (std::abs(w) < 1e-9) {
            continue;
        }
        Eigen::Vector3d X(pts4d.at<double>(0, i) / w,
                          pts4d.at<double>(1, i) / w,
                          pts4d.at<double>(2, i) / w);

        // Cheirality: in front of KF0 camera (world == KF0 frame).
        if (X.z() <= 0.0) {
            continue;
        }
        // Cheirality in the curr camera frame: X_c = R * X + t.
        const Eigen::Vector3d Xc = R * X + t;
        if (Xc.z() <= 0.0) {
            continue;
        }

        // Reprojection error in KF0.
        const double u0 = fx * X.x() / X.z() + cx;
        const double v0 = fy * X.y() / X.z() + cy;
        const cv::Point2f& p0 = in_ref[static_cast<size_t>(i)];
        const double e0 = std::hypot(u0 - p0.x, v0 - p0.y);
        if (e0 > config_.max_reproj_error_px) {
            continue;
        }
        // Reprojection error in KF1 (curr).
        const double u1 = fx * Xc.x() / Xc.z() + cx;
        const double v1 = fy * Xc.y() / Xc.z() + cy;
        const cv::Point2f& p1 = in_curr[static_cast<size_t>(i)];
        const double e1 = std::hypot(u1 - p1.x, v1 - p1.y);
        if (e1 > config_.max_reproj_error_px) {
            continue;
        }

        good.push_back({X, in_ref_kp[static_cast<size_t>(i)],
                        in_curr_kp[static_cast<size_t>(i)]});
        depths.push_back(X.z());
    }

    if (static_cast<int>(good.size()) < config_.min_triangulated_pts) {
        spdlog::debug("Initializer: too few good triangulated points ({} < {}) "
                      "at median disparity {:.2f}px — holding reference",
                      good.size(), config_.min_triangulated_pts, median_disp);
        return InitStatus::NOT_READY;
    }

    // ---- 6. build the seed map (KF0 = identity, KF1 = T_prev_curr) ----------
    local_map_->clear();

    auto kf0 = std::make_shared<Keyframe>();
    kf0->id             = local_map_->nextKeyframeId();
    kf0->frame_id       = ref_features_.meta.frame_id;
    kf0->timestamp_msec = ref_features_.meta.timestamp_msec;
    kf0->T_wc           = Eigen::Matrix4d::Identity();
    kf0->features       = ref_features_;
    kf0->landmark_refs.assign(ref_features_.keypoints.size(), -1);
    kf0->telemetry      = ref_telemetry_;
    kf0->num_inliers    = matches.num_inliers;
    kf0->inlier_ratio   = matches.inlier_ratio;

    auto kf1 = std::make_shared<Keyframe>();
    kf1->id             = local_map_->nextKeyframeId();
    kf1->frame_id       = curr.meta.frame_id;
    kf1->timestamp_msec = curr.meta.timestamp_msec;
    kf1->T_wc           = pose.T_prev_curr;  // camera->world of KF1 (KF0 = world)
    kf1->features       = curr;
    kf1->landmark_refs.assign(curr.keypoints.size(), -1);
    kf1->telemetry      = curr_tel;
    kf1->num_inliers    = matches.num_inliers;
    kf1->inlier_ratio   = matches.inlier_ratio;

    for (const GoodPoint& gp : good) {
        auto lm = std::make_shared<Landmark>();
        lm->id    = local_map_->nextLandmarkId();
        lm->pos_w = gp.pos_w;  // world == KF0 camera frame, metric
        // Representative descriptor: ORB of the observation in curr (KF1).
        if (gp.curr_kp >= 0 && gp.curr_kp < curr.descriptors.rows) {
            lm->descriptor = curr.descriptors.row(gp.curr_kp).clone();
        }
        lm->observations.push_back({kf0->id, static_cast<size_t>(gp.ref_kp)});
        lm->observations.push_back({kf1->id, static_cast<size_t>(gp.curr_kp)});
        lm->obs_count   = static_cast<int>(lm->observations.size());
        lm->num_visible = 2;
        lm->num_found   = 2;
        lm->state       = LandmarkState::CONVERGED;

        kf0->landmark_refs[static_cast<size_t>(gp.ref_kp)]  = static_cast<int64_t>(lm->id);
        kf1->landmark_refs[static_cast<size_t>(gp.curr_kp)] = static_cast<int64_t>(lm->id);

        local_map_->addLandmark(lm);
    }

    local_map_->addKeyframe(kf0);
    local_map_->addKeyframe(kf1);

    // Median seed depth (Z in KF0 frame).
    const size_t mid = depths.size() / 2;
    std::nth_element(depths.begin(), depths.begin() + mid, depths.end());
    median_seed_depth_ = depths[mid];

    T_prev_curr_ = pose.T_prev_curr;
    num_seeded_  = static_cast<int>(good.size());
    // Report-only: ref->curr matches that produced this seed (inlier-ratio
    // denominator). Does not influence the pose/triangulation above.
    num_matches_ = static_cast<int>(matches.num_matches);
    initialized_ = true;

    spdlog::debug("Initializer: SUCCESS — {} landmarks, median depth {:.1f} m, "
                  "median disparity {:.2f}px",
                  num_seeded_, median_seed_depth_, median_disp);
    return InitStatus::SUCCESS;
}

}  // namespace uavloc::vo
