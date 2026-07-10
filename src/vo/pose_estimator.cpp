#include "uavloc/vo/pose_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/LU>  // Matrix4d::inverse()
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>

namespace uavloc::vo {

namespace {

// Deterministic RANSAC seeds for reproducibility (CLAUDE.md "RNG seeding").
// Homography path reuses the documented "H thread" constant; the essential
// fallback uses the "F thread" constant so the two paths never share a stream.
constexpr uint64_t HOMOGRAPHY_RNG_SEED = 0xABCD1234ull;
constexpr uint64_t ESSENTIAL_RNG_SEED  = 0x5678EF00ull;

// minimum correspondences cv::findHomography needs to fit a 3x3 homography.
constexpr int MIN_POINTS_FOR_HOMOGRAPHY = 4;

// Degrees-per-radian for the nadir-angle gate.
constexpr double DEG_PER_RAD = 180.0 / M_PI;

// Expected ground-plane normal in the camera frame for a nadir-looking UAV
// camera: the optical axis (+Z). decomposeHomographyMat candidates whose normal
// deviates from this by more than nadir_normal_max_angle_deg are rejected.
const cv::Vec3d NADIR_NORMAL(0.0, 0.0, 1.0);

}  // namespace

// ---------------------------------------------------------------------------
// PoseEstimatorConfig
// ---------------------------------------------------------------------------
PoseEstimatorConfig PoseEstimatorConfig::fromYaml(const YAML::Node& node) {
    PoseEstimatorConfig cfg;  // start from defaults

    if (!node) {
        return cfg;
    }

    cfg.ransac_reproj_threshold_px =
        node["ransac_reproj_threshold_px"].as<double>(cfg.ransac_reproj_threshold_px);
    cfg.ransac_confidence =
        node["ransac_confidence"].as<double>(cfg.ransac_confidence);
    cfg.ransac_max_iters =
        node["ransac_max_iters"].as<int>(cfg.ransac_max_iters);

    cfg.min_matches_for_homography =
        node["min_matches_for_homography"].as<int>(cfg.min_matches_for_homography);
    cfg.min_inliers =
        node["min_inliers"].as<int>(cfg.min_inliers);
    cfg.min_inlier_ratio =
        node["min_inlier_ratio"].as<double>(cfg.min_inlier_ratio);

    cfg.nadir_normal_max_angle_deg =
        node["nadir_normal_max_angle_deg"].as<double>(cfg.nadir_normal_max_angle_deg);

    cfg.min_parallax_px =
        node["min_parallax_px"].as<double>(cfg.min_parallax_px);

    cfg.use_essential_fallback =
        node["use_essential_fallback"].as<bool>(cfg.use_essential_fallback);

    return cfg;
}

// ---------------------------------------------------------------------------
// PoseEstimator
// ---------------------------------------------------------------------------
PoseEstimator::PoseEstimator(const PoseEstimatorConfig& config,
                             const sensor::CameraModel& camera)
    : config_(config) {
    // Convert the Eigen intrinsics to OpenCV form at the matrix<->image seam
    // (coding.md: cv::eigen2cv only where an OpenCV algorithm consumes the
    // numeric result). findHomography / decomposeHomographyMat want cv::Matx33d.
    cv::Mat K_cv;
    cv::eigen2cv(camera.K(), K_cv);
    K_ = cv::Matx33d(K_cv);
}

const PoseEstimatorConfig& PoseEstimator::config() const {
    return config_;
}

namespace {

// Build the matched pixel-coordinate vectors from the two feature sets.
// matches use queryIdx -> ref keypoint, trainIdx -> curr keypoint.
bool buildPointPairs(const FeatureSet& ref,
                     const FeatureSet& curr,
                     const MatchesData& matches,
                     std::vector<cv::Point2f>& pts_ref,
                     std::vector<cv::Point2f>& pts_curr) {
    const int n_ref  = static_cast<int>(ref.keypoints.size());
    const int n_curr = static_cast<int>(curr.keypoints.size());
    pts_ref.reserve(matches.matches.size());
    pts_curr.reserve(matches.matches.size());
    for (const cv::DMatch& m : matches.matches) {
        if (m.queryIdx < 0 || m.queryIdx >= n_ref ||
            m.trainIdx < 0 || m.trainIdx >= n_curr) {
            return false;  // index out of range -> size mismatch / bad input
        }
        pts_ref.push_back(ref.keypoints[m.queryIdx].pt);
        pts_curr.push_back(curr.keypoints[m.trainIdx].pt);
    }
    return true;
}

// Write geometric inliers carried in `mask` back into `matches`.
void writeInliers(const cv::Mat& mask, MatchesData& matches) {
    const int n = static_cast<int>(matches.matches.size());
    matches.inlier_mask.assign(static_cast<size_t>(n), 0);
    matches.inliers.clear();
    matches.inliers.reserve(static_cast<size_t>(n));

    int num_inliers = 0;
    const bool has_mask = (!mask.empty() && mask.rows == n);
    for (int i = 0; i < n; ++i) {
        const bool is_inlier = has_mask ? (mask.at<uchar>(i) != 0) : true;
        if (is_inlier) {
            matches.inlier_mask[static_cast<size_t>(i)] = 1;
            matches.inliers.push_back(matches.matches[static_cast<size_t>(i)]);
            ++num_inliers;
        }
    }
    matches.num_inliers  = num_inliers;
    matches.inlier_ratio = (n > 0) ? static_cast<double>(num_inliers) /
                                         static_cast<double>(n)
                                   : 0.0;
}

// Angle (degrees) between a decomposition candidate normal and the nadir axis.
double nadirAngleDeg(const cv::Mat& normal) {
    cv::Vec3d n(normal.at<double>(0), normal.at<double>(1), normal.at<double>(2));
    const double nn = cv::norm(n);
    if (nn < std::numeric_limits<double>::epsilon()) {
        return 180.0;
    }
    double c = n.dot(NADIR_NORMAL) / nn;
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c) * DEG_PER_RAD;
}

// Pick the nadir-consistent (R, t, n) candidate among the homography
// decompositions, restricted to `candidates` (the cheirality-feasible subset
// returned by cv::filterHomographyDecompByVisibleRefpoints). Returns the index
// into the full decomposition arrays, or -1 when no candidate's plane normal
// lies within the configured cone of the optical axis.
int selectNadirSolution(const std::vector<cv::Mat>& normals,
                        const std::vector<int>& candidates,
                        double max_angle_deg) {
    int best = -1;
    double best_angle = max_angle_deg;
    for (int idx : candidates) {
        if (idx < 0 || idx >= static_cast<int>(normals.size())) {
            continue;
        }
        const double angle = nadirAngleDeg(normals[static_cast<size_t>(idx)]);
        if (angle <= best_angle) {
            best_angle = angle;
            best = idx;
        }
    }
    return best;
}

// Median pixel disparity over the geometric inliers. Two-view pose recovery is
// degenerate when this collapses (near-stationary camera), so the caller gates
// on it before trusting the decomposition.
double medianInlierDisparity(const std::vector<cv::Point2f>& pts_ref,
                             const std::vector<cv::Point2f>& pts_curr,
                             const cv::Mat& mask) {
    const int n = static_cast<int>(pts_ref.size());
    const bool has_mask = (!mask.empty() && mask.rows == n);
    std::vector<double> disp;
    disp.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        if (has_mask && mask.at<uchar>(i) == 0) {
            continue;
        }
        const cv::Point2f d = pts_curr[static_cast<size_t>(i)] -
                              pts_ref[static_cast<size_t>(i)];
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

// Collect only the inlier correspondences (per `mask`) into separate vectors —
// cv::filterHomographyDecompByVisibleRefpoints must be fed inliers only so that
// out-of-front points do not poison the cheirality vote.
void collectInlierPoints(const std::vector<cv::Point2f>& pts_ref,
                         const std::vector<cv::Point2f>& pts_curr,
                         const cv::Mat& mask,
                         std::vector<cv::Point2f>& in_ref,
                         std::vector<cv::Point2f>& in_curr) {
    const int n = static_cast<int>(pts_ref.size());
    const bool has_mask = (!mask.empty() && mask.rows == n);
    in_ref.clear();
    in_curr.clear();
    in_ref.reserve(static_cast<size_t>(n));
    in_curr.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        if (has_mask && mask.at<uchar>(i) == 0) {
            continue;
        }
        in_ref.push_back(pts_ref[static_cast<size_t>(i)]);
        in_curr.push_back(pts_curr[static_cast<size_t>(i)]);
    }
}

// Fill out_pose from an OpenCV rotation/translation pair.
//
// metric_scale <= 0  -> translation kept as a unit direction (monocular
//                       up-to-scale, the legacy behaviour).
// metric_scale  > 0  -> translation promoted to metres. The interpretation of
//   metric_scale depends on what t_cv carries:
//   * decompose_t_is_ratio == true  (cv::decomposeHomographyMat): t already
//     equals baseline/d_plane, so the metric baseline is t * d_plane; the caller
//     passes metric_scale = d_plane (slant range to the ground). DO NOT normalise
//     first — that would discard the baseline/d ratio and over-scale every step.
//   * decompose_t_is_ratio == false (cv::recoverPose, essential): t is a pure
//     unit direction with no magnitude, so normalise then scale by metric_scale.
void fillPose(const cv::Mat& R_cv, const cv::Mat& t_cv, double metric_scale,
              bool decompose_t_is_ratio, VOPoseData& out_pose) {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    cv::cv2eigen(R_cv, R);
    cv::cv2eigen(t_cv, t);

    if (metric_scale > 0.0) {
        if (decompose_t_is_ratio) {
            t *= metric_scale;  // baseline = (baseline/d_plane) * d_plane
        } else {
            const double tn = t.norm();
            if (tn > std::numeric_limits<double>::epsilon()) {
                t /= tn;        // direction only -> unit
            }
            t *= metric_scale;  // then promote to metres
        }
    } else {
        const double tn = t.norm();
        if (tn > std::numeric_limits<double>::epsilon()) {
            t /= tn;            // up-to-scale unit direction (legacy)
        }
    }

    out_pose.R_L_C = R;  // world->camera (T_cw): KEPT for triangulation
    out_pose.t_L_C = t;
    // [R|t] = T_cw_curr (world == prev). CLAUDE.md defines T_prev_curr as the
    // camera->world increment of curr w.r.t. prev (with KF0 = I that is T_wc_KF1),
    // so it is the INVERSE of [R|t]. Building it as [R|t] directly inverted the
    // whole system (Fix #2).
    Eigen::Matrix4d T_cw = Eigen::Matrix4d::Identity();
    T_cw.block<3, 3>(0, 0) = R;
    T_cw.block<3, 1>(0, 3) = t;
    out_pose.T_prev_curr = T_cw.inverse();  // camera->world (T_wc_KF1, KF0 = I)
}

// Optional essential-matrix fallback for non-planar / high-parallax geometry.
// Kept clearly separated from the homography-first primary path. Returns OK and
// fills out_pose / matches on success.
PoseStatus estimateEssential(const cv::Matx33d& K,
                             const PoseEstimatorConfig& cfg,
                             const std::vector<cv::Point2f>& pts_ref,
                             const std::vector<cv::Point2f>& pts_curr,
                             double metric_scale,
                             MatchesData& matches,
                             VOPoseData& out_pose) {
    cv::theRNG() = cv::RNG(ESSENTIAL_RNG_SEED);  // reproducible RANSAC
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts_ref, pts_curr, cv::Mat(K), cv::RANSAC,
                                     cfg.ransac_confidence,
                                     cfg.ransac_reproj_threshold_px, mask);
    if (E.empty() || !cv::checkRange(E)) {
        spdlog::warn("PoseEstimator: essential fallback failed (empty/non-finite E)");
        return PoseStatus::HOMOGRAPHY_FAILED;
    }

    cv::Mat R_cv, t_cv;
    const int n_in = cv::recoverPose(E, pts_ref, pts_curr, cv::Mat(K),
                                     R_cv, t_cv, mask);

    writeInliers(mask, matches);

    if (matches.num_inliers < cfg.min_inliers ||
        matches.inlier_ratio < cfg.min_inlier_ratio || n_in <= 0) {
        spdlog::warn("PoseEstimator: essential fallback inliers too few "
                     "({} / ratio {:.3f})",
                     matches.num_inliers, matches.inlier_ratio);
        return PoseStatus::NOT_ENOUGH_INLIERS;
    }

    fillPose(R_cv, t_cv, metric_scale, /*decompose_t_is_ratio=*/false, out_pose);
    spdlog::debug("PoseEstimator: essential fallback OK ({} inliers, ratio {:.3f})",
                  matches.num_inliers, matches.inlier_ratio);
    return PoseStatus::OK;
}

}  // namespace

PoseStatus PoseEstimator::estimate(const FeatureSet& ref,
                                   const FeatureSet& curr,
                                   MatchesData& matches,
                                   VOPoseData& out_pose,
                                   double metric_scale) {
    // ---- input validation ---------------------------------------------------
    if (!K_(0, 0) || !K_(1, 1)) {  // fx / fy must be non-zero
        spdlog::error("PoseEstimator: invalid intrinsics (fx={}, fy={})",
                      K_(0, 0), K_(1, 1));
        return PoseStatus::ERROR;
    }

    // ---- match-count gate ----------------------------------------------------
    if (matches.num_matches < config_.min_matches_for_homography) {
        spdlog::warn("PoseEstimator: not enough matches ({} < {})",
                     matches.num_matches, config_.min_matches_for_homography);
        return PoseStatus::NOT_ENOUGH_MATCHES;
    }

    // ---- build correspondences ----------------------------------------------
    std::vector<cv::Point2f> pts_ref, pts_curr;
    if (!buildPointPairs(ref, curr, matches, pts_ref, pts_curr)) {
        spdlog::error("PoseEstimator: match index out of range (size mismatch)");
        return PoseStatus::ERROR;
    }
    if (static_cast<int>(pts_ref.size()) < MIN_POINTS_FOR_HOMOGRAPHY) {
        spdlog::warn("PoseEstimator: too few point pairs ({}) for homography",
                     pts_ref.size());
        return PoseStatus::NOT_ENOUGH_MATCHES;
    }

    // ---- homography RANSAC ---------------------------------------------------
    cv::theRNG() = cv::RNG(HOMOGRAPHY_RNG_SEED);  // reproducible RANSAC
    cv::Mat mask;
    cv::Mat H = cv::findHomography(pts_ref, pts_curr, cv::RANSAC,
                                   config_.ransac_reproj_threshold_px, mask,
                                   config_.ransac_max_iters,
                                   config_.ransac_confidence);

    const bool homography_ok = (!H.empty() && cv::checkRange(H));
    if (!homography_ok) {
        spdlog::warn("PoseEstimator: cv::findHomography failed (empty/non-finite)");
        if (config_.use_essential_fallback) {
            return estimateEssential(K_, config_, pts_ref, pts_curr, metric_scale,
                                     matches, out_pose);
        }
        return PoseStatus::HOMOGRAPHY_FAILED;
    }

    // Geometric inliers are written back regardless of the downstream decision
    // so call sites (and the matcher contract) always see populated fields.
    writeInliers(mask, matches);

    // ---- inlier gate ---------------------------------------------------------
    if (matches.num_inliers < config_.min_inliers ||
        matches.inlier_ratio < config_.min_inlier_ratio) {
        spdlog::warn("PoseEstimator: not enough inliers ({} >= {} ? ratio {:.3f} "
                     ">= {:.3f} ?)",
                     matches.num_inliers, config_.min_inliers,
                     matches.inlier_ratio, config_.min_inlier_ratio);
        if (config_.use_essential_fallback) {
            return estimateEssential(K_, config_, pts_ref, pts_curr, metric_scale,
                                     matches, out_pose);
        }
        return PoseStatus::NOT_ENOUGH_INLIERS;
    }

    // ---- parallax gate -------------------------------------------------------
    // A near-stationary camera produces a degenerate two-view geometry; recover
    // nothing rather than chain a noisy step. Caller holds the previous pose.
    const double median_disp = medianInlierDisparity(pts_ref, pts_curr, mask);
    if (median_disp < config_.min_parallax_px) {
        spdlog::debug("PoseEstimator: low parallax (median disparity {:.2f}px < "
                      "{:.2f}px) — skipping frame",
                      median_disp, config_.min_parallax_px);
        return PoseStatus::LOW_PARALLAX;
    }

    // ---- nadir-aware decomposition ------------------------------------------
    std::vector<cv::Mat> Rs, ts, normals;
    const int n_sol = cv::decomposeHomographyMat(H, cv::Mat(K_), Rs, ts, normals);
    if (n_sol <= 0 || n_sol > 4) {
        spdlog::warn("PoseEstimator: decomposeHomographyMat returned {} solutions "
                     "(expected 1..4)", n_sol);
        if (config_.use_essential_fallback) {
            return estimateEssential(K_, config_, pts_ref, pts_curr, metric_scale,
                                     matches, out_pose);
        }
        return PoseStatus::DECOMPOSITION_FAILED;
    }

    // ---- cheirality filter (REL-2/4) ----------------------------------------
    // Keep only decompositions whose reference points stay in front of both
    // cameras. Feed INLIERS only. Falls back to all candidates if the filter
    // yields nothing (degenerate vote) so we never silently drop a valid frame.
    std::vector<cv::Point2f> in_ref, in_curr;
    collectInlierPoints(pts_ref, pts_curr, mask, in_ref, in_curr);

    std::vector<int> candidates;
    if (in_ref.size() >= static_cast<size_t>(MIN_POINTS_FOR_HOMOGRAPHY)) {
        cv::filterHomographyDecompByVisibleRefpoints(Rs, normals, in_ref, in_curr,
                                                     candidates);
    }
    if (candidates.empty()) {
        spdlog::warn("PoseEstimator: cheirality filter found no feasible "
                     "decomposition; considering all {} candidates", n_sol);
        candidates.resize(static_cast<size_t>(n_sol));
        for (int i = 0; i < n_sol; ++i) {
            candidates[static_cast<size_t>(i)] = i;
        }
    }

    const int sel = selectNadirSolution(normals, candidates,
                                        config_.nadir_normal_max_angle_deg);
    if (sel < 0) {
        spdlog::warn("PoseEstimator: no nadir-consistent decomposition (max angle "
                     "{:.1f} deg)",
                     config_.nadir_normal_max_angle_deg);
        if (config_.use_essential_fallback) {
            return estimateEssential(K_, config_, pts_ref, pts_curr, metric_scale,
                                     matches, out_pose);
        }
        return PoseStatus::DECOMPOSITION_FAILED;
    }

    fillPose(Rs[static_cast<size_t>(sel)], ts[static_cast<size_t>(sel)],
             metric_scale, /*decompose_t_is_ratio=*/true, out_pose);
    spdlog::debug("PoseEstimator: homography OK ({} inliers, ratio {:.3f}, "
                  "nadir angle {:.1f} deg)",
                  matches.num_inliers, matches.inlier_ratio,
                  nadirAngleDeg(normals[static_cast<size_t>(sel)]));
    return PoseStatus::OK;
}

}  // namespace uavloc::vo
