#include "uavloc/vo/projection_matcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>          // MatrixBase::determinant() definition (Eigen/LU)
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>

// GMS lives in OpenCV-contrib (xfeatures2d). Guard the include so the build
// never breaks when contrib is absent; when missing and use_gms is requested we
// log a warning once and skip the filter (graceful degradation, design §6.5).
#if __has_include(<opencv2/xfeatures2d.hpp>)
#  include <opencv2/xfeatures2d.hpp>
#  define UAVLOC_HAS_GMS 1
#else
#  define UAVLOC_HAS_GMS 0
#endif

namespace uavloc::vo {

// ---------------------------------------------------------------------------
// ProjectionMatcherConfig
// ---------------------------------------------------------------------------
ProjectionMatcherConfig ProjectionMatcherConfig::fromYaml(const YAML::Node& node) {
    ProjectionMatcherConfig cfg;  // start from defaults

    if (!node) {
        return cfg;
    }

    cfg.nn_ratio               = node["nn_ratio"].as<float>(cfg.nn_ratio);
    cfg.max_hamming_distance   = node["max_hamming_distance"].as<int>(cfg.max_hamming_distance);
    cfg.min_matches            = node["min_matches"].as<int>(cfg.min_matches);
    cfg.search_radius_px       = node["search_radius_px"].as<float>(cfg.search_radius_px);
    cfg.use_orientation_filter = node["use_orientation_filter"].as<bool>(cfg.use_orientation_filter);
    cfg.orientation_hist_bins  = node["orientation_hist_bins"].as<int>(cfg.orientation_hist_bins);
    cfg.orientation_top_bins   = node["orientation_top_bins"].as<int>(cfg.orientation_top_bins);
    cfg.use_gms                = node["use_gms"].as<bool>(cfg.use_gms);
    cfg.gms_threshold_factor   = node["gms_threshold_factor"].as<float>(cfg.gms_threshold_factor);
    cfg.use_cuda               = node["use_cuda"].as<bool>(cfg.use_cuda);

    return cfg;
}

namespace {

// Degrees in a full turn — used to wrap orientation differences before binning.
constexpr float FULL_TURN_DEG = 360.0f;

// k for the kNN match (best + second-best for the Lowe ratio test).
constexpr int KNN_K = 2;

// Hamming distance returned by the brute-force matcher is bounded by the
// descriptor bit width (32-byte ORB = 256 bits); used as a "no candidate"
// sentinel comparison upper bound.
constexpr float HAMMING_MAX = 256.0f;

// Wrap an angle difference into [0, 360).
float wrap360(float deg) {
    deg = std::fmod(deg, FULL_TURN_DEG);
    if (deg < 0.0f) {
        deg += FULL_TURN_DEG;
    }
    return deg;
}

// Project a reference pixel through the homography prior (p_curr ~= H * p_ref).
cv::Point2f projectPoint(const Eigen::Matrix3d& H, const cv::Point2f& p) {
    Eigen::Vector3d hp = H * Eigen::Vector3d(p.x, p.y, 1.0);
    if (std::abs(hp.z()) < std::numeric_limits<double>::epsilon()) {
        return p;  // degenerate projection; fall back to the source pixel
    }
    return cv::Point2f(static_cast<float>(hp.x() / hp.z()),
                       static_cast<float>(hp.y() / hp.z()));
}

// A homography prior is usable only when every entry is finite and the matrix is
// non-singular.
bool isUsablePrior(const Eigen::Matrix3d& H) {
    if (!H.allFinite()) {
        return false;
    }
    return std::abs(H.determinant()) > std::numeric_limits<double>::epsilon();
}

// Uniform spatial grid over the current keypoints for O(1) neighbourhood
// queries. Cell side length equals the search radius so a query touches a 3x3
// block of cells at most.
class KeypointGrid {
public:
    KeypointGrid(const std::vector<cv::KeyPoint>& kps, float cell_size)
        : cell_size_(std::max(cell_size, 1.0f)) {
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        for (const cv::KeyPoint& kp : kps) {
            min_x = std::min(min_x, kp.pt.x);
            min_y = std::min(min_y, kp.pt.y);
            max_x = std::max(max_x, kp.pt.x);
            max_y = std::max(max_y, kp.pt.y);
        }
        origin_x_ = min_x;
        origin_y_ = min_y;
        cols_ = std::max(1, static_cast<int>((max_x - min_x) / cell_size_) + 1);
        rows_ = std::max(1, static_cast<int>((max_y - min_y) / cell_size_) + 1);

        cells_.resize(static_cast<size_t>(cols_) * static_cast<size_t>(rows_));
        for (int i = 0; i < static_cast<int>(kps.size()); ++i) {
            int cx, cy;
            cellOf(kps[i].pt, cx, cy);
            cells_[static_cast<size_t>(cy) * cols_ + cx].push_back(i);
        }
    }

    // Append indices of all keypoints within `radius` of `center` into `out`.
    void query(const cv::Point2f& center, float radius,
               std::vector<int>& out) const {
        int cx, cy;
        cellOf(center, cx, cy);
        const int span = static_cast<int>(radius / cell_size_) + 1;
        const float r2 = radius * radius;
        for (int gy = cy - span; gy <= cy + span; ++gy) {
            if (gy < 0 || gy >= rows_) continue;
            for (int gx = cx - span; gx <= cx + span; ++gx) {
                if (gx < 0 || gx >= cols_) continue;
                for (int idx : cells_[static_cast<size_t>(gy) * cols_ + gx]) {
                    const float dx = kp_x_[static_cast<size_t>(idx)] - center.x;
                    const float dy = kp_y_[static_cast<size_t>(idx)] - center.y;
                    if (dx * dx + dy * dy <= r2) {
                        out.push_back(idx);
                    }
                }
            }
        }
    }

    void cacheCoords(const std::vector<cv::KeyPoint>& kps) {
        kp_x_.reserve(kps.size());
        kp_y_.reserve(kps.size());
        for (const cv::KeyPoint& kp : kps) {
            kp_x_.push_back(kp.pt.x);
            kp_y_.push_back(kp.pt.y);
        }
    }

private:
    void cellOf(const cv::Point2f& p, int& cx, int& cy) const {
        cx = static_cast<int>((p.x - origin_x_) / cell_size_);
        cy = static_cast<int>((p.y - origin_y_) / cell_size_);
        cx = std::min(std::max(cx, 0), cols_ - 1);
        cy = std::min(std::max(cy, 0), rows_ - 1);
    }

    float cell_size_;
    float origin_x_ = 0.0f;
    float origin_y_ = 0.0f;
    int   cols_ = 1;
    int   rows_ = 1;
    std::vector<std::vector<int>> cells_;
    std::vector<float> kp_x_;
    std::vector<float> kp_y_;
};

// Hamming distance between two 32-byte ORB descriptor rows.
int hammingDistance(const cv::Mat& a, int ra, const cv::Mat& b, int rb) {
    return static_cast<int>(cv::norm(a.row(ra), b.row(rb), cv::NORM_HAMMING));
}

// Keep, for each current keypoint, only the lowest-Hamming claiming ref point
// (many-to-one dedup guard). Operates in place on `matches`.
void deduplicateByTrain(std::vector<cv::DMatch>& matches) {
    std::unordered_map<int, int> best_for_train;  // trainIdx -> position in matches
    best_for_train.reserve(matches.size());
    std::vector<cv::DMatch> kept;
    kept.reserve(matches.size());
    for (const cv::DMatch& m : matches) {
        auto it = best_for_train.find(m.trainIdx);
        if (it == best_for_train.end()) {
            best_for_train.emplace(m.trainIdx, static_cast<int>(kept.size()));
            kept.push_back(m);
        } else if (m.distance < kept[it->second].distance) {
            kept[it->second] = m;
        }
    }
    matches.swap(kept);
}

// Orientation-consistency histogram filter (Mur-Artal et al.). Keeps only the
// matches whose orientation delta falls in the top-N most-populated bins.
void orientationFilter(const std::vector<cv::KeyPoint>& ref_kps,
                       const std::vector<cv::KeyPoint>& curr_kps,
                       const ProjectionMatcherConfig& cfg,
                       std::vector<cv::DMatch>& matches) {
    if (matches.empty() || cfg.orientation_hist_bins <= 0 ||
        cfg.orientation_top_bins <= 0) {
        return;
    }

    const int bins = cfg.orientation_hist_bins;
    const float bin_width = FULL_TURN_DEG / static_cast<float>(bins);

    std::vector<std::vector<int>> hist(static_cast<size_t>(bins));
    for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
        const cv::DMatch& m = matches[i];
        const float dtheta = wrap360(curr_kps[m.trainIdx].angle - ref_kps[m.queryIdx].angle);
        int b = static_cast<int>(dtheta / bin_width);
        if (b >= bins) b = bins - 1;
        if (b < 0) b = 0;
        hist[static_cast<size_t>(b)].push_back(i);
    }

    // Rank bins by population; keep the top-N.
    std::vector<int> bin_order(static_cast<size_t>(bins));
    for (int b = 0; b < bins; ++b) bin_order[static_cast<size_t>(b)] = b;
    const int keep_n = std::min(cfg.orientation_top_bins, bins);
    std::partial_sort(bin_order.begin(), bin_order.begin() + keep_n, bin_order.end(),
                      [&hist](int a, int b) {
                          return hist[static_cast<size_t>(a)].size() >
                                 hist[static_cast<size_t>(b)].size();
                      });

    std::vector<cv::DMatch> kept;
    kept.reserve(matches.size());
    for (int k = 0; k < keep_n; ++k) {
        for (int idx : hist[static_cast<size_t>(bin_order[static_cast<size_t>(k)])]) {
            kept.push_back(matches[idx]);
        }
    }
    matches.swap(kept);
}

// Optional GMS filter (Bian et al.). Returns true if it ran, false if it was
// requested but unavailable (caller then keeps the unfiltered matches).
bool gmsFilter(const FeatureSet& ref, const FeatureSet& curr,
               const ProjectionMatcherConfig& cfg,
               std::vector<cv::DMatch>& matches) {
#if UAVLOC_HAS_GMS
    if (matches.empty()) {
        return true;
    }
    // GMS needs image dimensions; estimate a bounding size from keypoint spread.
    auto sizeFromKps = [](const std::vector<cv::KeyPoint>& kps) {
        float max_x = 1.0f, max_y = 1.0f;
        for (const cv::KeyPoint& kp : kps) {
            max_x = std::max(max_x, kp.pt.x);
            max_y = std::max(max_y, kp.pt.y);
        }
        return cv::Size(static_cast<int>(max_x) + 1, static_cast<int>(max_y) + 1);
    };
    std::vector<cv::DMatch> gms_matches;
    cv::xfeatures2d::matchGMS(sizeFromKps(ref.keypoints), sizeFromKps(curr.keypoints),
                              ref.keypoints, curr.keypoints, matches, gms_matches,
                              /*withRotation=*/false, /*withScale=*/false,
                              static_cast<double>(cfg.gms_threshold_factor));
    matches.swap(gms_matches);
    return true;
#else
    (void)ref;
    (void)curr;
    (void)cfg;
    (void)matches;
    return false;
#endif
}

}  // namespace

// ---------------------------------------------------------------------------
// ProjectionMatcher
// ---------------------------------------------------------------------------
ProjectionMatcher::ProjectionMatcher(const ProjectionMatcherConfig& config)
    : config_(config) {
    // CPU brute-force Hamming matcher; crossCheck off so kNN (k=2) is available
    // for the Lowe ratio test. A CUDA path is selected at construction when
    // requested and OpenCV is built with CUDA; otherwise CPU.
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, /*crossCheck=*/false);
    if (config_.use_cuda) {
        // OpenCV's cv::cuda::DescriptorMatcher is not exposed through the same
        // cv::BFMatcher type; without a guaranteed CUDA build we stay on CPU.
        spdlog::info("ProjectionMatcher: backend=CPU (CUDA requested but not "
                     "enabled in this build)");
    } else {
        spdlog::info("ProjectionMatcher: backend=CPU (NORM_HAMMING brute-force)");
    }
}

const ProjectionMatcherConfig& ProjectionMatcher::config() const {
    return config_;
}

MatchStatus ProjectionMatcher::match(const FeatureSet& ref,
                                     const FeatureSet& curr,
                                     MatchesData& out,
                                     const Eigen::Matrix3d* H_ref_curr_prior) {
    // ---- 6.1 common pre-checks ----------------------------------------------
    out.ref_frame_id  = ref.meta.frame_id;
    out.curr_frame_id = curr.meta.frame_id;
    out.matches.clear();
    out.inliers.clear();
    out.inlier_mask.clear();
    out.num_matches  = 0;
    out.num_inliers  = 0;
    out.inlier_ratio = 0.0;

    if (ref.keypoints.empty() || curr.keypoints.empty()) {
        spdlog::warn("ProjectionMatcher: empty input (ref {} kps, curr {} kps)",
                     ref.keypoints.size(), curr.keypoints.size());
        return MatchStatus::EMPTY_INPUT;
    }
    if (!ref.hasDescriptors() || !curr.hasDescriptors() ||
        ref.descriptor_type != DescriptorType::ORB ||
        curr.descriptor_type != DescriptorType::ORB) {
        spdlog::warn("ProjectionMatcher: missing/non-ORB descriptors "
                     "(ref empty={}, curr empty={})",
                     !ref.hasDescriptors(), !curr.hasDescriptors());
        return MatchStatus::NO_DESCRIPTORS;
    }

    // ---- 6.2 windowed search-by-projection ----------------------------------
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    if (H_ref_curr_prior != nullptr) {
        if (isUsablePrior(*H_ref_curr_prior)) {
            H = *H_ref_curr_prior;
        } else {
            spdlog::warn("ProjectionMatcher: unusable homography prior; "
                         "falling back to identity prior");
        }
    } else {
        spdlog::debug("ProjectionMatcher: no prior given; using identity prior");
    }

    std::vector<cv::DMatch> windowed;
    {
        KeypointGrid grid(curr.keypoints, config_.search_radius_px);
        grid.cacheCoords(curr.keypoints);

        std::vector<int> cand;
        windowed.reserve(ref.keypoints.size());
        for (int i = 0; i < static_cast<int>(ref.keypoints.size()); ++i) {
            const cv::Point2f proj = projectPoint(H, ref.keypoints[i].pt);
            cand.clear();
            grid.query(proj, config_.search_radius_px, cand);
            if (cand.empty()) {
                continue;
            }

            int best_idx = -1, second_idx = -1;
            float best_dist = HAMMING_MAX, second_dist = HAMMING_MAX;
            for (int j : cand) {
                const int d = hammingDistance(ref.descriptors, i, curr.descriptors, j);
                if (d < best_dist) {
                    second_dist = best_dist;
                    second_idx  = best_idx;
                    best_dist   = static_cast<float>(d);
                    best_idx    = j;
                } else if (d < second_dist) {
                    second_dist = static_cast<float>(d);
                    second_idx  = j;
                }
            }

            if (best_idx < 0 || best_dist > static_cast<float>(config_.max_hamming_distance)) {
                continue;
            }
            // Ratio gate within the window (skip if there is no second-best).
            if (second_idx >= 0 && best_dist >= config_.nn_ratio * second_dist) {
                continue;
            }
            windowed.emplace_back(i, best_idx, best_dist);
        }
    }
    const size_t windowed_raw = windowed.size();

    deduplicateByTrain(windowed);
    if (config_.use_orientation_filter) {
        orientationFilter(ref.keypoints, curr.keypoints, config_, windowed);
    }
    if (config_.use_gms) {
        if (!gmsFilter(ref, curr, config_, windowed)) {
            spdlog::warn("ProjectionMatcher: use_gms requested but OpenCV-contrib "
                         "GMS unavailable at build time; skipping");
        }
    }
    spdlog::debug("ProjectionMatcher: windowed raw={} -> filtered={}",
                  windowed_raw, windowed.size());

    std::vector<cv::DMatch>* chosen = &windowed;

    // ---- 6.3 global brute-force fallback ------------------------------------
    std::vector<cv::DMatch> global;
    if (static_cast<int>(windowed.size()) < config_.min_matches) {
        std::vector<std::vector<cv::DMatch>> knn;
        matcher_->knnMatch(ref.descriptors, curr.descriptors, knn, KNN_K);

        global.reserve(knn.size());
        for (const std::vector<cv::DMatch>& m : knn) {
            if (m.empty()) {
                continue;
            }
            const cv::DMatch& best = m[0];
            if (best.distance > static_cast<float>(config_.max_hamming_distance)) {
                continue;
            }
            if (m.size() >= 2 && best.distance >= config_.nn_ratio * m[1].distance) {
                continue;
            }
            global.push_back(best);
        }
        const size_t global_raw = global.size();

        deduplicateByTrain(global);
        if (config_.use_orientation_filter) {
            orientationFilter(ref.keypoints, curr.keypoints, config_, global);
        }
        if (config_.use_gms) {
            gmsFilter(ref, curr, config_, global);
        }
        spdlog::debug("ProjectionMatcher: fallback global raw={} -> filtered={}",
                      global_raw, global.size());

        // The better of the two result sets populates `out`.
        if (global.size() > windowed.size()) {
            chosen = &global;
        }
    }

    out.matches     = std::move(*chosen);
    out.num_matches = static_cast<int>(out.matches.size());

    // ---- final min_matches check --------------------------------------------
    if (out.num_matches < config_.min_matches) {
        spdlog::warn("ProjectionMatcher: not enough matches ({} < {}) for "
                     "frames {} -> {}",
                     out.num_matches, config_.min_matches,
                     out.ref_frame_id, out.curr_frame_id);
        return MatchStatus::NOT_ENOUGH_MATCHES;
    }

    spdlog::debug("ProjectionMatcher: {} matches for frames {} -> {}",
                  out.num_matches, out.ref_frame_id, out.curr_frame_id);
    return MatchStatus::OK;
}

}  // namespace uavloc::vo
