#include "uavloc/new_vo/camera/base.h"
#include "uavloc/new_vo/data/common.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/frame_observation.h"
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/match/robust.h"
#include "uavloc/new_vo/solve/essential_solver.h"
#include "uavloc/util/angle.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/SVD>
#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace match {

namespace {

//! Widening of the epipolar band, expressed in acceleration-grid cells and
//! applied on each side of the band.
//!
//! This is NOT a tunable parameter and is deliberately kept out of the YAML
//! config: the band only narrows the CANDIDATE SET, the exact epipolar test
//! still runs on every surviving candidate, so this margin can cost speed but
//! can never change the produced matches. It absorbs the floating-point slack
//! between the affine band model and the cell-boundary arithmetic.
constexpr int CELL_SAFETY_MARGIN = 1;

//! Self-check switch (UAVLOC_MATCH_VERIFY=1): run both the grid-narrowed and
//! the exhaustive search for every keyframe pair and compare the results.
//! Read once; costs nothing when unset.
bool match_verify_enabled() {
    static const bool enabled = []() {
        const char* env = std::getenv("UAVLOC_MATCH_VERIFY");
        return env != nullptr && std::string(env) == "1";
    }();
    return enabled;
}

//! Continuous cell coordinate -> cell index, saturated a couple of cells beyond
//! the grid so the subsequent margin/clamp arithmetic stays well defined
//! (cvFloor of an out-of-range double is undefined behaviour).
int floor_cell_index(const double cell_coord, const int num_cells) {
    const double clamped = std::max(-2.0, std::min(static_cast<double>(num_cells) + 1.0, cell_coord));
    return cvFloor(clamped);
}

//! Unnormalised bearing w(u, v) such that bearing == w / ||w||.
//! For a perspective camera w = ((u - cx) / fx, (v - cy) / fy, 1), i.e. AFFINE
//! in (u, v) — which is what makes the epipolar band a strip in image space.
//! Recovered through the public camera interface (no downcast) as b / b.z().
bool point_to_unnormalized_bearing(const camera::Base* camera, const double u, const double v, Vec3_t& w) {
    const Vec3_t bearing = camera->convert_point_to_bearing(
        cv::Point2f(static_cast<float>(u), static_cast<float>(v)));
    if (!std::isfinite(bearing(2)) || std::abs(bearing(2)) < 1e-12) {
        return false;
    }
    w = bearing / bearing(2);
    return std::isfinite(w(0)) && std::isfinite(w(1));
}

//! Everything that only depends on the keyframe pair (not on idx_1), used to
//! narrow the epipolar search to a strip of grid cells of Keyframe 2.
struct EpipolarBandGrid {
    bool usable = false;

    const std::vector<std::vector<std::vector<unsigned int>>>* cells = nullptr;
    int num_cols = 0;
    int num_rows = 0;
    double min_x = 0.0;
    double min_y = 0.0;
    double cell_width = 0.0;
    double cell_height = 0.0;
    double inv_cell_width = 0.0;
    double inv_cell_height = 0.0;

    //! Affine model of the unnormalised bearing: w(u, v) = w00 + u * dwx + v * dwy.
    Vec3_t w00 = Vec3_t::Zero();
    Vec3_t dwx = Vec3_t::Zero();
    Vec3_t dwy = Vec3_t::Zero();

    //! max ||w|| over the image bounds (attained at a corner: ||w||^2 is convex).
    double r_max = 0.0;
    //! largest singular value of E_12.
    double sigma_max = 0.0;

    //! Keypoints that assign_keypoints_to_grid() dropped because they fall
    //! outside the image bounds — they live in no cell, so they must be added
    //! to every candidate list or the grid path would silently lose them.
    std::vector<unsigned int> outside_grid_indices;
};

EpipolarBandGrid build_epipolar_band_grid(const std::shared_ptr<data::Keyframe>& keyfrm_2, const Mat33_t& E_12) {
    EpipolarBandGrid band;

    const camera::Base* camera = keyfrm_2->camera_;
    if (camera == nullptr) {
        return band;
    }
    // The band model is affine only for a perspective projection.
    if (camera->model_type_ != camera::ModelType::Perspective) {
        return band;
    }

    const auto& frm_obs = keyfrm_2->frm_obs_;
    band.num_cols = static_cast<int>(frm_obs.num_grid_cols_);
    band.num_rows = static_cast<int>(frm_obs.num_grid_rows_);
    if (band.num_cols <= 0 || band.num_rows <= 0) {
        return band;
    }
    if (static_cast<int>(frm_obs.keypt_indices_in_cells_.size()) != band.num_cols) {
        return band;
    }
    for (const auto& col : frm_obs.keypt_indices_in_cells_) {
        if (static_cast<int>(col.size()) != band.num_rows) {
            return band;
        }
    }

    const double span_x = static_cast<double>(camera->img_bounds_.max_x_) - static_cast<double>(camera->img_bounds_.min_x_);
    const double span_y = static_cast<double>(camera->img_bounds_.max_y_) - static_cast<double>(camera->img_bounds_.min_y_);
    if (!(span_x > 0.0) || !(span_y > 0.0)) {
        return band;
    }

    band.cells = &frm_obs.keypt_indices_in_cells_;
    band.min_x = camera->img_bounds_.min_x_;
    band.min_y = camera->img_bounds_.min_y_;
    // Same expressions as data::assign_keypoints_to_grid().
    band.inv_cell_width = static_cast<double>(band.num_cols) / span_x;
    band.inv_cell_height = static_cast<double>(band.num_rows) / span_y;
    band.cell_width = span_x / static_cast<double>(band.num_cols);
    band.cell_height = span_y / static_cast<double>(band.num_rows);

    // Affine coefficients of w(u, v), sampled through the public camera API.
    Vec3_t w_10, w_01;
    if (!point_to_unnormalized_bearing(camera, 0.0, 0.0, band.w00)
        || !point_to_unnormalized_bearing(camera, 1.0, 0.0, w_10)
        || !point_to_unnormalized_bearing(camera, 0.0, 1.0, w_01)) {
        return band;
    }
    band.dwx = w_10 - band.w00;
    band.dwy = w_01 - band.w00;

    // max ||w|| over the image rectangle (||w||^2 is convex -> attained at a corner).
    const double corners_x[2] = {static_cast<double>(camera->img_bounds_.min_x_), static_cast<double>(camera->img_bounds_.max_x_)};
    const double corners_y[2] = {static_cast<double>(camera->img_bounds_.min_y_), static_cast<double>(camera->img_bounds_.max_y_)};
    for (const double u : corners_x) {
        for (const double v : corners_y) {
            Vec3_t w_corner;
            if (!point_to_unnormalized_bearing(camera, u, v, w_corner)) {
                return band;
            }
            band.r_max = std::max(band.r_max, w_corner.norm());
        }
    }
    if (!(band.r_max > 0.0) || !std::isfinite(band.r_max)) {
        return band;
    }

    Eigen::JacobiSVD<Mat33_t> svd(E_12);
    band.sigma_max = svd.singularValues()(0);
    if (!(band.sigma_max > 0.0) || !std::isfinite(band.sigma_max)) {
        return band;
    }

    // Collect the keypoints that were never assigned to a cell.
    const unsigned int num_keypts = static_cast<unsigned int>(frm_obs.undist_keypts_.size());
    for (unsigned int idx = 0; idx < num_keypts; ++idx) {
        int cell_idx_x = 0, cell_idx_y = 0;
        if (!data::get_cell_indices(camera, frm_obs.undist_keypts_.at(idx),
                                    frm_obs.num_grid_cols_, frm_obs.num_grid_rows_,
                                    band.inv_cell_width, band.inv_cell_height,
                                    cell_idx_x, cell_idx_y)) {
            band.outside_grid_indices.push_back(idx);
        }
    }

    band.usable = true;
    return band;
}

//! Gather every keypoint index of Keyframe 2 that can satisfy
//! |coef_a * u + coef_b * v + coef_c| <= band_half_width, by walking only the
//! grid cells the strip crosses (widened by CELL_SAFETY_MARGIN). The result is
//! sorted ascending, which is what keeps the caller's tie-breaking identical to
//! the exhaustive scan. Never under-selects: cells are only ever added.
void collect_band_candidates(const EpipolarBandGrid& band,
                             const double coef_a, const double coef_b, const double coef_c,
                             const double band_half_width,
                             std::vector<unsigned int>& candidates) {
    candidates.clear();
    // Keypoints outside the image bounds live in no cell — always candidates.
    candidates.insert(candidates.end(), band.outside_grid_indices.begin(), band.outside_grid_indices.end());

    const auto& cells = *band.cells;

    if (std::abs(coef_b) >= std::abs(coef_a)) {
        // Strip is more horizontal than vertical: sweep columns, solve for rows.
        const double inv_b = 1.0 / coef_b;
        for (int cell_idx_x = 0; cell_idx_x < band.num_cols; ++cell_idx_x) {
            const double u_lo = band.min_x + cell_idx_x * band.cell_width;
            const double u_hi = u_lo + band.cell_width;
            const double v_1 = (-band_half_width - coef_a * u_lo - coef_c) * inv_b;
            const double v_2 = (band_half_width - coef_a * u_lo - coef_c) * inv_b;
            const double v_3 = (-band_half_width - coef_a * u_hi - coef_c) * inv_b;
            const double v_4 = (band_half_width - coef_a * u_hi - coef_c) * inv_b;
            const double v_min = std::min(std::min(v_1, v_2), std::min(v_3, v_4));
            const double v_max = std::max(std::max(v_1, v_2), std::max(v_3, v_4));

            int row_lo = 0;
            int row_hi = band.num_rows - 1;
            if (std::isfinite(v_min) && std::isfinite(v_max)) {
                row_lo = std::max(0, floor_cell_index((v_min - band.min_y) * band.inv_cell_height, band.num_rows) - CELL_SAFETY_MARGIN);
                row_hi = std::min(band.num_rows - 1, floor_cell_index((v_max - band.min_y) * band.inv_cell_height, band.num_rows) + CELL_SAFETY_MARGIN);
            }
            for (int cell_idx_y = row_lo; cell_idx_y <= row_hi; ++cell_idx_y) {
                const auto& cell = cells[cell_idx_x][cell_idx_y];
                candidates.insert(candidates.end(), cell.begin(), cell.end());
            }
        }
    }
    else {
        // Strip is more vertical than horizontal: sweep rows, solve for columns.
        const double inv_a = 1.0 / coef_a;
        for (int cell_idx_y = 0; cell_idx_y < band.num_rows; ++cell_idx_y) {
            const double v_lo = band.min_y + cell_idx_y * band.cell_height;
            const double v_hi = v_lo + band.cell_height;
            const double u_1 = (-band_half_width - coef_b * v_lo - coef_c) * inv_a;
            const double u_2 = (band_half_width - coef_b * v_lo - coef_c) * inv_a;
            const double u_3 = (-band_half_width - coef_b * v_hi - coef_c) * inv_a;
            const double u_4 = (band_half_width - coef_b * v_hi - coef_c) * inv_a;
            const double u_min = std::min(std::min(u_1, u_2), std::min(u_3, u_4));
            const double u_max = std::max(std::max(u_1, u_2), std::max(u_3, u_4));

            int col_lo = 0;
            int col_hi = band.num_cols - 1;
            if (std::isfinite(u_min) && std::isfinite(u_max)) {
                col_lo = std::max(0, floor_cell_index((u_min - band.min_x) * band.inv_cell_width, band.num_cols) - CELL_SAFETY_MARGIN);
                col_hi = std::min(band.num_cols - 1, floor_cell_index((u_max - band.min_x) * band.inv_cell_width, band.num_cols) + CELL_SAFETY_MARGIN);
            }
            for (int cell_idx_x = col_lo; cell_idx_x <= col_hi; ++cell_idx_x) {
                const auto& cell = cells[cell_idx_x][cell_idx_y];
                candidates.insert(candidates.end(), cell.begin(), cell.end());
            }
        }
    }

    // Each keypoint belongs to at most one cell, so there are no duplicates;
    // sorting restores the ascending idx_2 order the inner loop relies on.
    std::sort(candidates.begin(), candidates.end());
}

} // unnamed namespace

unsigned int Robust::match_for_triangulation(const std::shared_ptr<data::Keyframe>& keyfrm_1,
                                             const std::shared_ptr<data::Keyframe>& keyfrm_2,
                                             const Mat33_t& E_12,
                                             std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs,
                                             const float residual_rad_thr) const {
    if (!match_verify_enabled()) {
        return match_for_triangulation_impl(keyfrm_1, keyfrm_2, E_12, matched_idx_pairs, residual_rad_thr, true);
    }

    // Self-check: the grid-narrowed search must reproduce the exhaustive one
    // bit-for-bit.
    std::vector<std::pair<unsigned int, unsigned int>> ref_pairs;
    const unsigned int ref_num = match_for_triangulation_impl(keyfrm_1, keyfrm_2, E_12, ref_pairs, residual_rad_thr, false);
    const unsigned int num_matches = match_for_triangulation_impl(keyfrm_1, keyfrm_2, E_12, matched_idx_pairs, residual_rad_thr, true);

    if (num_matches != ref_num || matched_idx_pairs != ref_pairs) {
        spdlog::error("match[VERIFY] MISMATCH kf1={} kf2={} grid_num={} exhaustive_num={} grid_pairs={} exhaustive_pairs={}",
                      keyfrm_1->id_, keyfrm_2->id_, num_matches, ref_num,
                      matched_idx_pairs.size(), ref_pairs.size());
        const std::size_t common = std::min(matched_idx_pairs.size(), ref_pairs.size());
        unsigned int printed = 0;
        for (std::size_t i = 0; i < common && printed < 5; ++i) {
            if (matched_idx_pairs.at(i) != ref_pairs.at(i)) {
                spdlog::error("match[VERIFY]   at {}: grid=({},{}) exhaustive=({},{})",
                              i, matched_idx_pairs.at(i).first, matched_idx_pairs.at(i).second,
                              ref_pairs.at(i).first, ref_pairs.at(i).second);
                ++printed;
            }
        }
    }
    else {
        spdlog::info("match[VERIFY] ok kf1={} kf2={} pairs={}", keyfrm_1->id_, keyfrm_2->id_, num_matches);
    }

    return num_matches;
}

unsigned int Robust::match_for_triangulation_impl(const std::shared_ptr<data::Keyframe>& keyfrm_1,
                                                  const std::shared_ptr<data::Keyframe>& keyfrm_2,
                                                  const Mat33_t& E_12,
                                                  std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs,
                                                  const float residual_rad_thr,
                                                  const bool use_grid) const {
    unsigned int num_matches = 0;

    // Project the center of Keyframe 1 to Keyframe 2
    // to acquire the epipole coordinates of the candidate Keyframe
    const Vec3_t cam_center_1 = keyfrm_1->get_trans_wc();
    const Mat33_t rot_2w = keyfrm_2->get_rot_cw();
    const Vec3_t trans_2w = keyfrm_2->get_trans_cw();
    Vec3_t epiplane_in_keyfrm_2;
    const bool valid_epiplane = keyfrm_2->camera_->reproject_to_bearing(rot_2w, trans_2w, cam_center_1, epiplane_in_keyfrm_2);

    // Acquire the 3D point information of the keframes
    const auto assoc_lms_in_keyfrm_1 = keyfrm_1->get_landmarks();
    const auto assoc_lms_in_keyfrm_2 = keyfrm_2->get_landmarks();
    const auto num_keypts_1 = keyfrm_1->frm_obs_.undist_keypts_.size();
    const auto num_keypts_2 = keyfrm_2->frm_obs_.undist_keypts_.size();

    // Save the matching information
    // Discard the already matched keypoints in Keyframe 2
    // to acquire a unique association to each keypoint in Keyframe 1
    std::vector<bool> is_already_matched_in_keyfrm_2(num_keypts_2, false);
    // Save the keypoint idx in Keyframe 2 which is already associated to the keypoint idx in Keyframe 1
    std::vector<int> matched_indices_2_in_keyfrm_1(num_keypts_1, -1);

    // Candidate keypoints of Keyframe 2, always visited in ASCENDING idx_2
    // order: the inner loop only ever updates its state inside the epipolar
    // inlier branch, so narrowing the scan to a superset of the epipolar
    // inliers — in the original order — leaves the result unchanged.
    const EpipolarBandGrid band = use_grid ? build_epipolar_band_grid(keyfrm_2, E_12) : EpipolarBandGrid{};

    // Exhaustive index list, used when the grid cannot be used (degenerate
    // geometry, missing grid) and by the reference path.
    std::vector<unsigned int> all_indices(num_keypts_2);
    for (unsigned int idx_2 = 0; idx_2 < num_keypts_2; ++idx_2) {
        all_indices.at(idx_2) = idx_2;
    }
    // Scratch buffer reused across idx_1 — never allocated inside the hot loop.
    std::vector<unsigned int> candidates;
    candidates.reserve(num_keypts_2);

    for (unsigned int idx_1 = 0; idx_1 < num_keypts_1; ++idx_1) {
        const auto& lm_1 = assoc_lms_in_keyfrm_1.at(idx_1);
        // Ignore if the keypoint of Keyframe is associated any 3D points
        if (lm_1) {
            continue;
        }

        // Check if it's a stereo keypoint or not
        const bool is_stereo_keypt_1 = !keyfrm_1->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm_1->frm_obs_.stereo_x_right_.at(idx_1);

        // Acquire the keypoints and ORB feature vectors
        const auto& keypt_1 = keyfrm_1->frm_obs_.undist_keypts_.at(idx_1);
        const Vec3_t& bearing_1 = keyfrm_1->frm_obs_.bearings_.at(idx_1);
        // Raw descriptor row pointer: Mat::ptr<T>(row) honours the matrix step,
        // so no continuity assumption is made, and no cv::Mat header is built
        // inside the inner loop.
        const uint32_t* const desc_1_ptr = keyfrm_1->frm_obs_.descriptors_.ptr<uint32_t>(idx_1);

        // Narrow the candidates of Keyframe 2 to the epipolar band.
        //
        //   check_epipolar_constraint() accepts iff |asin(c)| < thr, with
        //   c = (m . b2) / |E12 b2|, m = E12^T b1, thr = residual_rad_thr * scale_factor.
        //   With b2 = w / ||w|| the norm cancels: |m . w| < sin(thr) * |E12 w|,
        //   and |E12 w| <= sigma_max(E12) * ||w|| <= sigma_max * r_max.
        //   Hence every inlier satisfies |m . w(u, v)| <= T, T = sin(thr) * sigma_max * r_max,
        //   which is an AFFINE strip in (u, v) — a conservative superset.
        const std::vector<unsigned int>* candidate_indices = &all_indices;
        if (band.usable) {
            const Vec3_t m = E_12.transpose() * bearing_1;
            const double coef_a = m.dot(band.dwx);
            const double coef_b = m.dot(band.dwy);
            const double coef_c = m.dot(band.w00);
            const double thr = static_cast<double>(residual_rad_thr)
                               * static_cast<double>(keyfrm_1->orb_params_->scale_factors_.at(keypt_1.octave));
            // |asin(c)| < thr is vacuously true once thr reaches pi/2, and sin()
            // is no longer monotone there — saturate instead of shrinking the band.
            const double sin_thr = (thr >= M_PI / 2.0) ? 1.0 : std::sin(thr);
            const double band_half_width = sin_thr * band.sigma_max * band.r_max;

            const bool degenerate = (coef_a == 0.0 && coef_b == 0.0)
                                    || !std::isfinite(coef_a) || !std::isfinite(coef_b)
                                    || !std::isfinite(coef_c) || !std::isfinite(band_half_width);
            if (!degenerate) {
                collect_band_candidates(band, coef_a, coef_b, coef_c, band_half_width, candidates);
                candidate_indices = &candidates;
            }
        }

        // Find a keypoint in Keyframe 2 that has the minimum hamming distance
        unsigned int best_hamm_dist = HAMMING_DIST_THR_LOW;
        int best_idx_2 = -1;
        unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;

        for (const unsigned int idx_2 : *candidate_indices) {
            // Ignore if the keypoint is associated any 3D points
            // (because this function is used for triangulation)
            const auto& lm_2 = assoc_lms_in_keyfrm_2.at(idx_2);
            if (lm_2) {
                continue;
            }

            // Ignore if matches are already aquired
            if (is_already_matched_in_keyfrm_2.at(idx_2)) {
                continue;
            }

            if (check_orientation_ && std::abs(util::angle::diff(keypt_1.angle, keyfrm_2->frm_obs_.undist_keypts_.at(idx_2).angle)) > 30.0) {
                continue;
            }

            // Check if it's a stereo keypoint or not
            const bool is_stereo_keypt_2 = !keyfrm_2->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm_2->frm_obs_.stereo_x_right_.at(idx_2);

            // Acquire the keypoints and ORB feature vectors
            const Vec3_t& bearing_2 = keyfrm_2->frm_obs_.bearings_.at(idx_2);
            const uint32_t* const desc_2_ptr = keyfrm_2->frm_obs_.descriptors_.ptr<uint32_t>(idx_2);

            // Compute the distance
            const auto hamm_dist = compute_descriptor_distance_32(desc_1_ptr, desc_2_ptr);

            if (HAMMING_DIST_THR_LOW < hamm_dist || best_hamm_dist < hamm_dist) {
                continue;
            }

            if (valid_epiplane && !is_stereo_keypt_1 && !is_stereo_keypt_2) {
                // Do not use any keypoints near the epipole if both are not stereo keypoints
                const auto cos_dist = epiplane_in_keyfrm_2.dot(bearing_2);
                // The threshold of the minimum angle formed by the epipole and the bearing vector is 3.0 degree
                constexpr double cos_dist_thr = 0.99862953475;

                // Do not allow to match if the formed angle is narrower that the threshold value
                if (cos_dist_thr < cos_dist) {
                    continue;
                }
            }

            // Check consistency in Matrix E
            const bool is_inlier = check_epipolar_constraint(bearing_1, bearing_2, E_12,
                                                             keyfrm_1->orb_params_->scale_factors_.at(keypt_1.octave),
                                                             residual_rad_thr);
            if (is_inlier) {
                if (hamm_dist < best_hamm_dist) {
                    second_best_hamm_dist = best_hamm_dist;
                    best_hamm_dist = hamm_dist;
                    best_idx_2 = idx_2;
                }
                else if (hamm_dist < second_best_hamm_dist) {
                    second_best_hamm_dist = hamm_dist;
                }
            }
        }

        if (best_idx_2 < 0) {
            continue;
        }

        // Ratio test
        if (lowe_ratio_ * second_best_hamm_dist < static_cast<float>(best_hamm_dist)) {
            continue;
        }

        is_already_matched_in_keyfrm_2.at(best_idx_2) = true;
        matched_indices_2_in_keyfrm_1.at(idx_1) = best_idx_2;
        ++num_matches;
    }

    matched_idx_pairs.clear();
    matched_idx_pairs.reserve(num_matches);

    for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_keyfrm_1.size(); ++idx_1) {
        if (matched_indices_2_in_keyfrm_1.at(idx_1) < 0) {
            continue;
        }
        matched_idx_pairs.emplace_back(std::make_pair(idx_1, matched_indices_2_in_keyfrm_1.at(idx_1)));
    }

    return num_matches;
}

unsigned int Robust::match_keyframes(const std::shared_ptr<data::Keyframe>& keyfrm1, const std::shared_ptr<data::Keyframe>& keyfrm2,
                                     std::vector<std::shared_ptr<data::Landmark>>& matched_lms_in_frm,
                                     bool validate_with_essential_solver, bool use_fixed_seed) const {
    // Initialization
    const auto num_frm_keypts = keyfrm1->frm_obs_.undist_keypts_.size();
    const auto keyfrm_lms = keyfrm2->get_landmarks();
    unsigned int num_inlier_matches = 0;
    matched_lms_in_frm = std::vector<std::shared_ptr<data::Landmark>>(num_frm_keypts, nullptr);

    // Compute brute-force match
    std::vector<std::pair<int, int>> matches;
    brute_force_match(keyfrm1->frm_obs_, keyfrm2, matches);

    // Extract only inliers with eight-point RANSAC
    if (validate_with_essential_solver) {
        solve::EssentialSolver solver(keyfrm1->frm_obs_.bearings_, keyfrm2->frm_obs_.bearings_, matches, use_fixed_seed);
        solver.find_via_ransac(50, false);
        if (!solver.solution_is_valid()) {
            return 0;
        }
        const auto is_inlier_matches = solver.get_inlier_matches();

        // Save the information
        for (unsigned int i = 0; i < matches.size(); ++i) {
            if (!is_inlier_matches.at(i)) {
                continue;
            }
            const auto frm_idx = matches.at(i).first;
            const auto keyfrm_idx = matches.at(i).second;

            matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
            ++num_inlier_matches;
        }
    }
    else {
        // Save the information
        for (unsigned int i = 0; i < matches.size(); ++i) {
            const auto frm_idx = matches.at(i).first;
            const auto keyfrm_idx = matches.at(i).second;

            matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
            ++num_inlier_matches;
        }
    }

    return num_inlier_matches;
}

unsigned int Robust::match_frame_and_keyframe(data::Frame& frm, const std::shared_ptr<data::Keyframe>& keyfrm,
                                              std::vector<std::shared_ptr<data::Landmark>>& matched_lms_in_frm,
                                              bool use_fixed_seed) const {
    // Initialization
    const auto num_frm_keypts = frm.frm_obs_.undist_keypts_.size();
    const auto keyfrm_lms = keyfrm->get_landmarks();
    unsigned int num_inlier_matches = 0;
    matched_lms_in_frm = std::vector<std::shared_ptr<data::Landmark>>(num_frm_keypts, nullptr);

    // Compute brute-force match
    std::vector<std::pair<int, int>> matches;
    brute_force_match(frm.frm_obs_, keyfrm, matches);

    // Extract only inliers with RANSAC
    solve::EssentialSolver solver(frm.frm_obs_.bearings_, keyfrm->frm_obs_.bearings_, matches, use_fixed_seed);
    solver.find_via_ransac(1000, true);
    if (!solver.solution_is_valid()) {
        return 0;
    }
    const auto is_inlier_matches = solver.get_inlier_matches();

    // Save the information
    for (unsigned int i = 0; i < matches.size(); ++i) {
        if (!is_inlier_matches.at(i)) {
            continue;
        }
        const auto frm_idx = matches.at(i).first;
        const auto keyfrm_idx = matches.at(i).second;

        matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
        ++num_inlier_matches;
    }

    return num_inlier_matches;
}

unsigned int Robust::brute_force_match(const data::FrameObservation& frm_obs,
                                       const std::shared_ptr<data::Keyframe>& keyfrm,
                                       std::vector<std::pair<int, int>>& matches) const {
    unsigned int num_matches = 0;

    // 1. Acquire the Frame and Keyframe information

    const auto num_keypts_1 = frm_obs.undist_keypts_.size();
    const auto num_keypts_2 = keyfrm->frm_obs_.undist_keypts_.size();
    const auto keypts_1 = frm_obs.undist_keypts_;
    const auto keypts_2 = keyfrm->frm_obs_.undist_keypts_;
    const auto lms_2 = keyfrm->get_landmarks();
    const auto& descs_1 = frm_obs.descriptors_;
    const auto& descs_2 = keyfrm->frm_obs_.descriptors_;

    // 2. Acquire ORB descriptors in the Keyframe which are the first and second closest to the descriptors in the Frame
    //    it is assumed that keypoint in the Keyframe are associated to 3D points

    // Index 2 associated to each index 1
    auto matched_indices_2_in_1 = std::vector<int>(num_keypts_1, -1);
    // Avoid duplication
    std::unordered_set<int> already_matched_indices_1;

    for (unsigned int idx_2 = 0; idx_2 < num_keypts_2; ++idx_2) {
        // 3次元点が有効なもののみ対象にする
        const auto& lm_2 = lms_2.at(idx_2);
        if (!lm_2) {
            continue;
        }
        if (lm_2->will_be_erased()) {
            continue;
        }

        // Acquire the descriptor for index 2
        const auto& desc_2 = descs_2.row(idx_2);

        // Acquire the descriptors in the Frame which are the first and second closest to the descriptor in the Keyframe
        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx_1 = -1;
        unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;

        for (unsigned int idx_1 = 0; idx_1 < num_keypts_1; ++idx_1) {
            // Avoid duplication
            if (static_cast<bool>(already_matched_indices_1.count(idx_1))) {
                continue;
            }

            if (check_orientation_ && std::abs(util::angle::diff(keypts_1.at(idx_1).angle, keypts_2.at(idx_2).angle)) > 30.0) {
                continue;
            }

            const auto& desc_1 = descs_1.row(idx_1);

            const auto hamm_dist = compute_descriptor_distance_32(desc_2, desc_1);

            if (hamm_dist < best_hamm_dist) {
                second_best_hamm_dist = best_hamm_dist;
                best_hamm_dist = hamm_dist;
                best_idx_1 = idx_1;
            }
            else if (hamm_dist < second_best_hamm_dist) {
                second_best_hamm_dist = hamm_dist;
            }
        }

        if (HAMMING_DIST_THR_LOW < best_hamm_dist) {
            continue;
        }

        if (best_idx_1 < 0) {
            continue;
        }

        // Ratio test
        if (lowe_ratio_ * second_best_hamm_dist < static_cast<float>(best_hamm_dist)) {
            continue;
        }

        matched_indices_2_in_1.at(best_idx_1) = idx_2;
        // Avoid duplication
        already_matched_indices_1.insert(best_idx_1);

        ++num_matches;
    }

    matches.clear();
    matches.reserve(num_matches);
    for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_1.size(); ++idx_1) {
        const auto idx_2 = matched_indices_2_in_1.at(idx_1);
        if (idx_2 < 0) {
            continue;
        }
        matches.emplace_back(std::make_pair(idx_1, idx_2));
    }

    return num_matches;
}

} // namespace match
}} // namespace vo // namespace uavloc
