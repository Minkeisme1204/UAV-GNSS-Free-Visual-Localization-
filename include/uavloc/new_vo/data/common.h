#pragma once

#include "uavloc/common/type.h"
#include "uavloc/new_vo/camera/base.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <nlohmann/json_fwd.hpp>

namespace uavloc {
namespace vo {
namespace data {

struct FrameObservation;

// NOTE(port): the JSON (de)serialization helpers below are DECLARED here but
// their definitions are omitted in common.cpp (persistence cut to avoid the
// nlohmann/json + sqlite3 dependency). Only <nlohmann/json_fwd.hpp> is included
// because these signatures name nlohmann::json; wire the definitions back when
// persistence is re-enabled.
nlohmann::json convert_rotation_to_json(const Mat33_t& rot_cw);

Mat33_t convert_json_to_rotation(const nlohmann::json& json_rot_cw);

nlohmann::json convert_translation_to_json(const Vec3_t& trans_cw);

Vec3_t convert_json_to_translation(const nlohmann::json& json_trans_cw);

nlohmann::json convert_keypoints_to_json(const std::vector<cv::KeyPoint>& keypts);

std::vector<cv::KeyPoint> convert_json_to_keypoints(const nlohmann::json& json_keypts);

nlohmann::json convert_descriptors_to_json(const cv::Mat& descriptors);

cv::Mat convert_json_to_descriptors(const nlohmann::json& json_descriptors);

/**
 * Assign all keypoints to cells to accelerate projection matching
 * @param camera
 * @param undist_keypts
 * @param keypt_indices_in_cells
 */
void assign_keypoints_to_grid(const camera::Base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                              std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells,
                              unsigned int num_grid_cols, unsigned int num_grid_rows);

/**
 * Assign all keypoints to cells to accelerate projection matching
 * @param camera
 * @param undist_keypts
 * @return
 */
auto assign_keypoints_to_grid(const camera::Base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                              unsigned int num_grid_cols, unsigned int num_grid_rows)
    -> std::vector<std::vector<std::vector<unsigned int>>>;

/**
 * Get x-y index of the cell in which the specified keypoint is assigned
 * @param camera
 * @param keypt
 * @param cell_idx_x
 * @param cell_idx_y
 * @return
 */
inline bool get_cell_indices(const camera::Base* camera, const cv::KeyPoint& keypt,
                             const unsigned int num_grid_cols, const unsigned int num_grid_rows,
                             const double inv_cell_width, const double inv_cell_height,
                             int& cell_idx_x, int& cell_idx_y) {
    cell_idx_x = cvFloor((keypt.pt.x - camera->img_bounds_.min_x_) * inv_cell_width);
    cell_idx_y = cvFloor((keypt.pt.y - camera->img_bounds_.min_y_) * inv_cell_height);
    return (0 <= cell_idx_x && cell_idx_x < static_cast<int>(num_grid_cols)
            && 0 <= cell_idx_y && cell_idx_y < static_cast<int>(num_grid_rows));
}

/**
 * Get keypoint indices in cell(s) in which the specified point is located
 * @param camera
 * @param undist_keypts
 * @param keypt_indices_in_cells
 * @param ref_x
 * @param ref_y
 * @param margin
 * @param min_level
 * @param max_level
 * @return
 */
std::vector<unsigned int> get_keypoints_in_cell(const camera::Base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                                                const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells,
                                                const float ref_x, const float ref_y, const float margin,
                                                const unsigned int num_grid_cols, const unsigned int num_grid_rows,
                                                const int min_level = -1, const int max_level = -1);
std::vector<unsigned int> get_keypoints_in_cell(const camera::Base* camera, const FrameObservation& frm_obs,
                                                const float ref_x, const float ref_y, const float margin,
                                                const int min_level = -1, const int max_level = -1);

/**
 * Triangulate the keypoint using the disparity
 */
Vec3_t triangulate_stereo(const camera::Base* camera,
                          const Mat33_t& rot_wc,
                          const Vec3_t& trans_wc,
                          const FrameObservation& frm_obs,
                          const unsigned int idx);

} // namespace data
}} // namespace vo // namespace uavloc
