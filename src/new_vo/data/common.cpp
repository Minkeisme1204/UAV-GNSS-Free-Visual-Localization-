#include "uavloc/new_vo/data/common.h"
#include "uavloc/new_vo/data/frame_observation.h"
#include "uavloc/new_vo/camera/perspective_camera.h"

namespace uavloc {
namespace vo {
namespace data {

// TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency.
// The JSON (de)serialization helpers declared in common.h —
//   convert_rotation_to_json / convert_json_to_rotation
//   convert_translation_to_json / convert_json_to_translation
//   convert_keypoints_to_json / convert_json_to_keypoints
//   convert_descriptors_to_json / convert_json_to_descriptors
// have their DEFINITIONS omitted here (defining a function with an incomplete
// nlohmann::json return/parameter type is ill-formed, and <nlohmann/json.hpp>
// was dropped). Declarations are retained in common.h; wire the definitions to
// nlohmann/json when persistence is re-enabled.

void assign_keypoints_to_grid(const camera::Base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                              std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells,
                              unsigned int num_grid_cols, unsigned int num_grid_rows) {
    double inv_cell_width = static_cast<double>(num_grid_cols) / (camera->img_bounds_.max_x_ - camera->img_bounds_.min_x_);
    double inv_cell_height = static_cast<double>(num_grid_rows) / (camera->img_bounds_.max_y_ - camera->img_bounds_.min_y_);

    // Pre-allocate memory
    const unsigned int num_keypts = undist_keypts.size();
    const unsigned int num_to_reserve = 0.5 * num_keypts / (num_grid_cols * num_grid_rows);
    keypt_indices_in_cells.resize(num_grid_cols);
    for (auto& keypt_indices_in_row : keypt_indices_in_cells) {
        keypt_indices_in_row.resize(num_grid_rows);
        for (auto& keypt_indices_in_cell : keypt_indices_in_row) {
            keypt_indices_in_cell.reserve(num_to_reserve);
        }
    }

    // Calculate cell position and store
    for (unsigned int idx = 0; idx < num_keypts; ++idx) {
        const auto& keypt = undist_keypts.at(idx);
        int cell_idx_x, cell_idx_y;
        if (get_cell_indices(camera, keypt, num_grid_cols, num_grid_rows, inv_cell_width, inv_cell_height, cell_idx_x, cell_idx_y)) {
            keypt_indices_in_cells.at(cell_idx_x).at(cell_idx_y).push_back(idx);
        }
    }
}

auto assign_keypoints_to_grid(const camera::Base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                              unsigned int num_grid_cols, unsigned int num_grid_rows)
    -> std::vector<std::vector<std::vector<unsigned int>>> {
    std::vector<std::vector<std::vector<unsigned int>>> keypt_indices_in_cells;
    assign_keypoints_to_grid(camera, undist_keypts, keypt_indices_in_cells, num_grid_cols, num_grid_rows);
    return keypt_indices_in_cells;
}

std::vector<unsigned int> get_keypoints_in_cell(const camera::Base* camera, const data::FrameObservation& frm_obs,
                                                const float ref_x, const float ref_y, const float margin,
                                                const int min_level, const int max_level) {
    return get_keypoints_in_cell(camera, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_,
                                 ref_x, ref_y, margin,
                                 frm_obs.num_grid_cols_, frm_obs.num_grid_rows_,
                                 min_level, max_level);
}

std::vector<unsigned int> get_keypoints_in_cell(const camera::Base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                                                const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells,
                                                const float ref_x, const float ref_y, const float margin,
                                                const unsigned int num_grid_cols, const unsigned int num_grid_rows,
                                                const int min_level, const int max_level) {
    double inv_cell_width = static_cast<double>(num_grid_cols) / (camera->img_bounds_.max_x_ - camera->img_bounds_.min_x_);
    double inv_cell_height = static_cast<double>(num_grid_rows) / (camera->img_bounds_.max_y_ - camera->img_bounds_.min_y_);

    std::vector<unsigned int> indices;

    const int min_cell_idx_x = std::max(0, cvFloor((ref_x - camera->img_bounds_.min_x_ - margin) * inv_cell_width));
    if (static_cast<int>(num_grid_cols) <= min_cell_idx_x) {
        return indices;
    }

    const int max_cell_idx_x = std::min(static_cast<int>(num_grid_cols - 1), cvCeil((ref_x - camera->img_bounds_.min_x_ + margin) * inv_cell_width));
    if (max_cell_idx_x < 0) {
        return indices;
    }

    const int min_cell_idx_y = std::max(0, cvFloor((ref_y - camera->img_bounds_.min_y_ - margin) * inv_cell_height));
    if (static_cast<int>(num_grid_rows) <= min_cell_idx_y) {
        return indices;
    }

    const int max_cell_idx_y = std::min(static_cast<int>(num_grid_rows - 1), cvCeil((ref_y - camera->img_bounds_.min_y_ + margin) * inv_cell_height));
    if (max_cell_idx_y < 0) {
        return indices;
    }

    indices.reserve(undist_keypts.size());

    const bool check_min_level = 0 <= min_level;
    const bool check_max_level = 0 <= max_level;

    for (int cell_idx_x = min_cell_idx_x; cell_idx_x <= max_cell_idx_x; ++cell_idx_x) {
        for (int cell_idx_y = min_cell_idx_y; cell_idx_y <= max_cell_idx_y; ++cell_idx_y) {
            const auto& keypt_indices_in_cell = keypt_indices_in_cells.at(cell_idx_x).at(cell_idx_y);
            if (keypt_indices_in_cell.empty()) {
                continue;
            }

            for (unsigned int idx : keypt_indices_in_cell) {
                const auto& undist_keypt = undist_keypts.at(idx);

                if (check_min_level && undist_keypt.octave < min_level) {
                    continue;
                }
                if (check_max_level && max_level < undist_keypt.octave) {
                    continue;
                }

                const float dist_x = undist_keypt.pt.x - ref_x;
                const float dist_y = undist_keypt.pt.y - ref_y;

                if (std::abs(dist_x) < margin && std::abs(dist_y) < margin) {
                    indices.push_back(idx);
                }
            }
        }
    }

    return indices;
}

Vec3_t triangulate_stereo(const camera::Base* camera,
                          const Mat33_t& rot_wc,
                          const Vec3_t& trans_wc,
                          const FrameObservation& frm_obs,
                          const unsigned int idx) {
    assert(camera->setup_type_ != camera::SetupType::Monocular);

    // Perspective is the only camera model (see camera::PerspectiveCamera).
    auto perspective_camera = static_cast<const camera::PerspectiveCamera*>(camera);

    const float depth = frm_obs.depths_.empty() ? -1.0f : frm_obs.depths_.at(idx);
    if (0.0 < depth) {
        const float x = frm_obs.undist_keypts_.at(idx).pt.x;
        const float y = frm_obs.undist_keypts_.at(idx).pt.y;
        const float unproj_x = (x - perspective_camera->cx_) * depth * perspective_camera->fx_inv_;
        const float unproj_y = (y - perspective_camera->cy_) * depth * perspective_camera->fy_inv_;
        const Vec3_t pos_c{unproj_x, unproj_y, depth};

        // Convert from camera coordinates to world coordinates
        return rot_wc * pos_c + trans_wc;
    }

    return Vec3_t::Zero();
}

} // namespace data
}} // namespace vo // namespace uavloc
