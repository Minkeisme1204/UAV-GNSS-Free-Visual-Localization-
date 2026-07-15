#include "uavloc/new_vo/module/vo_frame_loader.h"

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {

VoFrameLoader::VoFrameLoader(camera::Base* camera,
                                 feature::OrbParams* orb_params,
                                 unsigned int min_area,
                                 feature::DescriptorType desc_type,
                                 const std::vector<std::vector<float>>& mask_rects,
                                 unsigned int num_grid_cols,
                                 unsigned int num_grid_rows)
    : orb_extractor_(orb_params, min_area, desc_type, mask_rects),
      camera_(camera),
      orb_params_(orb_params),
      num_grid_cols_(num_grid_cols),
      num_grid_rows_(num_grid_rows) {}

data::Frame VoFrameLoader::load(const sensor::FrameData& fd) {
    // 1. Grayscale conversion (ORB operates on single-channel imagery).
    cv::Mat gray;
    if (fd.image.channels() != 1) {
        cv::cvtColor(fd.image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = fd.image;
    }

    data::FrameObservation frm_obs;

    // 2. Extract ORB keypoints + descriptors (no feature mask at this seam).
    std::vector<cv::KeyPoint> keypts;
    orb_extractor_.extract(gray, cv::Mat(), keypts, frm_obs.descriptors_);
    if (keypts.empty()) {
        spdlog::warn("VoFrameLoader: no keypoints extracted from Frame {}", fd.frame_id);
    }

    // 3. Undistort keypoints.
    camera_->undistort_keypoints(keypts, frm_obs.undist_keypts_);

    // 4. Convert undistorted keypoints to bearing vectors.
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // 5. Assign keypoints to the acceleration grid (void overload, as stella's
    //    create_monocular_frame does — populates frm_obs_.keypt_indices_in_cells_).
    frm_obs.num_grid_cols_ = num_grid_cols_;
    frm_obs.num_grid_rows_ = num_grid_rows_;
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_,
                                   frm_obs.num_grid_cols_, frm_obs.num_grid_rows_);

    // 6. Build the pipeline Frame. Telemetry is intentionally NOT attached here
    //    (data::Frame has no telemetry field — held by a higher layer, keyed by
    //    frame_id). See header note.
    return data::Frame(static_cast<unsigned int>(fd.frame_id), fd.timestamp_msec,
                       camera_, orb_params_, frm_obs);
}

} // namespace vo
} // namespace uavloc
