#include "uavloc/new_vo/data/common.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
// NOTE(port): reuses the already-ported ORB extractor header (feature::OrbExtractor).
#include "uavloc/new_vo/feature/orb/orb_extractor.h"
// TODO(port): match/stereo.h (stereo setup) not cloned — nadir is monocular.
// The include is cut; Frame.cc does not reference any match::stereo symbol.
// See .docs/designs/new_vo_port_notes.md

#include <thread>

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace data {

Frame::Frame(unsigned int frame_id, const double timestamp, camera::Base* camera, feature::OrbParams* orb_params,
             const FrameObservation frm_obs)
    : id_(frame_id), timestamp_(timestamp), camera_(camera), orb_params_(orb_params), frm_obs_(frm_obs),
      // Initialize association with 3D points
      landmarks_(std::vector<std::shared_ptr<Landmark>>(frm_obs_.undist_keypts_.size(), nullptr)) {}

void Frame::set_pose_cw(const Mat44_t& pose_cw) {
    pose_is_valid_ = true;
    pose_cw_ = pose_cw;

    rot_cw_ = pose_cw_.block<3, 3>(0, 0);
    rot_wc_ = rot_cw_.transpose();
    trans_cw_ = pose_cw_.block<3, 1>(0, 3);
    trans_wc_ = -rot_cw_.transpose() * trans_cw_;
}

Mat44_t Frame::get_pose_cw() const {
    return pose_cw_;
}

Mat44_t Frame::get_pose_wc() const {
    Mat44_t pose_wc = Mat44_t::Identity();
    pose_wc.block<3, 3>(0, 0) = rot_wc_;
    pose_wc.block<3, 1>(0, 3) = trans_wc_;
    return pose_wc;
}

Vec3_t Frame::get_trans_wc() const {
    return trans_wc_;
}

Mat33_t Frame::get_rot_wc() const {
    return rot_wc_;
}

bool Frame::can_observe(const std::shared_ptr<Landmark>& lm, const float ray_cos_thr,
                        Vec2_t& reproj, float& x_right, unsigned int& pred_scale_level) const {
    const Vec3_t pos_w = lm->get_pos_in_world();

    const bool in_image = camera_->reproject_to_image(rot_cw_, trans_cw_, pos_w, reproj, x_right);
    if (!in_image) {
        return false;
    }

    const Vec3_t cam_to_lm_vec = pos_w - trans_wc_;
    const auto cam_to_lm_dist = cam_to_lm_vec.norm();
    const auto margin_far = 1.3;
    const auto margin_near = 1.0 / margin_far;
    if (!lm->is_inside_in_orb_scale(cam_to_lm_dist, margin_far, margin_near)) {
        return false;
    }

    const Vec3_t obs_mean_normal = lm->get_obs_mean_normal();
    const auto ray_cos = cam_to_lm_vec.dot(obs_mean_normal) / cam_to_lm_dist;
    if (ray_cos < ray_cos_thr) {
        return false;
    }

    pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, this->orb_params_->num_levels_, this->orb_params_->log_scale_factor_);
    return true;
}

bool Frame::has_landmark(const std::shared_ptr<Landmark>& lm) const {
    return static_cast<bool>(landmarks_idx_map_.count(lm));
}

void Frame::add_landmark(const std::shared_ptr<Landmark>& lm, const unsigned int idx) {
    SPDLOG_TRACE("Frame::add_landmark {} {} {}", id_, lm->id_, idx);
    assert(!has_landmark(lm));
    landmarks_.at(idx) = lm;
    landmarks_idx_map_[lm] = idx;
}

std::shared_ptr<Landmark> Frame::get_landmark(const unsigned int idx) const {
    return landmarks_.at(idx);
}

void Frame::erase_landmark_with_index(const unsigned int idx) {
    assert(landmarks_.at(idx));
    landmarks_idx_map_.erase(landmarks_.at(idx));
    landmarks_.at(idx) = nullptr;
}

void Frame::erase_landmark(const std::shared_ptr<Landmark>& lm) {
    assert(has_landmark(lm));
    auto idx = landmarks_idx_map_[lm];
    landmarks_idx_map_.erase(lm);
    landmarks_.at(idx) = nullptr;
}

std::vector<std::shared_ptr<Landmark>> Frame::get_landmarks() const {
    return landmarks_;
}

void Frame::erase_landmarks() {
    std::fill(landmarks_.begin(), landmarks_.end(), nullptr);
    landmarks_idx_map_.clear();
}

void Frame::set_landmarks(const std::vector<std::shared_ptr<Landmark>>& landmarks) {
    erase_landmarks();
    for (unsigned int idx = 0; idx < landmarks.size(); ++idx) {
        const auto& lm = landmarks.at(idx);
        if (lm) {
            add_landmark(lm, idx);
        }
    }
}

std::vector<unsigned int> Frame::get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level, const int max_level) const {
    return data::get_keypoints_in_cell(camera_, frm_obs_, ref_x, ref_y, margin, min_level, max_level);
}

Vec3_t Frame::triangulate_stereo(const unsigned int idx) const {
    return data::triangulate_stereo(camera_, rot_wc_, trans_wc_, frm_obs_, idx);
}

} // namespace data
}} // namespace vo // namespace uavloc
