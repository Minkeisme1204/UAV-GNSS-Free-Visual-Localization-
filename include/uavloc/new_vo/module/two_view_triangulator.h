#pragma once

#include "uavloc/common/type.h"

#include <memory>

namespace uavloc {
namespace vo {

namespace camera {
class Base;
} // namespace camera

namespace data {
class Keyframe;
} // namespace data

namespace module {

class TwoViewTriangulator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor
     */
    explicit TwoViewTriangulator(const std::shared_ptr<data::Keyframe>& keyfrm_1, const std::shared_ptr<data::Keyframe>& keyfrm_2,
                                   const float rays_parallax_deg_thr = 1.0);

    /**
     * Destructor
     */
    ~TwoViewTriangulator() = default;

    /**
     * Triangulate a Landmark between the keypoint idx_1 of keyfrm_1 and the keypoint idx_2 of keyfrm_2
     */
    bool triangulate(const unsigned idx_1, const unsigned int idx_2, Vec3_t& pos_w) const;

private:
    /**
     * Check depth is positive or not (if camera model is Equirectangular, always return true)
     */
    bool check_depth_is_positive(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw, camera::Base* camera) const;

    /**
     * Check reprojection error is within the acceptable threshold
     */
    template<typename T>
    bool check_reprojection_error(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw, camera::Base* camera,
                                  const cv::Point_<T>& keypt, const float x_right, const float sigma_sq, const bool is_stereo) const;

    /**
     * Check estimated and actual scale factors are within the acceptable threshold
     */
    bool check_scale_factors(const Vec3_t& pos_w, const float scale_factor_1, const float scale_factor_2) const;

    //! pointer to Keyframe 1
    std::shared_ptr<data::Keyframe> const keyfrm_1_;
    //! pointer to Keyframe 2
    std::shared_ptr<data::Keyframe> const keyfrm_2_;

    // camera poses of Keyframe 1
    const Mat33_t rot_1w_;
    const Mat33_t rot_w1_;
    const Vec3_t trans_1w_;
    const Mat44_t cam_pose_1w_;
    const Vec3_t cam_center_1_;

    // camera model of Keyframe 1
    camera::Base* const camera_1_;

    // camera poses fo Keyframe 2
    const Mat33_t rot_2w_;
    const Mat33_t rot_w2_;
    const Vec3_t trans_2w_;
    const Mat44_t cam_pose_2w_;
    const Vec3_t cam_center_2_;

    // camera model of Keyframe 2
    camera::Base* const camera_2_;

    const float ratio_factor_;

    const float cos_rays_parallax_thr_;
};

inline bool TwoViewTriangulator::check_depth_is_positive(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw, camera::Base* const camera) const {
    // Perspective is the only camera model; depth must be positive (cheirality).
    (void)camera;
    const auto pos_z = rot_cw.block<1, 3>(2, 0).dot(pos_w) + trans_cw(2);
    return 0 < pos_z;
}

inline bool TwoViewTriangulator::check_scale_factors(const Vec3_t& pos_w, const float scale_factor_1, const float scale_factor_2) const {
    const Vec3_t cam_1_to_lm_vec = pos_w - cam_center_1_;
    const auto cam_1_to_lm_dist = cam_1_to_lm_vec.norm();

    const Vec3_t cam_2_to_lm_vec = pos_w - cam_center_2_;
    const auto cam_2_to_lm_dist = cam_2_to_lm_vec.norm();

    if (cam_1_to_lm_dist == 0 || cam_2_to_lm_dist == 0) {
        return false;
    }

    const auto ratio_dists = cam_2_to_lm_dist / cam_1_to_lm_dist;
    const auto ratio_octave = scale_factor_1 / scale_factor_2;

    return ratio_octave / ratio_dists < ratio_factor_ && ratio_dists / ratio_octave < ratio_factor_;
}

} // namespace module
}} // namespace vo // namespace uavloc

