#pragma once

#include "uavloc/common/type.h"

#include <vector>

namespace uavloc {
namespace vo {

namespace camera {
class Base;
} // namespace camera

namespace data {
class Frame;
} // namespace data

namespace initialize {

class Base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Base() = delete;

    //! Constructor
    Base(const data::Frame& ref_frm,
         const unsigned int num_ransac_iters,
         const unsigned int min_num_valid_pts,
         const unsigned int min_num_triangulated,
         const float parallax_deg_thr,
         const float reproj_err_thr);

    //! Destructor
    virtual ~Base() = default;

    //! Initialize with the current Frame
    virtual bool initialize(const data::Frame& cur_frm, const std::vector<int>& ref_matches_with_cur) = 0;

    //! Get the rotation from the reference to the current
    Mat33_t get_rotation_ref_to_cur() const;

    //! Get the translation from the reference to the current
    Vec3_t get_translation_ref_to_cur() const;

    //! Get the triangulated 3D points with the origin of the reference Frame
    eigen_alloc_vector<Vec3_t> get_triangulated_pts() const;

    //! Get the valid/invalid flags of triangulated 3D points as keypoint indices in the reference Frame
    std::vector<bool> get_triangulated_flags() const;

protected:
    //! Find the most plausible pose and set them to the member variables (outputs)
    bool find_most_plausible_pose(const eigen_alloc_vector<Mat33_t>& init_rots, const eigen_alloc_vector<Vec3_t>& init_transes,
                                  const std::vector<bool>& is_inlier_match, const bool depth_is_positive);

    //! Generate 3D points from matches with valid and sufficient parallax
    unsigned int triangulate(const Mat33_t& rot_ref_to_cur, const Vec3_t& trans_ref_to_cur,
                             const std::vector<bool>& is_inlier_match, const bool depth_is_positive,
                             eigen_alloc_vector<Vec3_t>& triangulated_pts,
                             std::vector<bool>& is_triangulated,
                             unsigned int& num_triangulated_pts,
                             float& parallax_deg);

    //-----------------------------------------
    // reference Frame information

    //! camera model of reference Frame
    camera::Base* const ref_camera_;
    //! undistorted keypoints of reference Frame
    const std::vector<cv::KeyPoint> ref_undist_keypts_;
    //! bearing vectors of reference Frame
    const eigen_alloc_vector<Vec3_t> ref_bearings_;

    //-----------------------------------------
    // current Frame information

    //! camera matrix of current Frame
    camera::Base* cur_camera_;
    //! undistorted keypoints of current Frame
    std::vector<cv::KeyPoint> cur_undist_keypts_;
    //! bearing vectors of current Frame
    eigen_alloc_vector<Vec3_t> cur_bearings_;

    //-----------------------------------------
    // matching information

    //! matching between reference and current frames
    std::vector<std::pair<int, int>> ref_cur_matches_;

    //-----------------------------------------
    // parameters

    //! max number of iterations of RANSAC
    const unsigned int num_ransac_iters_;
    //! min number of triangulated pts
    const unsigned int min_num_triangulated_;
    //! min number of valid pts
    const unsigned int min_num_valid_pts_;
    //! min parallax
    const float parallax_deg_thr_;
    //! reprojection error threshold
    const float reproj_err_thr_;

    //-----------------------------------------
    // output variables

    //! initial rotation from reference to current
    Mat33_t rot_ref_to_cur_ = Mat33_t::Identity();
    //! initial translation from reference to current
    Vec3_t trans_ref_to_cur_ = Vec3_t::Zero();
    //! triangulated pts, with respect to indices of reference Frame
    eigen_alloc_vector<Vec3_t> triangulated_pts_;
    //! each indices of reference Frame is successfully triangulated or not
    std::vector<bool> is_triangulated_;
};

} // namespace initialize
}} // namespace vo // namespace uavloc
