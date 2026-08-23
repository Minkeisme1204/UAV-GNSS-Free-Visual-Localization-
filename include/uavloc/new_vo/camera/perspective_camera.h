#pragma once

// PerspectiveCamera — concrete camera::Base backed by the sensor::CameraModel.
//
// Adapter (design decision "a"): reuses the project's sensor camera model
// (intrinsics + Brown-Conrady distortion) so the new_vo geometry seam is a real
// implementation, not a stub. Monocular / perspective only. Distortion is
// handled through sensor::CameraModel::undistortPoints (returns normalised
// coordinates), then mapped back to pixel space with the pinhole intrinsics.
//
// The adapter itself is NEW code and follows the project PascalCase convention,
// but compute_image_bounds() IS a faithful mirror of
// stella_vslam::camera::perspective::compute_image_bounds().
// See .docs/designs/new_vo_port_notes.md.

#include "uavloc/new_vo/camera/base.h"
#include "uavloc/sensor/camera_model.h"
#include "uavloc/common/type.h"

#include <opencv2/core/types.hpp>

namespace uavloc {
namespace vo {
namespace camera {

class PerspectiveCamera : public Base {
public:
    PerspectiveCamera() = delete;

    //! Build from a sensor::CameraModel (intrinsics + distortion + image size).
    explicit PerspectiveCamera(const sensor::CameraModel& model);

    ~PerspectiveCamera() override = default;

    ImageBounds compute_image_bounds() const override;

    cv::Point2f undistort_point(const cv::Point2f& dist_pt) const override;

    Vec3_t convert_point_to_bearing(const cv::Point2f& undist_pt) const override;

    cv::Point2f convert_bearing_to_point(const Vec3_t& bearing) const override;

    bool reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w,
                            Vec2_t& reproj, float& x_right) const override;

    bool reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w,
                              Vec3_t& reproj) const override;

    //! Pinhole intrinsics (cached from model_ in the ctor). PUBLIC because the
    //! g2o reprojection / pose-optimisation edge wrappers read them directly
    //! (mirrors stella's camera::perspective public members).
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    double fx_inv_ = 0.0;
    double fy_inv_ = 0.0;

    //! Camera matrix in Eigen form (initialize::Perspective::get_camera_matrix).
    Mat33_t eigen_cam_matrix_ = Mat33_t::Identity();

private:
    //! Borrowed distortion/undistortion backend (held by value; copyable).
    sensor::CameraModel model_;

    //! Image dimensions (cached from model_).
    int cols_ = 0;
    int rows_ = 0;
};

} // namespace camera
}} // namespace vo // namespace uavloc
