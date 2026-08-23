#include "uavloc/new_vo/camera/perspective_camera.h"

#include <algorithm>
#include <vector>

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace camera {

PerspectiveCamera::PerspectiveCamera(const sensor::CameraModel& model)
    : model_(model) {
    const auto& in = model_.getIntrinsics();
    fx_ = in.fx;
    fy_ = in.fy;
    cx_ = in.cx;
    cy_ = in.cy;
    fx_inv_ = (fx_ != 0.0) ? 1.0 / fx_ : 0.0;
    fy_inv_ = (fy_ != 0.0) ? 1.0 / fy_ : 0.0;
    cols_ = in.width;
    rows_ = in.height;

    eigen_cam_matrix_ = in.K;

    name_ = in.camera_id;
    setup_type_ = SetupType::Monocular;
    model_type_ = ModelType::Perspective;
    // Must stay last: compute_image_bounds() dispatches to undistort_point(),
    // which reads fx_/fy_/cx_/cy_ and model_ (mirrors stella's ctor ordering).
    img_bounds_ = compute_image_bounds();
}

ImageBounds PerspectiveCamera::compute_image_bounds() const {
    spdlog::debug("compute image bounds");

    // distCoeffs() order: k1, k2, p1, p2, k3
    const Eigen::Matrix<double, 5, 1>& dist = model_.distCoeffs();

    if (dist(0) == 0.0 && dist(1) == 0.0 && dist(2) == 0.0 && dist(3) == 0.0 && dist(4) == 0.0) {
        // any distortion does not exist
        return ImageBounds(0.0f, static_cast<float>(cols_), 0.0f, static_cast<float>(rows_));
    }

    // distortion exists

    // corner coordinates: (x, y) = (col, row)
    const float cols = static_cast<float>(cols_);
    const float rows = static_cast<float>(rows_);
    const std::vector<cv::KeyPoint> corners{cv::KeyPoint(0.0f, 0.0f, 1.0f),   // left top
                                            cv::KeyPoint(cols, 0.0f, 1.0f),   // right top
                                            cv::KeyPoint(0.0f, rows, 1.0f),   // left bottom
                                            cv::KeyPoint(cols, rows, 1.0f)};  // right bottom

    std::vector<cv::KeyPoint> undist_corners;
    undistort_keypoints(corners, undist_corners);

    return ImageBounds(std::min(undist_corners.at(0).pt.x, undist_corners.at(2).pt.x),
                       std::max(undist_corners.at(1).pt.x, undist_corners.at(3).pt.x),
                       std::min(undist_corners.at(0).pt.y, undist_corners.at(1).pt.y),
                       std::max(undist_corners.at(2).pt.y, undist_corners.at(3).pt.y));
}

cv::Point2f PerspectiveCamera::undistort_point(const cv::Point2f& dist_pt) const {
    // normalizePixel undistorts to normalised camera coordinates; map to pixels.
    const cv::Point2f n = model_.normalizePixel(dist_pt);
    return cv::Point2f(static_cast<float>(fx_ * n.x + cx_),
                       static_cast<float>(fy_ * n.y + cy_));
}

Vec3_t PerspectiveCamera::convert_point_to_bearing(const cv::Point2f& undist_pt) const {
    const double x_n = (static_cast<double>(undist_pt.x) - cx_) / fx_;
    const double y_n = (static_cast<double>(undist_pt.y) - cy_) / fy_;
    return Vec3_t(x_n, y_n, 1.0).normalized();
}

cv::Point2f PerspectiveCamera::convert_bearing_to_point(const Vec3_t& bearing) const {
    return cv::Point2f(static_cast<float>(fx_ * bearing(0) / bearing(2) + cx_),
                       static_cast<float>(fy_ * bearing(1) / bearing(2) + cy_));
}

bool PerspectiveCamera::reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw,
                                           const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const {
    const Vec3_t pos_c = rot_cw * pos_w + trans_cw;
    x_right = -1.0f;
    if (pos_c(2) <= 0.0) {
        return false;
    }
    const double z_inv = 1.0 / pos_c(2);
    reproj(0) = fx_ * pos_c(0) * z_inv + cx_;
    reproj(1) = fy_ * pos_c(1) * z_inv + cy_;
    return (img_bounds_.min_x_ <= reproj(0) && reproj(0) < img_bounds_.max_x_
            && img_bounds_.min_y_ <= reproj(1) && reproj(1) < img_bounds_.max_y_);
}

bool PerspectiveCamera::reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw,
                                             const Vec3_t& pos_w, Vec3_t& reproj) const {
    const Vec3_t pos_c = rot_cw * pos_w + trans_cw;
    bool in_image = false;
    if (pos_c(2) > 0.0) {
        const double z_inv = 1.0 / pos_c(2);
        const double u = fx_ * pos_c(0) * z_inv + cx_;
        const double v = fy_ * pos_c(1) * z_inv + cy_;
        in_image = (img_bounds_.min_x_ <= u && u < img_bounds_.max_x_
                    && img_bounds_.min_y_ <= v && v < img_bounds_.max_y_);
    }
    reproj = pos_c.normalized();
    return in_image;
}

} // namespace camera
}} // namespace vo // namespace uavloc
