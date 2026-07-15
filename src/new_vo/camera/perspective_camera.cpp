#include "uavloc/new_vo/camera/perspective_camera.h"

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
    img_bounds_ = ImageBounds(0.0f, static_cast<float>(cols_), 0.0f, static_cast<float>(rows_));
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
