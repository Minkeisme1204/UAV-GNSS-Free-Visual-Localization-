#include "uavloc/sensor/camera_model.h"
#include <spdlog/spdlog.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <stdexcept>

namespace uavloc::sensor {

// ─── CameraIntrinsics ─────────────────────────────────────────────────────────

CameraIntrinsics CameraIntrinsics::fromYaml(const YAML::Node& node) {
    const auto& n = node["Camera"];
    if (!n) throw std::runtime_error("YAML missing 'Camera' section");

    CameraIntrinsics intr;
    intr.fx        = n["fx"].as<double>(intr.fx);
    intr.fy        = n["fy"].as<double>(intr.fy);
    intr.cx        = n["cx"].as<double>(intr.cx);
    intr.cy        = n["cy"].as<double>(intr.cy);
    intr.width     = n["width"].as<int>(intr.width);
    intr.height    = n["height"].as<int>(intr.height);
    intr.camera_id = n["camera_id"].as<std::string>(intr.camera_id);

    // Build K from individual intrinsics
    intr.K << intr.fx,      0.0, intr.cx,
                  0.0, intr.fy, intr.cy,
                  0.0,      0.0,     1.0;

    intr.dist_coeffs(0) = n["k1"].as<double>(0.0);
    intr.dist_coeffs(1) = n["k2"].as<double>(0.0);
    intr.dist_coeffs(2) = n["p1"].as<double>(0.0);
    intr.dist_coeffs(3) = n["p2"].as<double>(0.0);
    intr.dist_coeffs(4) = n["k3"].as<double>(0.0);

    return intr;
}

// ─── CameraModel ─────────────────────────────────────────────────────────────

CameraModel::CameraModel(const CameraIntrinsics& intrinsics)
    : intrinsics_(intrinsics) {}

const CameraIntrinsics& CameraModel::getIntrinsics() const { return intrinsics_; }

const Eigen::Matrix3d& CameraModel::K() const { return intrinsics_.K; }

const Eigen::Matrix<double, 5, 1>& CameraModel::distCoeffs() const {
    return intrinsics_.dist_coeffs;
}

int  CameraModel::width()  const { return intrinsics_.width; }
int  CameraModel::height() const { return intrinsics_.height; }

bool CameraModel::isValid() const {
    return intrinsics_.fx > 0.0 && intrinsics_.fy > 0.0
        && intrinsics_.width > 0 && intrinsics_.height > 0;
}

cv::Point2f CameraModel::normalizePixel(const cv::Point2f& pixel) const {
    std::vector<cv::Point2f> in  = {pixel};
    std::vector<cv::Point2f> out;
    undistortPoints(in, out);
    return out.front();
}

std::vector<cv::Point2f> CameraModel::normalizePixels(
    const std::vector<cv::Point2f>& pixels) const
{
    std::vector<cv::Point2f> out;
    undistortPoints(pixels, out);
    return out;
}

void CameraModel::undistortImage(const cv::Mat& input, cv::Mat& output) const {
    cv::Mat K_mat, D_mat;
    cv::eigen2cv(intrinsics_.K,            K_mat);
    cv::eigen2cv(intrinsics_.dist_coeffs,  D_mat);
    cv::undistort(input, output, K_mat, D_mat);
}

void CameraModel::undistortPoints(
    const std::vector<cv::Point2f>& input,
    std::vector<cv::Point2f>&       output) const
{
    if (input.empty()) { output.clear(); return; }
    cv::Mat K_mat, D_mat;
    cv::eigen2cv(intrinsics_.K,           K_mat);
    cv::eigen2cv(intrinsics_.dist_coeffs, D_mat);
    // Passing cv::noArray() as P keeps output in normalised camera coordinates
    cv::undistortPoints(input, output, K_mat, D_mat);
}

}  // namespace uavloc::sensor
