#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace uavloc::sensor {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    int width  = 0;
    int height = 0;

    // Intrinsic matrix — Eigen per project matrix-computation rule
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();

    // Brown-Conrady distortion: [k1, k2, p1, p2, k3]
    Eigen::Matrix<double, 5, 1> dist_coeffs = Eigen::Matrix<double, 5, 1>::Zero();

    std::string camera_id = "camera0";

    // Reads: fx, fy, cx, cy, width, height, k1, k2, p1, p2, k3, camera_id
    // Builds K automatically from fx/fy/cx/cy
    static CameraIntrinsics fromYaml(const YAML::Node& node);
};

class CameraModel {
public:
    explicit CameraModel(const CameraIntrinsics& intrinsics);

    const CameraIntrinsics&          getIntrinsics() const;
    const Eigen::Matrix3d&           K()             const;
    const Eigen::Matrix<double,5,1>& distCoeffs()    const;

    int  width()   const;
    int  height()  const;
    bool isValid() const;  // true when fx>0 && fy>0 && width>0 && height>0

    // Converts a distorted pixel to normalised image coordinates (z=1 plane)
    cv::Point2f normalizePixel(const cv::Point2f& pixel) const;

    // Batch version of normalizePixel
    std::vector<cv::Point2f> normalizePixels(
        const std::vector<cv::Point2f>& pixels) const;

    // Removes lens distortion from a full image
    void undistortImage(const cv::Mat& input, cv::Mat& output) const;

    // Removes lens distortion from a set of 2-D points
    void undistortPoints(
        const std::vector<cv::Point2f>& input,
        std::vector<cv::Point2f>&       output) const;

private:
    CameraIntrinsics intrinsics_;
};

}  // namespace uavloc::sensor

#endif  // CAMERA_MODEL_H
