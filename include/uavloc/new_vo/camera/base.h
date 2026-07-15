#pragma once

// camera::Base — abstract camera interface for the new_vo pipeline.
//
// Mirrors the geometry-related subset of stella_vslam::camera::base that the
// ported code references. The only concrete implementation is
// camera::PerspectiveCamera (a thin adapter over sensor::CameraModel); the
// pipeline is monocular / perspective-only. The non-pure batch helpers
// (undistort_keypoints / convert_*_to_*) loop over the pure virtuals and are
// defined in src/new_vo/camera/base.cpp (which also emits Base's vtable).

#include "uavloc/common/type.h"

#include <array>
#include <string>

#include <opencv2/core/types.hpp>

namespace uavloc {
namespace vo {
namespace camera {

//! Camera setup type (mirrors stella_vslam::camera::setup_type_t).
enum class SetupType {
    Monocular = 0,
    Stereo = 1,
    RGBD = 2
};

//! Camera projection model type (mirrors stella_vslam::camera::model_type_t).
//! Only Perspective is implemented; the enum is kept single-valued so the
//! constant model_type_ member on Base stays meaningful with minimal churn.
enum class ModelType {
    Perspective = 0
};

//! Valid image region after undistortion (mirrors stella_vslam::camera::ImageBounds).
struct ImageBounds {
    ImageBounds() = default;

    template<typename T, typename U>
    ImageBounds(const T min_x, const U max_x, const T min_y, const U max_y)
        : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y) {}

    float min_x_ = 0.0;
    float max_x_ = 0.0;
    float min_y_ = 0.0;
    float max_y_ = 0.0;
};

//! Abstract camera base; faithful to the geometry-related members referenced
//! across the port.
class Base {
public:
    virtual ~Base() = default;

    //! camera name (id for saving)
    std::string name_;

    //! setup type (monocular / stereo / RGBD)
    SetupType setup_type_ = SetupType::Monocular;

    //! projection model type
    ModelType model_type_ = ModelType::Perspective;

    //! image bounds used by the keypoint grid (data/common.{h,cc})
    ImageBounds img_bounds_;

    //! true baseline (referenced by match/projection.cc forward/backward test)
    float true_baseline_ = 0.0;

    //! focal length x baseline for stereo reprojection edges
    //! (optimize/ se3 reproj + pose-opt edge wrappers).
    float focal_x_baseline_ = 0.0;

    //! depth threshold used by Keyframe insertion / local-map cleaning
    //! (module/keyframe_inserter.cc, module/local_map_cleaner.cc).
    float depth_thr_ = 0.0;

    //! Undistort a distorted point
    virtual cv::Point2f undistort_point(const cv::Point2f& dist_pt) const = 0;

    //! Convert undistorted point to bearing vector
    virtual Vec3_t convert_point_to_bearing(const cv::Point2f& undist_pt) const = 0;

    //! Convert bearing vector to undistorted point
    virtual cv::Point2f convert_bearing_to_point(const Vec3_t& bearing) const = 0;

    //! Reproject a world point to the image plane
    //! (reprojected to inside of image -> true, to outside of image -> false)
    virtual bool reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const = 0;

    //! Reproject a world point to a bearing vector
    //! (reprojected to inside of image -> true, to outside of image -> false)
    virtual bool reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec3_t& reproj) const = 0;

    //! Undistort keypoints
    virtual void undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypts, std::vector<cv::KeyPoint>& undist_keypts) const;

    //! Convert undistorted keypoints to bearing vectors
    virtual void convert_keypoints_to_bearings(const std::vector<cv::KeyPoint>& undist_keypts, eigen_alloc_vector<Vec3_t>& bearings) const;

    //! Convert bearing vectors to undistorted points
    virtual void convert_bearings_to_points(const eigen_alloc_vector<Vec3_t>& bearings, std::vector<cv::Point2f>& undist_pts) const;
};

} // namespace camera
}} // namespace vo // namespace uavloc
