#include "uavloc/new_vo/camera/base.h"

#include <algorithm>
#include <iterator>
#include <vector>

// Bodies for the non-pure virtual batch helpers of camera::Base. Ported from
// stella_vslam::camera::base (generic implementations that loop over the pure
// virtuals undistort_point / convert_point_to_bearing / convert_bearing_to_point).
// Providing them here also emits camera::Base's vtable so concrete adapters
// (e.g. camera::PerspectiveCamera) link cleanly.

namespace uavloc {
namespace vo {
namespace camera {

void Base::undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypts,
                               std::vector<cv::KeyPoint>& undist_keypts) const {
    undist_keypts.resize(dist_keypts.size());
    for (unsigned long idx = 0; idx < dist_keypts.size(); ++idx) {
        undist_keypts.at(idx) = dist_keypts.at(idx);
        undist_keypts.at(idx).pt = undistort_point(dist_keypts.at(idx).pt);
    }
}

void Base::convert_keypoints_to_bearings(const std::vector<cv::KeyPoint>& undist_keypts,
                                         eigen_alloc_vector<Vec3_t>& bearings) const {
    bearings.clear();
    std::transform(undist_keypts.begin(), undist_keypts.end(), std::back_inserter(bearings),
                   [this](const cv::KeyPoint& undist_keypt) { return convert_point_to_bearing(undist_keypt.pt); });
}

void Base::convert_bearings_to_points(const eigen_alloc_vector<Vec3_t>& bearings,
                                      std::vector<cv::Point2f>& undist_pts) const {
    undist_pts.clear();
    std::transform(bearings.begin(), bearings.end(), std::back_inserter(undist_pts),
                   [this](const Vec3_t& bearing) { return convert_bearing_to_point(bearing); });
}

} // namespace camera
}} // namespace vo // namespace uavloc
