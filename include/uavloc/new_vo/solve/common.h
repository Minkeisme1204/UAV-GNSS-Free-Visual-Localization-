#pragma once

#include "uavloc/common/type.h"

#include <vector>

#include <opencv2/core/types.hpp>

namespace uavloc {
namespace vo {
namespace solve {

void normalize(const std::vector<cv::KeyPoint>& keypts, std::vector<cv::Point2f>& normalized_pts, Mat33_t& transform);

} // namespace solve
}} // namespace vo // namespace uavloc
