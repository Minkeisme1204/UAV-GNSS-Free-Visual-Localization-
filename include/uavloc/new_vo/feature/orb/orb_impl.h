#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace uavloc {
namespace vo {
namespace feature {

class OrbImpl {
public:
    OrbImpl();
    float ic_angle(const cv::Mat& image, const cv::Point2f& point) const;
    void compute_orb_descriptor(const cv::KeyPoint& keypt, const cv::Mat& image, uchar* desc) const;

    //! BRIEF orientation
    static constexpr unsigned int fast_patch_size_ = 31;
    //! half size of FAST patch
    static constexpr int fast_half_patch_size_ = fast_patch_size_ / 2;

private:
    //! Index limitation that used for calculating of keypoint orientation
    std::vector<int> u_max_;
};

} // namespace feature
} // namespace vo
} // namespace uavloc


