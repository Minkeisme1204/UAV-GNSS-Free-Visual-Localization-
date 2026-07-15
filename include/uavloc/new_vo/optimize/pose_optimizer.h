#pragma once

#include "uavloc/common/type.h"

namespace uavloc {
namespace vo {

namespace data {
class Frame;
struct FrameObservation;
class Keyframe;
} // namespace data

namespace camera {
class Base;
} // namespace camera

namespace feature {
struct OrbParams;
} // namespace feature

namespace optimize {

class PoseOptimizer {
public:
    /**
     * Perform pose optimization
     * @param frm
     * @return
     */
    virtual unsigned int optimize(const data::Frame& frm, Mat44_t& optimized_pose, std::vector<bool>& outlier_flags) const = 0;
    virtual unsigned int optimize(const data::Keyframe* keyfrm, Mat44_t& optimized_pose, std::vector<bool>& outlier_flags) const = 0;

    virtual unsigned int optimize(const Mat44_t& cam_pose_cw, const data::FrameObservation& frm_obs,
                                  const feature::OrbParams* OrbParams,
                                  const camera::Base* camera,
                                  const std::vector<std::shared_ptr<data::Landmark>>& landmarks,
                                  Mat44_t& optimized_pose,
                                  std::vector<bool>& outlier_flags) const = 0;
};

} // namespace optimize
}} // namespace vo // namespace uavloc
