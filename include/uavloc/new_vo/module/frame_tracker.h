#pragma once

#include "uavloc/common/type.h"
#include "uavloc/new_vo/optimize/pose_optimizer.h"

#include <memory>

namespace uavloc {
namespace vo {

namespace camera {
class Base;
} // namespace camera

namespace data {
class Frame;
class Keyframe;
} // namespace data

namespace module {

class FrameTracker {
public:
    explicit FrameTracker(camera::Base* camera,
                           const std::shared_ptr<optimize::PoseOptimizer>& pose_optimizer,
                           const unsigned int num_matches_thr = 20,
                           bool use_fixed_seed = false,
                           float margin = 20.0);

    bool motion_based_track(data::Frame& curr_frm, const data::Frame& last_frm, const Mat44_t& velocity) const;

    bool robust_match_based_track(data::Frame& curr_frm, const data::Frame& last_frm, const std::shared_ptr<data::Keyframe>& ref_keyfrm) const;

private:
    unsigned int discard_outliers(const std::vector<bool>& outlier_flags, data::Frame& curr_frm) const;

    const camera::Base* camera_;
    const unsigned int num_matches_thr_;
    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_;
    //! margin for projection matcher
    const float margin_;

    std::shared_ptr<optimize::PoseOptimizer> pose_optimizer_ = nullptr;
};

} // namespace module
}} // namespace vo // namespace uavloc

