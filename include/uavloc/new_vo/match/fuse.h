#pragma once

// match::Fuse — landmark-fusion matcher for the new_vo pipeline.
//
// Faithful port of stella_vslam::match::fuse (see .repo/stella_vslam
// src/stella_vslam/match/fuse.{h,cc}), trimmed to the monocular VO case:
// reprojects a set of 3D landmarks into a keyframe, finds the closest ORB
// keypoint, and classifies each hit as either a duplication with a landmark
// already observed by that keyframe (-> duplicated_lms_in_keyfrm) or a fresh
// association on an empty keypoint (-> new_connections). Reuses match::Base
// (Hamming distance) and the ported data:: types.

#include "uavloc/common/type.h"
#include "uavloc/new_vo/match/base.h"

#include <memory>
#include <unordered_map>

namespace uavloc {
namespace vo {

namespace data {
class Keyframe;
class Landmark;
class MapDatabase;
} // namespace data

namespace match {

class Fuse final : public Base {
public:
    explicit Fuse(const float lowe_ratio)
        : Base(lowe_ratio, false) {}

    ~Fuse() final = default;

    //! Reproject landmarks_to_check into keyfrm and detect duplication with the
    //! landmarks already observed by keyfrm.
    template<typename T>
    unsigned int detect_duplication(const std::shared_ptr<data::Keyframe>& keyfrm,
                                    const Mat33_t& rot_cw,
                                    const Vec3_t& trans_cw,
                                    const T& landmarks_to_check,
                                    const float margin,
                                    std::unordered_map<std::shared_ptr<data::Landmark>, std::shared_ptr<data::Landmark>>& duplicated_lms_in_keyfrm,
                                    std::unordered_map<unsigned int, std::shared_ptr<data::Landmark>>& new_connections,
                                    bool do_reprojection_matching = false) const;
};

} // namespace match
}} // namespace vo // namespace uavloc
