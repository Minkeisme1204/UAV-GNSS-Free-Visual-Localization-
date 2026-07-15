#pragma once

#include "uavloc/common/type.h"
#include "uavloc/new_vo/match/base.h"

#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Frame;
struct FrameObservation;
class Keyframe;
class Landmark;
} // namespace data

namespace match {

class Robust final : public Base {
public:
    explicit Robust(const float lowe_ratio, const bool check_orientation)
        : Base(lowe_ratio, check_orientation) {}

    ~Robust() final = default;

    unsigned int match_for_triangulation(const std::shared_ptr<data::Keyframe>& keyfrm_1,
                                         const std::shared_ptr<data::Keyframe>& keyfrm_2,
                                         const Mat33_t& E_12,
                                         std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs,
                                         const float residual_rad_thr) const;

    unsigned int match_keyframes(const std::shared_ptr<data::Keyframe>& keyfrm1, const std::shared_ptr<data::Keyframe>& keyfrm2,
                                 std::vector<std::shared_ptr<data::Landmark>>& matched_lms_in_frm,
                                 bool validate_with_essential_solver = true, bool use_fixed_seed = false) const;

    unsigned int match_frame_and_keyframe(data::Frame& frm, const std::shared_ptr<data::Keyframe>& keyfrm,
                                          std::vector<std::shared_ptr<data::Landmark>>& matched_lms_in_frm,
                                          bool use_fixed_seed = false) const;

    unsigned int brute_force_match(const data::FrameObservation& frm_obs, const std::shared_ptr<data::Keyframe>& keyfrm, std::vector<std::pair<int, int>>& matches) const;
};

} // namespace match
}} // namespace vo // namespace uavloc
