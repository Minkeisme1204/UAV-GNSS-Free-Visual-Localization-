#pragma once

#include "uavloc/new_vo/match/base.h"

namespace uavloc {
namespace vo {

namespace data {
class Frame;
} // namespace data

namespace match {

class Area final : public Base {
public:
    Area(const float lowe_ratio, const bool check_orientation)
        : Base(lowe_ratio, check_orientation) {}

    ~Area() final = default;

    unsigned int match_in_consistent_area(data::Frame& frm_1, data::Frame& frm_2, std::vector<cv::Point2f>& prev_matched_pts,
                                          std::vector<int>& matched_indices_2_in_frm_1, int margin = 10);
};

} // namespace match
}} // namespace vo // namespace uavloc
