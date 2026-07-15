#pragma once

#include "uavloc/common/type.h"
#include "uavloc/new_vo/initialize/base.h"

namespace uavloc {
namespace vo {

namespace data {
class Frame;
} // namespace data

namespace initialize {

class BearingVector final : public Base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BearingVector() = delete;

    //! Constructor
    BearingVector(const data::Frame& ref_frm,
                   const unsigned int num_ransac_iters,
                   const unsigned int min_num_triangulated,
                   const unsigned int min_num_valid_pts,
                   const float parallax_deg_thr,
                   const float reproj_err_thr,
                   bool use_fixed_seed = false);

    //! Destructor
    ~BearingVector() override;

    //! Initialize with the current Frame
    bool initialize(const data::Frame& cur_frm, const std::vector<int>& ref_matches_with_cur) override;

private:
    //! Reconstruct the initial map with the E matrix
    //! (NOTE: the output variables will be set if succeeded)
    bool reconstruct_with_E(const Mat33_t& E_ref_to_cur, const std::vector<bool>& is_inlier_match);

    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_;
};

} // namespace initialize
}} // namespace vo // namespace uavloc
