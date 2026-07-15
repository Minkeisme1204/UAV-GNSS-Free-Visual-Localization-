#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/initialize/bearing_vector.h"
#include "uavloc/new_vo/solve/essential_solver.h"

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace initialize {

BearingVector::BearingVector(const data::Frame& ref_frm,
                               const unsigned int num_ransac_iters,
                               const unsigned int min_num_triangulated,
                               const unsigned int min_num_valid_pts,
                               const float parallax_deg_thr,
                               const float reproj_err_thr,
                               bool use_fixed_seed)
    : Base(ref_frm, num_ransac_iters, min_num_triangulated, min_num_valid_pts, parallax_deg_thr, reproj_err_thr),
      use_fixed_seed_(use_fixed_seed) {
    spdlog::debug("CONSTRUCT: initialize::BearingVector");
}

BearingVector::~BearingVector() {
    spdlog::debug("DESTRUCT: initialize::BearingVector");
}

bool BearingVector::initialize(const data::Frame& cur_frm, const std::vector<int>& ref_matches_with_cur) {
    // set the current camera model
    cur_camera_ = cur_frm.camera_;
    // store the keypoints and bearings
    cur_undist_keypts_ = cur_frm.frm_obs_.undist_keypts_;
    cur_bearings_ = cur_frm.frm_obs_.bearings_;
    // align matching information
    ref_cur_matches_.clear();
    ref_cur_matches_.reserve(cur_frm.frm_obs_.undist_keypts_.size());
    for (unsigned int ref_idx = 0; ref_idx < ref_matches_with_cur.size(); ++ref_idx) {
        const auto cur_idx = ref_matches_with_cur.at(ref_idx);
        if (0 <= cur_idx) {
            ref_cur_matches_.emplace_back(std::make_pair(ref_idx, cur_idx));
        }
    }

    // compute an E matrix
    auto EssentialSolver = solve::EssentialSolver(ref_bearings_, cur_bearings_, ref_cur_matches_, use_fixed_seed_);
    EssentialSolver.find_via_ransac(num_ransac_iters_, false);

    // reconstruct map if the solution is valid
    if (EssentialSolver.solution_is_valid()) {
        const Mat33_t E_ref_to_cur = EssentialSolver.get_best_E_21();
        const auto is_inlier_match = EssentialSolver.get_inlier_matches();
        return reconstruct_with_E(E_ref_to_cur, is_inlier_match);
    }
    else {
        return false;
    }
}

bool BearingVector::reconstruct_with_E(const Mat33_t& E_ref_to_cur, const std::vector<bool>& is_inlier_match) {
    // found the most plausible pose from the FOUR hypothesis computed from the E matrix

    // decompose the E matrix
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    if (!solve::EssentialSolver::decompose(E_ref_to_cur, init_rots, init_transes)) {
        return false;
    }

    assert(init_rots.size() == 4);
    assert(init_transes.size() == 4);

    const auto pose_is_found = find_most_plausible_pose(init_rots, init_transes, is_inlier_match, false);
    if (!pose_is_found) {
        return false;
    }

    spdlog::info("initialization succeeded with E");
    return true;
}

} // namespace initialize
}} // namespace vo // namespace uavloc
