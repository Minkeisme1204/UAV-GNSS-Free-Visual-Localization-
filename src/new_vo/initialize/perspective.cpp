#include "uavloc/new_vo/camera/perspective_camera.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/initialize/perspective.h"
#include "uavloc/new_vo/solve/homography_solver.h"
#include "uavloc/new_vo/solve/fundamental_solver.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace initialize {

Perspective::Perspective(const data::Frame& ref_frm,
                         const unsigned int num_ransac_iters,
                         const unsigned int min_num_triangulated,
                         const unsigned int min_num_valid_pts,
                         const float parallax_deg_thr,
                         const float reproj_err_thr,
                         bool use_fixed_seed)
    : Base(ref_frm, num_ransac_iters, min_num_triangulated, min_num_valid_pts, parallax_deg_thr, reproj_err_thr),
      ref_cam_matrix_(get_camera_matrix(ref_frm.camera_)), use_fixed_seed_(use_fixed_seed) {
    spdlog::debug("CONSTRUCT: initialize::Perspective");
}

Perspective::~Perspective() {
    spdlog::debug("DESTRUCT: initialize::Perspective");
}

bool Perspective::initialize(const data::Frame& cur_frm, const std::vector<int>& ref_matches_with_cur) {
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

    // set the current camera matrix
    cur_cam_matrix_ = get_camera_matrix(cur_frm.camera_);

    // compute H and F matrices
    const float sigma = 1.0f;
    auto HomographySolver = solve::HomographySolver(ref_undist_keypts_, cur_undist_keypts_, ref_cur_matches_, sigma, use_fixed_seed_);
    auto FundamentalSolver = solve::FundamentalSolver(ref_undist_keypts_, cur_undist_keypts_, ref_cur_matches_, sigma, use_fixed_seed_);
    std::thread thread_for_H(&solve::HomographySolver::find_via_ransac, &HomographySolver, num_ransac_iters_, false);
    std::thread thread_for_F(&solve::FundamentalSolver::find_via_ransac, &FundamentalSolver, num_ransac_iters_, false);
    thread_for_H.join();
    thread_for_F.join();

    // compute a cost
    const auto cost_H = HomographySolver.get_best_cost();
    const auto cost_F = FundamentalSolver.get_best_cost();
    const float rel_cost_H = cost_H / (cost_H + cost_F);

    // select a case according to the cost
    if (0.5 > rel_cost_H && HomographySolver.solution_is_valid()) {
        spdlog::debug("reconstruct_with_H");
        const Mat33_t H_ref_to_cur = HomographySolver.get_best_H_21();
        const auto is_inlier_match = HomographySolver.get_inlier_matches();
        return reconstruct_with_H(H_ref_to_cur, is_inlier_match);
    }
    else if (FundamentalSolver.solution_is_valid()) {
        spdlog::debug("reconstruct_with_F");
        const Mat33_t F_ref_to_cur = FundamentalSolver.get_best_F_21();
        const auto is_inlier_match = FundamentalSolver.get_inlier_matches();
        return reconstruct_with_F(F_ref_to_cur, is_inlier_match);
    }
    else {
        return false;
    }
}

bool Perspective::reconstruct_with_H(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match) {
    // found the most plausible pose from the EIGHT hypothesis computed from the H matrix

    // decompose the H matrix
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    eigen_alloc_vector<Vec3_t> init_normals;
    if (!solve::HomographySolver::decompose(H_ref_to_cur, ref_cam_matrix_, cur_cam_matrix_, init_rots, init_transes, init_normals)) {
        return false;
    }

    assert(init_rots.size() == 8);
    assert(init_transes.size() == 8);

    const auto pose_is_found = find_most_plausible_pose(init_rots, init_transes, is_inlier_match, true);
    if (!pose_is_found) {
        return false;
    }

    spdlog::info("initialization succeeded with H");
    return true;
}

bool Perspective::reconstruct_with_F(const Mat33_t& F_ref_to_cur, const std::vector<bool>& is_inlier_match) {
    // found the most plausible pose from the FOUR hypothesis computed from the F matrix

    // decompose the F matrix
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    if (!solve::FundamentalSolver::decompose(F_ref_to_cur, ref_cam_matrix_, cur_cam_matrix_, init_rots, init_transes)) {
        return false;
    }

    assert(init_rots.size() == 4);
    assert(init_transes.size() == 4);

    const auto pose_is_found = find_most_plausible_pose(init_rots, init_transes, is_inlier_match, true);
    if (!pose_is_found) {
        return false;
    }

    spdlog::info("initialization succeeded with F");
    return true;
}

Mat33_t Perspective::get_camera_matrix(camera::Base* camera) {
    // Perspective is the only camera model (see camera::PerspectiveCamera).
    auto c = static_cast<camera::PerspectiveCamera*>(camera);
    return c->eigen_cam_matrix_;
}

} // namespace initialize
}} // namespace vo // namespace uavloc
