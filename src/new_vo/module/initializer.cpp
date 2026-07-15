// TODO(port): stella_vslam/config.h not cloned — unused in this translation unit
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/data/map_database.h"
#include "uavloc/new_vo/initialize/bearing_vector.h"
#include "uavloc/new_vo/initialize/perspective.h"
#include "uavloc/new_vo/match/area.h"
#include "uavloc/new_vo/module/initializer.h"
#include "uavloc/new_vo/optimize/global_bundle_adjuster.h"

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace module {

Initializer::Initializer(data::MapDatabase* map_db,
                         const YAML::Node& yaml_node)
    : map_db_(map_db),
      num_ransac_iters_(yaml_node["num_ransac_iterations"].as<unsigned int>(100)),
      min_num_valid_pts_(yaml_node["min_num_valid_pts"].as<unsigned int>(50)),
      min_num_triangulated_pts_(yaml_node["min_num_triangulated_pts"].as<unsigned int>(50)),
      parallax_deg_thr_(yaml_node["parallax_deg_threshold"].as<float>(1.0)),
      reproj_err_thr_(yaml_node["reprojection_error_threshold"].as<float>(4.0)),
      num_ba_iters_(yaml_node["num_ba_iterations"].as<unsigned int>(100)),
      scaling_factor_(yaml_node["scaling_factor"].as<float>(1.0)),
      use_fixed_seed_(yaml_node["use_fixed_seed"].as<bool>(false)),
      gain_threshold_(yaml_node["gain_threshold"].as<float>(1e-5)),
      verbose_(yaml_node["verbose"].as<bool>(false)) {
    spdlog::debug("CONSTRUCT: module::Initializer");
}

Initializer::~Initializer() {
    spdlog::debug("DESTRUCT: module::Initializer");
}

void Initializer::reset() {
    initializer_.reset(nullptr);
    state_ = InitializerState::NotReady;
    init_frm_id_ = 0;
    init_frm_stamp_ = 0.0;
}

InitializerState Initializer::get_state() const {
    return state_;
}

unsigned int Initializer::get_initial_frame_id() const {
    return init_frm_id_;
}

double Initializer::get_initial_frame_timestamp() const {
    return init_frm_stamp_;
}

bool Initializer::get_use_fixed_seed() const {
    return use_fixed_seed_;
}

bool Initializer::initialize(const camera::SetupType setup_type,
                             data::Frame& curr_frm) {
    switch (setup_type) {
        case camera::SetupType::Monocular: {
            // construct an Initializer if not constructed
            if (state_ == InitializerState::NotReady) {
                create_initializer(curr_frm);
                return false;
            }

            // try to initialize
            if (!try_initialize_for_monocular(curr_frm)) {
                // failed
                return false;
            }

            // create new map if succeeded
            create_map_for_monocular(curr_frm);
            break;
        }
        case camera::SetupType::Stereo:
        case camera::SetupType::RGBD: {
            state_ = InitializerState::Initializing;

            // try to initialize
            if (!try_initialize_for_stereo(curr_frm)) {
                // failed
                return false;
            }

            // create new map if succeeded
            create_map_for_stereo(curr_frm);
            break;
        }
        default: {
            throw std::runtime_error("Undefined camera setup");
        }
    }

    // check the state is succeeded or not
    if (state_ == InitializerState::Succeeded) {
        init_frm_id_ = curr_frm.id_;
        init_frm_stamp_ = curr_frm.timestamp_;
        return true;
    }
    else {
        return false;
    }
}

void Initializer::create_initializer(data::Frame& curr_frm) {
    // set the initial Frame
    init_frm_ = data::Frame(curr_frm);

    // initialize the previously matched coordinates
    prev_matched_coords_.resize(init_frm_.frm_obs_.undist_keypts_.size());
    for (unsigned int i = 0; i < init_frm_.frm_obs_.undist_keypts_.size(); ++i) {
        prev_matched_coords_.at(i) = init_frm_.frm_obs_.undist_keypts_.at(i).pt;
    }

    // initialize matchings (init_idx -> curr_idx)
    std::fill(init_matches_.begin(), init_matches_.end(), -1);

    // build a Initializer
    initializer_.reset(nullptr);
    // Perspective is the only camera model (see camera::PerspectiveCamera).
    initializer_ = std::unique_ptr<initialize::Perspective>(
        new initialize::Perspective(
            init_frm_, num_ransac_iters_, min_num_triangulated_pts_, min_num_valid_pts_,
            parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_));

    state_ = InitializerState::Initializing;
}

bool Initializer::try_initialize_for_monocular(data::Frame& curr_frm) {
    assert(state_ == InitializerState::Initializing);

    match::Area matcher(0.9, true);
    const auto num_matches = matcher.match_in_consistent_area(init_frm_, curr_frm, prev_matched_coords_, init_matches_, 100);

    if (num_matches < min_num_valid_pts_) {
        // rebuild the Initializer with the next Frame
        reset();
        return false;
    }

    // try to initialize with the initial Frame and the current Frame
    assert(initializer_);
    spdlog::debug("try to initialize with the initial Frame and the current Frame: Frame {} - Frame {}", init_frm_.id_, curr_frm.id_);
    return initializer_->initialize(curr_frm, init_matches_);
}

bool Initializer::create_map_for_monocular(data::Frame& curr_frm) {
    assert(state_ == InitializerState::Initializing);

    eigen_alloc_vector<Vec3_t> init_triangulated_pts;
    {
        assert(initializer_);
        init_triangulated_pts = initializer_->get_triangulated_pts();
        const auto is_triangulated = initializer_->get_triangulated_flags();

        // make invalid the matchings which have not been triangulated
        for (unsigned int i = 0; i < init_matches_.size(); ++i) {
            if (init_matches_.at(i) < 0) {
                continue;
            }
            if (is_triangulated.at(i)) {
                continue;
            }
            init_matches_.at(i) = -1;
        }

        // set the camera poses
        init_frm_.set_pose_cw(Mat44_t::Identity());
        Mat44_t cam_pose_cw = Mat44_t::Identity();
        cam_pose_cw.block<3, 3>(0, 0) = initializer_->get_rotation_ref_to_cur();
        cam_pose_cw.block<3, 1>(0, 3) = initializer_->get_translation_ref_to_cur();
        curr_frm.set_pose_cw(cam_pose_cw);

        // destruct the Initializer
        initializer_.reset(nullptr);
    }

    // create initial keyframes
    auto init_keyfrm = data::Keyframe::make_keyframe(map_db_->next_keyframe_id_++, init_frm_);
    auto curr_keyfrm = data::Keyframe::make_keyframe(map_db_->next_keyframe_id_++, curr_frm);
    curr_keyfrm->graph_node_->set_spanning_parent(init_keyfrm);
    init_keyfrm->graph_node_->add_spanning_child(curr_keyfrm);
    init_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
    curr_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
    map_db_->add_spanning_root(init_keyfrm);

    // add the keyframes to the map DB
    map_db_->add_keyframe(init_keyfrm);
    map_db_->add_keyframe(curr_keyfrm);

    // update the Frame statistics
    init_frm_.ref_keyfrm_ = init_keyfrm;
    curr_frm.ref_keyfrm_ = curr_keyfrm;
    map_db_->update_frame_statistics(init_frm_, false);
    map_db_->update_frame_statistics(curr_frm, false);

    // assign 2D-3D associations
    std::vector<std::shared_ptr<data::Landmark>> lms;
    for (unsigned int init_idx = 0; init_idx < init_matches_.size(); init_idx++) {
        const auto curr_idx = init_matches_.at(init_idx);
        if (curr_idx < 0) {
            continue;
        }

        // construct a Landmark
        auto lm = std::make_shared<data::Landmark>(map_db_->next_landmark_id_++, init_triangulated_pts.at(init_idx), curr_keyfrm);

        // set the assocications to the new keyframes
        lm->connect_to_keyframe(init_keyfrm, init_idx);
        lm->connect_to_keyframe(curr_keyfrm, curr_idx);

        // update the descriptor
        lm->compute_descriptor();
        // update the geometry
        lm->update_mean_normal_and_obs_scale_variance();

        // set the 2D-3D assocications to the current Frame
        curr_frm.add_landmark(lm, curr_idx);

        // add the Landmark to the map DB
        map_db_->add_landmark(lm);
        lms.push_back(lm);
    }

    // Pure monocular initialization has no metric reference, so the map scale
    // is indefinite and normalized to a median-depth of 1.0 below.
    const bool indefinite_scale = true;

    // global bundle adjustment
    const auto global_bundle_adjuster = optimize::GlobalBundleAdjuster(num_ba_iters_, true, verbose_);
    std::vector<std::shared_ptr<data::Keyframe>> keyfrms{init_keyfrm, curr_keyfrm};
    global_bundle_adjuster.optimize_for_initialization(keyfrms, lms, gain_threshold_);

    if (indefinite_scale) {
        // scale the map so that the median of depths is 1.0 (perspective only)
        const float median_scale = init_keyfrm->compute_median_depth(true);
        const auto inv_median_scale = 1.0 / median_scale;
        if (curr_keyfrm->get_num_tracked_landmarks(1) < min_num_triangulated_pts_ && median_scale < 0) {
            spdlog::info("seems to be wrong initialization, resetting");
            state_ = InitializerState::Wrong;
            return false;
        }
        scale_map(init_keyfrm, curr_keyfrm, inv_median_scale * scaling_factor_);
    }

    // update the current Frame pose
    curr_frm.set_pose_cw(curr_keyfrm->get_pose_cw());

    spdlog::info("new map created with {} points: Frame {} - Frame {}", map_db_->get_num_landmarks(), init_frm_.id_, curr_frm.id_);
    state_ = InitializerState::Succeeded;
    return true;
}

void Initializer::scale_map(const std::shared_ptr<data::Keyframe>& init_keyfrm, const std::shared_ptr<data::Keyframe>& curr_keyfrm, const double scale) {
    // scaling keyframes
    Mat44_t cam_pose_cw = curr_keyfrm->get_pose_cw();
    cam_pose_cw.block<3, 1>(0, 3) *= scale;
    curr_keyfrm->set_pose_cw(cam_pose_cw);

    // scaling landmarks
    const auto landmarks = init_keyfrm->get_landmarks();
    for (const auto& lm : landmarks) {
        if (!lm) {
            continue;
        }
        lm->set_pos_in_world(lm->get_pos_in_world() * scale);
        lm->update_mean_normal_and_obs_scale_variance();
    }
}

bool Initializer::try_initialize_for_stereo(data::Frame& curr_frm) {
    assert(state_ == InitializerState::Initializing);
    // count the number of valid depths
    unsigned int num_valid_depths = std::count_if(curr_frm.frm_obs_.depths_.begin(), curr_frm.frm_obs_.depths_.end(),
                                                  [](const float depth) {
                                                      return 0 < depth;
                                                  });
    return min_num_triangulated_pts_ <= num_valid_depths;
}

bool Initializer::create_map_for_stereo(data::Frame& curr_frm) {
    assert(state_ == InitializerState::Initializing);

    // create an initial Keyframe
    curr_frm.set_pose_cw(Mat44_t::Identity());
    auto curr_keyfrm = data::Keyframe::make_keyframe(map_db_->next_keyframe_id_++, curr_frm);
    curr_keyfrm->graph_node_->set_spanning_root(curr_keyfrm);
    map_db_->add_spanning_root(curr_keyfrm);

    // add to the map DB
    map_db_->add_keyframe(curr_keyfrm);

    // update the Frame statistics
    curr_frm.ref_keyfrm_ = curr_keyfrm;
    map_db_->update_frame_statistics(curr_frm, false);

    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
        // add a new Landmark if tht corresponding depth is valid
        const auto z = curr_frm.frm_obs_.depths_.at(idx);
        if (z <= 0) {
            continue;
        }

        // build a Landmark
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = std::make_shared<data::Landmark>(map_db_->next_landmark_id_++, pos_w, curr_keyfrm);

        // set the associations to the new Keyframe
        lm->connect_to_keyframe(curr_keyfrm, idx);

        // update the descriptor
        lm->compute_descriptor();
        // update the geometry
        lm->update_mean_normal_and_obs_scale_variance();

        // set the 2D-3D associations to the current Frame
        curr_frm.add_landmark(lm, idx);

        // add the Landmark to the map DB
        map_db_->add_landmark(lm);
    }

    spdlog::info("new map created with {} points: Frame {}", map_db_->get_num_landmarks(), curr_frm.id_);
    state_ = InitializerState::Succeeded;
    return true;
}

} // namespace module
}} // namespace vo // namespace uavloc
