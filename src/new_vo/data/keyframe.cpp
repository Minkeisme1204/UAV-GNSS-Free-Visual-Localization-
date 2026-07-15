#include "uavloc/new_vo/data/common.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/data/map_database.h"
// NOTE(port): reuses the already-ported ORB feature header (feature::OrbParams).
#include "uavloc/new_vo/feature/orb/orb_params.h"

#include <spdlog/spdlog.h>

// NOTE(port): the from_stmt / bind_to_stmt sqlite persistence bodies were cut
// along with their BLOB (de)serialization helpers. See
// .docs/designs/new_vo_port_notes.md

namespace uavloc {
namespace vo {
namespace data {

Keyframe::Keyframe(unsigned int id, const Frame& frm)
    : id_(id), timestamp_(frm.timestamp_),
      camera_(frm.camera_), orb_params_(frm.orb_params_),
      frm_obs_(frm.frm_obs_),
      landmarks_(frm.get_landmarks()) {
    // set pose parameters (pose_wc_, trans_wc_) using frm.pose_cw_
    set_pose_cw(frm.get_pose_cw());
}

Keyframe::Keyframe(const unsigned int id, const double timestamp,
                   const Mat44_t& pose_cw, camera::Base* camera,
                   const feature::OrbParams* orb_params, const FrameObservation& frm_obs)
    : id_(id),
      timestamp_(timestamp), camera_(camera),
      orb_params_(orb_params), frm_obs_(frm_obs),
      landmarks_(std::vector<std::shared_ptr<Landmark>>(frm_obs_.undist_keypts_.size(), nullptr)) {
    // set pose parameters (pose_wc_, trans_wc_) using pose_cw_
    set_pose_cw(pose_cw);

    // The following process needs to take place:
    //   should set the pointers of landmarks_ using add_landmark()
    //   should set connections using GraphNode->update_connections()
    //   should set spanning_parent_ using GraphNode->set_spanning_parent()
    //   should set spanning_children_ using GraphNode->add_spanning_child()
    //   should set loop_edges_ using GraphNode->add_loop_edge()
}

Keyframe::~Keyframe() {
    SPDLOG_TRACE("Keyframe::~Keyframe: {}", id_);
}

std::shared_ptr<Keyframe> Keyframe::make_keyframe(unsigned int id, const Frame& frm) {
    auto ptr = std::allocate_shared<Keyframe>(Eigen::aligned_allocator<Keyframe>(), id, frm);
    // covisibility graph node (connections is not assigned yet)
    ptr->graph_node_ = uavloc::make_unique<GraphNode>(ptr);
    return ptr;
}

std::shared_ptr<Keyframe> Keyframe::make_keyframe(
    const unsigned int id, const double timestamp,
    const Mat44_t& pose_cw, camera::Base* camera,
    const feature::OrbParams* orb_params, const FrameObservation& frm_obs) {
    auto ptr = std::allocate_shared<Keyframe>(
        Eigen::aligned_allocator<Keyframe>(),
        id, timestamp,
        pose_cw, camera, orb_params,
        frm_obs);
    // covisibility graph node (connections is not assigned yet)
    ptr->graph_node_ = uavloc::make_unique<GraphNode>(ptr);
    return ptr;
}

std::shared_ptr<Keyframe> Keyframe::from_stmt(sqlite3_stmt* stmt,
                                              CameraDatabase* cam_db,
                                              OrbParamsDatabase* orb_params_db,
                                              unsigned int next_keyframe_id) {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)stmt;
    (void)cam_db;
    (void)orb_params_db;
    (void)next_keyframe_id;
    return nullptr;
}

// TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency.
// Keyframe::to_json() (declared in Keyframe.h, returns nlohmann::json) has its
// DEFINITION omitted here — defining a function whose return type is the
// incomplete nlohmann::json (only json_fwd.hpp is included) is ill-formed.

bool Keyframe::bind_to_stmt(sqlite3* db, sqlite3_stmt* stmt) const {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)stmt;
    return false;
}

void Keyframe::set_pose_cw(const Mat44_t& pose_cw) {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    pose_cw_ = pose_cw;

    const Mat33_t rot_cw = pose_cw_.block<3, 3>(0, 0);
    const Vec3_t trans_cw = pose_cw_.block<3, 1>(0, 3);
    const Mat33_t rot_wc = rot_cw.transpose();
    trans_wc_ = -rot_wc * trans_cw;

    pose_wc_ = Mat44_t::Identity();
    pose_wc_.block<3, 3>(0, 0) = rot_wc;
    pose_wc_.block<3, 1>(0, 3) = trans_wc_;
}

Mat44_t Keyframe::get_pose_cw() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return pose_cw_;
}

Mat44_t Keyframe::get_pose_wc() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return pose_wc_;
}

Vec3_t Keyframe::get_trans_wc() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return trans_wc_;
}

Mat33_t Keyframe::get_rot_cw() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return pose_cw_.block<3, 3>(0, 0);
}

Vec3_t Keyframe::get_trans_cw() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return pose_cw_.block<3, 1>(0, 3);
}

void Keyframe::add_landmark(std::shared_ptr<Landmark> lm, const unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    landmarks_.at(idx) = lm;
}

void Keyframe::erase_landmark_with_index(const unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    landmarks_.at(idx) = nullptr;
}

void Keyframe::erase_landmark(const std::shared_ptr<Landmark>& lm) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    int idx = lm->get_index_in_keyframe(shared_from_this());
    if (0 <= idx) {
        landmarks_.at(static_cast<unsigned int>(idx)) = nullptr;
    }
}

void Keyframe::update_landmarks() {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    for (unsigned int idx = 0; idx < landmarks_.size(); ++idx) {
        auto lm = landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // update connection
        lm->add_observation(shared_from_this(), idx);
        // update geometry
        lm->update_mean_normal_and_obs_scale_variance();
        lm->compute_descriptor();
    }
}

std::vector<std::shared_ptr<Landmark>> Keyframe::get_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return landmarks_;
}

std::set<std::shared_ptr<Landmark>> Keyframe::get_valid_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    std::set<std::shared_ptr<Landmark>> valid_landmarks;

    for (const auto& lm : landmarks_) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        valid_landmarks.insert(lm);
    }

    return valid_landmarks;
}

unsigned int Keyframe::get_num_tracked_landmarks(const unsigned int min_num_obs_thr) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    unsigned int num_tracked_lms = 0;

    if (0 < min_num_obs_thr) {
        for (const auto& lm : landmarks_) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (min_num_obs_thr <= lm->num_observations()) {
                ++num_tracked_lms;
            }
        }
    }
    else {
        for (const auto& lm : landmarks_) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            ++num_tracked_lms;
        }
    }

    return num_tracked_lms;
}

std::shared_ptr<Landmark>& Keyframe::get_landmark(const unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return landmarks_.at(idx);
}

std::vector<unsigned int> Keyframe::get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin,
                                                          const int min_level, const int max_level) const {
    return data::get_keypoints_in_cell(camera_, frm_obs_, ref_x, ref_y, margin, min_level, max_level);
}

Vec3_t Keyframe::triangulate_stereo(const unsigned int idx) const {
    Mat44_t pose_wc;
    {
        std::lock_guard<std::mutex> lock(mtx_pose_);
        pose_wc = pose_wc_;
    }
    return data::triangulate_stereo(camera_, pose_wc.block<3, 3>(0, 0), pose_wc.block<3, 1>(0, 3), frm_obs_, idx);
}

float Keyframe::compute_median_depth(const bool abs) const {
    std::vector<std::shared_ptr<Landmark>> landmarks;
    Mat44_t pose_cw;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_pose_);
        landmarks = landmarks_;
        pose_cw = pose_cw_;
    }

    std::vector<float> depths;
    depths.reserve(frm_obs_.undist_keypts_.size());
    const Vec3_t rot_cw_z_row = pose_cw.block<1, 3>(2, 0);
    const float trans_cw_z = pose_cw(2, 3);

    for (const auto& lm : landmarks) {
        if (!lm) {
            continue;
        }
        const Vec3_t pos_w = lm->get_pos_in_world();
        const auto pos_c_z = rot_cw_z_row.dot(pos_w) + trans_cw_z;
        depths.push_back(abs ? std::abs(pos_c_z) : pos_c_z);
    }

    std::sort(depths.begin(), depths.end());

    return depths.at((depths.size() - 1) / 2);
}

float Keyframe::compute_median_distance() const {
    std::vector<std::shared_ptr<Landmark>> landmarks;
    Mat44_t pose_cw;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_pose_);
        landmarks = landmarks_;
        pose_cw = pose_cw_;
    }

    std::vector<float> distances;
    distances.reserve(frm_obs_.undist_keypts_.size());
    const Mat33_t rot_cw = pose_cw.block<3, 3>(0, 0);
    const Vec3_t trans_cw = pose_cw.block<3, 1>(0, 3);

    for (const auto& lm : landmarks) {
        if (!lm) {
            continue;
        }
        const Vec3_t pos_w = lm->get_pos_in_world();
        const Vec3_t pos_c = rot_cw * pos_w + trans_cw;
        float distance = pos_c.norm();
        distances.push_back(distance);
    }

    std::sort(distances.begin(), distances.end());

    return distances.at((distances.size() - 1) / 2);
}

bool Keyframe::depth_is_available() const {
    return camera_->setup_type_ != camera::SetupType::Monocular;
}

void Keyframe::set_not_to_be_erased() {
    cannot_be_erased_ = true;
}

void Keyframe::set_to_be_erased() {
    if (!graph_node_->has_loop_edge()) {
        cannot_be_erased_ = false;
    }
}

void Keyframe::prepare_for_erasing(MapDatabase* map_db) {
    if (graph_node_->is_spanning_root()) {
        spdlog::warn("cannot erase the root node: {}", id_);
        return;
    }

    // cannot erase if the frag is raised
    if (cannot_be_erased_) {
        return;
    }

    // 1. raise the flag which indicates it has been erased

    SPDLOG_TRACE("Keyframe::prepare_for_erasing {}", id_);
    will_be_erased_ = true;

    // 2. remove associations between keypoints and landmarks

    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        for (const auto& lm : landmarks_) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }
            lm->erase_observation(map_db, shared_from_this());
            if (!lm->will_be_erased()) {
                lm->compute_descriptor();
                lm->update_mean_normal_and_obs_scale_variance();
            }
        }
    }

    // 3. recover covisibility graph and spanning tree

    // remove covisibility information
    graph_node_->erase_all_connections();
    // recover spanning tree
    graph_node_->recover_spanning_connections();

    // 3. update Frame statistics

    map_db->replace_reference_keyframe(shared_from_this(), graph_node_->get_spanning_parent());

    // 4. remove myself from the databased

    map_db->erase_keyframe(shared_from_this());
}

bool Keyframe::will_be_erased() {
    return will_be_erased_;
}

} // namespace data
}} // namespace vo // namespace uavloc
