#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/data/map_database.h"
// TODO(port): util/ not cloned — see .docs/designs/new_vo_port_notes.md
#include "uavloc/util/converter.h"

#include <spdlog/spdlog.h>

// TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency.
// The following persistence entry points are handled specially:
//   * JSON methods — from_json / register_keyframe / register_landmark /
//     register_graph / register_association / to_json — have their DEFINITIONS
//     omitted (defining a function with an incomplete nlohmann::json return or
//     by-value parameter type is ill-formed; only json_fwd.hpp is available).
//   * sqlite methods — from_db / to_db / load_*_from_db / save_*_to_db /
//     load_association_from_stmt / bind_association_to_stmt — keep their
//     declarations, bodies replaced with a no-op returning false.
// The <nlohmann/json.hpp>, <sqlite3.h> and util/sqlite3.h includes were dropped.
// See .docs/designs/new_vo_port_notes.md

namespace uavloc {
namespace vo {
namespace data {

std::mutex MapDatabase::mtx_database_;

MapDatabase::MapDatabase(unsigned int min_num_shared_lms)
    : fixed_keyframe_id_threshold_(0), min_num_shared_lms_(min_num_shared_lms) {
    spdlog::debug("CONSTRUCT: data::MapDatabase");
}

MapDatabase::~MapDatabase() {
    clear();
    spdlog::debug("DESTRUCT: data::MapDatabase");
}

void MapDatabase::set_fixed_keyframe_id_threshold() {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    fixed_keyframe_id_threshold_ = next_keyframe_id_;
}

unsigned int MapDatabase::get_fixed_keyframe_id_threshold() {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    return fixed_keyframe_id_threshold_;
}

void MapDatabase::add_keyframe(const std::shared_ptr<Keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    keyframes_[keyfrm->id_] = keyfrm;
    last_inserted_keyfrm_ = keyfrm;
}

void MapDatabase::erase_keyframe(const std::shared_ptr<Keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    keyframes_.erase(keyfrm->id_);
}

std::shared_ptr<Keyframe> MapDatabase::get_keyframe(unsigned int id) const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    if (!keyframes_.count(id)) {
        return nullptr;
    }
    return keyframes_.at(id);
}

void MapDatabase::add_landmark(std::shared_ptr<Landmark>& lm) {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    landmarks_[lm->id_] = lm;
}

void MapDatabase::erase_landmark(unsigned int id) {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    landmarks_.erase(id);
}

std::shared_ptr<Landmark> MapDatabase::get_landmark(unsigned int id) const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    if (!landmarks_.count(id)) {
        return nullptr;
    }
    return landmarks_.at(id);
}

void MapDatabase::add_spanning_root(std::shared_ptr<Keyframe>& Keyframe) {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    spanning_roots_.push_back(Keyframe);
}

std::vector<std::shared_ptr<Keyframe>> MapDatabase::get_spanning_roots() {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    return spanning_roots_;
}

void MapDatabase::set_local_landmarks(const std::vector<std::shared_ptr<Landmark>>& local_lms) {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    local_landmarks_ = local_lms;
}

std::vector<std::shared_ptr<Landmark>> MapDatabase::get_local_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    return local_landmarks_;
}

std::vector<std::shared_ptr<Keyframe>> MapDatabase::get_all_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    std::vector<std::shared_ptr<Keyframe>> keyframes;
    keyframes.reserve(keyframes_.size());
    for (const auto& id_keyframe : keyframes_) {
        keyframes.push_back(id_keyframe.second);
    }
    return keyframes;
}

std::vector<std::shared_ptr<Keyframe>> MapDatabase::get_close_keyframes_2d(const Mat44_t& pose_cw,
                                                                            const Vec3_t& normal_vector,
                                                                            const double distance_threshold,
                                                                            const double angle_threshold) const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);

    // Close (within given thresholds) keyframes
    std::vector<std::shared_ptr<Keyframe>> filtered_keyframes;

    const double cos_angle_threshold = std::cos(angle_threshold);
    Mat44_t pose_wc = util::Converter::inverse_pose(pose_cw);

    // Calculate angles and distances between given pose and all keyframes
    Mat33_t M = pose_wc.block<3, 3>(0, 0);
    Vec3_t Mt = pose_wc.block<3, 1>(0, 3);
    for (const auto& id_keyframe : keyframes_) {
        Mat33_t N = id_keyframe.second->get_pose_wc().block<3, 3>(0, 0);
        Vec3_t Nt = id_keyframe.second->get_pose_wc().block<3, 1>(0, 3);
        // Angle between two cameras related to given pose and selected Keyframe
        const double cos_angle = ((M * N.transpose()).trace() - 1) / 2;
        // Distance between given pose and selected Keyframe
        const double dist = ((Nt - Nt.dot(normal_vector) * normal_vector)
                             - (Mt - Mt.dot(normal_vector) * normal_vector))
                                .norm();
        if (dist < distance_threshold && cos_angle > cos_angle_threshold) {
            filtered_keyframes.push_back(id_keyframe.second);
        }
    }

    return filtered_keyframes;
}

std::vector<std::shared_ptr<Keyframe>> MapDatabase::get_close_keyframes(const Mat44_t& pose_cw,
                                                                         const double distance_threshold,
                                                                         const double angle_threshold) const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);

    // Close (within given thresholds) keyframes
    std::vector<std::shared_ptr<Keyframe>> filtered_keyframes;

    const double cos_angle_threshold = std::cos(angle_threshold);
    Mat44_t pose_wc = util::Converter::inverse_pose(pose_cw);

    // Calculate angles and distances between given pose and all keyframes
    Mat33_t M = pose_wc.block<3, 3>(0, 0);
    Vec3_t Mt = pose_wc.block<3, 1>(0, 3);
    for (const auto& id_keyframe : keyframes_) {
        Mat33_t N = id_keyframe.second->get_pose_wc().block<3, 3>(0, 0);
        Vec3_t Nt = id_keyframe.second->get_pose_wc().block<3, 1>(0, 3);
        // Angle between two cameras related to given pose and selected Keyframe
        const double cos_angle = ((M * N.transpose()).trace() - 1) / 2;
        // Distance between given pose and selected Keyframe
        const double dist = (Nt - Mt).norm();
        if (dist < distance_threshold && cos_angle > cos_angle_threshold) {
            filtered_keyframes.push_back(id_keyframe.second);
        }
    }

    return filtered_keyframes;
}

unsigned int MapDatabase::get_num_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    return keyframes_.size();
}

std::vector<std::shared_ptr<Landmark>> MapDatabase::get_all_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    std::vector<std::shared_ptr<Landmark>> landmarks;
    landmarks.reserve(landmarks_.size());
    for (const auto& id_landmark : landmarks_) {
        landmarks.push_back(id_landmark.second);
    }
    return landmarks;
}

std::shared_ptr<Keyframe> MapDatabase::get_last_inserted_keyframe() const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    return last_inserted_keyfrm_;
}

unsigned int MapDatabase::get_num_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_map_access_);
    return landmarks_.size();
}

unsigned int MapDatabase::get_min_num_shared_lms() const {
    return min_num_shared_lms_;
}

void MapDatabase::clear() {
    std::lock_guard<std::mutex> lock(mtx_map_access_);

    landmarks_.clear();
    keyframes_.clear();
    last_inserted_keyfrm_ = nullptr;
    local_landmarks_.clear();
    spanning_roots_.clear();

    frm_stats_.clear();

    next_keyframe_id_ = 0;
    next_landmark_id_ = 0;
    fixed_keyframe_id_threshold_ = 0;

    spdlog::info("clear map database");
}

// -------------------------------------------------------------------------
// JSON (de)serialization definitions omitted (see banner above):
//   from_json / register_keyframe / register_landmark / register_graph /
//   register_association / to_json
// -------------------------------------------------------------------------

bool MapDatabase::from_db(sqlite3* db,
                           CameraDatabase* cam_db,
                           OrbParamsDatabase* orb_params_db) {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)cam_db;
    (void)orb_params_db;
    return false;
}

bool MapDatabase::load_keyframes_from_db(sqlite3* db,
                                          const std::string& table_name,
                                          CameraDatabase* cam_db,
                                          OrbParamsDatabase* orb_params_db) {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)table_name;
    (void)cam_db;
    (void)orb_params_db;
    return false;
}

bool MapDatabase::load_landmarks_from_db(sqlite3* db, const std::string& table_name) {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)table_name;
    return false;
}

void MapDatabase::load_association_from_stmt(sqlite3_stmt* stmt) {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)stmt;
}

bool MapDatabase::load_associations_from_db(sqlite3* db, const std::string& table_name) {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)table_name;
    return false;
}

bool MapDatabase::to_db(sqlite3* db) const {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    return false;
}

bool MapDatabase::save_keyframes_to_db(sqlite3* db, const std::string& table_name) const {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)table_name;
    return false;
}

bool MapDatabase::save_landmarks_to_db(sqlite3* db, const std::string& table_name) const {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)table_name;
    return false;
}

bool MapDatabase::bind_association_to_stmt(sqlite3_stmt* stmt,
                                            const std::shared_ptr<Keyframe>& keyfrm) const {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)stmt;
    (void)keyfrm;
    return false;
}

bool MapDatabase::save_associations_to_db(sqlite3* db, const std::string& table_name) const {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)table_name;
    return false;
}

} // namespace data
}} // namespace vo // namespace uavloc
