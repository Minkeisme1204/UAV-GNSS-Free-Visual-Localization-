#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/data/map_database.h"
// NOTE(port): match/Base.h is a FUTURE clone (later phase). The path below is
// the correct future location; the file does not exist yet — that is expected.
// compute_descriptor() below uses match::compute_descriptor_distance_32 and
// match::MAX_HAMMING_DIST from it. See .docs/designs/new_vo_port_notes.md
#include "uavloc/new_vo/match/base.h"

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace data {

Landmark::Landmark(unsigned int id, const Vec3_t& pos_w, const std::shared_ptr<Keyframe>& ref_keyfrm)
    : id_(id), first_keyfrm_id_(ref_keyfrm->id_), pos_w_(pos_w),
      ref_keyfrm_(ref_keyfrm) {}

Landmark::Landmark(const unsigned int id, const unsigned int first_keyfrm_id,
                   const Vec3_t& pos_w, const std::shared_ptr<Keyframe>& ref_keyfrm,
                   const unsigned int num_visible, const unsigned int num_found)
    : id_(id), first_keyfrm_id_(first_keyfrm_id), pos_w_(pos_w), ref_keyfrm_(ref_keyfrm),
      num_observable_(num_visible), num_observed_(num_found) {}

Landmark::~Landmark() {
    SPDLOG_TRACE("Landmark::~Landmark: {}", id_);
}

std::shared_ptr<Landmark> Landmark::from_stmt(sqlite3_stmt* stmt,
                                              std::unordered_map<unsigned int, std::shared_ptr<uavloc::vo::data::Keyframe>>& keyframes,
                                              unsigned int next_landmark_id,
                                              unsigned int next_keyframe_id) {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)stmt;
    (void)keyframes;
    (void)next_landmark_id;
    (void)next_keyframe_id;
    return nullptr;
}

bool Landmark::bind_to_stmt(sqlite3* db, sqlite3_stmt* stmt) const {
    // TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency
    (void)db;
    (void)stmt;
    return false;
}

void Landmark::set_pos_in_world(const Vec3_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    SPDLOG_TRACE("Landmark::set_pos_in_world {}", id_);
    pos_w_ = pos_w;
    has_valid_prediction_parameters_ = false;
}

Vec3_t Landmark::get_pos_in_world() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return pos_w_;
}

Vec3_t Landmark::get_obs_mean_normal() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    assert(has_valid_prediction_parameters_);
    return mean_normal_;
}

std::shared_ptr<Keyframe> Landmark::get_ref_keyframe() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return ref_keyfrm_.lock();
}

void Landmark::add_observation(const std::shared_ptr<Keyframe>& keyfrm, unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    SPDLOG_TRACE("Landmark::add_observation {} {} {}", id_, keyfrm->id_, idx);
    assert(!static_cast<bool>(observations_.count(keyfrm)));
    observations_[keyfrm] = idx;
    assert(static_cast<bool>(observations_.count(keyfrm)));

    has_valid_prediction_parameters_ = false;
    has_representative_descriptor_ = false;

    if (!keyfrm->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm->frm_obs_.stereo_x_right_.at(idx)) {
        num_observations_ += 2;
    }
    else {
        num_observations_ += 1;
    }
}

void Landmark::erase_observation(MapDatabase* map_db, const std::shared_ptr<Keyframe>& keyfrm) {
    bool discard = false;
    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        SPDLOG_TRACE("Landmark::erase_observation {} {}", id_, keyfrm->id_);

        assert(observations_.count(keyfrm));
        int idx = observations_.at(keyfrm);
        if (!keyfrm->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm->frm_obs_.stereo_x_right_.at(idx)) {
            num_observations_ -= 2;
        }
        else {
            num_observations_ -= 1;
        }

        observations_.erase(keyfrm);

        has_valid_prediction_parameters_ = false;
        has_representative_descriptor_ = false;

        if (observations_.empty()) {
            discard = true;
        }
        else {
            // Re-anchor the reference keyframe if it was this one, or if it has
            // been freed concurrently (async local mapping) leaving an expired
            // weak_ptr.
            const auto ref_keyfrm = ref_keyfrm_.lock();
            if (!ref_keyfrm || ref_keyfrm->id_ == keyfrm->id_) {
                ref_keyfrm_ = observations_.begin()->first.lock();
            }
        }
        assert(discard || observations_.count(ref_keyfrm_));
    }

    if (discard) {
        prepare_for_erasing(map_db);
    }
}

Landmark::observations_t Landmark::get_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return observations_;
}

unsigned int Landmark::num_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observations_;
}

bool Landmark::has_observation() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return 0 < num_observations_;
}

int Landmark::get_index_in_keyframe(const std::shared_ptr<Keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    if (observations_.count(keyfrm)) {
        return observations_.at(keyfrm);
    }
    else {
        return -1;
    }
}

bool Landmark::is_observed_in_keyframe(const std::shared_ptr<Keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<bool>(observations_.count(keyfrm));
}

bool Landmark::has_representative_descriptor() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return has_representative_descriptor_;
}

cv::Mat Landmark::get_descriptor() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    assert(has_representative_descriptor_);
    return descriptor_.clone();
}

void Landmark::compute_descriptor() {
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        assert(!has_representative_descriptor_);
        assert(!will_be_erased_);
        assert(!observations_.empty());
        observations = observations_;
    }
    SPDLOG_TRACE("Landmark::compute_descriptor {}", id_);

    // Append features of corresponding points
    std::vector<cv::Mat> descriptors;
    descriptors.reserve(observations.size());
    for (const auto& observation : observations) {
        auto keyfrm = observation.first.lock();
        const auto idx = observation.second;

        // The weak_ptr may have expired if the observing keyframe was erased and
        // freed concurrently (async local mapping): skip it instead of
        // dereferencing a null keyframe.
        if (keyfrm && !keyfrm->will_be_erased()) {
            descriptors.push_back(keyfrm->frm_obs_.descriptors_.row(idx));
        }
    }

    // Every observing keyframe vanished concurrently: nothing to summarise.
    if (descriptors.empty()) {
        return;
    }

    // Get median of Hamming distance
    // Calculate all the Hamming distances between every pair of the features
    const auto num_descs = descriptors.size();
    std::vector<std::vector<unsigned int>> hamm_dists(num_descs, std::vector<unsigned int>(num_descs));
    for (unsigned int i = 0; i < num_descs; ++i) {
        hamm_dists.at(i).at(i) = 0;
        for (unsigned int j = i + 1; j < num_descs; ++j) {
            const auto dist = match::compute_descriptor_distance_32(descriptors.at(i), descriptors.at(j));
            hamm_dists.at(i).at(j) = dist;
            hamm_dists.at(j).at(i) = dist;
        }
    }

    // Get the nearest value to median
    unsigned int best_median_dist = match::MAX_HAMMING_DIST;
    unsigned int best_idx = 0;
    for (unsigned idx = 0; idx < num_descs; ++idx) {
        std::vector<unsigned int> partial_hamm_dists(hamm_dists.at(idx).begin(), hamm_dists.at(idx).begin() + num_descs);
        std::sort(partial_hamm_dists.begin(), partial_hamm_dists.end());
        const auto median_dist = partial_hamm_dists.at(static_cast<unsigned int>(0.5 * (num_descs - 1)));

        if (median_dist < best_median_dist) {
            best_median_dist = median_dist;
            best_idx = idx;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        descriptor_ = descriptors.at(best_idx).clone();
        has_representative_descriptor_ = true;
    }
}

void Landmark::compute_mean_normal(const observations_t& observations,
                                   const Vec3_t& pos_w,
                                   Vec3_t& mean_normal) const {
    mean_normal = Vec3_t::Zero();
    for (const auto& observation : observations) {
        auto keyfrm = observation.first.lock();
        // Skip keyframes freed concurrently (async local mapping).
        if (!keyfrm) {
            continue;
        }
        const Vec3_t normal = pos_w - keyfrm->get_trans_wc();
        mean_normal = mean_normal + normal.normalized();
    }
    mean_normal = mean_normal.normalized();
}

void Landmark::compute_orb_scale_variance(const observations_t& observations,
                                          const std::shared_ptr<Keyframe>& ref_keyfrm,
                                          const Vec3_t& pos_w,
                                          float& max_valid_dist,
                                          float& min_valid_dist) const {
    const Vec3_t vec_ref_keyfrm_to_lm = pos_w - ref_keyfrm->get_trans_wc();
    const auto dist_ref_keyfrm_to_lm = vec_ref_keyfrm_to_lm.norm();
    assert(!observations.empty());
    const auto idx = observations.at(ref_keyfrm);
    const auto scale_level = ref_keyfrm->frm_obs_.undist_keypts_.at(idx).octave;
    const auto scale_factor = ref_keyfrm->orb_params_->scale_factors_.at(scale_level);
    const auto num_scale_levels = ref_keyfrm->orb_params_->num_levels_;

    max_valid_dist = dist_ref_keyfrm_to_lm * scale_factor;
    min_valid_dist = max_valid_dist * ref_keyfrm->orb_params_->inv_scale_factors_.at(num_scale_levels - 1);
}

void Landmark::update_mean_normal_and_obs_scale_variance() {
    SPDLOG_TRACE("Landmark::update_mean_normal_and_obs_scale_variance {}", id_);
    observations_t observations;
    std::shared_ptr<Keyframe> ref_keyfrm = nullptr;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        assert(!has_valid_prediction_parameters_);
        assert(!observations_.empty());
        assert(observations_.count(ref_keyfrm_));
        observations = observations_;
        ref_keyfrm = ref_keyfrm_.lock();
    }
    Vec3_t pos_w;
    {
        std::lock_guard<std::mutex> lock2(mtx_position_);
        pos_w = pos_w_;
    }

    Vec3_t mean_normal;
    compute_mean_normal(observations, pos_w, mean_normal);

    float max_valid_dist;
    float min_valid_dist;
    compute_orb_scale_variance(observations, ref_keyfrm, pos_w, max_valid_dist, min_valid_dist);

    {
        std::lock_guard<std::mutex> lock3(mtx_position_);
        max_valid_dist_ = max_valid_dist;
        min_valid_dist_ = min_valid_dist;
        mean_normal_ = mean_normal;
        has_valid_prediction_parameters_ = true;
    }
}

bool Landmark::has_valid_prediction_parameters() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return has_valid_prediction_parameters_;
}

float Landmark::get_min_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    assert(has_valid_prediction_parameters_);
    return min_valid_dist_;
}

float Landmark::get_max_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    assert(has_valid_prediction_parameters_);
    return max_valid_dist_;
}

unsigned int Landmark::predict_scale_level(const float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) const {
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mtx_position_);
        ratio = max_valid_dist_ / cam_to_lm_dist;
    }

    const auto pred_scale_level = static_cast<int>(std::ceil(std::log(ratio) / log_scale_factor));
    if (pred_scale_level < 0) {
        return 0;
    }
    else if (num_scale_levels <= static_cast<unsigned int>(pred_scale_level)) {
        return num_scale_levels - 1;
    }
    else {
        return static_cast<unsigned int>(pred_scale_level);
    }
}

void Landmark::prepare_for_erasing(MapDatabase* map_db) {
    SPDLOG_TRACE("Landmark::prepare_for_erasing {}", id_);
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        observations = observations_;
        observations_.clear();
        will_be_erased_ = true;
    }

    for (const auto& keyfrm_and_idx : observations) {
        // The observing keyframe may have been erased and freed concurrently
        // (async local mapping): skip the expired weak_ptr.
        if (auto keyfrm = keyfrm_and_idx.first.lock()) {
            keyfrm->erase_landmark_with_index(keyfrm_and_idx.second);
        }
    }

    map_db->erase_landmark(id_);
}

bool Landmark::will_be_erased() {
    return will_be_erased_;
}

void Landmark::connect_to_keyframe(const std::shared_ptr<Keyframe>& keyfrm, unsigned int idx) {
    assert(!observations_.count(keyfrm));
    keyfrm->add_landmark(shared_from_this(), idx);
    add_observation(keyfrm, idx);
}

void Landmark::replace(std::shared_ptr<Landmark> lm, data::MapDatabase* map_db) {
    SPDLOG_TRACE("Landmark::replace {} {}", id_, lm->id_);
    if (lm->id_ == id_) {
        return;
    }

    // 1. Erase this
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        observations = observations_;
    }

    prepare_for_erasing(map_db);

    // 2. Merge lm with this
    unsigned int num_observable, num_observed;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        num_observable = num_observable_;
        num_observed = num_observed_;
    }

    for (const auto& keyfrm_and_idx : observations) {
        const auto keyfrm = keyfrm_and_idx.first.lock();
        // Skip observing keyframes freed concurrently (async local mapping).
        if (keyfrm && !lm->is_observed_in_keyframe(keyfrm)) {
            lm->connect_to_keyframe(keyfrm, keyfrm_and_idx.second);
        }
    }

    lm->increase_num_observed(num_observed);
    lm->increase_num_observable(num_observable);
}

void Landmark::increase_num_observable(unsigned int num_observable) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observable_ += num_observable;
}

void Landmark::increase_num_observed(unsigned int num_observed) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observed_ += num_observed;
}

float Landmark::get_observed_ratio() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<float>(num_observed_) / num_observable_;
}

unsigned int Landmark::get_num_observed() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observed_;
}

unsigned int Landmark::get_num_observable() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observable_;
}

// TODO(port): serialization cut to avoid nlohmann/json + sqlite3 dependency.
// Landmark::to_json() (declared in Landmark.h, returns nlohmann::json) has its
// DEFINITION omitted here — defining a function whose return type is the
// incomplete nlohmann::json (only json_fwd.hpp is included) is ill-formed.
// Wire the definition to nlohmann/json when persistence is re-enabled.

} // namespace data
}} // namespace vo // namespace uavloc
