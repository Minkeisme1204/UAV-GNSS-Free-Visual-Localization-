#pragma once

#include "uavloc/common/type.h"

#include <map>
#include <mutex>
#include <atomic>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <nlohmann/json_fwd.hpp>

// sqlite3 forward declaration (persistence signatures kept, body cut — the
// <sqlite3.h> include was dropped to avoid the sqlite3 dependency).
// TODO(port): serialization cut — see .docs/designs/new_vo_port_notes.md
typedef struct sqlite3 sqlite3;
typedef struct sqlite3_stmt sqlite3_stmt;

namespace uavloc {
namespace vo {
namespace data {

class Frame;

class Keyframe;

class MapDatabase;

class Landmark : public std::enable_shared_from_this<Landmark> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Data structure for sorting keyframes by ID for consistent results in local map cleaning/BA
    using observations_t = std::map<std::weak_ptr<Keyframe>, unsigned int, id_less<std::weak_ptr<Keyframe>>>;

    //! constructor
    Landmark(unsigned int id, const Vec3_t& pos_w, const std::shared_ptr<Keyframe>& ref_keyfrm);

    //! constructor for map loading with computing parameters which can be recomputed
    Landmark(const unsigned int id, const unsigned int first_keyfrm_id,
             const Vec3_t& pos_w, const std::shared_ptr<Keyframe>& ref_keyfrm,
             const unsigned int num_visible, const unsigned int num_found);

    virtual ~Landmark();

    // Factory method for create Landmark
    static std::shared_ptr<Landmark> from_stmt(sqlite3_stmt* stmt,
                                               std::unordered_map<unsigned int, std::shared_ptr<uavloc::vo::data::Keyframe>>& keyframes,
                                               unsigned int next_landmark_id,
                                               unsigned int next_keyframe_id);

    /**
     * Save this Landmark information to db
     */
    static std::vector<std::pair<std::string, std::string>> columns() {
        return std::vector<std::pair<std::string, std::string>>{
            {"first_keyfrm", "INTEGER"},
            {"pos_w", "BLOB"},
            {"ref_keyfrm", "INTEGER"},
            {"n_vis", "INTEGER"},
            {"n_fnd", "INTEGER"}};
    };
    bool bind_to_stmt(sqlite3* db, sqlite3_stmt* stmt) const;

    //! set world coordinates of this Landmark
    void set_pos_in_world(const Vec3_t& pos_w);
    //! get world coordinates of this Landmark
    Vec3_t get_pos_in_world() const;

    //! get mean normalized vector of Keyframe->lm vectors, for keyframes such that observe the 3D point.
    Vec3_t get_obs_mean_normal() const;
    //! get reference Keyframe, a Keyframe at the creation of a given 3D point
    std::shared_ptr<Keyframe> get_ref_keyframe() const;

    //! add observation
    void add_observation(const std::shared_ptr<Keyframe>& keyfrm, unsigned int idx);
    //! erase observation
    void erase_observation(MapDatabase* map_db, const std::shared_ptr<Keyframe>& keyfrm);

    //! get observations (Keyframe and keypoint idx)
    observations_t get_observations() const;
    //! get number of observations
    unsigned int num_observations() const;
    //! whether this Landmark is observed from more than zero keyframes
    bool has_observation() const;

    //! get index of associated keypoint in the specified Keyframe
    int get_index_in_keyframe(const std::shared_ptr<Keyframe>& keyfrm) const;
    //! whether this Landmark is observed in the specified Keyframe
    bool is_observed_in_keyframe(const std::shared_ptr<Keyframe>& keyfrm) const;

    //! check the distance between Landmark and camera is in ORB scale variance
    inline bool is_inside_in_orb_scale(const float cam_to_lm_dist, const float margin_far, const float margin_near) const {
        const float max_dist = margin_far * get_max_valid_distance();
        const float min_dist = margin_near * get_min_valid_distance();
        return (min_dist <= cam_to_lm_dist && cam_to_lm_dist <= max_dist);
    }

    //! true if the Landmark has representative descriptor
    bool has_representative_descriptor() const;

    //! get representative descriptor
    cv::Mat get_descriptor() const;

    //! compute representative descriptor
    void compute_descriptor();

    //! update observation mean normal and ORB scale variance
    void update_mean_normal_and_obs_scale_variance();

    //! true if the Landmark has valid prediction parameters
    bool has_valid_prediction_parameters() const;
    //! get max valid distance between Landmark and camera
    float get_min_valid_distance() const;
    //! get min valid distance between Landmark and camera
    float get_max_valid_distance() const;

    //! predict scale level assuming this Landmark is observed in the specified Frame/Keyframe
    unsigned int predict_scale_level(const float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) const;

    //! erase this Landmark from database
    void prepare_for_erasing(MapDatabase* map_db);
    //! whether this Landmark will be erased shortly or not
    bool will_be_erased();

    //! Make an interconnection by Landmark::add_observation and Keyframe::add_landmark
    void connect_to_keyframe(const std::shared_ptr<Keyframe>& keyfrm, unsigned int idx);

    //! replace this with specified Landmark
    void replace(std::shared_ptr<Landmark> lm, data::MapDatabase* map_db);

    void increase_num_observable(unsigned int num_observable = 1);
    void increase_num_observed(unsigned int num_observed = 1);
    unsigned int get_num_observed() const;
    unsigned int get_num_observable() const;
    float get_observed_ratio() const;

    //! encode Landmark information as JSON
    nlohmann::json to_json() const;

public:
    unsigned int id_;
    unsigned int first_keyfrm_id_ = 0;
    unsigned int num_observations_ = 0;

protected:
    void compute_mean_normal(const observations_t& observations,
                             const Vec3_t& pos_w,
                             Vec3_t& mean_normal) const;
    void compute_orb_scale_variance(const observations_t& observations,
                                    const std::shared_ptr<Keyframe>& ref_keyfrm,
                                    const Vec3_t& pos_w,
                                    float& max_valid_dist,
                                    float& min_valid_dist) const;

private:
    //! world coordinates of this Landmark
    Vec3_t pos_w_;

    //! observations (Keyframe and keypoint index)
    observations_t observations_;

    //! true if the Landmark has representative descriptor
    std::atomic<bool> has_representative_descriptor_{false};
    //! representative descriptor
    cv::Mat descriptor_;

    //! reference Keyframe
    std::weak_ptr<Keyframe> ref_keyfrm_;

    // track counter
    unsigned int num_observable_ = 1;
    unsigned int num_observed_ = 1;

    //! this Landmark will be erased shortly or not
    std::atomic<bool> will_be_erased_{false};

    // parameters for prediction
    //! true if the Landmark has valid prediction parameters
    std::atomic<bool> has_valid_prediction_parameters_{false};
    //! Normalized average vector (unit vector) of Keyframe->lm, for keyframes such that observe the 3D point.
    Vec3_t mean_normal_ = Vec3_t::Zero();
    //! max valid distance between Landmark and camera
    float min_valid_dist_ = 0;
    //! min valid distance between Landmark and camera
    float max_valid_dist_ = 0;

    mutable std::mutex mtx_position_;
    mutable std::mutex mtx_observations_;
};

} // namespace data
}} // namespace vo // namespace uavloc
