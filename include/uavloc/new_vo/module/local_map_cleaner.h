#pragma once

#include <list>
#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Keyframe;
class Landmark;
class MapDatabase;
} // namespace data

namespace module {

class LocalMapCleaner {
public:
    /**
     * Constructor
     */
    explicit LocalMapCleaner(const YAML::Node& yaml_node, data::MapDatabase* map_db);

    /**
     * Destructor
     */
    ~LocalMapCleaner() = default;

    /**
     * Add fresh Landmark to check their redundancy
     */
    void add_fresh_landmark(std::shared_ptr<data::Landmark>& lm) {
        fresh_landmarks_.push_back(lm);
    }

    /**
     * Reset the buffer
     */
    void reset();

    /**
     * Remove redundant landmarks
     */
    unsigned int remove_invalid_landmarks(const unsigned int cur_keyfrm_id);

    /**
     * Remove redundant keyframes
     */
    unsigned int remove_redundant_keyframes(const std::shared_ptr<data::Keyframe>& cur_keyfrm) const;

    /**
     * Count the valid and the redundant observations in the specified Keyframe
     */
    void count_redundant_observations(const std::shared_ptr<data::Keyframe>& keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const;

private:
    //! map database
    data::MapDatabase* map_db_ = nullptr;

    //!
    double redundant_obs_ratio_thr_;

    //!
    double observed_ratio_thr_ = 0.3;

    //!
    unsigned int num_reliable_keyfrms_ = 2;

    //! Top n covisibilities to search (0 means disabled)
    unsigned int top_n_covisibilities_to_search_;

    //! fresh landmarks to check their redundancy
    std::list<std::shared_ptr<data::Landmark>> fresh_landmarks_;
};

} // namespace module
}} // namespace vo // namespace uavloc

