#pragma once

#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Frame;
class Keyframe;
class Landmark;
} // namespace data

namespace module {

class LocalMapUpdater {
public:
    using keyframe_to_num_shared_lms_t = nondeterministic::unordered_map<std::shared_ptr<data::Keyframe>, unsigned int>;

    //! Constructor
    explicit LocalMapUpdater(const unsigned int max_num_local_keyfrms);

    //! Destructor
    ~LocalMapUpdater() = default;

    //! Get the local keyframes
    std::vector<std::shared_ptr<data::Keyframe>> get_local_keyframes() const;

    //! Get the local landmarks
    std::vector<std::shared_ptr<data::Landmark>> get_local_landmarks() const;

    //! Get the nearest covisibility
    std::shared_ptr<data::Keyframe> get_nearest_covisibility() const;

    //! Acquire the new local map
    bool acquire_local_map(const std::vector<std::shared_ptr<data::Landmark>>& frm_lms);
    bool acquire_local_map(const std::vector<std::shared_ptr<data::Landmark>>& frm_lms,
                           unsigned int keyframe_id_threshold,
                           unsigned int& num_temporal_keyfrms);

private:
    //! Find the local keyframes
    bool find_local_keyframes(const std::vector<std::shared_ptr<data::Landmark>>& frm_lms,
                              unsigned int keyframe_id_threshold,
                              unsigned int& num_temporal_keyfrms);

    //! Count the number of shared landmarks between the current Frame and each of the neighbor keyframes
    auto count_num_shared_lms(
        const std::vector<std::shared_ptr<data::Landmark>>& frm_lms,
        unsigned int keyframe_id_threshold) const
        -> std::vector<std::pair<unsigned int, std::shared_ptr<data::Keyframe>>>;

    //! Find the first-order local keyframes
    auto find_first_local_keyframes(
        const std::vector<std::pair<unsigned int, std::shared_ptr<data::Keyframe>>>& keyfrm_weights,
        const unsigned int keyframe_id_threshold,
        std::unordered_set<unsigned int>& already_found_keyfrm_ids,
        unsigned int& num_temporal_keyfrms)
        -> std::vector<std::shared_ptr<data::Keyframe>>;

    //! Find the second-order local keyframes
    auto find_second_local_keyframes(const std::vector<std::shared_ptr<data::Keyframe>>& first_local_keyframes,
                                     unsigned int keyframe_id_threshold,
                                     std::unordered_set<unsigned int>& already_found_keyfrm_ids,
                                     unsigned int& num_temporal_keyfrms) const
        -> std::vector<std::shared_ptr<data::Keyframe>>;

    //! Find the local landmarks
    bool find_local_landmarks(const std::vector<std::shared_ptr<data::Landmark>>& frm_lms);

    // maximum number of the local keyframes
    const unsigned int max_num_local_keyfrms_;

    // found local keyframes
    std::vector<std::shared_ptr<data::Keyframe>> local_keyfrms_;
    // found local landmarks
    std::vector<std::shared_ptr<data::Landmark>> local_lms_;
    // the nearst Keyframe in covisibility graph, which will be found in find_first_local_keyframes()
    std::shared_ptr<data::Keyframe> nearest_covisibility_;
};

} // namespace module
}} // namespace vo // namespace uavloc

