#pragma once

#include "uavloc/new_vo/camera/base.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/keyframe.h"

#include <memory>

namespace uavloc {
namespace vo {

class MappingModule;

namespace data {
class MapDatabase;
} // namespace data

namespace module {

class KeyframeInserter {
public:
    explicit KeyframeInserter(const double max_interval = 1.0,
                               const double min_interval = 0.1,
                               const double max_distance = -1.0,
                               const double min_distance = -1.0,
                               const double lms_ratio_thr_almost_all_lms_are_tracked = 0.9,
                               const double lms_ratio_thr_view_changed = 0.8,
                               const unsigned int enough_lms_thr = 100,
                               const bool wait_for_local_bundle_adjustment = false);

    explicit KeyframeInserter(const YAML::Node& yaml_node);

    virtual ~KeyframeInserter() = default;

    void set_mapping_module(MappingModule* mapper);

    void reset();

    /**
     * Check the new Keyframe is needed or not
     */
    bool new_keyframe_is_needed(data::MapDatabase* map_db,
                                const data::Frame& curr_frm,
                                const unsigned int num_tracked_lms,
                                const unsigned int num_reliable_lms,
                                const data::Keyframe& ref_keyfrm,
                                const unsigned int min_num_obs_thr) const;

    /**
     * Insert the new Keyframe derived from the current Frame
     */
    void insert_new_keyframe(data::MapDatabase* map_db, data::Frame& curr_frm);

private:
    std::shared_ptr<data::Keyframe> create_new_keyframe(data::MapDatabase* map_db, data::Frame& curr_frm);

    //! mapping module
    MappingModule* mapper_ = nullptr;

    //! max interval to insert Keyframe
    const double max_interval_ = 1.0;
    const double min_interval_ = 0.1;
    const double max_distance_ = -1.0;
    const double min_distance_ = -1.0;

    //! Ratio-threshold of "the number of 3D points observed in the current Frame" / "that of 3D points observed in the last Keyframe"
    const double lms_ratio_thr_almost_all_lms_are_tracked_ = 0.9;
    const double lms_ratio_thr_view_changed_ = 0.8;

    const unsigned int enough_lms_thr_ = 100;
    const bool wait_for_local_bundle_adjustment_ = false;
};

} // namespace module
}} // namespace vo // namespace uavloc

