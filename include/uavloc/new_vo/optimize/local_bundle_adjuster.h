#pragma once

#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Keyframe;
class MapDatabase;
} // namespace data

namespace optimize {

class LocalBundleAdjuster {
public:
    /**
     * Perform optimization
     * @param map_db
     * @param curr_keyfrm
     * @param force_stop_flag
     */
    virtual void optimize(data::MapDatabase* map_db, const std::shared_ptr<data::Keyframe>& curr_keyfrm, bool* const force_stop_flag) const = 0;
};

} // namespace optimize
}} // namespace vo // namespace uavloc
