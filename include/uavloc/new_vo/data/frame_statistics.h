#pragma once

// data::FrameStatistics — per-frame odometry-evaluation bookkeeping.
//
// data::MapDatabase holds a FrameStatistics frm_stats_ BY VALUE and calls
// update_frame_statistics / replace_reference_keyframe / clear from its inline
// members, so this is a minimal by-value-copyable class. The bodies are inline
// no-ops: frame statistics is odometry-evaluation / publish only and is not
// needed by the pure-VO front-end (fill in when evaluation is wired).

#include "uavloc/common/type.h"

#include <memory>

namespace uavloc {
namespace vo {
namespace data {

class Frame;
class Keyframe;

class FrameStatistics {
public:
    FrameStatistics() = default;
    virtual ~FrameStatistics() = default;

    //! Update Frame statistics (referenced by MapDatabase::update_frame_statistics)
    void update_frame_statistics(const data::Frame& frm, const bool is_lost) {
        (void)frm;
        (void)is_lost;
    }

    //! Replace a reference Keyframe (referenced by MapDatabase::replace_reference_keyframe)
    void replace_reference_keyframe(const std::shared_ptr<data::Keyframe>& old_keyfrm,
                                    const std::shared_ptr<data::Keyframe>& new_keyfrm) {
        (void)old_keyfrm;
        (void)new_keyfrm;
    }

    //! Clear Frame statistics (referenced by MapDatabase::clear)
    void clear() {}
};

} // namespace data
}} // namespace vo // namespace uavloc
