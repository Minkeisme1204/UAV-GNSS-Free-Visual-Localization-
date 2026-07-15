#pragma once

#include "uavloc/new_vo/optimize/local_bundle_adjuster.h"

#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Keyframe;
class MapDatabase;
} // namespace data

namespace optimize {

class LocalBundleAdjusterG2o : public LocalBundleAdjuster {
public:
    /**
     * Constructor
     * @param yaml_node
     * @param num_first_iter
     * @param num_second_iter
     */
    explicit LocalBundleAdjusterG2o(const YAML::Node& yaml_node,
                                       const unsigned int num_first_iter = 5,
                                       const unsigned int num_second_iter = 10);

    /**
     * Destructor
     */
    virtual ~LocalBundleAdjusterG2o() = default;

    /**
     * Perform optimization
     * @param map_db
     * @param curr_keyfrm
     * @param force_stop_flag
     */
    void optimize(data::MapDatabase* map_db, const std::shared_ptr<data::Keyframe>& curr_keyfrm, bool* const force_stop_flag) const override;

private:
    //! number of iterations of first optimization
    const unsigned int num_first_iter_;
    //! number of iterations of second optimization
    const unsigned int num_second_iter_;
    //!
    const unsigned int use_additional_keyframes_for_monocular_ = false;
};

} // namespace optimize
}} // namespace vo // namespace uavloc
