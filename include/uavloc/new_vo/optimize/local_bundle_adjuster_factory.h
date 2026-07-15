#pragma once

#include "uavloc/new_vo/optimize/local_bundle_adjuster_g2o.h"
// TODO(port): gtsam backend not ported — g2o only

#include <memory>

namespace uavloc {
namespace vo {

namespace optimize {

class LocalBundleAdjusterFactory {
public:
    static std::unique_ptr<LocalBundleAdjuster> create(const YAML::Node& yaml_node) {
        const auto& backend = yaml_node["backend"].as<std::string>("g2o");
        if (backend == "g2o") {
            return std::unique_ptr<LocalBundleAdjuster>(new LocalBundleAdjusterG2o(yaml_node));
        }
        else if (backend == "gtsam") {
            // TODO(port): gtsam backend not ported — g2o only
            throw std::runtime_error("gtsam is not enabled");
        }
        else {
            throw std::runtime_error("Invalid backend");
        }
    }
};

} // namespace optimize
}} // namespace vo // namespace uavloc
