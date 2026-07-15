#pragma once

#include "uavloc/new_vo/optimize/pose_optimizer_g2o.h"
// TODO(port): gtsam backend not ported — g2o only
#include "uavloc/common/type.h"
#include "uavloc/util/yaml.h"

#include <memory>

namespace uavloc {
namespace vo {
namespace optimize {

class PoseOptimizerFactory {
public:
    static std::unique_ptr<PoseOptimizer> create(const YAML::Node& yaml_node) {
        const auto& backend = yaml_node["backend"].as<std::string>("g2o");
        if (backend == "g2o") {
            YAML::Node g2o_node = util::yaml_optional_ref(yaml_node, "g2o");
            return std::unique_ptr<PoseOptimizer>(new PoseOptimizerG2o(
                g2o_node["num_trials_robust"].as<unsigned int>(2),
                g2o_node["num_trials"].as<unsigned int>(2),
                g2o_node["num_each_iter"].as<unsigned int>(10)));
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
