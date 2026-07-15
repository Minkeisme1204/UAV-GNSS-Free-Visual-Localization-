#pragma once

#include <string>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

namespace uavloc {
namespace util {

inline YAML::Node yaml_optional_ref(const YAML::Node& ref_node, const std::string& key) {
    return ref_node[key] ? ref_node[key] : YAML::Node();
}

std::vector<std::vector<float>> get_rectangles(const YAML::Node& node);

} // namespace util
} // namespace uavloc
