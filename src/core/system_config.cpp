#include "uavloc/core/system_config.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <string>

namespace uavloc {
namespace core {

namespace {

//! Parse the "input_policy" string (case-insensitive). An unrecognised value
//! keeps the default and warns rather than throwing — same tolerant contract
//! as the .as<T>(default) reads below (a typo in an optional knob must not
//! take the system down; a missing "Camera:" section is a different matter,
//! see fromYaml()).
InputPolicy parse_input_policy(const std::string& name, InputPolicy fallback) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower == "block")        return InputPolicy::BLOCK;
    if (lower == "drop_oldest")  return InputPolicy::DROP_OLDEST;
    spdlog::warn("SystemConfig: unknown input_policy '{}' — "
                 "expected block|drop_oldest; keeping the default", name);
    return fallback;
}

} // namespace

SystemConfig SystemConfig::fromYaml(const YAML::Node& root) {
    SystemConfig cfg;

    // Delegate the algorithm blocks — never re-parse them here, so packaging
    // the pipeline cannot change how an existing mission config is read.
    //
    // This call PROPAGATES: sensor::CameraIntrinsics::fromYaml() throws when
    // the document has no "Camera:" section, and that is the behaviour we
    // want. Swallowing it would hand the pipeline a degenerate K (fx = fy = 0)
    // and turn a config mistake into silently wrong poses at run time. Fail at
    // load time, loudly, exactly like every other VOConfig::fromYaml() caller.
    cfg.vo     = vo::VOConfig::fromYaml(root);
    cfg.fusion = fusion::FusionConfig::fromYaml(root);
    // VOConfig already parsed root["Camera"]; reuse it so the two copies can
    // never disagree.
    cfg.camera = cfg.vo.camera_intrinsics;

    const YAML::Node node = root["System"] ? root["System"] : YAML::Node();

    cfg.async_input   = node["async_input"].as<bool>(cfg.async_input);
    cfg.input_policy  = parse_input_policy(
        node["input_policy"].as<std::string>(std::string("block")),
        cfg.input_policy);

    cfg.input_queue_capacity =
        node["input_queue_capacity"].as<unsigned int>(cfg.input_queue_capacity);
    if (cfg.input_queue_capacity == UNBOUNDED_INPUT_QUEUE_CAPACITY) {
        spdlog::warn("SystemConfig: input_queue_capacity = 0 means UNBOUNDED in "
                     "util::ThreadsafeQueue, which silently disables backpressure "
                     "(a BLOCK push would never wait and the queue would grow "
                     "without bound) — falling back to the default {}",
                     DEFAULT_INPUT_QUEUE_CAPACITY);
        cfg.input_queue_capacity = DEFAULT_INPUT_QUEUE_CAPACITY;
    }

    cfg.push_timeout_ms = node["push_timeout_ms"].as<unsigned int>(cfg.push_timeout_ms);
    if (cfg.push_timeout_ms == 0) {
        spdlog::warn("SystemConfig: push_timeout_ms = 0 turns input_policy BLOCK "
                     "into try-then-give-up (onFrame returns false instead of "
                     "waiting for room)");
    }

    cfg.enable_fusion   = node["enable_fusion"].as<bool>(cfg.enable_fusion);
    cfg.enable_geo      = node["enable_geo"].as<bool>(cfg.enable_geo);
    cfg.publish_images  = node["publish_images"].as<bool>(cfg.publish_images);
    cfg.stats_period_ms = node["stats_period_ms"].as<unsigned int>(cfg.stats_period_ms);

    cfg.extrapolator_buffer_span_sec =
        node["extrapolator_buffer_span_sec"].as<double>(cfg.extrapolator_buffer_span_sec);
    if (cfg.extrapolator_buffer_span_sec <= 0.0) {
        spdlog::warn("SystemConfig: extrapolator_buffer_span_sec = {} makes the "
                     "telemetry buffers UNBOUNDED — they will grow for the whole "
                     "flight", cfg.extrapolator_buffer_span_sec);
    }
    cfg.extrapolator_max_gap_sec =
        node["extrapolator_max_gap_sec"].as<double>(cfg.extrapolator_max_gap_sec);
    if (cfg.extrapolator_max_gap_sec < 0.0) {
        spdlog::warn("SystemConfig: negative extrapolator_max_gap_sec ({}) means "
                     "no limit, same as 0", cfg.extrapolator_max_gap_sec);
    }

    return cfg;
}

} // namespace core
} // namespace uavloc
