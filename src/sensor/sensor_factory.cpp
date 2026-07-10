#include "uavloc/sensor/sensor_factory.h"
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace uavloc::sensor {

// ─── SensorConfig ─────────────────────────────────────────────────────────────

SensorConfig SensorConfig::fromYaml(const YAML::Node& node) {
    const auto& n = node["Sensor"];
    if (!n) throw std::runtime_error("YAML missing 'Sensor' section");

    SensorConfig cfg;
    cfg.source_type = SensorFactory::parseSourceType(
        n["source_type"].as<std::string>("unknown"));

    switch (cfg.source_type) {
        case SensorSourceType::LIVE_CAMERA:
            cfg.live_camera = CameraStreamerConfig::fromYaml(node);
            break;
        case SensorSourceType::VIDEO_FILE:
            cfg.video_file = VideoReaderConfig::fromYaml(node);
            break;
        default:
            throw std::runtime_error("SensorConfig: unknown source_type '"
                + n["source_type"].as<std::string>() + "'");
    }
    return cfg;
}

// ─── SensorFactory ────────────────────────────────────────────────────────────

SensorSourceType SensorFactory::parseSourceType(const std::string& str) {
    if (str == "live_camera") return SensorSourceType::LIVE_CAMERA;
    if (str == "video_file")  return SensorSourceType::VIDEO_FILE;
    return SensorSourceType::UNKNOWN;
}

std::unique_ptr<DataInterface> SensorFactory::create(const SensorConfig& config) {
    switch (config.source_type) {
        case SensorSourceType::LIVE_CAMERA:
            spdlog::info("SensorFactory: creating CameraStreamer");
            return std::make_unique<CameraStreamer>(config.live_camera);

        case SensorSourceType::VIDEO_FILE:
            spdlog::info("SensorFactory: creating VideoReader ({})",
                config.video_file.video_path);
            return std::make_unique<VideoReader>(config.video_file);

        default:
            throw std::runtime_error("SensorFactory::create: source_type is UNKNOWN");
    }
}

}  // namespace uavloc::sensor
