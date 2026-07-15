#ifndef SENSOR_FACTORY_H
#define SENSOR_FACTORY_H

#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "uavloc/sensor/data_interface.h"
#include "uavloc/sensor/camera_streamer.h"
#include "uavloc/sensor/video_reader.h"

namespace uavloc::sensor {

enum class SensorSourceType {
    LIVE_CAMERA,
    VIDEO_FILE,
    UNKNOWN
};

struct SensorConfig {
    SensorSourceType source_type = SensorSourceType::UNKNOWN;

    CameraStreamerConfig live_camera;  // populated when source_type == LIVE_CAMERA
    VideoReaderConfig    video_file;   // populated when source_type == VIDEO_FILE

    // Reads sensor.source_type, then delegates to CameraStreamerConfig::fromYaml
    // or VideoReaderConfig::fromYaml (to be added to video_reader.h)
    static SensorConfig fromYaml(const YAML::Node& node);
};

class SensorFactory {
public:
    // Constructs and returns the correct DataInterface for the given config
    static std::unique_ptr<DataInterface> create(const SensorConfig& config);

    // Maps "live_camera" / "video_file" strings to SensorSourceType
    static SensorSourceType parseSourceType(const std::string& str);
};

}  // namespace uavloc::sensor

#endif  // SENSOR_FACTORY_H
