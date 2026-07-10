// test_drone_telemetry — exercises the configurable drone-log telemetry parser.
//
// Loads the real YenBai 800 m 57-column CSV using an inline YAML config that
// selects only a subset of columns, then prints a few records to verify the
// column mapping, the sensorAltitude→AGL choice, and the gimbal-tilt→off-nadir
// derivation. Headless-safe (no GUI).
//
// Usage (from build/):
//   ./tests/test_drone_telemetry [csv_path]
//   default csv: data/YenBai800m/cut_2025-07-30_800m-1x.csv

#include "uavloc/sensor/drone_telemetry_csv_reader.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <string>

using namespace uavloc;

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);

    const std::string csv_path =
        (argc > 1) ? argv[1] : "data/YenBai800m/cut_2025-07-30_800m-1x.csv";

    // Build a config in code (mirrors what fromYaml would parse). Only a subset
    // of the 57 columns is selected; altitude comes from sensorAltitude.
    // New compact syntax: each entry is a single-key map  field_logic: column.
    // A string value locates the column by header name; an integer value locates
    // it by 0-based index (here latitude_deg is pinned to index 30 as a spot-check
    // of the index branch — column 30 is sensorLatitude in the YenBai log).
    const std::string yaml_text = R"(
DroneTelemetry:
  csv_path: )" + csv_path + R"(
  has_header: true
  columns:
    - frame_id: imageId
    - roll_deg: roll
    - pitch_deg: pitch
    - yaw_deg: yaw
    - gimbal_tilt_deg: tilt
    - latitude_deg: sensorLatitude
    - longitude_deg: sensorLongitude
    - altitude_m: sensorAltitude
    - ground_speed_mps: groundSpeed
    - distance_to_target_m: distanceToTarget
)";

    YAML::Node root = YAML::Load(yaml_text);
    auto cfg = sensor::DroneTelemetryConfig::fromYaml(root["DroneTelemetry"]);

    sensor::DroneTelemetryCsvReader reader(cfg);
    if (!reader.load()) {
        spdlog::error("test_drone_telemetry: failed to load '{}'", csv_path);
        return 1;
    }

    spdlog::info("Loaded {} drone-log records", reader.size());

    const auto& recs = reader.records();
    const size_t n = std::min<size_t>(5, recs.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& r = recs[i];
        spdlog::info("rec[{}] frame_id={} alt(AGL)={:.2f}m tilt={:.3f}deg "
                     "off_nadir={:.3f}deg yaw={:.3f} lat={:.6f} lon={:.6f} "
                     "speed={:.2f} dist2tgt={:.2f}",
                     i, r.frame_id, r.sensor_altitude_m, r.gimbal_tilt_deg,
                     r.offNadirDeg(), r.yaw_deg, r.latitude_deg, r.longitude_deg,
                     r.ground_speed_mps, r.distance_to_target_m);
    }

    // Spot-check the frame_id lookup + adapter to sensor::TelemetryData.
    const uint64_t probe = recs.front().frame_id;
    auto hit = reader.byFrameId(probe);
    auto td  = sensor::DroneTelemetryCsvReader::toTelemetryData(hit);
    spdlog::info("byFrameId({}) -> TelemetryData: heading={:.3f} altitude_m={:.2f} "
                 "speed_mps={:.2f} valid={}",
                 probe, td.heading_deg, td.altitude_m, td.speed_mps, td.valid);

    // Metre-per-pixel scaling helper demo: ground distance along optical ray.
    if (hit.valid) {
        const double off_nadir_rad = hit.offNadirDeg() * M_PI / 180.0;
        const double d = hit.sensor_altitude_m / std::cos(off_nadir_rad);
        spdlog::info("slant range to ground d = altitude/cos(off_nadir) = {:.2f} m", d);
    }

    return 0;
}
