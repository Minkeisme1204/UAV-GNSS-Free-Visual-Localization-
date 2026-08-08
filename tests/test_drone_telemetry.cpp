// test_drone_telemetry — gated acceptance test for the configurable drone-log
// telemetry parser (sensor::DroneTelemetryCsvReader).
//
// The DroneTelemetry node (csv_path + has_header + the ordered column mapping)
// is taken straight from the mission config, so this test carries no path and no
// inline YAML: whatever the pipeline reads is what is exercised here.
//
// Gates: the file parses into >0 records, byFrameId() resolves the first record,
// every mapped field is finite, the derived off-nadir angle is inside [0, 90],
// the TelemetryData adapter preserves altitude/speed, and records stay ordered
// by frame_id. Headless-safe; soft-skips (returns 0) when the gitignored CSV is
// absent.
//
// Usage (from build/):
//   ./tests/test_drone_telemetry [config.yaml] [csv_path_override]

#include "uavloc/sensor/drone_telemetry_csv_reader.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef UAVLOC_MISSION_CONFIG_PATH
#define UAVLOC_MISSION_CONFIG_PATH "config/uavloc_yenbai800m_newvo.yaml"   // fallback; CMake injects the real path
#endif

namespace {

const std::string DEFAULT_CONFIG_PATH = UAVLOC_MISSION_CONFIG_PATH;

// Number of leading records printed for eyeballing the column mapping.
constexpr size_t PREVIEW_RECORDS = 5;

// Physical bound on the derived camera off-nadir angle |90 - gimbal_tilt|.
constexpr double OFF_NADIR_MIN_DEG = 0.0;
constexpr double OFF_NADIR_MAX_DEG = 90.0;

}  // namespace

int main(int argc, char** argv) {
    using namespace uavloc;

    spdlog::set_level(spdlog::level::info);

    const std::string config_path = (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    // ── Take the DroneTelemetry node from the mission config ─────────────────
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }

    const YAML::Node node = yaml["VideoReader"]["DroneTelemetry"];
    if (!node) {
        spdlog::error("config '{}' has no VideoReader.DroneTelemetry node", config_path);
        return 1;
    }

    auto cfg = sensor::DroneTelemetryConfig::fromYaml(node);
    if (argc > 2) cfg.csv_path = argv[2];   // manual-use override of just the CSV

    // ── Load (soft-skip when the gitignored dataset is absent) ───────────────
    sensor::DroneTelemetryCsvReader reader(cfg);
    if (!reader.load()) {
        spdlog::warn("test_drone_telemetry: cannot load '{}' — SKIPPED", cfg.csv_path);
        return 0;
    }

    int rc = 0;
    auto check = [&](bool ok, const std::string& what) {
        if (!ok) { spdlog::error("test_drone_telemetry FAIL: {}", what); rc = 1; }
    };

    spdlog::info("Loaded {} drone-log records from '{}'", reader.size(), cfg.csv_path);
    check(reader.size() > 0, "no records parsed");
    if (reader.size() == 0) return rc;

    const auto& recs = reader.records();
    const size_t n = std::min(PREVIEW_RECORDS, recs.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& r = recs[i];
        spdlog::info("rec[{}] frame_id={} alt(AGL)={:.2f}m tilt={:.3f}deg "
                     "off_nadir={:.3f}deg yaw={:.3f} lat={:.6f} lon={:.6f} speed={:.2f}",
                     i, r.frame_id, r.sensor_altitude_m, r.gimbal_tilt_deg,
                     r.offNadirDeg(), r.yaw_deg, r.latitude_deg, r.longitude_deg,
                     r.ground_speed_mps);
    }

    // ── frame_id lookup ──────────────────────────────────────────────────────
    const uint64_t probe = recs.front().frame_id;
    const auto hit = reader.byFrameId(probe);
    check(hit.valid, "byFrameId(front) returned an invalid record");
    check(hit.frame_id == probe, "byFrameId(front) returned a different frame_id");

    // ── Every mapped field is finite ─────────────────────────────────────────
    bool all_finite = true;
    for (const auto& r : recs) {
        if (!std::isfinite(r.latitude_deg) || !std::isfinite(r.longitude_deg) ||
            !std::isfinite(r.sensor_altitude_m) || !std::isfinite(r.yaw_deg) ||
            !std::isfinite(r.gimbal_tilt_deg)) {
            all_finite = false;
            break;
        }
    }
    check(all_finite, "a record has a non-finite lat/lon/altitude/yaw/tilt");

    // ── Derived off-nadir angle is finite and physically bounded ─────────────
    bool off_nadir_ok = true;
    for (const auto& r : recs) {
        const double a = r.offNadirDeg();
        if (!std::isfinite(a) || a < OFF_NADIR_MIN_DEG || a > OFF_NADIR_MAX_DEG) {
            off_nadir_ok = false;
            spdlog::error("frame_id={} off_nadir={} out of [{}, {}]",
                          r.frame_id, a, OFF_NADIR_MIN_DEG, OFF_NADIR_MAX_DEG);
            break;
        }
    }
    check(off_nadir_ok, "offNadirDeg() non-finite or outside [0, 90]");

    // ── TelemetryData adapter round-trip ─────────────────────────────────────
    const auto td = sensor::DroneTelemetryCsvReader::toTelemetryData(hit);
    spdlog::info("byFrameId({}) -> TelemetryData: heading={:.3f} altitude_m={:.2f} "
                 "speed_mps={:.2f} valid={}",
                 probe, td.heading_deg, td.altitude_m, td.speed_mps, td.valid);
    check(td.valid, "toTelemetryData() produced an invalid TelemetryData");
    check(td.altitude_m == hit.sensor_altitude_m, "adapter altitude_m != source altitude");
    check(td.speed_mps == hit.ground_speed_mps, "adapter speed_mps != source ground speed");

    // ── Records stay ordered by frame_id ─────────────────────────────────────
    bool ordered = true;
    for (size_t i = 1; i < recs.size(); ++i) {
        if (recs[i].frame_id < recs[i - 1].frame_id) { ordered = false; break; }
    }
    check(ordered, "records are not ordered by non-decreasing frame_id");

    if (rc == 0) spdlog::info("test_drone_telemetry: PASS");
    return rc;
}
