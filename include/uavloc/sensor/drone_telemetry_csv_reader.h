#pragma once

#include "uavloc/sensor/telemetry_data.h"
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace uavloc::sensor {

// ─────────────────────────────────────────────────────────────────────────────
// Drone-log telemetry (the "57-column" YenBai flight log format).
//
// This is a SEPARATE, parallel parser to TelemetryCsvReader (which handles the
// simple 10-column uavloc telemetry format). It does NOT replace it.
//
// The defining feature: the parser does NOT hard-code which of the 57 columns to
// read. Instead, the caller declares — in YAML — exactly which columns to extract,
// in which order, and which logical field each column maps to. Columns are located
// either by header NAME (preferred) or by 0-based INDEX. Anything not listed in the
// config is simply ignored.
// ─────────────────────────────────────────────────────────────────────────────

// Logical fields a CSV column can be mapped onto. This enum is the contract
// between the YAML config and the storage struct (DroneTelemetryRecord). Add a
// new member here + a matching field in DroneTelemetryRecord + a case in
// assignField() to support a new column — nothing else changes.
enum class DroneTelemetryField {
    UNKNOWN,
    FRAME_ID,           // imageId
    TIMESTAMP_MSEC,     // optional, if a timestamp column ever exists
    ROLL_DEG,           // roll
    PITCH_DEG,          // pitch
    YAW_DEG,            // yaw  → heading
    GIMBAL_PAN_DEG,     // pan
    GIMBAL_TILT_DEG,    // tilt (measured from the horizontal; ~90° ⇒ nadir)
    AMSL_M,             // amsl (above mean sea level)
    SENSOR_ALTITUDE_M,  // sensorAltitude — the real AGL height (NOT the 'agl' column,
                        //                  which is all-zero garbage in the real data)
    GROUND_SPEED_MPS,   // groundSpeed
    LATITUDE_DEG,       // sensorLatitude
    LONGITUDE_DEG,      // sensorLongitude
    DISTANCE_TO_TARGET_M  // distanceToTarget
};

// Maps a YAML field name string (e.g. "altitude_m") to a DroneTelemetryField.
// Returns DroneTelemetryField::UNKNOWN if the string is not recognised.
DroneTelemetryField droneFieldFromString(const std::string& field);

// One configured column: a logical field + how to locate its source column.
// Locate by header name (preferred) or, if name is empty, by 0-based index.
struct DroneColumnSpec {
    DroneTelemetryField field = DroneTelemetryField::UNKNOWN;
    std::string         name;          // header name, e.g. "sensorAltitude"
    int                 index = -1;    // 0-based column index; used only if name empty

    bool locateByName() const { return !name.empty(); }
};

// One parsed telemetry sample from the drone log. Only the fields whose columns
// were declared in the config are populated; the rest keep their defaults.
struct DroneTelemetryRecord {
    uint64_t frame_id       = 0;     // imageId
    double   timestamp_msec = 0.0;

    double roll_deg  = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg   = 0.0;

    double gimbal_pan_deg  = 0.0;
    double gimbal_tilt_deg = 0.0;    // from horizontal; ~90° ⇒ camera looks straight down

    double amsl_m             = 0.0;
    double sensor_altitude_m  = 0.0; // real AGL height of the sensor
    double ground_speed_mps   = 0.0;

    double latitude_deg  = 0.0;
    double longitude_deg = 0.0;

    double distance_to_target_m = 0.0;

    bool valid = false;

    // Camera off-nadir angle in degrees, derived from gimbal tilt.
    // Gimbal tilt is measured from the horizontal plane, so a tilt of 90°
    // corresponds to a nadir (straight-down) view; off_nadir = |90 - tilt|.
    // Used by the metre-conversion step as d = altitude_m / cos(off_nadir).
    double offNadirDeg() const;
};

// Configuration for DroneTelemetryCsvReader. Loaded from YAML under the
// "DroneTelemetry" key. All fields have defaults; missing keys never throw.
struct DroneTelemetryConfig {
    std::string csv_path;
    bool        has_header = true;

    // Ordered list of columns to extract. If left empty, a sensible default
    // mapping for the YenBai 800 m flight log is used (see defaultColumns()).
    std::vector<DroneColumnSpec> columns;

    static DroneTelemetryConfig fromYaml(const YAML::Node& node);

    // Default YenBai 800 m column mapping (by header name), used when the YAML
    // 'columns' list is empty.
    static std::vector<DroneColumnSpec> defaultColumns();
};

// Reads a drone-log telemetry CSV into memory using a configured column mapping.
// Provides O(1) lookup by frame_id (imageId) and ordered access.
class DroneTelemetryCsvReader {
public:
    explicit DroneTelemetryCsvReader(DroneTelemetryConfig config);

    // Parses the CSV file. Returns false on IO error or if no rows were read.
    bool load();

    // Returns the record matching frame_id (imageId) exactly; falls back to an
    // empty (invalid) record if not found.
    DroneTelemetryRecord byFrameId(uint64_t frame_id) const;

    // Ordered access to all parsed records (file order).
    const std::vector<DroneTelemetryRecord>& records() const { return records_; }

    bool   isLoaded() const { return loaded_; }
    size_t size()     const { return records_.size(); }

    // Adapter: convert a drone-log record into the pipeline's sensor::TelemetryData.
    //   sensorAltitude → altitude_m (AGL), yaw → heading_deg, etc.
    static TelemetryData toTelemetryData(const DroneTelemetryRecord& rec);

private:
    DroneTelemetryConfig                 config_;
    std::vector<DroneTelemetryRecord>    records_;
    std::unordered_map<uint64_t, size_t> frame_id_idx_;
    bool                                 loaded_ = false;

    static std::vector<std::string> splitLine(const std::string& line);

    // Resolves each configured column to a concrete 0-based index, given the
    // parsed header row. Columns that cannot be resolved are dropped (with a warn).
    std::vector<std::pair<DroneTelemetryField, int>> resolveColumns(
        const std::vector<std::string>& headers) const;

    // Assigns one raw cell string into the matching field of the record.
    static void assignField(DroneTelemetryRecord& rec,
                            DroneTelemetryField field,
                            const std::string&  value);
};

}  // namespace uavloc::sensor
