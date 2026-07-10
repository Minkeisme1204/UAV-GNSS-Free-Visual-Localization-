#include "uavloc/sensor/drone_telemetry_csv_reader.h"

#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

namespace uavloc::sensor {

namespace {

// String → DroneTelemetryField lookup. Keys are the logical field names used in
// the YAML 'field:' entries. Kept here (not in a header) so the mapping stays
// implementation-private and easy to extend.
const std::unordered_map<std::string, DroneTelemetryField>& fieldNameTable() {
    static const std::unordered_map<std::string, DroneTelemetryField> table = {
        {"frame_id",            DroneTelemetryField::FRAME_ID},
        {"timestamp_msec",      DroneTelemetryField::TIMESTAMP_MSEC},
        {"roll_deg",            DroneTelemetryField::ROLL_DEG},
        {"pitch_deg",           DroneTelemetryField::PITCH_DEG},
        {"yaw_deg",             DroneTelemetryField::YAW_DEG},
        {"gimbal_pan_deg",      DroneTelemetryField::GIMBAL_PAN_DEG},
        {"gimbal_tilt_deg",     DroneTelemetryField::GIMBAL_TILT_DEG},
        {"amsl_m",              DroneTelemetryField::AMSL_M},
        {"sensor_altitude_m",   DroneTelemetryField::SENSOR_ALTITUDE_M},
        {"altitude_m",          DroneTelemetryField::SENSOR_ALTITUDE_M},  // alias
        {"ground_speed_mps",    DroneTelemetryField::GROUND_SPEED_MPS},
        {"latitude_deg",        DroneTelemetryField::LATITUDE_DEG},
        {"longitude_deg",       DroneTelemetryField::LONGITUDE_DEG},
        {"distance_to_target_m", DroneTelemetryField::DISTANCE_TO_TARGET_M},
    };
    return table;
}

double toDouble(const std::string& s, double def = 0.0) {
    try { return std::stod(s); } catch (...) { return def; }
}

uint64_t toU64(const std::string& s) {
    // imageId values are integral but may arrive as "1.0"; round defensively.
    try { return static_cast<uint64_t>(std::llround(std::stod(s))); }
    catch (...) { return 0; }
}

}  // namespace

// ─── DroneTelemetryField helpers ──────────────────────────────────────────────

DroneTelemetryField droneFieldFromString(const std::string& field) {
    const auto& t = fieldNameTable();
    auto it = t.find(field);
    return it == t.end() ? DroneTelemetryField::UNKNOWN : it->second;
}

// ─── DroneTelemetryRecord ─────────────────────────────────────────────────────

double DroneTelemetryRecord::offNadirDeg() const {
    // Gimbal tilt is measured from the horizontal plane (~90° ⇒ straight down).
    return std::abs(90.0 - gimbal_tilt_deg);
}

// ─── DroneTelemetryConfig ─────────────────────────────────────────────────────

std::vector<DroneColumnSpec> DroneTelemetryConfig::defaultColumns() {
    // Sensible default mapping for the YenBai 800 m flight log (located by name).
    // Note: altitude uses 'sensorAltitude', NOT the all-zero 'agl' column.
    auto col = [](DroneTelemetryField f, const char* n) {
        DroneColumnSpec s;
        s.field = f;
        s.name  = n;
        return s;
    };
    return {
        col(DroneTelemetryField::FRAME_ID,           "imageId"),
        col(DroneTelemetryField::ROLL_DEG,           "roll"),
        col(DroneTelemetryField::PITCH_DEG,          "pitch"),
        col(DroneTelemetryField::YAW_DEG,            "yaw"),
        col(DroneTelemetryField::GIMBAL_PAN_DEG,     "pan"),
        col(DroneTelemetryField::GIMBAL_TILT_DEG,    "tilt"),
        col(DroneTelemetryField::AMSL_M,             "amsl"),
        col(DroneTelemetryField::SENSOR_ALTITUDE_M,  "sensorAltitude"),
        col(DroneTelemetryField::GROUND_SPEED_MPS,   "groundSpeed"),
        col(DroneTelemetryField::LATITUDE_DEG,       "sensorLatitude"),
        col(DroneTelemetryField::LONGITUDE_DEG,      "sensorLongitude"),
        col(DroneTelemetryField::DISTANCE_TO_TARGET_M, "distanceToTarget"),
    };
}

DroneTelemetryConfig DroneTelemetryConfig::fromYaml(const YAML::Node& node) {
    DroneTelemetryConfig cfg;
    if (!node) {
        spdlog::warn("DroneTelemetryConfig: empty YAML node, using defaults");
        cfg.columns = defaultColumns();
        return cfg;
    }

    cfg.csv_path   = node["csv_path"].as<std::string>(cfg.csv_path);
    cfg.has_header = node["has_header"].as<bool>(cfg.has_header);

    const YAML::Node& cols = node["columns"];
    if (cols && cols.IsSequence() && cols.size() > 0) {
        for (const auto& entry : cols) {
            if (!entry || !entry.IsMap()) {
                spdlog::warn("DroneTelemetryConfig: column entry is not a map, skipping");
                continue;
            }

            // ── Backward-compatible legacy form: { field:, name:, index: } ──
            if (entry["field"]) {
                DroneColumnSpec spec;
                const std::string field_str = entry["field"].as<std::string>("");
                spec.field = droneFieldFromString(field_str);
                if (spec.field == DroneTelemetryField::UNKNOWN) {
                    spdlog::warn("DroneTelemetryConfig: unknown field '{}' in columns, skipping",
                                 field_str);
                    continue;
                }
                spec.name  = entry["name"].as<std::string>("");
                spec.index = entry["index"].as<int>(-1);
                if (!spec.locateByName() && spec.index < 0) {
                    spdlog::warn("DroneTelemetryConfig: field '{}' has neither 'name' nor valid "
                                 "'index', skipping", field_str);
                    continue;
                }
                cfg.columns.push_back(spec);
                continue;
            }

            // ── New compact form: single-key map  field_logic: column_locator ──
            if (entry.size() != 1) {
                spdlog::warn("DroneTelemetryConfig: column entry is not a single-key map, "
                             "skipping");
                continue;
            }

            const auto it = entry.begin();
            const std::string field_str = it->first.as<std::string>("");
            const YAML::Node  locator   = it->second;

            DroneColumnSpec spec;
            spec.field = droneFieldFromString(field_str);
            if (spec.field == DroneTelemetryField::UNKNOWN) {
                spdlog::warn("DroneTelemetryConfig: unknown field '{}' in columns, skipping",
                             field_str);
                continue;
            }

            // Locate by 0-based INDEX when the value is an integer, otherwise by
            // header NAME. Probe the integer form first with a guard so a string
            // value never throws.
            bool by_index = false;
            int  index    = -1;
            if (locator && locator.IsScalar()) {
                try {
                    index    = locator.as<int>();
                    by_index = true;
                } catch (const YAML::Exception&) {
                    by_index = false;
                }
            }

            if (by_index) {
                if (index < 0) {
                    spdlog::warn("DroneTelemetryConfig: field '{}' has negative index {}, "
                                 "skipping", field_str, index);
                    continue;
                }
                spec.index = index;
            } else {
                spec.name = locator.as<std::string>("");
                if (spec.name.empty()) {
                    spdlog::warn("DroneTelemetryConfig: field '{}' has empty column locator, "
                                 "skipping", field_str);
                    continue;
                }
            }
            cfg.columns.push_back(spec);
        }
    }

    if (cfg.columns.empty()) {
        spdlog::info("DroneTelemetryConfig: no 'columns' configured, using default YenBai mapping");
        cfg.columns = defaultColumns();
    }
    return cfg;
}

// ─── DroneTelemetryCsvReader ──────────────────────────────────────────────────

DroneTelemetryCsvReader::DroneTelemetryCsvReader(DroneTelemetryConfig config)
    : config_(std::move(config)) {}

std::vector<std::string> DroneTelemetryCsvReader::splitLine(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        auto start = tok.find_first_not_of(" \t\r\n");
        auto end   = tok.find_last_not_of(" \t\r\n");
        tokens.push_back(start == std::string::npos ? "" : tok.substr(start, end - start + 1));
    }
    return tokens;
}

std::vector<std::pair<DroneTelemetryField, int>>
DroneTelemetryCsvReader::resolveColumns(const std::vector<std::string>& headers) const {
    std::unordered_map<std::string, int> header_idx;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i)
        header_idx[headers[i]] = i;

    std::vector<std::pair<DroneTelemetryField, int>> resolved;
    for (const auto& spec : config_.columns) {
        int idx = -1;
        if (spec.locateByName()) {
            auto it = header_idx.find(spec.name);
            if (it != header_idx.end()) {
                idx = it->second;
            } else {
                spdlog::warn("DroneTelemetryCsvReader: header column '{}' not found, "
                             "field left at default", spec.name);
                continue;
            }
        } else {
            idx = spec.index;  // header-less or index-located column
        }
        resolved.emplace_back(spec.field, idx);
    }
    return resolved;
}

void DroneTelemetryCsvReader::assignField(DroneTelemetryRecord& rec,
                                          DroneTelemetryField field,
                                          const std::string&  value) {
    switch (field) {
        case DroneTelemetryField::FRAME_ID:            rec.frame_id            = toU64(value);    break;
        case DroneTelemetryField::TIMESTAMP_MSEC:      rec.timestamp_msec      = toDouble(value); break;
        case DroneTelemetryField::ROLL_DEG:            rec.roll_deg            = toDouble(value); break;
        case DroneTelemetryField::PITCH_DEG:           rec.pitch_deg           = toDouble(value); break;
        case DroneTelemetryField::YAW_DEG:             rec.yaw_deg             = toDouble(value); break;
        case DroneTelemetryField::GIMBAL_PAN_DEG:      rec.gimbal_pan_deg      = toDouble(value); break;
        case DroneTelemetryField::GIMBAL_TILT_DEG:     rec.gimbal_tilt_deg     = toDouble(value); break;
        case DroneTelemetryField::AMSL_M:              rec.amsl_m              = toDouble(value); break;
        case DroneTelemetryField::SENSOR_ALTITUDE_M:   rec.sensor_altitude_m   = toDouble(value); break;
        case DroneTelemetryField::GROUND_SPEED_MPS:    rec.ground_speed_mps    = toDouble(value); break;
        case DroneTelemetryField::LATITUDE_DEG:        rec.latitude_deg        = toDouble(value); break;
        case DroneTelemetryField::LONGITUDE_DEG:       rec.longitude_deg       = toDouble(value); break;
        case DroneTelemetryField::DISTANCE_TO_TARGET_M: rec.distance_to_target_m = toDouble(value); break;
        case DroneTelemetryField::UNKNOWN:             /* ignored */                              break;
    }
}

bool DroneTelemetryCsvReader::load() {
    std::ifstream file(config_.csv_path);
    if (!file.is_open()) {
        spdlog::error("DroneTelemetryCsvReader: cannot open '{}'", config_.csv_path);
        return false;
    }

    std::vector<std::pair<DroneTelemetryField, int>> resolved;

    if (config_.has_header) {
        std::string header_line;
        if (!std::getline(file, header_line)) {
            spdlog::error("DroneTelemetryCsvReader: '{}' is empty", config_.csv_path);
            return false;
        }
        resolved = resolveColumns(splitLine(header_line));
    } else {
        // No header: every column must be located by explicit index.
        for (const auto& spec : config_.columns) {
            if (spec.index < 0) {
                spdlog::warn("DroneTelemetryCsvReader: has_header=false requires 'index' for "
                             "every column, skipping a name-only column");
                continue;
            }
            resolved.emplace_back(spec.field, spec.index);
        }
    }

    if (resolved.empty()) {
        spdlog::error("DroneTelemetryCsvReader: no columns could be resolved for '{}'",
                      config_.csv_path);
        return false;
    }

    size_t row_num = config_.has_header ? 1 : 0;
    std::string line;
    while (std::getline(file, line)) {
        ++row_num;
        if (line.empty() || line[0] == '#') continue;
        auto values = splitLine(line);

        DroneTelemetryRecord rec;
        for (const auto& [field, idx] : resolved) {
            if (idx < 0 || idx >= static_cast<int>(values.size())) continue;
            assignField(rec, field, values[idx]);
        }
        rec.valid = true;
        records_.push_back(rec);
    }

    if (records_.empty()) {
        spdlog::error("DroneTelemetryCsvReader: no data rows in '{}'", config_.csv_path);
        return false;
    }

    frame_id_idx_.reserve(records_.size());
    for (size_t i = 0; i < records_.size(); ++i)
        frame_id_idx_[records_[i].frame_id] = i;

    loaded_ = true;
    spdlog::info("DroneTelemetryCsvReader: loaded {} records from '{}' ({} columns mapped)",
                 records_.size(), config_.csv_path, resolved.size());
    return true;
}

DroneTelemetryRecord DroneTelemetryCsvReader::byFrameId(uint64_t frame_id) const {
    auto it = frame_id_idx_.find(frame_id);
    if (it != frame_id_idx_.end()) return records_[it->second];
    return {};
}

TelemetryData DroneTelemetryCsvReader::toTelemetryData(const DroneTelemetryRecord& rec) {
    TelemetryData t;
    t.frame_id      = rec.frame_id;
    t.timestamp_msec = rec.timestamp_msec;
    t.heading_deg   = rec.yaw_deg;
    t.pitch_deg     = rec.pitch_deg;
    t.roll_deg      = rec.roll_deg;
    t.gimbal_pan_deg  = rec.gimbal_pan_deg;
    t.gimbal_tilt_deg = rec.gimbal_tilt_deg;
    t.latitude_deg  = rec.latitude_deg;
    t.longitude_deg = rec.longitude_deg;
    t.altitude_m    = rec.sensor_altitude_m;  // real AGL height (NOT the 'agl' column)
    t.speed_mps     = rec.ground_speed_mps;
    t.climb_mps     = 0.0;                     // not present in the drone log
    t.valid         = rec.valid;
    return t;
}

}  // namespace uavloc::sensor
