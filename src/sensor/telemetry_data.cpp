#include "uavloc/sensor/telemetry_data.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace uavloc::sensor {

// ─── TelemetryCsvReader ───────────────────────────────────────────────────────

TelemetryCsvReader::TelemetryCsvReader(const std::string& csv_path)
    : csv_path_(csv_path) {}

// Split a single CSV line on commas, stripping leading/trailing whitespace
std::vector<std::string> TelemetryCsvReader::splitLine(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        // strip whitespace
        auto start = tok.find_first_not_of(" \t\r\n");
        auto end   = tok.find_last_not_of(" \t\r\n");
        tokens.push_back(start == std::string::npos ? "" : tok.substr(start, end - start + 1));
    }
    return tokens;
}

TelemetryData TelemetryCsvReader::parseRow(
    const std::vector<std::string>&            values,
    const std::unordered_map<std::string, int>& col) const
{
    auto get = [&](const std::string& key, double def = 0.0) -> double {
        auto it = col.find(key);
        if (it == col.end() || it->second >= static_cast<int>(values.size())) return def;
        try { return std::stod(values[it->second]); } catch (...) { return def; }
    };
    auto getU64 = [&](const std::string& key) -> uint64_t {
        auto it = col.find(key);
        if (it == col.end() || it->second >= static_cast<int>(values.size())) return 0;
        try { return std::stoull(values[it->second]); } catch (...) { return 0; }
    };

    TelemetryData t;
    t.timestamp_msec = get("timestamp_msec");
    t.frame_id       = getU64("frame_id");
    t.heading_deg    = get("heading_deg");
    t.pitch_deg      = get("pitch_deg");
    t.roll_deg       = get("roll_deg");
    t.latitude_deg   = get("latitude_deg");
    t.longitude_deg  = get("longitude_deg");
    t.altitude_m     = get("altitude_m");
    t.speed_mps      = get("speed_mps");
    t.climb_mps      = get("climb_mps");
    t.valid          = true;
    return t;
}

bool TelemetryCsvReader::load() {
    std::ifstream file(csv_path_);
    if (!file.is_open()) {
        spdlog::error("TelemetryCsvReader: cannot open '{}'", csv_path_);
        return false;
    }

    // Parse header row
    std::string line;
    if (!std::getline(file, line)) {
        spdlog::error("TelemetryCsvReader: '{}' is empty", csv_path_);
        return false;
    }

    auto headers = splitLine(line);
    std::unordered_map<std::string, int> col;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i)
        col[headers[i]] = i;

    if (col.find("timestamp_msec") == col.end()) {
        spdlog::error("TelemetryCsvReader: required column 'timestamp_msec' not found in '{}'",
            csv_path_);
        return false;
    }

    has_frame_id_col_ = col.count("frame_id") > 0;

    // Parse data rows
    size_t row_num = 1;
    while (std::getline(file, line)) {
        ++row_num;
        if (line.empty() || line[0] == '#') continue;
        auto values = splitLine(line);
        if (values.size() < col.size()) {
            spdlog::warn("TelemetryCsvReader: row {} has fewer columns than header, skipping",
                row_num);
            continue;
        }
        records_.push_back(parseRow(values, col));
    }

    if (records_.empty()) {
        spdlog::error("TelemetryCsvReader: no data rows in '{}'", csv_path_);
        return false;
    }

    // Sort by timestamp for binary search
    std::sort(records_.begin(), records_.end(),
        [](const TelemetryData& a, const TelemetryData& b) {
            return a.timestamp_msec < b.timestamp_msec;
        });

    // Build frame_id index
    if (has_frame_id_col_) {
        for (size_t i = 0; i < records_.size(); ++i)
            frame_id_idx_[records_[i].frame_id] = i;
    }

    loaded_ = true;
    spdlog::info("TelemetryCsvReader: loaded {} records from '{}' (frame_id col: {})",
        records_.size(), csv_path_, has_frame_id_col_);
    return true;
}

TelemetryData TelemetryCsvReader::syncByTimestamp(double timestamp_msec) const {
    if (records_.empty()) return {};

    // Binary search for first record with timestamp >= query
    auto it = std::lower_bound(records_.begin(), records_.end(), timestamp_msec,
        [](const TelemetryData& r, double t) { return r.timestamp_msec < t; });

    if (it == records_.end()) return records_.back();
    if (it == records_.begin()) return records_.front();

    // Pick the closer of the two neighbours
    auto prev = std::prev(it);
    if ((timestamp_msec - prev->timestamp_msec) <= (it->timestamp_msec - timestamp_msec))
        return *prev;
    return *it;
}

TelemetryData TelemetryCsvReader::syncByFrameId(
    uint64_t frame_id, double timestamp_msec) const
{
    if (has_frame_id_col_) {
        auto it = frame_id_idx_.find(frame_id);
        if (it != frame_id_idx_.end()) return records_[it->second];
    }
    return syncByTimestamp(timestamp_msec);
}

bool   TelemetryCsvReader::isLoaded() const { return loaded_; }
size_t TelemetryCsvReader::size()     const { return records_.size(); }

}  // namespace uavloc::sensor
