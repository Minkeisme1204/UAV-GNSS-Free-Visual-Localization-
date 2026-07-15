#include <uavloc/debug_viewer/telemetry_csv_reader.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <spdlog/spdlog.h>

namespace uavloc::debug_viewer {

namespace {

constexpr int CSV_EXPECTED_COLS = 57;
constexpr int CSV_MIN_COLS      = 33;   // highest required column index + 1

constexpr int COL_IMAGE_ID     = 0;
constexpr int COL_ROLL         = 21;
constexpr int COL_PITCH        = 22;
constexpr int COL_YAW          = 23;
constexpr int COL_GROUND_SPEED = 26;
constexpr int COL_SENSOR_LAT   = 30;
constexpr int COL_SENSOR_LON   = 31;
constexpr int COL_SENSOR_ALT   = 32;

// Trim leading and trailing ASCII whitespace from s.
std::string trim(const std::string& s) {
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return {};
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

} // anonymous namespace

std::vector<TelemetryRecord> load_telemetry_csv(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("load_telemetry_csv: cannot open file: " + path);
    }

    std::vector<TelemetryRecord> records;
    std::string line;
    int line_num = 1;

    // Skip header row
    if (!std::getline(file, line)) {
        return records;
    }

    while (std::getline(file, line)) {
        ++line_num;
        std::vector<std::string> tokens;
        tokens.reserve(CSV_EXPECTED_COLS);

        std::istringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(trim(token));
        }

        // Need at least CSV_MIN_COLS columns (indices 0..CSV_MIN_COLS-1)
        if (tokens.size() < static_cast<size_t>(CSV_MIN_COLS)) {
            spdlog::warn("load_telemetry_csv: line {} has only {} columns (need {}), skipping",
                         line_num, tokens.size(), CSV_MIN_COLS);
            continue;
        }

        TelemetryRecord rec;
        try {
            rec.latitude   = std::stod(tokens[COL_SENSOR_LAT]);
            rec.longitude  = std::stod(tokens[COL_SENSOR_LON]);
            rec.altitude_m = std::stod(tokens[COL_SENSOR_ALT]);

            // Skip rows where GPS is not yet locked (lat == 0.0)
            if (rec.latitude == 0.0) {
                spdlog::debug("load_telemetry_csv: line {} has lat==0.0, GPS not locked — skipping",
                              line_num);
                continue;
            }

            rec.frame_id     = std::stoi(tokens[COL_IMAGE_ID]);
            rec.roll_deg     = std::stod(tokens[COL_ROLL]);
            rec.pitch_deg    = std::stod(tokens[COL_PITCH]);
            rec.yaw_deg      = std::stod(tokens[COL_YAW]);
            rec.ground_speed = std::stod(tokens[COL_GROUND_SPEED]);
        } catch (const std::exception& ex) {
            spdlog::warn("load_telemetry_csv: line {} parse error: {} — skipping",
                         line_num, ex.what());
            continue;
        }

        records.push_back(rec);
    }

    return records;
}

} // namespace uavloc::debug_viewer
