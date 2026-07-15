#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

namespace uavloc::sensor {

// One telemetry sample from the UAV's onboard sensors.
// All angular fields are in degrees; distances in metres.
struct TelemetryData {
    // Sync keys — matched against FrameData::frame_id / timestamp_msec
    uint64_t frame_id       = 0;
    double   timestamp_msec = 0.0;

    // Attitude
    double heading_deg = 0.0;   // yaw: 0 = North, clockwise
    double pitch_deg   = 0.0;   // positive = nose up
    double roll_deg    = 0.0;   // positive = right-wing down

    // Gimbal orientation (camera pointing relative to the airframe)
    double gimbal_pan_deg  = 0.0;   // camera pan; added to airframe yaw for the
                                    // geographic view azimuth
    double gimbal_tilt_deg = 0.0;   // from the horizontal plane; ~90 = nadir view

    // Position (WGS-84)
    double latitude_deg  = 0.0;
    double longitude_deg = 0.0;
    double altitude_m    = 0.0;  // above ground level

    // Motion
    double speed_mps = 0.0;     // horizontal ground speed
    double climb_mps = 0.0;     // vertical speed, positive = climbing

    bool valid = false;
};

// Loads a telemetry CSV file into memory and synchronizes records to video
// frames by frame_id (exact) or timestamp (nearest-neighbour).
//
// Expected CSV format — header row required, column order is flexible:
//   timestamp_msec, frame_id, heading_deg, pitch_deg, roll_deg,
//   latitude_deg, longitude_deg, altitude_m, speed_mps, climb_mps
//
// Only timestamp_msec is mandatory. All other columns default to 0 when absent.
class TelemetryCsvReader {
public:
    explicit TelemetryCsvReader(const std::string& csv_path);

    // Parses the CSV file into memory. Returns false on IO or parse error.
    bool load();

    // Returns the record whose timestamp_msec is nearest to the given value.
    TelemetryData syncByTimestamp(double timestamp_msec) const;

    // Returns the record matching frame_id exactly.
    // Falls back to syncByTimestamp(timestamp_msec) if no exact match exists.
    TelemetryData syncByFrameId(uint64_t frame_id, double timestamp_msec) const;

    bool   isLoaded() const;
    size_t size()     const;

private:
    std::string                          csv_path_;
    std::vector<TelemetryData>           records_;           // sorted by timestamp_msec
    std::unordered_map<uint64_t, size_t> frame_id_idx_;     // frame_id → index in records_
    bool                                 loaded_           = false;
    bool                                 has_frame_id_col_ = false;

    static std::vector<std::string> splitLine(const std::string& line);

    TelemetryData parseRow(const std::vector<std::string>&            values,
                           const std::unordered_map<std::string, int>& col) const;
};

}  // namespace uavloc::sensor

#endif  // TELEMETRY_DATA_H
