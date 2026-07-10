#pragma once

namespace uavloc::debug_viewer {

struct TelemetryRecord {
    int    frame_id     = 0;
    double latitude     = 0.0;
    double longitude    = 0.0;
    double altitude_m   = 0.0;
    double roll_deg     = 0.0;
    double pitch_deg    = 0.0;
    double yaw_deg      = 0.0;
    double ground_speed = 0.0;
};

} // namespace uavloc::debug_viewer
