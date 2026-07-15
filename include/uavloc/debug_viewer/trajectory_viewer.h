#pragma once
#include <uavloc/debug_viewer/telemetry_record.h>
#include <memory>
#include <string>
#include <vector>

namespace uavloc::debug_viewer {

class TrajectoryViewer {
public:
    struct Config {
        std::string window_title        = "UAV Trajectory Viewer";
        int         window_w            = 1280;
        int         window_h            = 720;
        float       point_size          = 3.0f;
        int         coord_frame_every_n = 200;  // draw UAV axes every N records; 0 = disable
        float       coord_frame_scale   = 10.0f;   // visual scale of UAV orientation axes
    };

    TrajectoryViewer();                          // uses default Config
    explicit TrajectoryViewer(const Config& cfg);
    ~TrajectoryViewer();

    // Load all records at once before calling spin()
    void setRecords(const std::vector<TelemetryRecord>& records);

    // Blocking render loop — returns when window is closed
    void spin();

    // Non-blocking single frame — returns false when window should close
    bool spinOnce();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::debug_viewer
