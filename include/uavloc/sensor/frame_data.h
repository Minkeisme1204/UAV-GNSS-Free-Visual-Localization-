#ifndef FRAME_DATA_H
#define FRAME_DATA_H

#include "uavloc/sensor/telemetry_data.h"
#include <cstdint>
#include <string>
#include <opencv2/core.hpp>

namespace uavloc::sensor {

enum class FrameStatus {
    OK,
    END_OF_STREAM,
    EMPTY_FRAME,
    CAMERA_DISCONNECTED,
    TIMEOUT,
    ERROR
};

struct FrameData {
    uint64_t frame_id       = 0;
    double   timestamp_msec = 0.0;

    cv::Mat image;

    std::string camera_id;
    std::string source_name;

    TelemetryData telemetry;
    bool          has_telemetry = false;

    FrameStatus status = FrameStatus::OK;
    bool        valid  = false;

    bool HasImage() const { return !image.empty(); }

    void Reset() {
        frame_id       = 0;
        timestamp_msec = 0.0;
        image.release();
        camera_id.clear();
        source_name.clear();
        telemetry     = TelemetryData{};
        has_telemetry = false;
        status = FrameStatus::OK;
        valid  = false;
    }
};

}  // namespace uavloc::sensor

#endif  // FRAME_DATA_H
