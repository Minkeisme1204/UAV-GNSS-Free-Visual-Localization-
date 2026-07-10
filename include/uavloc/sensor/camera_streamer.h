#ifndef CAMERA_STREAMER_H
#define CAMERA_STREAMER_H

#include "uavloc/sensor/data_interface.h"
#include <cstdint>
#include <string>
#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

namespace uavloc::sensor {

struct CameraStreamerConfig {
    // --- Device ---
    uint8_t     device_id  = 0;   // local device index (e.g. /dev/video0 → 0)
    std::string source_uri;       // overrides device_id when non-empty: RTSP URL, GStreamer pipeline, or file path

    // --- Capture geometry ---
    int    width  = 640;
    int    height = 480;
    double fps    = 30.0;

    // --- OpenCV backend ---
    // cv::CAP_V4L2 for USB cameras on Linux, cv::CAP_GSTREAMER for pipeline strings
    int         backend = cv::CAP_ANY;
    std::string fourcc;           // four-character codec hint, e.g. "MJPG" to unlock high fps on USB cameras

    // --- Color format (follows stella_vslam color_order convention) ---
    // VideoCapture delivers BGR natively; convert only when downstream needs RGB/Gray
    enum class ColorOrder { BGR, RGB, GRAY };
    ColorOrder color_order = ColorOrder::BGR;

    // --- Timestamp ---
    // true  = wall-clock (std::chrono::steady_clock) written to FrameData::timestamp_msec
    // false = camera hardware PTS via CAP_PROP_POS_MSEC (unreliable on some drivers)
    bool use_system_timestamp = true;

    // --- Reliability ---
    int timeout_ms  = 100;        // per-frame read deadline; exceeded → FrameStatus::TIMEOUT
    int buffer_size = 1;          // VideoCapture internal queue depth; 1 = minimum latency

    static CameraStreamerConfig fromYaml(const YAML::Node& node);
};

class CameraStreamer : public DataInterface {
public:
    explicit CameraStreamer(const CameraStreamerConfig& config);
    ~CameraStreamer() override;

    bool        open()                    override;
    FrameStatus read(FrameData& frame)    override;
    void        close()                   override;
    bool        isOpened()          const override;
    std::string name()              const override;

private:
    CameraStreamerConfig config_;
    cv::VideoCapture     cap_;

    uint64_t frame_id_    = 0;
    bool     initialized_ = false;

    double   getTimestampMsec()                              const;
    bool     fillFrameData(const cv::Mat& raw, FrameData& frame);
};

}  // namespace uavloc::sensor

#endif // CAMERA_STREAMER_H