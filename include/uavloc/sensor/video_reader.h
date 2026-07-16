#ifndef VIDEO_READER_H
#define VIDEO_READER_H

#include "uavloc/sensor/data_interface.h"
#include "uavloc/sensor/telemetry_data.h"
#include "uavloc/sensor/drone_telemetry_csv_reader.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/videoio.hpp>
#include <cstdint>
#include <memory>

namespace uavloc::sensor {

struct VideoReaderConfig {
    std::string video_path;

    int start_frame = 0;
    int end_frame   = -1;   // inclusive; -1 = read until end

    bool loop                 = false;
    bool convert_to_grayscale = false;
    bool use_video_timestamp  = true;

    std::string camera_id   = "camera0";
    std::string source_name = "video_file";

    // Optional output resize: when BOTH are > 0, every decoded frame is resized
    // to exactly (resize_width x resize_height) before being published in
    // FrameData. The Camera intrinsics in the mission config must be calibrated
    // for the TARGET resolution. 0 (default) = publish at native resolution.
    int resize_width  = 0;
    int resize_height = 0;

    // ── Telemetry source A: simple 10-column uavloc CSV (TelemetryCsvReader) ──
    // Optional: path to telemetry CSV; empty = no telemetry sync
    std::string telemetry_csv_path;

    // ── Telemetry source B: 57-column drone flight log (DroneTelemetryCsvReader) ──
    // Enabled when drone_telemetry.csv_path is non-empty. When both sources are
    // configured, the drone log takes priority (and a warning is logged).
    bool                 use_drone_telemetry = false;
    DroneTelemetryConfig drone_telemetry;

    // Offset that maps the video's running frame counter onto the absolute
    // imageId used as the lookup key in the drone log. The cut .mkv does NOT
    // start at imageId 0, so the user supplies the absolute imageId of the
    // first decoded frame here. Lookup key formula (see fillFrameData):
    //   lookup_image_id = frame_id_ + telemetry_frame_id_offset
    // Only used for the drone-log source. Default 0 = no shift.
    int64_t telemetry_frame_id_offset = 0;

    static VideoReaderConfig fromYaml(const YAML::Node& node);
};

class VideoReader final : public DataInterface {
public:
    explicit VideoReader(const VideoReaderConfig& config);
    ~VideoReader() override;

    bool        open()                    override;
    FrameStatus read(FrameData& frame)    override;
    void        close()                   override;
    bool        isOpened()          const override;
    std::string name()              const override;

    double getFps()        const;
    int    getFrameCount() const;

private:
    VideoReaderConfig config_;
    cv::VideoCapture  cap_;

    uint64_t frame_id_      = 0;
    int      current_frame_ = 0;
    double   fps_           = 0.0;
    int      total_frames_  = 0;

    std::unique_ptr<TelemetryCsvReader>      telemetry_reader_;
    std::unique_ptr<DroneTelemetryCsvReader> drone_telemetry_reader_;

    double getTimestampMsec()                              const;
    bool   isEndFrameReached()                             const;
    bool   fillFrameData(const cv::Mat& image, FrameData& frame);
};


}  // namespace uavloc::sensor

#endif // VIDEO_READER_H