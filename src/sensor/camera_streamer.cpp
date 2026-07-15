#include "uavloc/sensor/camera_streamer.h"
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <stdexcept>

namespace uavloc::sensor {

// ─── CameraStreamerConfig ─────────────────────────────────────────────────────

static CameraStreamerConfig::ColorOrder parse_color_order(const std::string& s) {
    if (s == "RGB")                  return CameraStreamerConfig::ColorOrder::RGB;
    if (s == "GRAY" || s == "Gray") return CameraStreamerConfig::ColorOrder::GRAY;
    if (s == "BGR")                  return CameraStreamerConfig::ColorOrder::BGR;
    throw std::runtime_error("CameraStreamer: unknown color_order '" + s + "'");
}

static int parse_backend(const std::string& s) {
    if (s == "V4L2")      return cv::CAP_V4L2;
    if (s == "GSTREAMER") return cv::CAP_GSTREAMER;
    if (s == "FFMPEG")    return cv::CAP_FFMPEG;
    if (s == "ANY")       return cv::CAP_ANY;
    throw std::runtime_error("CameraStreamer: unknown backend '" + s + "'");
}

CameraStreamerConfig CameraStreamerConfig::fromYaml(const YAML::Node& node) {
    const auto& n = node["CameraStreamer"];
    if (!n) throw std::runtime_error("YAML missing 'CameraStreamer' section");

    CameraStreamerConfig cfg;
    cfg.device_id             = n["device_id"].as<uint8_t>(cfg.device_id);
    cfg.source_uri            = n["source_uri"].as<std::string>("");
    cfg.width                 = n["width"].as<int>(cfg.width);
    cfg.height                = n["height"].as<int>(cfg.height);
    cfg.fps                   = n["fps"].as<double>(cfg.fps);
    cfg.backend               = parse_backend(n["backend"].as<std::string>("ANY"));
    cfg.fourcc                = n["fourcc"].as<std::string>("");
    cfg.color_order           = parse_color_order(n["color_order"].as<std::string>("BGR"));
    cfg.use_system_timestamp  = n["use_system_timestamp"].as<bool>(cfg.use_system_timestamp);
    cfg.timeout_ms            = n["timeout_ms"].as<int>(cfg.timeout_ms);
    cfg.buffer_size           = n["buffer_size"].as<int>(cfg.buffer_size);
    return cfg;
}

// ─── CameraStreamer ───────────────────────────────────────────────────────────

CameraStreamer::CameraStreamer(const CameraStreamerConfig& config)
    : config_(config) {}

CameraStreamer::~CameraStreamer() { close(); }

bool CameraStreamer::open() {
    if (!config_.source_uri.empty()) {
        cap_.open(config_.source_uri, config_.backend);
    } else {
        cap_.open(config_.device_id, config_.backend);
    }

    if (!cap_.isOpened()) {
        spdlog::error("CameraStreamer: failed to open device_id={} uri='{}'",
            config_.device_id, config_.source_uri);
        return false;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  config_.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
    cap_.set(cv::CAP_PROP_FPS,          config_.fps);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   config_.buffer_size);

    if (config_.fourcc.size() == 4) {
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(
            config_.fourcc[0], config_.fourcc[1],
            config_.fourcc[2], config_.fourcc[3]));
    }

    initialized_ = true;
    spdlog::info("CameraStreamer: opened {}x{} @ {:.1f} fps (backend={})",
        static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)),
        cap_.get(cv::CAP_PROP_FPS),
        config_.backend);
    return true;
}

FrameStatus CameraStreamer::read(FrameData& frame) {
    if (!initialized_ || !cap_.isOpened())
        return FrameStatus::CAMERA_DISCONNECTED;

    cv::Mat raw;
    if (!cap_.read(raw)) {
        if (!cap_.isOpened()) {
            initialized_ = false;
            return FrameStatus::CAMERA_DISCONNECTED;
        }
        return FrameStatus::EMPTY_FRAME;
    }

    if (raw.empty()) return FrameStatus::EMPTY_FRAME;

    if (!fillFrameData(raw, frame)) return FrameStatus::ERROR;
    return FrameStatus::OK;
}

void CameraStreamer::close() {
    if (cap_.isOpened()) cap_.release();
    initialized_ = false;
}

bool CameraStreamer::isOpened() const { return initialized_ && cap_.isOpened(); }

std::string CameraStreamer::name() const { return "CameraStreamer"; }

// ─── private helpers ──────────────────────────────────────────────────────────

double CameraStreamer::getTimestampMsec() const {
    if (config_.use_system_timestamp) {
        using clock = std::chrono::steady_clock;
        return std::chrono::duration<double, std::milli>(
            clock::now().time_since_epoch()).count();
    }
    return cap_.get(cv::CAP_PROP_POS_MSEC);
}

bool CameraStreamer::fillFrameData(const cv::Mat& raw, FrameData& frame) {
    cv::Mat converted;
    switch (config_.color_order) {
        case CameraStreamerConfig::ColorOrder::RGB:
            cv::cvtColor(raw, converted, cv::COLOR_BGR2RGB);
            break;
        case CameraStreamerConfig::ColorOrder::GRAY:
            cv::cvtColor(raw, converted, cv::COLOR_BGR2GRAY);
            break;
        default:
            converted = raw;
            break;
    }

    frame.frame_id       = frame_id_++;
    frame.timestamp_msec = getTimestampMsec();
    frame.image          = converted;
    frame.camera_id      = "camera0";
    frame.source_name    = "live_camera";
    frame.status         = FrameStatus::OK;
    frame.valid          = true;
    return true;
}

}  // namespace uavloc::sensor
