#include "uavloc/sensor/video_reader.h"
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <stdexcept>

namespace uavloc::sensor {

// ─── VideoReaderConfig ────────────────────────────────────────────────────────

VideoReaderConfig VideoReaderConfig::fromYaml(const YAML::Node& node) {
    const auto& n = node["VideoReader"];
    if (!n) throw std::runtime_error("YAML missing 'VideoReader' section");

    VideoReaderConfig cfg;
    cfg.video_path          = n["video_path"].as<std::string>(cfg.video_path);
    cfg.start_frame         = n["start_frame"].as<int>(cfg.start_frame);
    cfg.end_frame           = n["end_frame"].as<int>(cfg.end_frame);
    cfg.loop                = n["loop"].as<bool>(cfg.loop);
    cfg.convert_to_grayscale = n["convert_to_grayscale"].as<bool>(cfg.convert_to_grayscale);
    cfg.use_video_timestamp  = n["use_video_timestamp"].as<bool>(cfg.use_video_timestamp);
    cfg.camera_id            = n["camera_id"].as<std::string>(cfg.camera_id);
    cfg.source_name          = n["source_name"].as<std::string>(cfg.source_name);
    cfg.telemetry_csv_path   = n["telemetry_csv_path"].as<std::string>("");
    cfg.resize_width         = n["resize_width"].as<int>(cfg.resize_width);
    cfg.resize_height        = n["resize_height"].as<int>(cfg.resize_height);

    // ── Telemetry source B: 57-column drone flight log ───────────────────────
    if (n["DroneTelemetry"]) {
        cfg.drone_telemetry     = DroneTelemetryConfig::fromYaml(n["DroneTelemetry"]);
        cfg.use_drone_telemetry = !cfg.drone_telemetry.csv_path.empty();
    }
    cfg.telemetry_frame_id_offset =
        n["telemetry_frame_id_offset"].as<int64_t>(cfg.telemetry_frame_id_offset);
    return cfg;
}

// ─── VideoReader ──────────────────────────────────────────────────────────────

VideoReader::VideoReader(const VideoReaderConfig& config)
    : config_(config) {}

VideoReader::~VideoReader() { close(); }

bool VideoReader::open() {
    if (config_.video_path.empty()) {
        spdlog::error("VideoReader: video_path is empty");
        return false;
    }

    cap_.open(config_.video_path);
    if (!cap_.isOpened()) {
        spdlog::error("VideoReader: cannot open '{}'", config_.video_path);
        return false;
    }

    fps_          = cap_.get(cv::CAP_PROP_FPS);
    total_frames_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));

    if (config_.start_frame > 0) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, config_.start_frame);
        current_frame_ = config_.start_frame;
    }

    const int src_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int src_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

    spdlog::info("VideoReader: opened '{}', {}x{} @ {:.1f} fps, {} frames",
        config_.video_path, src_w, src_h, fps_, total_frames_);

    // ── Optional output resize: both dimensions required to activate ─────────
    if ((config_.resize_width > 0) != (config_.resize_height > 0)) {
        spdlog::warn("VideoReader: resize needs BOTH resize_width and resize_height > 0 "
                     "(got {}x{}) — resize disabled",
                     config_.resize_width, config_.resize_height);
        config_.resize_width  = 0;
        config_.resize_height = 0;
    } else if (config_.resize_width > 0 && config_.resize_height > 0) {
        spdlog::info("VideoReader: output resize enabled {}x{} (source {}x{})",
                     config_.resize_width, config_.resize_height, src_w, src_h);
    }

    // ── Telemetry source selection (drone-log B takes priority over simple A) ──
    const bool has_drone  = !config_.drone_telemetry.csv_path.empty();
    const bool has_simple = !config_.telemetry_csv_path.empty();

    if (has_drone && has_simple) {
        spdlog::warn("VideoReader: both DroneTelemetry and telemetry_csv_path configured — "
                     "using the drone log and ignoring telemetry_csv_path");
    }

    if (has_drone) {
        drone_telemetry_reader_ =
            std::make_unique<DroneTelemetryCsvReader>(config_.drone_telemetry);
        if (!drone_telemetry_reader_->load()) {
            spdlog::warn("VideoReader: drone telemetry CSV failed to load — "
                         "continuing without telemetry");
            drone_telemetry_reader_.reset();
        } else {
            spdlog::info("VideoReader: drone telemetry enabled, lookup starts at imageId {} "
                         "(offset {})",
                         config_.start_frame + config_.telemetry_frame_id_offset,
                         config_.telemetry_frame_id_offset);
        }
    } else if (has_simple) {
        telemetry_reader_ = std::make_unique<TelemetryCsvReader>(config_.telemetry_csv_path);
        if (!telemetry_reader_->load()) {
            spdlog::warn("VideoReader: telemetry CSV failed to load — continuing without telemetry");
            telemetry_reader_.reset();
        }
    }

    return true;
}

FrameStatus VideoReader::read(FrameData& frame) {
    if (!cap_.isOpened()) return FrameStatus::ERROR;

    if (isEndFrameReached()) {
        if (config_.loop) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, config_.start_frame);
            current_frame_ = config_.start_frame;
        } else {
            return FrameStatus::END_OF_STREAM;
        }
    }

    cv::Mat raw;
    if (!cap_.read(raw)) {
        if (current_frame_ >= total_frames_) return FrameStatus::END_OF_STREAM;
        return FrameStatus::EMPTY_FRAME;
    }

    if (raw.empty()) return FrameStatus::EMPTY_FRAME;

    // Resize BEFORE any grayscale conversion so every downstream consumer sees
    // the target resolution (intrinsics must be calibrated for it).
    if (config_.resize_width > 0 && config_.resize_height > 0 &&
        (raw.cols != config_.resize_width || raw.rows != config_.resize_height)) {
        cv::Mat resized;
        cv::resize(raw, resized,
                   cv::Size(config_.resize_width, config_.resize_height),
                   0, 0, cv::INTER_AREA);
        raw = resized;
    }

    ++current_frame_;

    if (!fillFrameData(raw, frame)) return FrameStatus::ERROR;
    return FrameStatus::OK;
}

void VideoReader::close() {
    if (cap_.isOpened()) cap_.release();
}

bool VideoReader::isOpened() const { return cap_.isOpened(); }

std::string VideoReader::name() const { return config_.source_name; }

double VideoReader::getFps()        const { return fps_; }
int    VideoReader::getFrameCount() const { return total_frames_; }

// ─── private helpers ─────────────────────────────────────────────────────────

double VideoReader::getTimestampMsec() const {
    if (config_.use_video_timestamp) return cap_.get(cv::CAP_PROP_POS_MSEC);

    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double, std::milli>(
        clock::now().time_since_epoch()).count();
}

bool VideoReader::isEndFrameReached() const {
    if (config_.end_frame < 0) return false;
    return current_frame_ > config_.end_frame;
}

bool VideoReader::fillFrameData(const cv::Mat& image, FrameData& frame) {
    cv::Mat out;
    if (config_.convert_to_grayscale) {
        cv::cvtColor(image, out, cv::COLOR_BGR2GRAY);
    } else {
        out = image;
    }

    frame.frame_id       = frame_id_++;
    frame.timestamp_msec = getTimestampMsec();
    frame.image          = out;
    frame.camera_id      = config_.camera_id;
    frame.source_name    = config_.source_name;
    frame.status         = FrameStatus::OK;
    frame.valid          = true;

    if (drone_telemetry_reader_ && drone_telemetry_reader_->isLoaded()) {
        // frame_id_ counts delivered frames from 0; the drone log keys on the
        // absolute imageId, so shift by the user-supplied offset (the absolute
        // imageId of the first decoded frame). A miss yields an invalid record.
        const int64_t lookup_id =
            static_cast<int64_t>(frame.frame_id) + config_.telemetry_frame_id_offset;
        DroneTelemetryRecord rec;
        if (lookup_id >= 0)
            rec = drone_telemetry_reader_->byFrameId(static_cast<uint64_t>(lookup_id));
        frame.telemetry     = DroneTelemetryCsvReader::toTelemetryData(rec);
        frame.has_telemetry = frame.telemetry.valid;
    } else if (telemetry_reader_ && telemetry_reader_->isLoaded()) {
        frame.telemetry     = telemetry_reader_->syncByFrameId(frame.frame_id, frame.timestamp_msec);
        frame.has_telemetry = frame.telemetry.valid;
    }

    return true;
}

}  // namespace uavloc::sensor
