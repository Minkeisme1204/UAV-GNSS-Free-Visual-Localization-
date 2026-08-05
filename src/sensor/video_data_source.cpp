#include "uavloc/sensor/video_data_source.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <spdlog/spdlog.h>

#include "uavloc/sensor/frame_data.h"
#include "uavloc/sensor/video_reader.h"

namespace uavloc {
namespace sensor {

VideoDataSourceConfig VideoDataSourceConfig::fromYaml(const YAML::Node& node) {
    VideoDataSourceConfig cfg;
    if (!node) return cfg;

    cfg.max_consecutive_bad_reads =
        node["max_consecutive_bad_reads"].as<unsigned int>(cfg.max_consecutive_bad_reads);
    cfg.inter_frame_delay_ms =
        node["inter_frame_delay_ms"].as<unsigned int>(cfg.inter_frame_delay_ms);

    if (cfg.max_consecutive_bad_reads == 0) {
        spdlog::warn("VideoDataSourceConfig: max_consecutive_bad_reads = 0 would stop the "
                     "stream on the first non-OK read — clamped to 1");
        cfg.max_consecutive_bad_reads = 1;
    }
    return cfg;
}

VideoDataSource::VideoDataSource(std::unique_ptr<DataInterface> source,
                                 const VideoDataSourceConfig&   config)
    : source_(std::move(source)), config_(config) {
    if (config_.max_consecutive_bad_reads == 0) {
        config_.max_consecutive_bad_reads = 1;
    }
    if (!source_) {
        spdlog::warn("VideoDataSource: constructed with a null source — startStreaming() "
                     "will fail");
    }
}

VideoDataSource::~VideoDataSource() {
    stopStreaming();
}

bool VideoDataSource::startStreaming() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);

    if (streaming_.load() && thread_) {
        spdlog::debug("VideoDataSource: already streaming");
        return true;
    }
    // A previous run that ended by itself (EOS) leaves a joinable thread behind.
    if (thread_) {
        thread_->join();
        thread_.reset();
    }
    if (!source_) {
        spdlog::error("VideoDataSource::startStreaming: no source");
        return false;
    }
    if (!source_->isOpened() && !source_->open()) {
        spdlog::error("VideoDataSource::startStreaming: cannot open source '{}'",
                      source_->name());
        return false;
    }

    total_frames_.store(0);
    if (const auto* reader = dynamic_cast<const VideoReader*>(source_.get())) {
        total_frames_.store(reader->getFrameCount());
    }

    streaming_.store(true);
    thread_ = std::make_unique<std::thread>(&VideoDataSource::processing, this);
    spdlog::info("VideoDataSource: streaming from '{}' (bad-read limit {}, delay {} ms)",
                 source_->name(), config_.max_consecutive_bad_reads,
                 config_.inter_frame_delay_ms);
    return true;
}

bool VideoDataSource::stopStreaming() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);

    streaming_.store(false);
    if (!thread_) {
        return true;  // never started, or already stopped — idempotent
    }
    // Refuse to self-join: stopStreaming() called from inside a channel
    // callback runs ON the streaming thread. The flag above already ends the
    // loop after the current sample; joining here would deadlock.
    if (thread_->get_id() == std::this_thread::get_id()) {
        spdlog::warn("VideoDataSource::stopStreaming called from a channel callback — "
                     "the loop will exit, but the thread cannot join itself");
        return false;
    }
    thread_->join();
    thread_.reset();
    spdlog::info("VideoDataSource: stopped after {} frames", frames_published_.load());
    return true;
}

bool VideoDataSource::isStreaming() const { return streaming_.load(); }

double VideoDataSource::completion() const {
    const int total = total_frames_.load();
    if (total <= 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double done = static_cast<double>(frames_published_.load()) /
                        static_cast<double>(total);
    return done > 1.0 ? 1.0 : done;
}

uint64_t VideoDataSource::framesPublished() const { return frames_published_.load(); }

std::string VideoDataSource::name() const {
    return source_ ? source_->name() : std::string("<null source>");
}

// ─── private ─────────────────────────────────────────────────────────────────

void VideoDataSource::publishEvent(StreamEventKind kind, const std::string& message) {
    StreamEvent ev;
    ev.kind     = kind;
    ev.frame_id = last_frame_id_.load();
    ev.message  = message;
    event_channel_(ev);
}

void VideoDataSource::processing() {
    unsigned int consecutive_bad = 0;

    while (streaming_.load()) {
        FrameData        fd;
        const FrameStatus status = source_->read(fd);

        const bool good = (status == FrameStatus::OK) && fd.valid && fd.HasImage();
        if (!good) {
            ++consecutive_bad;

            if (status == FrameStatus::END_OF_STREAM) {
                publishEvent(StreamEventKind::END_OF_STREAM,
                             "source reported END_OF_STREAM");
                break;
            }
            if (status == FrameStatus::CAMERA_DISCONNECTED) {
                publishEvent(StreamEventKind::DISCONNECTED, "source disconnected");
                break;
            }
            // Non-terminal anomaly. Report the FIRST one of a run only: a cut
            // container yields hundreds of them and one event per read would
            // bury the END_OF_STREAM that follows.
            if (consecutive_bad == 1) {
                publishEvent(status == FrameStatus::TIMEOUT ? StreamEventKind::TIMEOUT
                                                            : StreamEventKind::READ_ERROR,
                             "read did not yield a usable frame");
            }
            if (consecutive_bad >= config_.max_consecutive_bad_reads) {
                publishEvent(StreamEventKind::END_OF_STREAM,
                             std::to_string(consecutive_bad) +
                                 " consecutive non-OK reads — treating as end of stream");
                break;
            }
            continue;
        }

        consecutive_bad = 0;
        const double t_msec = fd.timestamp_msec;

        // ── fan-out, kcb PCSDataAdapter::processing() order ──────────────────
        // attitude → gimbal → gnss → IMAGE LAST. The image channel triggers the
        // processing cycle downstream and reads the other three back out of the
        // extrapolator at this very timestamp, so their samples must already be
        // buffered when it fires.
        //
        // No telemetry ⇒ ONLY the image channel: the three others carry no
        // measurement for this instant and a source must not invent one.
        if (fd.has_telemetry) {
            AttitudeData att;
            att.roll_deg  = fd.telemetry.roll_deg;
            att.pitch_deg = fd.telemetry.pitch_deg;
            att.yaw_deg   = fd.telemetry.heading_deg;
            attitude_channel_(t_msec, att);

            GimbalData gim;
            gim.pan_deg  = fd.telemetry.gimbal_pan_deg;
            gim.tilt_deg = fd.telemetry.gimbal_tilt_deg;
            gimbal_channel_(t_msec, gim);

            GnssData gnss;
            gnss.latitude_deg  = fd.telemetry.latitude_deg;
            gnss.longitude_deg = fd.telemetry.longitude_deg;
            gnss.altitude_m    = fd.telemetry.altitude_m;
            gnss.speed_mps     = fd.telemetry.speed_mps;
            gnss_channel_(t_msec, gnss);
        }

        ImageData img;
        img.frame_id  = fd.frame_id;
        img.image     = fd.image;
        img.camera_id = fd.camera_id;

        last_frame_id_.store(fd.frame_id);
        frames_published_.fetch_add(1);
        image_channel_(t_msec, img);

        if (config_.inter_frame_delay_ms > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.inter_frame_delay_ms));
        }
    }

    // Whether the loop ended on EOS or on stopStreaming(), the source is no
    // longer producing — isStreaming() must say so without waiting for a join.
    streaming_.store(false);
}

}  // namespace sensor
}  // namespace uavloc
