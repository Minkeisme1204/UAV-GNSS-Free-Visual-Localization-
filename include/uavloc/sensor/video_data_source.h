#pragma once

//! sensor::VideoDataSource — push-style adapter over a pull-style
//! sensor::DataInterface (S6a, .docs/designs/system_manager_design.md §7-S6a).
//!
//! It owns a reader thread that loops exactly like kcb's
//! `PCSDataAdapter::processing()`: read one sample, then fan it out on the four
//! typed channels of DataSourceInterface — attitude → gimbal → gnss → IMAGE,
//! image last, all with the same timestamp.
//!
//! Named after the video-file case it was written for, but it wraps ANY
//! DataInterface (VideoReader, CameraStreamer, a test double), because that is
//! the only API it uses.
//!
//! Note what this does and does not buy, per §3.1 of the design: publishing four
//! channels off ONE telemetry row does not by itself make the pipeline
//! multi-rate — every sample carries the same timestamp on all four channels.
//! It creates the JOINT that a genuinely multi-rate rig (camera + separate AHRS)
//! plugs into later; the interpolation lives in core::Extrapolator (S6b).

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "uavloc/sensor/data_interface.h"
#include "uavloc/sensor/data_source_interface.h"

namespace uavloc {
namespace sensor {

struct VideoDataSourceConfig {
    //! End-of-stream guard (design §3.6, problem P2-1). A cut container keeps
    //! answering EMPTY_FRAME forever instead of END_OF_STREAM, so every driver
    //! had to carry its own guard. This many CONSECUTIVE non-OK reads makes the
    //! source publish StreamEventKind::END_OF_STREAM and stop. The default
    //! matches the hand-rolled guard the drivers used (test_full_flight,
    //! test_system_manager: MAX_CONSECUTIVE_BAD_READS = 100).
    unsigned int max_consecutive_bad_reads = 100;

    //! Pause between two reads [ms]. 0 = replay as fast as the reader allows,
    //! which is what offline evaluation wants. A non-zero value throttles a
    //! file source to something closer to real time (kcb hard-codes 50 ms).
    unsigned int inter_frame_delay_ms = 0;

    //! Reads "VideoDataSource:" — every key optional, missing keys keep the
    //! defaults above.
    static VideoDataSourceConfig fromYaml(const YAML::Node& node);
};

class VideoDataSource final : public DataSourceInterface {
public:
    //! Takes ownership of `source`. A null source is accepted (construction
    //! never throws) but startStreaming() then fails.
    explicit VideoDataSource(std::unique_ptr<DataInterface> source,
                             const VideoDataSourceConfig&   config = {});
    ~VideoDataSource() override;

    bool startStreaming() override;
    bool stopStreaming()  override;
    bool isStreaming() const override;

    //! frames published / total frames of the wrapped reader, or NaN when the
    //! total is unknown (live camera, or a reader that does not report one).
    //! Approximate for a source configured with start_frame/end_frame: the
    //! denominator is the whole container.
    double completion() const override;

    //! Frames published on the image channel so far.
    uint64_t framesPublished() const;

    std::string name() const;

private:
    void processing();
    void publishEvent(StreamEventKind kind, const std::string& message);

    std::unique_ptr<DataInterface> source_;
    VideoDataSourceConfig          config_;

    std::atomic<bool>            streaming_{false};
    std::unique_ptr<std::thread> thread_;
    mutable std::mutex           lifecycle_mutex_;

    std::atomic<uint64_t> frames_published_{0};
    std::atomic<uint64_t> last_frame_id_{0};
    //! Frame count of the wrapped reader, 0 = unknown. Set in startStreaming()
    //! before the thread spawns; atomic because completion() may be called from
    //! any thread, including a channel callback.
    std::atomic<int>      total_frames_{0};
};

}  // namespace sensor
}  // namespace uavloc
