#include "uavloc/core/system_manager.h"

#include "uavloc/anchor/anchor_query.h"
#include "uavloc/core/extrapolator.h"
#include "uavloc/core/rate_estimator.h"
#include "uavloc/fusion/fusion_module.h"
#include "uavloc/sensor/geo_reference.h"
#include "uavloc/sensor/telemetry_data.h"
#include "uavloc/util/scoped_timer.h"
#include "uavloc/util/threadsafe_queue.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <utility>

namespace uavloc {
namespace core {

namespace {

const char* state_name(SystemState s) {
    switch (s) {
        case SystemState::CREATED:  return "CREATED";
        case SystemState::RUNNING:  return "RUNNING";
        case SystemState::STOPPING: return "STOPPING";
        case SystemState::STOPPED:  return "STOPPED";
    }
    return "UNKNOWN";
}

const char* policy_name(InputPolicy p) {
    switch (p) {
        case InputPolicy::BLOCK:       return "BLOCK";
        case InputPolicy::DROP_OLDEST: return "DROP_OLDEST";
    }
    return "UNKNOWN";
}

const char* stream_event_name(sensor::StreamEventKind k) {
    switch (k) {
        case sensor::StreamEventKind::END_OF_STREAM: return "END_OF_STREAM";
        case sensor::StreamEventKind::READ_ERROR:    return "READ_ERROR";
        case sensor::StreamEventKind::DISCONNECTED:  return "DISCONNECTED";
        case sensor::StreamEventKind::TIMEOUT:       return "TIMEOUT";
    }
    return "UNKNOWN";
}

//! Wrap an angle into (-180, 180] [deg].
//!
//! ── WHY THIS RANGE, AND WHY ONLY ON INTERPOLATED SAMPLES ─────────────────────
//! sensor::TelemetryData documents heading as "0 = North, clockwise" but pins NO
//! range, and the two producers in this repo genuinely disagree: the MUN-FRL
//! telemetry CSVs carry yaw in [0, 360) (measured: 0 rows outside it on
//! dataset3 and dataset6) while the YenBai drone log carries it in (-180, 180]
//! (measured: 47202 of 91741 rows are negative). So there is no single "the"
//! range a consumer could demand.
//!
//! Nor does one exist downstream. src/fusion/factors.h uses heading (and gimbal
//! pan) in exactly two places, both invariant to a 360° shift:
//!   * rotation_enu_camera() feeds them straight into Eigen::AngleAxisd, i.e.
//!     into sin/cos;
//!   * the delta-yaw measurement is wrap_angle(psi_k - psi_{k-1}) in
//!     fusion_module.cpp, a wrapped DIFFERENCE.
//! Nothing else reads the field except the viewer HUD (display only).
//!
//! What DOES have a range is core::Extrapolator's INTERPOLATED output: it comes
//! out of a quaternion round trip, hence in (-180, 180]. An EXACT hit is a
//! verbatim copy of the producer's sample (rule ①) and MUST stay untouched —
//! normalising it to [0, 360) would rewrite YenBai's angles, and normalising it
//! to (-180, 180] would rewrite MUN-FRL's yaw = 359 into -1. Either rewrite
//! shifts a sine/cosine by an ULP and moves the fused numbers, which is exactly
//! what the S6c bit-identical gate forbids.
//!
//! Hence: the assembly step normalises the INTERPOLATED path only, to the range
//! the extrapolator already documents for it. The call is a no-op on today's
//! implementation on purpose — it pins the invariant at the point of
//! consumption instead of trusting a distant header comment.
double wrap180_deg(double deg) {
    if (!std::isfinite(deg)) {
        return deg;
    }
    double d = std::fmod(deg + 180.0, 360.0);
    if (d <= 0.0) {
        d += 360.0;
    }
    return d - 180.0;
}

//! |sin(pitch)| beyond which the ZYX decomposition is in gimbal lock and yaw /
//! roll are no longer separable (~89.99°). Numerical guard, not a tunable: it
//! only selects the degenerate branch of the formula below.
constexpr double GIMBAL_LOCK_SIN_PITCH = 0.99999999;

constexpr double RAD2DEG = 180.0 / M_PI;

//! ZYX (yaw → pitch → roll) Euler decomposition of a rotation matrix, in
//! degrees. Ranges: yaw/roll in (-180, 180], pitch in [-90, 90]. In gimbal lock
//! yaw is pinned to 0 and the whole rotation is expressed through roll.
void euler_zyx_deg(const Eigen::Matrix3d& R, float& roll_deg, float& pitch_deg,
                   float& yaw_deg) {
    const double sin_pitch = std::max(-1.0, std::min(1.0, -R(2, 0)));
    const double pitch     = std::asin(sin_pitch);

    double yaw  = 0.0;
    double roll = 0.0;
    if (std::abs(sin_pitch) >= GIMBAL_LOCK_SIN_PITCH) {
        roll = std::atan2(-R(1, 2), R(1, 1));
    } else {
        yaw  = std::atan2(R(1, 0), R(0, 0));
        roll = std::atan2(R(2, 1), R(2, 2));
    }

    roll_deg  = static_cast<float>(roll * RAD2DEG);
    pitch_deg = static_cast<float>(pitch * RAD2DEG);
    yaw_deg   = static_cast<float>(yaw * RAD2DEG);
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

namespace {

//! Non-null while THIS thread is inside SystemManager::Impl::processFrame() of
//! that instance. Read by the anchor result handler to decide whether it is
//! being re-entered from inside the pipeline (synchronous producer, gate
//! already held by onFrame) or called from a producer's own thread (gate not
//! held). Taking the shared gate a SECOND time on the same thread can deadlock
//! against a stop() waiting for it exclusively, so the distinction has to be
//! made, and a thread_local marker is the only way to make it without adding a
//! second gate.
thread_local const void* g_in_pipeline_impl = nullptr;

//! RAII setter for the marker above.
class PipelineScope {
public:
    explicit PipelineScope(const void* impl) : prev_(g_in_pipeline_impl) {
        g_in_pipeline_impl = impl;
    }
    ~PipelineScope() { g_in_pipeline_impl = prev_; }

    PipelineScope(const PipelineScope&)            = delete;
    PipelineScope& operator=(const PipelineScope&) = delete;

private:
    const void* prev_;
};

} // namespace

class SystemManager::Impl {
public:
    explicit Impl(const SystemConfig& config)
        : config_(config), rate_(config.stats_fps_window_sec) {}

    ~Impl() { stop(); }

    // ── lifecycle ────────────────────────────────────────────────────────────

    bool setup() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (setup_done_) {
            spdlog::error("SystemManager::setup: already set up — ignoring");
            return false;
        }
        if (state_.load() != SystemState::CREATED) {
            spdlog::error("SystemManager::setup: cannot set up in state {}",
                          state_name(state_.load()));
            return false;
        }
        // Degenerate intrinsics would silently produce wrong poses instead of
        // failing, so refuse them here (SystemConfig::fromYaml already throws
        // when the "Camera:" section is missing altogether).
        const auto& cam = config_.camera;
        if (!(cam.fx > 0.0 && cam.fy > 0.0 && cam.width > 0 && cam.height > 0)) {
            spdlog::error("SystemManager::setup: invalid camera intrinsics "
                          "(fx={}, fy={}, {}x{})",
                          cam.fx, cam.fy, cam.width, cam.height);
            return false;
        }

        try {
            vo_ = std::make_unique<vo::VOModule>(config_.vo);
            if (config_.enable_fusion) {
                fusion_ = std::make_unique<fusion::FusionModule>(config_.fusion);
            }
        } catch (const std::exception& ex) {
            spdlog::error("SystemManager::setup: building the pipeline failed: {}",
                          ex.what());
            vo_.reset();
            fusion_.reset();
            return false;
        }

        // VOModule fires its data-out callbacks at the END of process_frame(),
        // on the caller's thread. Cache the payload instead of publishing from
        // there so onFrame() can keep the documented publication order
        // (debugFrame before debugVoData, design §4.6).
        vo_->add_data_out_callback([this](const vo::VOData& data) {
            last_vo_data_ = data;
            have_vo_data_ = true;
        });

        if (fusion_) {
            // Registered ONCE here rather than polling latest() after each
            // push: a callback stays correct when the fusion back-end runs on
            // its own thread, where "the newest result" is not necessarily the
            // one produced by the push we just made.
            fusion_->add_result_callback(
                [this](const fusion::FusionResult& res) { onFusionResult(res); });
        }

        // The input queue only exists in async mode: its mere absence is what
        // makes the synchronous path bit-for-bit the S3/S4 path.
        if (config_.async_input) {
            std::size_t capacity = config_.input_queue_capacity;
            if (capacity == SystemConfig::UNBOUNDED_INPUT_QUEUE_CAPACITY) {
                // SystemConfig::fromYaml() already rewrites 0, but a config
                // built in code can still carry it. An unbounded input queue
                // disables backpressure altogether (no push ever waits, nothing
                // is ever dropped) and lets memory grow without bound, so it is
                // corrected rather than honoured.
                capacity = SystemConfig::DEFAULT_INPUT_QUEUE_CAPACITY;
                spdlog::warn("SystemManager::setup: input_queue_capacity 0 "
                             "(unbounded) is not allowed — using {}",
                             capacity);
            }
            input_queue_ =
                std::make_unique<util::ThreadsafeQueue<sensor::FrameData>>(capacity);
        }

        // The absolute-position producer, if one was attached. Its result sink
        // is registered BEFORE its setup() so a producer that starts working
        // during setup cannot lose a fix.
        if (anchor_) {
            anchor_->setResultCallback(
                [this](const anchor::AbsoluteFix& fix) { onAnchorFix(fix); });
            if (!anchor_->setup()) {
                spdlog::error("SystemManager::setup: the attached anchor refused "
                              "to set up — refusing to run WITHOUT absolute "
                              "positions (a silent run would look like a valid "
                              "experiment)");
                vo_.reset();
                fusion_.reset();
                return false;
            }
            if (!config_.enable_geo) {
                // setEnuOrigin() is only ever called from the geo-anchoring
                // branch of onFusionResult(), so with geo disabled the producer
                // would never learn the frame its fixes must be expressed in
                // and would (correctly) emit nothing.
                spdlog::warn("SystemManager::setup: an anchor is attached but "
                             "enable_geo is false — no ENU origin will ever be "
                             "handed to it, so NO fix can be produced");
            }
        }

        // The telemetry time-buffer of the four-channel path (design §3.3).
        // Built unconditionally: it costs nothing when only onFrame() is used,
        // and it must exist before the first onAttitude() a source can fire.
        extrapolator_ = std::make_unique<Extrapolator>(
            config_.extrapolator_buffer_span_sec, config_.extrapolator_max_gap_sec);

        setup_done_ = true;
        spdlog::info("SystemManager: set up (fusion={}, geo={}, publish_images={}, "
                     "async_input={}, policy={}, queue_capacity={}, "
                     "extrapolator_span={}s, max_gap={}s)",
                     config_.enable_fusion, config_.enable_geo,
                     config_.publish_images, config_.async_input,
                     policy_name(config_.input_policy),
                     input_queue_ ? input_queue_->capacity() : 0,
                     config_.extrapolator_buffer_span_sec,
                     config_.extrapolator_max_gap_sec);
        return true;
    }

    // ── source ───────────────────────────────────────────────────────────────

    void attachSource(std::unique_ptr<sensor::DataSourceInterface> source) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (state_.load() != SystemState::CREATED) {
            spdlog::error("SystemManager::attachSource: refused in state {} — attach "
                          "before start()", state_name(state_.load()));
            return;  // `source` is destroyed here; nothing was subscribed
        }
        {
            std::lock_guard<std::mutex> slock(source_mutex_);
            if (source_) {
                spdlog::warn("SystemManager::attachSource: replacing the attached source");
                source_->clearCallbacks();
            }
            source_ = std::move(source);
        }
        if (!source_) {
            return;
        }
        // Subscribe in the order the channels fire for one sample, so the
        // wiring reads like the contract it implements. Every lambda captures
        // `this` only: SystemManager outlives the source (stop() unsubscribes
        // and joins it before anything is torn down).
        source_->attitudeChannel().add(
            [this](double t, const sensor::AttitudeData& a) { onAttitude(t, a); });
        source_->gimbalChannel().add(
            [this](double t, const sensor::GimbalData& g) { onGimbal(t, g); });
        source_->gnssChannel().add(
            [this](double t, const sensor::GnssData& g) { onGnss(t, g); });
        source_->imageChannel().add(
            [this](double t, const sensor::ImageData& i) { onImage(t, i); });
        source_->eventChannel().add(
            [this](const sensor::StreamEvent& e) { onStreamEvent(e); });
        spdlog::info("SystemManager: source attached (5 channels subscribed)");
    }

    double sourceCompletion() const {
        std::lock_guard<std::mutex> lock(source_mutex_);
        return source_ ? source_->completion()
                       : std::numeric_limits<double>::quiet_NaN();
    }

    //! Turn the source's reading thread off (pause) or on (resume) WITHOUT
    //! touching state_ or any pipeline component — the one thing that separates
    //! a pause from stop(). Shares stop()'s lock so the two can never interleave
    //! on the same source; lifecycle_mutex_ is never taken on the data path, so
    //! joining the reader while holding it is safe (same argument as stop()).
    bool setSourceStreaming(bool want_streaming) {
        const char* verb = want_streaming ? "resumeSource" : "pauseSource";
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (state_.load() != SystemState::RUNNING) {
            spdlog::warn("SystemManager::{}: refused in state {} — only a RUNNING "
                         "system can pause/resume its input", verb,
                         state_name(state_.load()));
            return false;
        }
        sensor::DataSourceInterface* src = rawSource();
        if (src == nullptr) {
            spdlog::warn("SystemManager::{}: no source attached", verb);
            return false;
        }
        if (source_paused_ == !want_streaming) {  // already in the wanted state
            spdlog::debug("SystemManager::{}: source already {}", verb,
                          want_streaming ? "streaming" : "paused");
            return true;
        }
        // The callbacks stay subscribed and the source stays OPEN, so a file
        // source is not rewound: the reader picks up at the next frame.
        const bool ok = want_streaming ? src->startStreaming() : src->stopStreaming();
        if (!ok) {
            spdlog::error("SystemManager::{}: the source refused to {} — nothing "
                          "changed", verb, want_streaming ? "start" : "stop");
            return false;
        }
        source_paused_ = !want_streaming;
        spdlog::info("SystemManager: source {} (system stays RUNNING)",
                     want_streaming ? "RESUMED" : "PAUSED");
        return true;
    }

    bool isSourcePaused() const {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        return source_paused_;
    }

    void setAnchor(std::unique_ptr<anchor::AnchorInterface> anchor) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (state_.load() != SystemState::CREATED || setup_done_) {
            spdlog::error("SystemManager::setAnchor: refused in state {} "
                          "(setup_done={}) — attach before setup()",
                          state_name(state_.load()), setup_done_);
            return;  // `anchor` is destroyed here; nothing was registered
        }
        if (anchor_) {
            spdlog::warn("SystemManager::setAnchor: replacing the attached anchor");
            anchor_->stop();
        }
        anchor_ = std::move(anchor);
        spdlog::info("SystemManager: anchor attached (absolute-position producer)");
    }

    bool start() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!setup_done_) {
            spdlog::error("SystemManager::start: setup() has not succeeded");
            return false;
        }
        if (state_.load() != SystemState::CREATED) {
            spdlog::error("SystemManager::start: already started (state {})",
                          state_name(state_.load()));
            return false;
        }
        // Order of design §4.4: back-end first, then our own threads, then
        // (S6) the source. Each step unwinds the previous ones on failure.
        try {
            if (fusion_) {
                fusion_->start();  // no-op when fusion runs synchronously
            }
        } catch (const std::exception& ex) {
            spdlog::error("SystemManager::start: fusion start failed: {}", ex.what());
            return false;
        }
        // The producer goes up right after its consumer, so a fix can never
        // arrive before the back-end that has to swallow it exists.
        if (anchor_ && !anchor_->start()) {
            spdlog::error("SystemManager::start: the attached anchor refused to start");
            if (fusion_) {
                fusion_->stop();
            }
            return false;
        }
        try {
            // Spawned BEFORE the state flips: the processing thread parks in
            // pop_blocking() and the monitor thread in its timed wait, so
            // neither can miss anything by starting early.
            if (input_queue_) {
                proc_thread_ = std::thread([this] { processingLoop(); });
            }
            if (config_.stats_period_ms > 0) {
                monitor_thread_ = std::thread([this] { monitorLoop(); });
            }
        } catch (const std::system_error& ex) {
            spdlog::error("SystemManager::start: cannot spawn a thread: {}", ex.what());
            joinWorkerThreads();
            if (anchor_) {
                anchor_->stop();
            }
            if (fusion_) {
                fusion_->stop();
            }
            return false;
        }
        // Flipped BEFORE the source starts: the very first sample the source
        // publishes must find the system RUNNING, or it would be refused.
        // Nothing else can be in flight yet — an external producer racing a
        // start() is the caller's own ordering problem.
        state_.store(SystemState::RUNNING);

        // Step 3/4 of design §4.4: the source goes last, and its failure
        // unwinds every step above it.
        sensor::DataSourceInterface* src = rawSource();
        if (src != nullptr && !src->startStreaming()) {
            spdlog::error("SystemManager::start: the attached source refused to stream");
            state_.store(SystemState::CREATED);
            src->clearCallbacks();
            joinWorkerThreads();
            if (anchor_) {
                anchor_->stop();
            }
            if (fusion_) {
                fusion_->stop();
            }
            return false;
        }
        spdlog::info("SystemManager: RUNNING ({} input, source {})",
                     input_queue_ ? "async" : "inline",
                     src != nullptr ? "streaming" : "none");
        return true;
    }

    //! Teardown, reverse of start() (design §4.4). The concurrency contract it
    //! implements is documented on SystemManager::stop(); the mechanism is:
    //!
    //!   * `state_` (atomic) flips to STOPPING first, so onFrame() refuses from
    //!     that instant on — checked again UNDER the gate below, which is what
    //!     makes the refusal authoritative rather than a racy hint;
    //!   * the queue is shut down BEFORE the gate is taken, so a producer
    //!     parked in push_blocking_if_full() is woken and can release its
    //!     shared gate hold. Taking the gate first would let stop() wait on a
    //!     producer that is itself waiting for the consumer we are about to
    //!     stop;
    //!   * the exclusive gate acquisition is a BARRIER: it returns only once
    //!     every onFrame() that had already passed the state check has left the
    //!     pipeline. Nothing is torn down before that point;
    //!   * the processing thread is then joined. ThreadsafeQueue::pop_blocking()
    //!     keeps handing out queued items after shutdown() and only fails once
    //!     the queue is empty, so the join IS the drain — accepted frames are
    //!     never discarded. The worker threads deliberately do NOT touch the
    //!     gate, so joining them while holding it cannot deadlock.
    void stop() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (state_.load() != SystemState::RUNNING) {
            spdlog::debug("SystemManager::stop: state is {} — nothing to tear down",
                          state_name(state_.load()));
            return;
        }

        // 0. The SOURCE first, and its callbacks before its thread (design
        //    §4.4 step 1, from kcb's AhrsVprFusion::Stop): once the channels
        //    are cut nothing new can reach us, and stopStreaming() then joins
        //    the reader. Deliberately done while the state is still RUNNING, so
        //    a sample already inside onImage() is finished rather than
        //    refused — "no accepted frame is lost" starts here.
        //    The join happens WITHOUT source_mutex_ held: a channel callback
        //    that queries sourceCompletion() would otherwise deadlock the join.
        if (sensor::DataSourceInterface* src = rawSource()) {
            src->clearCallbacks();
            src->stopStreaming();
        }

        // 0b. The ANCHOR next, and for the same reason as the source: it is a
        //     producer with a thread of its own. Its stop() joins that thread,
        //     so once it returns no result callback can still be in flight —
        //     which is what lets step 5 tear the fusion back-end down. Done
        //     while the state is still RUNNING so a fix already in flight is
        //     delivered rather than refused.
        if (anchor_) {
            anchor_->stop();
        }

        // 1. New frames are refused from here on.
        state_.store(SystemState::STOPPING);

        // 2. Wake producers blocked on a full queue (queued items are kept).
        if (input_queue_) {
            input_queue_->shutdown();
        }

        // 3. Barrier: wait for the onFrame() calls already in flight.
        {
            std::unique_lock<std::shared_mutex> gate(gate_mutex_);
        }

        // 4. Drain + join the workers.
        joinWorkerThreads();

        // 5. Back-end last — nothing can be using it any more.
        if (fusion_) {
            fusion_->stop();
        }
        state_.store(SystemState::STOPPED);
        const SystemStats st = stats_snapshot();
        spdlog::info("SystemManager: STOPPED (received={}, processed={}, dropped={})",
                     st.frames_received, st.frames_processed, st.frames_dropped);
    }

    SystemState state() const { return state_.load(); }

    // ── input ────────────────────────────────────────────────────────────────

    bool onFrame(const sensor::FrameData& fd) {
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.frames_received;
        }
        // Cheap lock-free refusal FIRST. It is not authoritative (stop() may
        // win the race right after it), but it keeps a hot producer loop from
        // touching the gate at all once stop() has started — otherwise a
        // reader-preferring shared_mutex could starve stop()'s exclusive
        // acquisition indefinitely.
        if (state_.load() != SystemState::RUNNING) {
            spdlog::debug("SystemManager::onFrame: refused in state {}",
                          state_name(state_.load()));
            countDrop();
            return false;
        }

        // Gate held for the whole of the rest of this call. stop() cannot get
        // past its barrier while any of these holds exist, so nothing this
        // function uses can be torn down under it.
        std::shared_lock<std::shared_mutex> gate(gate_mutex_);

        // Authoritative re-check: stop() sets STOPPING BEFORE it takes the gate
        // exclusively, so seeing RUNNING here means stop() has not reached its
        // barrier yet and will therefore wait for this call.
        if (state_.load() != SystemState::RUNNING) {
            spdlog::debug("SystemManager::onFrame: refused in state {} (raced stop)",
                          state_name(state_.load()));
            countDrop();
            return false;
        }
        if (!fd.valid || !fd.HasImage()) {
            spdlog::debug("SystemManager::onFrame: frame {} carries no image",
                          fd.frame_id);
            countDrop();
            return false;
        }

        if (input_queue_) {
            // ASYNC: hand the frame over and return. cv::Mat is refcounted, so
            // the copy into the queue costs a header, not the pixels.
            const bool accepted =
                (config_.input_policy == InputPolicy::BLOCK)
                    ? input_queue_->push_blocking_if_full(
                          fd, std::chrono::milliseconds(config_.push_timeout_ms))
                    : input_queue_->push_dropping_if_full(fd);
            if (!accepted) {
                // BLOCK      : deadline expired (or shutting down) — nothing queued.
                // DROP_OLDEST: an older frame was discarded (or shutting down).
                // Either way exactly one frame was lost, and the producer is
                // told so; see the note on SystemManager::onFrame().
                countDrop();
                return false;
            }
            return true;
        }

        processFrame(fd);
        return true;
    }

    // ── the four typed channels (design §3.1 / §4.8) ─────────────────────────
    //
    // The three non-image channels only FILL the extrapolator. They take no
    // gate: `extrapolator_` is created by setup() and released only when Impl
    // is destroyed, so unlike the pipeline components it cannot be torn down
    // under a concurrent call. The RUNNING check is therefore a policy (do not
    // buffer for a system that is not accepting frames), not a safety barrier.

    bool onAttitude(double t_msec, const sensor::AttitudeData& att) {
        if (!channelsReady("onAttitude")) {
            return false;
        }
        extrapolator_->addAttitude(t_msec, att);
        return true;
    }

    bool onGimbal(double t_msec, const sensor::GimbalData& gim) {
        if (!channelsReady("onGimbal")) {
            return false;
        }
        extrapolator_->addGimbal(t_msec, gim);
        return true;
    }

    bool onGnss(double t_msec, const sensor::GnssData& gnss) {
        if (!channelsReady("onGnss")) {
            return false;
        }
        extrapolator_->addGnss(t_msec, gnss);
        return true;
    }

    //! THE ASSEMBLY STEP — kcb's SynchronizeData(). Note what it does NOT do:
    //! it does not run any pipeline stage itself. It rebuilds a FrameData and
    //! hands it to onFrame(), the same entry point the pre-assembled path uses.
    //! One pipeline, two front doors; that is what makes the two paths
    //! bit-identical instead of merely similar.
    bool onImage(double t_msec, const sensor::ImageData& img) {
        if (!channelsReady("onImage")) {
            // Counted as a received+dropped frame by onFrame() below in every
            // other refusal path; do it here too so the books balance.
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.frames_received;
                ++stats_.frames_dropped;
            }
            return false;
        }

        sensor::FrameData fd;
        fd.frame_id       = img.frame_id;
        fd.timestamp_msec = t_msec;
        fd.image          = img.image;  // cv::Mat is refcounted; no pixel copy
        fd.camera_id      = img.camera_id;
        fd.status         = sensor::FrameStatus::OK;
        fd.valid          = true;

        sensor::AttitudeData att;
        sensor::GimbalData   gim;
        sensor::GnssData     gnss;
        SampleOrigin att_origin = SampleOrigin::EXACT;
        SampleOrigin gim_origin = SampleOrigin::EXACT;

        const bool have_att  = extrapolator_->getAttitude(t_msec, att, &att_origin);
        const bool have_gim  = extrapolator_->getGimbal(t_msec, gim, &gim_origin);
        const bool have_gnss = extrapolator_->getGnss(t_msec, gnss);

        if (have_att && have_gim && have_gnss) {
            sensor::TelemetryData& t = fd.telemetry;
            t.frame_id       = img.frame_id;
            t.timestamp_msec = t_msec;
            t.roll_deg       = att.roll_deg;
            t.pitch_deg      = att.pitch_deg;
            // Angle-range normalisation, INTERPOLATED PATH ONLY — see
            // wrap180_deg() for the measurements this rule rests on.
            t.heading_deg    = (att_origin == SampleOrigin::INTERPOLATED)
                                   ? wrap180_deg(att.yaw_deg)
                                   : att.yaw_deg;
            t.gimbal_pan_deg = (gim_origin == SampleOrigin::INTERPOLATED)
                                   ? wrap180_deg(gim.pan_deg)
                                   : gim.pan_deg;
            t.gimbal_tilt_deg = gim.tilt_deg;
            t.latitude_deg    = gnss.latitude_deg;
            t.longitude_deg   = gnss.longitude_deg;
            t.altitude_m      = gnss.altitude_m;
            t.speed_mps       = gnss.speed_mps;
            // climb_mps stays 0: no channel carries it (see stream_types.h) and
            // nothing reads it. Inventing a value would be worse than a zero
            // that is documented as "not measured".
            t.valid          = true;
            fd.has_telemetry = true;
        } else {
            // Not all three channels cover this instant. Exactly what
            // VideoReader does when the CSV has no row for a frame:
            // has_telemetry = false, telemetry left default. NO invention.
            spdlog::debug("SystemManager::onImage: frame {} at {:.3f} ms has no "
                          "complete state (attitude={}, gimbal={}, gnss={}) — "
                          "forwarded without telemetry",
                          img.frame_id, t_msec, have_att, have_gim, have_gnss);
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.frames_without_telemetry;
        }

        return onFrame(fd);
    }

    void onStreamEvent(const sensor::StreamEvent& event) {
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            switch (event.kind) {
                case sensor::StreamEventKind::END_OF_STREAM:
                case sensor::StreamEventKind::DISCONNECTED:
                    stats_.source_ended = true;
                    break;
                case sensor::StreamEventKind::READ_ERROR:
                case sensor::StreamEventKind::TIMEOUT:
                    ++stats_.stream_errors;
                    break;
            }
        }
        spdlog::info("SystemManager: stream event {} at frame {} ('{}')",
                     stream_event_name(event.kind), event.frame_id, event.message);
        // The system deliberately does NOT stop itself: the owner decides when
        // to tear down (design §4.5). Publishing the counters is how the owner
        // finds out, without having to poll.
        callbacks_.on_stats(stats_snapshot());
    }

    //! The pipeline itself, in the exact order of design §4.6. Runs on the
    //! caller's thread in synchronous mode, on the processing thread otherwise.
    //! Serialized: `last_vo_data_` and the VO front-end are single-writer, and
    //! two producers calling onFrame() at once must not interleave here.
    void processFrame(const sensor::FrameData& fd) {
        std::lock_guard<std::mutex> proc_lock(process_mutex_);

        // Marks this thread as "inside the pipeline" for the whole call: see
        // onAnchorFix() for the one decision that depends on it.
        PipelineScope pipeline_scope(this);

        const auto t0 = std::chrono::steady_clock::now();

        // 1. VO front-end — exactly the call the offline driver makes.
        have_vo_data_          = false;
        const vo::VOResult res = vo_->process_frame(fd);

        // 2. on_frame_processed. Built only when someone listens: it copies the
        //    tracked observations (and, if publish_images, keeps the image
        //    buffer alive).
        if (!callbacks_.on_frame_processed.empty()) {
            FrameProcessed fp;
            fp.frame_id       = res.frame_id;
            fp.timestamp_msec = res.timestamp_msec;
            if (config_.publish_images) {
                fp.image = fd.image;  // cv::Mat is refcounted; no pixel copy
            }
            fp.num_tracked   = static_cast<int>(res.tracked_observations.size());
            fp.num_landmarks = res.num_landmarks;
            fp.num_inliers   = res.num_inliers;
            fp.tracked_observations = res.tracked_observations;
            // The telemetry the pipeline saw — on the four-channel path this is
            // what the assembly step rebuilt, so this channel is where that
            // step becomes observable.
            fp.telemetry     = fd.telemetry;
            fp.has_telemetry = fd.has_telemetry;
            callbacks_.on_frame_processed(fp.timestamp_msec, fp);
        }

        // 3. on_vo_data — the payload VOModule built during step 1.
        if (have_vo_data_) {
            callbacks_.on_vo_data(last_vo_data_.result.timestamp_msec, last_vo_data_);
        }

        // 4. Fusion back-end. Telemetry is passed EXACTLY as the offline driver
        //    passes it: a default-constructed record when the frame has none.
        if (fusion_) {
            const sensor::TelemetryData telem =
                fd.has_telemetry ? fd.telemetry : sensor::TelemetryData{};
            {
                std::lock_guard<std::mutex> lock(geo_mutex_);
                frame_telemetry_       = telem;
                frame_telemetry_valid_ = fd.has_telemetry && telem.altitude_m > 0.0;
            }
            // Steps 5 and 6 (LocalizationOutput + on_lag_window) happen inside
            // onFusionResult(), which push() invokes — inline here in
            // synchronous fusion mode, on the fusion thread otherwise.
            util::ScopedTimer _t(util::ProfileStage::FUSION_PUSH);
            fusion_->push(res, telem);
        }

        // 6a2. VO state transitions (lost / re-init). Evaluated HERE, before the
        //      anchor request below, because that request has to know whether
        //      this frame is the one the VO chain came back on. It is the SAME
        //      and ONLY accounting that feeds stats_.reinit_events — a second
        //      detector would eventually disagree with the first, silently.
        const bool reinit_now = noteVoState(res);

        // 6b. Ask the absolute-position producer for a fix — on keyframes,
        //     because that is the cadence a place-recognition front-end can
        //     sustain and the instant the back-end has a state X(k) to attach a
        //     fix to, AND on the frame a re-initialized VO starts tracking
        //     again, whether or not it is a keyframe: the relative chain just
        //     restarted from an unconstrained frame, so this is where an
        //     absolute measurement is worth the most. AFTER the fusion push on
        //     purpose: the prediction the query carries is then the estimate
        //     INCLUDING this keyframe's smoother update, which is what a real
        //     producer would search with. Whether a request actually yields a
        //     fix is entirely the producer's decision (see
        //     AnchorInterface::requestFix).
        //
        //     A frame that is BOTH a keyframe and the re-init frame sends ONE
        //     request, labelled REINIT: the event outranks the cadence, and two
        //     requests for one instant would double-count on the producer side.
        if (anchor_ && (res.is_keyframe || reinit_now)) {
            anchor::AnchorQuery q;
            q.reason = reinit_now ? anchor::AnchorRequestReason::REINIT
                                  : anchor::AnchorRequestReason::KEYFRAME;
            q.timestamp_msec = res.timestamp_msec;
            q.frame_id       = res.frame_id;
            q.image          = fd.image;  // cv::Mat is refcounted; no pixel copy
            {
                std::lock_guard<std::mutex> lock(output_mutex_);
                if (last_fusion_valid_ && last_fusion_.has_pose) {
                    q.xy_enu_pred = last_fusion_.T_enu_c.translation().head<2>();
                    q.cov_pred    = last_fusion_.xy_covariance;
                    q.prediction_valid = last_fusion_.covariance_valid;
                }
            }
            // Exactly the telemetry the pipeline saw — agl_m <= 0 is how a
            // frame without telemetry announces itself, and nothing is invented.
            q.agl_m           = fd.has_telemetry ? fd.telemetry.altitude_m : 0.0;
            q.yaw_deg         = fd.has_telemetry ? fd.telemetry.heading_deg : 0.0;
            q.gimbal_pan_deg  = fd.has_telemetry ? fd.telemetry.gimbal_pan_deg : 0.0;
            q.gimbal_tilt_deg = fd.has_telemetry ? fd.telemetry.gimbal_tilt_deg : 0.0;
            // Full attitude, not just heading: a BEV-rectifying consumer needs
            // roll and pitch too, and they are right here in the same struct.
            q.roll_deg        = fd.has_telemetry ? fd.telemetry.roll_deg : 0.0;
            q.pitch_deg       = fd.has_telemetry ? fd.telemetry.pitch_deg : 0.0;
            const char* reason_str =
                q.reason == anchor::AnchorRequestReason::REINIT ? "REINIT"
                                                               : "KEYFRAME";
            // Stage 1 of the three-stage absolute-measurement trace
            // (anchor[REQ] → anchor[EMIT] → anchor[APPLY]). `info` on purpose:
            // one line per REQUEST, i.e. per keyframe / re-init frame, not per
            // frame. `frame` + `ts` are the correlation keys across the three
            // stages.
            //
            // Printed BEFORE the call, not after: a SYNCHRONOUS producer runs
            // the whole generation — and, through the result callback, the
            // back-end intake — inside requestFix(), so a line printed after
            // the call would appear BELOW the EMIT and APPLY lines it caused
            // and read like an out-of-order log.
            spdlog::info("anchor[REQ] frame={} ts={:.1f} reason={}",
                         q.frame_id, q.timestamp_msec, reason_str);
            // Acceptance is not production (see AnchorInterface::requestFix):
            // only a REFUSED query gets a second line, because that is the one
            // outcome the three stages below can never account for — nothing
            // downstream will ever mention this query again.
            if (!anchor_->requestFix(q)) {
                spdlog::info("anchor[REQ] frame={} ts={:.1f} reason={} "
                             "accepted=no — producer refused the query",
                             q.frame_id, q.timestamp_msec, reason_str);
            }
        }

        // 7. Counters. The monitor thread publishes them on its own cadence.
        //    The VO state machine was already accounted for at step 6a2.
        const double dt_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        updateStats(dt_sec);
    }

    bool pushAbsoluteFix(const anchor::AbsoluteFix& fix) {
        // Same gate as onFrame(), for the same reason: this call reaches into
        // the fusion back-end, so it must not be in flight while stop() shuts
        // that back-end down. See the comment on stop().
        if (state_.load() != SystemState::RUNNING) {
            spdlog::debug("SystemManager::pushAbsoluteFix: refused in state {}",
                          state_name(state_.load()));
            return false;
        }
        std::shared_lock<std::shared_mutex> gate(gate_mutex_);
        if (state_.load() != SystemState::RUNNING) {
            spdlog::debug("SystemManager::pushAbsoluteFix: refused in state {} "
                          "(raced stop)", state_name(state_.load()));
            return false;
        }
        if (!fusion_) {
            spdlog::debug("SystemManager::pushAbsoluteFix: fusion is disabled");
            return false;
        }
        fusion_->push_absolute_fix(fix);
        return true;
    }

    // ── output ───────────────────────────────────────────────────────────────

    SystemCallbacks& callbacks() { return callbacks_; }

    LocalizationOutput latest() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        return latest_output_;
    }

    SystemStats stats() const { return stats_snapshot(); }

    fusion::FusionFixStats fixStats() const {
        return fusion_ ? fusion_->fix_stats() : fusion::FusionFixStats{};
    }

private:
    // ── helpers ──────────────────────────────────────────────────────────────

    //! Common precondition of the four channels. Logs at debug: a source that
    //! keeps publishing after stop() would otherwise flood the log.
    bool channelsReady(const char* who) const {
        if (state_.load() != SystemState::RUNNING) {
            spdlog::debug("SystemManager::{}: refused in state {}", who,
                          state_name(state_.load()));
            return false;
        }
        if (!extrapolator_) {
            spdlog::error("SystemManager::{}: setup() has not run", who);
            return false;
        }
        return true;
    }

    //! Borrowed pointer to the attached source. Taken under source_mutex_ and
    //! returned raw ON PURPOSE, so callers can invoke a BLOCKING method
    //! (stopStreaming() joins a thread) without holding the mutex.
    sensor::DataSourceInterface* rawSource() const {
        std::lock_guard<std::mutex> lock(source_mutex_);
        return source_.get();
    }

    // ── worker threads ───────────────────────────────────────────────────────

    //! Body of the processing thread (async_input only). pop_blocking() keeps
    //! returning queued frames after shutdown() and only fails once the queue
    //! is empty, so this loop drains before it exits — that is the "no accepted
    //! frame is discarded" half of the stop() contract. It deliberately does
    //! NOT take the gate: stop() joins this thread while holding it.
    void processingLoop() {
        spdlog::debug("SystemManager: processing thread started");
        sensor::FrameData fd;
        while (input_queue_->pop_blocking(fd)) {
            processFrame(fd);
            fd.Reset();  // release the image buffer while waiting for the next
        }
        spdlog::debug("SystemManager: processing thread stopped");
    }

    //! Body of the monitor thread: publish SystemStats on a fixed cadence that
    //! does NOT depend on the processing load (design §4.3). Publishing happens
    //! with no lock held, so a subscriber may do anything except call back into
    //! SystemManager (design §R-c).
    void monitorLoop() {
        const auto period = std::chrono::milliseconds(config_.stats_period_ms);
        spdlog::debug("SystemManager: monitor thread started ({} ms)",
                      config_.stats_period_ms);
        std::unique_lock<std::mutex> lock(monitor_mutex_);
        while (!monitor_stop_) {
            // Predicate form: a spurious wakeup does not fake a tick, and a
            // stop request is honoured immediately instead of after the period.
            if (monitor_cv_.wait_for(lock, period, [this] { return monitor_stop_; })) {
                break;
            }
            lock.unlock();
            callbacks_.on_stats(stats_snapshot());
            lock.lock();
        }
        spdlog::debug("SystemManager: monitor thread stopped");
    }

    //! Stop and join both worker threads. Safe to call when they were never
    //! spawned. Must NOT be called while holding the gate exclusively unless
    //! the workers are gate-free — they are, on purpose.
    void joinWorkerThreads() {
        if (proc_thread_.joinable()) {
            if (input_queue_) {
                input_queue_->shutdown();  // idempotent; also covers the unwind path
            }
            proc_thread_.join();
        }
        if (monitor_thread_.joinable()) {
            {
                std::lock_guard<std::mutex> lock(monitor_mutex_);
                monitor_stop_ = true;
            }
            monitor_cv_.notify_all();
            monitor_thread_.join();
        }
    }

    // ── internals ────────────────────────────────────────────────────────────

    //! on_fusion_result → LocalizationOutput → on_localization → on_lag_window.
    //! Runs on the caller's thread in synchronous fusion mode, on the fusion
    //! thread otherwise.
    void onFusionResult(const fusion::FusionResult& res) {
        // Raw back-end result FIRST, so a subscriber observes it before the
        // production output derived from it. No payload has to be assembled
        // here (the result IS the payload), and an empty slot returns
        // immediately.
        callbacks_.on_fusion_result(res.timestamp_msec, res);

        // Newest raw back-end result — the prediction an AnchorQuery carries.
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            last_fusion_       = res;
            last_fusion_valid_ = true;
        }

        LocalizationOutput out;
        out.timestamp_msec = res.timestamp_msec;
        out.frame_id       = res.frame_id;
        out.health         = res.health;
        // S7: the ONE definition of the horizontal accuracy lives in the
        // fusion module (fusion::horizontal_accuracy_m); NaN here means "no
        // covariance", never "perfectly certain".
        out.accuracy_m =
            static_cast<float>(fusion::horizontal_accuracy_m(res));
        const bool accuracy_known = std::isfinite(out.accuracy_m);

        // Set inside the geo block below when THIS call chose the ENU origin,
        // so the anchor can be told about it AFTER geo_mutex_ is released — a
        // producer's setEnuOrigin() must never run under one of our mutexes.
        bool   geo_just_anchored = false;
        double anchored_lat = 0.0, anchored_lon = 0.0;

        if (res.has_pose) {
            euler_zyx_deg(res.T_enu_c.rotation(), out.roll, out.pitch, out.yaw);

            if (config_.enable_geo) {
                std::lock_guard<std::mutex> lock(geo_mutex_);
                if (!geo_.initialized() && frame_telemetry_valid_) {
                    // Anchor at the first fused pose that coincides with usable
                    // telemetry — the same instant the fusion graph anchors
                    // X(0), so ENU (0,0,·) maps back onto this lat/lon.
                    //
                    // alt0 = 0 ON PURPOSE — do NOT "fix" this to the telemetry
                    // altitude. The fused Z is ALREADY an AGL: the graph
                    // anchors X(0) at ENU (0, 0, agl_0) with agl_0 taken from
                    // sensor::TelemetryData::altitude_m, which is above ground
                    // level. Passing that altitude as alt0 as well would make
                    // latlon_from_enu() return agl_0 + agl_k, counting the
                    // anchor AGL twice. With alt0 = 0, LocalizationOutput::
                    // altitude_m is exactly the AGL the back-end estimated.
                    geo_.init(frame_telemetry_.latitude_deg,
                              frame_telemetry_.longitude_deg, 0.0, 0.0);
                    geo_just_anchored = true;
                    anchored_lat      = frame_telemetry_.latitude_deg;
                    anchored_lon      = frame_telemetry_.longitude_deg;
                    spdlog::info("SystemManager: geo anchored at frame {} "
                                 "(lat0={:.7f}, lon0={:.7f}, alt0=0 — "
                                 "altitude_m is AGL)",
                                 res.frame_id, frame_telemetry_.latitude_deg,
                                 frame_telemetry_.longitude_deg);
                }
                if (geo_.initialized()) {
                    // T_enu_c is ALREADY in ENU, so this is the pure inverse of
                    // enu() — never latlon(), which would apply R_enu_w again.
                    const sensor::LatLonAlt lla =
                        geo_.latlon_from_enu(res.T_enu_c.translation());
                    out.latitude   = lla.lat;
                    out.longitude  = lla.lon;
                    out.altitude_m = lla.alt;
                    // A position whose uncertainty cannot be quantified is not
                    // a usable fix for the outside world (system_types.h): the
                    // coordinates are still published, but flagged invalid.
                    out.valid      = accuracy_known;
                    if (!accuracy_known) {
                        spdlog::debug("SystemManager: frame {} has a fused pose "
                                      "but no covariance — output marked invalid",
                                      res.frame_id);
                    }
                }
            }
        }

        // THE one place the absolute-position producer learns which ENU frame
        // its fixes must be expressed in. Same instant the fusion graph anchors
        // X(0), so a fix at ENU (0, 0) means "here" for both sides — the whole
        // reason the origin is pushed DOWN instead of being read from the
        // producer's own config.
        if (geo_just_anchored && anchor_) {
            anchor_->setEnuOrigin(anchored_lat, anchored_lon);
        }

        {
            // latest() must already return this sample when a subscriber asks
            // for it, so the snapshot is stored BEFORE the slot fires — and the
            // lock is released first, because the slot must never be invoked
            // under one of our mutexes.
            std::lock_guard<std::mutex> lock(output_mutex_);
            latest_output_ = out;
        }
        callbacks_.on_localization(out);

        // The corrected window only changes at a smoother update, and copying
        // it is not free — skip it when nobody draws it.
        if (res.graph_updated && fusion_ && !callbacks_.on_lag_window.empty()) {
            callbacks_.on_lag_window(fusion_->getLagWindow());
        }
    }

    //! Sink of anchor::AnchorInterface — and DELIBERATELY nothing more than a
    //! hand-over to the back-end (kcb's satcom callback does exactly one
    //! Push()). Any work done here would run on the producer's thread and
    //! would couple its latency to the pipeline.
    void onAnchorFix(const anchor::AbsoluteFix& fix) {
        if (g_in_pipeline_impl == this) {
            // Synchronous producer, re-entered from processFrame(): the caller
            // already holds the teardown gate as a reader, so nothing can be
            // torn down under us and taking it a second time on this thread
            // could deadlock against a stop() waiting for it exclusively.
            if (fusion_) {
                fusion_->push_absolute_fix(fix);
            } else {
                spdlog::debug("SystemManager: anchor fix dropped — fusion is disabled");
            }
            return;
        }
        // Asynchronous producer, on its own thread: full gate, exactly like an
        // external caller of pushAbsoluteFix().
        pushAbsoluteFix(fix);
    }

    void countDrop() {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.frames_dropped;
    }

    //! Same lost / re-init accounting as tests/test_full_flight.cpp, so the
    //! packaged counters mean exactly what the driver's do. THE single detector:
    //! stats_.reinit_events and the REINIT anchor request are both derived from
    //! this one call, so they can never drift apart.
    //!
    //! Returns true on the frame the VO started TRACKING again after a
    //! re-initialization — i.e. exactly the frame stats_.reinit_events counts.
    //! The very first initialization of a run is NOT a re-init and returns
    //! false: there is no broken chain to re-anchor there.
    //!
    //! Called from processFrame() BEFORE the anchor request; must be called
    //! exactly once per processed frame.
    bool noteVoState(const vo::VOResult& res) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        bool reinit_now = false;

        if (res.state == vo::VOTrackingState::TRACKING) {
            if (!had_first_init_) {
                had_first_init_ = true;
            } else if (pending_reinit_) {
                ++stats_.reinit_events;
                pending_reinit_ = false;
                reinit_now      = true;
            }
        }
        if (res.state == vo::VOTrackingState::LOST &&
            prev_vo_state_ != vo::VOTrackingState::LOST) {
            ++stats_.lost_events;
        }
        if (had_first_init_ && res.state == vo::VOTrackingState::NOT_INITIALIZED) {
            pending_reinit_ = true;
        }
        prev_vo_state_ = res.state;
        return reinit_now;
    }

    //! Frame + throughput counters. The VO state machine is accounted for in
    //! noteVoState(), which runs earlier in the same frame.
    void updateStats(double dt_sec) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.frames_processed;
        proc_sec_total_ += dt_sec;
        stats_.fps_processed =
            (proc_sec_total_ > 0.0)
                ? static_cast<double>(stats_.frames_processed) / proc_sec_total_
                : 0.0;
        rate_.add(std::chrono::steady_clock::now(), dt_sec);
    }

    SystemStats stats_snapshot() const {
        // queue_depth is read OUTSIDE stats_mutex_ (the queue has its own lock)
        // so the two are only mutually consistent to within a frame — a
        // monitoring figure, not an invariant.
        const std::size_t depth = input_queue_ ? input_queue_->size() : 0;
        std::lock_guard<std::mutex> lock(stats_mutex_);
        SystemStats out  = stats_;
        out.queue_depth  = depth;
        out.state        = state_.load();

        // The windowed figures are derived at SNAPSHOT time on purpose, not
        // cached in updateStats(): a stalled pipeline stops calling
        // updateStats() altogether, so a cached value would freeze at its last
        // reading while this one decays to 0 as the window empties.
        const RateEstimator::Sample rs = rate_.snapshot(std::chrono::steady_clock::now());
        out.fps_windowed   = rs.fps;
        out.proc_ms_mean   = rs.proc_ms_mean;
        out.fps_window_sec = rate_.window_sec();
        return out;
    }

    // ── members ──────────────────────────────────────────────────────────────

    const SystemConfig config_;

    std::unique_ptr<vo::VOModule>          vo_;
    std::unique_ptr<fusion::FusionModule>  fusion_;

    //! Telemetry time-buffer of the four-channel path. Built by setup() and
    //! never released before ~Impl, which is what lets the buffering channels
    //! skip the teardown gate.
    std::unique_ptr<Extrapolator> extrapolator_;

    //! The attached push-style source. Guarded by source_mutex_, which is NEVER
    //! held across a blocking source call (see rawSource()).
    mutable std::mutex                           source_mutex_;
    std::unique_ptr<sensor::DataSourceInterface> source_;

    //! The attached absolute-position producer. Written only by setAnchor()
    //! (refused after setup()), so every later read is on an immutable pointer
    //! and needs no lock — the same discipline vo_ / fusion_ follow.
    std::unique_ptr<anchor::AnchorInterface> anchor_;

    // Lifecycle. state_ is atomic so onFrame()/state() read it without the
    // lifecycle mutex, which stop() holds while the back-end drains.
    mutable std::mutex       lifecycle_mutex_;
    std::atomic<SystemState> state_{SystemState::CREATED};
    bool                     setup_done_ = false;
    //! Input paused by setSourceStreaming(false). Guarded by lifecycle_mutex_,
    //! like every other lifecycle decision. NOT reset by stop(): once the system
    //! is down the flag is meaningless (pause/resume are refused anyway).
    bool                     source_paused_ = false;

    //! Shared = "an onFrame() call is inside the pipeline", exclusive = "stop()
    //! is tearing it down". See the comment on stop() for why this is a barrier
    //! and not a plain mutex, and why it must be taken AFTER the queue
    //! shutdown. Worker threads never touch it.
    mutable std::shared_mutex gate_mutex_;

    //! Serializes processFrame() between concurrent producers (sync mode). In
    //! async mode only the processing thread runs it, so it is uncontended.
    std::mutex process_mutex_;

    // Async input path — both null / not-joinable in synchronous mode.
    std::unique_ptr<util::ThreadsafeQueue<sensor::FrameData>> input_queue_;
    std::thread                                               proc_thread_;

    // Monitor thread.
    std::thread             monitor_thread_;
    std::mutex              monitor_mutex_;
    std::condition_variable monitor_cv_;
    bool                    monitor_stop_ = false;  // guarded by monitor_mutex_

    // Every output channel, main output included (design §3.7).
    SystemCallbacks callbacks_;

    // VOData handed over by VOModule during process_frame(); touched only on
    // the thread that calls onFrame().
    vo::VOData last_vo_data_;
    bool       have_vo_data_ = false;

    // Newest main output, for latest(), and the newest RAW back-end result —
    // the search prior an AnchorQuery carries. Both under output_mutex_.
    mutable std::mutex   output_mutex_;
    LocalizationOutput   latest_output_;
    fusion::FusionResult last_fusion_;
    bool                 last_fusion_valid_ = false;

    // Geo anchoring + the telemetry of the frame currently in flight.
    mutable std::mutex          geo_mutex_;
    sensor::GeoReferencer       geo_;
    sensor::TelemetryData       frame_telemetry_;
    bool                        frame_telemetry_valid_ = false;

    // Counters.
    mutable std::mutex  stats_mutex_;
    SystemStats         stats_;
    double              proc_sec_total_ = 0.0;
    //! Live (trailing-window) throughput. Guarded by stats_mutex_ — the class
    //! is deliberately not thread-safe on its own.
    RateEstimator       rate_;
    vo::VOTrackingState prev_vo_state_  = vo::VOTrackingState::NOT_INITIALIZED;
    bool                had_first_init_ = false;
    bool                pending_reinit_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────

SystemManager::SystemManager(const SystemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

SystemManager::~SystemManager() = default;

void SystemManager::attachSource(std::unique_ptr<sensor::DataSourceInterface> source) {
    impl_->attachSource(std::move(source));
}

double SystemManager::sourceCompletion() const { return impl_->sourceCompletion(); }

bool SystemManager::pauseSource()        { return impl_->setSourceStreaming(false); }
bool SystemManager::resumeSource()       { return impl_->setSourceStreaming(true); }
bool SystemManager::isSourcePaused() const { return impl_->isSourcePaused(); }

void SystemManager::setAnchor(std::unique_ptr<anchor::AnchorInterface> anchor) {
    impl_->setAnchor(std::move(anchor));
}

bool SystemManager::setup() { return impl_->setup(); }
bool SystemManager::start() { return impl_->start(); }
void SystemManager::stop()  { impl_->stop(); }

SystemState SystemManager::state() const { return impl_->state(); }

bool SystemManager::onFrame(const sensor::FrameData& fd) { return impl_->onFrame(fd); }

bool SystemManager::onAttitude(double t_msec, const sensor::AttitudeData& att) {
    return impl_->onAttitude(t_msec, att);
}

bool SystemManager::onGimbal(double t_msec, const sensor::GimbalData& gim) {
    return impl_->onGimbal(t_msec, gim);
}

bool SystemManager::onGnss(double t_msec, const sensor::GnssData& gnss) {
    return impl_->onGnss(t_msec, gnss);
}

bool SystemManager::onImage(double t_msec, const sensor::ImageData& img) {
    return impl_->onImage(t_msec, img);
}

void SystemManager::onStreamEvent(const sensor::StreamEvent& event) {
    impl_->onStreamEvent(event);
}

bool SystemManager::pushAbsoluteFix(const anchor::AbsoluteFix& fix) {
    return impl_->pushAbsoluteFix(fix);
}

SystemCallbacks& SystemManager::callbacks() { return impl_->callbacks(); }

LocalizationOutput SystemManager::latest() const { return impl_->latest(); }
SystemStats        SystemManager::stats()  const { return impl_->stats(); }

fusion::FusionFixStats SystemManager::fixStats() const { return impl_->fixStats(); }

} // namespace core
} // namespace uavloc
