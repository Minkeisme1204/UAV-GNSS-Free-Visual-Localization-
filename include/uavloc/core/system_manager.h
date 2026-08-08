#pragma once

// SystemManager — the whole uavloc pipeline behind one object (S5: synchronous
// AND asynchronous input). See .docs/designs/system_manager_design.md
// §4.1 / §4.2 / §4.3 / §4.4 / §4.6.
//
// It owns the VO front-end, the fusion back-end and the geo-referencer, and
// wires them together in exactly the order the offline driver
// (tests/test_full_flight.cpp) uses today, so packaging the pipeline cannot
// change a single number of the evaluated baseline (that is the S4 gate).
//
// Scope, on purpose NOT here yet:
//   * LocalizationOutput::accuracy_m stays 0                           (S7)
//
// ── Two input paths, ONE pipeline (S6c) ──────────────────────────────────────
// onFrame() takes an ALREADY-SYNCHRONISED sample; the four typed channels
// (onImage/onAttitude/onGimbal/onGnss) take the raw sensor streams. The three
// non-image channels only fill a core::Extrapolator; onImage() is the assembly
// step (design §4.8, kcb's SynchronizeData()): it queries the extrapolator AT
// THE IMAGE TIMESTAMP, rebuilds a sensor::FrameData and calls onFrame(). There
// is no second copy of the pipeline — the channel path is a pre-stage in front
// of the very same onFrame(), which is what keeps the two bit-identical.
//
// ── Threading ────────────────────────────────────────────────────────────────
// Two independent axes (design §4.2):
//
//   SystemConfig::async_input == false  (default, DETERMINISTIC)
//       onFrame() runs the whole pipeline inline on the caller's thread. No
//       input queue, no processing thread. This is the path every offline
//       evaluation and every bit-identical regression gate uses.
//
//   SystemConfig::async_input == true
//       onFrame() only enqueues and returns; one processing thread pops and
//       runs the pipeline. SystemConfig::input_policy decides what a full
//       queue means: BLOCK (producer waits up to push_timeout_ms; no frame
//       lost) or DROP_OLDEST (producer never waits; the oldest queued frame is
//       discarded).
//
// A monitor thread publishes SystemStats every SystemConfig::stats_period_ms
// in BOTH modes (0 disables it). Its cadence is deliberately independent of
// the processing load.
//
// The fusion result callback — and therefore on_localization and every slot fed
// from it — fires on the fusion thread when
// FusionConfig::async_enabled is true, so subscribers must be thread-safe and
// must never call back into SystemManager (design §R-c).

#include "uavloc/anchor/absolute_fix.h"
#include "uavloc/anchor/anchor_interface.h"
#include "uavloc/core/system_config.h"
#include "uavloc/core/system_types.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/sensor/data_source_interface.h"
#include "uavloc/sensor/frame_data.h"
#include "uavloc/sensor/stream_types.h"
#include "uavloc/util/callback_slot.h"

#include <memory>
#include <vector>

namespace uavloc {
namespace core {

//! Every channel SystemManager publishes on, as ONE named event struct
//! (design §3.7). Deliberately a MEMBER of SystemManager, never a static bus:
//! libuavloc is a library, so two managers must be able to coexist and a test
//! must start from an empty slot.
//!
//! The leading `double` of a channel is the frame/result timestamp [ms] — the
//! same value the payload carries, hoisted out so a consumer can dispatch on
//! time without unpacking the payload type.
//!
//! Emission order per processed frame is fixed (design §4.6):
//!   on_frame_processed → on_vo_data → on_fusion_result → on_localization
//!   → on_lag_window
//! on_stats is independent: the monitor thread emits it on its own cadence.
struct SystemCallbacks {
    //! MAIN output: one fused fix per processed frame.
    util::CallbackSlot<void(const LocalizationOutput&)> on_localization;

    //! Raw VO payload (pose + map points) of the frame just processed.
    util::CallbackSlot<void(double, const vo::VOData&)> on_vo_data;

    //! RAW fusion result, emitted BEFORE the LocalizationOutput built from it.
    //! LocalizationOutput deliberately drops the back-end's internal states (ENU
    //! pose, scale S(k), AGL bias b(k)), so an evaluation driver that has to
    //! report them needs this channel.
    util::CallbackSlot<void(double, const fusion::FusionResult&)> on_fusion_result;

    //! The smoothed lag window, only when the graph was actually updated.
    util::CallbackSlot<void(const std::vector<fusion::FusionLagPose>&)> on_lag_window;

    //! Image + tracking figures. The payload is assembled ONLY when this slot
    //! has at least one subscriber (it copies the tracked observations).
    util::CallbackSlot<void(double, const FrameProcessed&)> on_frame_processed;

    //! Monitoring counters, emitted from the MONITOR thread every
    //! SystemConfig::stats_period_ms.
    util::CallbackSlot<void(const SystemStats&)> on_stats;
};

class SystemManager {
public:
    SystemManager() = delete;

    explicit SystemManager(const SystemConfig& config);

    ~SystemManager();

    SystemManager(const SystemManager&)            = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    // ── source (optional — design §4.5) ──────────────────────────────────────

    //! Take ownership of a push-style source and subscribe to its five channels
    //! (attitude / gimbal / GNSS / image / events). The system OWNS the source
    //! but never drives it: it never calls read(), and start()/stop() are what
    //! sequence startStreaming()/stopStreaming() correctly — that sequencing is
    //! the whole reason ownership sits here rather than in the application.
    //!
    //! Must be called BEFORE start(); a call in any other state is refused (and
    //! `source` is destroyed on return). Attaching a second source replaces the
    //! first, which is unsubscribed and destroyed.
    void attachSource(std::unique_ptr<sensor::DataSourceInterface> source);

    //! Replay progress of the attached source in [0, 1], or NaN when there is
    //! no source or it cannot know (a live camera). Exists because the source
    //! is held by unique_ptr, so the application has no handle to ask directly.
    double sourceCompletion() const;

    //! PAUSE the input: stop the source's reading thread and nothing else.
    //!
    //! Explicitly NOT stop(): the system state stays RUNNING, the channel
    //! subscriptions stay in place and no component (VO, fusion, anchor,
    //! threads) is torn down — only the producer goes quiet. A file source is
    //! not rewound either, so resumeSource() continues at the next frame.
    //!
    //! Two named methods rather than one setSourceRunning(bool) on purpose:
    //! the preconditions and the failure meanings differ, and a call site
    //! reading `pauseSource()` cannot be mistaken for the permanent `stop()`.
    //!
    //! Returns false — and changes nothing — when there is no source, when the
    //! system is not RUNNING, or when the source refuses to stop (which is what
    //! happens if this is called from inside the source's own data callback:
    //! the reading thread cannot join itself). Idempotent: pausing twice
    //! succeeds. Safe to call from any thread EXCEPT the source's reading
    //! thread.
    bool pauseSource();

    //! RESUME a paused input: restart the source's reading thread where it left
    //! off. Same restrictions as pauseSource(); returns false when there is no
    //! source, when the system is not RUNNING, or when the source refuses to
    //! start. Idempotent.
    bool resumeSource();

    //! True while the input is paused by pauseSource(). Only meaningful while
    //! the system is RUNNING — stop() shuts the source down for good and does
    //! not touch this flag.
    bool isSourcePaused() const;

    // ── absolute-position producer (optional — M1/M2) ────────────────────────

    //! Take ownership of an absolute-position producer (the M1
    //! anchor::FakeAnchor today, VPR later). The system then:
    //!   * sets it up and starts/stops it inside its OWN lifecycle;
    //!   * hands it the ENU origin as soon as the pipeline has chosen one
    //!     (anchor::AnchorInterface::setEnuOrigin) — the producer never picks
    //!     a coordinate frame of its own;
    //!   * calls requestFix() once per VO KEYFRAME (a producer that cannot run
    //!     that often refuses the request; the cadence is ITS business);
    //!   * forwards every returned fix to the fusion back-end and does NOTHING
    //!     else on the producer's thread.
    //!
    //! Must be called BEFORE setup(); a call in any other state is refused (and
    //! `anchor` is destroyed on return). Attaching a second producer replaces
    //! the first, which is stopped and destroyed.
    //!
    //! ⚠ A producer whose setup() fails makes SystemManager::setup() fail too.
    //! Running on silently with no absolute positions would turn a broken
    //! experiment into a plausible-looking result.
    void setAnchor(std::unique_ptr<anchor::AnchorInterface> anchor);

    // ── lifecycle ────────────────────────────────────────────────────────────

    //! Build the VO / fusion / geo components and register the internal fusion
    //! callback. Spawns NO thread and opens NO device, so a unit test can call
    //! it freely. Returns false (and logs) when called twice or when the
    //! configuration is unusable.
    bool setup();

    //! Start accepting frames: starts the fusion back-end, spawns the
    //! processing thread (async_input only) and the monitor thread, then moves
    //! to RUNNING. Returns false when setup() has not succeeded, or when
    //! already started.
    bool start();

    //! Stop accepting frames and shut the back-end down, in the reverse order
    //! of start() (design §4.4). Idempotent; also called by the destructor.
    //!
    //! Guarantees, and they hold even when another thread is inside onFrame()
    //! at the same time:
    //!   * every frame ALREADY ACCEPTED (onFrame() returned true) is processed
    //!     before stop() returns — the input queue is drained, never discarded;
    //!   * a producer blocked in onFrame() waiting for queue space is woken and
    //!     leaves with false, so stop() cannot deadlock against it;
    //!   * no pipeline component is torn down while a concurrent onFrame() is
    //!     still using it;
    //!   * once stop() has returned, onFrame() refuses immediately.
    void stop();

    SystemState state() const;

    // ── input ────────────────────────────────────────────────────────────────

    //! Hand one frame to the pipeline. Callable from any thread, at any time,
    //! including concurrently with stop().
    //!   * async_input == false: runs the whole pipeline inline and returns
    //!     when the frame has been processed.
    //!   * async_input == true: enqueues and returns immediately.
    //!
    //! Returns false when the frame was NOT taken in full: the system is not
    //! RUNNING, the frame carries no image, a BLOCK push hit push_timeout_ms,
    //! or — under DROP_OLDEST — the queue was full and an older frame had to be
    //! discarded to make room. ⚠ That last case is the one asymmetry: the frame
    //! just handed in IS queued, but a previous one was lost, so the return
    //! value reports "the system is losing frames", which is what a producer
    //! needs to know. Every false return increments
    //! SystemStats::frames_dropped exactly once.
    bool onFrame(const sensor::FrameData& fd);

    //! ── the four typed channels (design §3.1) ────────────────────────────────
    //! A source publishes attitude → gimbal → gnss → IMAGE for one sample, all
    //! with the same timestamp. Only onImage() triggers work.

    //! Buffer one attitude / gimbal / GNSS sample. Returns false when the
    //! system is not RUNNING (the sample is then dropped, not queued).
    bool onAttitude(double t_msec, const sensor::AttitudeData& att);
    bool onGimbal(double t_msec, const sensor::GimbalData& gim);
    bool onGnss(double t_msec, const sensor::GnssData& gnss);

    //! THE ASSEMBLY STEP. Queries the extrapolator at `t_msec` for the three
    //! other channels, rebuilds a sensor::FrameData and forwards it to
    //! onFrame() — so the return value, the counters and the whole downstream
    //! behaviour are onFrame()'s.
    //!
    //! ⚠ A frame whose attitude, gimbal OR GNSS is missing at `t_msec` is
    //! forwarded with `has_telemetry = false`, exactly as sensor::VideoReader
    //! does when the CSV has no row for a frame. NOTHING is invented, and
    //! SystemStats::frames_without_telemetry counts these.
    bool onImage(double t_msec, const sensor::ImageData& img);

    //! Record an out-of-band source event. END_OF_STREAM / DISCONNECTED set
    //! SystemStats::source_ended; the system does NOT stop itself — the owner
    //! decides when to tear down — but on_stats fires so the owner finds out.
    void onStreamEvent(const sensor::StreamEvent& event);

    //! Forward an absolute position measurement to the fusion back-end (M1).
    //! Returns false when the system is not RUNNING or fusion is disabled.
    bool pushAbsoluteFix(const anchor::AbsoluteFix& fix);

    // ── output ───────────────────────────────────────────────────────────────

    //! Every output channel, including the main one:
    //!   sys.callbacks().on_localization.add([](const LocalizationOutput& o){ … });
    //!
    //! Subscribing / removing is safe at any time, from any thread, including
    //! from inside a callback. Slots may fire on the fusion thread (see the
    //! threading note above) and, for on_stats, on the monitor thread, so a
    //! subscriber must be thread-safe and must never call back into
    //! SystemManager (design §R-c).
    SystemCallbacks& callbacks();

    // ── queries ──────────────────────────────────────────────────────────────

    //! Newest LocalizationOutput (thread-safe snapshot). `valid == false` until
    //! the first fused, geo-anchored fix exists.
    LocalizationOutput latest() const;

    //! Counter snapshot (thread-safe). The same value the monitor thread
    //! emits on callbacks().on_stats every SystemConfig::stats_period_ms.
    SystemStats stats() const;

    //! Cumulative absolute-fix accounting of the fusion back-end (M1). All
    //! zeros when fusion is disabled or setup() has not run.
    fusion::FusionFixStats fixStats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace core
} // namespace uavloc
