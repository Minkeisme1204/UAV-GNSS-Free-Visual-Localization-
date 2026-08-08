#pragma once

//! sensor::DataSourceInterface — the PUSH-style counterpart of DataInterface
//! (S6a, .docs/designs/system_manager_design.md §3.1, §4.8).
//!
//! DataInterface is a PULL API: the consumer calls read() in its own loop.
//! DataSourceInterface inverts that: the source owns a thread, and publishes
//! four typed channels plus an event channel. Shape taken from kcb's
//! `DataSourceInterface` (setImagesCallback / setAHRSCallback / ... +
//! StartStreaming/StopStreaming/isStreaming/completion), with two changes:
//!
//!   1. the raw `std::vector<std::function<...>>` members are replaced by
//!      util::CallbackSlot, the project's single publish/subscribe primitive
//!      (§3.7). That buys unsubscription by id and a snapshot-under-lock
//!      emission, neither of which the kcb version has (it iterates its vectors
//!      with no lock at all while another thread may push_back — a data race);
//!   2. the callbacks return void, not bool. kcb returns bool to signal "the
//!      receiver is congested", but CallbackSlot is specialized on
//!      `void(Args...)` by project-wide decision, and it fans out to N
//!      subscribers whose N answers have no single meaning. Backpressure
//!      therefore lives on the RECEIVER side: core::SystemManager (S6c) counts
//!      its own dropped frames in SystemStats. A source cannot learn it from
//!      here — see the S6a report.
//!
//! ── Channel contract ─────────────────────────────────────────────────────────
//!   * Subscribers run SYNCHRONOUSLY on the source's streaming thread. A slow
//!     subscriber throttles the source; a blocking one stalls it.
//!   * For one sample, a source publishes attitude → gimbal → gnss → IMAGE, in
//!     that order, with the SAME timestamp. The image channel is what triggers
//!     downstream processing, so it must come last: the buffers of the other
//!     three have to already hold that instant when it fires.
//!   * A sample whose non-image channels are unknown publishes the image
//!     channel ONLY. A source never invents attitude/gimbal/gnss values.
//!   * Subscribe before startStreaming(). Subscribing later is safe but may
//!     miss samples already published.

#include <limits>

#include "uavloc/sensor/stream_types.h"
#include "uavloc/util/callback_slot.h"

namespace uavloc {
namespace sensor {

class DataSourceInterface {
public:
    using AttitudeSlot = util::CallbackSlot<void(double, const AttitudeData&)>;
    using GimbalSlot   = util::CallbackSlot<void(double, const GimbalData&)>;
    using GnssSlot     = util::CallbackSlot<void(double, const GnssData&)>;
    using ImageSlot    = util::CallbackSlot<void(double, const ImageData&)>;
    using EventSlot    = util::CallbackSlot<void(const StreamEvent&)>;

    virtual ~DataSourceInterface() = default;

    DataSourceInterface(const DataSourceInterface&)            = delete;
    DataSourceInterface& operator=(const DataSourceInterface&) = delete;

    // ── channels (first callback argument is the sample timestamp [ms]) ──────
    AttitudeSlot& attitudeChannel();
    GimbalSlot&   gimbalChannel();
    GnssSlot&     gnssChannel();
    ImageSlot&    imageChannel();
    EventSlot&    eventChannel();

    //! Drops every subscriber of all five channels. Modelled on kcb's
    //! `clearCallbacks()`, which `AhrsVprFusion::Stop()` calls FIRST so that
    //! nothing can touch the system while it is being torn down. Safe to call
    //! while streaming, but a call already in flight still completes (see the
    //! CallbackSlot contract, note 3).
    void clearCallbacks();

    //! Opens the underlying device/file if needed and starts the streaming
    //! thread. Returns false if the source could not be opened. Idempotent:
    //! a second call while streaming returns true and does nothing.
    virtual bool startStreaming() = 0;

    //! Stops the streaming thread and joins it. Idempotent and never blocks
    //! forever. Must not be called from inside a channel callback (that would
    //! be a self-join); implementations detect and refuse that.
    virtual bool stopStreaming() = 0;

    virtual bool isStreaming() const = 0;

    //! Replay progress in [0,1], or NaN when the source cannot know it (a live
    //! camera). Same role as kcb's `DatasetReaderBase::completion()`.
    virtual double completion() const { return std::numeric_limits<double>::quiet_NaN(); }

protected:
    DataSourceInterface() = default;

    AttitudeSlot attitude_channel_;
    GimbalSlot   gimbal_channel_;
    GnssSlot     gnss_channel_;
    ImageSlot    image_channel_;
    EventSlot    event_channel_;
};

}  // namespace sensor
}  // namespace uavloc
