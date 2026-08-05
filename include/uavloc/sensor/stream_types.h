#pragma once

//! Payload types of the four PUSH channels of sensor::DataSourceInterface
//! (S6a, .docs/designs/system_manager_design.md §3.1-§3.2, §3.6).
//!
//! The split follows the kcb `DataInputTypeDefs.hpp` shape: one struct per
//! physical sensor, and the TIMESTAMP IS NOT A MEMBER — it is the first
//! argument of every channel callback. Two reasons: a real rig timestamps each
//! sensor independently (so the timestamp belongs to the *sample*, not to the
//! payload type), and core::Extrapolator (S6b) keys its buffers on that
//! argument.
//!
//! ── How TelemetryData maps onto the four channels ────────────────────────────
//!   TelemetryData::timestamp_msec  → the `double t_msec` callback argument
//!   TelemetryData::frame_id        → ImageData::frame_id
//!   TelemetryData::roll_deg        → AttitudeData::roll_deg
//!   TelemetryData::pitch_deg       → AttitudeData::pitch_deg
//!   TelemetryData::heading_deg     → AttitudeData::yaw_deg      (yaw == heading)
//!   TelemetryData::gimbal_pan_deg  → GimbalData::pan_deg
//!   TelemetryData::gimbal_tilt_deg → GimbalData::tilt_deg
//!   TelemetryData::latitude_deg    → GnssData::latitude_deg
//!   TelemetryData::longitude_deg   → GnssData::longitude_deg
//!   TelemetryData::altitude_m      → GnssData::altitude_m       (AGL, §4.7b)
//!   TelemetryData::speed_mps       → GnssData::speed_mps
//!
//! NOT carried by the four channels — reassembly is lossless only because these
//! are inert today; anything that starts using them must extend a channel:
//!   TelemetryData::climb_mps  — no producer sets it (TelemetryCsvReader parses
//!       a `climb_mps` column, DroneTelemetryCsvReader hard-codes 0.0) and no
//!       consumer reads it. §3.2 of the design lists four GnssData fields and
//!       climb is not one of them, so it stays out until a consumer appears.
//!   TelemetryData::valid      — a per-sample flag of the PULL path; on the push
//!       path "the channel fired" carries the same information (a source that
//!       has no telemetry for a frame publishes the image channel only).
//! FrameData::source_name is likewise not carried: it names the source object,
//! not the sample, and the receiver already knows which source it subscribed to.

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace uavloc {
namespace sensor {

//! One camera frame. Publishing this channel TRIGGERS a processing cycle
//! (§3.1), so a source must publish it LAST among the four channels of a
//! sample.
//!
//! `image` shares its buffer with the source's decoded frame (cv::Mat is
//! reference-counted): subscribers may read it and keep a reference, but must
//! never write through it.
struct ImageData {
    uint64_t    frame_id = 0;
    cv::Mat     image;
    std::string camera_id;
};

//! Airframe attitude from the AHRS. Degrees; conventions are those of
//! TelemetryData: yaw 0 = North clockwise, pitch positive = nose up, roll
//! positive = right wing down.
struct AttitudeData {
    double roll_deg  = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg   = 0.0;
};

//! Camera pointing relative to the airframe. Degrees; `tilt_deg` is measured
//! from the horizontal plane, so 90 = nadir.
struct GimbalData {
    double pan_deg  = 0.0;
    double tilt_deg = 0.0;
};

//! GNSS fix. `altitude_m` is ABOVE GROUND LEVEL (design §4.7b), not AMSL —
//! same convention as TelemetryData::altitude_m.
struct GnssData {
    double latitude_deg  = 0.0;
    double longitude_deg = 0.0;
    double altitude_m    = 0.0;
    double speed_mps     = 0.0;
};

//! Why a source stopped producing, or a non-fatal read anomaly (§3.6).
enum class StreamEventKind {
    END_OF_STREAM,   //!< the source will produce nothing more
    READ_ERROR,      //!< a read failed; the source keeps trying
    DISCONNECTED,    //!< the device went away
    TIMEOUT          //!< a read exceeded its deadline; the source keeps trying
};

//! Out-of-band notification from a source. `frame_id` is the id of the LAST
//! frame published before the event (0 if none), so a receiver can tell how far
//! the stream got.
struct StreamEvent {
    StreamEventKind kind = StreamEventKind::END_OF_STREAM;
    uint64_t        frame_id = 0;
    std::string     message;
};

}  // namespace sensor
}  // namespace uavloc
