#pragma once

// core public data types (S2) — the contract between SystemManager and the
// application layer. See .docs/designs/system_manager_design.md §3.2.
//
// LocalizationOutput is the ONE production output: a flat POD an autopilot /
// ROS bridge / flight log can consume without knowing anything about VO,
// GTSAM or the factor graph. FrameProcessed and SystemStats are debug /
// monitoring channels, delivered through util::CallbackSlot (see
// core::SystemCallbacks in system_manager.h).
//
// This header stays back-end-free on purpose: no GTSAM, no g2o. Only OpenCV
// (image buffer), Eigen, std and the already-gtsam-free fusion data types.

#include "uavloc/fusion/fusion_data.h"
#include "uavloc/sensor/telemetry_data.h"

#include <opencv2/core.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <vector>

namespace uavloc {
namespace core {

//! Lifecycle state of SystemManager (§4.4). CREATED → RUNNING on start(),
//! RUNNING → STOPPING → STOPPED while stop() tears the pipeline down.
enum class SystemState {
    CREATED,
    RUNNING,
    STOPPING,
    STOPPED
};

//! What onFrame() does when the async input queue is full (§4.2). Orthogonal
//! to SystemConfig::async_input.
enum class InputPolicy {
    BLOCK,       //!< producer waits (bounded by push_timeout_ms); no frame lost
    DROP_OLDEST  //!< producer never waits; the oldest queued frame is discarded
};

//! MAIN output — what the outside world (autopilot, ROS, flight log) consumes.
//! Flat POD by design; no pipeline type leaks through it.
struct LocalizationOutput {
    double       timestamp_msec = 0.0;  //!< frame timestamp [ms]
    unsigned int frame_id       = 0;

    double latitude  = 0.0;  //!< [deg]
    double longitude = 0.0;  //!< [deg]

    //! Height ABOVE GROUND LEVEL [m] — the same convention as
    //! sensor::TelemetryData::altitude_m and as the Z of
    //! fusion::FusionResult::T_enu_c (the graph anchors X(0) at ENU
    //! (0, 0, agl_0), so its vertical coordinate IS an AGL).
    //! ⚠ This is NOT AMSL: a consumer that needs mean-sea-level altitude must
    //! add the terrain elevation at the geo anchor point.
    double altitude_m = 0.0;

    //! Horizontal 1-sigma uncertainty radius [m]: the LARGEST semi-axis of the
    //! 1-sigma error ellipse of the fused position, i.e.
    //! fusion::horizontal_accuracy_m() applied to the ENU marginal covariance
    //! of the fused keyframe state X(k) (S7).
    //!
    //! **NaN when no covariance is available** (graph not anchored yet, or the
    //! marginalization failed) — never 0. 0 would mean "perfectly certain",
    //! the most dangerous value to hand an autopilot. A NaN accuracy also
    //! forces `valid = false`: a position whose uncertainty is unknown is not
    //! a usable fix.
    //!
    //! ⚠⚠ MODEL uncertainty, NOT a calibrated confidence radius. Measured on
    //! YenBai (.docs/reports/m1_fake_anchor.md §6.2) the graph reported ~17 m
    //! while the true horizontal error was ~2 000 m — over-confident by ~118×,
    //! because the graph models VO error as independent noise (√n) while the
    //! real drift is systematic (n). Use it as a relative indicator, not as a
    //! metre-level bound.
    float accuracy_m = std::numeric_limits<float>::quiet_NaN();

    //! ⚠ ATTITUDE OF THE **CAMERA IN ENU**, NOT the airframe in NED.
    //! These are the ZYX (yaw-pitch-roll) Euler angles [deg] of the rotation
    //! part of fusion::FusionResult::T_enu_c, i.e. of R_enu_camera. They
    //! therefore contain the gimbal chain and the camera-axis alignment. A
    //! consumer that wants airframe roll/pitch/heading must invert the gimbal
    //! chain built in src/fusion/factors.h `rotation_enu_camera()` (ENU→NED
    //! flip, R_body_cam, r_cam_alignment); that inversion is deferred to S7.
    float roll  = 0.0f;  //!< [deg], camera-in-ENU
    float pitch = 0.0f;  //!< [deg], camera-in-ENU
    float yaw   = 0.0f;  //!< [deg], camera-in-ENU

    fusion::FusionHealth health = fusion::FusionHealth::INITIALIZING;

    //! false = this sample carries no usable fix; consumers must skip it.
    bool valid = false;
};

//! Image + tracking figures of one processed frame — for the display layer.
struct FrameProcessed {
    unsigned int frame_id       = 0;
    double       timestamp_msec = 0.0;

    //! EMPTY unless SystemConfig::publish_images = true (an attached image
    //! keeps the whole frame buffer alive; see design §R-e).
    cv::Mat image;

    int num_tracked   = 0;  //!< landmarks matched into this frame
    int num_landmarks = 0;  //!< landmarks in the local map
    int num_inliers   = 0;  //!< inliers kept by pose optimization

    //! Pixel coordinates of the tracked observations [px].
    std::vector<Eigen::Vector2d> tracked_observations;

    //! Telemetry the pipeline actually saw for this frame (S6c). On the
    //! onFrame() path it is a copy of what the caller supplied; on the
    //! four-channel path it is what the assembly step REBUILT from the
    //! extrapolator — which makes that step observable to a test and to the
    //! display layer. `has_telemetry == false` means the frame carried none;
    //! `telemetry` is then default-constructed, never invented.
    sensor::TelemetryData telemetry;
    bool                  has_telemetry = false;
};

//! Monitoring figures, published periodically from the monitor thread
//! (SystemConfig::stats_period_ms).
struct SystemStats {
    unsigned long long frames_received  = 0;  //!< handed to onFrame()
    unsigned long long frames_processed = 0;  //!< actually ran through the pipeline
    unsigned long long frames_dropped   = 0;  //!< lost to DROP_OLDEST / refused pushes

    std::size_t queue_depth   = 0;    //!< items currently in the input queue
    double      fps_processed = 0.0;  //!< processing rate [frames/s]

    unsigned int lost_events   = 0;  //!< VO tracking losses so far
    unsigned int reinit_events = 0;  //!< VO re-initializations so far

    //! An attached source published sensor::StreamEventKind::END_OF_STREAM (or
    //! DISCONNECTED). SystemManager records it and does NOT stop itself — the
    //! owner decides when to tear down (design §4.5), so this flag is how the
    //! owner learns the stream is over.
    bool source_ended = false;

    //! Non-terminal source anomalies seen so far (READ_ERROR / TIMEOUT).
    unsigned int stream_errors = 0;

    //! Image samples that arrived with no complete attitude+gimbal+GNSS state
    //! at their timestamp, and were therefore handed to the pipeline WITHOUT
    //! telemetry (never with invented values).
    unsigned long long frames_without_telemetry = 0;

    SystemState state = SystemState::CREATED;
};

} // namespace core
} // namespace uavloc
