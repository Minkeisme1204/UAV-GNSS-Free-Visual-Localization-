#pragma once

// AnchorQuery — everything a place-recognition producer is given about ONE
// instant, and the request half of the anchor module's request/response seam
// (the response half is anchor::AbsoluteFix).
//
// Shape follows kcb_slam's satcom interface (RequestPlaceRecognition → result
// callback), with one deliberate difference: kcb passes a stamp and a position
// and then `(void)`s both in the implementation, because the timestamp it
// actually uses lives inside the keyframe the producer fetches itself. Here the
// timestamp is a MANDATORY field of the query and the ONLY key that ties the
// resulting fix back to the state X(k) of that instant — one path, no shadow
// copy that can silently disagree.
//
// A producer is free to ignore any field it does not need (the M1 FakeAnchor
// ignores `image`); nothing here is a promise that the value is usable, which
// is what `prediction_valid` exists to say.

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>

namespace uavloc::anchor {

//! WHY the pipeline is asking. A producer is allowed — and expected — to treat
//! the two differently, because they are not the same kind of request:
//!
//!  * KEYFRAME — the regular CADENCE. One request per keyframe, most of which a
//!    real producer cannot afford to answer; refusing them is normal operation.
//!  * REINIT — an EVENT: the VO chain just broke and was welded back together,
//!    so the relative measurement restarted from a fresh, unconstrained frame
//!    (scale back at 1.0). This is the instant an absolute measurement is worth
//!    the most, and a duty cycle that happens to fall between two slots must not
//!    be what decides whether it is taken.
//!
//! Nothing here FORCES a producer to answer a REINIT — the data gates (usable
//! telemetry, known ENU origin, a match) still apply, and so may an
//! anti-flooding limit. What the field guarantees is only that the producer
//! CAN tell the two apart; a producer that ignores it degrades to the old
//! cadence-only behaviour.
enum class AnchorRequestReason {
    KEYFRAME,
    REINIT
};

struct AnchorQuery {
    //! Timestamp of the image [ms]. THE key: the fix produced from this query
    //! must carry the same value in AbsoluteFix::timestamp_msec, because that
    //! is what the back-end matches against its keyframe states.
    double timestamp_msec = 0.0;

    //! Frame id of the image. Diagnostics and dataset lookup only — never used
    //! to attach the fix to a state (frame ids are not unique across a
    //! re-initialisation, timestamps are monotone).
    unsigned int frame_id = 0;

    //! The image itself. cv::Mat is refcounted, so carrying it costs a header;
    //! a producer that does not look at pixels (FakeAnchor) simply ignores it.
    cv::Mat image;

    //! Current filter estimate of the horizontal position [m, ENU] and its 2x2
    //! covariance [m²] — the search prior a real VPR uses to restrict the
    //! candidate area.
    Eigen::Vector2d xy_enu_pred = Eigen::Vector2d::Zero();
    Eigen::Matrix2d cov_pred    = Eigen::Matrix2d::Identity();

    //! false = the two fields above carry NO information (the back-end has no
    //! pose or no covariance yet). Present so that the default identity
    //! covariance can never be mistaken for "1 m² and confident".
    bool prediction_valid = false;

    //! Telemetry of the instant, as the pipeline saw it. `agl_m <= 0` means the
    //! frame had no usable telemetry at all.
    double agl_m           = 0.0;
    double yaw_deg         = 0.0;
    double gimbal_pan_deg  = 0.0;
    //! ⚠ Measured FROM THE HORIZONTAL PLANE: ~90 is nadir, not 0. The off-nadir
    //! angle is `|90 - gimbal_tilt_deg|` (sensor::TelemetryData:26).
    double gimbal_tilt_deg = 0.0;

    //! Airframe roll and pitch [deg]. A consumer that rectifies the frame to a
    //! bird's-eye view needs the FULL attitude, not just heading — cancelling
    //! yaw alone leaves the perspective of an oblique camera intact and leaves
    //! the frame CENTRE standing in for the point below the aircraft. On the Yen
    //! Bai set those two are a median 110 m apart.
    //!
    //! Both already travel in sensor::TelemetryData; they simply were not copied
    //! here before.
    double roll_deg  = 0.0;
    double pitch_deg = 0.0;

    //! Why this request exists (see AnchorRequestReason). Defaults to the
    //! cadence case, so an existing caller keeps its previous meaning.
    AnchorRequestReason reason = AnchorRequestReason::KEYFRAME;
};

} // namespace uavloc::anchor
