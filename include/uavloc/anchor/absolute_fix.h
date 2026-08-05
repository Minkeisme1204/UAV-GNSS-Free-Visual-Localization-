#pragma once

// AbsoluteFix — a single absolute horizontal position measurement (M1).
//
// The RESPONSE half of the anchor module's request/response seam (the request
// half is anchor::AnchorQuery): the shared data type between whatever produces
// absolute positions — anchor::FakeAnchor, driven by groundtruth, today; VPR +
// geo-referencing later — and the fusion back-end that consumes them. `fusion`
// deliberately does not know where a fix came from, so swapping the producer
// is the whole of M6.
//
// Frame: xy_enu is expressed in the SAME ENU frame as the fusion state X(k),
// i.e. the local tangent frame anchored at the fusion initialization instant.

#include <Eigen/Core>

namespace uavloc::anchor {

//! One absolute horizontal (East, North) position measurement with its
//! uncertainty. Vertical position is deliberately absent: altitude is already
//! measured by telemetry AGL (fusion::AglFactor).
struct AbsoluteFix {
    //! Timestamp of the image the fix was computed from, used to attach the
    //! fix to the keyframe state X(k) of that instant.
    double timestamp_msec = 0.0;

    //! Measured position (East, North) in metres, same origin as X(0).
    Eigen::Vector2d xy_enu = Eigen::Vector2d::Zero();

    //! 2x2 covariance of xy_enu [m^2]. Must be symmetric positive definite.
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();

    //! Producer-side confidence in [0, 1] — how much the PRODUCER trusts this
    //! match, from what only the producer knows (number of inliers, descriptor
    //! quality, agreement with an independent altitude source …). It is NOT a
    //! function of the fusion estimate.
    //!
    //! Read by fusion since 2026-08-02: a fix below
    //! fusion::FusionConfig::fix_min_confidence is dropped
    //! (fusion::FusionFixStats::low_confidence).
    //!
    //! ⚠ DEBT — this field is now the MAIN outlier defence. The back-end's
    //! self-consistency Mahalanobis gate is disabled by default (it fed a
    //! drift loop: a drifted estimate rejects the correct fixes that would
    //! have fixed it), and `cov` below is the only other thing shaping a fix's
    //! influence. The M1 fake anchor hard-codes confidence = 1.0 and a constant
    //! cov, so at M1 there is NO defence at all. A VPR producer (M3–M5) MUST
    //! fill both fields honestly — one wrong match with confidence 1.0 will
    //! drag the whole trajectory and nothing downstream will stop it.
    double confidence = 0.0;

    //! False marks a non-measurement (e.g. VPR found no match); consumers
    //! must ignore every other field then.
    bool valid = false;
};

} // namespace uavloc::anchor
