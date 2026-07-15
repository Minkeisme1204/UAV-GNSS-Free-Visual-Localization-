#pragma once

// Fusion module public data types (Stage F1).
//
// FusionResult is the per-frame output of the GTSAM fixed-lag smoother
// back-end: the fused camera pose in ENU plus the auxiliary states the F1
// graph estimates (scale S(k), mount azimuth θ, AGL bias b(k)). See
// .docs/theory/heading_agl_prior_tactics.md §7 for the factor table.
//
// This header is intentionally gtsam-free: only Eigen/std types cross the
// public API boundary.

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uavloc::fusion {

//! Health of the fusion estimate (kcb §2.3 state machine; F1 skeleton uses a
//! placeholder keyframe-count rule until the real convergence checks land).
enum class FusionHealth {
    INITIALIZING,
    CONVERGED,
    DRIFTING
};

//! Per-frame fused output.
struct FusionResult {
    unsigned int frame_id       = 0;
    double       timestamp_msec = 0.0;

    //! Fused camera pose in the ENU frame (camera → ENU).
    Eigen::Isometry3d T_enu_c = Eigen::Isometry3d::Identity();

    double scale      = 1.0;   //!< current metric-scale estimate S(k)
    double theta_deg  = 0.0;   //!< mount-azimuth estimate θ
    double agl_bias_m = 0.0;   //!< AGL-vs-ENU-Z bias b(k)

    FusionHealth health   = FusionHealth::INITIALIZING;
    bool         has_pose = false;

    //! Which optimized keyframe state X(k) this pose is propagated from.
    unsigned int base_keyframe_id = 0;
    //! True right after a smoother update (i.e. this frame was a keyframe).
    bool graph_updated = false;
};

} // namespace uavloc::fusion
