#pragma once

// TrajectoryAligner — the 4-DoF (yaw + translation) VO-world -> groundtruth-ENU
// rigid transform used by the overlay drivers, plus the flat-earth GPS -> ENU
// conversion that produces the groundtruth side of every correspondence.
//
// Why 4 DoF and not a free Umeyama: on a near-collinear camera window (a UAV
// flying roughly straight) the roll about the flight axis is unconstrained, so
// a free 3-D rotation tilts the whole map. Roll/pitch are therefore locked by
// the normal of the landmark carpet (the ground plane), and only the yaw about
// Up plus the translation are fitted. Scale is fixed at 1 — VO is metric, so a
// residual scale error must stay visible as overlay drift instead of being
// absorbed silently.
//
// DEPENDENCY DISCIPLINE: this class is Eigen + built-in types only. It knows
// nothing about vo::VOResult, sensor::TelemetryData or cv::Mat. That rule
// originally came from old R4 ("uavloc_debug_viewer must not link libuavloc"),
// which no longer holds — the viewer is a MODULE of libuavloc since 2026-08-08
// (see the revised R4 in CLAUDE.md). The rule is kept anyway because it is what
// makes this class unit-testable without a pipeline (tests/test_viewer_align.cpp)
// and reusable by any driver. SystemBridge unwraps the uavloc types and feeds
// plain vectors in.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace uavloc::debug_viewer {

class TrajectoryAligner {
public:
    struct Config {
        //! Number of correspondences accumulated before the transform may be
        //! frozen. Must be >= 1.
        int    window_frames = 50;
        //! Optional freeze gate: when > 0, freezing is delayed until the
        //! groundtruth window spans at least this many metres horizontally
        //! (guards the yaw fit against a near-stationary window). 0 = off.
        double min_spread_m  = 0.0;
    };

    TrajectoryAligner() = default;
    explicit TrajectoryAligner(const Config& cfg);

    // ── Groundtruth side: flat-earth GPS -> ENU ─────────────────────────────
    //! Anchor the ENU frame. The FIRST call wins for a given run: re-anchoring
    //! mid-flight would move every previously emitted point.
    void setEnuOrigin(double lat0, double lon0, double alt0);
    bool hasEnuOrigin() const { return has_origin_; }
    //! Flat-earth conversion against the anchored origin. Returns the zero
    //! vector when no origin has been set.
    //!
    //! Backed by the historical float-component ENUPoint, widened to double
    //! here. Do not claim more than float precision from the result, and do
    //! not "clean up" the conversion: it is inside a byte-identical
    //! regression gate (the UAVLOC_VIEWER_DUMP pose dump).
    Eigen::Vector3d gpsToEnu(double lat, double lon, double alt) const;

    // ── Fitting ─────────────────────────────────────────────────────────────
    //! Append one (VO world position, groundtruth ENU position) pair. Ignored
    //! once the transform is frozen.
    void addCorrespondence(const Eigen::Vector3d& src_vo,
                           const Eigen::Vector3d& dst_enu);
    //! Replace the landmark cloud (VO world, metres) whose best-fit plane
    //! supplies the "up" direction. Needs >= 3 points to be usable; with fewer
    //! the freeze falls back to a free 3-D Umeyama.
    void setPlanePoints(const std::vector<Eigen::Vector3d>& pts_vo);

    //! True when the accumulated window satisfies both the count and the
    //! spread gate — i.e. tryFreeze() would fit the transform.
    bool readyToFreeze() const;
    //! Fit and freeze the transform. Returns true EXACTLY on the call that
    //! froze it: false if it was already frozen, and false if readyToFreeze()
    //! is not yet satisfied.
    bool tryFreeze();
    bool isFrozen() const { return frozen_; }

    //! Number of correspondences accumulated so far.
    std::size_t correspondenceCount() const { return src_.size(); }

    // ── Applying ────────────────────────────────────────────────────────────
    //! The frozen transform; identity until tryFreeze() succeeds.
    const Eigen::Matrix4d& T() const { return T_align_; }
    Eigen::Vector3d applyPoint(const Eigen::Vector3d& p_vo) const;
    Eigen::Matrix3d applyRotation(const Eigen::Matrix3d& R_vo) const;

    // ── Diagnostics (also used by the unit test) ────────────────────────────
    //! Least-squares plane through `pts`: centroid + unit normal (the singular
    //! vector of the smallest singular value of the centred 3xN matrix).
    //! False when fewer than 3 points are available (plane underdetermined).
    static bool fitPlane(const std::vector<Eigen::Vector3d>& pts,
                         Eigen::Vector3d& centroid, Eigen::Vector3d& normal);
    //! Tilt (degrees, folded to <= 90) between ENU Up and the normal of the
    //! best-fit plane through `pts` AFTER applying `T`. Negative when the plane
    //! cannot be fitted.
    static double alignedPlaneTiltDeg(const std::vector<Eigen::Vector3d>& pts,
                                      const Eigen::Matrix4d& T);
    //! Horizontal (E,N) bounding-box diagonal of a point set, in metres — the
    //! quantity the min_spread_m gate compares against.
    static double horizontalSpreadM(const std::vector<Eigen::Vector3d>& pts);

private:
    Config config_{};

    bool   has_origin_ = false;
    double lat0_       = 0.0;
    double lon0_       = 0.0;
    double alt0_       = 0.0;

    std::vector<Eigen::Vector3d> src_;         // VO world positions
    std::vector<Eigen::Vector3d> dst_;         // matching groundtruth ENU
    std::vector<Eigen::Vector3d> plane_pts_;   // landmark cloud (VO world)

    Eigen::Matrix4d T_align_ = Eigen::Matrix4d::Identity();
    bool            frozen_  = false;
};

} // namespace uavloc::debug_viewer
