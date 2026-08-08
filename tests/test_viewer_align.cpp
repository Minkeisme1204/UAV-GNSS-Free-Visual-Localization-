// test_viewer_align — unit test for debug_viewer::TrajectoryAligner, the 4-DoF
// (yaw + translation) VO-world -> groundtruth-ENU fit extracted from
// tests/test_vo_viewer.cpp.
//
// Checks:
//   A1  EXACT RECOVERY of a known transform: correspondences generated with a
//       known yaw θ and translation t, plus a horizontal landmark plane below
//       the camera track, must give back exactly [Rz(θ) | t].
//   A2  COUNT GATE: with fewer than window_frames correspondences the aligner
//       is not frozen and tryFreeze() refuses; the transform stays identity.
//   A3  SPREAD GATE: a full window that is spatially concentrated does not
//       freeze while min_spread_m is above the window's horizontal extent, and
//       does freeze once the window spreads out.
//   A4  GPS -> ENU: the origin maps to (0,0,0); one degree of latitude and one
//       degree of longitude give the documented flat-earth scales, and
//       setEnuOrigin is first-call-wins. Tolerances are FLOAT-tight, not
//       double-tight: src/debug_viewer/gps_to_enu.h stores its components in
//       float, so no more than float precision may be claimed here.
//   A5  applyPoint / applyRotation agree with T() by construction, both before
//       (identity) and after the freeze.
//
// Pure synthetic unit test: no data files, no config, no GL, headless.

#include "uavloc/debug_viewer/trajectory_aligner.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace {

using uavloc::debug_viewer::TrajectoryAligner;

int g_rc = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        g_rc = 1;
        spdlog::error("FAIL: {}", what);
    }
}

// Flat-earth constants — READ FROM src/debug_viewer/gps_to_enu.h (that header
// is private to the viewer library, so the reference values are restated here
// rather than included).
constexpr double GPS_M_PER_DEG_LAT = 111319.5;
constexpr double GPS_DEG2RAD       = M_PI / 180.0;

// Tolerance for the exact-recovery checks. The fit is a closed-form 2-D
// Procrustes on noiseless input, so anything above round-off is a real defect.
constexpr double TOL_EXACT = 1e-9;
// Float-truncation tolerance (relative): gps_to_enu returns float components.
constexpr double TOL_FLOAT_REL = 1e-6;

// A ground plane of `n x n` points at z = 0 in the VO frame, spanning
// [-extent, +extent] in x and y — enough structure for a determined plane fit.
std::vector<Eigen::Vector3d> makeGroundPlane(int n, double extent) {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const double u = -extent + 2.0 * extent * i / (n - 1);
            const double v = -extent + 2.0 * extent * j / (n - 1);
            pts.emplace_back(u, v, 0.0);
        }
    }
    return pts;
}

// ── A1: exact recovery of a known yaw + translation ─────────────────────────
void test_exact_recovery() {
    const int    window = 20;
    const double yaw    = 0.7;                       // rad
    const Eigen::Vector3d t(123.5, -47.25, 9.75);    // metres
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    TrajectoryAligner aligner(TrajectoryAligner::Config{window, 0.0});
    // The landmark carpet is at z = 0 in the VO frame; the camera track sits
    // above it, so the plane normal is signed toward +Z and R0 comes out
    // identity — leaving exactly the yaw + translation to be recovered.
    aligner.setPlanePoints(makeGroundPlane(5, 200.0));

    // A curved (non-collinear) camera track so the 2-D yaw fit is determined.
    for (int i = 0; i < window; ++i) {
        const double s = static_cast<double>(i);
        const Eigen::Vector3d src(10.0 * s, 0.5 * s * s, 300.0);
        aligner.addCorrespondence(src, R * src + t);
    }

    check(aligner.readyToFreeze(), "A1: full window should be ready to freeze");
    check(aligner.tryFreeze(), "A1: tryFreeze() must return true on the freeze");
    check(aligner.isFrozen(), "A1: aligner must report frozen");
    check(!aligner.tryFreeze(), "A1: a second tryFreeze() must return false");

    Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
    expected.block<3, 3>(0, 0) = R;
    expected.block<3, 1>(0, 3) = t;
    const double err = (aligner.T() - expected).cwiseAbs().maxCoeff();
    check(err < TOL_EXACT, "A1: recovered transform must match [Rz(yaw)|t]");
    if (err >= TOL_EXACT) {
        spdlog::error("A1: max |T - expected| = {:.3e}", err);
    }

    // The landmark plane must be level after the alignment (that is the whole
    // point of locking roll/pitch to its normal).
    const double tilt = TrajectoryAligner::alignedPlaneTiltDeg(
        makeGroundPlane(5, 200.0), aligner.T());
    check(tilt >= 0.0 && tilt < 1e-6,
          "A1: aligned landmark plane must be level (tilt ~ 0 deg)");

    // Correspondences arriving after the freeze must be ignored.
    const std::size_t n_before = aligner.correspondenceCount();
    aligner.addCorrespondence(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
    check(aligner.correspondenceCount() == n_before,
          "A1: correspondences after the freeze must be ignored");
}

// ── A2: the count gate ──────────────────────────────────────────────────────
void test_count_gate() {
    const int window = 50;
    TrajectoryAligner aligner(TrajectoryAligner::Config{window, 0.0});
    aligner.setPlanePoints(makeGroundPlane(5, 200.0));

    for (int i = 0; i < window - 1; ++i) {
        const double s = static_cast<double>(i);
        const Eigen::Vector3d src(10.0 * s, 0.5 * s * s, 300.0);
        aligner.addCorrespondence(src, src + Eigen::Vector3d(1.0, 2.0, 3.0));
        check(!aligner.readyToFreeze(), "A2: must not be ready before the window is full");
        check(!aligner.tryFreeze(), "A2: tryFreeze() must refuse before the window is full");
        check(!aligner.isFrozen(), "A2: must not be frozen before the window is full");
    }
    check(aligner.T().isApprox(Eigen::Matrix4d::Identity()),
          "A2: transform must stay identity while unfrozen");
    check(aligner.correspondenceCount() == static_cast<std::size_t>(window - 1),
          "A2: correspondence count must match what was pushed");

    // The very next one completes the window.
    aligner.addCorrespondence(Eigen::Vector3d(500.0, 100.0, 300.0),
                              Eigen::Vector3d(501.0, 102.0, 303.0));
    check(aligner.tryFreeze(), "A2: tryFreeze() must succeed once the window is full");
}

// ── A3: the horizontal-spread gate ──────────────────────────────────────────
void test_spread_gate() {
    const int    window     = 10;
    const double min_spread = 100.0;   // metres
    TrajectoryAligner aligner(TrajectoryAligner::Config{window, min_spread});
    aligner.setPlanePoints(makeGroundPlane(5, 200.0));

    // A full window packed into a ~1 m box: count satisfied, spread not.
    for (int i = 0; i < window; ++i) {
        const double s = static_cast<double>(i) * 0.1;
        const Eigen::Vector3d src(s, s * s, 300.0);
        aligner.addCorrespondence(src, Eigen::Vector3d(s, s * s, 0.0));
    }
    check(!aligner.readyToFreeze(), "A3: concentrated window must not be ready");
    check(!aligner.tryFreeze(), "A3: concentrated window must not freeze");
    check(!aligner.isFrozen(), "A3: aligner must stay unfrozen under the spread gate");

    // Now walk far enough away to exceed the gate.
    for (int i = 0; i < window; ++i) {
        const double s = 200.0 + 20.0 * static_cast<double>(i);
        const Eigen::Vector3d src(s, 0.3 * s, 300.0);
        aligner.addCorrespondence(src, Eigen::Vector3d(s, 0.3 * s, 0.0));
    }
    check(aligner.readyToFreeze(), "A3: spread-out window must become ready");
    check(aligner.tryFreeze(), "A3: spread-out window must freeze");

    // And the gate's own measure agrees with the reason it opened.
    std::vector<Eigen::Vector3d> box{{0.0, 0.0, 0.0}, {3.0, 4.0, 99.0}};
    check(std::abs(TrajectoryAligner::horizontalSpreadM(box) - 5.0) < TOL_EXACT,
          "A3: horizontalSpreadM must be the (E,N) bounding-box diagonal");
    check(TrajectoryAligner::horizontalSpreadM({}) == 0.0,
          "A3: horizontalSpreadM of an empty set is 0");
}

// ── A4: flat-earth GPS -> ENU ───────────────────────────────────────────────
void test_gps_to_enu() {
    const double lat0 = 21.702056;
    const double lon0 = 104.849973;
    const double alt0 = 250.0;

    TrajectoryAligner aligner;
    check(!aligner.hasEnuOrigin(), "A4: a fresh aligner has no ENU origin");
    check(aligner.gpsToEnu(lat0, lon0, alt0).isZero(),
          "A4: without an origin the conversion returns zero");

    aligner.setEnuOrigin(lat0, lon0, alt0);
    check(aligner.hasEnuOrigin(), "A4: origin must be set");

    // The origin itself maps to (0, 0, 0), exactly.
    const Eigen::Vector3d o = aligner.gpsToEnu(lat0, lon0, alt0);
    check(o.x() == 0.0 && o.y() == 0.0 && o.z() == 0.0,
          "A4: the origin must map to exactly (0,0,0)");

    // One degree of latitude -> M_LAT metres North (exactly representable in
    // float, so this one is exact).
    const Eigen::Vector3d north = aligner.gpsToEnu(lat0 + 1.0, lon0, alt0);
    check(north.y() == static_cast<float>(GPS_M_PER_DEG_LAT),
          "A4: one degree of latitude must be M_LAT metres North");
    check(north.x() == 0.0, "A4: a pure latitude step has no East component");

    // One degree of longitude -> M_LAT * cos(lat0) metres East. The scale is
    // computed in double and truncated to float, so compare both ways.
    // Checked to FLOAT precision, not bit-exactly, for two reasons: the
    // compiler constant-folds cos() here while the library calls libm at run
    // time (up to 1 ULP apart), and gps_to_enu stores its components in float
    // — whether that narrowing survives optimisation is a compiler decision
    // this test must not pin down.
    const double expect_e = GPS_M_PER_DEG_LAT * std::cos(lat0 * GPS_DEG2RAD);
    const Eigen::Vector3d east = aligner.gpsToEnu(lat0, lon0 + 1.0, alt0);
    check(std::abs(east.x() - expect_e) / expect_e < TOL_FLOAT_REL,
          "A4: one degree of longitude must be M_LAT*cos(lat0) metres East");
    check(east.y() == 0.0, "A4: a pure longitude step has no North component");

    // Altitude is a plain difference.
    const Eigen::Vector3d up = aligner.gpsToEnu(lat0, lon0, alt0 + 37.5);
    check(up.z() == 37.5, "A4: Up is the raw altitude difference");

    // First call wins: a second origin must not move the frame.
    aligner.setEnuOrigin(lat0 + 5.0, lon0 + 5.0, 0.0);
    check(aligner.gpsToEnu(lat0, lon0, alt0).isZero(),
          "A4: setEnuOrigin must be ignored after the first call");
}

// ── A5: applyPoint / applyRotation vs T() ───────────────────────────────────
void test_apply_consistency() {
    const Eigen::Vector3d p(31.0, -12.0, 4.5);
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(0.31, Eigen::Vector3d(0.2, -0.7, 0.6).normalized())
            .toRotationMatrix();

    // Before the freeze the transform is identity, so both are no-ops.
    TrajectoryAligner fresh;
    check(fresh.applyPoint(p).isApprox(p),
          "A5: applyPoint is the identity before the freeze");
    check(fresh.applyRotation(R).isApprox(R),
          "A5: applyRotation is the identity before the freeze");

    // After the freeze both must agree with T() exactly.
    const int    window = 20;
    const double yaw    = -1.15;
    const Eigen::Vector3d t(7.0, 800.0, -3.5);
    const Eigen::Matrix3d Rz =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    TrajectoryAligner aligner(TrajectoryAligner::Config{window, 0.0});
    aligner.setPlanePoints(makeGroundPlane(5, 200.0));
    for (int i = 0; i < window; ++i) {
        const double s = static_cast<double>(i);
        const Eigen::Vector3d src(8.0 * s, 0.4 * s * s, 250.0);
        aligner.addCorrespondence(src, Rz * src + t);
    }
    check(aligner.tryFreeze(), "A5: the window must freeze");

    const Eigen::Vector3d by_api = aligner.applyPoint(p);
    const Eigen::Vector3d by_mat =
        aligner.T().block<3, 3>(0, 0) * p + aligner.T().block<3, 1>(0, 3);
    check((by_api - by_mat).cwiseAbs().maxCoeff() < TOL_EXACT,
          "A5: applyPoint must equal R*p + t of T()");

    const Eigen::Matrix3d rot_api = aligner.applyRotation(R);
    const Eigen::Matrix3d rot_mat = aligner.T().block<3, 3>(0, 0) * R;
    check((rot_api - rot_mat).cwiseAbs().maxCoeff() < TOL_EXACT,
          "A5: applyRotation must equal R_align * R");
}

} // namespace

int main() {
    spdlog::set_level(spdlog::level::info);

    test_exact_recovery();
    test_count_gate();
    test_spread_gate();
    test_gps_to_enu();
    test_apply_consistency();

    if (g_rc != 0) {
        spdlog::error("test_viewer_align: FAIL");
        return g_rc;
    }
    spdlog::info("test_viewer_align: PASS");
    return g_rc;
}
