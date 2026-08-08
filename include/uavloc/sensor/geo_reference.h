#pragma once

// GeoReferencer — converts a position expressed in the VO world frame (the local,
// gravity-unaware frame anchored at the first geo-referenced camera pose) into
// geographic latitude / longitude / altitude, and back.
//
// The mapping is:  p_enu = R_enu_w * pos_vo   (metres, East-North-Up)
// followed by a flat-earth inverse anchored at (lat0, lon0, alt0). The flat-earth
// constants match src/debug_viewer/gps_to_enu.h so both code paths agree on ENU
// metres (valid for trajectories up to a few kilometres near the anchor).
//
// This is pure geometry with NO policy (no origin selection, no relocalization);
// the anchor module (Stage 2) wraps it with that policy. Header-only so both the
// library and the tests can use it without a link dependency.

#include <cmath>

#include <Eigen/Core>

namespace uavloc::sensor {

// A geographic point: WGS-84 latitude/longitude in degrees, altitude in metres.
struct LatLonAlt {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
};

class GeoReferencer {
public:
    // Metres-per-degree of latitude (flat-earth), shared with gps_to_enu.h.
    static constexpr double METERS_PER_DEG_LAT = 111319.5;

    // Anchor the local VO world frame at (lat0, lon0, alt0) with the geographic
    // view azimuth psi_deg (0 = North, clockwise), i.e. airframe heading + gimbal
    // pan. Builds R_enu_w = [world VO -> ENU]. Columns follow the near-nadir
    // convention: world X -> (cosψ, -sinψ, 0), world Y -> (-sinψ, -cosψ, 0),
    // world Z -> (0, 0, -1) (VO optical/forward axis points down into the ground).
    void init(double lat0, double lon0, double alt0, double psi_deg) {
        lat0_          = lat0;
        lon0_          = lon0;
        alt0_          = alt0;
        m_per_deg_lon_ = METERS_PER_DEG_LAT * std::cos(lat0 * DEG2RAD);

        const double psi = psi_deg * DEG2RAD;
        const double c   = std::cos(psi);
        const double s   = std::sin(psi);
        R_enu_w_.col(0) = Eigen::Vector3d(c, -s, 0.0);
        R_enu_w_.col(1) = Eigen::Vector3d(-s, -c, 0.0);
        R_enu_w_.col(2) = Eigen::Vector3d(0.0, 0.0, -1.0);

        initialized_ = true;
    }

    bool initialized() const { return initialized_; }

    // World VO -> ENU rotation (metres in, metres out). Identity until init().
    const Eigen::Matrix3d& R_enu_w() const { return R_enu_w_; }

    // Forward flat-earth: a geographic point -> ENU metres relative to the anchor.
    Eigen::Vector3d enu(double lat, double lon, double alt) const {
        return Eigen::Vector3d((lon - lon0_) * m_per_deg_lon_,
                               (lat - lat0_) * METERS_PER_DEG_LAT,
                               alt - alt0_);
    }

    // Pure inverse of enu(): ENU metres (relative to the anchor) -> lat/lon/alt.
    // Use this for a position that is ALREADY in the ENU frame — e.g.
    // fusion::FusionResult::T_enu_c, whose translation the back-end produced in
    // ENU directly. It does NOT apply R_enu_w, unlike latlon(), which expects a
    // position in the VO world frame and would rotate an ENU input a second time.
    LatLonAlt latlon_from_enu(const Eigen::Vector3d& p_enu) const {
        LatLonAlt out;
        out.lon = lon0_ + p_enu.x() / m_per_deg_lon_;
        out.lat = lat0_ + p_enu.y() / METERS_PER_DEG_LAT;
        out.alt = alt0_ + p_enu.z();
        return out;
    }

    // Inverse: a VO world position -> geographic latitude / longitude / altitude.
    LatLonAlt latlon(const Eigen::Vector3d& pos_vo) const {
        const Eigen::Vector3d p_enu = R_enu_w_ * pos_vo;  // E, N, U in metres
        LatLonAlt out;
        out.lon = lon0_ + p_enu.x() / m_per_deg_lon_;
        out.lat = lat0_ + p_enu.y() / METERS_PER_DEG_LAT;
        out.alt = alt0_ + p_enu.z();
        return out;
    }

private:
    static constexpr double DEG2RAD = M_PI / 180.0;

    double lat0_          = 0.0;
    double lon0_          = 0.0;
    double alt0_          = 0.0;
    double m_per_deg_lon_ = METERS_PER_DEG_LAT;

    Eigen::Matrix3d R_enu_w_ = Eigen::Matrix3d::Identity();
    bool            initialized_ = false;
};

}  // namespace uavloc::sensor
