#pragma once
#include <cmath>

namespace uavloc::debug_viewer {

constexpr double DEG2RAD = M_PI / 180.0;

struct ENUPoint { float e, n, u; };

inline ENUPoint gps_to_enu(double lat, double lon, double alt,
                            double lat0, double lon0, double alt0)
{
    constexpr double M_LAT   = 111319.5;   // metres per degree latitude
    const     double m_lon   = 111319.5 * std::cos(lat0 * DEG2RAD);
    return {
        static_cast<float>((lon - lon0) * m_lon),
        static_cast<float>((lat - lat0) * M_LAT),
        static_cast<float>(alt - alt0)
    };
}

} // namespace uavloc::debug_viewer
