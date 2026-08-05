// Extrapolator implementation (S6b) — see include/uavloc/core/extrapolator.h
// for the four rules this file exists to enforce, and
// .docs/designs/system_manager_design.md §3.3 / §3.4 for why they are what they
// are.

#include "uavloc/core/extrapolator.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <mutex>

namespace uavloc {
namespace core {

namespace {

constexpr double MSEC_PER_SEC = 1000.0;
constexpr double PI           = 3.14159265358979323846;
constexpr double DEG_TO_RAD   = PI / 180.0;
constexpr double RAD_TO_DEG   = 180.0 / PI;

//! One buffered sample: the timestamp is stored next to the payload because the
//! payload types (sensor::AttitudeData & co) deliberately do not carry one.
template <typename T>
struct TimedSample {
    double t_msec = 0.0;
    T      payload{};
};

//! One channel: a time-sorted deque plus the mutex that guards it. Per-channel
//! locking (never two at once) means the three channels cannot contend, and
//! there is no lock ordering to get wrong.
template <typename T>
struct Channel {
    mutable std::mutex          mutex;
    std::deque<TimedSample<T>>  samples;
};

//! What a lookup found around the requested instant.
enum class Lookup {
    MISS,     //!< empty buffer, or the instant is outside the buffered interval
    EXACT,    //!< a sample with EXACTLY that timestamp exists
    BRACKET   //!< the instant lies strictly between two samples
};

//! Insert `payload` at `t_msec`, keeping the deque sorted by time, then drop
//! whatever fell out of `span_msec`. `name` only labels the log lines.
template <typename T>
void addSample(Channel<T>& ch, double span_msec, double t_msec, const T& payload,
               const char* name) {
    if (!std::isfinite(t_msec)) {
        spdlog::warn("Extrapolator: {} sample with non-finite timestamp refused", name);
        return;
    }

    std::lock_guard<std::mutex> lock(ch.mutex);
    auto& s = ch.samples;

    // Too old to survive the trim below — refuse it instead of inserting and
    // immediately erasing it.
    if (span_msec > 0.0 && !s.empty() && t_msec < s.back().t_msec - span_msec) {
        spdlog::debug("Extrapolator: {} sample at {:.3f} ms is older than the {:.0f} ms "
                      "buffer span (newest {:.3f} ms), refused",
                      name, t_msec, span_msec, s.back().t_msec);
        return;
    }

    if (!s.empty() && t_msec <= s.back().t_msec) {
        // Out-of-order (or duplicate) arrival: find where it belongs.
        auto it = std::lower_bound(s.begin(), s.end(), t_msec,
                                   [](const TimedSample<T>& e, double t) { return e.t_msec < t; });
        if (it != s.end() && it->t_msec == t_msec) {
            // Rule ①: an exact query must have ONE candidate, so a repeated
            // timestamp overwrites rather than piling up.
            spdlog::warn("Extrapolator: {} sample at {:.3f} ms already buffered, replaced",
                         name, t_msec);
            it->payload = payload;
            return;
        }
        spdlog::debug("Extrapolator: {} sample at {:.3f} ms arrived out of order "
                      "(newest {:.3f} ms), inserted in place",
                      name, t_msec, s.back().t_msec);
        s.insert(it, TimedSample<T>{t_msec, payload});
    } else {
        s.push_back(TimedSample<T>{t_msec, payload});
    }

    if (span_msec > 0.0) {
        const double horizon = s.back().t_msec - span_msec;
        while (!s.empty() && s.front().t_msec < horizon) {
            s.pop_front();
        }
    }
}

//! Locate `t_msec` in `ch`. On EXACT, `lo` holds the stored sample. On BRACKET,
//! `lo` / `hi` hold the neighbours and `alpha` the position between them
//! (0 at `lo`, 1 at `hi`). `max_gap_msec` > 0 turns a bracket wider than itself
//! into a MISS: the neighbours are too far apart for the value between them to
//! be a measurement rather than an invention.
template <typename T>
Lookup lookupSample(const Channel<T>& ch, double t_msec, double max_gap_msec, T& lo, T& hi,
                    double& alpha) {
    if (!std::isfinite(t_msec)) {
        return Lookup::MISS;
    }

    std::lock_guard<std::mutex> lock(ch.mutex);
    const auto& s = ch.samples;
    if (s.empty()) {
        return Lookup::MISS;
    }

    auto it = std::lower_bound(s.begin(), s.end(), t_msec,
                               [](const TimedSample<T>& e, double t) { return e.t_msec < t; });

    // Exact hit: bit-for-bit equality, no epsilon (rule ①).
    if (it != s.end() && it->t_msec == t_msec) {
        lo = it->payload;
        return Lookup::EXACT;
    }

    // Outside the buffered interval — no extrapolation (rule ③).
    if (it == s.end() || it == s.begin()) {
        return Lookup::MISS;
    }

    const auto& hi_s = *it;
    const auto& lo_s = *(it - 1);
    if (max_gap_msec > 0.0 && (hi_s.t_msec - lo_s.t_msec) > max_gap_msec) {
        // A hole in the channel, not a sampling interval. Report a MISS and let
        // the caller decide (SystemManager builds a frame with no telemetry
        // rather than one with an invented attitude).
        spdlog::debug("Extrapolator: query at {:.3f} ms falls in a {:.3f} ms gap "
                      "({:.3f} -> {:.3f} ms), wider than the {:.0f} ms limit — refused",
                      t_msec, hi_s.t_msec - lo_s.t_msec, lo_s.t_msec, hi_s.t_msec,
                      max_gap_msec);
        return Lookup::MISS;
    }
    lo    = lo_s.payload;
    hi    = hi_s.payload;
    alpha = (t_msec - lo_s.t_msec) / (hi_s.t_msec - lo_s.t_msec);
    return Lookup::BRACKET;
}

template <typename T>
std::size_t channelCount(const Channel<T>& ch) {
    std::lock_guard<std::mutex> lock(ch.mutex);
    return ch.samples.size();
}

template <typename T>
void channelClear(Channel<T>& ch) {
    std::lock_guard<std::mutex> lock(ch.mutex);
    ch.samples.clear();
}

double lerp(double a, double b, double alpha) {
    return a + (b - a) * alpha;
}

//! ZYX (yaw→pitch→roll) chain — the convention of sensor::AttitudeData.
Eigen::Quaterniond quatFromAttitude(const sensor::AttitudeData& a) {
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(a.yaw_deg * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(a.pitch_deg * DEG_TO_RAD, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(a.roll_deg * DEG_TO_RAD, Eigen::Vector3d::UnitX()));
}

//! Inverse of quatFromAttitude. Hand-written rather than Eigen::eulerAngles()
//! because the latter returns the first angle in [0, π], which for a ZYX chain
//! puts YAW there and flips the other two — not the range a consumer of
//! AttitudeData expects.
sensor::AttitudeData attitudeFromQuat(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    sensor::AttitudeData a;
    a.pitch_deg = std::asin(std::clamp(-R(2, 0), -1.0, 1.0)) * RAD_TO_DEG;
    a.yaw_deg   = std::atan2(R(1, 0), R(0, 0)) * RAD_TO_DEG;
    a.roll_deg  = std::atan2(R(2, 1), R(2, 2)) * RAD_TO_DEG;
    return a;
}

//! ZY (pan→tilt) chain — the convention of sensor::GimbalData.
Eigen::Quaterniond quatFromGimbal(const sensor::GimbalData& g) {
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(g.pan_deg * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(g.tilt_deg * DEG_TO_RAD, Eigen::Vector3d::UnitY()));
}

//! Inverse of quatFromGimbal. With only two angles the middle column of R is
//! untouched by the tilt rotation, so pan comes out of that column and stays
//! well-conditioned even at tilt = 90° (nadir — our normal operating point,
//! where a naive extraction from the first column would divide by cos(tilt)).
sensor::GimbalData gimbalFromQuat(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    sensor::GimbalData g;
    g.pan_deg  = std::atan2(-R(0, 1), R(1, 1)) * RAD_TO_DEG;
    g.tilt_deg = std::atan2(-R(2, 0), R(2, 2)) * RAD_TO_DEG;
    return g;
}

} // namespace

class Extrapolator::Impl {
public:
    Impl(double buffer_span_sec, double max_gap_sec)
        : span_msec_(buffer_span_sec > 0.0 ? buffer_span_sec * MSEC_PER_SEC : 0.0),
          max_gap_msec_(max_gap_sec > 0.0 ? max_gap_sec * MSEC_PER_SEC : 0.0) {
        if (buffer_span_sec < 0.0) {
            spdlog::warn("Extrapolator: negative buffer_span_sec ({:.3f}) treated as unbounded",
                         buffer_span_sec);
        } else if (buffer_span_sec == 0.0) {
            spdlog::warn("Extrapolator: buffer_span_sec = 0 — buffers are UNBOUNDED and will "
                         "grow for the whole flight");
        } else {
            spdlog::debug("Extrapolator: buffer span {:.0f} ms", span_msec_);
        }
        if (max_gap_msec_ > 0.0) {
            spdlog::debug("Extrapolator: interpolation limited to gaps <= {:.0f} ms",
                          max_gap_msec_);
        }
    }

    double span_msec() const { return span_msec_; }
    double max_gap_msec() const { return max_gap_msec_; }

    Channel<sensor::AttitudeData> attitude;
    Channel<sensor::GimbalData>   gimbal;
    Channel<sensor::GnssData>     gnss;

private:
    //! 0 = unbounded.
    double span_msec_ = 0.0;
    //! 0 = no limit (the S6b behaviour).
    double max_gap_msec_ = 0.0;
};

Extrapolator::Extrapolator(double buffer_span_sec, double max_gap_sec)
    : impl_(std::make_unique<Impl>(buffer_span_sec, max_gap_sec)) {}

Extrapolator::~Extrapolator() = default;

void Extrapolator::addAttitude(double t_msec, const sensor::AttitudeData& att) {
    addSample(impl_->attitude, impl_->span_msec(), t_msec, att, "attitude");
}

void Extrapolator::addGimbal(double t_msec, const sensor::GimbalData& gim) {
    addSample(impl_->gimbal, impl_->span_msec(), t_msec, gim, "gimbal");
}

void Extrapolator::addGnss(double t_msec, const sensor::GnssData& gnss) {
    addSample(impl_->gnss, impl_->span_msec(), t_msec, gnss, "gnss");
}

namespace {

//! Report the rule that produced a result, when the caller asked for it.
void setOrigin(SampleOrigin* origin, SampleOrigin value) {
    if (origin != nullptr) {
        *origin = value;
    }
}

} // namespace

bool Extrapolator::getAttitude(double t_msec, sensor::AttitudeData& out,
                               SampleOrigin* origin) const {
    sensor::AttitudeData lo, hi;
    double               alpha = 0.0;
    switch (lookupSample(impl_->attitude, t_msec, impl_->max_gap_msec(), lo, hi, alpha)) {
        case Lookup::EXACT:
            out = lo;
            setOrigin(origin, SampleOrigin::EXACT);
            return true;
        case Lookup::BRACKET:
            out = attitudeFromQuat(quatFromAttitude(lo).slerp(alpha, quatFromAttitude(hi)));
            setOrigin(origin, SampleOrigin::INTERPOLATED);
            return true;
        case Lookup::MISS:
        default:
            return false;
    }
}

bool Extrapolator::getGimbal(double t_msec, sensor::GimbalData& out,
                             SampleOrigin* origin) const {
    sensor::GimbalData lo, hi;
    double             alpha = 0.0;
    switch (lookupSample(impl_->gimbal, t_msec, impl_->max_gap_msec(), lo, hi, alpha)) {
        case Lookup::EXACT:
            out = lo;
            setOrigin(origin, SampleOrigin::EXACT);
            return true;
        case Lookup::BRACKET:
            out = gimbalFromQuat(quatFromGimbal(lo).slerp(alpha, quatFromGimbal(hi)));
            setOrigin(origin, SampleOrigin::INTERPOLATED);
            return true;
        case Lookup::MISS:
        default:
            return false;
    }
}

bool Extrapolator::getGnss(double t_msec, sensor::GnssData& out,
                           SampleOrigin* origin) const {
    sensor::GnssData lo, hi;
    double           alpha = 0.0;
    switch (lookupSample(impl_->gnss, t_msec, impl_->max_gap_msec(), lo, hi, alpha)) {
        case Lookup::EXACT:
            out = lo;
            setOrigin(origin, SampleOrigin::EXACT);
            return true;
        case Lookup::BRACKET: {
            sensor::GnssData r;
            r.latitude_deg  = lerp(lo.latitude_deg, hi.latitude_deg, alpha);
            r.longitude_deg = lerp(lo.longitude_deg, hi.longitude_deg, alpha);
            r.altitude_m    = lerp(lo.altitude_m, hi.altitude_m, alpha);
            r.speed_mps     = lerp(lo.speed_mps, hi.speed_mps, alpha);
            out             = r;
            setOrigin(origin, SampleOrigin::INTERPOLATED);
            return true;
        }
        case Lookup::MISS:
        default:
            return false;
    }
}

void Extrapolator::clear() {
    channelClear(impl_->attitude);
    channelClear(impl_->gimbal);
    channelClear(impl_->gnss);
}

std::size_t Extrapolator::attitudeCount() const {
    return channelCount(impl_->attitude);
}

std::size_t Extrapolator::gimbalCount() const {
    return channelCount(impl_->gimbal);
}

std::size_t Extrapolator::gnssCount() const {
    return channelCount(impl_->gnss);
}

} // namespace core
} // namespace uavloc
