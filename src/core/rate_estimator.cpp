#include "uavloc/core/rate_estimator.h"

#include <spdlog/spdlog.h>

#include <algorithm>

namespace uavloc {
namespace core {

namespace {

//! Seconds between two steady_clock instants, as a plain double.
double seconds_between(RateEstimator::TimePoint from, RateEstimator::TimePoint to) {
    return std::chrono::duration<double>(to - from).count();
}

} // namespace

RateEstimator::RateEstimator(double window_sec) : window_sec_(window_sec) {
    if (window_sec_ <= 0.0) {
        spdlog::warn("RateEstimator: window_sec = {} is not a usable window "
                     "(it would make every rate a division by zero) — falling "
                     "back to {} s",
                     window_sec, MIN_WINDOW_SEC);
        window_sec_ = MIN_WINDOW_SEC;
    }
}

void RateEstimator::add(TimePoint t, double dt_sec) {
    if (!started_) {
        first_add_ = t;
        started_   = true;
    }
    samples_.emplace_back(t, dt_sec);

    // Evict everything older than one window before the sample just added, so
    // the deque stays bounded by the arrival rate times the window.
    const TimePoint cutoff =
        t - std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(window_sec_));
    while (!samples_.empty() && samples_.front().first < cutoff) {
        samples_.pop_front();
    }
}

RateEstimator::Sample RateEstimator::snapshot(TimePoint now) const {
    Sample out;
    if (!started_) {
        return out;  // {0, 0} — nothing has ever been processed
    }

    const TimePoint cutoff =
        now - std::chrono::duration_cast<Clock::duration>(
                  std::chrono::duration<double>(window_sec_));

    std::size_t count  = 0;
    double      sum_dt = 0.0;
    for (const auto& s : samples_) {
        if (s.first >= cutoff) {
            ++count;
            sum_dt += s.second;
        }
    }

    // The denominator is min(window, elapsed-since-first-frame), NOT the window
    // alone: during the first window_sec of a run the window is only partially
    // filled, and dividing by the full window would report a rate several times
    // too low exactly when someone is watching the numbers come up. The price is
    // that the first fraction of a second is jumpy (few samples over a tiny
    // elapsed time); it settles within one window.
    const double elapsed = seconds_between(first_add_, now);
    const double denom   = std::min(window_sec_, elapsed);

    out.fps = (denom > 0.0) ? static_cast<double>(count) / denom : 0.0;
    out.proc_ms_mean =
        (count > 0) ? 1000.0 * sum_dt / static_cast<double>(count) : 0.0;
    return out;
}

double RateEstimator::window_sec() const {
    return window_sec_;
}

} // namespace core
} // namespace uavloc
