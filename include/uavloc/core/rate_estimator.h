#pragma once

// RateEstimator — a trailing-window throughput estimator for the LIVE figures
// published in core::SystemStats (fps_windowed / proc_ms_mean).
//
// Why it exists: SystemStats::fps_processed is a CUMULATIVE mean since start
// (frames_processed / total pipeline seconds). It converges and then freezes,
// so it can neither react to a slowdown nor show a stall. This class answers a
// different question — "how many frames went through in the LAST window_sec
// seconds of WALL CLOCK?" — which is the one a live readout needs.
//
// ── The stall property (the reason for the const snapshot) ───────────────────
// snapshot() takes `now` from the caller and does NOT mutate anything: it only
// counts the samples that are still inside [now - window_sec, now]. Eviction
// happens in add(). Consequently, when the pipeline stalls, add() stops being
// called, but every later snapshot() sees fewer and fewer in-window samples and
// the reported rate DECAYS toward 0 instead of freezing at its last value. That
// decay is the whole point — do not "optimise" it away by caching the last
// computed rate.
//
// ── Threading ───────────────────────────────────────────────────────────────
// NOT thread-safe by itself, on purpose: the owner already holds a lock around
// its counters and a second one here would be pure overhead. core::SystemManager
// serialises every add()/snapshot() call with stats_mutex_.

#include <chrono>
#include <deque>
#include <utility>

namespace uavloc {
namespace core {

class RateEstimator {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    //! Fallback window used when a non-positive window_sec is configured [s].
    static constexpr double MIN_WINDOW_SEC = 1.0;

    struct Sample {
        double fps          = 0.0;  //!< frames/s over the trailing window
        double proc_ms_mean = 0.0;  //!< mean pipeline time per frame in the window [ms]
    };

    explicit RateEstimator(double window_sec);

    //! Record one processed frame: its completion instant and the pipeline time
    //! it took. Also evicts everything older than one window before `t`.
    void add(TimePoint t, double dt_sec);

    //! Derived figures as of `now`. const — it does NOT evict (eviction happens
    //! in add()), so a stalled pipeline keeps decaying toward 0 on every call.
    Sample snapshot(TimePoint now) const;

    double window_sec() const;

private:
    double window_sec_;

    bool      started_ = false;
    TimePoint first_add_{};

    //! (completion instant, pipeline seconds) of every frame still in window.
    std::deque<std::pair<TimePoint, double>> samples_;
};

} // namespace core
} // namespace uavloc
