// test_rate_estimator — unit test for core::RateEstimator, the trailing-window
// throughput estimator behind SystemStats::fps_windowed / proc_ms_mean.
//
// Why this test exists: the live HUD readout must do the one thing the existing
// cumulative figure (SystemStats::fps_processed) cannot — REACT. A cumulative
// mean converges and then freezes, so a stalled pipeline still reads whatever
// it last converged to. The whole design rests on the windowed rate decaying to
// 0 instead, and that property is pure arithmetic over time points, so it can be
// checked exactly, with no dataset, no threads and no real sleeps.
//
// Checks:
//   R1  a fresh estimator reports 0 fps and 0 ms (never a divide-by-zero NaN)
//   R2  a steady 10 Hz stream reads ~10 fps and the mean pipeline time fed in
//   R3  STALL: a snapshot two windows after the last frame reads EXACTLY 0
//   R4  warm-up: within the first window the denominator is the ELAPSED time,
//       not the full window (otherwise the run would start out reading ~1/3 of
//       the true rate)
//   R5  a single frame does not blow the division up
//   R6  a burst inside one window cannot report more than count / window_sec
//   R7  eviction stays bounded: far more than a window's worth of frames still
//       yields the steady-state rate at the last frame and 0 far in the future
//
// Every time point is SYNTHETIC (Clock::time_point{} + milliseconds), so the
// test is deterministic and instantaneous.

#include "uavloc/core/rate_estimator.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>

namespace {

using uavloc::core::RateEstimator;
using Clock     = RateEstimator::Clock;
using TimePoint = RateEstimator::TimePoint;

int g_rc = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        g_rc = 1;
        spdlog::error("FAIL: {}", what);
    }
}

//! Synthetic instant: `ms` milliseconds after an arbitrary fixed epoch.
TimePoint at_ms(long long ms) {
    return TimePoint{} + std::chrono::milliseconds(ms);
}

bool near(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

} // namespace

int main() {
    spdlog::set_level(spdlog::level::info);

    // ── R1: nothing recorded yet ────────────────────────────────────────────
    {
        RateEstimator est(3.0);
        const RateEstimator::Sample s = est.snapshot(at_ms(1234));
        check(s.fps == 0.0, "R1: an empty estimator must report 0 fps");
        check(s.proc_ms_mean == 0.0, "R1: an empty estimator must report 0 ms/frame");
        check(est.window_sec() == 3.0, "R1: window_sec() must return the configured window");
    }

    // ── R2: steady 10 Hz for 10 s, window 3 s ───────────────────────────────
    // Also the fixture R3 and R7 reuse, so it is built once here.
    constexpr double WINDOW_SEC   = 3.0;
    constexpr long long PERIOD_MS = 100;    // 10 Hz
    constexpr long long SPAN_MS   = 10000;  // 10 s
    constexpr double PROC_SEC     = 0.037;  // constant pipeline time per frame
    {
        RateEstimator est(WINDOW_SEC);
        for (long long t = 0; t <= SPAN_MS; t += PERIOD_MS) {
            est.add(at_ms(t), PROC_SEC);
        }
        const RateEstimator::Sample s = est.snapshot(at_ms(SPAN_MS));
        check(near(s.fps, 10.0, 0.5),
              "R2: a steady 10 Hz stream must read ~10 fps over the window");
        check(near(s.proc_ms_mean, 1000.0 * PROC_SEC, 1.0),
              "R2: proc_ms_mean must reproduce the constant pipeline time fed in");

        // ── R3: THE STALL PROPERTY ──────────────────────────────────────────
        // Nothing was added after SPAN_MS. Two windows later the trailing window
        // is empty, so the rate must read exactly 0 — a frozen last value here
        // would defeat the entire purpose of the windowed figure.
        const RateEstimator::Sample stalled =
            est.snapshot(at_ms(SPAN_MS + static_cast<long long>(2 * WINDOW_SEC * 1000)));
        check(stalled.fps == 0.0,
              "R3: a stalled pipeline must decay to EXACTLY 0 fps, not freeze at "
              "its last reading");
        check(stalled.proc_ms_mean == 0.0,
              "R3: a stalled pipeline reports no pipeline time either");

        // ── R7: eviction stays bounded ──────────────────────────────────────
        // 101 frames were fed, far more than the ~31 a 3 s window holds at
        // 10 Hz. The estimator has no size() to inspect, so the observable
        // proxy is that it still behaves exactly like the steady state.
        const RateEstimator::Sample last = est.snapshot(at_ms(SPAN_MS));
        check(last.fps == s.fps && last.proc_ms_mean == s.proc_ms_mean,
              "R7: repeated snapshots at the same instant must be identical "
              "(snapshot must not mutate state)");
        check(est.snapshot(at_ms(SPAN_MS + 60000)).fps == 0.0,
              "R7: a snapshot far in the future must be 0, i.e. old samples "
              "cannot linger past the window");
    }

    // ── R4: warm-up denominator ─────────────────────────────────────────────
    // 10 frames at 10 Hz spanning only ~1 s, window 3 s. Dividing by the full
    // window would read ~3.3 fps; dividing by the elapsed time reads ~10.
    {
        RateEstimator est(WINDOW_SEC);
        for (int i = 0; i < 10; ++i) {
            est.add(at_ms(i * PERIOD_MS), PROC_SEC);
        }
        const RateEstimator::Sample s = est.snapshot(at_ms(9 * PERIOD_MS));
        check(near(s.fps, 10.0, 1.5),
              "R4: during warm-up the denominator must be the ELAPSED time");
        check(s.fps > 5.0,
              "R4: warm-up must NOT divide by the full window (that would read "
              "~3.3 fps for a true 10 Hz stream)");
    }

    // ── R5: a single frame ──────────────────────────────────────────────────
    {
        RateEstimator est(WINDOW_SEC);
        est.add(at_ms(0), PROC_SEC);
        const RateEstimator::Sample s = est.snapshot(at_ms(0));
        check(std::isfinite(s.fps) && s.fps >= 0.0,
              "R5: one frame with zero elapsed time must not blow the division up");
        check(near(s.proc_ms_mean, 1000.0 * PROC_SEC, 1e-9),
              "R5: one frame still reports its own pipeline time");
    }

    // ── R6: burst ceiling ───────────────────────────────────────────────────
    // 100 frames inside 10 ms; a full window later the rate is capped at
    // count / window_sec — a burst can never be reported as an infinite rate.
    {
        RateEstimator est(WINDOW_SEC);
        for (int i = 0; i < 100; ++i) {
            est.add(at_ms(i / 10), PROC_SEC);  // 100 frames over 10 ms
        }
        const RateEstimator::Sample s =
            est.snapshot(at_ms(static_cast<long long>(WINDOW_SEC * 1000)));
        check(near(s.fps, 100.0 / WINDOW_SEC, 0.5),
              "R6: a burst must be reported as count / window_sec");
        check(s.fps <= 100.0 / WINDOW_SEC + 1e-9,
              "R6: the reported rate must never exceed count / window_sec");
    }

    if (g_rc != 0) {
        spdlog::error("test_rate_estimator: FAIL");
        return g_rc;
    }
    spdlog::info("test_rate_estimator: PASS");
    return g_rc;
}
