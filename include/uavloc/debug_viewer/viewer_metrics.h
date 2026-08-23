#pragma once

#include <string>
#include <vector>

namespace uavloc::debug_viewer {

// A single per-frame VO metric sample fed to the live line chart. Carries only
// the two quantities the chart plots so producers (the VO worker) enqueue a
// tiny, cheap value with no GL/ImGui involvement. inliers and landmarks are
// stored as float so they slot directly into the plot's double history.
struct FrameMetrics {
    int   frame_id  = 0;
    float inliers   = 0.0f;   // tracked/seeded inliers this frame
    float landmarks = 0.0f;   // total landmarks in the local map

    // HUD scalars (latest-wins; shown as text, not plotted).
    float heading_deg     = 0.0f;  // course-over-ground from VO, 0 = North CW
    float heading_tel_deg = 0.0f;  // telemetry heading + gimbal pan, 0 = North CW
    float distance_m      = 0.0f;  // accumulated VO path length (metres)
};

// A single process-performance sample fed to the "Performance" streaming line
// charts. The PRODUCER measures (e.g. the algorithm driver sampling
// /proc/self); the viewer only plots — keeping the module decoupled from any
// OS-specific measurement mechanism. POD only (no GL/ImGui involvement).
struct PerfSample {
    double t_sec       = 0.0;   // wall-clock seconds since the producer started
    float  cpu_percent = 0.0f;  // process CPU usage; may exceed 100 with threads
    float  rss_mb      = 0.0f;  // resident set size (MB)
};

// One live throughput sample for the HUD. The PRODUCER measures (core::
// SystemManager, which owns a trailing-window RateEstimator and publishes on
// the monitor thread); the viewer only formats — it owns no clock and no
// counter. POD only (no GL/ImGui involvement).
struct ThroughputSample {
    double fps_windowed = 0.0;  // live wall-clock rate over the trailing window [frames/s]
    double fps_mean     = 0.0;  // cumulative mean since start [frames/s]
    double proc_ms_mean = 0.0;  // mean pipeline time per frame in the window [ms]
    double window_sec   = 0.0;  // window length [s] — the viewer only labels it
};

// One row of the "Profiling" table: the accumulated timing of one measured
// computation block. The PRODUCER measures and derives every number (the viewer
// only formats them), keeping the viewer decoupled from the profiling
// mechanism. POD only (no GL/ImGui involvement).
struct ProfileRow {
    std::string        name;              // module/stage name, e.g. "ORB_EXTRACT"
    float              mean_ms  = 0.0f;   // total_ms / count — the headline number
    float              last_ms  = 0.0f;   // most recent sample
    float              max_ms   = 0.0f;   // largest sample so far
    unsigned long long count    = 0;      // samples accumulated
    float              percent  = 0.0f;   // share of the per-frame total (%)
};

// One live localization-accuracy sample: the distance between the system's
// PREDICTED position and the GROUNDTRUTH position of the SAME frame, in metres.
// The PRODUCER pairs the two by frame id and does the subtraction (the viewer
// owns no notion of "which pose belongs to which frame"), so the viewer only
// bins and plots. POD only (no GL/ImGui involvement).
//
// ⚠ Meaning depends entirely on what the producer calls "groundtruth". When it
// is telemetry-derived it carries the telemetry's own error; when the same
// source also feeds an absolute-fix generator the number is CONTAMINATED and
// must not be reported as system accuracy (.claude/rules/reporting.md).
struct ErrorSample {
    int   frame_id = 0;
    float err_2d_m = 0.0f;   // horizontal distance (E/N plane)
    float err_3d_m = 0.0f;   // full 3D distance
};

// A full profiling table snapshot (latest-wins: each push supersedes the
// previous one). Row order is the producer's order and is preserved by the
// viewer so the table does not flicker.
struct ProfileSnapshot {
    double                  t_sec    = 0.0;  // producer wall-clock seconds
    int                     frame_id = 0;    // frame the snapshot was taken at
    std::vector<ProfileRow> rows;
};

} // namespace uavloc::debug_viewer
