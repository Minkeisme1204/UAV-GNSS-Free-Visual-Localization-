#pragma once

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

} // namespace uavloc::debug_viewer
