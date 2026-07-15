#pragma once

#include <uavloc/debug_viewer/telemetry_record.h>
#include <uavloc/debug_viewer/inferred_pose.h>
#include <uavloc/debug_viewer/viewer_metrics.h>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

// Forward declarations keep this public header light: the full definitions are
// only needed in the implementation TU (and in callers that already include
// OpenCV / spdlog). cv::Mat and the spdlog sink appear only in by-reference /
// shared_ptr signatures, so an incomplete type is sufficient here.
namespace cv { class Mat; }
namespace spdlog { namespace sinks { class sink; } }

namespace uavloc::debug_viewer {

class DebugViewer {
public:
    struct Config {
        std::string window_title        = "UAV Debug Viewer";
        int         window_w            = 1280;
        int         window_h            = 720;
        int         max_trajectory_pts  = 10000;
        // Process only 1 out of every `frame_stride` video frames; the rest are
        // skipped without decoding (cap.grab()). Higher => reads through the
        // video faster. Must be >= 1 (1 = process every frame).
        int         frame_stride        = 1;
        int         vector_every_n      = 50;    // draw heading arrow every N frames
        float       arrow_scale         = 8.0f;  // length of heading arrows (metres)
        float       coord_frame_scale   = 200.0f;// size of NED coord frame gizmos (metres)
        int         coord_frame_every_n = 500;   // draw full coord frame every N frames
        // Metres -> render-unit multiplier. The raw ENU trajectory can span many
        // kilometres; shrink the whole world so it fits on screen. Applied
        // uniformly to point positions and gizmo sizes (everything stays in
        // metres conceptually). e.g. 0.01 => 1 render unit == 100 m.
        float       display_scale       = 0.01f;
        // Extra multiplier on the vertical (Up/altitude) axis only, on top of
        // display_scale. The horizontal flight spans kilometres while altitude
        // varies only ~100 m, so the path looks flat. Use >1 to exaggerate
        // altitude relative to the horizontal plane, <1 (or 0) to flatten it.
        float       vertical_scale      = 1.0f;

        // ── UI control panel (checkbox-toggled side panels) ──────────────────
        // Push only 1 of every `video_stride` frames into the live video panel
        // (a cheap drop-oldest slot). Higher => fewer texture uploads. >= 1.
        int         video_stride        = 3;
        // Maximum number of captured log lines kept in the scrolling terminal.
        int         log_capacity        = 500;
        // Maximum number of metric samples retained by the line chart.
        int         metric_plot_history = 2000;
        // Width (in samples) of the auto-follow window on the metric line charts:
        // the X axis auto-slides to show only the newest `metric_plot_follow_window`
        // samples (Y auto-fits) so the latest values are always visible without
        // panning the plot with the mouse. >= 1.
        int         metric_plot_follow_window = 500;
        // Number of bins used by the value-distribution histograms.
        int         histogram_bins      = 30;
        // Maximum number of process-performance samples (pushPerf) retained by
        // the "Performance" streaming line charts.
        int         perf_plot_history   = 2000;
        // Initial visibility of the side panels (toggled live by checkbox). The
        // inliers and landmarks metrics each open their own resizable window with
        // a per-frame line chart and a value-distribution histogram.
        bool        show_log            = false;
        bool        show_inliers        = false;
        bool        show_landmarks      = false;
        bool        show_video          = false;
        // Show a separate "Tracking" video sub-window with the tracked landmark
        // keypoints drawn on the frame and the landmark count annotated. Fed via
        // pushTrackingFrame(); toggled live by the "Tracking" checkbox.
        bool        show_tracking       = false;
        // Draw a red marker at each estimate-trajectory position where tracking
        // was LOST (discrete points, toggled live by the "Lost state poses"
        // checkbox).
        bool        show_lost_poses     = false;
        // Show a small heads-up-display panel with the VO course-over-ground
        // heading, the telemetry heading, and the accumulated path length. Fed via
        // pushMetrics(); toggled live by the "HUD" checkbox.
        bool        show_hud            = false;
        // Show the "Performance" window with two streaming line charts (process
        // CPU % and RSS MB over wall time). Fed via pushPerf(); toggled live by
        // the "Performance" checkbox.
        bool        show_perf           = false;
        // Show the groundtruth trajectory (green polyline + its periodic
        // coord-frame gizmos). Toggled live by the "GroundTruth" checkbox.
        // Groundtruth data keeps accumulating while hidden — only the rendering
        // is gated; re-enabling re-uploads the full accumulated trajectory.
        bool        show_groundtruth    = true;
        // Render the estimate trajectory as per-pose XYZ axes gizmos (one small
        // red/green/blue coordinate frame per estimate pose, oriented by the
        // pose roll/pitch/yaw) instead of the orange polyline. Toggled live by
        // the "EstOdom" checkbox; toggling swaps the drawables from the same
        // accumulated buffer (no data loss).
        bool        est_odom            = false;
        // Draw an axes gizmo for 1 of every N estimate poses (>= 1) in EstOdom
        // mode.
        int         est_odom_every_n    = 1;
        // Size of the estimate pose-axes gizmos (metres; same display scaling
        // as the groundtruth coord-frame gizmos).
        float       est_odom_axes_scale = 10.0f;
        // Periodically re-center the 3D camera on the latest groundtruth point
        // (auto lookat every N points). Disable to pan/orbit freely with the
        // mouse. Toggled live by the "Follow camera" checkbox.
        bool        follow_camera       = true;
    };

    DebugViewer();
    explicit DebugViewer(const Config& cfg);
    ~DebugViewer();

    // Pre-load a batch of records before run() is called (not thread-safe vs run())
    void loadBatch(const std::vector<TelemetryRecord>& records);
    // Scan directory for *.csv and *.mp4/*.avi/*.mkv; loads both automatically
    void loadDirectory(const std::string& dir_path);
    // Push a single record from any thread (thread-safe)
    void pushRecord(const TelemetryRecord& r);
    // Push a metric estimate pose (already in display ENU, metres) from any
    // thread; rendered as the inferred/estimate polyline (orange). Thread-safe.
    void pushPose(const InferredPose& p);
    // Push a metric groundtruth pose (already in display ENU, metres) from any
    // thread; rendered as the groundtruth polyline (green). Thread-safe.
    void pushGroundtruthPose(const InferredPose& p);
    // Push a metric pose (already in display ENU, metres) marking a LOST tracking
    // position from any thread; rendered as a discrete red marker when the "Lost
    // state poses" checkbox is enabled. Thread-safe.
    void pushLostPose(const InferredPose& p);
    // Replace the map-point cloud (already in display ENU, metres) from any
    // thread; rendered as a 3D point cloud when the "Landmarks" checkbox is
    // enabled. Each call supersedes the previous cloud (drop-oldest snapshot).
    // Thread-safe.
    void pushMapPoints(const std::vector<Eigen::Vector3f>& pts_enu);
    // Push a per-frame metric sample (inliers + landmark count) from any thread;
    // drained into the live line chart by the render loop. Thread-safe.
    void pushMetrics(const FrameMetrics& m);
    // Push a process-performance sample (CPU % + RSS MB at t_sec) from any
    // thread; drained into the "Performance" streaming line charts by the
    // render loop (ring buffer capped at Config::perf_plot_history). Thread-safe.
    void pushPerf(const PerfSample& s);
    // Push the latest video frame from any thread; cloned into a single
    // drop-oldest slot and uploaded as a texture by the render loop. Thread-safe.
    void pushFrame(const cv::Mat& image);
    // Push the latest tracking-overlay frame (video with landmark keypoints +
    // annotations already drawn) from any thread; cloned into a single
    // drop-oldest slot and uploaded to the "Tracking" sub-window by the render
    // loop when the Tracking checkbox is enabled. Thread-safe.
    void pushTrackingFrame(const cv::Mat& image);
    // Returns a spdlog sink that captures formatted log lines into the viewer's
    // scrolling terminal panel. Attach it to the default logger's sink list
    // before logging. Lazily created; the same instance is returned thereafter.
    std::shared_ptr<spdlog::sinks::sink> logSink();
    // Blocking render loop — registers listeners on DebugViewerCallbacks
    void run();
    // Request shutdown from any thread
    void stop();
    // True iff the most recent run() actually opened a GL window (i.e. not
    // headless). Valid to read after run() returns. Lets a caller decide whether
    // a worker thread should be force-stopped (GUI) or allowed to finish
    // (headless).
    bool hadDisplay() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::debug_viewer
