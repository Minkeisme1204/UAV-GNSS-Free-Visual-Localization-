#pragma once

#include <uavloc/debug_viewer/telemetry_record.h>
#include <uavloc/debug_viewer/inferred_pose.h>
#include <uavloc/debug_viewer/viewer_metrics.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Forward declarations keep this public header light: the full definitions are
// only needed in the implementation TU (and in callers that already include
// OpenCV / spdlog). cv::Mat and the spdlog sink appear only in by-reference /
// shared_ptr signatures, so an incomplete type is sufficient here.
namespace cv { class Mat; }
namespace spdlog { namespace sinks { class sink; } }
// attach() takes it by reference only — a forward declaration keeps the whole
// pipeline out of every translation unit that draws a window.
namespace uavloc::core { class SystemManager; }

namespace uavloc::debug_viewer {

//! One smoother-corrected fused pose, keyed by the frame it belongs to.
//! Consumed by DebugViewer::pushFusedCorrection(): the viewer drops every
//! stored fused sample with frame_id >= the first correction's frame_id and
//! replaces them with the corrected chain (position in display ENU metres +
//! ENU orientation quaternion, same convention as pushFusedPose).
struct FusedCorrection {
    uint64_t           frame_id = 0;
    Eigen::Vector3f    pos      = Eigen::Vector3f::Zero();
    Eigen::Quaternionf q        = Eigen::Quaternionf::Identity();
};

//! One ABSOLUTE-POSITION measurement, drawn as a discrete marker so it can be
//! seen next to the trajectories it is supposed to correct. `pos` is in the
//! same display ENU metres as pushFusedPose(), i.e. the caller has already
//! applied whatever anchor mapping the fused line uses — the viewer does not
//! know about coordinate frames.
//!
//! `accepted` is the CONSUMER's verdict (did the back-end actually use it),
//! not the producer's opinion; a marker whose verdict is not known yet is
//! simply pushed as not accepted and re-pushed later. See
//! DebugViewer::setAnchorFixes().
//! `from_reinit` marks a RE-ANCHOR measurement — one requested because the VO
//! chain had just been re-initialized, not because the regular cadence came
//! round. It is drawn as a SQUARE instead of a circle so the two are told apart
//! at a glance: whether re-anchoring fires at all is the whole question those
//! runs exist to answer, and a picture that merges them cannot answer it.
struct AnchorFixMarker {
    Eigen::Vector3f pos         = Eigen::Vector3f::Zero();
    bool            accepted    = false;
    bool            from_reinit = false;
};

//! Running totals of the absolute-fix markers the viewer draws, as its own
//! bookkeeping sees them: how many measurements the producer generated and how
//! the back-end judged them. Exposed because a driver has to REPORT those
//! numbers (.claude/rules/reporting.md) and the bookkeeping now lives here.
struct AnchorFixCounters {
    std::size_t generated        = 0;
    std::size_t accepted         = 0;
    std::size_t rejected         = 0;
    std::size_t pending          = 0;  //!< emitted, verdict not known yet
    std::size_t reinit_generated = 0;
    std::size_t reinit_accepted  = 0;
};

class DebugViewer {
public:
    //! State of the run-control button drawn on the main panel. The viewer
    //! knows nothing about what is being run — it only tracks which label to
    //! draw ("Start" when IDLE or PAUSED, "Stop" when RUNNING, a DISABLED
    //! "Finished" when FINISHED) and reports the click.
    //!
    //! IDLE and PAUSED are distinct because the DRIVER usually has to do
    //! different work for the first Start (bring the whole pipeline up) and for
    //! a later one (resume a paused input); the button looks the same in both.
    //!
    //! FINISHED is the TERMINAL state: the pipeline has been shut down for
    //! good, so no click could ever be honoured again. It exists because a
    //! button that says "Start" and then silently refuses is a display telling
    //! a lie — the state where the run is over is visually different from the
    //! state where it is merely paused.
    enum class RunState { IDLE, RUNNING, PAUSED, FINISHED };

    //! Handler invoked (on the render thread) when the user clicks the
    //! run-control button. `want_run` is true for Start (run / resume), false
    //! for Stop (pause); the handler returns true if the request SUCCEEDED. On
    //! false the button state is left untouched and the viewer logs the refusal
    //! — the display never claims a transition that did not happen.
    //!
    //! Deliberately a single std::function, NOT a CallbackSlot: this is a
    //! command with exactly one owner (the driver that owns the pipeline) and
    //! it needs an answer back. CallbackSlot is a `void(Args...)` fan-out to N
    //! subscribers with no return path, so it cannot carry the verdict, and
    //! "N owners of one pipeline" is not a meaningful configuration.
    using RunControlHandler = std::function<bool(bool want_run)>;

    //! Observer of every FUSED DISPLAY POSE the bridge hands to
    //! pushFusedPose(), in push order, with the frame it belongs to and the
    //! position already mapped into the groundtruth ENU frame.
    //!
    //! It exists so a TEST can record that sequence (tests/test_vo_viewer.cpp's
    //! UAVLOC_VIEWER_DUMP regression gate) without the recording itself
    //! becoming a viewer feature: writing a CSV is an artifact of the
    //! experiment, not of the display. Invoked on whichever thread produced the
    //! pose (the fusion thread in async mode), so the sink must be thread-safe.
    using FusedPoseSink =
        std::function<void(unsigned int frame_id, const Eigen::Vector3d& enu_display)>;

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
        // Show the "Profiling" window: a table of the measured computation
        // blocks with their mean/last/max runtime (ms), call count and share of
        // the per-frame total. Fed via pushProfile(); toggled live by the
        // "Profiling" checkbox.
        bool        show_profile        = false;
        // Show the "Error (m)" window: predicted-vs-groundtruth position error
        // in metres — a per-frame line chart, a distribution histogram, and a
        // running summary (count / mean / median / p95 / max, all recomputed as
        // samples arrive). Fed via pushErrorSample(); toggled live by the
        // "Error (m)" checkbox. Empty until the producer pushes.
        bool        show_error          = false;
        // Master gate for the VO estimate trajectory — orange line, or orange
        // axes in EstOdom mode. Toggled live by the "VO" checkbox. Estimate
        // data keeps accumulating while hidden — only the rendering is gated;
        // re-enabling re-uploads the full accumulated trajectory.
        bool        show_estimate       = true;
        // Show the groundtruth trajectory (green polyline + its periodic
        // coord-frame gizmos). Toggled live by the "GroundTruth" checkbox.
        // Groundtruth data keeps accumulating while hidden — only the rendering
        // is gated; re-enabling re-uploads the full accumulated trajectory.
        bool        show_groundtruth    = true;
        // "EstOdom" rendering STYLE selector for ALL THREE trajectories.
        // OFF (default) = line rendering: green groundtruth polyline (+ its
        // periodic uav_* coord-frame gizmos at coord_frame_every_n cadence),
        // orange estimate polyline, magenta fused polyline. ON = every
        // trajectory becomes a chain of per-pose XYZ axes gizmos, colour-coded
        // by source: groundtruth = all 3 rays GREEN (monochrome, same green as
        // the GT line), estimate = all 3 rays ORANGE (same orange as the
        // estimate line), fused = standard RGB axes (X red, Y green, Z blue).
        // The "VO"/"GroundTruth"/"Fused" checkboxes stay master visibility
        // gates in both modes — EstOdom only chooses the style. Toggled live by the
        // "EstOdom" checkbox; toggling swaps the drawables from the same
        // accumulated buffers (no data loss).
        bool        est_odom            = false;
        // In EstOdom mode, draw an axes gizmo for 1 of every N poses (>= 1);
        // shared by all three trajectories (groundtruth / estimate / fused).
        int         est_odom_every_n    = 1;
        // Length of the pose-axes rays in EstOdom mode (metres; same display
        // scaling as the groundtruth coord-frame gizmos); shared by all three
        // trajectories.
        float       est_odom_axes_scale = 10.0f;
        // Periodically re-center the 3D camera on the latest groundtruth point
        // (auto lookat every N points). Disable to pan/orbit freely with the
        // mouse. Toggled live by the "Follow camera" checkbox.
        bool        follow_camera       = true;
        // Show the fused (VO+telemetry back-end) trajectory (magenta polyline).
        // Toggled live by the "Fused" checkbox. Same semantics as the
        // "GroundTruth" checkbox: fused data keeps accumulating while hidden —
        // only the rendering is gated; re-enabling re-uploads the full
        // accumulated trajectory.
        bool        show_fused          = true;
        // Show the absolute-position fix markers (setAnchorFixes): yellow where
        // the back-end accepted the measurement, purple where it rejected it or
        // has not judged it yet. Toggled live by the "Anchor fixes" checkbox.
        bool        show_anchor_fixes   = true;

        // ── Overlay-driver parameters ───────────────────────────────────────
        // These do not configure the window; they configure the DRIVER-side
        // bridge that feeds it (TrajectoryAligner, the HUD heading, and the
        // performance sampler). They live here so one YAML section describes
        // the whole overlay instead of half of it.
        //
        // Number of initial TRACKING frames accumulated before the VO-world ->
        // groundtruth-ENU rigid transform is frozen. Clamped to >= 1.
        int         alignment_window_frames = 50;
        // Optional freeze gate: when > 0, delay freezing the alignment until the
        // groundtruth window spans at least this many metres horizontally
        // (guards the yaw estimate against a near-stationary window).
        // 0 = off (freeze as soon as the window is full).
        double      alignment_min_spread_m  = 0.0;
        // Minimum per-frame baseline (metres) before the course-over-ground
        // heading shown on the HUD is refreshed (anti-jitter when hovering).
        double      hud_min_baseline_m      = 0.5;
        // Period (ms) of the /proc CPU%/RSS performance sampler thread that
        // feeds pushPerf(). Clamped to >= 50.
        int         perf_sample_ms          = 500;

        // ── Run policy supplied by the APPLICATION (never read from YAML) ────
        // These three are decisions only the application can make; the viewer
        // merely executes them. They are deliberately absent from fromYaml():
        // a config file must not be able to auto-start a pipeline or silently
        // cap a measurement run.
        //
        // Start the attached SystemManager as soon as run() begins instead of
        // waiting for the user to press "Start". Every situation in which
        // nobody CAN press the button (headless, a deterministic dump gate, an
        // explicit override) must set this, or the run would hang forever.
        bool        autostart               = false;
        // Human-readable justification for `autostart`, logged verbatim next to
        // the decision so a log reader can tell WHY a run started by itself.
        std::string autostart_reason;
        // Hard cap on processed frames; 0 = run until end of stream. The
        // supervisor stops the system as soon as it sees the count, and the
        // bridge additionally ignores every output past the cap-th frame so the
        // cap is exact on the OUTPUT side too (a late fusion result is judged by
        // the frame it belongs to, not by when it landed).
        std::size_t max_frames              = 0;

        //! Read a Config from the mission YAML's "DebugViewer:" sub-tree.
        //! Follows the project pattern node["key"].as<T>(default), so missing
        //! keys never throw and an absent/!node section yields the defaults.
        static Config fromYaml(const YAML::Node& node);
    };

    DebugViewer();
    explicit DebugViewer(const Config& cfg);
    ~DebugViewer();

    // ── Bridge to the pipeline (S3/S4) ───────────────────────────────────────
    //! Subscribe to `sys`'s four display channels (on_frame_processed /
    //! on_vo_data / on_fusion_result / on_lag_window), install the Start/Stop
    //! run control, and remember `sys` so run() can supervise it. The reference
    //! must outlive this viewer.
    //!
    //! From here on run() also owns the run: it auto-starts the system when
    //! Config::autostart is set, spawns the end-of-data supervisor and the
    //! performance sampler, and joins both before it returns.
    //!
    //! Call before run(). Attaching twice is refused (logged) — two bridges
    //! would double every trajectory.
    void attach(core::SystemManager& sys);
    //! Install the fused-pose observer (see FusedPoseSink). Pass an empty
    //! function to remove it. Call before start/run; there is exactly one sink.
    void setFusedPoseSink(FusedPoseSink sink);
    //! Record one ABSOLUTE-POSITION measurement the producer just generated.
    //! `xy_fusion_enu` is in the back-end's own ENU frame (the frame of
    //! fusion::FusionResult::T_enu_c); the viewer supplies the missing altitude
    //! from the newest fused pose and applies the same display anchoring as the
    //! fused line. The accepted/rejected verdict is NOT given here — it is
    //! polled from the back-end later. Thread-safe.
    //! Only drawn once enableAnchorFixTracking() has been called.
    void pushAnchorFix(const Eigen::Vector2d& xy_fusion_enu, bool from_reinit);
    //! Turn on absolute-fix marker drawing and the verdict polling, with a
    //! PERMANENT warning line the producer must supply. The warning is not
    //! decoration: when the "measurements" are manufactured (the M1 fake
    //! anchor derives them from groundtruth), a screenshot of this window must
    //! not be able to travel without that caveat
    //! (.claude/rules/reporting.md). Pass an empty string only when the fixes
    //! really are independent measurements.
    void enableAnchorFixTracking(const std::string& contamination_warning);
    //! Snapshot of the marker bookkeeping (thread-safe). All zeros when
    //! enableAnchorFixTracking() was never called.
    AnchorFixCounters anchorFixCounters() const;

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
    // Push a fused (VO+telemetry back-end) pose from any thread: position
    // already in display ENU (metres) plus the ENU orientation quaternion and
    // the source frame id (so a later pushFusedCorrection can splice the
    // sample out). Rendered — when the "Fused" checkbox is enabled — as the
    // magenta polyline (position only) in line mode, or as standard-RGB
    // pose-axes gizmos (position + orientation) in EstOdom mode. Thread-safe.
    void pushFusedPose(const Eigen::Vector3f& enu_pos,
                       const Eigen::Quaternionf& q_enu,
                       uint64_t frame_id);
    // Redraw the fused trajectory's recent past with the smoother-corrected
    // poses: remove ALL stored fused samples with
    // frame_id >= corrected.front().frame_id, append `corrected` in order, and
    // force a full rebuild of the fused drawables (line and/or axes per the
    // current EstOdom mode). Subsequent pushFusedPose calls append after the
    // correction as before. Empty input is ignored. Thread-safe (applied in
    // the render-loop drain, in push order relative to pushFusedPose).
    void pushFusedCorrection(const std::vector<FusedCorrection>& corrected);
    // Replace the whole absolute-fix marker set (already in display ENU,
    // metres) from any thread. REPLACE, not append, on purpose: a fix's
    // accepted/rejected verdict only becomes known some frames after the
    // measurement was made, so the producer of this list owns it and re-pushes
    // the corrected snapshot. Thread-safe.
    void setAnchorFixes(const std::vector<AnchorFixMarker>& fixes);
    // Two lines of text drawn at the TOP of the viewer's main panel: a
    // permanent RED warning and a plain status/counter line. Empty strings hide
    // the respective line. Thread-safe.
    //
    // The warning line exists for one reason: a screenshot of this window must
    // never be able to travel without the caveat that produced it (the M1 fake
    // anchor derives its "measurements" from groundtruth). Putting the caveat
    // in the log only would fail that test — .claude/rules/reporting.md
    // requires the warning where the numbers are shown.
    void setAnchorOverlay(const std::string& warning, const std::string& stats);
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
    // Push a profiling table snapshot from any thread; stored in a single
    // LATEST-WINS slot (the table only ever shows the newest snapshot) and
    // rendered by the "Profiling" window. Row order is preserved. Thread-safe.
    void pushProfile(const ProfileSnapshot& p);
    // Push one predicted-vs-groundtruth position error sample (metres, already
    // paired by frame id by the producer) from any thread; drained into the
    // "Error (m)" window's line chart, histogram and running summary by the
    // render loop (ring buffer capped at Config::metric_plot_history).
    // Thread-safe.
    void pushErrorSample(const ErrorSample& s);
    // Push the latest video frame from any thread; cloned into a single
    // drop-oldest slot and uploaded as a texture by the render loop. Thread-safe.
    void pushFrame(const cv::Mat& image);
    // Push the latest tracking-overlay frame (video with landmark keypoints +
    // annotations already drawn) from any thread; cloned into a single
    // drop-oldest slot and uploaded to the "Tracking" sub-window by the render
    // loop when the Tracking checkbox is enabled. Thread-safe.
    void pushTrackingFrame(const cv::Mat& image);
    // Install the run-control handler. Until one is installed the button is not
    // drawn at all (a viewer with nothing to drive — e.g. demo_debug_viewer —
    // must not show a dead button). Pass an empty handler to remove it.
    // Thread-safe; call before run().
    void setRunControl(RunControlHandler handler);
    // Set the button state from the producer side, for the transitions the user
    // did not cause: an auto-started run (headless / dump / autostart env) is
    // RUNNING from the outset, and a run whose data ended and whose pipeline
    // was shut down is FINISHED — the button then goes grey and stops inviting
    // a click that could only be refused. Thread-safe.
    void setRunState(RunState state);
    // Current button state. Thread-safe.
    RunState runState() const;
    // Returns a spdlog sink that captures formatted log lines into the viewer's
    // scrolling terminal panel. Attach it to the default logger's sink list
    // before logging. Lazily created; the same instance is returned thereafter.
    std::shared_ptr<spdlog::sinks::sink> logSink();
    // Blocking render loop — registers listeners on DebugViewerCallbacks.
    // With a system attached (see attach()) it additionally owns the RUN: it
    // performs the configured auto-start, spawns the supervisor and the
    // performance sampler, and joins both before returning — on the headless
    // path (where it does not block on a window) just as much as on the GUI one.
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
