#include <uavloc/debug_viewer/debug_viewer.h>
#include <uavloc/debug_viewer/debug_viewer_callbacks.h>
#include <uavloc/debug_viewer/inferred_pose.h>
#include <uavloc/debug_viewer/telemetry_csv_reader.h>

#include "barcode_decoder.h"
#include "debug_viewer_bridge.h"
#include "gps_to_enu.h"
#include "log_ring_sink.h"

#include <opencv2/opencv.hpp>

#include <guik/viewer/light_viewer.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/primitives/primitives.hpp>
#include <imgui.h>
#include <implot.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace uavloc::debug_viewer {

namespace {

// Recognised container extensions when scanning a directory for the video.
const std::vector<std::string> VIDEO_EXTENSIONS = {".mkv", ".mp4", ".avi"};

// Display scale (0..1) for the live video sub-window in the GL overlay.
constexpr double VIDEO_OVERLAY_SCALE = 0.30;

// Re-centre the camera on the latest groundtruth point every N appended points.
constexpr int CAMERA_RECENTER_EVERY_N = 30;

// Light cap on the Mode-A push loop so the viewer thread does not peg a core
// busy-spinning when spin_once() returns without vsync-blocking.
constexpr int VIEWER_SPIN_SLEEP_MS = 5;

// Initial (resizable) size of a metric window — first use only; the user is
// free to drag it larger/smaller afterwards.
constexpr int METRIC_PLOT_W = 420;
constexpr int METRIC_PLOT_H = 360;

// Initial (resizable) size of the "Profiling" table window — first use only.
constexpr int PROFILE_TABLE_W = 620;
constexpr int PROFILE_TABLE_H = 420;

// GL resource name for the checkbox-toggled video side panel.
const char* const VIDEO_PANEL_NAME = "VO video";

// GL resource name for the checkbox-toggled tracking-overlay side panel.
const char* const TRACKING_PANEL_NAME = "VO tracking";

// Colours (RGBA) for the three trajectories.
constexpr float GT_COLOR[4]       = {0.2f, 0.9f, 0.4f, 1.0f};  // green
constexpr float INFERRED_COLOR[4] = {0.95f, 0.55f, 0.1f, 1.0f}; // orange
constexpr float FUSED_COLOR[4]    = {0.90f, 0.20f, 0.90f, 1.0f}; // magenta

// EstOdom-only "error links": one straight segment from each drawn FUSED pose
// (the system's final output) to the groundtruth pose of the SAME frame id.
// A paler magenta than the fused line itself so the link cannot be mistaken
// for the trajectory.
constexpr float FUSED_LINK_COLOR[4] = {0.85f, 0.45f, 0.95f, 1.0f}; // pale magenta

// Colour + screen-space point size for the discrete LOST-state markers.
constexpr float LOST_COLOR[4]           = {0.95f, 0.15f, 0.15f, 1.0f}; // red
constexpr float LOST_MARKER_POINT_SCALE = 8.0f;

// Colour + screen-space point size for the VO map-point cloud (pushMapPoints).
constexpr float MAP_POINT_COLOR[4]      = {0.40f, 0.70f, 1.00f, 1.0f}; // light blue
constexpr float MAP_POINT_SCALE         = 4.0f;

// Absolute-position fix markers (setAnchorFixes). Deliberately the two colours
// no trajectory uses, and a bigger point than any of them: a fix is a discrete
// event that must be findable at a glance next to the lines it corrects.
constexpr float FIX_ACCEPTED_COLOR[4] = {1.00f, 0.90f, 0.10f, 1.0f}; // yellow
constexpr float FIX_REJECTED_COLOR[4] = {0.65f, 0.20f, 0.95f, 1.0f}; // purple
constexpr float FIX_MARKER_POINT_SCALE = 12.0f;

// RE-ANCHOR fixes (AnchorFixMarker::from_reinit) — requested because the VO
// chain restarted, not because the cadence came round. Different HUE and a
// SQUARE point, so the two kinds stay distinguishable even in a greyscale
// screenshot; bigger, because a re-anchor is a rare event worth finding.
constexpr float FIX_REINIT_ACCEPTED_COLOR[4] = {0.10f, 1.00f, 0.60f, 1.0f}; // green
constexpr float FIX_REINIT_REJECTED_COLOR[4] = {1.00f, 0.45f, 0.00f, 1.0f}; // orange
constexpr float FIX_REINIT_MARKER_POINT_SCALE = 16.0f;

// Colour of the permanent contamination banner (setAnchorOverlay).
constexpr float WARNING_TEXT_COLOR[4] = {1.00f, 0.25f, 0.25f, 1.0f};

// Run-control button (setRunControl): the labels it alternates between, plus
// the terminal one. RUN_BUTTON_FINISHED is drawn DISABLED — once the pipeline
// has been shut down for good, offering "Start" would be a lie the user only
// finds out about by clicking it.
constexpr const char* RUN_BUTTON_START    = "Start";
constexpr const char* RUN_BUTTON_STOP     = "Stop";
constexpr const char* RUN_BUTTON_FINISHED = "Finished";
// Explanation drawn next to the disabled button, so the grey widget is not a
// mystery: the run is over and only closing the window is left.
constexpr const char* RUN_FINISHED_HINT =
    "run over - close the window to exit";

// Draw one auto-follow streaming line chart of `values` vs `xs`.
//
// Plotting backend: ImPlot is preferred (guik creates the ImPlot context for its
// own plot API, so ImPlot::GetCurrentContext() is valid here). When no context
// is present we fall back to ImGui core (PlotLines); we never call
// ImPlot::CreateContext() ourselves, so there is no risk of a duplicate-context
// crash. Render-thread only — all data is read from the render-thread-owned
// ring buffers.
//
// Auto-follow: the X axis is slid every frame to show only the newest
// `follow_window` SAMPLES (Y auto-fits), so the latest values are always on
// screen without the user panning with the mouse. With ImPlot this is a hard
// SetupAxisLimits(..., Always) on X + an AutoFit Y flag; the ImGui-core
// fallback simply plots the trailing `follow_window`-sample slice.
// `plot_id` must be unique within the enclosing window ("##line", "##cpu", ...).
void drawStreamingLine(const char* plot_id, const char* x_label,
                       const char* value_label,
                       const std::vector<double>& xs,
                       const std::vector<double>& values,
                       int follow_window, float height) {
    const int n      = static_cast<int>(values.size());
    const int follow = std::max(1, follow_window);
    if (n == 0) return;

    if (ImPlot::GetCurrentContext() != nullptr) {
        if (ImPlot::BeginPlot(plot_id, ImVec2(-1, height))) {
            // Y auto-fits to the visible data; X is force-followed to the latest
            // window every frame so the newest samples are always in view.
            ImPlot::SetupAxes(x_label, value_label, 0, ImPlotAxisFlags_AutoFit);
            const double x_last = xs.back();
            double x_lo = xs[static_cast<std::size_t>((n > follow) ? (n - follow) : 0)];
            if (x_last - x_lo < 1e-9) x_lo = x_last - 1.0;  // guard degenerate range
            ImPlot::SetupAxisLimits(ImAxis_X1, x_lo, x_last, ImPlotCond_Always);
            ImPlot::PlotLine(value_label, xs.data(), values.data(), n);
            ImPlot::EndPlot();
        }
    } else {
        // ImGui core fallback: needs float arrays. Auto-follow: plot only the
        // trailing `follow`-sample slice so the newest values are always shown
        // (ImGui::PlotLines otherwise draws the whole array squeezed to fit).
        const double mn = *std::min_element(values.begin(), values.end());
        const double mx = *std::max_element(values.begin(), values.end());
        const int    start   = (n > follow) ? (n - follow) : 0;
        const int    slice_n = n - start;
        std::vector<float> vf(values.begin() + start, values.end());
        ImGui::PlotLines(plot_id, vf.data(), slice_n, 0, value_label,
                         static_cast<float>(mn), static_cast<float>(mx),
                         ImVec2(-1, height));
    }
}

// Draw a metric panel inside a resizable ImGui window: a per-frame line chart on
// top (drawStreamingLine) and a value-distribution histogram below. Both plots
// fill the available content region (ImVec2(-1, ...)) so they grow/shrink with
// the window. Same ImPlot-preferred / ImGui-core-fallback policy as
// drawStreamingLine (the histogram is manually binned in the fallback).
void drawMetricWindow(const char* title, bool* open,
                      const std::vector<double>& frame_x,
                      const std::vector<double>& values,
                      const char* value_label, int hist_bins,
                      int follow_window) {
    ImGui::SetNextWindowSize(ImVec2(METRIC_PLOT_W, METRIC_PLOT_H),
                             ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(title, open)) {
        ImGui::End();
        return;
    }

    const int n = static_cast<int>(values.size());
    if (n == 0) {
        ImGui::TextUnformatted("No samples yet.");
        ImGui::End();
        return;
    }

    // Split the window vertically: line chart (top half) + histogram (bottom).
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float  line_h = avail.y * 0.5f;
    const int    bins   = std::max(1, hist_bins);

    drawStreamingLine("##line", "frame", value_label, frame_x, values,
                      follow_window, line_h);

    if (ImPlot::GetCurrentContext() != nullptr) {
        if (ImPlot::BeginPlot("##hist", ImVec2(-1, -1))) {
            ImPlot::SetupAxes(value_label, "count", 0, 0);
            ImPlot::PlotHistogram(value_label, values.data(), n, bins);
            ImPlot::EndPlot();
        }
    } else {
        // ImGui core fallback: manually binned histogram.
        const double mn = *std::min_element(values.begin(), values.end());
        const double mx = *std::max_element(values.begin(), values.end());
        std::vector<float> counts(static_cast<std::size_t>(bins), 0.0f);
        const double range = std::max(mx - mn, 1e-9);
        for (double v : values) {
            int b = static_cast<int>((v - mn) / range * bins);
            b = std::max(0, std::min(bins - 1, b));
            counts[static_cast<std::size_t>(b)] += 1.0f;
        }
        ImGui::PlotHistogram("##hist", counts.data(), bins, 0, value_label,
                             0.0f, FLT_MAX, ImVec2(-1, -1));
    }

    ImGui::End();
}

// Draw the "Performance" window: two stacked streaming line charts (process
// CPU % and RSS MB, both vs wall time in seconds) fed by pushPerf(). Reuses
// drawStreamingLine for the follow-window/autofit behaviour of the metric
// plots. Render-thread only.
void drawPerfWindow(bool* open,
                    const std::vector<double>& t_sec,
                    const std::vector<double>& cpu_percent,
                    const std::vector<double>& rss_mb,
                    int follow_window) {
    ImGui::SetNextWindowSize(ImVec2(METRIC_PLOT_W, METRIC_PLOT_H),
                             ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Performance", open)) {
        ImGui::End();
        return;
    }

    if (t_sec.empty()) {
        ImGui::TextUnformatted("No samples yet.");
        ImGui::End();
        return;
    }

    // Split the window vertically: CPU chart (top half) + RSS chart (bottom).
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float  half_h = avail.y * 0.5f;

    drawStreamingLine("##cpu", "t (s)", "% (may exceed 100 with threads)",
                      t_sec, cpu_percent, follow_window, half_h);
    drawStreamingLine("##rss", "t (s)", "RSS (MB)",
                      t_sec, rss_mb, follow_window, -1.0f);

    ImGui::End();
}

// Nearest-rank percentile of an ALREADY SORTED sample vector (same ruler as
// tests/eval_common.cpp::percentile, so a number read off this window and one
// computed by the offline evaluator mean the same thing). q in [0, 1].
double sorted_percentile(const std::vector<double>& sorted, double q) {
    if (sorted.empty()) {
        return 0.0;
    }
    const double rank = q * static_cast<double>(sorted.size());
    auto idx = static_cast<std::size_t>(std::ceil(rank));
    if (idx == 0) {
        idx = 1;
    }
    if (idx > sorted.size()) {
        idx = sorted.size();
    }
    return sorted[idx - 1];
}

// Draw the "Error (m)" window: predicted-vs-groundtruth position error, fed by
// pushErrorSample(). Layout, top to bottom: a running summary line, a per-frame
// line chart, and a distribution histogram — the same line+histogram pair the
// metric windows use, so the two read alike.
//
// The summary deliberately reports count / mean / MEDIAN / p95 / max rather
// than a lone mean: a single number hides the tail this project cares about
// (.claude/rules/reporting.md requires median · p95 · max). It is recomputed
// from the whole retained history every frame — O(n log n) on a ring buffer
// capped at metric_plot_history, and only while the window is open.
// Render-thread only.
void drawErrorWindow(bool* open,
                     const std::vector<double>& frame_x,
                     const std::vector<double>& err_2d,
                     const std::vector<double>& err_3d,
                     bool* use_3d,
                     int   hist_bins,
                     int   follow_window) {
    ImGui::SetNextWindowSize(ImVec2(METRIC_PLOT_W, METRIC_PLOT_H),
                             ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Error (m)", open)) {
        ImGui::End();
        return;
    }

    // Horizontal error is the default reading; the 3D one adds the altitude
    // component, which has its own (telemetry-AGL) error budget.
    ImGui::Checkbox("3D (include altitude)", use_3d);

    const std::vector<double>& values = *use_3d ? err_3d : err_2d;
    const int                  n      = static_cast<int>(values.size());
    if (n == 0) {
        ImGui::TextUnformatted("No samples yet "
                               "(the producer pushes one per fused frame that "
                               "has a groundtruth of the same frame id).");
        ImGui::End();
        return;
    }

    // Running summary over the whole retained history.
    std::vector<double> sorted(values);
    std::sort(sorted.begin(), sorted.end());
    double sum = 0.0;
    for (double v : values) {
        sum += v;
    }
    const double mean   = sum / static_cast<double>(n);
    const double median = sorted_percentile(sorted, 0.50);
    const double p95    = sorted_percentile(sorted, 0.95);
    const double max_v  = sorted.back();

    ImGui::Text("n = %d   mean = %.2f m   median = %.2f m   p95 = %.2f m   "
                "max = %.2f m", n, mean, median, p95, max_v);
    ImGui::Text("last  = %.2f m  (frame %.0f)", values.back(),
                frame_x.empty() ? 0.0 : frame_x.back());
    ImGui::Separator();

    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  line_h = avail.y * 0.5f;
    const int    bins   = std::max(1, hist_bins);
    const char*  label  = *use_3d ? "err_3d (m)" : "err_2d (m)";

    drawStreamingLine("##err_line", "frame", label, frame_x, values,
                      follow_window, line_h);

    if (ImPlot::GetCurrentContext() != nullptr) {
        if (ImPlot::BeginPlot("##err_hist", ImVec2(-1, -1))) {
            ImPlot::SetupAxes(label, "count", 0, 0);
            ImPlot::PlotHistogram(label, values.data(), n, bins);
            ImPlot::EndPlot();
        }
    } else {
        // ImGui core fallback: manually binned histogram (same policy as
        // drawMetricWindow).
        const double mn = sorted.front();
        const double mx = sorted.back();
        std::vector<float> counts(static_cast<std::size_t>(bins), 0.0f);
        const double range = std::max(mx - mn, 1e-9);
        for (double v : values) {
            int b = static_cast<int>((v - mn) / range * bins);
            b = std::max(0, std::min(bins - 1, b));
            counts[static_cast<std::size_t>(b)] += 1.0f;
        }
        ImGui::PlotHistogram("##err_hist", counts.data(), bins, 0, label,
                             0.0f, FLT_MAX, ImVec2(-1, -1));
    }

    ImGui::End();
}

// Draw the "Profiling" window: one table row per measured computation block,
// showing the MEAN runtime (the headline number) plus last/max/calls/share.
// Fed by pushProfile(); the row order is the producer's (stable, no flicker).
// Render-thread only.
void drawProfileWindow(bool* open, const ProfileSnapshot& snap) {
    ImGui::SetNextWindowSize(ImVec2(PROFILE_TABLE_W, PROFILE_TABLE_H),
                             ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Profiling", open)) {
        ImGui::End();
        return;
    }

    if (snap.rows.empty()) {
        ImGui::TextUnformatted("No samples yet "
                               "(profiling is opt-in: set UAVLOC_PROFILE=1).");
        ImGui::End();
        return;
    }

    ImGui::Text("frame %d   t = %.1f s", snap.frame_id, snap.t_sec);
    ImGui::Separator();

    const ImGuiTableFlags flags = ImGuiTableFlags_Borders |
                                  ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_SizingStretchProp |
                                  ImGuiTableFlags_ScrollY;
    if (ImGui::BeginTable("profile_table", 6, flags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Module");
        ImGui::TableSetupColumn("mean ms");
        ImGui::TableSetupColumn("last ms");
        ImGui::TableSetupColumn("max ms");
        ImGui::TableSetupColumn("calls");
        ImGui::TableSetupColumn("%");
        ImGui::TableHeadersRow();

        double mean_sum = 0.0;
        for (const ProfileRow& r : snap.rows) {
            mean_sum += static_cast<double>(r.mean_ms);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(r.name.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", r.mean_ms);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", r.last_ms);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", r.max_ms);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%llu", static_cast<unsigned long long>(r.count));
            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%.1f", r.percent);
        }

        // Total row: the sum of the per-block means (blocks may nest, so this is
        // a reference figure, not a wall-clock budget).
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("TOTAL (sum of means)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.3f", mean_sum);
        ImGui::EndTable();
    }

    ImGui::End();
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Pimpl
// ---------------------------------------------------------------------------
struct DebugViewer::Impl {
    explicit Impl(const Config& c) : cfg(c) {}

    Config cfg;

    std::string csv_path;
    std::string video_path;

    // imageId -> full telemetry record, built from the CSV.
    std::unordered_map<int, TelemetryRecord> csv_by_id;

    // Pre-loaded groundtruth batch (used when no video/CSV pairing is needed).
    std::vector<TelemetryRecord> preloaded;

    // Groundtruth polyline in ENU.
    std::vector<Eigen::Vector3f> gt_points;
    // Per-point roll/pitch/yaw (degrees, parallel to gt_points) so the GT
    // trajectory can be re-rendered as monochrome green pose-axes gizmos in
    // "EstOdom" mode (same convention as the uav_* coord frames).
    std::vector<Eigen::Vector3f> gt_rpy_deg;
    std::vector<std::string>     gt_coord_names;
    // Poses of the periodic GT coord-frame gizmos (parallel to gt_coord_names)
    // so they can be re-uploaded when the "GroundTruth" checkbox is re-enabled.
    std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>>
                                 gt_coord_poses;

    // Inferred (external) polyline in ENU, fed via pushRecord()/pushPose().
    std::vector<Eigen::Vector3f> inferred_points;
    // Per-point roll/pitch/yaw (degrees, parallel to inferred_points) so the
    // estimate trajectory can be re-rendered as pose-axes gizmos ("EstOdom").
    std::vector<Eigen::Vector3f> inferred_rpy_deg;
    // frame id -> index into gt_points, the lookup side of the fused <->
    // groundtruth error links (fused_frame_ids is the other side). Filled ONLY
    // by the metric pushGroundtruthPose() path: the GPS/barcode path carries
    // imageIds from a different numbering, and mixing the two would
    // manufacture false pairings.
    // Indices stay valid because gt_points only ever grows.
    std::unordered_map<uint64_t, std::size_t> gt_index_by_frame;

    // Discrete LOST-state markers in render units, fed via pushLostPose().
    std::vector<Eigen::Vector3f> lost_points;

    // Fused (VO+telemetry back-end) trajectory in render units, fed via
    // pushFusedPose(): positions plus parallel ENU orientation quaternions and
    // parallel source frame ids (splice key for pushFusedCorrection). The
    // magenta polyline uses positions only; the "EstOdom" RGB pose-axes use
    // positions + quaternions. Accumulates regardless of the "Fused" checkbox
    // (only rendering is gated), same as the estimate line. Frame ids arrive
    // monotonically non-decreasing (per-frame VO stream).
    std::vector<Eigen::Vector3f> fused_points;
    std::vector<Eigen::Quaternionf,
                Eigen::aligned_allocator<Eigen::Quaternionf>> fused_quats;
    std::vector<uint64_t>        fused_frame_ids;

    // Absolute-position fix markers (setAnchorFixes): a REPLACE-style snapshot,
    // because a fix's accepted/rejected verdict is only known later and the
    // producer re-pushes the corrected list. Producer stores raw display-ENU
    // metres under anchor_fix_mutex_; the render thread splits them into the
    // FOUR render-unit clouds below on the dirty flag — verdict (accepted /
    // rejected) crossed with origin (cadence / re-anchor), because a drawable
    // carries one colour and one point shape.
    std::mutex                   anchor_fix_mutex_;
    std::vector<AnchorFixMarker> anchor_fix_latest_;
    std::atomic<bool>            anchor_fix_dirty_{false};
    std::vector<Eigen::Vector3f> anchor_fix_accepted_render_;
    std::vector<Eigen::Vector3f> anchor_fix_rejected_render_;
    std::vector<Eigen::Vector3f> anchor_fix_reinit_accepted_render_;
    std::vector<Eigen::Vector3f> anchor_fix_reinit_rejected_render_;

    // Run-control button (setRunControl / setRunState). The handler is owned by
    // the driver and invoked WITHOUT run_control_mutex_ held (it calls into the
    // pipeline, which may take its own locks and join threads).
    std::mutex                          run_control_mutex_;
    DebugViewer::RunControlHandler      run_control_;
    std::atomic<bool>                   run_control_set_{false};
    std::atomic<DebugViewer::RunState>  run_state_{DebugViewer::RunState::IDLE};

    // Two banner lines drawn at the top of the main panel (setAnchorOverlay):
    // a permanent RED warning and a plain counter line.
    std::mutex  banner_mutex_;
    std::string banner_warning_;
    std::string banner_stats_;

    // Latest map-point cloud (drop-oldest snapshot), fed via pushMapPoints().
    // Producer stores raw display-ENU metres under map_points_mutex_; the render
    // thread scales it into map_points_render_ (render units) on the dirty flag.
    std::mutex                   map_points_mutex_;
    std::vector<Eigen::Vector3f> map_points_latest_;
    std::atomic<bool>            map_points_dirty_{false};
    std::vector<Eigen::Vector3f> map_points_render_;

    // ENU origin (first valid groundtruth record).
    bool   origin_set = false;
    double origin_lat = 0.0, origin_lon = 0.0, origin_alt = 0.0;

    // Thread-safe ingestion queue for external/inferred records.
    std::mutex                 queue_mutex;
    std::deque<TelemetryRecord> inferred_queue;

    // A fused pose sample as pushed: display-ENU position + ENU orientation +
    // the source frame id (splice key for corrections).
    struct FusedSample {
        Eigen::Vector3f    pos;
        Eigen::Quaternionf q;
        uint64_t           frame_id = 0;
    };

    // A fused-channel event, drained by the render loop in push order (a
    // single queue keeps live samples and corrections strictly ordered):
    // either one live sample (is_correction == false, `correction` empty) or
    // a smoother-correction batch (is_correction == true) that replaces every
    // stored sample with frame_id >= correction.front().frame_id.
    struct FusedEvent {
        bool                         is_correction = false;
        FusedSample                  sample;
        std::vector<FusedCorrection> correction;
    };

    // Thread-safe metric pose queues (already in display ENU; no GPS round-trip).
    std::deque<InferredPose>   est_q_;   // estimate -> inferred polyline (orange)
    std::deque<InferredPose>   gt_q_;    // groundtruth -> gt polyline (green)
    std::deque<InferredPose>   lost_q_;  // LOST positions -> red markers
    std::deque<FusedEvent,
               Eigen::aligned_allocator<FusedEvent>> fused_q_; // fused events

    std::atomic<bool> stop_flag{false};
    std::atomic<bool> had_display{false};

    // ---- UI control panel ---------------------------------------------------
    // Metric samples (producer: worker) drained under queue_mutex like the poses.
    std::deque<FrameMetrics> metric_q_;

    // Render-thread-only metric history (x = frame id) feeding the line chart.
    std::vector<double> frame_x_;
    std::vector<double> inliers_hist_;
    std::vector<double> landmarks_hist_;

    // Performance samples (producer: any thread) drained under queue_mutex like
    // the metrics, into the render-thread-only histories below (x = t_sec).
    std::deque<PerfSample> perf_q_;
    std::vector<double>    perf_t_;
    std::vector<double>    perf_cpu_;
    std::vector<double>    perf_rss_;

    // Predicted-vs-groundtruth error samples (producer: any thread) drained
    // under queue_mutex like the metrics, into the render-thread-only histories
    // feeding the "Error (m)" window (x = frame id).
    std::deque<ErrorSample> error_q_;
    std::vector<double>     error_frame_x_;
    std::vector<double>     error_2d_;
    std::vector<double>     error_3d_;

    // Profiling table: a single LATEST-WINS slot (the table shows only the
    // newest snapshot, so queueing older ones would be pure waste). Producer:
    // any thread, under queue_mutex; consumer: the render thread, which copies
    // it into profile_render_ on the dirty flag.
    ProfileSnapshot   profile_latest_;
    std::atomic<bool> profile_dirty_{false};
    ProfileSnapshot   profile_render_;

    // HUD throughput figures: same LATEST-WINS shape as the profiling table
    // (only the newest reading is ever shown, so queueing older ones is waste).
    // Producer: the monitor thread, under queue_mutex; consumer: the render
    // thread, which copies it into throughput_render_ on the dirty flag.
    ThroughputSample  throughput_latest_;
    std::atomic<bool> throughput_dirty_{false};
    ThroughputSample  throughput_render_;

    // Latest video frame: single drop-oldest slot (producer: worker).
    std::mutex        frame_mutex_;
    cv::Mat           frame_latest_;
    std::atomic<bool> frame_dirty_{false};

    // Latest tracking-overlay frame: separate single drop-oldest slot so the
    // "Tracking" sub-window is independent of the plain "Video stream" panel.
    std::mutex        tracking_mutex_;
    cv::Mat           tracking_latest_;
    std::atomic<bool> tracking_dirty_{false};

    // Captured log lines: written by the sink (any thread), read by render thread.
    std::shared_ptr<LogRing>     log_ring_;
    std::shared_ptr<LogRingSink> log_sink_;

    // Render-thread-only state (no lock needed): checkbox flags + whether the
    // matching GL resource is currently live (for tear-down on toggle-off).
    bool show_log_       = false;
    bool show_inliers_   = false;
    bool show_landmarks_ = false;
    bool show_video_        = false;
    bool video_active_      = false;
    bool show_tracking_     = false;
    bool tracking_active_   = false;
    bool show_lost_poses_   = false;
    bool lost_markers_active_ = false;
    bool map_points_active_ = false;
    // "Anchor fixes" checkbox + whether each of the four marker clouds is live.
    bool show_anchor_fixes_       = true;
    bool anchor_fix_accepted_active_ = false;
    bool anchor_fix_rejected_active_ = false;
    bool anchor_fix_reinit_accepted_active_ = false;
    bool anchor_fix_reinit_rejected_active_ = false;
    bool show_hud_          = false;
    bool show_perf_         = false;
    bool show_profile_      = false;
    // "Error (m)" checkbox + which of the two error series the window plots
    // (in-window "3D (include altitude)" checkbox; horizontal is the default).
    bool show_error_        = false;
    bool error_use_3d_      = false;
    // "VO" checkbox: master gate for the VO estimate trajectory (orange
    // polyline, or orange pose-axes chain in EstOdom mode). Estimate data
    // keeps accumulating while hidden — same semantics as GroundTruth.
    bool show_estimate_     = true;
    // "GroundTruth" checkbox: gates the rendering of the GT polyline + its
    // coord-frame gizmos (data keeps accumulating while hidden).
    bool show_groundtruth_  = true;
    bool gt_line_active_    = false;
    bool gt_gizmos_active_  = false;
    // "Fused" checkbox: gates the rendering of the fused polyline (data keeps
    // accumulating while hidden — same semantics as GroundTruth).
    bool show_fused_        = true;
    bool fused_line_active_ = false;
    // "EstOdom" checkbox: OFF = line rendering for all three trajectories,
    // ON = per-pose axes chains (GT green, estimate orange, fused RGB).
    bool show_est_odom_        = false;
    // "Follow camera" checkbox: gates the periodic auto-recenter lookat().
    bool follow_camera_        = true;
    bool inferred_line_active_ = false;
    // EstOdom error link set (fused pose -> groundtruth of the same frame):
    // it has no checkbox of its own, it follows the EstOdom mode.
    bool fused_links_active_ = false;
    // Names of the live pose-axes drawables per trajectory (for tear-down on
    // toggle) and the next buffer index not yet considered for an axes gizmo.
    // Same swap mechanism for all three: estimate / groundtruth / fused.
    std::vector<std::string> est_axes_names_;
    std::size_t              est_axes_next_idx_ = 0;
    std::vector<std::string> gt_axes_names_;
    std::size_t              gt_axes_next_idx_ = 0;
    std::vector<std::string> fused_axes_names_;
    std::size_t              fused_axes_next_idx_ = 0;

    // Latest HUD scalars (render-thread-only; refreshed by drain_aux).
    float hud_heading_     = 0.0f;
    float hud_heading_tel_ = 0.0f;
    float hud_distance_    = 0.0f;

    BarcodeDecoder decoder;

    // Bridge to a core::SystemManager (attach()): the four channel handlers,
    // the display anchoring, and the two threads that own the run. Null until
    // attach() is called — a bare viewer (demo_debug_viewer, TrajectoryViewer
    // users) never allocates one.
    std::unique_ptr<SystemBridge> bridge;

    // ---- helpers -----------------------------------------------------------

    Eigen::Vector3f toEnu(const TelemetryRecord& r) {
        if (!origin_set) {
            origin_lat = r.latitude;
            origin_lon = r.longitude;
            origin_alt = r.altitude_m;
            origin_set = true;
        }
        auto p = gps_to_enu(r.latitude, r.longitude, r.altitude_m,
                            origin_lat, origin_lon, origin_alt);
        const float s = cfg.display_scale;
        // Altitude (Up) gets an extra vertical_scale so it can be exaggerated or
        // flattened independently of the horizontal plane.
        return Eigen::Vector3f(p.e * s, p.n * s, p.u * s * cfg.vertical_scale);
    }

    // Scale a metric ENU point into render units. Mirrors the scaling applied by
    // toEnu() but skips the GPS conversion — the input is already display ENU
    // (metres): display_scale uniformly, plus vertical_scale on Up only.
    Eigen::Vector3f applyScale(double x, double y, double z) const {
        const float s = cfg.display_scale;
        return Eigen::Vector3f(static_cast<float>(x) * s,
                               static_cast<float>(y) * s,
                               static_cast<float>(z) * s * cfg.vertical_scale);
    }

    // Build a coordinate-frame gizmo transform at render-unit position `pt`
    // from roll/pitch/yaw in degrees (ZYX: yaw about Up, then pitch, then
    // roll — the convention of the GT coord frames). `scale_m` is the gizmo
    // size in metres; display_scale converts it into render units. Shared by
    // the GT stream gizmos and the "EstOdom" per-pose axes so both use the
    // exact same orientation convention.
    Eigen::Affine3f gizmoTransform(const Eigen::Vector3f& pt,
                                   float roll_deg, float pitch_deg,
                                   float yaw_deg, float scale_m) const {
        const float y  = yaw_deg   * static_cast<float>(DEG2RAD);
        const float p  = pitch_deg * static_cast<float>(DEG2RAD);
        const float ro = roll_deg  * static_cast<float>(DEG2RAD);
        return Eigen::Translation3f(pt)
            * Eigen::AngleAxisf(y,  Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(p,  Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(ro, Eigen::Vector3f::UnitX())
            * Eigen::UniformScaling<float>(scale_m * cfg.display_scale);
    }

    // Pose-axes gizmo transform from an orientation quaternion (already in
    // the display/ENU frame) instead of roll/pitch/yaw — used by the fused
    // trajectory whose orientation arrives as a quaternion. Same scaling
    // treatment as gizmoTransform.
    Eigen::Affine3f gizmoTransformQuat(const Eigen::Vector3f& pt,
                                       const Eigen::Quaternionf& q,
                                       float scale_m) const {
        return Eigen::Translation3f(pt) * q.normalized()
            * Eigen::UniformScaling<float>(scale_m * cfg.display_scale);
    }

    bool lookupTelemetry(int image_id, TelemetryRecord& out) const {
        auto it = csv_by_id.find(image_id);
        if (it == csv_by_id.end()) return false;
        out = it->second;
        return true;
    }
};

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
DebugViewer::DebugViewer() : impl_(std::make_unique<Impl>(Config{})) {}

DebugViewer::DebugViewer(const Config& cfg) : impl_(std::make_unique<Impl>(cfg)) {}

DebugViewer::~DebugViewer() = default;

// ---------------------------------------------------------------------------
// Data loading
// ---------------------------------------------------------------------------
void DebugViewer::loadBatch(const std::vector<TelemetryRecord>& records) {
    impl_->preloaded = records;
    spdlog::info("DebugViewer: pre-loaded {} groundtruth records", records.size());
}

void DebugViewer::loadDirectory(const std::string& dir_path) {
    namespace fs = std::filesystem;

    if (!fs::is_directory(dir_path)) {
        spdlog::error("DebugViewer::loadDirectory: not a directory: {}", dir_path);
        return;
    }

    std::string found_csv, found_video;
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (ext == ".csv" && found_csv.empty()) {
            found_csv = entry.path().string();
        } else if (std::find(VIDEO_EXTENSIONS.begin(), VIDEO_EXTENSIONS.end(), ext)
                       != VIDEO_EXTENSIONS.end() && found_video.empty()) {
            found_video = entry.path().string();
        }
    }

    impl_->csv_path   = found_csv;
    impl_->video_path = found_video;
    spdlog::info("DebugViewer::loadDirectory: csv='{}' video='{}'",
                 found_csv.empty() ? "<none>" : found_csv,
                 found_video.empty() ? "<none>" : found_video);

    if (!found_csv.empty()) {
        try {
            auto records = load_telemetry_csv(found_csv);
            for (const auto& r : records) impl_->csv_by_id[r.frame_id] = r;
            spdlog::info("DebugViewer: indexed {} CSV records by imageId", impl_->csv_by_id.size());
        } catch (const std::exception& ex) {
            spdlog::error("DebugViewer: failed to load CSV: {}", ex.what());
        }
    }
}

void DebugViewer::pushRecord(const TelemetryRecord& r) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->inferred_queue.push_back(r);
}

void DebugViewer::pushPose(const InferredPose& p) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->est_q_.push_back(p);
}

void DebugViewer::pushGroundtruthPose(const InferredPose& p) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->gt_q_.push_back(p);
}

void DebugViewer::pushLostPose(const InferredPose& p) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->lost_q_.push_back(p);
}

void DebugViewer::pushFusedPose(const Eigen::Vector3f& enu_pos,
                                const Eigen::Quaternionf& q_enu,
                                uint64_t frame_id) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    Impl::FusedEvent ev;
    ev.sample = {enu_pos, q_enu, frame_id};
    impl_->fused_q_.push_back(std::move(ev));
}

void DebugViewer::pushFusedCorrection(const std::vector<FusedCorrection>& corrected) {
    if (corrected.empty()) return;
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    Impl::FusedEvent ev;
    ev.is_correction = true;
    ev.correction    = corrected;
    impl_->fused_q_.push_back(std::move(ev));
}

void DebugViewer::setAnchorFixes(const std::vector<AnchorFixMarker>& fixes) {
    {
        std::lock_guard<std::mutex> lock(impl_->anchor_fix_mutex_);
        impl_->anchor_fix_latest_ = fixes;
    }
    impl_->anchor_fix_dirty_.store(true);
}

void DebugViewer::setAnchorOverlay(const std::string& warning,
                                   const std::string& stats) {
    std::lock_guard<std::mutex> lock(impl_->banner_mutex_);
    impl_->banner_warning_ = warning;
    impl_->banner_stats_   = stats;
}

// ---------------------------------------------------------------------------
// Bridge to the pipeline — thin delegation; the logic lives in SystemBridge
// (src/debug_viewer/debug_viewer_bridge.{h,cpp}). It sits here only because
// SystemBridge is owned by the pimpl, which is private to this TU.
// ---------------------------------------------------------------------------
void DebugViewer::attach(core::SystemManager& sys) {
    if (!impl_->bridge) {
        impl_->bridge = std::make_unique<SystemBridge>(*this, impl_->cfg);
    }
    impl_->bridge->attach(sys);
}

void DebugViewer::setFusedPoseSink(FusedPoseSink sink) {
    if (!impl_->bridge) {
        impl_->bridge = std::make_unique<SystemBridge>(*this, impl_->cfg);
    }
    impl_->bridge->setFusedPoseSink(std::move(sink));
}

void DebugViewer::pushAnchorFix(const Eigen::Vector2d& xy_fusion_enu,
                                bool from_reinit) {
    if (!impl_->bridge) {
        spdlog::warn("DebugViewer::pushAnchorFix: no system attached — fix dropped");
        return;
    }
    impl_->bridge->pushAnchorFix(xy_fusion_enu, from_reinit);
}

void DebugViewer::enableAnchorFixTracking(const std::string& contamination_warning) {
    if (!impl_->bridge) {
        impl_->bridge = std::make_unique<SystemBridge>(*this, impl_->cfg);
    }
    impl_->bridge->enableAnchorFixTracking(contamination_warning);
}

AnchorFixCounters DebugViewer::anchorFixCounters() const {
    if (!impl_->bridge) {
        return AnchorFixCounters{};
    }
    return impl_->bridge->anchorFixCounters();
}

void DebugViewer::setRunControl(RunControlHandler handler) {
    const bool has = static_cast<bool>(handler);
    {
        std::lock_guard<std::mutex> lock(impl_->run_control_mutex_);
        impl_->run_control_ = std::move(handler);
    }
    impl_->run_control_set_.store(has);
    spdlog::debug("DebugViewer: run control {}", has ? "installed" : "removed");
}

void DebugViewer::setRunState(RunState state) {
    impl_->run_state_.store(state);
}

DebugViewer::RunState DebugViewer::runState() const {
    return impl_->run_state_.load();
}

void DebugViewer::pushMapPoints(const std::vector<Eigen::Vector3f>& pts_enu) {
    {
        // Replace the single drop-oldest snapshot so the render thread always
        // reads a stable, complete cloud (never a partial copy).
        std::lock_guard<std::mutex> lock(impl_->map_points_mutex_);
        impl_->map_points_latest_ = pts_enu;
    }
    impl_->map_points_dirty_.store(true);
}

void DebugViewer::pushMetrics(const FrameMetrics& m) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->metric_q_.push_back(m);
}

void DebugViewer::pushPerf(const PerfSample& s) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->perf_q_.push_back(s);
}

void DebugViewer::pushErrorSample(const ErrorSample& s) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->error_q_.push_back(s);
}

void DebugViewer::pushProfile(const ProfileSnapshot& p) {
    {
        // Replace the single latest-wins slot so the render thread always reads
        // a complete table (never a half-updated one).
        std::lock_guard<std::mutex> lock(impl_->queue_mutex);
        impl_->profile_latest_ = p;
    }
    impl_->profile_dirty_.store(true);
}

void DebugViewer::pushThroughput(const ThroughputSample& s) {
    {
        // Replace the single latest-wins slot so the render thread always reads
        // a consistent set of figures (never a half-updated one).
        std::lock_guard<std::mutex> lock(impl_->queue_mutex);
        impl_->throughput_latest_ = s;
    }
    impl_->throughput_dirty_.store(true);
}

void DebugViewer::pushFrame(const cv::Mat& image) {
    if (image.empty()) return;
    {
        // Clone into the single slot (drop-oldest) so the worker never blocks on
        // the render thread and the render thread reads a stable buffer.
        std::lock_guard<std::mutex> lock(impl_->frame_mutex_);
        image.copyTo(impl_->frame_latest_);
    }
    impl_->frame_dirty_.store(true);
}

void DebugViewer::pushTrackingFrame(const cv::Mat& image) {
    if (image.empty()) return;
    {
        // Independent drop-oldest slot for the tracking-overlay sub-window.
        std::lock_guard<std::mutex> lock(impl_->tracking_mutex_);
        image.copyTo(impl_->tracking_latest_);
    }
    impl_->tracking_dirty_.store(true);
}

std::shared_ptr<spdlog::sinks::sink> DebugViewer::logSink() {
    if (!impl_->log_sink_) {
        impl_->log_ring_ = std::make_shared<LogRing>();
        impl_->log_ring_->capacity =
            static_cast<std::size_t>(std::max(1, impl_->cfg.log_capacity));
        impl_->log_sink_ = std::make_shared<LogRingSink>(impl_->log_ring_);
    }
    return impl_->log_sink_;
}

void DebugViewer::stop() {
    impl_->stop_flag.store(true);
}

bool DebugViewer::hadDisplay() const {
    return impl_->had_display.load();
}

// ---------------------------------------------------------------------------
// run()
// ---------------------------------------------------------------------------
void DebugViewer::run() {
    impl_->stop_flag.store(false);

    // Seed the render-thread-only checkbox state from the config defaults.
    impl_->show_log_       = impl_->cfg.show_log;
    impl_->show_inliers_   = impl_->cfg.show_inliers;
    impl_->show_landmarks_ = impl_->cfg.show_landmarks;
    impl_->show_video_     = impl_->cfg.show_video;
    impl_->show_tracking_  = impl_->cfg.show_tracking;
    impl_->show_lost_poses_ = impl_->cfg.show_lost_poses;
    impl_->show_hud_       = impl_->cfg.show_hud;
    impl_->show_perf_      = impl_->cfg.show_perf;
    impl_->show_profile_   = impl_->cfg.show_profile;
    impl_->show_error_     = impl_->cfg.show_error;
    impl_->show_estimate_    = impl_->cfg.show_estimate;
    impl_->show_groundtruth_ = impl_->cfg.show_groundtruth;
    impl_->show_est_odom_    = impl_->cfg.est_odom;
    impl_->follow_camera_    = impl_->cfg.follow_camera;
    impl_->show_fused_       = impl_->cfg.show_fused;
    impl_->show_anchor_fixes_ = impl_->cfg.show_anchor_fixes;

    // --- Own the run: auto-start + supervisor + perf sampler (S4). ----------
    // Deliberately BEFORE the GL init, which is where the driver used to do it:
    // the pipeline must come up at the same point in the sequence as before.
    // A refused auto-start spawned no thread, so there is nothing to join and
    // returning here cannot hang.
    if (impl_->bridge && !impl_->bridge->begin()) {
        return;
    }

    // --- Decide whether a GL window can be opened. -------------------------
    guik::LightViewer* viewer = nullptr;
    // Treat an unset OR empty DISPLAY/WAYLAND_DISPLAY as headless. `DISPLAY=`
    // yields a non-null but empty string, which would otherwise pass the null
    // check and crash GLFW init.
    auto env_nonempty = [](const char* key) {
        const char* v = std::getenv(key);
        return v != nullptr && v[0] != '\0';
    };
    const bool has_display = env_nonempty("DISPLAY") || env_nonempty("WAYLAND_DISPLAY");
    if (!has_display) {
        spdlog::warn("DebugViewer: no DISPLAY/WAYLAND_DISPLAY — running headless (no rendering)");
    } else {
        try {
            viewer = guik::LightViewer::instance(
                Eigen::Vector2i(impl_->cfg.window_w, impl_->cfg.window_h),
                /*background=*/false, impl_->cfg.window_title);
            if (viewer == nullptr) {
                spdlog::warn("DebugViewer: LightViewer::instance() returned null — running headless");
            }
        } catch (const std::exception& ex) {
            spdlog::warn("DebugViewer: GL init failed ({}) — running headless", ex.what());
            viewer = nullptr;
        } catch (...) {
            spdlog::warn("DebugViewer: GL init failed (unknown) — running headless");
            viewer = nullptr;
        }
    }

    impl_->had_display.store(viewer != nullptr);

    if (viewer != nullptr) {
        viewer->use_orbit_camera_control();
        viewer->register_ui_callback("debug_status", [this]() {
            ImGui::SetNextWindowSize(ImVec2(320, 200), ImGuiCond_FirstUseEver);
            ImGui::Begin("Debug Viewer");
            // Run control ABOVE the banner: it carries no numbers, so it cannot
            // detach a figure from its caveat, and it is the one widget the user
            // looks for first. Drawn only when a driver installed a handler.
            if (impl_->run_control_set_.load()) {
                const RunState state = impl_->run_state_.load();
                const bool running  = (state == RunState::RUNNING);
                const bool finished = (state == RunState::FINISHED);
                // Terminal state: the pipeline is down for good, so the widget
                // must not invite a click. Grey it out and say why, instead of
                // showing "Start" and refusing in the log where nobody looks.
                if (finished) {
                    ImGui::BeginDisabled(true);
                    ImGui::Button(RUN_BUTTON_FINISHED);
                    ImGui::EndDisabled();
                    ImGui::SameLine();
                    ImGui::TextDisabled("%s", RUN_FINISHED_HINT);
                } else if (ImGui::Button(running ? RUN_BUTTON_STOP
                                                 : RUN_BUTTON_START)) {
                    const bool want_run = !running;
                    RunControlHandler handler;
                    {
                        std::lock_guard<std::mutex> lock(impl_->run_control_mutex_);
                        handler = impl_->run_control_;
                    }
                    // Called with no viewer lock held: the handler drives the
                    // pipeline and may block (it joins the reading thread).
                    // Exceptions must not escape into the GL loop.
                    bool ok = false;
                    try {
                        ok = handler && handler(want_run);
                    } catch (const std::exception& ex) {
                        spdlog::error("DebugViewer: run control threw: {}", ex.what());
                    } catch (...) {
                        spdlog::error("DebugViewer: run control threw a non-std exception");
                    }
                    if (ok) {
                        impl_->run_state_.store(want_run ? RunState::RUNNING
                                                         : RunState::PAUSED);
                        spdlog::info("DebugViewer: run control -> {}",
                                     want_run ? "RUNNING" : "PAUSED");
                    } else {
                        spdlog::error("DebugViewer: run control refused the "
                                      "{} request — button state unchanged",
                                      want_run ? "start/resume" : "pause");
                    }
                }
                ImGui::Separator();
            }
            // Contamination banner FIRST, before any number this window shows:
            // a screenshot must carry the caveat with it.
            {
                std::string warning, stats;
                {
                    std::lock_guard<std::mutex> lock(impl_->banner_mutex_);
                    warning = impl_->banner_warning_;
                    stats   = impl_->banner_stats_;
                }
                if (!warning.empty()) {
                    ImGui::PushStyleColor(
                        ImGuiCol_Text,
                        ImVec4(WARNING_TEXT_COLOR[0], WARNING_TEXT_COLOR[1],
                               WARNING_TEXT_COLOR[2], WARNING_TEXT_COLOR[3]));
                    ImGui::TextWrapped("%s", warning.c_str());
                    ImGui::PopStyleColor();
                }
                if (!stats.empty()) {
                    ImGui::TextWrapped("%s", stats.c_str());
                }
                if (!warning.empty() || !stats.empty()) {
                    ImGui::Separator();
                }
            }
            ImGui::Text("Groundtruth pts : %zu", impl_->gt_points.size());
            ImGui::Text("Inferred pts    : %zu", impl_->inferred_points.size());
            if (impl_->origin_set) {
                ImGui::Text("Origin lat/lon  : %.6f, %.6f", impl_->origin_lat, impl_->origin_lon);
            }
            ImGui::Separator();
            // Control panel: each checkbox toggles a side panel. The flags are
            // render-thread-only state consumed by drain_aux()/the log window.
            ImGui::TextUnformatted("Panels");
            ImGui::Checkbox("Log terminal", &impl_->show_log_);
            ImGui::Checkbox("Inliers", &impl_->show_inliers_);
            ImGui::Checkbox("Landmarks", &impl_->show_landmarks_);
            ImGui::Checkbox("Video stream", &impl_->show_video_);
            ImGui::Checkbox("Tracking", &impl_->show_tracking_);
            ImGui::Checkbox("Lost state poses", &impl_->show_lost_poses_);
            ImGui::Checkbox("HUD", &impl_->show_hud_);
            ImGui::Checkbox("Performance", &impl_->show_perf_);
            ImGui::Checkbox("Profiling", &impl_->show_profile_);
            ImGui::Checkbox("Error (m)", &impl_->show_error_);
            ImGui::Checkbox("VO", &impl_->show_estimate_);
            ImGui::Checkbox("GroundTruth", &impl_->show_groundtruth_);
            ImGui::Checkbox("EstOdom", &impl_->show_est_odom_);
            ImGui::Checkbox("Follow camera", &impl_->follow_camera_);
            ImGui::Checkbox("Fused", &impl_->show_fused_);
            ImGui::Checkbox("Anchor fixes", &impl_->show_anchor_fixes_);
            ImGui::End();

            // HUD: VO course-over-ground heading, telemetry heading, path
            // length, and the live throughput block.
            if (impl_->show_hud_) {
                ImGui::SetNextWindowSize(ImVec2(260, 200), ImGuiCond_FirstUseEver);
                ImGui::Begin("HUD", &impl_->show_hud_);
                ImGui::Text("Heading(VO)  : %.1f deg", impl_->hud_heading_);
                ImGui::Text("Heading(tel) : %.1f deg", impl_->hud_heading_tel_);
                ImGui::Text("Distance     : %.1f m",   impl_->hud_distance_);
                ImGui::Separator();
                // FPS (Ns)   — WALL-CLOCK frames/s over the last N seconds. It
                //              reacts: a stalled pipeline decays it to 0.
                // FPS (mean) — CUMULATIVE mean since start, pipeline time only
                //              (idle and decode excluded). It converges and
                //              then stops moving.
                // Proc       — mean pipeline time per frame inside the window.
                // The two FPS numbers are NOT redundant: the first says what the
                // system is delivering right now, the second what the pipeline
                // alone is capable of on average.
                ImGui::Text("FPS (%.0fs)   : %.1f", impl_->throughput_render_.window_sec,
                                                    impl_->throughput_render_.fps_windowed);
                ImGui::Text("FPS (mean)   : %.1f",  impl_->throughput_render_.fps_mean);
                ImGui::Text("Proc         : %.0f ms/f", impl_->throughput_render_.proc_ms_mean);
                ImGui::End();
            }

            // Log terminal: scrolling read-only copy of the captured log lines.
            if (impl_->show_log_) {
                ImGui::SetNextWindowSize(ImVec2(560, 240), ImGuiCond_FirstUseEver);
                ImGui::Begin("Log terminal", &impl_->show_log_);
                if (ImGui::BeginChild("log_scroll", ImVec2(0, 0), false,
                                      ImGuiWindowFlags_HorizontalScrollbar)) {
                    std::vector<std::string> lines_copy;
                    if (impl_->log_ring_) {
                        std::lock_guard<std::mutex> lock(impl_->log_ring_->mutex);
                        lines_copy.assign(impl_->log_ring_->lines.begin(),
                                          impl_->log_ring_->lines.end());
                    }
                    for (const auto& l : lines_copy) {
                        ImGui::TextUnformatted(l.c_str());
                    }
                    // Auto-scroll to the bottom while the user is already at the end.
                    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) {
                        ImGui::SetScrollHereY(1.0f);
                    }
                }
                ImGui::EndChild();
                ImGui::End();
            }

            // Metric windows: each its own resizable ImGui window holding a
            // per-frame line chart + a value-distribution histogram. Data is read
            // straight from the render-thread-owned ring buffers (no lock).
            const int hist_bins   = impl_->cfg.histogram_bins;
            const int follow_win  = impl_->cfg.metric_plot_follow_window;
            if (impl_->show_inliers_) {
                drawMetricWindow("Inliers", &impl_->show_inliers_,
                                 impl_->frame_x_, impl_->inliers_hist_,
                                 "inliers", hist_bins, follow_win);
            }
            if (impl_->show_landmarks_) {
                drawMetricWindow("Landmarks", &impl_->show_landmarks_,
                                 impl_->frame_x_, impl_->landmarks_hist_,
                                 "landmarks", hist_bins, follow_win);
            }
            if (impl_->show_perf_) {
                drawPerfWindow(&impl_->show_perf_, impl_->perf_t_,
                               impl_->perf_cpu_, impl_->perf_rss_, follow_win);
            }
            if (impl_->show_profile_) {
                drawProfileWindow(&impl_->show_profile_, impl_->profile_render_);
            }
            if (impl_->show_error_) {
                drawErrorWindow(&impl_->show_error_, impl_->error_frame_x_,
                                impl_->error_2d_, impl_->error_3d_,
                                &impl_->error_use_3d_, hist_bins, follow_win);
            }
        });
    }

    // --- Helper lambdas that touch GL (only used when viewer != nullptr). --
    // Upload the GT polyline. No-op while the "GroundTruth" checkbox is off or
    // EstOdom (axes style) is on — the gt_points buffer keeps accumulating
    // regardless (only rendering gated).
    auto refresh_gt_line = [&]() {
        if (viewer && impl_->show_groundtruth_ && !impl_->show_est_odom_ &&
            impl_->gt_points.size() >= 2) {
            viewer->update_drawable(
                "gt_trajectory",
                std::make_shared<glk::ThinLines>(impl_->gt_points, /*line_strip=*/true),
                guik::FlatColor(GT_COLOR[0], GT_COLOR[1], GT_COLOR[2], GT_COLOR[3]));
            impl_->gt_line_active_ = true;
        }
    };
    // GT rendering, gated by the "GroundTruth" checkbox (master) and styled by
    // "EstOdom": line mode = green polyline + periodic uav_* coord-frame gizmos
    // (coord_frame_every_n cadence); EstOdom mode = monochrome GREEN pose-axes
    // chain at est_odom_every_n cadence over the accumulated GT records (same
    // gizmoTransform orientation convention as the uav_* frames). Toggling
    // either checkbox swaps the drawables from the accumulated buffers
    // (gt_points/gt_rpy_deg + gt_coord_names/gt_coord_poses) — no data loss.
    // Called every drain pass.
    auto refresh_gt_display = [&]() {
        if (!viewer) return;
        const bool line_mode = impl_->show_groundtruth_ && !impl_->show_est_odom_;
        const bool axes_mode = impl_->show_groundtruth_ && impl_->show_est_odom_;
        // Tear down whichever style is no longer active (stale drawables must
        // not linger across a mode/visibility switch).
        if (!line_mode) {
            if (impl_->gt_line_active_) {
                viewer->remove_drawable("gt_trajectory");
                impl_->gt_line_active_ = false;
            }
            if (impl_->gt_gizmos_active_) {
                for (const std::string& name : impl_->gt_coord_names) {
                    viewer->remove_drawable(name);
                }
                impl_->gt_gizmos_active_ = false;
            }
        }
        if (!axes_mode &&
            (!impl_->gt_axes_names_.empty() || impl_->gt_axes_next_idx_ != 0)) {
            for (const std::string& name : impl_->gt_axes_names_) {
                viewer->remove_drawable(name);
            }
            impl_->gt_axes_names_.clear();
            impl_->gt_axes_next_idx_ = 0;
        }
        if (line_mode) {
            if (!impl_->gt_line_active_) {
                refresh_gt_line();
            }
            if (!impl_->gt_gizmos_active_) {
                for (std::size_t i = 0; i < impl_->gt_coord_names.size(); ++i) {
                    viewer->update_drawable(impl_->gt_coord_names[i],
                                            glk::Primitives::coordinate_system(),
                                            guik::VertexColor(impl_->gt_coord_poses[i]));
                }
                impl_->gt_gizmos_active_ = true;
            }
        } else if (axes_mode) {
            // Upload a green axes gizmo for every GT pose not yet considered
            // (covers both the back-fill right after the toggle, when
            // gt_axes_next_idx_ is 0, and fresh points).
            const std::size_t every = static_cast<std::size_t>(
                std::max(1, impl_->cfg.est_odom_every_n));
            while (impl_->gt_axes_next_idx_ < impl_->gt_points.size()) {
                const std::size_t i = impl_->gt_axes_next_idx_++;
                if (i % every != 0) continue;
                const Eigen::Vector3f& rpy = impl_->gt_rpy_deg[i];
                const Eigen::Affine3f T = impl_->gizmoTransform(
                    impl_->gt_points[i], rpy.x(), rpy.y(), rpy.z(),
                    impl_->cfg.est_odom_axes_scale);
                const std::string name = "gt_axes_" + std::to_string(i);
                viewer->update_drawable(
                    name, glk::Primitives::coordinate_system(),
                    guik::FlatColor(GT_COLOR[0], GT_COLOR[1], GT_COLOR[2],
                                    GT_COLOR[3], T));
                impl_->gt_axes_names_.push_back(name);
            }
        }
    };
    // Estimate-trajectory rendering, gated by the "VO" checkbox (master) with
    // the same semantics as the GroundTruth/Fused gates — estimate buffers
    // keep accumulating while hidden — and styled by "EstOdom": line mode =
    // the orange polyline; EstOdom mode = one monochrome ORANGE pose-axes
    // gizmo per (est_odom_every_n-th) estimate pose, oriented by the buffered
    // per-pose roll/pitch/yaw via the same gizmoTransform helper as the GT
    // coord frames (RGB axes belong to the fused trajectory). Toggling either
    // checkbox swaps the drawables from the SAME accumulated buffer — no data
    // loss; re-enabling re-uploads the full accumulated trajectory.
    // `new_data` forces a polyline re-upload when fresh points arrived.
    auto refresh_est_display = [&](bool new_data) {
        if (!viewer) return;
        const bool line_mode = impl_->show_estimate_ && !impl_->show_est_odom_;
        const bool axes_mode = impl_->show_estimate_ && impl_->show_est_odom_;
        // Tear down whichever style is no longer active (stale drawables must
        // not linger across a mode/visibility switch).
        if (!line_mode && impl_->inferred_line_active_) {
            viewer->remove_drawable("inferred_trajectory");
            impl_->inferred_line_active_ = false;
        }
        if (!axes_mode &&
            (!impl_->est_axes_names_.empty() || impl_->est_axes_next_idx_ != 0)) {
            for (const std::string& name : impl_->est_axes_names_) {
                viewer->remove_drawable(name);
            }
            impl_->est_axes_names_.clear();
            impl_->est_axes_next_idx_ = 0;
        }
        if (line_mode && impl_->inferred_points.size() >= 2) {
            // (Re-)upload the orange polyline on fresh points, or in full when
            // the line just became active (re-enable / axes->line swap).
            if (new_data || !impl_->inferred_line_active_) {
                viewer->update_drawable(
                    "inferred_trajectory",
                    std::make_shared<glk::ThinLines>(impl_->inferred_points,
                                                     /*line_strip=*/true),
                    guik::FlatColor(INFERRED_COLOR[0], INFERRED_COLOR[1],
                                    INFERRED_COLOR[2], INFERRED_COLOR[3]));
                impl_->inferred_line_active_ = true;
            }
        } else if (axes_mode) {
            // Upload an orange axes gizmo for every pose not yet considered
            // (covers both the back-fill right after the toggle, when
            // est_axes_next_idx_ is 0, and fresh points).
            const std::size_t every = static_cast<std::size_t>(
                std::max(1, impl_->cfg.est_odom_every_n));
            while (impl_->est_axes_next_idx_ < impl_->inferred_points.size()) {
                const std::size_t i = impl_->est_axes_next_idx_++;
                if (i % every != 0) continue;
                const Eigen::Vector3f& rpy = impl_->inferred_rpy_deg[i];
                const Eigen::Affine3f T = impl_->gizmoTransform(
                    impl_->inferred_points[i], rpy.x(), rpy.y(), rpy.z(),
                    impl_->cfg.est_odom_axes_scale);
                const std::string name = "est_axes_" + std::to_string(i);
                viewer->update_drawable(
                    name, glk::Primitives::coordinate_system(),
                    guik::FlatColor(INFERRED_COLOR[0], INFERRED_COLOR[1],
                                    INFERRED_COLOR[2], INFERRED_COLOR[3], T));
                impl_->est_axes_names_.push_back(name);
            }
        }
    };
    // Fused-trajectory rendering, gated by the "Fused" checkbox (master) with
    // the same semantics as the GroundTruth line — fused buffers keep
    // accumulating while hidden — and styled by "EstOdom": line mode = magenta
    // polyline (positions only); EstOdom mode = standard-RGB pose-axes chain
    // (X red, Y green, Z blue) at est_odom_every_n cadence, oriented by the
    // buffered per-pose ENU quaternion. Toggling either checkbox swaps the
    // drawables from the accumulated buffers (no data loss). `new_data` forces
    // a polyline re-upload when fresh points arrived this drain pass.
    auto refresh_fused_display = [&](bool new_data) {
        if (!viewer) return;
        const bool line_mode = impl_->show_fused_ && !impl_->show_est_odom_;
        const bool axes_mode = impl_->show_fused_ && impl_->show_est_odom_;
        // Tear down whichever style is no longer active.
        if (!line_mode && impl_->fused_line_active_) {
            viewer->remove_drawable("fused_trajectory");
            impl_->fused_line_active_ = false;
        }
        if (!axes_mode &&
            (!impl_->fused_axes_names_.empty() || impl_->fused_axes_next_idx_ != 0)) {
            for (const std::string& name : impl_->fused_axes_names_) {
                viewer->remove_drawable(name);
            }
            impl_->fused_axes_names_.clear();
            impl_->fused_axes_next_idx_ = 0;
        }
        if (line_mode && impl_->fused_points.size() >= 2) {
            if (new_data || !impl_->fused_line_active_) {
                viewer->update_drawable(
                    "fused_trajectory",
                    std::make_shared<glk::ThinLines>(impl_->fused_points,
                                                     /*line_strip=*/true),
                    guik::FlatColor(FUSED_COLOR[0], FUSED_COLOR[1],
                                    FUSED_COLOR[2], FUSED_COLOR[3]));
                impl_->fused_line_active_ = true;
            }
        } else if (axes_mode) {
            // Upload an RGB axes gizmo for every fused pose not yet considered
            // (back-fill after the toggle + fresh points).
            const std::size_t every = static_cast<std::size_t>(
                std::max(1, impl_->cfg.est_odom_every_n));
            while (impl_->fused_axes_next_idx_ < impl_->fused_points.size()) {
                const std::size_t i = impl_->fused_axes_next_idx_++;
                if (i % every != 0) continue;
                const Eigen::Affine3f T = impl_->gizmoTransformQuat(
                    impl_->fused_points[i], impl_->fused_quats[i],
                    impl_->cfg.est_odom_axes_scale);
                const std::string name = "fused_axes_" + std::to_string(i);
                viewer->update_drawable(name,
                                        glk::Primitives::coordinate_system(),
                                        guik::VertexColor(T));
                impl_->fused_axes_names_.push_back(name);
            }
        }
    };
    // EstOdom-only: one GL_LINES segment per drawn FUSED pose — the system's
    // final output, so the segment measures the error of the SYSTEM, not of
    // bare VO — joining it to the groundtruth pose of the SAME frame id, so
    // the per-frame error is readable directly off the 3D view. Deliberately
    // has NO checkbox of its own — selecting EstOdom shows it; each side is
    // still gated by its own trajectory checkbox, because a link whose
    // endpoint is hidden means nothing. Same `est_odom_every_n` cadence as the
    // axes gizmos, so every link starts on a drawn gizmo. A fused frame with
    // no groundtruth entry draws NO link — never paired with an approximate
    // neighbour, which would draw a wrong segment.
    auto refresh_error_links = [&](const char* name,
                                   const std::vector<Eigen::Vector3f>& pred_pts,
                                   const auto& pred_ids, bool visible,
                                   bool rebuild, bool& active,
                                   const float (&color)[4]) {
        if (!viewer) return;
        if (!visible) {
            if (active) {
                viewer->remove_drawable(name);
                active = false;
            }
            return;
        }
        if (active && !rebuild) return;
        const std::size_t every = static_cast<std::size_t>(
            std::max(1, impl_->cfg.est_odom_every_n));
        const std::size_t n = std::min(pred_pts.size(), pred_ids.size());
        std::vector<Eigen::Vector3f> verts;
        verts.reserve(2 * (n / every + 1));
        for (std::size_t i = 0; i < n; i += every) {
            const auto it =
                impl_->gt_index_by_frame.find(static_cast<uint64_t>(pred_ids[i]));
            if (it == impl_->gt_index_by_frame.end()) continue;
            verts.push_back(pred_pts[i]);
            verts.push_back(impl_->gt_points[it->second]);
        }
        if (verts.empty()) {
            if (active) {
                viewer->remove_drawable(name);
                active = false;
            }
            return;
        }
        viewer->update_drawable(
            name, std::make_shared<glk::ThinLines>(verts, /*line_strip=*/false),
            guik::FlatColor(color[0], color[1], color[2], color[3]));
        active = true;
    };
    // Discrete red markers at each LOST-tracking position. `rebuild` forces a
    // re-upload when new points arrived; otherwise this only handles the
    // checkbox toggle (draw on enable, remove on disable) without re-uploading.
    auto refresh_lost_markers = [&](bool rebuild) {
        if (!viewer) return;
        if (impl_->show_lost_poses_ && !impl_->lost_points.empty()) {
            if (rebuild || !impl_->lost_markers_active_) {
                viewer->update_drawable(
                    "lost_markers",
                    std::make_shared<glk::PointCloudBuffer>(impl_->lost_points),
                    guik::FlatColor(LOST_COLOR[0], LOST_COLOR[1],
                                    LOST_COLOR[2], LOST_COLOR[3])
                        .set_point_scale(LOST_MARKER_POINT_SCALE)
                        .set_point_shape_circle());
                impl_->lost_markers_active_ = true;
            }
        } else if (impl_->lost_markers_active_) {
            viewer->remove_drawable("lost_markers");
            impl_->lost_markers_active_ = false;
        }
    };
    // Absolute-position fix markers, gated by the "Anchor fixes" checkbox: four
    // clouds, the accepted/rejected verdict crossed with the cadence/re-anchor
    // origin (yellow and purple CIRCLES for the regular cadence, green and
    // orange SQUARES for a re-anchor). A fresh setAnchorFixes() snapshot
    // re-splits and re-uploads all of them; otherwise this only honours the
    // checkbox toggle.
    auto refresh_anchor_fixes = [&]() {
        if (!viewer) return;
        bool rebuild = false;
        if (impl_->anchor_fix_dirty_.exchange(false)) {
            std::lock_guard<std::mutex> lock(impl_->anchor_fix_mutex_);
            impl_->anchor_fix_accepted_render_.clear();
            impl_->anchor_fix_rejected_render_.clear();
            impl_->anchor_fix_reinit_accepted_render_.clear();
            impl_->anchor_fix_reinit_rejected_render_.clear();
            for (const AnchorFixMarker& m : impl_->anchor_fix_latest_) {
                const Eigen::Vector3f p =
                    impl_->applyScale(m.pos.x(), m.pos.y(), m.pos.z());
                if (m.from_reinit) {
                    if (m.accepted) {
                        impl_->anchor_fix_reinit_accepted_render_.push_back(p);
                    } else {
                        impl_->anchor_fix_reinit_rejected_render_.push_back(p);
                    }
                } else if (m.accepted) {
                    impl_->anchor_fix_accepted_render_.push_back(p);
                } else {
                    impl_->anchor_fix_rejected_render_.push_back(p);
                }
            }
            rebuild = true;
        }
        auto refresh_one = [&](const char* name,
                               const std::vector<Eigen::Vector3f>& pts,
                               const float (&color)[4], float point_scale,
                               bool square, bool& active) {
            if (impl_->show_anchor_fixes_ && !pts.empty()) {
                if (rebuild || !active) {
                    guik::ShaderSetting setting =
                        guik::FlatColor(color[0], color[1], color[2], color[3]);
                    setting.set_point_scale(point_scale);
                    if (square) {
                        setting.set_point_shape_rectangle();
                    } else {
                        setting.set_point_shape_circle();
                    }
                    viewer->update_drawable(
                        name, std::make_shared<glk::PointCloudBuffer>(pts),
                        setting);
                    active = true;
                }
            } else if (active) {
                viewer->remove_drawable(name);
                active = false;
            }
        };
        refresh_one("anchor_fix_accepted", impl_->anchor_fix_accepted_render_,
                    FIX_ACCEPTED_COLOR, FIX_MARKER_POINT_SCALE, false,
                    impl_->anchor_fix_accepted_active_);
        refresh_one("anchor_fix_rejected", impl_->anchor_fix_rejected_render_,
                    FIX_REJECTED_COLOR, FIX_MARKER_POINT_SCALE, false,
                    impl_->anchor_fix_rejected_active_);
        refresh_one("anchor_fix_reinit_accepted",
                    impl_->anchor_fix_reinit_accepted_render_,
                    FIX_REINIT_ACCEPTED_COLOR, FIX_REINIT_MARKER_POINT_SCALE,
                    true, impl_->anchor_fix_reinit_accepted_active_);
        refresh_one("anchor_fix_reinit_rejected",
                    impl_->anchor_fix_reinit_rejected_render_,
                    FIX_REINIT_REJECTED_COLOR, FIX_REINIT_MARKER_POINT_SCALE,
                    true, impl_->anchor_fix_reinit_rejected_active_);
    };
    // VO map-point cloud (light blue), gated by the "Landmarks" checkbox. When a
    // fresh snapshot arrived (dirty), it is scaled into render units and the
    // drawable is re-uploaded; otherwise this only honours the checkbox toggle
    // (draw on enable / remove on disable) without re-uploading.
    auto refresh_map_points = [&]() {
        if (!viewer) return;
        bool rebuild = false;
        if (impl_->map_points_dirty_.load()) {
            std::lock_guard<std::mutex> lock(impl_->map_points_mutex_);
            impl_->map_points_render_.clear();
            impl_->map_points_render_.reserve(impl_->map_points_latest_.size());
            for (const auto& p : impl_->map_points_latest_) {
                impl_->map_points_render_.push_back(
                    impl_->applyScale(p.x(), p.y(), p.z()));
            }
            impl_->map_points_dirty_.store(false);
            rebuild = true;
        }
        if (impl_->show_landmarks_ && !impl_->map_points_render_.empty()) {
            if (rebuild || !impl_->map_points_active_) {
                viewer->update_drawable(
                    "map_points",
                    std::make_shared<glk::PointCloudBuffer>(impl_->map_points_render_),
                    guik::FlatColor(MAP_POINT_COLOR[0], MAP_POINT_COLOR[1],
                                    MAP_POINT_COLOR[2], MAP_POINT_COLOR[3])
                        .set_point_scale(MAP_POINT_SCALE)
                        .set_point_shape_circle());
                impl_->map_points_active_ = true;
            }
        } else if (impl_->map_points_active_) {
            viewer->remove_drawable("map_points");
            impl_->map_points_active_ = false;
        }
    };

    auto append_gt = [&](const TelemetryRecord& r) {
        DebugViewerCallbacks::on_telemetry(r);
        const Eigen::Vector3f pt = impl_->toEnu(r);
        impl_->gt_points.push_back(pt);
        impl_->gt_rpy_deg.emplace_back(static_cast<float>(r.roll_deg),
                                       static_cast<float>(r.pitch_deg),
                                       static_cast<float>(r.yaw_deg));

        // Periodic heading coord frame from yaw. The name + pose are always
        // buffered (so a hidden trajectory can be fully re-uploaded when the
        // "GroundTruth" checkbox is re-enabled); the upload itself is gated on
        // both the checkbox AND line mode (EstOdom replaces the uav_* gizmos
        // with the green pose-axes chain).
        const int every = impl_->cfg.coord_frame_every_n;
        if (viewer && every > 0 &&
            (static_cast<int>(impl_->gt_points.size()) % every) == 0) {
            const Eigen::Affine3f T = impl_->gizmoTransform(
                pt, static_cast<float>(r.roll_deg),
                static_cast<float>(r.pitch_deg),
                static_cast<float>(r.yaw_deg), impl_->cfg.coord_frame_scale);
            const std::string name = "uav_" + std::to_string(r.frame_id);
            impl_->gt_coord_names.push_back(name);
            impl_->gt_coord_poses.push_back(T);
            if (impl_->show_groundtruth_ && !impl_->show_est_odom_) {
                viewer->update_drawable(name,
                                        glk::Primitives::coordinate_system(),
                                        guik::VertexColor(T));
            }
        }

        refresh_gt_line();
        if (viewer && impl_->follow_camera_ && (static_cast<int>(impl_->gt_points.size()) % CAMERA_RECENTER_EVERY_N) == 0) {
            viewer->lookat(pt);
        }
    };

    // Drain queued inferred records (GPS) + metric pose queues (already ENU)
    // into their respective polylines.
    auto drain_inferred = [&]() {
        std::deque<TelemetryRecord> local;
        std::deque<InferredPose>    est_local;
        std::deque<InferredPose>    gt_local;
        std::deque<InferredPose>    lost_local;
        std::deque<Impl::FusedEvent,
                   Eigen::aligned_allocator<Impl::FusedEvent>> fused_local;
        {
            std::lock_guard<std::mutex> lock(impl_->queue_mutex);
            local.swap(impl_->inferred_queue);
            est_local.swap(impl_->est_q_);
            gt_local.swap(impl_->gt_q_);
            lost_local.swap(impl_->lost_q_);
            fused_local.swap(impl_->fused_q_);
        }
        for (const auto& r : local) {
            impl_->inferred_points.push_back(impl_->toEnu(r));
            impl_->inferred_rpy_deg.emplace_back(
                static_cast<float>(r.roll_deg),
                static_cast<float>(r.pitch_deg),
                static_cast<float>(r.yaw_deg));
        }
        for (const auto& p : est_local) {
            impl_->inferred_points.push_back(impl_->applyScale(p.x, p.y, p.z));
            impl_->inferred_rpy_deg.emplace_back(
                static_cast<float>(p.roll_deg),
                static_cast<float>(p.pitch_deg),
                static_cast<float>(p.yaw_deg));
        }
        for (const auto& p : gt_local) {
            impl_->gt_points.push_back(impl_->applyScale(p.x, p.y, p.z));
            impl_->gt_rpy_deg.emplace_back(static_cast<float>(p.roll_deg),
                                           static_cast<float>(p.pitch_deg),
                                           static_cast<float>(p.yaw_deg));
            impl_->gt_index_by_frame.emplace(static_cast<uint64_t>(p.frame_id),
                                             impl_->gt_points.size() - 1);
        }
        for (const auto& p : lost_local) {
            impl_->lost_points.push_back(impl_->applyScale(p.x, p.y, p.z));
        }
        bool fused_new_data  = false;
        bool fused_corrected = false;
        for (const auto& ev : fused_local) {
            if (!ev.is_correction) {
                const Impl::FusedSample& s = ev.sample;
                impl_->fused_points.push_back(
                    impl_->applyScale(s.pos.x(), s.pos.y(), s.pos.z()));
                impl_->fused_quats.push_back(s.q);
                impl_->fused_frame_ids.push_back(s.frame_id);
                fused_new_data = true;
                continue;
            }
            // Correction batch: splice out every stored sample with
            // frame_id >= the first corrected frame_id (frame ids are
            // monotonic, so a back-scan finds the exact cut), then append the
            // corrected (smoothed) chain in order. Live samples pushed after
            // the correction re-append behind it as usual.
            const uint64_t cut = ev.correction.front().frame_id;
            std::size_t keep = impl_->fused_frame_ids.size();
            while (keep > 0 && impl_->fused_frame_ids[keep - 1] >= cut) {
                --keep;
            }
            impl_->fused_points.resize(keep);
            impl_->fused_quats.resize(keep);
            impl_->fused_frame_ids.resize(keep);
            for (const auto& c : ev.correction) {
                impl_->fused_points.push_back(
                    impl_->applyScale(c.pos.x(), c.pos.y(), c.pos.z()));
                impl_->fused_quats.push_back(c.q);
                impl_->fused_frame_ids.push_back(c.frame_id);
            }
            fused_new_data  = true;
            fused_corrected = true;
        }
        // A correction invalidates the incremental fused drawables: tear down
        // the polyline and every uploaded axes gizmo and reset the append
        // cursor so refresh_fused_display re-uploads the FULL spliced buffer
        // (whichever style the current EstOdom mode selects).
        if (fused_corrected && viewer) {
            if (impl_->fused_line_active_) {
                viewer->remove_drawable("fused_trajectory");
                impl_->fused_line_active_ = false;
            }
            for (const std::string& name : impl_->fused_axes_names_) {
                viewer->remove_drawable(name);
            }
            impl_->fused_axes_names_.clear();
            impl_->fused_axes_next_idx_ = 0;
            if (impl_->fused_links_active_) {
                viewer->remove_drawable("fused_gt_links");
                impl_->fused_links_active_ = false;
            }
        }
        // Estimate display: always called so the "VO"/"EstOdom" checkbox
        // toggles are honoured even when no new points arrived this pass
        // (remove on disable / style swap / full re-upload on re-enable).
        refresh_est_display(!local.empty() || !est_local.empty());
        if (!gt_local.empty())                    refresh_gt_line();
        // Fused display: rebuild on new points; otherwise still called so the
        // "Fused"/"EstOdom" checkbox toggles are honoured (remove on disable /
        // style swap / full re-upload from the accumulated buffers on re-enable).
        refresh_fused_display(fused_new_data);
        // Honour the GroundTruth + EstOdom checkbox toggles (remove on disable /
        // style swap / full re-upload from the accumulated buffers on re-enable);
        // in EstOdom mode this also uploads green axes for fresh GT points.
        refresh_gt_display();
        // EstOdom error links: rebuilt whenever either endpoint buffer grew
        // (new groundtruth can complete a pair that had none), otherwise only
        // the checkbox/mode toggle is honoured.
        const bool links_rebuild = !gt_local.empty() || fused_new_data;
        refresh_error_links("fused_gt_links", impl_->fused_points,
                            impl_->fused_frame_ids,
                            impl_->show_est_odom_ && impl_->show_fused_ &&
                                impl_->show_groundtruth_,
                            links_rebuild, impl_->fused_links_active_,
                            FUSED_LINK_COLOR);
        // Rebuild the marker cloud when new LOST points arrived; otherwise still
        // call to honour the checkbox toggle (draw on enable / remove on disable).
        refresh_lost_markers(!lost_local.empty());
        // Refresh the VO map-point cloud (honours the "Landmarks" checkbox and
        // any fresh pushMapPoints() snapshot).
        refresh_map_points();
        // Same for the absolute-fix markers ("Anchor fixes" checkbox +
        // setAnchorFixes() snapshot).
        refresh_anchor_fixes();
    };

    // Drain the UI-panel queues (metrics + video) and push them to guik. Runs
    // ONLY on the render thread; every GL/ImGui-touching call lives here.
    auto drain_aux = [&]() {
        if (!viewer) return;

        // 1) Metrics: swap the queue (under queue_mutex, like the poses) into the
        //    render-thread-only history, trimming to the configured cap.
        std::deque<FrameMetrics> metrics_local;
        {
            std::lock_guard<std::mutex> lock(impl_->queue_mutex);
            metrics_local.swap(impl_->metric_q_);
        }
        if (!metrics_local.empty()) {
            const std::size_t cap = static_cast<std::size_t>(
                std::max(1, impl_->cfg.metric_plot_history));
            for (const auto& m : metrics_local) {
                impl_->frame_x_.push_back(static_cast<double>(m.frame_id));
                impl_->inliers_hist_.push_back(static_cast<double>(m.inliers));
                impl_->landmarks_hist_.push_back(static_cast<double>(m.landmarks));
            }
            auto trim = [cap](std::vector<double>& v) {
                if (v.size() > cap) {
                    v.erase(v.begin(),
                            v.begin() + static_cast<std::ptrdiff_t>(v.size() - cap));
                }
            };
            trim(impl_->frame_x_);
            trim(impl_->inliers_hist_);
            trim(impl_->landmarks_hist_);

            // HUD scalars are latest-wins: take the newest sample drained.
            const FrameMetrics& latest = metrics_local.back();
            impl_->hud_heading_     = latest.heading_deg;
            impl_->hud_heading_tel_ = latest.heading_tel_deg;
            impl_->hud_distance_    = latest.distance_m;
        }

        // 1b) Performance samples: same swap-drain into the render-thread-only
        //     histories feeding the "Performance" streaming line charts.
        std::deque<PerfSample> perf_local;
        {
            std::lock_guard<std::mutex> lock(impl_->queue_mutex);
            perf_local.swap(impl_->perf_q_);
        }
        if (!perf_local.empty()) {
            const std::size_t perf_cap = static_cast<std::size_t>(
                std::max(1, impl_->cfg.perf_plot_history));
            for (const auto& s : perf_local) {
                impl_->perf_t_.push_back(s.t_sec);
                impl_->perf_cpu_.push_back(static_cast<double>(s.cpu_percent));
                impl_->perf_rss_.push_back(static_cast<double>(s.rss_mb));
            }
            auto trim_perf = [perf_cap](std::vector<double>& v) {
                if (v.size() > perf_cap) {
                    v.erase(v.begin(),
                            v.begin() + static_cast<std::ptrdiff_t>(v.size() - perf_cap));
                }
            };
            trim_perf(impl_->perf_t_);
            trim_perf(impl_->perf_cpu_);
            trim_perf(impl_->perf_rss_);
        }

        // 1b2) Error samples: same swap-drain, into the histories feeding the
        //      "Error (m)" line chart + histogram. Capped by the same
        //      metric_plot_history the per-frame metric charts use, so the
        //      running summary is a summary of a BOUNDED window, not of the
        //      whole flight — say so wherever the number is quoted.
        std::deque<ErrorSample> error_local;
        {
            std::lock_guard<std::mutex> lock(impl_->queue_mutex);
            error_local.swap(impl_->error_q_);
        }
        if (!error_local.empty()) {
            const std::size_t err_cap = static_cast<std::size_t>(
                std::max(1, impl_->cfg.metric_plot_history));
            for (const auto& s : error_local) {
                impl_->error_frame_x_.push_back(static_cast<double>(s.frame_id));
                impl_->error_2d_.push_back(static_cast<double>(s.err_2d_m));
                impl_->error_3d_.push_back(static_cast<double>(s.err_3d_m));
            }
            auto trim_err = [err_cap](std::vector<double>& v) {
                if (v.size() > err_cap) {
                    v.erase(v.begin(),
                            v.begin() + static_cast<std::ptrdiff_t>(v.size() - err_cap));
                }
            };
            trim_err(impl_->error_frame_x_);
            trim_err(impl_->error_2d_);
            trim_err(impl_->error_3d_);
        }

        // 1c) Profiling table: latest-wins single slot, copied into the
        //     render-thread-only snapshot the "Profiling" window formats.
        if (impl_->profile_dirty_.exchange(false)) {
            std::lock_guard<std::mutex> lock(impl_->queue_mutex);
            impl_->profile_render_ = impl_->profile_latest_;
        }

        // 1d) HUD throughput figures: latest-wins single slot, same shape.
        if (impl_->throughput_dirty_.exchange(false)) {
            std::lock_guard<std::mutex> lock(impl_->queue_mutex);
            impl_->throughput_render_ = impl_->throughput_latest_;
        }

        // 2) The metric line/histogram windows are drawn directly in the
        //    ui_callback (ImPlot/ImGui) from the ring buffers filled above; no
        //    guik-managed plot resource to set up or tear down here.

        // 3) Video panel: upload the latest frame as a texture while enabled.
        if (impl_->show_video_) {
            if (impl_->frame_dirty_.exchange(false)) {
                cv::Mat local;
                {
                    std::lock_guard<std::mutex> lock(impl_->frame_mutex_);
                    local = impl_->frame_latest_.clone();  // own buffer for the upload
                }
                if (!local.empty()) {
                    viewer->update_image(VIDEO_PANEL_NAME, glk::create_texture(local),
                                         VIDEO_OVERLAY_SCALE);
                    impl_->video_active_ = true;
                }
            }
        } else if (impl_->video_active_) {
            viewer->remove_image(VIDEO_PANEL_NAME);
            impl_->video_active_ = false;
        }

        // 4) Tracking panel: same drop-oldest upload as the video panel but for
        //    the landmark-overlay frames (its own slot + GL resource).
        if (impl_->show_tracking_) {
            if (impl_->tracking_dirty_.exchange(false)) {
                cv::Mat local;
                {
                    std::lock_guard<std::mutex> lock(impl_->tracking_mutex_);
                    local = impl_->tracking_latest_.clone();
                }
                if (!local.empty()) {
                    viewer->update_image(TRACKING_PANEL_NAME,
                                         glk::create_texture(local),
                                         VIDEO_OVERLAY_SCALE);
                    impl_->tracking_active_ = true;
                }
            }
        } else if (impl_->tracking_active_) {
            viewer->remove_image(TRACKING_PANEL_NAME);
            impl_->tracking_active_ = false;
        }
    };

    // --- Mode A: pre-loaded batch and/or live metric pushes (no video). ----
    // Entered whenever no video is configured. Renders any pre-loaded GT batch,
    // then holds the window open draining the push queues (pushPose /
    // pushGroundtruthPose / pushRecord). Headless: returns immediately after the
    // batch (push queues drain harmlessly into nothing).
    if (impl_->video_path.empty()) {
        if (!impl_->preloaded.empty()) {
            spdlog::info("DebugViewer: rendering {} pre-loaded records", impl_->preloaded.size());
            for (const auto& r : impl_->preloaded) {
                if (impl_->stop_flag.load()) break;
                append_gt(r);
            }
        } else {
            spdlog::info("DebugViewer: no video / batch — live push mode (pushPose/pushGroundtruthPose)");
        }
        // Hold the window open (or return immediately when headless).
        while (viewer && !impl_->stop_flag.load()) {
            drain_inferred();
            drain_aux();
            if (!viewer->spin_once()) break;
            std::this_thread::sleep_for(
                std::chrono::milliseconds(VIEWER_SPIN_SLEEP_MS));
        }
        guik::LightViewer::destroy();
        // Both service threads are joined BEFORE run() returns, on this
        // headless-capable branch as much as on the GUI one.
        if (impl_->bridge) {
            impl_->bridge->end(impl_->had_display.load());
        }
        spdlog::info("DebugViewer: push/pre-loaded run finished");
        return;
    }

    // --- Mode B: video + per-frame barcode decode. -------------------------

    cv::VideoCapture cap(impl_->video_path);
    if (!cap.isOpened()) {
        spdlog::error("DebugViewer::run: cannot open video: {}", impl_->video_path);
        guik::LightViewer::destroy();
        if (impl_->bridge) {
            impl_->bridge->end(impl_->had_display.load());
        }
        return;
    }
    spdlog::info("DebugViewer: streaming video {} ({}x{}, {} frames)",
                 impl_->video_path,
                 static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
                 static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
                 static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT)));

    const int stride = std::max(1, impl_->cfg.frame_stride);
    if (stride > 1) {
        spdlog::info("DebugViewer: frame_stride={} (processing 1 of every {} frames)",
                     stride, stride);
    }

    cv::Mat frame, gray;
    int decoded = 0, missed = 0;
    long frame_index = -1;
    while (!impl_->stop_flag.load()) {
        // Advance to the next frame; grab() skips decoding so skipped frames are
        // cheap. Only the kept frames (every `stride`) are retrieved + processed.
        if (!cap.grab()) {
            spdlog::info("DebugViewer: end of stream");
            break;
        }
        ++frame_index;
        if (frame_index % stride != 0) {
            continue;
        }
        if (!cap.retrieve(frame) || frame.empty()) {
            spdlog::info("DebugViewer: end of stream");
            break;
        }

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        int image_id = 0;
        if (impl_->decoder.decode(gray, image_id)) {
            TelemetryRecord rec;
            if (impl_->lookupTelemetry(image_id, rec)) {
                append_gt(rec);
                ++decoded;
            } else {
                // No CSV (or id not present): still have a valid id; skip ENU.
                ++missed;
                spdlog::debug("DebugViewer: imageId {} not in CSV index", image_id);
            }
        } else {
            ++missed;
        }

        drain_inferred();
        drain_aux();

        if (viewer) {
            viewer->update_image("video", glk::create_texture(frame), VIDEO_OVERLAY_SCALE);
            if (!viewer->spin_once()) {
                spdlog::info("DebugViewer: window closed by user");
                break;
            }
        }
    }

    spdlog::info("DebugViewer: finished — decoded {} frames, {} without telemetry", decoded, missed);

    // Keep the window open after the video ends so the user can inspect the
    // final trajectory and close it manually. Headless runs (viewer == null)
    // skip this and exit immediately.
    if (viewer && !impl_->stop_flag.load()) {
        spdlog::info("DebugViewer: holding window open — close it to exit");
        while (!impl_->stop_flag.load()) {
            drain_inferred();
            drain_aux();
            if (!viewer->spin_once()) break;
        }
    }

    guik::LightViewer::destroy();
    if (impl_->bridge) {
        impl_->bridge->end(impl_->had_display.load());
    }
}

} // namespace uavloc::debug_viewer
