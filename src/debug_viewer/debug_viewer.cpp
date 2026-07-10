#include <uavloc/debug_viewer/debug_viewer.h>
#include <uavloc/debug_viewer/debug_viewer_callbacks.h>
#include <uavloc/debug_viewer/inferred_pose.h>
#include <uavloc/debug_viewer/telemetry_csv_reader.h>

#include "barcode_decoder.h"
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
#include <cstddef>
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

// GL resource name for the checkbox-toggled video side panel.
const char* const VIDEO_PANEL_NAME = "VO video";

// GL resource name for the checkbox-toggled tracking-overlay side panel.
const char* const TRACKING_PANEL_NAME = "VO tracking";

// Colours (RGBA) for the two trajectories.
constexpr float GT_COLOR[4]       = {0.2f, 0.9f, 0.4f, 1.0f};  // green
constexpr float INFERRED_COLOR[4] = {0.95f, 0.55f, 0.1f, 1.0f}; // orange

// Colour + screen-space point size for the discrete LOST-state markers.
constexpr float LOST_COLOR[4]           = {0.95f, 0.15f, 0.15f, 1.0f}; // red
constexpr float LOST_MARKER_POINT_SCALE = 8.0f;

// Draw a metric panel inside a resizable ImGui window: a per-frame line chart on
// top and a value-distribution histogram below. Both plots fill the available
// content region (ImVec2(-1, ...)) so they grow/shrink with the window.
//
// Plotting backend: ImPlot is preferred (guik creates the ImPlot context for its
// own plot API, so ImPlot::GetCurrentContext() is valid here). When no context
// is present we fall back to ImGui core (PlotLines + a manually-binned
// PlotHistogram); we never call ImPlot::CreateContext() ourselves, so there is no
// risk of a duplicate-context crash. Render-thread only — all data is read from
// the render-thread-owned ring buffers.
//
// Auto-follow: the line chart's X axis is slid every frame to show only the
// newest `follow_window` samples (Y auto-fits), so the latest values are always
// on screen without the user panning with the mouse. With ImPlot this is a hard
// SetupAxisLimits(..., Always) on X + an AutoFit Y flag; the ImGui-core fallback
// simply plots the trailing `follow_window`-sample slice.
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
    const int    follow = std::max(1, follow_window);

    if (ImPlot::GetCurrentContext() != nullptr) {
        if (ImPlot::BeginPlot("##line", ImVec2(-1, line_h))) {
            // Y auto-fits to the visible data; X is force-followed to the latest
            // window every frame so the newest samples are always in view.
            ImPlot::SetupAxes("frame", value_label, 0, ImPlotAxisFlags_AutoFit);
            const double x_last = frame_x.back();
            double x_lo = x_last - static_cast<double>(follow);
            if (x_lo < frame_x.front()) x_lo = frame_x.front();
            if (x_last - x_lo < 1.0) x_lo = x_last - 1.0;  // guard degenerate range
            ImPlot::SetupAxisLimits(ImAxis_X1, x_lo, x_last, ImPlotCond_Always);
            ImPlot::PlotLine(value_label, frame_x.data(), values.data(), n);
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("##hist", ImVec2(-1, -1))) {
            ImPlot::SetupAxes(value_label, "count", 0, 0);
            ImPlot::PlotHistogram(value_label, values.data(), n, bins);
            ImPlot::EndPlot();
        }
    } else {
        // ImGui core fallback: needs float arrays and a pre-binned histogram.
        const double mn = *std::min_element(values.begin(), values.end());
        const double mx = *std::max_element(values.begin(), values.end());
        // Auto-follow: plot only the trailing `follow`-sample slice so the newest
        // values are always shown (ImGui::PlotLines otherwise draws the whole
        // array squeezed to fit).
        const int    start   = (n > follow) ? (n - follow) : 0;
        const int    slice_n = n - start;
        std::vector<float> vf(values.begin() + start, values.end());
        ImGui::PlotLines("##line", vf.data(), slice_n, 0, value_label,
                         static_cast<float>(mn), static_cast<float>(mx),
                         ImVec2(-1, line_h));

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
    std::vector<std::string>     gt_coord_names;

    // Inferred (external) polyline in ENU, fed via pushRecord()/pushPose().
    std::vector<Eigen::Vector3f> inferred_points;

    // Discrete LOST-state markers in render units, fed via pushLostPose().
    std::vector<Eigen::Vector3f> lost_points;

    // ENU origin (first valid groundtruth record).
    bool   origin_set = false;
    double origin_lat = 0.0, origin_lon = 0.0, origin_alt = 0.0;

    // Thread-safe ingestion queue for external/inferred records.
    std::mutex                 queue_mutex;
    std::deque<TelemetryRecord> inferred_queue;

    // Thread-safe metric pose queues (already in display ENU; no GPS round-trip).
    std::deque<InferredPose>   est_q_;   // estimate -> inferred polyline (orange)
    std::deque<InferredPose>   gt_q_;    // groundtruth -> gt polyline (green)
    std::deque<InferredPose>   lost_q_;  // LOST positions -> red markers

    std::atomic<bool> stop_flag{false};
    std::atomic<bool> had_display{false};

    // ---- UI control panel ---------------------------------------------------
    // Metric samples (producer: worker) drained under queue_mutex like the poses.
    std::deque<FrameMetrics> metric_q_;

    // Render-thread-only metric history (x = frame id) feeding the line chart.
    std::vector<double> frame_x_;
    std::vector<double> inliers_hist_;
    std::vector<double> landmarks_hist_;

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
    bool show_hud_          = false;

    // Latest HUD scalars (render-thread-only; refreshed by drain_aux).
    float hud_heading_     = 0.0f;
    float hud_heading_tel_ = 0.0f;
    float hud_distance_    = 0.0f;

    BarcodeDecoder decoder;

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

void DebugViewer::pushMetrics(const FrameMetrics& m) {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    impl_->metric_q_.push_back(m);
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
            ImGui::End();

            // HUD: VO course-over-ground heading, telemetry heading, path length.
            if (impl_->show_hud_) {
                ImGui::SetNextWindowSize(ImVec2(240, 110), ImGuiCond_FirstUseEver);
                ImGui::Begin("HUD", &impl_->show_hud_);
                ImGui::Text("Heading(VO)  : %.1f deg", impl_->hud_heading_);
                ImGui::Text("Heading(tel) : %.1f deg", impl_->hud_heading_tel_);
                ImGui::Text("Distance     : %.1f m",   impl_->hud_distance_);
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
        });
    }

    // --- Helper lambdas that touch GL (only used when viewer != nullptr). --
    auto refresh_gt_line = [&]() {
        if (viewer && impl_->gt_points.size() >= 2) {
            viewer->update_drawable(
                "gt_trajectory",
                std::make_shared<glk::ThinLines>(impl_->gt_points, /*line_strip=*/true),
                guik::FlatColor(GT_COLOR[0], GT_COLOR[1], GT_COLOR[2], GT_COLOR[3]));
        }
    };
    auto refresh_inferred_line = [&]() {
        if (viewer && impl_->inferred_points.size() >= 2) {
            viewer->update_drawable(
                "inferred_trajectory",
                std::make_shared<glk::ThinLines>(impl_->inferred_points, /*line_strip=*/true),
                guik::FlatColor(INFERRED_COLOR[0], INFERRED_COLOR[1],
                                INFERRED_COLOR[2], INFERRED_COLOR[3]));
        }
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

    auto append_gt = [&](const TelemetryRecord& r) {
        DebugViewerCallbacks::on_telemetry(r);
        const Eigen::Vector3f pt = impl_->toEnu(r);
        impl_->gt_points.push_back(pt);

        // Periodic heading coord frame from yaw.
        const int every = impl_->cfg.coord_frame_every_n;
        if (viewer && every > 0 &&
            (static_cast<int>(impl_->gt_points.size()) % every) == 0) {
            const float y  = static_cast<float>(r.yaw_deg   * DEG2RAD);
            const float p  = static_cast<float>(r.pitch_deg * DEG2RAD);
            const float ro = static_cast<float>(r.roll_deg  * DEG2RAD);
            Eigen::Affine3f T = Eigen::Translation3f(pt)
                * Eigen::AngleAxisf(y,  Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(p,  Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(ro, Eigen::Vector3f::UnitX())
                * Eigen::UniformScaling<float>(impl_->cfg.coord_frame_scale
                                               * impl_->cfg.display_scale);
            const std::string name = "uav_" + std::to_string(r.frame_id);
            viewer->update_drawable(name, glk::Primitives::coordinate_system(),
                                    guik::VertexColor(T));
            impl_->gt_coord_names.push_back(name);
        }

        refresh_gt_line();
        if (viewer && (static_cast<int>(impl_->gt_points.size()) % CAMERA_RECENTER_EVERY_N) == 0) {
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
        {
            std::lock_guard<std::mutex> lock(impl_->queue_mutex);
            local.swap(impl_->inferred_queue);
            est_local.swap(impl_->est_q_);
            gt_local.swap(impl_->gt_q_);
            lost_local.swap(impl_->lost_q_);
        }
        for (const auto& r : local) {
            impl_->inferred_points.push_back(impl_->toEnu(r));
        }
        for (const auto& p : est_local) {
            impl_->inferred_points.push_back(impl_->applyScale(p.x, p.y, p.z));
        }
        for (const auto& p : gt_local) {
            impl_->gt_points.push_back(impl_->applyScale(p.x, p.y, p.z));
        }
        for (const auto& p : lost_local) {
            impl_->lost_points.push_back(impl_->applyScale(p.x, p.y, p.z));
        }
        if (!local.empty() || !est_local.empty()) refresh_inferred_line();
        if (!gt_local.empty())                    refresh_gt_line();
        // Rebuild the marker cloud when new LOST points arrived; otherwise still
        // call to honour the checkbox toggle (draw on enable / remove on disable).
        refresh_lost_markers(!lost_local.empty());
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
        spdlog::info("DebugViewer: push/pre-loaded run finished");
        return;
    }

    // --- Mode B: video + per-frame barcode decode. -------------------------

    cv::VideoCapture cap(impl_->video_path);
    if (!cap.isOpened()) {
        spdlog::error("DebugViewer::run: cannot open video: {}", impl_->video_path);
        guik::LightViewer::destroy();
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
}

} // namespace uavloc::debug_viewer
