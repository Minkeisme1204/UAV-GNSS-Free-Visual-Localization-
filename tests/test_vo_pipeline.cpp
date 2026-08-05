// test_vo_pipeline — run the full VO pipeline end-to-end on the YenBai video and
// render an accumulated trajectory image.
//
// The pipeline is now encapsulated behind a single representative object,
// uavloc::vo::VOModule (stella_vslam / SVO style: feed a frame, get a pose). This
// driver only opens the video, pumps frames through VOModule::processFrame, and
// plots the returned GLOBAL poses. All orchestration (NOT_INITIALIZED ->
// INITIALIZED -> TRACKING -> LOST welding) lives inside VOModule.
//
// Monocular translation is metric when telemetry altitude is present (VOModule
// promotes it from the slant range), otherwise up-to-scale.
//
// Headless-safe: no GUI window; the trajectory is only written via cv::imwrite.
#include "uavloc/sensor/camera_model.h"
#include "uavloc/sensor/geo_reference.h"
#include "uavloc/sensor/video_reader.h"
#include "uavloc/vo/common.h"
#include "uavloc/vo/vo_module.h"

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

// Optional debug-viewer overlay: estimate (orange) vs groundtruth (green) in a
// live 3D viewer. Compiled in only when Iridescence + uavloc_debug_viewer are
// available (CMake defines UAVLOC_WITH_DEBUG_VIEWER and links the library).
#ifdef UAVLOC_WITH_DEBUG_VIEWER
#include <uavloc/debug_viewer/debug_viewer.h>
#include <uavloc/debug_viewer/inferred_pose.h>
#include <uavloc/debug_viewer/viewer_metrics.h>
#include "gps_to_enu.h"     // src/debug_viewer (on this target's include path)
#include <Eigen/Geometry>   // Eigen::umeyama
#include <Eigen/SVD>        // JacobiSVD (mount-convention probe)
#include <thread>
#include <utility>
#endif

namespace uavloc {
namespace {

// Default mission config when none is supplied on the command line.
const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m_newvo.yaml";

// Where the trajectory visualization is written (current working directory).
const std::string OUTPUT_IMAGE_PATH = "vo_trajectory.png";

// Where the geo-referenced latitude/longitude trajectory is written.
const std::string OUTPUT_LATLON_CSV_PATH = "estimated_trajectory_latlon.csv";

// Degrees per full turn — used to wrap heading angles into [0, 360).
const double DEGREES_PER_TURN = 360.0;

// Radians -> degrees for the course-over-ground heading.
const double DEG_PER_RAD = 180.0 / M_PI;

// Wrap an angle in degrees into [0, 360).
double wrap360(double deg) {
    deg = std::fmod(deg, DEGREES_PER_TURN);
    if (deg < 0.0) deg += DEGREES_PER_TURN;
    return deg;
}

// ── Trajectory image presentation constants (rendering only — NOT pipeline
//    thresholds; those are loaded from YAML). ────────────────────────────────
const int    CANVAS_SIZE_PX = 1000;  // square canvas side length
const int    CANVAS_MARGIN_PX = 60;  // border kept clear of the fitted path
const double POLYLINE_THICKNESS = 1.5;
const int    MARKER_RADIUS_PX = 6;

// Tracking-overlay presentation constants (debug-viewer sub-window only — NOT
// pipeline thresholds): circle radius for a tracked landmark keypoint, and the
// annotation text placement/scale.
const int    TRACK_KP_RADIUS_PX  = 9;   // tracked-landmark circle radius (px)
const int    TRACK_KP_THICKNESS  = 2;   // tracked-landmark circle stroke (px)
const int    TRACK_TEXT_X_PX     = 10;
const int    TRACK_TEXT_Y_PX     = 30;
const double TRACK_TEXT_SCALE    = 0.8;
const int    TRACK_TEXT_THICKNESS = 2;

// Holds the reader, camera, VO module and the accumulated trajectory. Built once
// by setup(), then driven by the main loop.
struct PipelineContext {
    YAML::Node yaml;

    sensor::VideoReaderConfig reader_cfg;
    std::unique_ptr<sensor::VideoReader> reader;
    std::unique_ptr<sensor::CameraModel> camera;

    std::unique_ptr<vo::VOModule> vo;

    // World positions of the camera at each processed frame, for plotting (GLOBAL
    // frame, as returned by VOResult::T_wc).
    std::vector<Eigen::Vector3d> positions;

    std::string output_path = OUTPUT_IMAGE_PATH;
};

// Maximum number of consecutive EMPTY_FRAME results tolerated before the reader
// is treated as exhausted (some codecs over-report CAP_PROP_FRAME_COUNT).
const int MAX_CONSECUTIVE_EMPTY_FRAMES = 30;

// Reads the next valid (OK + non-empty) frame from the reader into `out`.
// Returns false at end-of-stream, on unrecoverable error, or when too many
// consecutive empty frames indicate the stream is effectively exhausted.
bool readNextValidFrame(sensor::VideoReader& reader, sensor::FrameData& out) {
    int consecutive_empty = 0;
    while (true) {
        auto status = reader.read(out);

        if (status == sensor::FrameStatus::END_OF_STREAM) {
            return false;
        }
        if (status == sensor::FrameStatus::OK && out.valid && out.HasImage()) {
            return true;
        }
        if (status == sensor::FrameStatus::EMPTY_FRAME) {
            if (++consecutive_empty >= MAX_CONSECUTIVE_EMPTY_FRAMES) {
                spdlog::info("Reader exhausted: {} consecutive empty frames — "
                             "treating as end of stream",
                             consecutive_empty);
                return false;
            }
        }
        spdlog::warn("Skipping frame — status={}", static_cast<int>(status));
        if (status == sensor::FrameStatus::ERROR ||
            status == sensor::FrameStatus::CAMERA_DISCONNECTED) {
            return false;
        }
    }
}

// Build the reader, camera model and VO module from the mission config. Returns
// false (and logs) on any failure so main() can bail out cleanly.
bool setup(PipelineContext& ctx, const std::string& config_path) {
    // ── Load config ───────────────────────────────────────────────────────────
    try {
        ctx.yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return false;
    }
    spdlog::info("Config loaded from '{}'", config_path);

    // ── Parse configs ─────────────────────────────────────────────────────────
    sensor::CameraIntrinsics intrinsics;
    vo::VOConfig vo_cfg;
    try {
        // VideoReaderConfig reads the ROOT node (it owns the VideoReader: block).
        ctx.reader_cfg = sensor::VideoReaderConfig::fromYaml(ctx.yaml);
        // CameraIntrinsics::fromYaml indexes node["Camera"] itself -> pass root.
        intrinsics     = sensor::CameraIntrinsics::fromYaml(ctx.yaml);
        // VOConfig aggregates every VO stage config from the "VO:" subtree.
        vo_cfg         = vo::VOConfig::fromYaml(ctx.yaml["VO"]);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return false;
    }

    // ── Open video ────────────────────────────────────────────────────────────
    ctx.reader = std::make_unique<sensor::VideoReader>(ctx.reader_cfg);
    if (!ctx.reader->open()) {
        spdlog::error("Failed to open video: '{}'", ctx.reader_cfg.video_path);
        return false;
    }
    spdlog::info("Video opened — fps: {:.1f}, total frames: {}",
                 ctx.reader->getFps(), ctx.reader->getFrameCount());

    // ── Camera model ──────────────────────────────────────────────────────────
    ctx.camera = std::make_unique<sensor::CameraModel>(intrinsics);
    if (!ctx.camera->isValid()) {
        spdlog::error("Invalid camera intrinsics (fx/fy/width/height must be > 0)");
        return false;
    }

    // ── VO module (owns the whole pipeline) ────────────────────────────────────
    ctx.vo = std::make_unique<vo::VOModule>(vo_cfg, *ctx.camera);

    // Anchor the first position at the world origin.
    ctx.positions.push_back(Eigen::Vector3d::Zero());
    return true;
}

// Project the accumulated world positions onto the ground plane (top-down: X
// horizontal, Z vertical) and write a fitted polyline + start/end markers.
bool renderTrajectory(const std::vector<Eigen::Vector3d>& positions,
                      const std::string& output_path) {
    if (positions.size() < 2) {
        spdlog::warn("Trajectory has < 2 points; nothing to render");
        return false;
    }

    // Auto-fit: bounds over the plotted (X, Z) coordinates.
    double min_x = positions.front().x(), max_x = min_x;
    double min_z = positions.front().z(), max_z = min_z;
    for (const Eigen::Vector3d& p : positions) {
        min_x = std::min(min_x, p.x());
        max_x = std::max(max_x, p.x());
        min_z = std::min(min_z, p.z());
        max_z = std::max(max_z, p.z());
    }

    const double span_x = std::max(max_x - min_x, 1e-9);
    const double span_z = std::max(max_z - min_z, 1e-9);
    const double draw_area = static_cast<double>(CANVAS_SIZE_PX - 2 * CANVAS_MARGIN_PX);
    // Uniform scale preserves the trajectory shape (aspect ratio).
    const double scale = draw_area / std::max(span_x, span_z);

    // Map a world (X, Z) to a canvas pixel; flip Y so +Z points up on screen.
    auto toPixel = [&](const Eigen::Vector3d& p) {
        const double px = CANVAS_MARGIN_PX + (p.x() - min_x) * scale;
        const double py = CANVAS_SIZE_PX - CANVAS_MARGIN_PX - (p.z() - min_z) * scale;
        return cv::Point(static_cast<int>(px), static_cast<int>(py));
    };

    cv::Mat canvas(CANVAS_SIZE_PX, CANVAS_SIZE_PX, CV_8UC3,
                   cv::Scalar(30, 30, 30));

    for (std::size_t i = 1; i < positions.size(); ++i) {
        cv::line(canvas, toPixel(positions[i - 1]), toPixel(positions[i]),
                 cv::Scalar(0, 255, 0), static_cast<int>(POLYLINE_THICKNESS),
                 cv::LINE_AA);
    }
    // Start (blue) and end (red) markers.
    cv::circle(canvas, toPixel(positions.front()), MARKER_RADIUS_PX,
               cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
    cv::circle(canvas, toPixel(positions.back()), MARKER_RADIUS_PX,
               cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);

    if (!cv::imwrite(output_path, canvas)) {
        spdlog::error("Failed to write trajectory image to '{}'", output_path);
        return false;
    }
    spdlog::info("Trajectory image written to '{}'", output_path);
    return true;
}

}  // namespace
}  // namespace uavloc

int main(int argc, char** argv) {
    using namespace uavloc;

    // Honour SPDLOG_LEVEL so a debug run can surface the per-frame POS lines.
    spdlog::cfg::load_env_levels();

    const std::string config_path =
        (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    PipelineContext ctx;
    if (!setup(ctx, config_path)) {
        return 1;
    }

    // ── Per-frame loop bookkeeping ─────────────────────────────────────────────
    std::size_t frames_processed   = 0;
    std::size_t pose_successes      = 0;
    std::size_t metric_frames       = 0;  // TRACKING frames that supplied a metric scale
    std::size_t num_keyframes       = 0;  // keyframes promoted (incl. KF1 at init)
    std::size_t lost_frames         = 0;  // frames where tracking was LOST
    double      trajectory_length_m = 0.0;  // sum of per-step translation magnitudes

    // Global keyframe positions, to report the mean inter-keyframe baseline.
    std::vector<Eigen::Vector3d> keyframe_positions;

    // ── Geo-referencing (Stage 0) ──────────────────────────────────────────────
    // Anchored at the first has_pose frame that carries valid telemetry. Converts
    // the VO world position into lat/lon; groundtruth telemetry is used ONLY to
    // anchor the origin + measure error (never fed back per-frame).
    sensor::GeoReferencer geo;
    Eigen::Vector3d       geo_ref_pos0 = Eigen::Vector3d::Zero();  // VO pos at anchor
    double                geo_anchor_psi_deg = 0.0;                // azimuth at anchor
    std::ofstream         latlon_csv(OUTPUT_LATLON_CSV_PATH);
    if (latlon_csv.is_open()) {
        latlon_csv << "frame_id,lat,lon,alt\n";
    } else {
        spdlog::warn("Could not open '{}' for writing", OUTPUT_LATLON_CSV_PATH);
    }
    // Horizontal geo error (metres) between the estimated lat/lon and groundtruth
    // telemetry, collected per has_pose frame with valid telemetry. Oracle metric.
    std::vector<double> geo_err_h;
    // Latest VO course-over-ground heading (deg, 0=North CW), held across frames
    // whose baseline is below hud_min_baseline_m so it does not jitter on hover.
    double last_cog_heading_deg = 0.0;
    // Minimum per-frame baseline (metres) before the COG heading is refreshed.
    const double hud_min_baseline_m =
        ctx.yaml["DebugViewer"]["hud_min_baseline_m"].as<double>(0.5);

    // Shared shutdown flag between the VO worker and the (optional) viewer on the
    // main thread. Only ever flips to true in the debug-viewer build.
    std::atomic<bool> stop_requested{false};

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    namespace dv = uavloc::debug_viewer;

    // Alignment window: number of initial TRACKING frames over which the (frozen)
    // VO-world -> groundtruth-ENU rigid transform is computed (Umeyama, scale
    // fixed = 1 since VO is metric). Loaded from YAML so it is not a magic number.
    const int alignment_window = std::max(
        1, ctx.yaml["DebugViewer"]["alignment_window_frames"].as<int>(50));

    // Viewer config: forward optional display/vertical scale (defaults preserved).
    dv::DebugViewer::Config viewer_cfg;
    viewer_cfg.display_scale =
        ctx.yaml["DebugViewer"]["display_scale"].as<float>(viewer_cfg.display_scale);
    viewer_cfg.vertical_scale =
        ctx.yaml["DebugViewer"]["vertical_scale"].as<float>(viewer_cfg.vertical_scale);
    viewer_cfg.video_stride =
        ctx.yaml["DebugViewer"]["video_stride"].as<int>(viewer_cfg.video_stride);
    viewer_cfg.log_capacity =
        ctx.yaml["DebugViewer"]["log_capacity"].as<int>(viewer_cfg.log_capacity);
    viewer_cfg.metric_plot_history =
        ctx.yaml["DebugViewer"]["metric_plot_history"].as<int>(viewer_cfg.metric_plot_history);
    viewer_cfg.metric_plot_follow_window =
        ctx.yaml["DebugViewer"]["metric_plot_follow_window"].as<int>(viewer_cfg.metric_plot_follow_window);
    viewer_cfg.histogram_bins =
        ctx.yaml["DebugViewer"]["histogram_bins"].as<int>(viewer_cfg.histogram_bins);
    viewer_cfg.show_log =
        ctx.yaml["DebugViewer"]["show_log"].as<bool>(viewer_cfg.show_log);
    viewer_cfg.show_inliers =
        ctx.yaml["DebugViewer"]["show_inliers"].as<bool>(viewer_cfg.show_inliers);
    viewer_cfg.show_landmarks =
        ctx.yaml["DebugViewer"]["show_landmarks"].as<bool>(viewer_cfg.show_landmarks);
    viewer_cfg.show_video =
        ctx.yaml["DebugViewer"]["show_video"].as<bool>(viewer_cfg.show_video);
    viewer_cfg.show_tracking =
        ctx.yaml["DebugViewer"]["show_tracking"].as<bool>(viewer_cfg.show_tracking);
    viewer_cfg.show_lost_poses =
        ctx.yaml["DebugViewer"]["show_lost_poses"].as<bool>(viewer_cfg.show_lost_poses);
    viewer_cfg.show_hud =
        ctx.yaml["DebugViewer"]["show_hud"].as<bool>(viewer_cfg.show_hud);
    const int video_stride = std::max(1, viewer_cfg.video_stride);
    dv::DebugViewer viewer(viewer_cfg);
    viewer.loadBatch({});  // empty -> live push mode (holds window, drains queues)

    // Capture all spdlog output into the viewer's scrolling log-terminal panel.
    spdlog::default_logger()->sinks().push_back(viewer.logSink());

    // Alignment accumulation state (worker-thread-local logically, captured by ref).
    std::vector<Eigen::Vector3d> align_src;   // p_cam in the VO world frame
    std::vector<Eigen::Vector3d> align_dst;   // matching groundtruth ENU
    std::vector<std::pair<int, Eigen::Vector3d>> est_buffer;  // pending estimates
    std::vector<std::pair<int, Eigen::Vector3d>> lost_buffer; // pending LOST markers
    Eigen::Matrix4d T_align     = Eigen::Matrix4d::Identity();
    bool   aligned              = false;
    bool   gt_origin_set        = false;
    double gt_o_lat = 0.0, gt_o_lon = 0.0, gt_o_alt = 0.0;
#endif

    // VO pipeline body. Runs on a worker thread in the debug-viewer build (the
    // viewer owns the main thread for GL); runs inline otherwise.
    auto run_pipeline = [&]() {
#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // Transform a VO-world position through the frozen alignment and push it as a
    // discrete LOST-state marker. Defined at run_pipeline scope so both the LOST
    // branch and the alignment-freeze flush can reach it (push_estimate lives
    // inside the TRACKING block and is not visible from the LOST branch).
    auto push_lost = [&](int fid, const Eigen::Vector3d& p_world) {
        const Eigen::Vector3d est =
            (T_align * p_world.homogeneous()).hnormalized();
        dv::InferredPose lp;
        lp.frame_id = fid;
        lp.x = est.x(); lp.y = est.y(); lp.z = est.z();
        viewer.pushLostPose(lp);
    };
#endif
    sensor::FrameData frame;
    while (!stop_requested.load() && readNextValidFrame(*ctx.reader, frame)) {
        ++frames_processed;

#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // Stream every `video_stride` frame into the viewer's video panel.
        if (frames_processed % static_cast<std::size_t>(video_stride) == 0) {
            viewer.pushFrame(frame.image);
        }
#endif

        // ── Feed the frame to the VO module (synchronous: frame -> pose) ───────
        const vo::VOResult r = ctx.vo->processFrame(frame);

        // Metric-scale availability (summary stat) only counts TRACKING-mode frames.
        if ((r.state == vo::VOTrackingState::TRACKING ||
             r.state == vo::VOTrackingState::LOST) &&
            frame.has_telemetry && frame.telemetry.altitude_m > 0.0) {
            ++metric_frames;
        }

        if (r.state == vo::VOTrackingState::LOST) {
            ++lost_frames;
#ifdef UAVLOC_WITH_DEBUG_VIEWER
            // On LOST, VOResult carries no valid pose; mark the last valid
            // (TRACKING) position — the point where tracking broke — so the
            // marker sits exactly on the estimate polyline. Push through the same
            // T_align once aligned, otherwise buffer until the transform freezes.
            if (!ctx.positions.empty()) {
                const Eigen::Vector3d p_lost = ctx.positions.back();
                if (aligned) {
                    push_lost(r.frame_id, p_lost);
                } else {
                    lost_buffer.emplace_back(r.frame_id, p_lost);
                }
            }
#endif
        }

        if (!r.has_pose) {
            continue;  // NOT_INITIALIZED / LOST — nothing to plot this frame
        }

        // GLOBAL world position for plotting (continuous across LOST/re-init).
        const Eigen::Vector3d p = r.T_wc.block<3, 1>(0, 3);
        // Path length: sum of consecutive GLOBAL position deltas (matching how
        // net_displacement and mean_kf_baseline measure distance with .norm() of a
        // position difference). T_prev_curr is a world-frame SE3 increment, so its
        // own translation is NOT the inter-frame displacement — accumulating it
        // overcounts. The increment is identity on the first step of a
        // (re-)initialised map; at those boundaries the welded global position can
        // jump, so they are skipped (the boundary step contributes nothing, as the
        // previous implementation intended).
        const bool reinit_boundary =
            r.T_prev_curr.isApprox(Eigen::Matrix4d::Identity(), 1e-9);
        const Eigen::Vector3d prev_pos =
            ctx.positions.empty() ? p : ctx.positions.back();
        if (!ctx.positions.empty() && !reinit_boundary) {
            trajectory_length_m += (p - prev_pos).norm();
        }
        ctx.positions.push_back(p);
        ++pose_successes;

        // ── Geo-referencing (Stage 0) ─────────────────────────────────────────
        // Anchor the VO world at the first geo-referenceable pose using telemetry
        // lat/lon/alt and the geographic view azimuth psi = heading + gimbal pan.
        if (!geo.initialized() && frame.has_telemetry &&
            frame.telemetry.altitude_m > 0.0) {
            const double psi_deg =
                frame.telemetry.heading_deg + frame.telemetry.gimbal_pan_deg;
            geo.init(frame.telemetry.latitude_deg, frame.telemetry.longitude_deg,
                     frame.telemetry.altitude_m, psi_deg);
            geo_ref_pos0       = p;  // VO position that coincides with the geo origin
            geo_anchor_psi_deg = psi_deg;
            spdlog::info("Geo-reference anchored at frame {}: lat {:.6f}, lon {:.6f}, "
                         "alt {:.1f} m, psi {:.1f} deg",
                         frame.frame_id, frame.telemetry.latitude_deg,
                         frame.telemetry.longitude_deg, frame.telemetry.altitude_m,
                         psi_deg);
        }

        // Update the VO course-over-ground heading (ENU, 0 = North CW). Held when
        // the per-frame baseline is too small to trust the direction.
        if (geo.initialized()) {
            const Eigen::Vector3d dp     = p - prev_pos;
            const Eigen::Vector3d dp_enu = geo.R_enu_w() * dp;
            if (!reinit_boundary && dp.norm() >= hud_min_baseline_m) {
                last_cog_heading_deg =
                    wrap360(std::atan2(dp_enu.x(), dp_enu.y()) * DEG_PER_RAD);
            }

            // Geo-referenced lat/lon of this pose (relative to the geo anchor).
            const sensor::LatLonAlt ll = geo.latlon(p - geo_ref_pos0);
            if (latlon_csv.is_open()) {
                latlon_csv << frame.frame_id << ',' << std::fixed
                           << std::setprecision(7) << ll.lat << ',' << ll.lon
                           << ',' << std::setprecision(2) << ll.alt << '\n';
            }

            // Horizontal error vs groundtruth telemetry (oracle metric only).
            if (frame.has_telemetry && frame.telemetry.altitude_m > 0.0) {
                const Eigen::Vector3d est_enu = geo.R_enu_w() * (p - geo_ref_pos0);
                const Eigen::Vector3d gt_enu  = geo.enu(
                    frame.telemetry.latitude_deg, frame.telemetry.longitude_deg,
                    frame.telemetry.altitude_m);
                geo_err_h.push_back((est_enu - gt_enu).head<2>().norm());
            }
        }

        if (r.is_keyframe) {
            ++num_keyframes;
            keyframe_positions.push_back(p);
        }

#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // Inlier count: seeded landmarks at (re-)init, tracked landmarks otherwise.
        // HUD: VO course-over-ground heading, telemetry heading (heading + gimbal
        // pan), and the accumulated metric path length.
        const float hud_heading_tel = static_cast<float>(wrap360(
            frame.telemetry.heading_deg + frame.telemetry.gimbal_pan_deg));
        dv::FrameMetrics fm;
        fm.frame_id        = r.frame_id;
        fm.inliers         = static_cast<float>(r.num_inliers);
        fm.landmarks       = static_cast<float>(r.num_landmarks);
        fm.heading_deg     = static_cast<float>(last_cog_heading_deg);
        fm.heading_tel_deg = frame.has_telemetry ? hud_heading_tel : 0.0f;
        fm.distance_m      = static_cast<float>(trajectory_length_m);
        viewer.pushMetrics(fm);

        // Tracking sub-window: overlay the tracked landmark keypoints on the
        // frame and annotate the landmark counts, then push on the dedicated
        // channel (gated by the same video_stride as the plain video panel).
        if (frames_processed % static_cast<std::size_t>(video_stride) == 0) {
            cv::Mat overlay = frame.image.clone();
            for (const Eigen::Vector2d& obs : r.tracked_observations) {
                cv::circle(overlay,
                           cv::Point(static_cast<int>(std::lround(obs.x())),
                                     static_cast<int>(std::lround(obs.y()))),
                           TRACK_KP_RADIUS_PX, cv::Scalar(255, 0, 0),  // blue (BGR)
                           TRACK_KP_THICKNESS, cv::LINE_AA);
            }
            const std::string label =
                "Landmarks: " + std::to_string(r.num_landmarks) +
                "  tracked: " + std::to_string(r.tracked_observations.size());
            cv::putText(overlay, label,
                        cv::Point(TRACK_TEXT_X_PX, TRACK_TEXT_Y_PX),
                        cv::FONT_HERSHEY_SIMPLEX, TRACK_TEXT_SCALE,
                        cv::Scalar(0, 255, 255), TRACK_TEXT_THICKNESS,
                        cv::LINE_AA);
            viewer.pushTrackingFrame(overlay);
        }
#endif

        spdlog::debug("POS frame {} pos {:.4f} {:.4f} {:.4f}",
                      frame.frame_id, p.x(), p.y(), p.z());

#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // ── Overlay: estimate vs groundtruth (TRACKING frames only) ───────────
        // Initialisation frames are excluded from the alignment correspondences
        // (matching the original driver, which aligned over TRACKING frames).
        if (r.state == vo::VOTrackingState::TRACKING &&
            frame.has_telemetry && frame.telemetry.altitude_m > 0.0) {
            if (!gt_origin_set) {
                gt_o_lat     = frame.telemetry.latitude_deg;
                gt_o_lon     = frame.telemetry.longitude_deg;
                gt_o_alt     = frame.telemetry.altitude_m;
                gt_origin_set = true;
            }
            const dv::ENUPoint e = dv::gps_to_enu(
                frame.telemetry.latitude_deg, frame.telemetry.longitude_deg,
                frame.telemetry.altitude_m, gt_o_lat, gt_o_lon, gt_o_alt);
            const Eigen::Vector3d gt_enu(e.e, e.n, e.u);
            const Eigen::Vector3d p_cam = p;

            // Groundtruth is independent of alignment — push it every frame.
            dv::InferredPose gp;
            gp.frame_id = r.frame_id;
            gp.x = gt_enu.x(); gp.y = gt_enu.y(); gp.z = gt_enu.z();
            viewer.pushGroundtruthPose(gp);

            auto push_estimate = [&](int fid, const Eigen::Vector3d& p_world) {
                const Eigen::Vector3d est =
                    (T_align * p_world.homogeneous()).hnormalized();
                dv::InferredPose ep;
                ep.frame_id = fid;
                ep.x = est.x(); ep.y = est.y(); ep.z = est.z();
                viewer.pushPose(ep);
            };

            if (!aligned) {
                // Accumulate correspondences; buffer estimates until the frozen
                // transform is known, then flush them aligned.
                align_src.push_back(p_cam);
                align_dst.push_back(gt_enu);
                est_buffer.emplace_back(r.frame_id, p_cam);

                if (static_cast<int>(align_src.size()) >= alignment_window) {
                    Eigen::Matrix3Xd S(3, align_src.size());
                    Eigen::Matrix3Xd D(3, align_dst.size());
                    for (std::size_t i = 0; i < align_src.size(); ++i) {
                        S.col(static_cast<Eigen::Index>(i)) = align_src[i];
                        D.col(static_cast<Eigen::Index>(i)) = align_dst[i];
                    }
                    // scale fixed = 1 (VO already metric); exposes scale drift.
                    T_align = Eigen::umeyama(S, D, /*with_scaling=*/false);

                    double sse = 0.0;
                    for (std::size_t i = 0; i < align_src.size(); ++i) {
                        const Eigen::Vector3d a =
                            (T_align * align_src[i].homogeneous()).hnormalized();
                        sse += (a - align_dst[i]).squaredNorm();
                    }
                    const double rms =
                        std::sqrt(sse / static_cast<double>(align_src.size()));
                    aligned = true;
                    spdlog::info("alignment computed at frame {}, RMS={:.3f} m",
                                 frame.frame_id, rms);

                    // Convention diagnostics (oracle, measurement-only; never fed
                    // back). (a) Full-3D deviation between the telemetry R_enu_w and
                    // the Umeyama VO->ENU rotation — inflated by the ill-conditioned
                    // vertical DOF over near-planar flight. (b) A 2D Procrustes fit
                    // of the horizontal VO->ENU map from per-frame displacements,
                    // whose determinant reveals the handedness convention and whose
                    // azimuth residual vs R_enu_w exposes any mount offset.
                    if (geo.initialized()) {
                        const Eigen::Matrix3d Ra = geo.R_enu_w();
                        const Eigen::Matrix3d Rb = T_align.block<3, 3>(0, 0);
                        const double ct =
                            std::clamp((Ra.transpose() * Rb).trace() * 0.5 - 0.5,
                                       -1.0, 1.0);
                        spdlog::info("R_enu_w vs T_align rotation deviation: {:.2f} deg "
                                     "(3D; vertical DOF ill-conditioned)",
                                     std::acos(ct) * DEG_PER_RAD);

                        Eigen::Matrix2d M = Eigen::Matrix2d::Zero();
                        for (std::size_t i = 1; i < align_src.size(); ++i) {
                            const Eigen::Vector2d v =
                                (align_src[i] - align_src[i - 1]).head<2>();
                            const Eigen::Vector2d g =
                                (align_dst[i] - align_dst[i - 1]).head<2>();
                            M += g * v.transpose();
                        }
                        Eigen::JacobiSVD<Eigen::Matrix2d> svd(
                            M, Eigen::ComputeFullU | Eigen::ComputeFullV);
                        const Eigen::Matrix2d Q  = svd.matrixU() * svd.matrixV().transpose();
                        const Eigen::Matrix2d Rh = Ra.block<2, 2>(0, 0);
                        // Both are reflections (det -1): the handedness convention is
                        // correct. The azimuth residual is a mount-calibration offset.
                        const double q_ang = std::atan2(Q(1, 0), Q(0, 0)) * DEG_PER_RAD;
                        const double r_ang = std::atan2(Rh(1, 0), Rh(0, 0)) * DEG_PER_RAD;
                        spdlog::info("Horizontal VO->ENU: Procrustes det {:+.2f}, "
                                     "R_enu_w det {:+.2f}; azimuth residual {:.2f} deg "
                                     "(mount-calibration, entangled with in-window heading)",
                                     Q.determinant(), Rh.determinant(),
                                     wrap360(q_ang - r_ang));
                    }

                    for (const auto& fp : est_buffer) {
                        push_estimate(fp.first, fp.second);
                    }
                    est_buffer.clear();
                    // Flush any LOST markers gathered before alignment froze.
                    for (const auto& fp : lost_buffer) {
                        push_lost(fp.first, fp.second);
                    }
                    lost_buffer.clear();
                }
            } else {
                push_estimate(r.frame_id, p_cam);
            }
        }
#endif
    }
    ctx.reader->close();
#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // Signal the viewer (main thread) that the stream is exhausted.
    stop_requested.store(true);
    viewer.stop();
#endif
    };  // run_pipeline

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // Viewer owns the main thread (GL singleton); VO runs on a worker. When the
    // user closes the window run() returns early — flag the worker and join.
    std::thread worker(run_pipeline);
    viewer.run();              // blocking on a display; returns immediately headless
    if (viewer.hadDisplay()) {
        stop_requested.store(true);
    }
    worker.join();
#else
    run_pipeline();
#endif

    // ── Render + summary ──────────────────────────────────────────────────────
    const bool wrote = renderTrajectory(ctx.positions, ctx.output_path);

    // Net displacement in the GLOBAL frame: last plotted position relative to the
    // first (the world origin).
    const double final_translation =
        ctx.positions.empty()
            ? 0.0
            : (ctx.positions.back() - ctx.positions.front()).norm();
    const bool   metric = (metric_frames > 0);
    const std::size_t final_landmarks = ctx.vo->numLandmarks();
    spdlog::info("Summary — frames processed: {}, successful poses: {}, "
                 "tracking-LOST frames: {}, metric frames: {}",
                 frames_processed, pose_successes, lost_frames, metric_frames);
    // Mean inter-keyframe baseline = average global distance between consecutive
    // keyframes.
    double mean_kf_baseline = 0.0;
    if (keyframe_positions.size() >= 2) {
        double sum = 0.0;
        for (std::size_t i = 1; i < keyframe_positions.size(); ++i) {
            sum += (keyframe_positions[i] - keyframe_positions[i - 1]).norm();
        }
        mean_kf_baseline = sum / static_cast<double>(keyframe_positions.size() - 1);
    }
    spdlog::info("Local map — keyframes inserted: {}, landmarks (final): {}, "
                 "mean inter-KF baseline: {:.3f} {}",
                 num_keyframes, final_landmarks, mean_kf_baseline,
                 metric ? "m" : "(up-to-scale)");
    spdlog::info("Trajectory — path length: {:.3f} {} (sum of per-frame steps), "
                 "net displacement: {:.3f} {}, output: '{}'",
                 trajectory_length_m, metric ? "m" : "(up-to-scale)",
                 final_translation, metric ? "m" : "(up-to-scale)",
                 ctx.output_path);

    // ── Geo-referencing summary (oracle metrics; not fed back into the estimate) ─
    if (latlon_csv.is_open()) {
        latlon_csv.close();
        spdlog::info("Geo-referenced trajectory written to '{}' ({} rows)",
                     OUTPUT_LATLON_CSV_PATH, geo_err_h.size());
    }
    if (!geo_err_h.empty()) {
        std::vector<double> sorted = geo_err_h;
        std::sort(sorted.begin(), sorted.end());
        const double mean =
            std::accumulate(sorted.begin(), sorted.end(), 0.0) /
            static_cast<double>(sorted.size());
        const double median = sorted[sorted.size() / 2];
        const double max_err = sorted.back();
        spdlog::info("Geo error vs groundtruth (horizontal) — mean: {:.2f} m, "
                     "median: {:.2f} m, max: {:.2f} m ({} frames)",
                     mean, median, max_err, sorted.size());
    } else {
        spdlog::warn("Geo error: no geo-referenced frames with telemetry");
    }

    return wrote ? 0 : 1;
}
