// test_vo_viewer — LIVE 3D viewer driver for the new_vo VOModule.
//
// Mirrors the debug-viewer mechanism of the (deprecated) tests/test_vo_pipeline.cpp,
// ported onto the new_vo public API (uavloc::vo::VOModule::process_frame + the
// VOData publish callback):
//
//   * The VO pipeline runs on a WORKER thread; DebugViewer::run() owns the MAIN
//     thread (GL singleton, blocking live window). A shared std::atomic<bool>
//     stop_requested coordinates shutdown: the worker sets it when the video ends
//     and calls viewer.stop(); when the user closes the window run() returns and
//     the main thread flags the worker, then joins.
//   * viewer.loadBatch({}) -> live push mode. Per processed frame the worker pushes
//     the groundtruth pose (telemetry lat/lon/alt -> ENU via gps_to_enu.h) and,
//     once a 4-DoF gravity-aligned (yaw + translation, roll/pitch locked by the
//     landmark-plane normal) VO-world -> groundtruth-ENU transform is frozen over
//     an initial alignment window of TRACKING frames, the aligned VO estimate
//     (pushPose) and the latest keyframe map-point cloud (pushMapPoints). A free
//     3-DoF-rotation Umeyama is degenerate here: the near-collinear camera window
//     leaves the roll about the flight axis unconstrained, tilting the whole map.
//   * Headless-safe: viewer.run() returns immediately with no DISPLAY. Everything
//     is behind #ifdef UAVLOC_WITH_DEBUG_VIEWER; without it the pipeline just runs
//     on the main thread to completion (or the UAVLOC_VO_MAXFRAMES cap) and exits 0.
//   * Soft-skips (returns 0) when the gitignored dataset video is unavailable.
//   * Side-panel parity with the old test_vo_pipeline: per-frame FrameMetrics
//     (inliers/landmarks plots + HUD heading/distance), a "Tracking" sub-window
//     overlaying VOResult::tracked_observations on the frame, LOST-pose markers,
//     and a "Performance" panel fed by a /proc-based CPU%/RSS sampler thread.
//
// Usage (from build/):
//     ./tests/test_vo_viewer [config.yaml]
//     UAVLOC_VO_MAXFRAMES=200 ./tests/test_vo_viewer [config.yaml]   # capped run
// Run with a DISPLAY to open the live window (trajectory + groundtruth + cloud).
#include "uavloc/sensor/video_reader.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/new_vo/vo_module.h"

#include <Eigen/Core>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <string>
#include <vector>

#ifdef UAVLOC_WITH_DEBUG_VIEWER
#include "uavloc/debug_viewer/debug_viewer.h"
#include "uavloc/debug_viewer/inferred_pose.h"
#include "uavloc/debug_viewer/viewer_metrics.h"
#include "uavloc/util/sys_monitor.h"
#include "gps_to_enu.h"    // src/debug_viewer (on this target's include path)
#include <Eigen/Geometry>  // Eigen::umeyama, Quaterniond, AngleAxisd
#include <Eigen/SVD>       // JacobiSVD (landmark-plane fit)
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <utility>
#endif

namespace uavloc {
namespace {

const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m.yaml";

// Maximum consecutive EMPTY_FRAME results tolerated before the reader is treated
// as exhausted (some codecs over-report CAP_PROP_FRAME_COUNT).
constexpr int MAX_CONSECUTIVE_EMPTY_FRAMES = 30;

#ifdef UAVLOC_WITH_DEBUG_VIEWER
// ── HUD heading formula (mirrors the old test_vo_pipeline) ───────────────────
// Degrees per full turn — used to wrap heading angles into [0, 360).
constexpr double DEGREES_PER_TURN = 360.0;
// Radians -> degrees for the course-over-ground heading.
constexpr double DEG_PER_RAD = 180.0 / M_PI;

// Wrap an angle in degrees into [0, 360).
double wrap360(double deg) {
    deg = std::fmod(deg, DEGREES_PER_TURN);
    if (deg < 0.0) deg += DEGREES_PER_TURN;
    return deg;
}

// Tracking-overlay presentation constants (debug-viewer sub-window only — NOT
// pipeline thresholds): circle radius for a tracked landmark keypoint, and the
// annotation text placement/scale.
constexpr int    TRACK_KP_RADIUS_PX   = 4;   // tracked-landmark circle radius (px)
constexpr int    TRACK_KP_THICKNESS   = 2;   // tracked-landmark circle stroke (px)
constexpr int    TRACK_TEXT_X_PX      = 10;
constexpr int    TRACK_TEXT_Y_PX      = 30;
constexpr double TRACK_TEXT_SCALE     = 0.8;
constexpr int    TRACK_TEXT_THICKNESS = 2;

// ── 4-DoF alignment helpers ──────────────────────────────────────────────────
// Least-squares plane fit through `pts`: centroid + unit normal = the singular
// vector of the smallest singular value of the centered 3xN matrix. Returns
// false when fewer than 3 points are available (plane underdetermined).
bool fitPlane(const std::vector<Eigen::Vector3d>& pts,
              Eigen::Vector3d& centroid, Eigen::Vector3d& normal) {
    if (pts.size() < 3) {
        return false;
    }
    centroid = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& p : pts) {
        centroid += p;
    }
    centroid /= static_cast<double>(pts.size());
    Eigen::Matrix3Xd M(3, static_cast<Eigen::Index>(pts.size()));
    for (std::size_t i = 0; i < pts.size(); ++i) {
        M.col(static_cast<Eigen::Index>(i)) = pts[i] - centroid;
    }
    Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(M, Eigen::ComputeFullU);
    normal = svd.matrixU().col(2).normalized();
    return true;
}

// Tilt (degrees, folded to <= 90) between the ENU Up axis and the normal of the
// best-fit plane through `pts` AFTER applying the rigid transform T. Negative
// when the plane cannot be fitted (too few points).
double alignedPlaneTiltDeg(const std::vector<Eigen::Vector3d>& pts,
                           const Eigen::Matrix4d& T) {
    std::vector<Eigen::Vector3d> transformed;
    transformed.reserve(pts.size());
    for (const Eigen::Vector3d& p : pts) {
        transformed.push_back((T * p.homogeneous()).hnormalized());
    }
    Eigen::Vector3d c, n;
    if (!fitPlane(transformed, c, n)) {
        return -1.0;
    }
    const double cos_ang =
        std::min(1.0, std::abs(n.dot(Eigen::Vector3d::UnitZ())));
    return std::acos(cos_ang) * DEG_PER_RAD;
}

// Horizontal (E,N) bounding-box diagonal of a point set, in metres. Used by the
// optional `alignment_min_spread_m` freeze gate.
double horizontalSpreadM(const std::vector<Eigen::Vector3d>& pts) {
    if (pts.empty()) {
        return 0.0;
    }
    double min_e = pts.front().x(), max_e = min_e;
    double min_n = pts.front().y(), max_n = min_n;
    for (const Eigen::Vector3d& p : pts) {
        min_e = std::min(min_e, p.x());
        max_e = std::max(max_e, p.x());
        min_n = std::min(min_n, p.y());
        max_n = std::max(max_n, p.y());
    }
    return std::hypot(max_e - min_e, max_n - min_n);
}
#endif  // UAVLOC_WITH_DEBUG_VIEWER

// Reads the next valid (OK + non-empty) frame from the reader into `out`.
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
                return false;
            }
        }
        if (status == sensor::FrameStatus::ERROR ||
            status == sensor::FrameStatus::CAMERA_DISCONNECTED) {
            return false;
        }
    }
}

}  // namespace
}  // namespace uavloc

int main(int argc, char** argv) {
    using namespace uavloc;

    // Honour SPDLOG_LEVEL for a verbose per-frame run.
    spdlog::cfg::load_env_levels();

    const std::string config_path =
        (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    // Optional hard cap on frames fed to the pipeline (headless smoke). 0 = run to
    // END_OF_STREAM. Mirrors test_vo_pipeline_new's UAVLOC_VO_MAXFRAMES.
    const char* mf = std::getenv("UAVLOC_VO_MAXFRAMES");
    const int max_frames_env = mf ? std::max(0, std::atoi(mf)) : 0;

    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }

    sensor::VideoReaderConfig reader_cfg;
    vo::VOConfig              vo_cfg;
    try {
        reader_cfg = sensor::VideoReaderConfig::fromYaml(yaml);
        vo_cfg     = vo::VOConfig::fromYaml(yaml);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }

    sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::warn("test_vo_viewer: cannot open video '{}' — SKIPPED",
                     reader_cfg.video_path);
        return 0;  // soft-skip when the dataset is absent
    }
    spdlog::info("Video opened — fps: {:.1f}, total frames: {}",
                 reader.getFps(), reader.getFrameCount());

    vo::VOModule vo_module(vo_cfg);

    // ── Latest keyframe map-point cloud (world, metres), captured on the worker
    //    thread via the VOData publish callback. Guarded by a mutex because the
    //    callback fires inside process_frame (worker thread) while the same worker
    //    later reads it to push into the viewer. ──────────────────────────────────
    std::mutex          map_mutex;
    std::vector<Vec3_t> latest_map_points;
    bool                map_dirty = false;

    vo_module.add_data_out_callback([&](const vo::VOData& data) {
        if (data.map_updated && !data.map_points.empty()) {
            std::lock_guard<std::mutex> lk(map_mutex);
            latest_map_points = data.map_points;
            map_dirty         = true;
        }
    });

    // Shared shutdown flag between the VO worker and the (optional) viewer on the
    // main thread. Only ever flips to true in the debug-viewer build (headless
    // runs never spawn a worker).
    std::atomic<bool> stop_requested{false};

    std::size_t frames_processed = 0;
    std::size_t pose_successes   = 0;

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    namespace dv = uavloc::debug_viewer;

    // Alignment window: number of initial TRACKING frames over which the (frozen)
    // VO-world -> groundtruth-ENU rigid transform is computed (4-DoF gravity-
    // aligned, scale fixed = 1 since VO is metric). Loaded from YAML so it is
    // not a magic number.
    const int alignment_window = std::max(
        1, yaml["DebugViewer"]["alignment_window_frames"].as<int>(50));
    // Optional freeze gate: when > 0, delay freezing the alignment until the
    // groundtruth window spans at least this many metres horizontally (guards
    // the yaw estimate against a near-stationary window). 0 = off (default,
    // preserves the count-only behaviour).
    const double alignment_min_spread_m =
        yaml["DebugViewer"]["alignment_min_spread_m"].as<double>(0.0);

    // Viewer config: forward the display/vertical scale + side-panel keys from
    // the YAML DebugViewer: section (defaults preserved when keys are absent).
    dv::DebugViewer::Config viewer_cfg;
    viewer_cfg.display_scale =
        yaml["DebugViewer"]["display_scale"].as<float>(viewer_cfg.display_scale);
    viewer_cfg.vertical_scale =
        yaml["DebugViewer"]["vertical_scale"].as<float>(viewer_cfg.vertical_scale);
    viewer_cfg.video_stride =
        yaml["DebugViewer"]["video_stride"].as<int>(viewer_cfg.video_stride);
    viewer_cfg.log_capacity =
        yaml["DebugViewer"]["log_capacity"].as<int>(viewer_cfg.log_capacity);
    viewer_cfg.metric_plot_history =
        yaml["DebugViewer"]["metric_plot_history"].as<int>(viewer_cfg.metric_plot_history);
    viewer_cfg.metric_plot_follow_window =
        yaml["DebugViewer"]["metric_plot_follow_window"].as<int>(viewer_cfg.metric_plot_follow_window);
    viewer_cfg.histogram_bins =
        yaml["DebugViewer"]["histogram_bins"].as<int>(viewer_cfg.histogram_bins);
    viewer_cfg.perf_plot_history =
        yaml["DebugViewer"]["perf_plot_history"].as<int>(viewer_cfg.perf_plot_history);
    viewer_cfg.show_log =
        yaml["DebugViewer"]["show_log"].as<bool>(viewer_cfg.show_log);
    viewer_cfg.show_inliers =
        yaml["DebugViewer"]["show_inliers"].as<bool>(viewer_cfg.show_inliers);
    viewer_cfg.show_landmarks =
        yaml["DebugViewer"]["show_landmarks"].as<bool>(true);
    viewer_cfg.show_video =
        yaml["DebugViewer"]["show_video"].as<bool>(viewer_cfg.show_video);
    viewer_cfg.show_tracking =
        yaml["DebugViewer"]["show_tracking"].as<bool>(viewer_cfg.show_tracking);
    viewer_cfg.show_lost_poses =
        yaml["DebugViewer"]["show_lost_poses"].as<bool>(viewer_cfg.show_lost_poses);
    viewer_cfg.show_hud =
        yaml["DebugViewer"]["show_hud"].as<bool>(viewer_cfg.show_hud);
    viewer_cfg.show_perf =
        yaml["DebugViewer"]["show_perf"].as<bool>(viewer_cfg.show_perf);
    viewer_cfg.show_groundtruth =
        yaml["DebugViewer"]["show_groundtruth"].as<bool>(viewer_cfg.show_groundtruth);
    viewer_cfg.est_odom =
        yaml["DebugViewer"]["est_odom"].as<bool>(viewer_cfg.est_odom);
    viewer_cfg.est_odom_every_n =
        yaml["DebugViewer"]["est_odom_every_n"].as<int>(viewer_cfg.est_odom_every_n);
    viewer_cfg.est_odom_axes_scale =
        yaml["DebugViewer"]["est_odom_axes_scale"].as<float>(viewer_cfg.est_odom_axes_scale);
    const int video_stride = std::max(1, viewer_cfg.video_stride);
    // Minimum per-frame baseline (metres) before the COG heading is refreshed
    // (anti-jitter when hovering / near-stationary). Same key as the old test.
    const double hud_min_baseline_m =
        yaml["DebugViewer"]["hud_min_baseline_m"].as<double>(0.5);
    // Period (ms) of the /proc CPU%/RSS performance sampler thread.
    const int perf_sample_ms =
        std::max(50, yaml["DebugViewer"]["perf_sample_ms"].as<int>(500));
    dv::DebugViewer viewer(viewer_cfg);
    viewer.loadBatch({});  // empty -> live push mode (holds window, drains queues)

    // Capture spdlog output into the viewer's scrolling log panel.
    spdlog::default_logger()->sinks().push_back(viewer.logSink());

    // Alignment accumulation state (worker-thread-local; captured by ref).
    std::vector<Eigen::Vector3d> align_src;   // p_cam in the VO world frame
    std::vector<Eigen::Vector3d> align_dst;   // matching groundtruth ENU
    // Pending estimates buffered until the alignment freezes. The FULL T_wc is
    // kept (not just the position) so the flushed poses carry roll/pitch/yaw
    // for the viewer's "EstOdom" pose-axes rendering.
    std::vector<std::pair<int, Eigen::Matrix4d>,
                Eigen::aligned_allocator<std::pair<int, Eigen::Matrix4d>>>
        est_buffer;
    std::vector<std::pair<int, Eigen::Vector3d>> lost_buffer; // pending LOST markers
    Eigen::Matrix4d T_align  = Eigen::Matrix4d::Identity();
    bool   aligned           = false;
    bool   gt_origin_set     = false;
    double gt_o_lat = 0.0, gt_o_lon = 0.0, gt_o_alt = 0.0;

    // HUD bookkeeping (worker-thread-local): last plotted VO-world position,
    // accumulated metric path length, and the held course-over-ground heading.
    Eigen::Vector3d last_pos            = Eigen::Vector3d::Zero();
    bool            has_last_pos        = false;
    double          trajectory_length_m = 0.0;
    double          last_cog_heading_deg = 0.0;
#endif

    // VO pipeline body. Runs on a worker thread in the debug-viewer build (the
    // viewer owns the main thread for GL); runs inline otherwise.
    auto run_pipeline = [&]() {
#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // Transform a VO-world position through the frozen alignment and push it
        // as a discrete LOST-state marker (red, "Lost state poses" checkbox).
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
        while (!stop_requested.load() && readNextValidFrame(reader, frame)) {
            ++frames_processed;

#ifdef UAVLOC_WITH_DEBUG_VIEWER
            // Stream every `video_stride` frame into the viewer's video panel.
            if (frames_processed % static_cast<std::size_t>(video_stride) == 0) {
                viewer.pushFrame(frame.image);
            }
#endif

            // ── Feed the frame to the VO module (synchronous: frame -> pose) ────
            const vo::VOResult r = vo_module.process_frame(frame);

            if (r.has_pose) {
                ++pose_successes;
            }

#ifdef UAVLOC_WITH_DEBUG_VIEWER
            // ── LOST markers: mark the last valid position — the point where
            //    tracking broke — so the marker sits on the estimate polyline.
            //    Push through T_align once aligned, otherwise buffer. ────────────
            if (r.state == vo::VOTrackingState::LOST && has_last_pos) {
                if (aligned) {
                    push_lost(static_cast<int>(r.frame_id), last_pos);
                } else {
                    lost_buffer.emplace_back(static_cast<int>(r.frame_id), last_pos);
                }
            }

            // ── HUD bookkeeping + per-frame metrics (has_pose frames) ───────────
            if (r.has_pose) {
                const Eigen::Vector3d p = r.T_wc.block<3, 1>(0, 3);
                // Path length: sum of consecutive GLOBAL position deltas. The
                // increment is identity on the first step of a (re-)initialised
                // map; at those boundaries the position can jump, so skip them.
                const bool reinit_boundary =
                    r.T_prev_curr.isApprox(Eigen::Matrix4d::Identity(), 1e-9);
                const Eigen::Vector3d dp = has_last_pos
                    ? Eigen::Vector3d(p - last_pos) : Eigen::Vector3d::Zero();
                if (has_last_pos && !reinit_boundary) {
                    trajectory_length_m += dp.norm();
                    // VO course-over-ground heading (ENU, 0 = North CW): rotate
                    // the per-frame displacement into ENU through the frozen
                    // Umeyama alignment (the old test used geo.R_enu_w() here).
                    // Held when the baseline is too small to trust the direction.
                    if (aligned && dp.norm() >= hud_min_baseline_m) {
                        const Eigen::Vector3d dp_enu =
                            T_align.block<3, 3>(0, 0) * dp;
                        last_cog_heading_deg = wrap360(
                            std::atan2(dp_enu.x(), dp_enu.y()) * DEG_PER_RAD);
                    }
                }
                last_pos     = p;
                has_last_pos = true;

                // Per-frame metrics: inliers/landmarks line charts + HUD scalars.
                const float hud_heading_tel =
                    frame.has_telemetry
                        ? static_cast<float>(wrap360(
                              frame.telemetry.heading_deg +
                              frame.telemetry.gimbal_pan_deg))
                        : 0.0f;
                dv::FrameMetrics fm;
                fm.frame_id        = static_cast<int>(r.frame_id);
                fm.inliers         = static_cast<float>(r.num_inliers);
                fm.landmarks       = static_cast<float>(r.num_landmarks);
                fm.heading_deg     = static_cast<float>(last_cog_heading_deg);
                fm.heading_tel_deg = hud_heading_tel;
                fm.distance_m      = static_cast<float>(trajectory_length_m);
                viewer.pushMetrics(fm);

                // Tracking sub-window: overlay the tracked landmark keypoints on
                // the frame and annotate the counts (same stride as the plain
                // video panel).
                if (frames_processed % static_cast<std::size_t>(video_stride) == 0) {
                    cv::Mat overlay = frame.image.clone();
                    for (const Eigen::Vector2d& obs : r.tracked_observations) {
                        cv::circle(overlay,
                                   cv::Point(static_cast<int>(std::lround(obs.x())),
                                             static_cast<int>(std::lround(obs.y()))),
                                   TRACK_KP_RADIUS_PX,
                                   cv::Scalar(0, 255, 0),  // green (BGR)
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
            }

            // ── Overlay: estimate vs groundtruth (TRACKING frames only) ─────────
            if (r.has_pose && r.state == vo::VOTrackingState::TRACKING &&
                frame.has_telemetry && frame.telemetry.valid &&
                frame.telemetry.altitude_m > 0.0) {
                const auto& t = frame.telemetry;
                if (!gt_origin_set) {
                    gt_o_lat      = t.latitude_deg;
                    gt_o_lon      = t.longitude_deg;
                    gt_o_alt      = t.altitude_m;
                    gt_origin_set = true;
                }
                const dv::ENUPoint e = dv::gps_to_enu(
                    t.latitude_deg, t.longitude_deg, t.altitude_m,
                    gt_o_lat, gt_o_lon, gt_o_alt);
                const Eigen::Vector3d gt_enu(e.e, e.n, e.u);
                const Eigen::Vector3d p_cam = r.T_wc.block<3, 1>(0, 3);

                // Groundtruth is independent of alignment — push it every frame.
                dv::InferredPose gp;
                gp.frame_id = static_cast<int>(r.frame_id);
                gp.x = gt_enu.x(); gp.y = gt_enu.y(); gp.z = gt_enu.z();
                viewer.pushGroundtruthPose(gp);

                auto push_estimate = [&](int fid, const Eigen::Matrix4d& T_wc) {
                    const Eigen::Vector3d p_world = T_wc.block<3, 1>(0, 3);
                    const Eigen::Vector3d est =
                        (T_align * p_world.homogeneous()).hnormalized();
                    dv::InferredPose ep;
                    ep.frame_id = fid;
                    ep.x = est.x(); ep.y = est.y(); ep.z = est.z();
                    // Orientation for the viewer's "EstOdom" pose axes: rotate
                    // the VO camera rotation into ENU through the frozen
                    // alignment, then extract ZYX (yaw, pitch, roll) — the same
                    // convention the viewer's gizmo helper composes back as
                    // Rz(yaw) * Ry(pitch) * Rx(roll).
                    const Eigen::Matrix3d R_enu =
                        T_align.block<3, 3>(0, 0) * T_wc.block<3, 3>(0, 0);
                    const Eigen::Vector3d ypr = R_enu.eulerAngles(2, 1, 0);
                    ep.yaw_deg   = ypr.x() * DEG_PER_RAD;
                    ep.pitch_deg = ypr.y() * DEG_PER_RAD;
                    ep.roll_deg  = ypr.z() * DEG_PER_RAD;
                    viewer.pushPose(ep);
                };

                if (!aligned) {
                    // Accumulate correspondences; buffer estimates until the frozen
                    // transform is known, then flush them aligned.
                    align_src.push_back(p_cam);
                    align_dst.push_back(gt_enu);
                    est_buffer.emplace_back(static_cast<int>(r.frame_id), r.T_wc);

                    if (static_cast<int>(align_src.size()) >= alignment_window &&
                        (alignment_min_spread_m <= 0.0 ||
                         horizontalSpreadM(align_dst) >= alignment_min_spread_m)) {
                        const std::size_t n_corr = align_src.size();
                        Eigen::Matrix3Xd S(3, n_corr);
                        Eigen::Matrix3Xd D(3, n_corr);
                        for (std::size_t i = 0; i < n_corr; ++i) {
                            S.col(static_cast<Eigen::Index>(i)) = align_src[i];
                            D.col(static_cast<Eigen::Index>(i)) = align_dst[i];
                        }
                        // Diagnostic reference ONLY: the previous free-3D-rotation
                        // Umeyama (scale fixed = 1). On this near-collinear camera
                        // window its roll about the flight axis is degenerate — it
                        // is computed solely for the before/after tilt log below.
                        const Eigen::Matrix4d T_umeyama3d =
                            Eigen::umeyama(S, D, /*with_scaling=*/false);

                        // Snapshot the latest keyframe map-point cloud (VO world,
                        // metres) — the landmark carpet defines the ground plane.
                        std::vector<Eigen::Vector3d> plane_pts;
                        {
                            std::lock_guard<std::mutex> lk(map_mutex);
                            plane_pts.assign(latest_map_points.begin(),
                                             latest_map_points.end());
                        }

                        Eigen::Vector3d plane_c, plane_n;
                        if (fitPlane(plane_pts, plane_c, plane_n)) {
                            // 1) VO-side "up": landmark-plane normal, signed from
                            //    the plane centroid TOWARD the mean camera position
                            //    of the window (the camera flies above the ground).
                            Eigen::Vector3d cam_mean = Eigen::Vector3d::Zero();
                            for (const Eigen::Vector3d& s : align_src) {
                                cam_mean += s;
                            }
                            cam_mean /= static_cast<double>(n_corr);
                            Eigen::Vector3d up_vo = plane_n;
                            if (up_vo.dot(cam_mean - plane_c) < 0.0) {
                                up_vo = -up_vo;
                            }

                            // 2) R0: rotate up_vo onto ENU Up — kills roll/pitch.
                            const Eigen::Matrix3d R0 =
                                Eigen::Quaterniond::FromTwoVectors(
                                    up_vo, Eigen::Vector3d::UnitZ())
                                    .toRotationMatrix();

                            // 3) Yaw about Up + translation: 2D Procrustes (no
                            //    scale) on the horizontal components of the R0-
                            //    levelled window. With C = sum(d_c * s'_c^T) the
                            //    optimal yaw is atan2(C10 - C01, C00 + C11).
                            Eigen::Vector3d s_mean = Eigen::Vector3d::Zero();
                            Eigen::Vector3d d_mean = Eigen::Vector3d::Zero();
                            std::vector<Eigen::Vector3d> s_lev(n_corr);
                            for (std::size_t i = 0; i < n_corr; ++i) {
                                s_lev[i] = R0 * align_src[i];
                                s_mean += s_lev[i];
                                d_mean += align_dst[i];
                            }
                            s_mean /= static_cast<double>(n_corr);
                            d_mean /= static_cast<double>(n_corr);
                            Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
                            for (std::size_t i = 0; i < n_corr; ++i) {
                                const Eigen::Vector2d sc =
                                    (s_lev[i] - s_mean).head<2>();
                                const Eigen::Vector2d dc =
                                    (align_dst[i] - d_mean).head<2>();
                                C += dc * sc.transpose();
                            }
                            const double yaw = std::atan2(C(1, 0) - C(0, 1),
                                                          C(0, 0) + C(1, 1));
                            const Eigen::Matrix3d R_yaw =
                                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                    .toRotationMatrix();

                            // 4) T_align = [R_yaw*R0 | t]; t from the centroids
                            //    (E,N via the 2D fit; U = mean altitude offset —
                            //    yaw does not change the Up component).
                            T_align.setIdentity();
                            T_align.block<3, 3>(0, 0) = R_yaw * R0;
                            T_align.block<3, 1>(0, 3) = d_mean - R_yaw * s_mean;

                            // 5) Quantitative tilt check (headless-verifiable):
                            //    re-fit the plane on the ALIGNED landmarks.
                            spdlog::info("alignment: landmark-plane tilt after "
                                         "align = {:.2f} deg (4-DoF gravity-aligned, "
                                         "yaw {:.2f} deg, {} plane points)",
                                         alignedPlaneTiltDeg(plane_pts, T_align),
                                         yaw * DEG_PER_RAD, plane_pts.size());
                            spdlog::info("alignment: OLD free-3D umeyama would give "
                                         "landmark-plane tilt = {:.2f} deg "
                                         "(degenerate roll, for comparison only)",
                                         alignedPlaneTiltDeg(plane_pts, T_umeyama3d));
                        } else {
                            spdlog::warn("alignment: no landmark snapshot to fit "
                                         "the ground plane ({} pts) — falling back "
                                         "to free 3D umeyama (roll unconstrained)",
                                         plane_pts.size());
                            T_align = T_umeyama3d;
                        }
                        aligned = true;
                        spdlog::info("alignment frozen at frame {} over {} TRACKING "
                                     "correspondences", r.frame_id, n_corr);
                        for (const auto& fp : est_buffer) {
                            push_estimate(fp.first, fp.second);
                        }
                        est_buffer.clear();
                        // Flush LOST markers gathered before alignment froze.
                        for (const auto& fp : lost_buffer) {
                            push_lost(fp.first, fp.second);
                        }
                        lost_buffer.clear();
                    }
                } else {
                    push_estimate(static_cast<int>(r.frame_id), r.T_wc);
                }
            }

            // ── Map-point cloud: push the latest keyframe cloud through the frozen
            //    alignment (only once the transform exists). ────────────────────
            if (aligned) {
                std::vector<Vec3_t> pts_world;
                {
                    std::lock_guard<std::mutex> lk(map_mutex);
                    if (map_dirty) {
                        pts_world = latest_map_points;
                        map_dirty = false;
                    }
                }
                if (!pts_world.empty()) {
                    std::vector<Eigen::Vector3f> pts_enu;
                    pts_enu.reserve(pts_world.size());
                    for (const Vec3_t& p : pts_world) {
                        const Eigen::Vector3d en =
                            (T_align * p.homogeneous()).hnormalized();
                        pts_enu.emplace_back(static_cast<float>(en.x()),
                                             static_cast<float>(en.y()),
                                             static_cast<float>(en.z()));
                    }
                    viewer.pushMapPoints(pts_enu);
                }
            }
#endif

            if (max_frames_env > 0 &&
                frames_processed >= static_cast<std::size_t>(max_frames_env)) {
                break;
            }
        }
        reader.close();
#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // Signal the viewer (main thread) that the stream is exhausted.
        stop_requested.store(true);
        viewer.stop();
#endif
    };  // run_pipeline

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // Performance sampler: a tiny dedicated thread that every `perf_sample_ms`
    // takes a util::SysMonitor sample (CPU % + RSS MB from /proc, measured
    // inside the util module) and pushes a PerfSample into the viewer's
    // "Performance" streaming charts. Producer-side measurement — the viewer
    // only plots (module decoupling). Terminates on stop_requested.
    std::atomic<std::size_t> perf_samples{0};
    std::thread perf_sampler([&]() {
        uavloc::util::SysMonitor mon;
        while (!stop_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(perf_sample_ms));
            uavloc::util::SysStats s;
            if (mon.sample(s)) {
                viewer.pushPerf(dv::PerfSample{s.t_sec, s.cpu_percent, s.rss_mb});
                ++perf_samples;
            }
        }
    });

    // Viewer owns the main thread (GL singleton); VO runs on a worker. When the
    // user closes the window run() returns early — flag the worker and join.
    std::thread worker(run_pipeline);
    viewer.run();              // blocking on a display; returns immediately headless
    if (viewer.hadDisplay()) {
        stop_requested.store(true);
    }
    worker.join();
    stop_requested.store(true);  // run_pipeline sets it too; belt-and-braces
    perf_sampler.join();
    spdlog::info("perf sampler: {} samples", perf_samples.load());
#else
    run_pipeline();
#endif

    spdlog::info("test_vo_viewer: frames_processed={} pose_successes={} final_state={}",
                 frames_processed, pose_successes,
                 static_cast<int>(vo_module.get_state()));
    return 0;
}
