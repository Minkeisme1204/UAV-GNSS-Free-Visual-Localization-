// test_vo_debug_new — headless diagnostic driver for the new_vo VOModule using
// the publish/subscribe VOData callback (the same channel fusion/anchor/viewer
// consume).
//
// Mirrors the old tests/test_vo_debug.cpp: it builds a vo::VOModule from the
// YenBai mission YAML, registers an add_data_out_callback() lambda that collects
// each frame's GLOBAL pose (T_wc, when has_pose) and keeps the latest keyframe
// map-point cloud, feeds up to MAX_FRAMES valid frames from sensor::VideoReader,
// and renders a top-down world X-Y trajectory image (vo_new_trajectory.png)
// using the fit -> project -> cv::imwrite pattern. It asserts the callback fired
// at least once, the pipeline reached TRACKING, at least one keyframe was
// inserted, and all collected poses are finite. Soft-skips (returns 0) when the
// gitignored dataset video is unavailable, mirroring tests/test_vo_pipeline_new.
//
// When built with UAVLOC_WITH_DEBUG_VIEWER (Iridescence + uavloc_debug_viewer),
// the collected VO-world poses/points are aligned to groundtruth ENU (Umeyama,
// scale fixed) and pushed to a live 3D DebugViewer via pushPose/pushMapPoints
// (the point cloud is toggled by the viewer's "Landmarks" checkbox). Headless
// runs exit cleanly (no DISPLAY).
//
// Usage (from build/):
//     ./tests/test_vo_debug_new [config.yaml] [max_frames] [trajectory.png]
#include "uavloc/sensor/video_reader.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/new_vo/vo_module.h"

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#ifdef UAVLOC_WITH_DEBUG_VIEWER
#include "uavloc/debug_viewer/debug_viewer.h"
#include "gps_to_enu.h"    // src/debug_viewer (on this target's include path)
#include <Eigen/Geometry>  // Eigen::umeyama
#include <thread>
#include <utility>
#endif

namespace uavloc {
namespace {

const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m.yaml";

// Number of valid frames to process before stopping. Overridable via argv[2].
constexpr int DEBUG_MAX_FRAMES = 60;

// Maximum consecutive EMPTY_FRAME results tolerated before the reader is treated
// as exhausted (some codecs over-report CAP_PROP_FRAME_COUNT).
constexpr int MAX_CONSECUTIVE_EMPTY_FRAMES = 30;

// Where the top-down trajectory visualization is written (current working dir).
const std::string OUTPUT_IMAGE_PATH = "vo_new_trajectory.png";

// ── Trajectory image presentation constants (rendering only — NOT pipeline
//    thresholds). Kept here so no magic numbers leak into the draw code. ───────
const int    CANVAS_SIZE_PX       = 1000;
const int    CANVAS_MARGIN_PX     = 80;
const int    POLYLINE_THICKNESS   = 2;
const int    MARKER_RADIUS_PX     = 7;
const int    HEADING_EVERY_N      = 2;
const int    HEADING_LEN_PX       = 26;
const int    HEADING_THICKNESS    = 1;
const double HEADING_TIP_LENGTH   = 0.35;
const int    SCALE_BAR_MARGIN_PX  = 20;
const int    SCALE_BAR_HEIGHT_PX  = 8;
const double SCALE_BAR_TARGET_FRAC = 0.25;
const double TEXT_SCALE            = 0.5;
const int    TEXT_THICKNESS        = 1;
const double HEADING_EPS           = 1e-9;

// One plotted pose sample: GLOBAL camera position (X/Y used, Z dropped) plus a
// ground-plane heading derived from the T_wc rotation.
struct PoseSample {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector2d heading  = Eigen::Vector2d::Zero();
};

// Round a positive value down to a "nice" 1/2/5 x 10^k number.
double niceRound(double value) {
    if (value <= 0.0) return 0.0;
    const double exp10 = std::floor(std::log10(value));
    const double base  = std::pow(10.0, exp10);
    const double frac  = value / base;
    double nice = 1.0;
    if (frac >= 5.0)      nice = 5.0;
    else if (frac >= 2.0) nice = 2.0;
    return nice * base;
}

// Project the gathered poses onto the ground plane (top-down: world X horizontal,
// world Y vertical; Z dropped) and write a fitted polyline with start/end
// markers, per-pose heading arrows and a metric scale bar. Mirrors the auto-fit +
// toPixel + imwrite pattern of the old test_vo_debug::renderTrajectory.
bool renderTrajectory(const std::vector<PoseSample>& poses,
                      const std::string& output_path) {
    if (poses.size() < 2) {
        spdlog::warn("Trajectory has < 2 poses; nothing to render");
        return false;
    }

    double min_x = poses.front().position.x(), max_x = min_x;
    double min_y = poses.front().position.y(), max_y = min_y;
    for (const PoseSample& s : poses) {
        min_x = std::min(min_x, s.position.x());
        max_x = std::max(max_x, s.position.x());
        min_y = std::min(min_y, s.position.y());
        max_y = std::max(max_y, s.position.y());
    }

    const double span_x    = std::max(max_x - min_x, 1e-9);
    const double span_y    = std::max(max_y - min_y, 1e-9);
    const double draw_area = static_cast<double>(CANVAS_SIZE_PX - 2 * CANVAS_MARGIN_PX);
    const double scale     = draw_area / std::max(span_x, span_y);

    auto toPixel = [&](const Eigen::Vector3d& p) {
        const double px = CANVAS_MARGIN_PX + (p.x() - min_x) * scale;
        const double py = CANVAS_SIZE_PX - CANVAS_MARGIN_PX - (p.y() - min_y) * scale;
        return cv::Point(static_cast<int>(px), static_cast<int>(py));
    };

    cv::Mat canvas(CANVAS_SIZE_PX, CANVAS_SIZE_PX, CV_8UC3, cv::Scalar(30, 30, 30));

    for (std::size_t i = 1; i < poses.size(); ++i) {
        cv::line(canvas, toPixel(poses[i - 1].position), toPixel(poses[i].position),
                 cv::Scalar(0, 255, 0), POLYLINE_THICKNESS, cv::LINE_AA);
    }

    for (std::size_t i = 0; i < poses.size(); i += HEADING_EVERY_N) {
        const Eigen::Vector2d& h = poses[i].heading;
        if (h.norm() < HEADING_EPS) continue;
        const Eigen::Vector2d dir = h.normalized();
        const cv::Point base = toPixel(poses[i].position);
        const cv::Point tip(
            base.x + static_cast<int>(dir.x() * HEADING_LEN_PX),
            base.y - static_cast<int>(dir.y() * HEADING_LEN_PX));
        cv::arrowedLine(canvas, base, tip, cv::Scalar(255, 255, 0),
                        HEADING_THICKNESS, cv::LINE_AA, 0, HEADING_TIP_LENGTH);
    }

    cv::circle(canvas, toPixel(poses.front().position), MARKER_RADIUS_PX,
               cv::Scalar(255, 128, 0), 2, cv::LINE_AA);
    cv::circle(canvas, toPixel(poses.front().position), 2,
               cv::Scalar(255, 128, 0), cv::FILLED, cv::LINE_AA);
    cv::putText(canvas, "start", toPixel(poses.front().position) + cv::Point(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, cv::Scalar(255, 128, 0),
                TEXT_THICKNESS, cv::LINE_AA);
    cv::circle(canvas, toPixel(poses.back().position), MARKER_RADIUS_PX,
               cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(canvas, "end", toPixel(poses.back().position) + cv::Point(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, cv::Scalar(0, 0, 255),
                TEXT_THICKNESS, cv::LINE_AA);

    const double meters_per_px = 1.0 / scale;
    const double target_m      = draw_area * SCALE_BAR_TARGET_FRAC * meters_per_px;
    const double bar_m         = niceRound(target_m);
    const int    bar_px        = static_cast<int>(bar_m * scale);
    if (bar_px > 0) {
        const int y0 = CANVAS_SIZE_PX - SCALE_BAR_MARGIN_PX;
        const int x0 = SCALE_BAR_MARGIN_PX;
        const int x1 = x0 + bar_px;
        cv::line(canvas, cv::Point(x0, y0), cv::Point(x1, y0),
                 cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
        cv::line(canvas, cv::Point(x0, y0 - SCALE_BAR_HEIGHT_PX),
                 cv::Point(x0, y0 + SCALE_BAR_HEIGHT_PX),
                 cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
        cv::line(canvas, cv::Point(x1, y0 - SCALE_BAR_HEIGHT_PX),
                 cv::Point(x1, y0 + SCALE_BAR_HEIGHT_PX),
                 cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
        cv::putText(canvas, std::to_string(static_cast<int>(bar_m)) + " m (metric)",
                    cv::Point(x0, y0 - SCALE_BAR_HEIGHT_PX - 6),
                    cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, cv::Scalar(220, 220, 220),
                    TEXT_THICKNESS, cv::LINE_AA);
    }

    cv::putText(canvas, "top-down VO trajectory (world X-Y, altitude dropped)",
                cv::Point(SCALE_BAR_MARGIN_PX, 30), cv::FONT_HERSHEY_SIMPLEX,
                TEXT_SCALE, cv::Scalar(200, 200, 200), TEXT_THICKNESS, cv::LINE_AA);

    if (!cv::imwrite(output_path, canvas)) {
        spdlog::error("Failed to write trajectory image to '{}'", output_path);
        return false;
    }
    spdlog::info("Trajectory image written to '{}' ({} poses)", output_path, poses.size());
    return true;
}

const char* stateName(vo::VOTrackingState s) {
    switch (s) {
        case vo::VOTrackingState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case vo::VOTrackingState::TRACKING:        return "TRACKING";
        case vo::VOTrackingState::LOST:            return "LOST";
    }
    return "UNKNOWN";
}

bool poseIsFinite(const Mat44_t& T) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(T(r, c))) return false;
        }
    }
    return true;
}

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

    spdlog::cfg::load_env_levels();

    const std::string config_path = (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;
    const int         max_frames  = (argc > 2) ? std::max(1, std::atoi(argv[2])) : DEBUG_MAX_FRAMES;
    const std::string output_path = (argc > 3) ? std::string(argv[3]) : OUTPUT_IMAGE_PATH;

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
        spdlog::warn("test_vo_debug_new: cannot open video '{}' — SKIPPED", reader_cfg.video_path);
        return 0;  // soft-skip when the dataset is absent
    }

    vo::VOModule vo_module(vo_cfg);

    // ── Collected via the VOData publish callback ───────────────────────────────
    std::vector<PoseSample>  poses;               // finite has_pose samples (VO frame)
    std::vector<Vec3_t>      latest_map_points;    // last keyframe map cloud (VO frame)
    int  callback_calls   = 0;
    bool reached_tracking = false;
    bool saw_keyframe     = false;
    bool all_poses_finite = true;

    vo_module.add_data_out_callback([&](const vo::VOData& data) {
        ++callback_calls;
        const vo::VOResult& r = data.result;
        if (r.state == vo::VOTrackingState::TRACKING) reached_tracking = true;
        if (r.is_keyframe)                            saw_keyframe = true;
        if (r.has_pose) {
            if (!poseIsFinite(r.T_wc)) {
                all_poses_finite = false;
            } else {
                PoseSample s;
                s.position = r.T_wc.block<3, 1>(0, 3);
                const Eigen::Vector2d h = (-r.T_wc.block<3, 1>(0, 1)).head<2>();
                if (h.norm() >= HEADING_EPS) s.heading = h;
                poses.push_back(s);
            }
        }
        if (data.map_updated && !data.map_points.empty()) {
            latest_map_points = data.map_points;
        }
    });

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // VO-world position (has_pose) paired with the frame's groundtruth ENU, used
    // to freeze a rigid VO->ENU transform (Umeyama, scale fixed = 1 since VO is
    // metric). Filled in the main loop where telemetry is available.
    std::vector<Eigen::Vector3d> align_src;  // p_cam in the VO world frame
    std::vector<Eigen::Vector3d> align_dst;  // matching groundtruth ENU
    bool   gt_origin_set = false;
    double gt_o_lat = 0.0, gt_o_lon = 0.0, gt_o_alt = 0.0;
#endif

    int frames_fed = 0;
    sensor::FrameData frame;
    while (frames_fed < max_frames && readNextValidFrame(reader, frame)) {
        const vo::VOResult r = vo_module.process_frame(frame);
        ++frames_fed;

#ifdef UAVLOC_WITH_DEBUG_VIEWER
        if (r.has_pose && r.state == vo::VOTrackingState::TRACKING &&
            frame.has_telemetry && frame.telemetry.valid) {
            const auto& t = frame.telemetry;
            if (!gt_origin_set) {
                gt_o_lat = t.latitude_deg;
                gt_o_lon = t.longitude_deg;
                gt_o_alt = t.altitude_m;
                gt_origin_set = true;
            }
            const debug_viewer::ENUPoint e = debug_viewer::gps_to_enu(
                t.latitude_deg, t.longitude_deg, t.altitude_m, gt_o_lat, gt_o_lon, gt_o_alt);
            align_src.push_back(r.T_wc.block<3, 1>(0, 3));
            align_dst.emplace_back(e.e, e.n, e.u);
        }
#endif
    }
    reader.close();

    if (frames_fed == 0) {
        spdlog::warn("test_vo_debug_new: no valid frames in stream — SKIPPED");
        return 0;
    }

    renderTrajectory(poses, output_path);

    int rc = 0;
    auto check = [&](bool ok, const std::string& what) {
        if (!ok) { spdlog::error("test_vo_debug_new FAIL: {}", what); rc = 1; }
    };
    check(callback_calls >= 1, "data-out callback never fired");
    check(reached_tracking,    "pipeline never reached TRACKING state");
    check(saw_keyframe,        "no keyframe inserted");
    check(all_poses_finite,    "a T_wc pose contained non-finite values");
    check(!poses.empty(),      "no poses collected via callback");

    spdlog::info(
        "test_vo_debug_new: frames_fed={} callback_calls={} poses={} map_points={} "
        "reached_tracking={} saw_keyframe={} final_state={}",
        frames_fed, callback_calls, poses.size(), latest_map_points.size(),
        reached_tracking, saw_keyframe, stateName(vo_module.get_state()));

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    namespace dv = uavloc::debug_viewer;
    if (align_src.size() >= 2) {
        Eigen::Matrix3Xd S(3, align_src.size());
        Eigen::Matrix3Xd D(3, align_dst.size());
        for (std::size_t i = 0; i < align_src.size(); ++i) {
            S.col(static_cast<Eigen::Index>(i)) = align_src[i];
            D.col(static_cast<Eigen::Index>(i)) = align_dst[i];
        }
        const Eigen::Matrix4d T_align = Eigen::umeyama(S, D, /*with_scaling=*/false);

        dv::DebugViewer::Config viewer_cfg;
        viewer_cfg.show_landmarks = true;  // reveal the pushed map-point cloud
        dv::DebugViewer viewer(viewer_cfg);
        viewer.loadBatch({});  // empty -> live push mode (holds window / drains queues)

        // Groundtruth (green) + aligned VO estimate (orange) trajectories.
        for (std::size_t i = 0; i < align_dst.size(); ++i) {
            dv::InferredPose gp;
            gp.x = align_dst[i].x(); gp.y = align_dst[i].y(); gp.z = align_dst[i].z();
            viewer.pushGroundtruthPose(gp);

            const Eigen::Vector3d est = (T_align * align_src[i].homogeneous()).hnormalized();
            dv::InferredPose ep;
            ep.x = est.x(); ep.y = est.y(); ep.z = est.z();
            viewer.pushPose(ep);
        }

        // Latest keyframe map-point cloud, aligned to ENU.
        std::vector<Eigen::Vector3f> pts_enu;
        pts_enu.reserve(latest_map_points.size());
        for (const Vec3_t& p : latest_map_points) {
            const Eigen::Vector3d e = (T_align * p.homogeneous()).hnormalized();
            pts_enu.emplace_back(static_cast<float>(e.x()),
                                 static_cast<float>(e.y()),
                                 static_cast<float>(e.z()));
        }
        viewer.pushMapPoints(pts_enu);
        spdlog::info("test_vo_debug_new: pushed {} trajectory + {} map points to viewer",
                     align_dst.size(), pts_enu.size());

        // The live 3D window is opt-in: viewer.run() blocks until the window is
        // closed when a display is present, which would hang ctest. Only spin it
        // up when UAVLOC_VO_VIEWER is set (manual inspection); by default the
        // headless deliverable (callback asserts + trajectory PNG) is enough.
        const char* viewer_env = std::getenv("UAVLOC_VO_VIEWER");
        if (viewer_env != nullptr && viewer_env[0] != '\0') {
            viewer.run();  // returns immediately when headless (no DISPLAY)
        } else {
            spdlog::info("test_vo_debug_new: set UAVLOC_VO_VIEWER=1 to open the live 3D viewer");
        }
    } else {
        spdlog::warn("test_vo_debug_new: too few TRACKING correspondences for alignment "
                     "— skipping viewer push");
    }
#endif

    if (rc == 0) spdlog::info("test_vo_debug_new: PASS");
    return rc;
}
