// test_vo_debug — compact per-frame diagnostic driver for the VO pipeline.
//
// Feeds the first DEBUG_MAX_FRAMES valid frames of the mission video through the
// public uavloc::vo::VOModule (feed a frame, get a pose) and logs a full,
// human-readable block for each frame: identifiers, tracking state, pose
// availability, keyframe flag, feature/match/inlier/landmark counts, and the
// GLOBAL pose translation (T_wc) plus the world-frame increment (T_prev_curr).
//
// In addition to the per-frame log it renders a top-down trajectory image from
// the poses gathered over the run (reusing the fit/project/imwrite pattern of
// test_vo_pipeline): each has_pose frame is plotted from its T_wc translation
// using ONLY the X and Y components (altitude / Z is dropped), consecutive
// points are joined into a polyline, and a small heading arrow is drawn every
// few poses. Headless-safe, synchronous (processFrame), no GUI.
//
// Usage (from build/):
//     ./tests/test_vo_debug [config.yaml] [max_frames] [trajectory.png]
// Defaults: config/uavloc_yenbai800m.yaml, DEBUG_MAX_FRAMES frames,
//           vo_debug_trajectory.png.
#include "uavloc/sensor/camera_model.h"
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
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace uavloc {
namespace {

// Default mission config when none is supplied on the command line (matches
// test_vo_pipeline so both drivers share one config).
const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m.yaml";

// Number of valid frames to process before stopping. Overridable via argv[2].
constexpr int DEBUG_MAX_FRAMES = 20;

// Maximum consecutive EMPTY_FRAME results tolerated before the reader is treated
// as exhausted (some codecs over-report CAP_PROP_FRAME_COUNT).
constexpr int MAX_CONSECUTIVE_EMPTY_FRAMES = 30;

// Where the top-down trajectory visualization is written (current working
// directory, mirroring how test_vo_pipeline writes vo_trajectory.png).
const std::string OUTPUT_IMAGE_PATH = "vo_debug_trajectory.png";

// ── Trajectory image presentation constants (rendering only — NOT pipeline
//    thresholds). Kept here so no magic numbers leak into the draw code. ───────
const int    CANVAS_SIZE_PX     = 1000;  // square canvas side length
const int    CANVAS_MARGIN_PX   = 80;    // border kept clear of the fitted path
const int    POLYLINE_THICKNESS = 2;
const int    MARKER_RADIUS_PX   = 7;     // start/end marker radius
const int    HEADING_EVERY_N    = 2;     // draw a heading arrow every N poses
const int    HEADING_LEN_PX     = 26;    // fixed on-screen heading arrow length
const int    HEADING_THICKNESS  = 1;
const double HEADING_TIP_LENGTH = 0.35;  // cv::arrowedLine tip fraction
const int    SCALE_BAR_MARGIN_PX = 20;   // scale-bar inset from the canvas edge
const int    SCALE_BAR_HEIGHT_PX = 8;    // scale-bar tick height
const double SCALE_BAR_TARGET_FRAC = 0.25;  // target bar length ~1/4 of draw area
const double TEXT_SCALE          = 0.5;
const int    TEXT_THICKNESS      = 1;
const double HEADING_EPS         = 1e-9;  // below this the heading is undefined

// One plotted pose sample: the GLOBAL camera position and a heading direction on
// the ground plane. Only X/Y of the position are ever used (Z / altitude is
// intentionally dropped for the top-down view).
struct PoseSample {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // Heading on the world XY plane. Derived from T_wc rotation: the camera's
    // image-up axis (camera -Y) expressed in the world frame, projected to XY.
    // For this near-nadir setup the world XY plane IS the ground plane (the VO
    // world frame is anchored to the first, downward-looking camera), so this is
    // the UAV/camera heading over ground. Zero vector when it is degenerate.
    Eigen::Vector2d heading = Eigen::Vector2d::Zero();
};

// Round a positive value down to a "nice" 1/2/5 x 10^k number, for a readable
// metric scale bar (e.g. 137 m -> 100 m, 620 m -> 500 m).
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
// world Y vertical; Z / altitude dropped) and write a fitted polyline with
// start/end markers, per-pose heading arrows and a metric scale bar. Mirrors the
// auto-fit + toPixel + imwrite pattern of test_vo_pipeline::renderTrajectory.
bool renderTrajectory(const std::vector<PoseSample>& poses,
                      const std::string& output_path) {
    if (poses.size() < 2) {
        spdlog::warn("Trajectory has < 2 poses; nothing to render");
        return false;
    }

    // Auto-fit: bounds over the plotted (X, Y) coordinates.
    double min_x = poses.front().position.x(), max_x = min_x;
    double min_y = poses.front().position.y(), max_y = min_y;
    for (const PoseSample& s : poses) {
        min_x = std::min(min_x, s.position.x());
        max_x = std::max(max_x, s.position.x());
        min_y = std::min(min_y, s.position.y());
        max_y = std::max(max_y, s.position.y());
    }

    const double span_x   = std::max(max_x - min_x, 1e-9);
    const double span_y   = std::max(max_y - min_y, 1e-9);
    const double draw_area = static_cast<double>(CANVAS_SIZE_PX - 2 * CANVAS_MARGIN_PX);
    // Uniform scale preserves the trajectory shape (1:1 aspect ratio, no skew).
    const double scale = draw_area / std::max(span_x, span_y);

    // Map a world (X, Y) to a canvas pixel; flip Y so +Y points up on screen.
    auto toPixel = [&](const Eigen::Vector3d& p) {
        const double px = CANVAS_MARGIN_PX + (p.x() - min_x) * scale;
        const double py = CANVAS_SIZE_PX - CANVAS_MARGIN_PX - (p.y() - min_y) * scale;
        return cv::Point(static_cast<int>(px), static_cast<int>(py));
    };

    cv::Mat canvas(CANVAS_SIZE_PX, CANVAS_SIZE_PX, CV_8UC3, cv::Scalar(30, 30, 30));

    // Trajectory polyline (green).
    for (std::size_t i = 1; i < poses.size(); ++i) {
        cv::line(canvas, toPixel(poses[i - 1].position), toPixel(poses[i].position),
                 cv::Scalar(0, 255, 0), POLYLINE_THICKNESS, cv::LINE_AA);
    }

    // Per-pose heading arrows (cyan), drawn with a fixed pixel length so they stay
    // legible regardless of the fitted scale. +Y up on screen means the screen dy
    // is the negated world-Y component.
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

    // Start marker: blue ring (hollow) to stand out from the filled end marker.
    cv::circle(canvas, toPixel(poses.front().position), MARKER_RADIUS_PX,
               cv::Scalar(255, 128, 0), 2, cv::LINE_AA);
    cv::circle(canvas, toPixel(poses.front().position), 2,
               cv::Scalar(255, 128, 0), cv::FILLED, cv::LINE_AA);
    cv::putText(canvas, "start", toPixel(poses.front().position) + cv::Point(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, cv::Scalar(255, 128, 0),
                TEXT_THICKNESS, cv::LINE_AA);
    // End marker: filled red dot.
    cv::circle(canvas, toPixel(poses.back().position), MARKER_RADIUS_PX,
               cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(canvas, "end", toPixel(poses.back().position) + cv::Point(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, cv::Scalar(0, 0, 255),
                TEXT_THICKNESS, cv::LINE_AA);

    // Metric scale bar (bottom-left). Translation here is METRIC (VOModule
    // promotes monocular scale from telemetry altitude), so distances are metres.
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

    // Header caption: what the axes mean.
    cv::putText(canvas, "top-down VO trajectory (world X-Y, altitude dropped)",
                cv::Point(SCALE_BAR_MARGIN_PX, 30), cv::FONT_HERSHEY_SIMPLEX,
                TEXT_SCALE, cv::Scalar(200, 200, 200), TEXT_THICKNESS, cv::LINE_AA);

    if (!cv::imwrite(output_path, canvas)) {
        spdlog::error("Failed to write trajectory image to '{}'", output_path);
        return false;
    }
    spdlog::info("Trajectory image written to '{}' ({} poses)", output_path,
                 poses.size());
    return true;
}

// Human-readable name for a VO tracking state (for the per-frame log block).
const char* stateName(vo::VOTrackingState s) {
    switch (s) {
        case vo::VOTrackingState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case vo::VOTrackingState::INITIALIZED:     return "INITIALIZED";
        case vo::VOTrackingState::TRACKING:        return "TRACKING";
        case vo::VOTrackingState::LOST:            return "LOST";
        case vo::VOTrackingState::FAILED:          return "FAILED";
    }
    return "UNKNOWN";
}

// Reads the next valid (OK + non-empty) frame from the reader into `out`.
// Returns false at end-of-stream, on unrecoverable error, or when too many
// consecutive empty frames indicate the stream is effectively exhausted.
// Mirrors the reader loop in test_vo_pipeline.
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

// Log one clearly delimited block for a single processed frame.
void logFrameResult(int index, const sensor::FrameData& frame,
                    const vo::VOResult& r) {
    const Eigen::Vector3d t_wc   = r.T_wc.block<3, 1>(0, 3);
    const Eigen::Vector3d t_incr = r.T_prev_curr.block<3, 1>(0, 3);

    spdlog::info("──── frame #{} (id {}) ─────────────────────────────────",
                 index, frame.frame_id);
    spdlog::info("  timestamp_msec : {:.3f}", r.timestamp_msec);
    spdlog::info("  state          : {}", stateName(r.state));
    spdlog::info("  has_pose       : {}   is_keyframe : {}",
                 r.has_pose, r.is_keyframe);
    spdlog::info("  keypoints      : {}   matches : {}   inliers : {}",
                 r.num_keypoints, r.num_matches, r.num_inliers);
    spdlog::info("  landmarks      : {}   inlier_ratio : {:.4f}",
                 r.num_landmarks, r.inlier_ratio);
    spdlog::info("  tracked_obs    : {}", r.tracked_observations.size());
    spdlog::info("  T_wc trans     : [{:+.4f}, {:+.4f}, {:+.4f}]",
                 t_wc.x(), t_wc.y(), t_wc.z());
    spdlog::info("  T_prev_curr t  : [{:+.4f}, {:+.4f}, {:+.4f}]",
                 t_incr.x(), t_incr.y(), t_incr.z());
    spdlog::info("  telemetry      : {} (alt {:.1f} m)",
                 frame.has_telemetry, frame.telemetry.altitude_m);
}

}  // namespace
}  // namespace uavloc

int main(int argc, char** argv) {
    using namespace uavloc;

    // Honour SPDLOG_LEVEL so a run can raise/lower verbosity.
    spdlog::cfg::load_env_levels();

    const std::string config_path =
        (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;
    const int max_frames =
        (argc > 2) ? std::max(1, std::atoi(argv[2])) : DEBUG_MAX_FRAMES;
    const std::string output_path =
        (argc > 3) ? std::string(argv[3]) : OUTPUT_IMAGE_PATH;

    // ── Load config ────────────────────────────────────────────────────────────
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }
    spdlog::info("Config loaded from '{}'", config_path);
    spdlog::info("Processing the first {} valid frames", max_frames);

    // ── Parse configs (sensor / camera / VO) ───────────────────────────────────
    sensor::VideoReaderConfig reader_cfg;
    sensor::CameraIntrinsics  intrinsics;
    vo::VOConfig              vo_cfg;
    try {
        reader_cfg = sensor::VideoReaderConfig::fromYaml(yaml);
        intrinsics = sensor::CameraIntrinsics::fromYaml(yaml);
        vo_cfg     = vo::VOConfig::fromYaml(yaml["VO"]);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }

    // ── Open video ─────────────────────────────────────────────────────────────
    sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::error("Failed to open video: '{}'", reader_cfg.video_path);
        return 1;
    }
    spdlog::info("Video opened — fps: {:.1f}, total frames: {}",
                 reader.getFps(), reader.getFrameCount());

    // ── Camera model ───────────────────────────────────────────────────────────
    sensor::CameraModel camera(intrinsics);
    if (!camera.isValid()) {
        spdlog::error("Invalid camera intrinsics (fx/fy/width/height must be > 0)");
        return 1;
    }

    // ── VO module (owns the whole pipeline) ────────────────────────────────────
    auto vo = std::make_unique<vo::VOModule>(vo_cfg, camera);

    // ── Per-frame loop ─────────────────────────────────────────────────────────
    int frames_processed = 0;
    int pose_successes   = 0;
    int num_keyframes    = 0;
    int lost_frames      = 0;

    // Poses gathered for the top-down trajectory image (has_pose frames only).
    std::vector<PoseSample> poses;

    sensor::FrameData frame;
    while (frames_processed < max_frames &&
           readNextValidFrame(reader, frame)) {
        const vo::VOResult r = vo->processFrame(frame);
        ++frames_processed;

        logFrameResult(frames_processed, frame, r);

        if (r.has_pose) {
            ++pose_successes;
            // Collect the GLOBAL position and a ground-plane heading for plotting.
            // Heading = camera image-up axis (camera -Y) in the world frame,
            // projected onto the XY plane (see PoseSample docs).
            PoseSample s;
            s.position = r.T_wc.block<3, 1>(0, 3);
            const Eigen::Vector2d h = (-r.T_wc.block<3, 1>(0, 1)).head<2>();
            if (h.norm() >= HEADING_EPS) s.heading = h;
            poses.push_back(s);
        }
        if (r.is_keyframe)                           ++num_keyframes;
        if (r.state == vo::VOTrackingState::LOST)    ++lost_frames;
    }
    reader.close();

    // ── Top-down trajectory image (X/Y only, altitude dropped) ─────────────────
    renderTrajectory(poses, output_path);

    // ── Summary ────────────────────────────────────────────────────────────────
    spdlog::info("════════════════════ SUMMARY ════════════════════");
    spdlog::info("  frames processed : {}", frames_processed);
    spdlog::info("  successful poses : {}", pose_successes);
    spdlog::info("  keyframes        : {}", num_keyframes);
    spdlog::info("  LOST frames      : {}", lost_frames);
    spdlog::info("  landmarks (final): {}", vo->numLandmarks());
    spdlog::info("  final state      : {}", stateName(vo->state()));

    return 0;
}
