// test_vo_pipeline_new — end-to-end smoke test for the new_vo VOModule.
//
// Opens the YenBai video via sensor::VideoReader, feeds frames through
// vo::VOModule::process_frame, and asserts the pipeline reaches TRACKING,
// builds landmarks, produces finite poses, and inserts at least one keyframe.
// Headless-safe. Soft-skips (returns 0) when the (gitignored) dataset video is
// unavailable, mirroring tests/test_vo_frame_loader.cpp.
//
// Opt-in FULL-dataset eval mode (env UAVLOC_VO_FULL=1 or argv --full): removes
// the early-exit, processes every frame until END_OF_STREAM, logs progress
// every 500 frames, accumulates run statistics (TRACKING count, LOST events,
// re-inits, keyframes, max landmarks, trajectory length), renders a top-down
// world X-Y trajectory PNG (vo_full_trajectory.png), and prints a stats summary.
// FULL mode does NOT assert the final state is TRACKING (a long flight may end
// LOST/re-initialising). Default mode is unchanged (fast ctest smoke).
#include "uavloc/sensor/video_reader.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/new_vo/vo_module.h"

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {
const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m_newvo.yaml";

constexpr int MAX_FRAMES = 10000;  // hard cap on frames fed to the pipeline (smoke)

// Where the FULL-mode top-down trajectory visualization is written (cwd).
const std::string FULL_OUTPUT_IMAGE_PATH = "vo_full_trajectory.png";

// Progress is logged every this many frames in FULL mode.
constexpr int FULL_PROGRESS_EVERY = 500;

// ── Trajectory image presentation constants (rendering only — NOT pipeline
//    thresholds). Adapted from tests/test_vo_debug_new.cpp. ───────────────────
const int    CANVAS_SIZE_PX        = 1000;
const int    CANVAS_MARGIN_PX      = 80;
const int    POLYLINE_THICKNESS    = 2;
const int    MARKER_RADIUS_PX      = 7;
const int    SCALE_BAR_MARGIN_PX   = 20;
const int    SCALE_BAR_HEIGHT_PX   = 8;
const double SCALE_BAR_TARGET_FRAC = 0.25;
const double TEXT_SCALE            = 0.5;
const int    TEXT_THICKNESS        = 1;

bool pose_is_finite(const uavloc::Mat44_t& T) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(T(r, c))) return false;
        }
    }
    return true;
}

const char* state_name(uavloc::vo::VOTrackingState s) {
    switch (s) {
        case uavloc::vo::VOTrackingState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case uavloc::vo::VOTrackingState::TRACKING:        return "TRACKING";
        case uavloc::vo::VOTrackingState::LOST:            return "LOST";
    }
    return "UNKNOWN";
}

// Round a positive value down to a "nice" 1/2/5 x 10^k number.
double nice_round(double value) {
    if (value <= 0.0) return 0.0;
    const double exp10 = std::floor(std::log10(value));
    const double base  = std::pow(10.0, exp10);
    const double frac  = value / base;
    double nice = 1.0;
    if (frac >= 5.0)      nice = 5.0;
    else if (frac >= 2.0) nice = 2.0;
    return nice * base;
}

// Project world positions (X horizontal, Y vertical; Z dropped) onto a fitted
// top-down canvas and write a polyline with start/end markers and a metric scale
// bar. Mirrors the auto-fit -> toPixel -> imwrite pattern of test_vo_debug_new.
bool render_trajectory(const std::vector<Eigen::Vector3d>& positions,
                       const std::string& output_path) {
    if (positions.size() < 2) {
        spdlog::warn("Trajectory has < 2 poses; nothing to render");
        return false;
    }

    double min_x = positions.front().x(), max_x = min_x;
    double min_y = positions.front().y(), max_y = min_y;
    for (const Eigen::Vector3d& p : positions) {
        min_x = std::min(min_x, p.x());
        max_x = std::max(max_x, p.x());
        min_y = std::min(min_y, p.y());
        max_y = std::max(max_y, p.y());
    }

    const double span_x    = std::max(max_x - min_x, 1e-9);
    const double span_y    = std::max(max_y - min_y, 1e-9);
    const double draw_area = static_cast<double>(CANVAS_SIZE_PX - 2 * CANVAS_MARGIN_PX);
    const double scale     = draw_area / std::max(span_x, span_y);

    auto to_pixel = [&](const Eigen::Vector3d& p) {
        const double px = CANVAS_MARGIN_PX + (p.x() - min_x) * scale;
        const double py = CANVAS_SIZE_PX - CANVAS_MARGIN_PX - (p.y() - min_y) * scale;
        return cv::Point(static_cast<int>(px), static_cast<int>(py));
    };

    cv::Mat canvas(CANVAS_SIZE_PX, CANVAS_SIZE_PX, CV_8UC3, cv::Scalar(30, 30, 30));

    for (std::size_t i = 1; i < positions.size(); ++i) {
        cv::line(canvas, to_pixel(positions[i - 1]), to_pixel(positions[i]),
                 cv::Scalar(0, 255, 0), POLYLINE_THICKNESS, cv::LINE_AA);
    }

    cv::circle(canvas, to_pixel(positions.front()), MARKER_RADIUS_PX,
               cv::Scalar(255, 128, 0), 2, cv::LINE_AA);
    cv::putText(canvas, "start", to_pixel(positions.front()) + cv::Point(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, cv::Scalar(255, 128, 0),
                TEXT_THICKNESS, cv::LINE_AA);
    cv::circle(canvas, to_pixel(positions.back()), MARKER_RADIUS_PX,
               cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(canvas, "end", to_pixel(positions.back()) + cv::Point(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, cv::Scalar(0, 0, 255),
                TEXT_THICKNESS, cv::LINE_AA);

    const double meters_per_px = 1.0 / scale;
    const double target_m      = draw_area * SCALE_BAR_TARGET_FRAC * meters_per_px;
    const double bar_m         = nice_round(target_m);
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
    spdlog::info("Trajectory image written to '{}' ({} poses)", output_path, positions.size());
    return true;
}

// FULL mode is enabled by env UAVLOC_VO_FULL=1 (any non-empty value) or by
// passing --full anywhere on the command line.
bool full_mode_enabled(int argc, char** argv) {
    const char* env = std::getenv("UAVLOC_VO_FULL");
    if (env != nullptr && env[0] != '\0') return true;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--full") == 0) return true;
    }
    return false;
}
}  // namespace

int main(int argc, char** argv) {
    using namespace uavloc;

    // Resolve config path: first non-flag positional argument, else default.
    std::string config_path = DEFAULT_CONFIG_PATH;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--full") == 0) continue;
        config_path = std::string(argv[i]);
        break;
    }

    const bool full_mode = full_mode_enabled(argc, argv);

    // Optional FULL-mode frame cap: when UAVLOC_VO_MAXFRAMES > 0, stop the
    // processing loop after that many frames are fed and fall through to the
    // existing stats + vo_full_trajectory.png render. Unset/0 keeps the default
    // behavior (run to END_OF_STREAM). Has no effect on the smoke path.
    const char* mf = std::getenv("UAVLOC_VO_MAXFRAMES");
    const int max_frames_env = mf ? std::atoi(mf) : 0;

    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }

    sensor::VideoReaderConfig reader_cfg;
    vo::VOConfig vo_cfg;
    try {
        reader_cfg = sensor::VideoReaderConfig::fromYaml(yaml);
        vo_cfg     = vo::VOConfig::fromYaml(yaml);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }

    sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::warn("test_vo_pipeline_new: cannot open video '{}' — SKIPPED", reader_cfg.video_path);
        return 0;  // soft-skip when the dataset is absent
    }

    vo::VOModule vo_module(vo_cfg);

    spdlog::info("test_vo_pipeline_new: mode={}", full_mode ? "FULL" : "smoke");

    bool reached_tracking = false;
    bool saw_keyframe     = false;
    bool all_poses_finite = true;
    int  max_landmarks    = 0;
    int  frames_fed       = 0;

    // FULL-mode statistics.
    int    tracking_frames = 0;
    int    lost_events     = 0;   // TRACKING/NOT_INIT -> LOST transitions
    int    reinit_events   = 0;   // (later) NOT_INITIALIZED -> TRACKING after first init
    int    keyframe_count  = 0;
    double trajectory_len  = 0.0; // sum of ||T_prev_curr translation|| over has_pose frames
    bool   had_first_init  = false;
    bool   pending_reinit  = false;  // saw NOT_INITIALIZED after first init; awaiting TRACKING
    vo::VOTrackingState prev_state = vo::VOTrackingState::NOT_INITIALIZED;
    std::vector<Eigen::Vector3d> positions;  // world positions for FULL-mode PNG

    // Smoke mode is capped at MAX_FRAMES; FULL mode ignores the cap and runs to
    // END_OF_STREAM.
    int count = 0; 
    sensor::FrameData fd;
    while (full_mode || frames_fed < MAX_FRAMES) {
        auto status = reader.read(fd);
        if (status == sensor::FrameStatus::END_OF_STREAM) break;
        if (status == sensor::FrameStatus::ERROR ||
            status == sensor::FrameStatus::CAMERA_DISCONNECTED) break;
        if (status != sensor::FrameStatus::OK || !fd.valid || !fd.HasImage()) continue;

        const vo::VOResult res = vo_module.process_frame(fd);
        ++frames_fed;

        if (res.state == vo::VOTrackingState::TRACKING) reached_tracking = true;
        if (res.is_keyframe) saw_keyframe = true;
        if (res.has_pose && !pose_is_finite(res.T_wc)) all_poses_finite = false;
        if (res.num_landmarks > max_landmarks) max_landmarks = res.num_landmarks;

        if (full_mode) {
            if (res.state == vo::VOTrackingState::TRACKING) {
                ++tracking_frames;
                if (!had_first_init) {
                    had_first_init = true;
                } else if (pending_reinit) {
                    ++reinit_events;
                    pending_reinit = false;
                }
            }
            if (res.state == vo::VOTrackingState::LOST &&
                prev_state != vo::VOTrackingState::LOST) {
                ++lost_events;
            }
            if (had_first_init && res.state == vo::VOTrackingState::NOT_INITIALIZED) {
                pending_reinit = true;
            }
            if (res.is_keyframe) ++keyframe_count;
            if (res.has_pose) {
                trajectory_len += res.T_prev_curr.block<3, 1>(0, 3).norm();
                positions.push_back(res.T_wc.block<3, 1>(0, 3));
            }
            prev_state = res.state;

            if (frames_fed % FULL_PROGRESS_EVERY == 0) {
                spdlog::info("test_vo_pipeline_new[FULL]: frames_fed={} state={} num_landmarks={}",
                             frames_fed, state_name(res.state), res.num_landmarks);
            }
            // Optional cap: stop the FULL run early once enough frames are fed,
            // then fall through to the stats + PNG render below.
            if (max_frames_env > 0 && frames_fed >= max_frames_env) break;
            continue;  // FULL mode runs to END_OF_STREAM — no early exit
        }

        // Smoke mode: stop early once enough evidence is gathered.
        if (reached_tracking && saw_keyframe && max_landmarks > 0) {
            if (frames_fed > 30) break;
        }
        ++count;
        std::cout << "\rframes_fed=" << frames_fed
                  << " reached_tracking=" << reached_tracking
                  << " saw_keyframe=" << saw_keyframe
                  << " max_landmarks=" << max_landmarks
                  << std::flush;
    }
    reader.close();

    if (frames_fed == 0) {
        spdlog::warn("test_vo_pipeline_new: no valid frames in stream — SKIPPED");
        return 0;
    }

    int rc = 0;
    auto check = [&](bool ok, const std::string& what) {
        if (!ok) { spdlog::error("test_vo_pipeline_new FAIL: {}", what); rc = 1; }
    };

    if (full_mode) {
        render_trajectory(positions, FULL_OUTPUT_IMAGE_PATH);

        // FULL-mode assertions: reached TRACKING at least once, every collected
        // pose finite, and at least one keyframe. Deliberately NO final-state
        // assertion (a long flight can end LOST / re-initialising).
        check(reached_tracking, "pipeline never reached TRACKING state");
        check(saw_keyframe,     "no keyframe inserted");
        check(all_poses_finite, "a T_wc pose contained non-finite values");

        spdlog::info("==== test_vo_pipeline_new FULL stats ====");
        spdlog::info("  frames_processed  = {}", frames_fed);
        spdlog::info("  tracking_frames   = {}", tracking_frames);
        spdlog::info("  lost_events       = {}", lost_events);
        spdlog::info("  reinit_events     = {}", reinit_events);
        spdlog::info("  keyframes_seen    = {}", keyframe_count);
        spdlog::info("  max_landmarks     = {}", max_landmarks);
        spdlog::info("  trajectory_length = {:.2f} m", trajectory_len);
        spdlog::info("  poses_collected   = {}", positions.size());
        spdlog::info("  final_state       = {}", state_name(vo_module.get_state()));
        spdlog::info("=========================================");

        if (rc == 0) spdlog::info("test_vo_pipeline_new: PASS (FULL)");
        return rc;
    }

    check(reached_tracking, "pipeline never reached TRACKING state");
    check(max_landmarks > 0, "map has no landmarks");
    check(saw_keyframe, "no keyframe inserted");
    check(all_poses_finite, "a T_wc pose contained non-finite values");

    spdlog::info(
        "test_vo_pipeline_new: frames_fed={} reached_tracking={} saw_keyframe={} max_landmarks={} "
        "final_state={}",
        frames_fed, reached_tracking, saw_keyframe, max_landmarks,
        static_cast<int>(vo_module.get_state()));

    if (rc == 0) spdlog::info("test_vo_pipeline_new: PASS");
    return rc;
}
