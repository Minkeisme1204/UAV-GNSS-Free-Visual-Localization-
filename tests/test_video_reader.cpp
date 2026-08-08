// test_video_reader — gated acceptance test for sensor::VideoReader.
//
// Reads a capped prefix of the mission video and asserts the stream contract:
// stream metadata is sane, every OK frame carries a non-empty image of a
// constant size, frame_id strictly increases, timestamp_msec never goes
// backwards, and — when the config declares a telemetry source — at least one
// frame arrives with telemetry attached.
//
// Headless-safe: highgui is only touched when a display server is present AND
// UAVLOC_VR_SHOW != "0" (same convention as test_ts_reader). Soft-skips
// (returns 0) when the gitignored dataset video is absent.
//
// Usage (from build/):
//   ./tests/test_video_reader [config.yaml]
// Env:
//   UAVLOC_VR_MAXFRAMES  cap on OK frames read. Default: 100.
//   UAVLOC_VR_SHOW=0     force headless even with a display.

#include "uavloc/sensor/video_reader.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdlib>
#include <string>

#ifndef UAVLOC_MISSION_CONFIG_PATH
#define UAVLOC_MISSION_CONFIG_PATH "config/uavloc_yenbai800m_newvo.yaml"   // fallback; CMake injects the real path
#endif

namespace {

const std::string DEFAULT_CONFIG_PATH = UAVLOC_MISSION_CONFIG_PATH;

// Frame budget of the smoke run; overridable via UAVLOC_VR_MAXFRAMES.
constexpr int DEFAULT_MAX_FRAMES = 100;

// HUD text placement/appearance — display only, no effect on the gates.
constexpr double HUD_FONT_SCALE = 0.55;
constexpr int    HUD_LINE_1_Y   = 24;
constexpr int    HUD_LINE_2_Y   = 48;
constexpr int    HUD_X          = 8;

int envInt(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    return std::atoi(v);
}

}  // namespace

int main(int argc, char** argv) {
    using namespace uavloc;

    spdlog::set_level(spdlog::level::info);

    const std::string config_path = (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    // ── Load config ───────────────────────────────────────────────────────────
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }

    sensor::VideoReaderConfig cfg;
    try {
        cfg = sensor::VideoReaderConfig::fromYaml(yaml);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse VideoReaderConfig: {}", e.what());
        return 1;
    }

    spdlog::info("Config loaded from '{}'", config_path);
    spdlog::info("  video_path          : {}", cfg.video_path);
    spdlog::info("  start_frame         : {}", cfg.start_frame);
    spdlog::info("  end_frame           : {}", cfg.end_frame);
    spdlog::info("  convert_to_grayscale: {}", cfg.convert_to_grayscale);
    spdlog::info("  use_video_timestamp : {}", cfg.use_video_timestamp);

    // A telemetry source is configured when either CSV path is non-empty.
    const bool telemetry_expected =
        !cfg.drone_telemetry.csv_path.empty() || !cfg.telemetry_csv_path.empty();

    // ── Open reader (soft-skip when the gitignored dataset is absent) ────────
    sensor::VideoReader reader(cfg);
    if (!reader.open()) {
        spdlog::warn("test_video_reader: cannot open video '{}' — SKIPPED", cfg.video_path);
        return 0;
    }

    int rc = 0;
    auto check = [&](bool ok, const std::string& what) {
        if (!ok) { spdlog::error("test_video_reader FAIL: {}", what); rc = 1; }
    };

    check(reader.getFps() > 0.0, "getFps() is not positive");
    check(reader.getFrameCount() > 0, "getFrameCount() is not positive");

    spdlog::info("Video opened — fps: {:.2f}, total frames: {}",
                 reader.getFps(), reader.getFrameCount());

    // ── Display gate: only with a display server AND UAVLOC_VR_SHOW != "0" ───
    const char* display_env = std::getenv("DISPLAY");
    const char* wayland_env = std::getenv("WAYLAND_DISPLAY");
    const bool  has_display = (display_env && *display_env) ||
                              (wayland_env && *wayland_env);
    const char* show_env    = std::getenv("UAVLOC_VR_SHOW");
    const bool  show        = has_display &&
                              !(show_env && std::string(show_env) == "0");
    if (!show)
        spdlog::info("running headless (no display or UAVLOC_VR_SHOW=0) — no highgui");

    const std::string win = "test_video_reader";
    if (show) cv::namedWindow(win, cv::WINDOW_AUTOSIZE);

    const int max_frames = envInt("UAVLOC_VR_MAXFRAMES", DEFAULT_MAX_FRAMES);
    const int wait_ms    = std::max(1, static_cast<int>(1000.0 / std::max(reader.getFps(), 1.0)));

    // ── Read loop ─────────────────────────────────────────────────────────────
    sensor::FrameData frame;
    int      ok_frames        = 0;
    int      telemetry_frames = 0;
    bool     first            = true;
    cv::Size ref_size;
    uint64_t prev_frame_id    = 0;
    double   prev_ts_msec     = 0.0;
    bool     size_constant    = true;
    bool     ids_increasing   = true;
    bool     ts_monotonic     = true;
    bool     frames_valid     = true;

    while (ok_frames < max_frames) {
        const auto status = reader.read(frame);

        if (status == sensor::FrameStatus::END_OF_STREAM) {
            spdlog::info("End of stream after {} OK frames", ok_frames);
            break;
        }
        if (status == sensor::FrameStatus::ERROR ||
            status == sensor::FrameStatus::CAMERA_DISCONNECTED) {
            spdlog::error("reader.read() returned status={}", static_cast<int>(status));
            rc = 1;
            break;
        }
        if (status != sensor::FrameStatus::OK) {
            spdlog::warn("skipping frame — status={}", static_cast<int>(status));
            continue;
        }

        // ── Per-frame gates ──────────────────────────────────────────────────
        if (!frame.valid || !frame.HasImage()) frames_valid = false;

        if (frame.HasImage()) {
            const cv::Size sz = frame.image.size();
            if (first) ref_size = sz;
            else if (sz != ref_size) size_constant = false;
        }

        if (!first) {
            if (frame.frame_id <= prev_frame_id)        ids_increasing = false;
            if (frame.timestamp_msec < prev_ts_msec)    ts_monotonic   = false;
        }
        prev_frame_id = frame.frame_id;
        prev_ts_msec  = frame.timestamp_msec;
        first         = false;

        if (frame.has_telemetry) ++telemetry_frames;
        ++ok_frames;

        // ── Optional display ─────────────────────────────────────────────────
        if (show) {
            cv::Mat display = frame.image.clone();
            if (display.channels() == 1)
                cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);

            const std::string hud =
                "frame=" + std::to_string(frame.frame_id)
                + "  t=" + std::to_string(static_cast<int>(frame.timestamp_msec)) + "ms"
                + "  " + std::to_string(frame.image.cols) + "x" + std::to_string(frame.image.rows);
            cv::putText(display, hud, {HUD_X, HUD_LINE_1_Y},
                        cv::FONT_HERSHEY_SIMPLEX, HUD_FONT_SCALE, {0, 255, 0}, 1, cv::LINE_AA);

            if (frame.has_telemetry) {
                const auto& t = frame.telemetry;
                const std::string tel =
                    "hdg=" + std::to_string(static_cast<int>(t.heading_deg)) + "deg"
                    + "  alt=" + std::to_string(static_cast<int>(t.altitude_m)) + "m"
                    + "  spd=" + std::to_string(static_cast<int>(t.speed_mps)) + "m/s";
                cv::putText(display, tel, {HUD_X, HUD_LINE_2_Y},
                            cv::FONT_HERSHEY_SIMPLEX, HUD_FONT_SCALE, {0, 220, 255}, 1, cv::LINE_AA);
            }

            cv::imshow(win, display);
            const int key = cv::waitKey(wait_ms);
            if (key == 'q' || key == 27) break;
        }
    }

    reader.close();
    if (show) cv::destroyAllWindows();

    // ── Acceptance gates ──────────────────────────────────────────────────────
    check(ok_frames > 0, "no frame read OK");
    check(frames_valid, "an OK frame was not valid or had an empty image");
    check(size_constant, "image size changed across the run");
    check(ids_increasing, "frame_id not strictly increasing");
    check(ts_monotonic, "timestamp_msec decreased between frames");
    if (telemetry_expected)
        check(telemetry_frames > 0,
              "config declares a telemetry source but no frame carried telemetry");

    spdlog::info("test_video_reader: {} OK frames, {}x{}, telemetry on {} frames "
                 "(telemetry configured: {})",
                 ok_frames, ref_size.width, ref_size.height, telemetry_frames,
                 telemetry_expected);

    if (rc == 0) spdlog::info("test_video_reader: PASS");
    return rc;
}
