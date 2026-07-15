#include "uavloc/sensor/video_reader.h"
#include <spdlog/spdlog.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

static const std::string CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m.yaml";

int main() {
    // ── Load config ───────────────────────────────────────────────────────────
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(CONFIG_PATH);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", CONFIG_PATH, e.what());
        return 1;
    }

    uavloc::sensor::VideoReaderConfig cfg;
    try {
        cfg = uavloc::sensor::VideoReaderConfig::fromYaml(yaml);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse VideoReaderConfig: {}", e.what());
        return 1;
    }

    spdlog::info("Config loaded from '{}'", CONFIG_PATH);
    spdlog::info("  video_path          : {}", cfg.video_path);
    spdlog::info("  start_frame         : {}", cfg.start_frame);
    spdlog::info("  end_frame           : {}", cfg.end_frame);
    spdlog::info("  loop                : {}", cfg.loop);
    spdlog::info("  convert_to_grayscale: {}", cfg.convert_to_grayscale);
    spdlog::info("  use_video_timestamp : {}", cfg.use_video_timestamp);
    spdlog::info("  camera_id           : {}", cfg.camera_id);
    spdlog::info("  source_name         : {}", cfg.source_name);

    // ── Open reader ───────────────────────────────────────────────────────────
    uavloc::sensor::VideoReader reader(cfg);

    if (!reader.open()) {
        spdlog::error("Failed to open video: '{}'", cfg.video_path);
        return 1;
    }

    spdlog::info("Video opened — fps: {:.1f}, total frames: {}",
        reader.getFps(), reader.getFrameCount());

    // ── Playback loop ─────────────────────────────────────────────────────────
    const std::string win = "test_video_reader";
    cv::namedWindow(win, cv::WINDOW_AUTOSIZE);

    uavloc::sensor::FrameData frame;
    int displayed = 0;

    while (true) {
        auto status = reader.read(frame);

        if (status == uavloc::sensor::FrameStatus::END_OF_STREAM) {
            spdlog::info("End of stream after {} frames", displayed);
            break;
        }

        if (status != uavloc::sensor::FrameStatus::OK || !frame.valid) {
            spdlog::warn("Skipping frame — status={}", static_cast<int>(status));
            continue;
        }

        cv::Mat display = frame.image.clone();
        if (display.channels() == 1)
            cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);

        std::string hud = "frame=" + std::to_string(frame.frame_id)
            + "  t=" + std::to_string(static_cast<int>(frame.timestamp_msec)) + "ms"
            + "  " + std::to_string(frame.image.cols) + "x" + std::to_string(frame.image.rows);

        cv::putText(display, hud, {8, 24},
            cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 0}, 1, cv::LINE_AA);

        if (frame.has_telemetry) {
            const auto& t = frame.telemetry;
            std::string tel =
                "hdg=" + std::to_string(static_cast<int>(t.heading_deg)) + "deg"
                + "  alt=" + std::to_string(static_cast<int>(t.altitude_m)) + "m"
                + "  spd=" + std::to_string(static_cast<int>(t.speed_mps)) + "m/s"
                + "  pitch=" + std::to_string(static_cast<int>(t.pitch_deg)) + "deg"
                + "  roll=" + std::to_string(static_cast<int>(t.roll_deg)) + "deg";
            cv::putText(display, tel, {8, 48},
                cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 220, 255}, 1, cv::LINE_AA);
        }

        cv::imshow(win, display);
        ++displayed;

        int wait_ms = static_cast<int>(1000.0 / std::max(reader.getFps(), 1.0));
        int key = cv::waitKey(wait_ms);
        if (key == 'q' || key == 27) break;
    }

    reader.close();
    cv::destroyAllWindows();

    spdlog::info("Done — displayed {} frames", displayed);
    return 0;
}
