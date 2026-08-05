// test_adjacent_matching — load two adjacent video frames, extract ORB
// features on each, match them, report statistics, and write a side-by-side
// match visualization to disk. Headless-safe (no GUI window).
#include "uavloc/sensor/video_reader.h"
#include "uavloc/vo/feature_detector.h"
#include "uavloc/vo/tracking.h"

#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono> 
#include <string>

namespace uavloc {
namespace {

// Default mission config when none is supplied on the command line. Mirrors the
// CONFIG_PATH convention used by tests/test_video_reader.cpp.
const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m_newvo.yaml";

// Where the match visualization is written (current working directory).
const std::string OUTPUT_IMAGE_PATH = "adjacent_matches.png";

// Reads the next valid (OK + non-empty) frame from the reader into `out`.
// Returns false at end-of-stream or on unrecoverable error.
bool readNextValidFrame(sensor::VideoReader& reader, sensor::FrameData& out) {
    while (true) {
        auto status = reader.read(out);

        if (status == sensor::FrameStatus::END_OF_STREAM) {
            return false;
        }
        if (status == sensor::FrameStatus::OK && out.valid && out.HasImage()) {
            return true;
        }
        spdlog::warn("Skipping frame — status={}", static_cast<int>(status));
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

    const std::string config_path =
        (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    // ── Load config ───────────────────────────────────────────────────────────
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }
    spdlog::info("Config loaded from '{}'", config_path);

    sensor::VideoReaderConfig reader_cfg;
    vo::FeatureDetectorConfig detector_cfg;
    vo::ProjectionMatcherConfig matcher_cfg;
    try {
        reader_cfg   = sensor::VideoReaderConfig::fromYaml(yaml);
        detector_cfg = vo::FeatureDetectorConfig::fromYaml(yaml["VO"]);
        matcher_cfg  = vo::ProjectionMatcherConfig::fromYaml(yaml["VO"]);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }

    // ── Open video ────────────────────────────────────────────────────────────
    sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::error("Failed to open video: '{}'", reader_cfg.video_path);
        return 1;
        
    }
    spdlog::info("Video opened — fps: {:.1f}, total frames: {}",
                 reader.getFps(), reader.getFrameCount());

    // ── Grab two adjacent valid frames ────────────────────────────────────────
    sensor::FrameData frame_prev;
    sensor::FrameData frame_curr;

    if (!readNextValidFrame(reader, frame_prev)) {
        spdlog::error("No valid frames available in the video stream");
        reader.close();
        return 1;
    }
    if (!readNextValidFrame(reader, frame_curr)) {
        spdlog::error("Only one valid frame available — need two adjacent frames");
        reader.close();
        return 1;
    }
    reader.close();

    spdlog::info("Captured adjacent frames: prev frame_id={} t={:.0f}ms, "
                 "curr frame_id={} t={:.0f}ms",
                 frame_prev.frame_id, frame_prev.timestamp_msec,
                 frame_curr.frame_id, frame_curr.timestamp_msec);

    // ── Feature extraction ────────────────────────────────────────────────────
    auto detector = vo::createFeatureDetector(detector_cfg);
    spdlog::info("Feature detector backend: {}", detector->name());

    vo::FeatureSet feat_prev;
    vo::FeatureSet feat_curr;



    auto st_prev = detector->detect(frame_prev, feat_prev);
    auto st_curr = detector->detect(frame_curr, feat_curr);

    spdlog::info("Frame {} keypoints: {}", frame_prev.frame_id, feat_prev.keypoints.size());
    spdlog::info("Frame {} keypoints: {}", frame_curr.frame_id, feat_curr.keypoints.size());

    if (st_prev != vo::FeatureStatus::OK || !feat_prev.hasDescriptors()) {
        spdlog::error("Feature extraction failed on prev frame — status={}",
                      static_cast<int>(st_prev));
        return 1;
    }
    if (st_curr != vo::FeatureStatus::OK || !feat_curr.hasDescriptors()) {
        spdlog::error("Feature extraction failed on curr frame — status={}",
                      static_cast<int>(st_curr));
        return 1;
    }

    // ── Matching ──────────────────────────────────────────────────────────────
    vo::ProjectionMatcher matcher(matcher_cfg);
    vo::MatchesData matches;
    auto match_status = matcher.match(feat_prev, feat_curr, matches, nullptr);

    if (match_status != vo::MatchStatus::OK) {
        spdlog::error("Matching failed — status={}, num_matches={}",
                      static_cast<int>(match_status), matches.num_matches);
        return 1;
    }

    const std::size_t min_kp =
        std::min(feat_prev.keypoints.size(), feat_curr.keypoints.size());
    const double match_ratio =
        (min_kp > 0) ? static_cast<double>(matches.num_matches) / static_cast<double>(min_kp)
                     : 0.0;
    spdlog::info("Matches: {} (ratio vs. smaller keypoint set: {:.3f})",
                 matches.num_matches, match_ratio);

    // ── Visualization ─────────────────────────────────────────────────────────
    cv::Mat img_prev = frame_prev.image;
    cv::Mat img_curr = frame_curr.image;
    if (img_prev.channels() == 1) cv::cvtColor(img_prev, img_prev, cv::COLOR_GRAY2BGR);
    if (img_curr.channels() == 1) cv::cvtColor(img_curr, img_curr, cv::COLOR_GRAY2BGR);

    cv::Mat vis;
    cv::drawMatches(img_prev, feat_prev.keypoints,
                    img_curr, feat_curr.keypoints,
                    matches.matches, vis,
                    cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    if (!cv::imwrite(OUTPUT_IMAGE_PATH, vis)) {
        spdlog::error("Failed to write visualization to '{}'", OUTPUT_IMAGE_PATH);
        return 1;
    }
    spdlog::info("Match visualization written to '{}'", OUTPUT_IMAGE_PATH);

    spdlog::info("Done — {} matches between frame {} and frame {}",
                 matches.num_matches, frame_prev.frame_id, frame_curr.frame_id);
    return 0;
}
