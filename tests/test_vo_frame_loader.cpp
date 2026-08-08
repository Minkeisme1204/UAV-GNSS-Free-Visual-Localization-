// test_vo_frame_loader — load ONE real frame via sensor::VideoReader, run it
// through vo::VoFrameLoader (ORB extract → undistort → bearings → grid → frame),
// and assert the resulting data::Frame is well-formed. Headless-safe. If the
// dataset video is unavailable, the test soft-skips (returns 0) so CI stays green.
#include "uavloc/sensor/video_reader.h"
#include "uavloc/sensor/camera_model.h"
#include "uavloc/new_vo/camera/perspective_camera.h"
#include "uavloc/new_vo/feature/orb/orb_params.h"
#include "uavloc/new_vo/module/vo_frame_loader.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <string>

#ifndef UAVLOC_MISSION_CONFIG_PATH
#define UAVLOC_MISSION_CONFIG_PATH "config/uavloc_yenbai800m_newvo.yaml"   // fallback; CMake injects the real path
#endif

namespace {
const std::string DEFAULT_CONFIG_PATH = UAVLOC_MISSION_CONFIG_PATH;
}  // namespace

int main(int argc, char** argv) {
    using namespace uavloc;

    const std::string config_path = (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }

    sensor::VideoReaderConfig reader_cfg;
    sensor::CameraIntrinsics intrinsics;
    try {
        reader_cfg = sensor::VideoReaderConfig::fromYaml(yaml);
        intrinsics = sensor::CameraIntrinsics::fromYaml(yaml);  // reads node["Camera"]
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }

    // ── Open the video and grab one valid frame ───────────────────────────────
    sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::warn("test_vo_frame_loader: cannot open video '{}' — SKIPPED",
                     reader_cfg.video_path);
        return 0;  // soft-skip when the (gitignored) dataset is absent
    }

    sensor::FrameData fd;
    bool got = false;
    while (true) {
        auto status = reader.read(fd);
        if (status == sensor::FrameStatus::END_OF_STREAM) break;
        if (status == sensor::FrameStatus::OK && fd.valid && fd.HasImage()) { got = true; break; }
        if (status == sensor::FrameStatus::ERROR ||
            status == sensor::FrameStatus::CAMERA_DISCONNECTED) break;
    }
    reader.close();

    if (!got) {
        spdlog::warn("test_vo_frame_loader: no valid frame in stream — SKIPPED");
        return 0;
    }

    // ── Build the loader and load the frame ───────────────────────────────────
    sensor::CameraModel cam_model(intrinsics);
    vo::camera::PerspectiveCamera camera(cam_model);
    vo::feature::OrbParams orb_params("orb", 1.2f, 8, 20, 7);

    vo::VoFrameLoader loader(&camera, &orb_params);
    vo::data::Frame frm = loader.load(fd);

    // ── Assertions ────────────────────────────────────────────────────────────
    const std::size_t num_kpts = frm.frm_obs_.undist_keypts_.size();
    int rc = 0;
    auto check = [&](bool ok, const std::string& what) {
        if (!ok) { spdlog::error("test_vo_frame_loader FAIL: {}", what); rc = 1; }
    };

    check(num_kpts > 0, "no keypoints extracted");
    check(static_cast<std::size_t>(frm.frm_obs_.descriptors_.rows) == num_kpts,
          "descriptor rows != keypoint count");
    check(frm.frm_obs_.bearings_.size() == num_kpts, "bearings size != keypoint count");
    check(frm.id_ == static_cast<unsigned int>(fd.frame_id), "frame id mismatch");
    check(frm.timestamp_ == fd.timestamp_msec, "timestamp mismatch");
    check(!frm.frm_obs_.keypt_indices_in_cells_.empty(), "keypoint grid not built");

    spdlog::info(
        "test_vo_frame_loader: frame_id={} t={:.0f}ms keypoints={} descriptors={}x{} bearings={}",
        frm.id_, frm.timestamp_, num_kpts, frm.frm_obs_.descriptors_.rows,
        frm.frm_obs_.descriptors_.cols, frm.frm_obs_.bearings_.size());

    if (rc == 0) spdlog::info("test_vo_frame_loader: PASS");
    return rc;
}
