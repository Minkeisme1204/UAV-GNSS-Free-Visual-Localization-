// test_fusion_offline — F1 offline evaluation driver for the fusion back-end.
//
// Runs the REAL pipeline on a mission config: sensor::VideoReader →
// vo::VOModule::process_frame (synchronous) → fusion::FusionModule::push
// (forced synchronous — deterministic offline evaluation) and dumps a
// per-frame CSV for offline error analysis (fused Z vs telemetry AGL, scale
// evolution, keyframe cadence).
//
// Usage:   ./tests/test_fusion_offline [config.yaml] [out.csv]
// Env:     UAVLOC_VO_MAXFRAMES caps the number of frames fed (0/unset = run
//          to END_OF_STREAM).
//
// Mirrors tests/test_vo_pipeline_new.cpp (soft-skip when the gitignored
// dataset video is absent; same lost/reinit accounting). Headless, no viewer.

#include "uavloc/sensor/video_reader.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/fusion/fusion_module.h"

#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

namespace {
const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m_newvo.yaml";
const std::string DEFAULT_OUTPUT_CSV = "fusion_offline.csv";

constexpr int CSV_PRECISION = 12;

const char* state_name(uavloc::vo::VOTrackingState s) {
    switch (s) {
        case uavloc::vo::VOTrackingState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case uavloc::vo::VOTrackingState::TRACKING:        return "TRACKING";
        case uavloc::vo::VOTrackingState::LOST:            return "LOST";
    }
    return "UNKNOWN";
}
}  // namespace

int main(int argc, char** argv) {
    using namespace uavloc;

    const std::string config_path = (argc > 1) ? argv[1] : DEFAULT_CONFIG_PATH;
    const std::string output_csv  = (argc > 2) ? argv[2] : DEFAULT_OUTPUT_CSV;

    // Optional frame cap: when UAVLOC_VO_MAXFRAMES > 0, stop after that many
    // frames are fed. Unset/0 = run to END_OF_STREAM.
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
    vo::VOConfig              vo_cfg;
    fusion::FusionConfig      fusion_cfg;
    try {
        reader_cfg = sensor::VideoReaderConfig::fromYaml(yaml);
        vo_cfg     = vo::VOConfig::fromYaml(yaml);
        fusion_cfg = fusion::FusionConfig::fromYaml(yaml);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }

    // Force synchronous fusion regardless of the YAML: this driver is a
    // deterministic offline evaluation tool — the graph must be updated inline
    // so latest() after each push() reflects exactly this frame, and repeated
    // runs are bit-for-bit reproducible.
    fusion_cfg.async_enabled = false;

    sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::warn("test_fusion_offline: cannot open video '{}' — SKIPPED",
                     reader_cfg.video_path);
        return 0;  // soft-skip when the dataset is absent
    }

    std::ofstream csv(output_csv);
    if (!csv.is_open()) {
        spdlog::error("test_fusion_offline: cannot open output CSV '{}'", output_csv);
        return 1;
    }
    csv << std::setprecision(CSV_PRECISION);
    csv << "frame_id,state,has_pose,vo_x,vo_y,vo_z,fx,fy,fz,fused_has_pose,"
           "scale,agl_bias_m,graph_updated,is_keyframe,has_telemetry,"
           "lat,lon,alt,heading_deg\n";

    vo::VOModule         vo_module(vo_cfg);
    fusion::FusionModule fusion(fusion_cfg);
    fusion.start();  // no-op in sync mode; kept for API symmetry

    spdlog::info("test_fusion_offline: config='{}' out='{}' max_frames={}",
                 config_path, output_csv, max_frames_env);

    // Run statistics (same lost/reinit accounting as test_vo_pipeline_new).
    int    frames_fed        = 0;
    int    tracking_frames   = 0;
    int    keyframe_count    = 0;
    int    lost_events       = 0;
    int    reinit_events     = 0;
    bool   reached_tracking  = false;
    bool   all_fused_finite  = true;
    bool   had_first_init    = false;
    bool   pending_reinit    = false;
    vo::VOTrackingState prev_state = vo::VOTrackingState::NOT_INITIALIZED;

    // Fused-Z vs telemetry-AGL error over TRACKING frames with telemetry.
    double abs_z_err_sum = 0.0;
    int    abs_z_err_n   = 0;
    double scale_min     =  std::numeric_limits<double>::infinity();
    double scale_max     = -std::numeric_limits<double>::infinity();
    double final_fused_z = std::numeric_limits<double>::quiet_NaN();
    double final_telem_alt = std::numeric_limits<double>::quiet_NaN();

    sensor::FrameData fd;
    while (true) {
        auto status = reader.read(fd);
        if (status == sensor::FrameStatus::END_OF_STREAM) break;
        if (status == sensor::FrameStatus::ERROR ||
            status == sensor::FrameStatus::CAMERA_DISCONNECTED) break;
        if (status != sensor::FrameStatus::OK || !fd.valid || !fd.HasImage()) continue;

        const vo::VOResult res = vo_module.process_frame(fd);
        ++frames_fed;

        // Push every frame; frames without telemetry get a default (invalid)
        // TelemetryData so the fusion module can decide what to use.
        const sensor::TelemetryData telem =
            fd.has_telemetry ? fd.telemetry : sensor::TelemetryData{};
        fusion.push(res, telem);
        const fusion::FusionResult fres = fusion.latest();

        const Eigen::Vector3d vo_t = res.T_wc.block<3, 1>(0, 3);
        const Eigen::Vector3d f_t  = fres.T_enu_c.translation();

        csv << res.frame_id << ','
            << state_name(res.state) << ','
            << static_cast<int>(res.has_pose) << ','
            << vo_t.x() << ',' << vo_t.y() << ',' << vo_t.z() << ','
            << f_t.x() << ',' << f_t.y() << ',' << f_t.z() << ','
            << static_cast<int>(fres.has_pose) << ','
            << fres.scale << ','
            << fres.agl_bias_m << ','
            << static_cast<int>(fres.graph_updated) << ','
            << static_cast<int>(res.is_keyframe) << ','
            << static_cast<int>(fd.has_telemetry) << ','
            << telem.latitude_deg << ','
            << telem.longitude_deg << ','
            << telem.altitude_m << ','
            << telem.heading_deg << '\n';

        // ── statistics ───────────────────────────────────────────────────────
        if (res.state == vo::VOTrackingState::TRACKING) {
            reached_tracking = true;
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
        prev_state = res.state;

        if (fres.has_pose) {
            if (!fres.T_enu_c.matrix().allFinite()) all_fused_finite = false;
            scale_min = std::min(scale_min, fres.scale);
            scale_max = std::max(scale_max, fres.scale);
            final_fused_z = f_t.z();
            if (res.state == vo::VOTrackingState::TRACKING && fd.has_telemetry) {
                abs_z_err_sum += std::abs(f_t.z() - fd.telemetry.altitude_m);
                ++abs_z_err_n;
            }
        }
        if (fd.has_telemetry) final_telem_alt = fd.telemetry.altitude_m;

        if (max_frames_env > 0 && frames_fed >= max_frames_env) break;
    }
    fusion.stop();
    reader.close();
    csv.close();

    if (frames_fed == 0) {
        spdlog::warn("test_fusion_offline: no valid frames in stream — SKIPPED");
        return 0;
    }

    const double mean_abs_z_err =
        (abs_z_err_n > 0) ? abs_z_err_sum / abs_z_err_n
                          : std::numeric_limits<double>::quiet_NaN();

    spdlog::info("==== test_fusion_offline summary ====");
    spdlog::info("  frames_processed  = {}", frames_fed);
    spdlog::info("  tracking_frames   = {}", tracking_frames);
    spdlog::info("  keyframes         = {}", keyframe_count);
    spdlog::info("  lost_events       = {}", lost_events);
    spdlog::info("  reinit_events     = {}", reinit_events);
    spdlog::info("  final_fused_z     = {:.3f} m (final telemetry alt = {:.3f} m)",
                 final_fused_z, final_telem_alt);
    spdlog::info("  mean |fz - alt|   = {:.3f} m over {} TRACKING frames with telemetry",
                 mean_abs_z_err, abs_z_err_n);
    spdlog::info("  scale range       = [{:.6f}, {:.6f}]", scale_min, scale_max);
    spdlog::info("  csv               = '{}'", output_csv);
    spdlog::info("=====================================");

    int rc = 0;
    if (!reached_tracking) {
        spdlog::error("test_fusion_offline FAIL: pipeline never reached TRACKING");
        rc = 1;
    }
    if (!all_fused_finite) {
        spdlog::error("test_fusion_offline FAIL: a fused pose contained non-finite values");
        rc = 1;
    }
    if (rc == 0) spdlog::info("test_fusion_offline: PASS");
    return rc;
}
