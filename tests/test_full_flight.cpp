// test_full_flight — full-flight evaluation driver: groundtruth-vs-fused CSV.
//
// Runs the REAL pipeline on a mission config: sensor::VideoReader →
// vo::VOModule::process_frame → fusion::FusionModule::push and dumps a
// per-frame CSV with groundtruth ENU coordinates (telemetry lat/lon → ENU),
// fused predicted coordinates, tracked-landmark counts, coordinate errors and
// per-frame processing time (FPS).
//
// Mode is FORCED in code regardless of the YAML (the point of this driver):
//   * VO:     async_enabled = true + wait_for_local_bundle_adjustment = true
//             (mapping thread runs, tracking blocks per keyframe until local
//             BA finishes — the "async but wait for local BA" mode)
//   * Fusion: synchronous, so latest() after each push() reflects this frame.
//
// Groundtruth: ENU anchored at the telemetry of the first frame where the
// fused pose exists (matches the fusion X(0) anchor instant). The vertical
// axis uses telemetry AGL directly — same convention as fused Z (X(0) is
// anchored at (0, 0, agl_0)).
//
// The raw horizontal error includes the unresolved mount-azimuth offset θ
// (F1 limitation: θ is prior-only), so the summary also reports the RMS after
// a 4-DoF (yaw + translation) alignment fitted over all evaluated frames.
//
// Usage:   ./tests/test_full_flight [config.yaml] [out.csv]
// Env:     UAVLOC_VO_MAXFRAMES caps the number of frames fed (0/unset = run
//          to END_OF_STREAM).
//
// Mirrors tests/test_fusion_offline.cpp (soft-skip when the gitignored
// dataset video is absent; same lost/reinit accounting). Headless, no viewer.

#include "uavloc/sensor/video_reader.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/fusion/fusion_module.h"

#include "gps_to_enu.h"

#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

namespace {
const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai500m.yaml";
const std::string DEFAULT_OUTPUT_CSV = "full_flight.csv";

constexpr int CSV_PRECISION = 12;

const char* state_name(uavloc::vo::VOTrackingState s) {
    switch (s) {
        case uavloc::vo::VOTrackingState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case uavloc::vo::VOTrackingState::TRACKING:        return "TRACKING";
        case uavloc::vo::VOTrackingState::LOST:            return "LOST";
    }
    return "UNKNOWN";
}

//! 4-DoF (yaw + translation) least-squares alignment of 2-D source points onto
//! 2-D destination points: closed-form optimal yaw from the 2x2 correlation,
//! then the centroid-matching translation. Returns the aligned RMS.
double aligned_rms_2d(const std::vector<Eigen::Vector2d>& src,
                      const std::vector<Eigen::Vector2d>& dst,
                      double& yaw_deg_out) {
    const std::size_t n = src.size();
    if (n < 2) {
        yaw_deg_out = std::numeric_limits<double>::quiet_NaN();
        return std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::Vector2d s_mean = Eigen::Vector2d::Zero();
    Eigen::Vector2d d_mean = Eigen::Vector2d::Zero();
    for (std::size_t i = 0; i < n; ++i) { s_mean += src[i]; d_mean += dst[i]; }
    s_mean /= static_cast<double>(n);
    d_mean /= static_cast<double>(n);

    Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
    for (std::size_t i = 0; i < n; ++i)
        C += (dst[i] - d_mean) * (src[i] - s_mean).transpose();

    const double yaw = std::atan2(C(1, 0) - C(0, 1), C(0, 0) + C(1, 1));
    yaw_deg_out = yaw * 180.0 / M_PI;
    Eigen::Matrix2d R;
    R << std::cos(yaw), -std::sin(yaw),
         std::sin(yaw),  std::cos(yaw);
    const Eigen::Vector2d t = d_mean - R * s_mean;

    double sq_sum = 0.0;
    for (std::size_t i = 0; i < n; ++i)
        sq_sum += (R * src[i] + t - dst[i]).squaredNorm();
    return std::sqrt(sq_sum / static_cast<double>(n));
}
}  // namespace

int main(int argc, char** argv) {
    using namespace uavloc;
    namespace dv = uavloc::debug_viewer;
    using clock = std::chrono::steady_clock;

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

    // Forced mode (see file header): VO async with the per-keyframe local-BA
    // handshake; fusion synchronous so latest() reflects exactly this frame.
    vo_cfg.async_enabled                    = true;
    vo_cfg.wait_for_local_bundle_adjustment = true;
    fusion_cfg.async_enabled                = false;

    sensor::VideoReader reader(reader_cfg);
    if (!reader.open()) {
        spdlog::warn("test_full_flight: cannot open video '{}' — SKIPPED",
                     reader_cfg.video_path);
        return 0;  // soft-skip when the dataset is absent
    }

    std::ofstream csv(output_csv);
    if (!csv.is_open()) {
        spdlog::error("test_full_flight: cannot open output CSV '{}'", output_csv);
        return 1;
    }
    csv << std::setprecision(CSV_PRECISION);
    csv << "frame_id,timestamp_msec,state,is_keyframe,"
           "num_landmarks,num_tracked,num_inliers,"
           "gt_lat,gt_lon,gt_e,gt_n,gt_u,"
           "vo_x,vo_y,vo_z,pred_x,pred_y,pred_z,"
           "err_2d_m,err_z_m,err_3d_m,scale,agl_bias_m,dt_ms,"
           "median_map_depth,median_depth_num_lms\n";

    vo::VOModule         vo_module(vo_cfg);
    fusion::FusionModule fusion(fusion_cfg);
    fusion.start();  // no-op in sync mode; kept for API symmetry

    spdlog::info("test_full_flight: config='{}' out='{}' max_frames={} "
                 "(VO async+wait_for_local_BA, fusion sync)",
                 config_path, output_csv, max_frames_env);

    // Run statistics (same lost/reinit accounting as test_fusion_offline).
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

    // Groundtruth ENU anchor: telemetry of the first frame with a fused pose.
    bool   gt_anchored = false;
    double lat0 = 0.0, lon0 = 0.0;

    // Error accumulation over TRACKING frames with telemetry + fused pose.
    double err2d_sq_sum = 0.0, err3d_sq_sum = 0.0, abs_z_err_sum = 0.0;
    int    err_n = 0;
    std::vector<Eigen::Vector2d> align_src;  // fused XY
    std::vector<Eigen::Vector2d> align_dst;  // groundtruth EN

    double scale_min =  std::numeric_limits<double>::infinity();
    double scale_max = -std::numeric_limits<double>::infinity();

    // FPS: processing time (VO + fusion, excludes video decode) and wall time.
    double proc_sec_total = 0.0;
    const auto wall_start = clock::now();

    const double nan = std::numeric_limits<double>::quiet_NaN();

    // A cut MKV can advertise more container frames than are actually
    // decodable; VideoReader then returns EMPTY_FRAME forever instead of
    // END_OF_STREAM. Treat a run of consecutive non-OK reads as end of stream.
    constexpr int MAX_CONSECUTIVE_BAD_READS = 100;
    int consecutive_bad_reads = 0;

    sensor::FrameData fd;
    while (true) {
        auto status = reader.read(fd);
        if (status == sensor::FrameStatus::END_OF_STREAM) break;
        if (status == sensor::FrameStatus::ERROR ||
            status == sensor::FrameStatus::CAMERA_DISCONNECTED) break;
        if (status != sensor::FrameStatus::OK || !fd.valid || !fd.HasImage()) {
            if (++consecutive_bad_reads >= MAX_CONSECUTIVE_BAD_READS) {
                spdlog::warn("test_full_flight: {} consecutive bad reads — "
                             "treating as end of stream", consecutive_bad_reads);
                break;
            }
            continue;
        }
        consecutive_bad_reads = 0;

        const auto t0 = clock::now();
        const vo::VOResult res = vo_module.process_frame(fd);
        const sensor::TelemetryData telem =
            fd.has_telemetry ? fd.telemetry : sensor::TelemetryData{};
        fusion.push(res, telem);
        const fusion::FusionResult fres = fusion.latest();
        const double dt_ms =
            std::chrono::duration<double, std::milli>(clock::now() - t0).count();
        proc_sec_total += dt_ms * 1e-3;
        ++frames_fed;

        const Eigen::Vector3d vo_t = res.T_wc.block<3, 1>(0, 3);
        const Eigen::Vector3d f_t  = fres.T_enu_c.translation();

        // Usable telemetry = synced record with a plausible AGL (viewer gate).
        const bool telem_ok = fd.has_telemetry && telem.altitude_m > 0.0;

        // Anchor the groundtruth ENU frame at the fusion-init instant.
        if (!gt_anchored && fres.has_pose && telem_ok) {
            gt_anchored = true;
            lat0 = telem.latitude_deg;
            lon0 = telem.longitude_deg;
            spdlog::info("groundtruth ENU anchored at frame {} (lat0={:.7f}, lon0={:.7f})",
                         res.frame_id, lat0, lon0);
        }

        double gt_e = nan, gt_n = nan, gt_u = nan;
        double err_2d = nan, err_z = nan, err_3d = nan;
        if (gt_anchored && telem_ok) {
            const dv::ENUPoint p = dv::gps_to_enu(
                telem.latitude_deg, telem.longitude_deg, telem.altitude_m,
                lat0, lon0, 0.0);
            gt_e = p.e;
            gt_n = p.n;
            gt_u = telem.altitude_m;  // AGL — same convention as fused Z
            if (fres.has_pose) {
                err_2d = std::hypot(f_t.x() - gt_e, f_t.y() - gt_n);
                err_z  = f_t.z() - gt_u;
                err_3d = std::sqrt(err_2d * err_2d + err_z * err_z);
                if (res.state == vo::VOTrackingState::TRACKING) {
                    err2d_sq_sum  += err_2d * err_2d;
                    err3d_sq_sum  += err_3d * err_3d;
                    abs_z_err_sum += std::abs(err_z);
                    ++err_n;
                    align_src.emplace_back(f_t.x(), f_t.y());
                    align_dst.emplace_back(gt_e, gt_n);
                }
            }
        }

        csv << res.frame_id << ','
            << res.timestamp_msec << ','
            << state_name(res.state) << ','
            << static_cast<int>(res.is_keyframe) << ','
            << res.num_landmarks << ','
            << res.tracked_observations.size() << ','
            << res.num_inliers << ','
            << (telem_ok ? telem.latitude_deg : nan) << ','
            << (telem_ok ? telem.longitude_deg : nan) << ','
            << gt_e << ',' << gt_n << ',' << gt_u << ','
            << vo_t.x() << ',' << vo_t.y() << ',' << vo_t.z() << ','
            << f_t.x() << ',' << f_t.y() << ',' << f_t.z() << ','
            << err_2d << ',' << err_z << ',' << err_3d << ','
            << fres.scale << ','
            << fres.agl_bias_m << ','
            << dt_ms << ','
            << res.median_map_depth << ','
            << res.median_depth_num_lms << '\n';

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
        }

        if (max_frames_env > 0 && frames_fed >= max_frames_env) break;
    }
    const double wall_sec_total =
        std::chrono::duration<double>(clock::now() - wall_start).count();
    fusion.stop();
    reader.close();
    csv.close();

    if (frames_fed == 0) {
        spdlog::warn("test_full_flight: no valid frames in stream — SKIPPED");
        return 0;
    }

    const double rms_2d = (err_n > 0) ? std::sqrt(err2d_sq_sum / err_n) : nan;
    const double rms_3d = (err_n > 0) ? std::sqrt(err3d_sq_sum / err_n) : nan;
    const double mean_abs_z = (err_n > 0) ? abs_z_err_sum / err_n : nan;
    double yaw_fit_deg = nan;
    const double rms_2d_aligned = aligned_rms_2d(align_src, align_dst, yaw_fit_deg);
    const double fps_proc = (proc_sec_total > 0.0) ? frames_fed / proc_sec_total : nan;
    const double fps_wall = (wall_sec_total > 0.0) ? frames_fed / wall_sec_total : nan;

    spdlog::info("==== test_full_flight summary ====");
    spdlog::info("  config            = '{}'", config_path);
    spdlog::info("  frames_processed  = {}", frames_fed);
    spdlog::info("  tracking_frames   = {}", tracking_frames);
    spdlog::info("  keyframes         = {}", keyframe_count);
    spdlog::info("  lost_events       = {}", lost_events);
    spdlog::info("  reinit_events     = {}", reinit_events);
    spdlog::info("  fps (proc)        = {:.2f}  (VO+fusion only, {:.1f} s)",
                 fps_proc, proc_sec_total);
    spdlog::info("  fps (wall)        = {:.2f}  (incl. decode, {:.1f} s)",
                 fps_wall, wall_sec_total);
    spdlog::info("  eval frames       = {} (TRACKING + telemetry + fused pose)", err_n);
    spdlog::info("  RMS err_2d (raw)  = {:.2f} m   (includes mount-azimuth offset)", rms_2d);
    spdlog::info("  RMS err_3d (raw)  = {:.2f} m", rms_3d);
    spdlog::info("  mean |err_z|      = {:.2f} m", mean_abs_z);
    spdlog::info("  RMS err_2d 4-DoF  = {:.2f} m   (after yaw+translation fit, yaw = {:.2f} deg)",
                 rms_2d_aligned, yaw_fit_deg);
    spdlog::info("  scale range       = [{:.6f}, {:.6f}]", scale_min, scale_max);
    spdlog::info("  csv               = '{}'", output_csv);
    spdlog::info("==================================");

    int rc = 0;
    if (!reached_tracking) {
        spdlog::error("test_full_flight FAIL: pipeline never reached TRACKING");
        rc = 1;
    }
    if (!all_fused_finite) {
        spdlog::error("test_full_flight FAIL: a fused pose contained non-finite values");
        rc = 1;
    }
    if (rc == 0) spdlog::info("test_full_flight: PASS");
    return rc;
}
