// test_full_flight — full-flight evaluation driver: groundtruth-vs-fused CSV.
//
// Runs the REAL pipeline on a mission config: sensor::VideoReader →
// core::SystemManager (VO front-end + fusion back-end + geo-referencer) and
// dumps a per-frame CSV with groundtruth ENU coordinates (telemetry lat/lon →
// ENU), fused predicted coordinates, tracked-landmark counts, coordinate errors
// and per-frame processing time (FPS).
//
// Since S4 the driver no longer drives vo::VOModule / fusion::FusionModule
// itself: it feeds core::SystemManager::onFrame() and reads what it needs off
// the debug channels — callbacks().on_vo_data for the VOResult and
// callbacks().on_fusion_result
// for the raw FusionResult (the production LocalizationOutput does not carry
// the ENU pose / scale / AGL-bias columns this CSV reports). Everything else —
// groundtruth ENU, every error figure, the fake-fix generator and the CSV
// layout — stayed in the driver, and the CSV is bit-identical to the
// pre-SystemManager baseline apart from the dt_ms column.
//
// CSV layout: the 30 historical columns are FROZEN (they are the regression
// gate). The absolute-fix gate observability added afterwards is APPENDED at
// the end — fix_dist_m (distance flown since the last APPLIED fix, the `s` of
// the drift term), fix_gated_resid_m / fix_gate_radius_m (most recent
// rejection), and the cumulative per-reason drop counters n_age_expired /
// n_unmatched / n_marginalized / n_queue_dropped. The fix_status column now
// names the exact drop reason instead of the merged "too_late".
//
// Mode is FORCED in code regardless of the YAML (the point of this driver):
//   * VO:     async_enabled = true + wait_for_local_bundle_adjustment = true
//             (mapping thread runs, tracking blocks per keyframe until local
//             BA finishes — the "async but wait for local BA" mode)
//   * Fusion: synchronous, so the FusionResult observed on the debug channel
//             during onFrame() belongs to exactly this frame.
//   * System: async_input = false (S4 covers the synchronous path only),
//             publish_images = false.
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
// Fake absolute fixes (milestone M1) come from anchor::FakeAnchor, attached to
// the SystemManager through anchor::AnchorInterface — the same request/response
// seam the real VPR producer will use, so this driver exercises the production
// path instead of a private shortcut. The generator used to live in this file;
// only the SCORING stayed (which fix landed on which CSV row, which outlier the
// back-end caught). Its configuration is layered — struct defaults, then the
// mission YAML node `FakeAnchor:` (absent in every config/ file today, so it
// changes nothing), then the UAVLOC_FIX_* environment, which always wins — and
// it is OFF by default: with UAVLOC_FAKE_FIX unset no anchor is attached at all
// and the run is the plain VO+fusion regression gate. See
// tests/driver_common.h; test_vo_viewer resolves it with the same code.
//
// ⚠ The generator move (2026-08-03) changed WHERE the RNG is drawn, so a run
// WITH fixes is not bit-comparable against a pre-move baseline; a run WITHOUT
// fixes is (that is the gate). The fake groundtruth is now read from the drone
// telemetry CSV in double precision, where the driver used the float-valued
// debug_viewer::gps_to_enu — a sub-centimetre difference in the fix positions.
//
// ⚠ Any error figure produced with UAVLOC_FAKE_FIX=1 must be reported together
// with the sentence "fixes are generated from groundtruth" — it measures the
// back-end's ability to consume absolute positions, NOT the accuracy of a real
// system.
//
// ── UAVLOC_INPUT_MODE — the S6c equivalence switch ───────────────────────────
// frame    (default) the driver reads a FrameData from VideoReader and calls
//          SystemManager::onFrame() — the S4/S5 path, unchanged.
// channels the driver SPLITS that same FrameData into the four typed channels
//          and calls onAttitude → onGimbal → onGnss → onImage (image LAST,
//          which is the contract that makes the assembly step work). The
//          manager then re-assembles the frame through core::Extrapolator.
// Everything downstream of onFrame() is literally the same code, so the two
// modes must produce a bit-identical CSV; that equality IS the S6c gate. The
// switch lives in the driver, not in the YAML, because it selects an
// experiment, not a system property.
//
// Usage:   ./tests/test_full_flight [config.yaml] [out.csv]
// Env:     UAVLOC_INPUT_MODE=frame|channels  (default frame)
//          UAVLOC_VO_MAXFRAMES caps the number of frames fed (0/unset = run
//          to END_OF_STREAM).
//          UAVLOC_PROFILE=1 enables the stage profiler (off by default): the
//          run then also writes profile_<out.csv stem>.csv and logs a summary
//          table. Profiling only reads clocks — it never alters the pipeline.
//          UAVLOC_FAKE_FIX=1        enable the fake-fix generator (default 0)
//          UAVLOC_FIX_SIGMA_M       Gaussian noise sigma [m]      (default 20)
//          UAVLOC_FIX_EVERY_KF      push one fix every N keyframes(default 20)
//          UAVLOC_FIX_REINIT_MIN_KF minimum request gap between two fixes that
//                                   BYPASS the cadence on a VO re-init
//                                   (default 0 = every re-init bypasses)
//          UAVLOC_FIX_OUTLIER_RATE  fraction of gross-outlier fixes(default 0)
//          UAVLOC_FIX_OUTLIER_MIN_M / _MAX_M  outlier magnitude band
//                                                        (default 200 / 1000)
//          UAVLOC_FIX_LATENCY_KF    delay a fix by N keyframes before pushing
//                                   it (proves the smoother corrects the past)
//          UAVLOC_FIX_SEED          RNG seed (default 42; same seed ⇒ same
//                                   CSV, byte for byte)
//          UAVLOC_FIX_DRIFT_RATE    override FusionConfig::fix_drift_rate_m_per_m
//                                   [m of gate inflation per m flown]. Unset =
//                                   whatever the config says (default 0 = off).
//                                   Measured per dataset — see Phase2_roadmap
//                                   §2.5 (RPE p95 @1000 m): ds6 ≈ 0.15,
//                                   ds3 ≈ 0.18, YenBai 800 m ≈ 0.68.
//          UAVLOC_FIX_KERNEL        override FusionConfig::fix_robust_kernel
//                                   (none|huber|tukey). Unset = config default
//                                   (huber since 2026-08-02) — switching
//                                   kernels must be an explicit experiment,
//                                   never a silent one.
//          UAVLOC_FIX_GATE=0|1      override FusionConfig::fix_gate_enabled,
//                                   the self-consistency Mahalanobis gate.
//                                   Unset = config default (OFF). Set to 1
//                                   together with UAVLOC_FIX_KERNEL=tukey and
//                                   UAVLOC_FIX_DRIFT_RATE=0 to reproduce the
//                                   M1 configuration.
//          UAVLOC_ACCURACY_CSV=<path> ALSO write a sidecar CSV pairing the
//                                   self-reported LocalizationOutput::
//                                   accuracy_m with the true err_2d of the same
//                                   frame (S7). A SEPARATE file on purpose: the
//                                   main CSV is the bit-identical gate and must
//                                   keep its exact column set.
//
// Mirrors tests/test_fusion_offline.cpp (soft-skip when the gitignored
// dataset video is absent; same lost/reinit accounting). Headless, no viewer.

#include "uavloc/sensor/stream_types.h"
#include "uavloc/sensor/video_reader.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/anchor/absolute_fix.h"
#include "uavloc/anchor/fake_anchor.h"
#include "uavloc/core/system_config.h"
#include "uavloc/core/system_manager.h"
#include "uavloc/fusion/fusion_config.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/util/scoped_timer.h"

#include "driver_common.h"
#include "eval_common.h"
#include "gps_to_enu.h"

#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai500m.yaml";
const std::string DEFAULT_OUTPUT_CSV = "full_flight.csv";

constexpr int CSV_PRECISION = 12;

const char* health_name(uavloc::fusion::FusionHealth h) {
    switch (h) {
        case uavloc::fusion::FusionHealth::INITIALIZING: return "INITIALIZING";
        case uavloc::fusion::FusionHealth::CONVERGED:    return "CONVERGED";
        case uavloc::fusion::FusionHealth::DRIFTING:     return "DRIFTING";
    }
    return "UNKNOWN";
}

const char* kernel_name(uavloc::fusion::FixRobustKernel k) {
    switch (k) {
        case uavloc::fusion::FixRobustKernel::NONE:  return "none";
        case uavloc::fusion::FixRobustKernel::HUBER: return "huber";
        case uavloc::fusion::FixRobustKernel::TUKEY: return "tukey";
    }
    return "unknown";
}

const char* state_name(uavloc::vo::VOTrackingState s) {
    switch (s) {
        case uavloc::vo::VOTrackingState::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case uavloc::vo::VOTrackingState::TRACKING:        return "TRACKING";
        case uavloc::vo::VOTrackingState::LOST:            return "LOST";
    }
    return "UNKNOWN";
}

//! How the driver hands a frame to core::SystemManager (UAVLOC_INPUT_MODE).
enum class InputMode {
    FRAME,    //!< onFrame(FrameData) — the pre-S6c path
    CHANNELS  //!< onAttitude/onGimbal/onGnss/onImage — the S6c path
};

//! Reads UAVLOC_INPUT_MODE. An unrecognised value keeps the default and warns:
//! silently falling back would make a typo look like a passing gate.
InputMode input_mode_from_env() {
    const char* v = std::getenv("UAVLOC_INPUT_MODE");
    if (v == nullptr || *v == '\0') return InputMode::FRAME;
    const std::string s(v);
    if (s == "frame")    return InputMode::FRAME;
    if (s == "channels") return InputMode::CHANNELS;
    spdlog::warn("test_full_flight: unknown UAVLOC_INPUT_MODE '{}' — "
                 "expected frame|channels; using frame", s);
    return InputMode::FRAME;
}

//! Publish one FrameData as the four typed channels, in the order a real source
//! uses: attitude → gimbal → gnss → IMAGE LAST (the image triggers processing,
//! so the other three must already be buffered when it fires). A frame with no
//! telemetry publishes the image channel ONLY — the driver invents nothing,
//! exactly like sensor::VideoDataSource.
bool feed_as_channels(uavloc::core::SystemManager& sys,
                      const uavloc::sensor::FrameData& fd) {
    const double t = fd.timestamp_msec;
    if (fd.has_telemetry) {
        uavloc::sensor::AttitudeData att;
        att.roll_deg  = fd.telemetry.roll_deg;
        att.pitch_deg = fd.telemetry.pitch_deg;
        att.yaw_deg   = fd.telemetry.heading_deg;
        sys.onAttitude(t, att);

        uavloc::sensor::GimbalData gim;
        gim.pan_deg  = fd.telemetry.gimbal_pan_deg;
        gim.tilt_deg = fd.telemetry.gimbal_tilt_deg;
        sys.onGimbal(t, gim);

        uavloc::sensor::GnssData gnss;
        gnss.latitude_deg  = fd.telemetry.latitude_deg;
        gnss.longitude_deg = fd.telemetry.longitude_deg;
        gnss.altitude_m    = fd.telemetry.altitude_m;
        gnss.speed_mps     = fd.telemetry.speed_mps;
        sys.onGnss(t, gnss);
    }

    uavloc::sensor::ImageData img;
    img.frame_id  = fd.frame_id;
    img.image     = fd.image;
    img.camera_id = fd.camera_id;
    return sys.onImage(t, img);
}

// The scoring maths (percentile / least_squares_slope / aligned_rms_2d) and
// the profiler dump (dump_profile / profile_csv_path) moved to
// tests/eval_common.{h,cpp} at S8, unchanged, so every evaluation driver
// reports with the SAME ruler. Brought back into this namespace by name so the
// call sites below read exactly as before.
using uavloc::eval::aligned_rms_2d;
using uavloc::eval::dump_profile;
using uavloc::eval::least_squares_slope;
using uavloc::eval::percentile;
using uavloc::eval::profile_csv_path;

// Input-side configuration shared with tests/test_vo_viewer.cpp (S9):
// force_offline_run_mode() is the run mode both drivers must use, and
// fake_anchor_config() layers defaults ← YAML "FakeAnchor:" ← UAVLOC_FIX_*.
// The eleven UAVLOC_FIX_* / UAVLOC_FAKE_FIX names are unchanged on purpose: an
// old command line must still mean the same experiment.
using uavloc::eval::env_int;
using uavloc::eval::fake_anchor_config;
using uavloc::eval::force_offline_run_mode;
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

    // Which front door of SystemManager this run uses (see the file header).
    const InputMode input_mode = input_mode_from_env();

    // Stage profiling is opt-in (UAVLOC_PROFILE=1) and read-only: it never
    // changes the pipeline, so the summary numbers must match a run without it.
    const char* prof_env = std::getenv("UAVLOC_PROFILE");
    const bool profile_enabled = prof_env != nullptr && std::atoi(prof_env) != 0;
    util::Profiler::set_enabled(profile_enabled);
    util::Profiler::set_thread_label("driver");

    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("Failed to load YAML '{}': {}", config_path, e.what());
        return 1;
    }

    sensor::VideoReaderConfig reader_cfg;
    core::SystemConfig        sys_cfg;
    try {
        reader_cfg = sensor::VideoReaderConfig::fromYaml(yaml);
        sys_cfg    = core::SystemConfig::fromYaml(yaml);
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }

    // Forced mode (see file header): VO async with the per-keyframe local-BA
    // handshake; fusion synchronous so the result seen on the debug channel
    // belongs to exactly this frame; input inline on this thread. The four
    // values live in tests/driver_common.cpp so that test_vo_viewer forces the
    // SAME ones — the cross-driver parity gate compares their pose sequences,
    // which is meaningless if the two run in different modes.
    force_offline_run_mode(sys_cfg);
    // Not shared: no viewer here, so never keep frame buffers alive.
    sys_cfg.publish_images = false;

    // ── Gate-experiment overrides (driver only) ──────────────────────────────
    // The absolute-fix gate ablation of the M1 post-mortem needs to sweep two
    // knobs. They are read from the environment and NOT from the mission YAML:
    // config/ describes the system, an ablation is an experiment. Unset ⇒ the
    // configured value is untouched, so a plain run stays the regression gate.
    if (const char* v = std::getenv("UAVLOC_FIX_DRIFT_RATE");
        v != nullptr && *v != '\0') {
        const double rho = std::atof(v);
        if (rho < 0.0) {
            spdlog::warn("test_full_flight: UAVLOC_FIX_DRIFT_RATE={} is negative "
                         "— ignored (the drift term inflates the gate, it cannot "
                         "shrink it)", rho);
        } else {
            sys_cfg.fusion.fix_drift_rate_m_per_m = rho;
            spdlog::warn("test_full_flight: fix_drift_rate_m_per_m OVERRIDDEN to "
                         "{} m/m by UAVLOC_FIX_DRIFT_RATE", rho);
        }
    }
    if (const char* v = std::getenv("UAVLOC_FIX_KERNEL"); v != nullptr && *v != '\0') {
        const std::string k(v);
        if (k == "none") {
            sys_cfg.fusion.fix_robust_kernel = fusion::FixRobustKernel::NONE;
        } else if (k == "huber") {
            sys_cfg.fusion.fix_robust_kernel = fusion::FixRobustKernel::HUBER;
        } else if (k == "tukey") {
            sys_cfg.fusion.fix_robust_kernel = fusion::FixRobustKernel::TUKEY;
        } else {
            spdlog::warn("test_full_flight: unknown UAVLOC_FIX_KERNEL '{}' — "
                         "expected none|huber|tukey; keeping the configured "
                         "kernel", k);
        }
        spdlog::warn("test_full_flight: fix_robust_kernel requested '{}' by "
                     "UAVLOC_FIX_KERNEL", k);
    }
    if (const char* v = std::getenv("UAVLOC_FIX_GATE"); v != nullptr && *v != '\0') {
        sys_cfg.fusion.fix_gate_enabled = std::atoi(v) != 0;
        spdlog::warn("test_full_flight: fix_gate_enabled OVERRIDDEN to {} by "
                     "UAVLOC_FIX_GATE (the self-consistency Mahalanobis gate)",
                     sys_cfg.fusion.fix_gate_enabled);
    }

    // What the absolute-fix intake ACTUALLY resolved to, after YAML and env.
    // Printed rather than inferred: a matrix of kernel/gate cells is worthless
    // if the cell labels are assumed instead of read back (2026-08-03 incident).
    spdlog::info("test_full_flight: fix intake resolved — gate={} (chi2={}), "
                 "kernel={} (huber_k={}, tukey_c={}), drift_rate={} m/m, "
                 "min_confidence={}",
                 sys_cfg.fusion.fix_gate_enabled, sys_cfg.fusion.fix_gate_chi2,
                 kernel_name(sys_cfg.fusion.fix_robust_kernel),
                 sys_cfg.fusion.huber_k, sys_cfg.fusion.fix_tukey_c,
                 sys_cfg.fusion.fix_drift_rate_m_per_m,
                 sys_cfg.fusion.fix_min_confidence);

    // ⚠ Determinism guard. Every RANSAC in new_vo seeds itself from
    // std::random_device unless VO.use_fixed_seed is true, so without that key
    // two runs of the SAME config produce DIFFERENT keyframes — measured on
    // config/uavloc_yenbai800m.yaml (that file was DELETED on 2026-08-03; the
    // name is kept here because the measurement belongs to it and must not be
    // re-attributed to the surviving config/uavloc_yenbai800m_newvo.yaml, which
    // does set use_fixed_seed: true), where two identical runs diverged at frame
    // 8 and ended with 43 vs 4 applied fixes over the first 2050 frames. This
    // driver's CSV is used as a bit-identical regression gate and as the source
    // of published ablation tables; neither is meaningful without a fixed seed.
    if (!sys_cfg.vo.use_fixed_seed) {
        spdlog::warn("test_full_flight: VO.use_fixed_seed is FALSE in '{}' — the "
                     "VO RANSACs are seeded from std::random_device, so this run "
                     "is NOT reproducible and its numbers may NOT be compared "
                     "against another run (add 'use_fixed_seed: true' under VO: "
                     "in the mission config)", config_path);
    }

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
           "median_map_depth,median_depth_num_lms,"
           "fix_e,fix_n,fix_is_outlier,fix_status,"
           // Gate observability, APPENDED after the 30 historical columns so
           // every stored baseline still compares column-by-column.
           "fix_dist_m,fix_gated_resid_m,fix_gate_radius_m,"
           "n_age_expired,n_unmatched,n_marginalized,n_queue_dropped\n";

    // ── S7 accuracy sidecar (optional, off by default) ───────────────────────
    // accuracy_m deliberately does NOT enter the CSV above: that file is the
    // bit-identical regression gate, and a new column would invalidate every
    // stored baseline. UAVLOC_ACCURACY_CSV writes a SEPARATE file pairing the
    // self-reported accuracy with the TRUE horizontal error of the same frame —
    // the evidence for how (in)consistent the estimator is.
    std::ofstream acc_csv;
    if (const char* acc_path = std::getenv("UAVLOC_ACCURACY_CSV")) {
        if (acc_path[0] != '\0') {
            acc_csv.open(acc_path);
            if (acc_csv.is_open()) {
                acc_csv << std::setprecision(CSV_PRECISION);
                acc_csv << "frame_id,accuracy_m,err_2d_m,health,valid\n";
                spdlog::info("test_full_flight: accuracy sidecar → '{}'", acc_path);
            } else {
                spdlog::warn("test_full_flight: cannot open accuracy sidecar '{}'",
                             acc_path);
            }
        }
    }

    // Slots the debug-channel subscribers write into. Declared BEFORE the
    // SystemManager so they outlive it: the subscribers capture them by
    // reference and are only unregistered when the manager is destroyed.
    vo::VOResult         res;
    fusion::FusionResult fres;
    bool have_res  = false;
    bool have_fres = false;
    //! Production output of the frame in flight (S7 accuracy sidecar only).
    core::LocalizationOutput loc;

    // The whole pipeline behind one object. The two debug channels below are
    // the ONLY way this driver observes the internals: the per-frame VOResult
    // and the raw FusionResult. Both fire synchronously inside onFrame() (VO
    // on this thread, fusion inline because fusion.async_enabled = false), so
    // reading the slots right after onFrame() returns is race-free.
    core::SystemManager sys(sys_cfg);

    sys.callbacks().on_vo_data.add([&](double /*timestamp_msec*/, const vo::VOData& d) {
        res      = d.result;
        have_res = true;
    });
    sys.callbacks().on_fusion_result.add(
        [&](double /*timestamp_msec*/, const fusion::FusionResult& r) {
            fres      = r;
            have_fres = true;
        });
    // Production output — used ONLY by the optional accuracy sidecar below, so
    // that the reported accuracy_m is literally the number a consumer gets.
    sys.callbacks().on_localization.add(
        [&](const core::LocalizationOutput& o) { loc = o; });

    // setup()/start() deliberately happen further down, right before the read
    // loop: the fake anchor has to be attached BEFORE setup(), and its
    // generation callback writes into the per-fix accounting declared below.

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

    // ── fake-fix producer (library) + per-fix accounting (here) ──────────────
    const bool fix_enabled = env_int("UAVLOC_FAKE_FIX", 0) != 0;
    // Resolved only when it is going to be used: the helper logs the PROVENANCE
    // of every_kf/sigma_m/seed, and printing that on a run with no anchor
    // attached would suggest an experiment that is not happening.
    const anchor::FakeAnchorConfig fix_cfg =
        fix_enabled ? fake_anchor_config(yaml, reader_cfg)
                    : anchor::FakeAnchorConfig{};

    // A fix pushed at frame F is consumed at the NEXT keyframe graph update, so
    // its verdict is only observable a few frames later — as a delta of the
    // module's cumulative counters. CSV rows are therefore held back until every
    // fix recorded on them has resolved (bounded: ~one keyframe interval).
    struct BufferedRow {
        std::string prefix;   //!< row up to and including fix_is_outlier + ','
        //! none | applied | gated | low_confidence | age_expired | unmatched |
        //! marginalized | queue_dropped | no_graph — one string per drop
        //! reason, so a rejected fix can be diagnosed from the CSV alone (M1
        //! could not).
        std::string status;
        std::string suffix;   //!< the appended gate-observability columns
    };
    struct PendingFix {
        std::size_t row_seq    = 0;
        bool        is_outlier = false;
    };
    std::deque<BufferedRow> row_buf;
    std::deque<PendingFix>  pending_fixes;
    std::size_t             row_seq_next = 0;   //!< global index of the next row
    std::size_t             row_buf_base = 0;   //!< global index of row_buf.front()

    fusion::FusionFixStats prev_fstats{};
    //! Stats snapshot of the frame in flight — the source of the appended
    //! gate-observability CSV columns.
    fusion::FusionFixStats cur_fstats{};
    int    fixes_generated    = 0;
    int    outliers_generated = 0;
    int    outliers_caught    = 0;   //!< outliers rejected by the gate
    //! Fixes produced by a re-anchor request (the VO chain had just restarted)
    //! and the frames they landed on. Kept SEPARATE from the cadence fixes:
    //! merged, a run could never say whether the re-anchor path fired.
    int                       fixes_from_reinit = 0;
    std::vector<unsigned int> reinit_fix_frames;
    std::vector<double> fix_spacing_m;  //!< GT path length between two fixes
    double fix_last_path_m    = 0.0;
    bool   fix_have_last_path = false;

    // What the producer manufactured DURING the onFrame() call in flight, seen
    // through anchor::FakeAnchor::setGenerationCallback. Reset at the top of
    // every iteration; consumed when the CSV row of that frame is composed.
    double fix_e_col       = std::numeric_limits<double>::quiet_NaN();
    double fix_n_col       = std::numeric_limits<double>::quiet_NaN();
    int    fix_outlier_col = 0;
    bool   fix_gen_this_frame = false;

    // Trajectory-quality accumulators (summary only, read-only on the pipeline).
    std::vector<double> err2d_samples;   //!< every evaluated |err_2d|
    std::vector<double> err2d_path_m;    //!< GT path length at those samples
    double gt_path_m       = 0.0;        //!< cumulative groundtruth distance
    double prev_gt_e = 0.0, prev_gt_n = 0.0;
    bool   have_prev_gt    = false;
    Eigen::Vector3d prev_pred = Eigen::Vector3d::Zero();
    bool   have_prev_pred  = false;
    double max_kf_jump_m   = 0.0;        //!< biggest fused jump at a graph update

    //! Attribute `n` freshly resolved fixes (all with the same verdict) to the
    //! oldest unresolved fixes, in push order. The module consumes fixes FIFO,
    //! and with UAVLOC_FIX_EVERY_KF >= 2 at most one fix is ever in flight, so
    //! this mapping is exact; it only degrades to an ordering guess if several
    //! fixes resolve inside a single graph update.
    auto resolve_fixes = [&](unsigned long long n, const char* status,
                             bool counts_as_outlier_catch) {
        for (unsigned long long i = 0; i < n && !pending_fixes.empty(); ++i) {
            const PendingFix pf = pending_fixes.front();
            pending_fixes.pop_front();
            if (pf.row_seq >= row_buf_base &&
                pf.row_seq - row_buf_base < row_buf.size()) {
                row_buf[pf.row_seq - row_buf_base].status = status;
            }
            if (counts_as_outlier_catch && pf.is_outlier) {
                ++outliers_caught;
            }
        }
    };

    //! Write out every buffered row that precedes the oldest unresolved fix.
    auto flush_rows = [&]() {
        while (!row_buf.empty() &&
               (pending_fixes.empty() || row_buf_base < pending_fixes.front().row_seq)) {
            csv << row_buf.front().prefix << row_buf.front().status
                << row_buf.front().suffix << '\n';
            row_buf.pop_front();
            ++row_buf_base;
        }
    };

    // ── attach the fake absolute-position producer (M1) ──────────────────────
    // OFF by default: with UAVLOC_FAKE_FIX unset nothing is attached, and the
    // run is the plain VO+fusion regression gate. The driver keeps a BORROWED
    // pointer for the producer-side counters only — ownership sits in
    // SystemManager, which sequences setup/start/stop.
    anchor::FakeAnchor* fake_anchor = nullptr;
    if (fix_enabled) {
        if (fix_cfg.telemetry.csv_path.empty()) {
            spdlog::error("test_full_flight: UAVLOC_FAKE_FIX=1 but '{}' configures "
                          "no VideoReader.DroneTelemetry.csv_path — the fake "
                          "anchor has no groundtruth to read; aborting instead of "
                          "reporting a run with zero fixes", config_path);
            return 1;
        }
        auto fa = std::make_unique<anchor::FakeAnchor>(fix_cfg);
        // Manufacturing record → the CSV fix columns + the outlier bookkeeping.
        // Fires INSIDE sys.onFrame(), on this thread (the producer is
        // synchronous), before the row of that frame is composed.
        fa->setGenerationCallback([&](const anchor::FakeFixRecord& rec) {
            fix_e_col          = rec.fix.xy_enu.x();
            fix_n_col          = rec.fix.xy_enu.y();
            fix_outlier_col    = rec.is_outlier ? 1 : 0;
            fix_gen_this_frame = true;
            ++fixes_generated;
            if (rec.is_outlier) ++outliers_generated;
            if (rec.from_reinit) {
                ++fixes_from_reinit;
                reinit_fix_frames.push_back(rec.frame_id);
            }
            // Queued at GENERATION time, not at emission: with a latency the
            // fix leaves later, but generation order IS emission order (FIFO),
            // so the verdict-to-row mapping below stays exact.
            pending_fixes.push_back({row_seq_next, rec.is_outlier});
        });
        fake_anchor = fa.get();
        sys.setAnchor(std::move(fa));
        spdlog::warn("test_full_flight: FAKE ABSOLUTE FIXES ENABLED — fixes are "
                     "generated FROM GROUNDTRUTH; every error figure of this run "
                     "measures the back-end, not a real system");
        spdlog::info("  fake fix: sigma={:.1f} m, every {} KFs, outlier_rate={:.2f} "
                     "[{:.0f}, {:.0f}] m, latency={} KFs, seed={}, csv='{}' "
                     "(frame_id_offset={})",
                     fix_cfg.sigma_m, fix_cfg.every_kf, fix_cfg.outlier_rate,
                     fix_cfg.outlier_min_m, fix_cfg.outlier_max_m,
                     fix_cfg.latency_kf, fix_cfg.seed,
                     fix_cfg.telemetry.csv_path, fix_cfg.frame_id_offset);
    }

    if (!sys.setup() || !sys.start()) {
        spdlog::error("test_full_flight: SystemManager setup/start failed");
        return 1;
    }

    spdlog::info("test_full_flight: config='{}' out='{}' max_frames={} input_mode={} "
                 "(VO async+wait_for_local_BA, fusion sync)",
                 config_path, output_csv, max_frames_env,
                 input_mode == InputMode::CHANNELS ? "channels" : "frame");

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
        sensor::FrameStatus status;
        {
            util::ScopedTimer _t(util::ProfileStage::VIDEO_DECODE);
            status = reader.read(fd);
        }
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

        // One call runs the whole pipeline; `res` / `fres` are refreshed by the
        // debug-channel subscribers before it returns. dt_ms therefore also
        // covers the event publication, unlike the pre-S4 driver — it is a
        // wall-clock measurement and never was reproducible anyway.
        have_res  = false;
        have_fres = false;
        // Filled by the fake anchor's generation callback DURING onFrame().
        fix_e_col          = nan;
        fix_n_col          = nan;
        fix_outlier_col    = 0;
        fix_gen_this_frame = false;
        const auto t0 = clock::now();
        // The ONLY difference between the two modes. Everything past
        // SystemManager::onFrame() is the same code either way.
        const bool accepted = (input_mode == InputMode::CHANNELS)
                                  ? feed_as_channels(sys, fd)
                                  : sys.onFrame(fd);
        const double dt_ms =
            std::chrono::duration<double, std::milli>(clock::now() - t0).count();
        if (!accepted || !have_res || !have_fres) {
            spdlog::error("test_full_flight: SystemManager refused frame {} "
                          "(accepted={}, vo={}, fusion={}) — stopping",
                          fd.frame_id, accepted, have_res, have_fres);
            break;
        }
        const sensor::TelemetryData telem =
            fd.has_telemetry ? fd.telemetry : sensor::TelemetryData{};
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
                    err2d_samples.push_back(err_2d);
                    err2d_path_m.push_back(gt_path_m);
                }
            }
        }

        // Cumulative groundtruth path length (G1a regressor + fix spacing).
        if (gt_anchored && telem_ok && std::isfinite(gt_e) && std::isfinite(gt_n)) {
            if (have_prev_gt) {
                gt_path_m += std::hypot(gt_e - prev_gt_e, gt_n - prev_gt_n);
            }
            prev_gt_e   = gt_e;
            prev_gt_n   = gt_n;
            have_prev_gt = true;
        }

        // Fused-trajectory jerk: the biggest step between two consecutive
        // frames whose second frame carried a graph update.
        if (fres.has_pose) {
            if (have_prev_pred && fres.graph_updated) {
                max_kf_jump_m = std::max(max_kf_jump_m, (f_t - prev_pred).norm());
            }
            prev_pred      = f_t;
            have_prev_pred = true;
        }

        // ── absolute-fix accounting ──────────────────────────────────────────
        // Verdicts of previously pushed fixes, read as counter deltas — one
        // delta per drop reason since the counters were split.
        {
            const fusion::FusionFixStats st = sys.fixStats();
            resolve_fixes(st.applied      - prev_fstats.applied,      "applied",      false);
            resolve_fixes(st.gated        - prev_fstats.gated,        "gated",        true);
            resolve_fixes(st.low_confidence - prev_fstats.low_confidence,
                          "low_confidence", true);
            resolve_fixes(st.age_expired  - prev_fstats.age_expired,  "age_expired",  false);
            resolve_fixes(st.unmatched    - prev_fstats.unmatched,    "unmatched",    false);
            resolve_fixes(st.marginalized - prev_fstats.marginalized, "marginalized", false);
            resolve_fixes(st.queue_dropped - prev_fstats.queue_dropped,
                          "queue_dropped", false);
            resolve_fixes(st.no_graph     - prev_fstats.no_graph,     "no_graph",     false);
            prev_fstats = st;
            cur_fstats  = st;
        }

        // Fix SPACING — the only fix bookkeeping that has to wait for the frame
        // to be scored, because it is measured in groundtruth path length and
        // gt_path_m is only current at this point. Whether a fix was produced
        // (and the cadence that decided it) is the producer's business.
        if (fix_gen_this_frame) {
            if (fix_have_last_path) {
                fix_spacing_m.push_back(gt_path_m - fix_last_path_m);
            }
            fix_last_path_m    = gt_path_m;
            fix_have_last_path = true;
        }

        std::ostringstream row;
        row << std::setprecision(CSV_PRECISION);
        row << res.frame_id << ','
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
            << res.median_depth_num_lms << ','
            << fix_e_col << ',' << fix_n_col << ',' << fix_outlier_col << ',';

        // Appended gate-observability columns (see the CSV header). `s` is the
        // distance flown since the last APPLIED fix — the quantity the drift
        // term of the gate is built on — and the two residual columns describe
        // the most recent rejection.
        std::ostringstream row_suffix;
        row_suffix << std::setprecision(CSV_PRECISION);
        row_suffix << ',' << cur_fstats.distance_since_fix_m
                   << ',' << cur_fstats.last_gated_residual_m
                   << ',' << cur_fstats.last_gated_gate_radius_m
                   << ',' << cur_fstats.age_expired
                   << ',' << cur_fstats.unmatched
                   << ',' << cur_fstats.marginalized
                   << ',' << cur_fstats.queue_dropped;
        row_buf.push_back({row.str(), "none", row_suffix.str()});
        ++row_seq_next;
        flush_rows();

        if (acc_csv.is_open()) {
            acc_csv << res.frame_id << ',' << loc.accuracy_m << ',' << err_2d << ','
                    << health_name(fres.health) << ','
                    << static_cast<int>(loc.valid) << '\n';
        }

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
    sys.stop();
    reader.close();

    // Final verdicts (the last keyframe update may have resolved fixes), then
    // drain the row buffer: anything still unresolved keeps status "none".
    {
        const fusion::FusionFixStats st = sys.fixStats();
        resolve_fixes(st.applied      - prev_fstats.applied,      "applied",      false);
        resolve_fixes(st.gated        - prev_fstats.gated,        "gated",        true);
        resolve_fixes(st.low_confidence - prev_fstats.low_confidence,
                      "low_confidence", true);
        resolve_fixes(st.age_expired  - prev_fstats.age_expired,  "age_expired",  false);
        resolve_fixes(st.unmatched    - prev_fstats.unmatched,    "unmatched",    false);
        resolve_fixes(st.marginalized - prev_fstats.marginalized, "marginalized", false);
        resolve_fixes(st.queue_dropped - prev_fstats.queue_dropped,
                      "queue_dropped", false);
        resolve_fixes(st.no_graph     - prev_fstats.no_graph,     "no_graph",     false);
        prev_fstats = st;
    }
    for (const auto& r : row_buf) {
        csv << r.prefix << r.status << r.suffix << '\n';
    }
    row_buf.clear();
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
    spdlog::info("  input_mode        = {}",
                 input_mode == InputMode::CHANNELS ? "channels" : "frame");
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
    spdlog::info("  p95 err_2d        = {:.2f} m", percentile(err2d_samples, 0.95));
    spdlog::info("  max err_2d        = {:.2f} m",
                 err2d_samples.empty()
                     ? nan
                     : *std::max_element(err2d_samples.begin(), err2d_samples.end()));
    spdlog::info("  err_2d vs GT path = {:+.2f} m/km   (least-squares slope; "
                 "GT path {:.2f} km)",
                 1000.0 * least_squares_slope(err2d_path_m, err2d_samples),
                 gt_path_m * 1e-3);
    spdlog::info("  max pos jump      = {:.2f} m   (consecutive frames, "
                 "graph_updated)", max_kf_jump_m);

    const fusion::FusionFixStats fstats = sys.fixStats();
    spdlog::info("  fix injected      = {}", fstats.injected);
    spdlog::info("  fix applied/gated = {}/{}", fstats.applied, fstats.gated);
    spdlog::info("  fix dropped: low_confidence={} age_expired={} unmatched={} "
                 "marginalized={} queue_dropped={} no_graph={}",
                 fstats.low_confidence, fstats.age_expired, fstats.unmatched,
                 fstats.marginalized, fstats.queue_dropped, fstats.no_graph);
    spdlog::info("  fix gate: enabled={}, kernel={}, drift_rate={} m/m, "
                 "min_confidence={}, s at end={:.1f} m, last gated "
                 "residual={:.1f} m vs radius={:.1f} m",
                 sys_cfg.fusion.fix_gate_enabled,
                 kernel_name(sys_cfg.fusion.fix_robust_kernel),
                 sys_cfg.fusion.fix_drift_rate_m_per_m,
                 sys_cfg.fusion.fix_min_confidence,
                 fstats.distance_since_fix_m, fstats.last_gated_residual_m,
                 fstats.last_gated_gate_radius_m);
    if (fake_anchor != nullptr) {
        spdlog::warn("  fixes were GENERATED FROM GROUNDTRUTH — the error figures "
                     "above measure the back-end, not a real system");
        spdlog::info("  fix generated     = {} ({} outliers, latency {} KFs, "
                     "sigma {:.1f} m, every {} KFs, seed {})",
                     fixes_generated, outliers_generated, fix_cfg.latency_kf,
                     fix_cfg.sigma_m, fix_cfg.every_kf, fix_cfg.seed);
        // Producer-side counters, straight from the module: they are the only
        // way to tell "the cadence skipped it" from "there was no groundtruth".
        const anchor::FakeAnchorStats ast = fake_anchor->stats();
        spdlog::info("  anchor requests   = {} (generated={}, emitted={}, skipped: "
                     "cadence={} no_telemetry={} no_groundtruth={} no_origin={})",
                     ast.requested, ast.generated, ast.emitted,
                     ast.skipped_cadence, ast.skipped_no_telemetry,
                     ast.skipped_no_groundtruth, ast.skipped_no_origin);
        // Cadence fixes vs re-anchor fixes, never summed into one figure.
        {
            std::ostringstream frames;
            for (std::size_t i = 0; i < reinit_fix_frames.size(); ++i) {
                if (i != 0) frames << ' ';
                frames << reinit_fix_frames[i];
            }
            spdlog::info("  fix by reason     = cadence {} | re-anchor {}   "
                         "(reinit requests={}, throttled={}, "
                         "reinit_min_kf_gap={}); re-anchor frames: [{}]",
                         fixes_generated - fixes_from_reinit, fixes_from_reinit,
                         ast.reinit_requested, ast.reinit_throttled,
                         fix_cfg.reinit_min_kf_gap, frames.str());
        }
        spdlog::info("  VO re-inits       = {} (lost {}) — one re-anchor request "
                     "is issued per re-init", reinit_events, lost_events);
        spdlog::info("  outlier catch     = {}/{} ({:.1f} %)",
                     outliers_caught, outliers_generated,
                     (outliers_generated > 0)
                         ? 100.0 * outliers_caught / outliers_generated : 0.0);
        spdlog::info("  fix spacing       = median {:.1f} m, p95 {:.1f} m   "
                     "(GT path length between consecutive fixes)",
                     percentile(fix_spacing_m, 0.5), percentile(fix_spacing_m, 0.95));
    }
    spdlog::info("  csv               = '{}'", output_csv);
    spdlog::info("==================================");

    if (profile_enabled) {
        dump_profile(profile_csv_path(output_csv));
    }

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
