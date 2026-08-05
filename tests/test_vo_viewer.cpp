// test_vo_viewer — LIVE 3D viewer driver, driven by core::SystemManager (S8).
//
// Since S8 the driver no longer owns vo::VOModule / fusion::FusionModule / a
// reader loop. It builds ONE core::SystemManager, hands it a push-style
// sensor::VideoDataSource, subscribes to sys.callbacks() and BRIDGES those
// events into debug_viewer::DebugViewer. That bridge lives here, in the
// application layer, on purpose: libuavloc must not link uavloc_debug_viewer
// (design R4), so the system publishes typed events and the driver decides
// what to draw.
//
//   * The pipeline runs on the SOURCE's streaming thread (async_input = false
//     ⇒ VideoDataSource reads a frame and runs the whole pipeline inline
//     before reading the next one — the same pacing the old worker loop had,
//     and no frame is ever dropped). DebugViewer::run() owns the MAIN thread
//     (GL singleton, blocking live window). A supervisor thread waits for the
//     end of data (or the UAVLOC_VO_MAXFRAMES cap) and calls sys.stop(); the
//     window then STAYS OPEN with the full visualization until the user closes
//     it, exactly as before.
//   * viewer.loadBatch({}) -> live push mode. Per processed frame the driver
//     pushes the groundtruth pose (telemetry lat/lon/alt -> ENU via
//     gps_to_enu.h) and, once a 4-DoF gravity-aligned (yaw + translation,
//     roll/pitch locked by the landmark-plane normal) VO-world -> groundtruth-
//     ENU transform is frozen over an initial alignment window of TRACKING
//     frames, the aligned VO estimate (pushPose) and the latest keyframe
//     map-point cloud (pushMapPoints). A free 3-DoF-rotation Umeyama is
//     degenerate here: the near-collinear camera window leaves the roll about
//     the flight axis unconstrained, tilting the whole map.
//   * Headless-safe: viewer.run() returns immediately with no DISPLAY.
//     Everything is behind #ifdef UAVLOC_WITH_DEBUG_VIEWER; without it the
//     pipeline just runs to completion (or the cap) and exits 0.
//   * Soft-skips (returns 0) when the gitignored dataset video is unavailable.
//   * Side panels unchanged: per-frame FrameMetrics (inliers/landmarks plots +
//     HUD heading/distance), a "Tracking" sub-window overlaying the tracked
//     observations on the frame, LOST-pose markers, the "Profiling" table and a
//     "Performance" panel fed by a /proc-based CPU%/RSS sampler thread.
//   * "Error (m)" panel: |fused - groundtruth| of the SAME frame, as a per-frame
//     line chart + a distribution histogram + a live count/mean/median/p95/max
//     summary. Enable with DebugViewer.show_error (or the checkbox). ⚠ It is a
//     raw error — it still carries the mount-azimuth offset θ, so it is an upper
//     bound rather than a 4-DoF-aligned ATE, and with UAVLOC_FAKE_FIX=1 the
//     groundtruth it scores against is also what generates the fixes.
//
// ── Which callback does what (design §3.5 / §4.6) ────────────────────────────
// The emission order inside one processed frame is fixed:
//     on_frame_processed → on_vo_data → on_fusion_result → on_localization
//     → on_lag_window
// so the driver splits its old single-loop body along that order:
//     on_frame_processed  image + telemetry of the frame (publish_images = true)
//     on_vo_data          VOResult + map points ⇒ HUD, metrics, tracking
//                         overlay, LOST markers, groundtruth/estimate/alignment,
//                         map-point cloud, profiling snapshot
//     on_fusion_result    fused display pose (anchor-translated)
//     on_lag_window       smoother correction of the fused line
//
// ── Fake absolute fixes (M1) — OFF by default ────────────────────────────────
// UAVLOC_FAKE_FIX=1 attaches anchor::FakeAnchor to the SystemManager, exactly
// as tests/test_full_flight.cpp does, and uses the SAME UAVLOC_FIX_* variable
// names (SIGMA_M / EVERY_KF / OUTLIER_RATE / OUTLIER_MIN_M / OUTLIER_MAX_M /
// LATENCY_KF / SEED / REINIT_MIN_KF). The point here is to SEE the fixes:
//     * a marker at every fix position, mapped with the same g0 + (p - f0)
//       anchor translation as the fused line so it lands where it belongs;
//     * CIRCLES for the regular cadence — YELLOW where the back-end applied the
//       measurement, PURPLE where it rejected it or has not judged it yet
//       (verdicts come from fusion::FusionFixStats, polled by the supervisor
//       thread);
//     * SQUARES for a RE-ANCHOR fix, i.e. one requested because the VO chain
//       had just been re-initialized — GREEN applied, ORANGE rejected/pending;
//     * a counter line (generated / accepted / rejected / pending, plus the
//       re-anchor pair) and a PERMANENT RED BANNER in the viewer's main panel.
// ⚠ Those "measurements" are manufactured from GROUNDTRUTH. The banner is not
// decoration: a screenshot of this window must not be able to travel without
// the caveat (.claude/rules/reporting.md).
//
// ── Start/Stop button vs. auto-start ─────────────────────────────────────────
// On a DISPLAY the pipeline does NOT start by itself: the viewer draws a Start
// button (DebugViewer::setRunControl) and this driver's handler drives it.
// Start/Stop is a PAUSE toggle, pressable as often as you like:
//     first Start  -> SystemManager::start()   (the whole system comes up ONCE)
//     Stop         -> SystemManager::pauseSource()   (only the reader stops;
//                     the system stays RUNNING and the video is not rewound)
//     Start again  -> SystemManager::resumeSource()  (continues at the next frame)
// SystemManager::stop() stays what it always was — the permanent shutdown — and
// is called only when the data ends or the window is closed.
// The run auto-starts, with no button involved, whenever nobody could press it:
//     * headless (no DISPLAY and no WAYLAND_DISPLAY) — run() returns at once;
//     * UAVLOC_VIEWER_DUMP set — the deterministic regression gate;
//     * UAVLOC_VIEWER_AUTOSTART=1 — force it even with a GUI.
// Without UAVLOC_WITH_DEBUG_VIEWER there is no viewer at all, so it always
// auto-starts. The chosen mode and its reason are logged.
//
// Usage (from build/):
//     ./tests/test_vo_viewer [config.yaml]
//     UAVLOC_VO_MAXFRAMES=200 ./tests/test_vo_viewer [config.yaml]   # capped run
//     UAVLOC_VIEWER_DUMP=poses.csv ./tests/test_vo_viewer …          # see below
//     UAVLOC_VIEWER_AUTOSTART=1 ./tests/test_vo_viewer …             # skip the button
//     UAVLOC_FAKE_FIX=1 ./tests/test_vo_viewer …                     # see above
// Run with a DISPLAY to open the live window (trajectory + groundtruth + cloud).
#include "uavloc/anchor/fake_anchor.h"
#include "uavloc/core/system_config.h"
#include "uavloc/core/system_manager.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/sensor/video_data_source.h"
#include "uavloc/sensor/video_reader.h"
#include "uavloc/util/scoped_timer.h"

#include "driver_common.h"

#include <Eigen/Core>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef UAVLOC_WITH_DEBUG_VIEWER
#include "uavloc/debug_viewer/debug_viewer.h"
#include "uavloc/debug_viewer/inferred_pose.h"
#include "uavloc/debug_viewer/viewer_metrics.h"
#include "uavloc/util/sys_monitor.h"
#include "gps_to_enu.h"    // src/debug_viewer (on this target's include path)
#include <Eigen/Geometry>  // Eigen::umeyama, Quaterniond, AngleAxisd
#include <Eigen/SVD>       // JacobiSVD (landmark-plane fit)
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <tuple>
#include <utility>
#endif

namespace uavloc {
namespace {

const std::string DEFAULT_CONFIG_PATH =
    "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/config/uavloc_yenbai800m_newvo.yaml";

// Poll period [ms] of the supervisor thread that watches for end-of-data /
// the frame cap. Not a pipeline parameter — only how fast teardown reacts.
constexpr int SUPERVISOR_POLL_MS = 20;

// The fake-anchor switches (the eleven UAVLOC_FIX_* / UAVLOC_FAKE_FIX names)
// and the forced run mode are resolved by tests/driver_common.{h,cpp}, shared
// with tests/test_full_flight.cpp: one command line must mean one experiment in
// both drivers, and both must run the pipeline in the same mode or their pose
// sequences cannot be compared (tests/test_driver_parity.cpp).
using uavloc::eval::env_int;
using uavloc::eval::fake_anchor_config;
using uavloc::eval::force_offline_run_mode;

//! The sentence that must stay on screen for as long as fake fixes are being
//! injected. Not a log line: a screenshot of the window has to carry it.
const char* const FAKE_ANCHOR_BANNER =
    "FAKE ANCHOR DANG BAT - cac diem 'fix' sinh tu GROUNDTRUTH, "
    "KHONG phai hieu nang cua he thong that.";

//! One absolute fix as this driver tracks it: the position the producer
//! reported (FUSION ENU — the same frame as fusion::FusionResult::T_enu_c) plus
//! the consumer's verdict, which only arrives a few frames later.
struct AnchorFixRecord {
    Eigen::Vector3d pos_fusion = Eigen::Vector3d::Zero();
    bool            resolved   = false;
    bool            accepted   = false;
    //! Produced by a RE-ANCHOR request (VO had just re-initialized) rather than
    //! by the regular cadence — drawn as a square marker and counted apart.
    bool            from_reinit = false;
};

#ifdef UAVLOC_WITH_DEBUG_VIEWER
// Digits written to the UAVLOC_VIEWER_DUMP CSV (S8 gate B).
constexpr int DUMP_PRECISION = 12;

//! True iff the environment variable is set AND non-empty. `DISPLAY=` yields a
//! non-null but empty string, which DebugViewer::run() already treats as
//! headless; the run-mode decision here must agree with it exactly, or the
//! driver would wait for a button in a window that was never opened.
bool env_nonempty(const char* key) {
    const char* v = std::getenv(key);
    return v != nullptr && v[0] != '\0';
}

// ── HUD heading formula (mirrors the old test_vo_pipeline) ───────────────────
// Degrees per full turn — used to wrap heading angles into [0, 360).
constexpr double DEGREES_PER_TURN = 360.0;
// Radians -> degrees for the course-over-ground heading.
constexpr double DEG_PER_RAD = 180.0 / M_PI;

// Wrap an angle in degrees into [0, 360).
double wrap360(double deg) {
    deg = std::fmod(deg, DEGREES_PER_TURN);
    if (deg < 0.0) deg += DEGREES_PER_TURN;
    return deg;
}

// Tracking-overlay presentation constants (debug-viewer sub-window only — NOT
// pipeline thresholds): circle radius for a tracked landmark keypoint, and the
// annotation text placement/scale.
constexpr int    TRACK_KP_RADIUS_PX   = 4;   // tracked-landmark circle radius (px)
constexpr int    TRACK_KP_THICKNESS   = 2;   // tracked-landmark circle stroke (px)
constexpr int    TRACK_TEXT_X_PX      = 10;
constexpr int    TRACK_TEXT_Y_PX      = 30;
constexpr double TRACK_TEXT_SCALE     = 0.8;
constexpr int    TRACK_TEXT_THICKNESS = 2;

// ── 4-DoF alignment helpers ──────────────────────────────────────────────────
// Least-squares plane fit through `pts`: centroid + unit normal = the singular
// vector of the smallest singular value of the centered 3xN matrix. Returns
// false when fewer than 3 points are available (plane underdetermined).
bool fitPlane(const std::vector<Eigen::Vector3d>& pts,
              Eigen::Vector3d& centroid, Eigen::Vector3d& normal) {
    if (pts.size() < 3) {
        return false;
    }
    centroid = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& p : pts) {
        centroid += p;
    }
    centroid /= static_cast<double>(pts.size());
    Eigen::Matrix3Xd M(3, static_cast<Eigen::Index>(pts.size()));
    for (std::size_t i = 0; i < pts.size(); ++i) {
        M.col(static_cast<Eigen::Index>(i)) = pts[i] - centroid;
    }
    Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(M, Eigen::ComputeFullU);
    normal = svd.matrixU().col(2).normalized();
    return true;
}

// Tilt (degrees, folded to <= 90) between the ENU Up axis and the normal of the
// best-fit plane through `pts` AFTER applying the rigid transform T. Negative
// when the plane cannot be fitted (too few points).
double alignedPlaneTiltDeg(const std::vector<Eigen::Vector3d>& pts,
                           const Eigen::Matrix4d& T) {
    std::vector<Eigen::Vector3d> transformed;
    transformed.reserve(pts.size());
    for (const Eigen::Vector3d& p : pts) {
        transformed.push_back((T * p.homogeneous()).hnormalized());
    }
    Eigen::Vector3d c, n;
    if (!fitPlane(transformed, c, n)) {
        return -1.0;
    }
    const double cos_ang =
        std::min(1.0, std::abs(n.dot(Eigen::Vector3d::UnitZ())));
    return std::acos(cos_ang) * DEG_PER_RAD;
}

// Horizontal (E,N) bounding-box diagonal of a point set, in metres. Used by the
// optional `alignment_min_spread_m` freeze gate.
double horizontalSpreadM(const std::vector<Eigen::Vector3d>& pts) {
    if (pts.empty()) {
        return 0.0;
    }
    double min_e = pts.front().x(), max_e = min_e;
    double min_n = pts.front().y(), max_n = min_n;
    for (const Eigen::Vector3d& p : pts) {
        min_e = std::min(min_e, p.x());
        max_e = std::max(max_e, p.x());
        min_n = std::min(min_n, p.y());
        max_n = std::max(max_n, p.y());
    }
    return std::hypot(max_e - min_e, max_n - min_n);
}
#endif  // UAVLOC_WITH_DEBUG_VIEWER

}  // namespace
}  // namespace uavloc

int main(int argc, char** argv) {
    using namespace uavloc;

    // Honour SPDLOG_LEVEL for a verbose per-frame run.
    spdlog::cfg::load_env_levels();

    const std::string config_path =
        (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    // Optional hard cap on frames fed to the pipeline (headless smoke). 0 = run to
    // END_OF_STREAM. Mirrors test_vo_pipeline_new's UAVLOC_VO_MAXFRAMES.
    const char* mf = std::getenv("UAVLOC_VO_MAXFRAMES");
    const std::size_t max_frames_env =
        mf ? static_cast<std::size_t>(std::max(0, std::atoi(mf))) : 0u;

    // Stage profiling is opt-in (UAVLOC_PROFILE=1) and read-only: with it off
    // the pipeline behaves exactly as before. When on, the pipeline thread
    // publishes a per-frame snapshot into the viewer's "Profiling" table.
    const char* prof_env = std::getenv("UAVLOC_PROFILE");
    const bool profile_enabled = prof_env != nullptr && std::atoi(prof_env) != 0;
    util::Profiler::set_enabled(profile_enabled);

    // ── UAVLOC_VIEWER_DUMP — the deterministic headless verification path ────
    // This driver is a live tool: worker thread + GUI, so it has no
    // bit-identical gate of its own. When this variable names a file, the run
    // writes the sequence of FUSED DISPLAY POSES (exactly the values handed to
    // DebugViewer::pushFusedPose) as CSV. Since S9 the deterministic run mode
    // (inline input + synchronous fusion + VO async with the local-BA
    // handshake) is forced UNCONDITIONALLY by force_offline_run_mode(), not
    // only here — so a dump run and a plain run are the same experiment.
    // Combine with `env -u DISPLAY` and UAVLOC_VO_MAXFRAMES for a fully
    // deterministic comparison run.
    const char* dump_env = std::getenv("UAVLOC_VIEWER_DUMP");
    const std::string dump_path = (dump_env != nullptr) ? std::string(dump_env)
                                                        : std::string();

    // ── Run mode: viewer Start button vs. auto-start ─────────────────────────
    // Auto-start is the DEFAULT and only the GUI path opts out of it: every
    // situation where nobody can click the button must run by itself, or the
    // regression gates that drive this binary would hang forever.
    bool        autostart        = true;
    std::string autostart_reason = "built without the debug viewer";
#ifdef UAVLOC_WITH_DEBUG_VIEWER
    if (!env_nonempty("DISPLAY") && !env_nonempty("WAYLAND_DISPLAY")) {
        autostart_reason = "headless (no DISPLAY/WAYLAND_DISPLAY)";
    } else if (!dump_path.empty()) {
        autostart_reason = "UAVLOC_VIEWER_DUMP set (deterministic gate)";
    } else if (env_int("UAVLOC_VIEWER_AUTOSTART", 0) != 0) {
        autostart_reason = "UAVLOC_VIEWER_AUTOSTART=1";
    } else {
        autostart = false;
    }
#endif

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
        sys_cfg    = core::SystemConfig::fromYaml(yaml);  // VO + Fusion + Camera + System
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return 1;
    }
    // The viewer draws the frame and the tracking overlay, so it needs the
    // image on the debug channel (off by default — design R-e). This is the ONE
    // configuration key that legitimately differs from test_full_flight: it only
    // decides whether the image rides along on the debug channel.
    sys_cfg.publish_images = true;
    // ALWAYS, not just under UAVLOC_VIEWER_DUMP. Until S9 this driver forced the
    // synchronous input/fusion path only in the dump branch and never forced the
    // VO mode at all, so a viewer run and a test_full_flight run on the same
    // config were two different experiments that happened to agree. Forcing the
    // same four keys in both makes that agreement a property instead of an
    // observation — and tests/test_driver_parity.cpp checks it.
    force_offline_run_mode(sys_cfg);

    // Opened HERE rather than inside VideoDataSource so a missing (gitignored)
    // dataset stays a soft skip instead of a start() failure. VideoDataSource
    // only opens what is not open yet.
    auto reader = std::make_unique<sensor::VideoReader>(reader_cfg);
    if (!reader->open()) {
        spdlog::warn("test_vo_viewer: cannot open video '{}' — SKIPPED",
                     reader_cfg.video_path);
        return 0;  // soft-skip when the dataset is absent
    }
    spdlog::info("Video opened — fps: {:.1f}, total frames: {}",
                 reader->getFps(), reader->getFrameCount());

    // The whole pipeline behind one object; the source is OWNED by it, so the
    // start/stop ordering is enforced there and not re-invented here (§4.5).
    core::SystemManager sys(sys_cfg);
    sys.attachSource(std::make_unique<sensor::VideoDataSource>(
        std::move(reader), sensor::VideoDataSourceConfig::fromYaml(yaml["VideoDataSource"])));

    // Shared shutdown flag between the supervisor, the perf sampler and the
    // (optional) viewer on the main thread.
    std::atomic<bool> stop_requested{false};

    std::atomic<std::size_t> frames_processed{0};
    std::atomic<std::size_t> pose_successes{0};
    // Tracking state of the last processed frame (== VOModule::get_state()).
    std::atomic<int> final_state{
        static_cast<int>(vo::VOTrackingState::NOT_INITIALIZED)};

    // ── UAVLOC_VO_MAXFRAMES, made EXACT ──────────────────────────────────────
    // The supervisor stops the system as soon as it sees the count, but that
    // takes a poll period, so a few extra frames can still run through the
    // pipeline. These two make the cap exact on the OUTPUT side: everything
    // past the cap-th frame is ignored by the bridge, keyed by frame id so a
    // fusion result that arrives late (async fusion) is judged by the frame it
    // belongs to, not by when it landed.
    std::atomic<bool>          cap_reached{false};
    std::atomic<unsigned long long> cap_last_frame_id{0};
    auto beyond_cap_index = [&](std::size_t index) {
        return max_frames_env > 0 && index > max_frames_env;
    };

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    namespace dv = uavloc::debug_viewer;

    auto beyond_cap_frame = [&](unsigned long long frame_id) {
        return cap_reached.load() && frame_id > cap_last_frame_id.load();
    };

    // Alignment window: number of initial TRACKING frames over which the (frozen)
    // VO-world -> groundtruth-ENU rigid transform is computed (4-DoF gravity-
    // aligned, scale fixed = 1 since VO is metric). Loaded from YAML so it is
    // not a magic number.
    const int alignment_window = std::max(
        1, yaml["DebugViewer"]["alignment_window_frames"].as<int>(50));
    // Optional freeze gate: when > 0, delay freezing the alignment until the
    // groundtruth window spans at least this many metres horizontally (guards
    // the yaw estimate against a near-stationary window). 0 = off (default,
    // preserves the count-only behaviour).
    const double alignment_min_spread_m =
        yaml["DebugViewer"]["alignment_min_spread_m"].as<double>(0.0);

    // Viewer config: forward the display/vertical scale + side-panel keys from
    // the YAML DebugViewer: section (defaults preserved when keys are absent).
    dv::DebugViewer::Config viewer_cfg;
    viewer_cfg.display_scale =
        yaml["DebugViewer"]["display_scale"].as<float>(viewer_cfg.display_scale);
    viewer_cfg.vertical_scale =
        yaml["DebugViewer"]["vertical_scale"].as<float>(viewer_cfg.vertical_scale);
    viewer_cfg.video_stride =
        yaml["DebugViewer"]["video_stride"].as<int>(viewer_cfg.video_stride);
    viewer_cfg.log_capacity =
        yaml["DebugViewer"]["log_capacity"].as<int>(viewer_cfg.log_capacity);
    viewer_cfg.metric_plot_history =
        yaml["DebugViewer"]["metric_plot_history"].as<int>(viewer_cfg.metric_plot_history);
    viewer_cfg.metric_plot_follow_window =
        yaml["DebugViewer"]["metric_plot_follow_window"].as<int>(viewer_cfg.metric_plot_follow_window);
    viewer_cfg.histogram_bins =
        yaml["DebugViewer"]["histogram_bins"].as<int>(viewer_cfg.histogram_bins);
    viewer_cfg.perf_plot_history =
        yaml["DebugViewer"]["perf_plot_history"].as<int>(viewer_cfg.perf_plot_history);
    viewer_cfg.show_log =
        yaml["DebugViewer"]["show_log"].as<bool>(viewer_cfg.show_log);
    viewer_cfg.show_inliers =
        yaml["DebugViewer"]["show_inliers"].as<bool>(viewer_cfg.show_inliers);
    viewer_cfg.show_landmarks =
        yaml["DebugViewer"]["show_landmarks"].as<bool>(true);
    viewer_cfg.show_video =
        yaml["DebugViewer"]["show_video"].as<bool>(viewer_cfg.show_video);
    viewer_cfg.show_tracking =
        yaml["DebugViewer"]["show_tracking"].as<bool>(viewer_cfg.show_tracking);
    viewer_cfg.show_lost_poses =
        yaml["DebugViewer"]["show_lost_poses"].as<bool>(viewer_cfg.show_lost_poses);
    viewer_cfg.show_hud =
        yaml["DebugViewer"]["show_hud"].as<bool>(viewer_cfg.show_hud);
    viewer_cfg.show_perf =
        yaml["DebugViewer"]["show_perf"].as<bool>(viewer_cfg.show_perf);
    // "Profiling" table panel; the table stays empty unless UAVLOC_PROFILE=1.
    viewer_cfg.show_profile =
        yaml["DebugViewer"]["show_profile"].as<bool>(viewer_cfg.show_profile);
    // "Error (m)" panel: predicted-vs-groundtruth error line chart + histogram
    // + running summary. Fed by push_error_sample() below.
    viewer_cfg.show_error =
        yaml["DebugViewer"]["show_error"].as<bool>(viewer_cfg.show_error);
    viewer_cfg.show_estimate =
        yaml["DebugViewer"]["show_estimate"].as<bool>(viewer_cfg.show_estimate);
    viewer_cfg.show_groundtruth =
        yaml["DebugViewer"]["show_groundtruth"].as<bool>(viewer_cfg.show_groundtruth);
    viewer_cfg.est_odom =
        yaml["DebugViewer"]["est_odom"].as<bool>(viewer_cfg.est_odom);
    viewer_cfg.est_odom_every_n =
        yaml["DebugViewer"]["est_odom_every_n"].as<int>(viewer_cfg.est_odom_every_n);
    viewer_cfg.est_odom_axes_scale =
        yaml["DebugViewer"]["est_odom_axes_scale"].as<float>(viewer_cfg.est_odom_axes_scale);
    viewer_cfg.show_fused =
        yaml["DebugViewer"]["show_fused"].as<bool>(viewer_cfg.show_fused);
    const int video_stride = std::max(1, viewer_cfg.video_stride);
    // Minimum per-frame baseline (metres) before the COG heading is refreshed
    // (anti-jitter when hovering / near-stationary). Same key as the old test.
    const double hud_min_baseline_m =
        yaml["DebugViewer"]["hud_min_baseline_m"].as<double>(0.5);
    // Period (ms) of the /proc CPU%/RSS performance sampler thread.
    const int perf_sample_ms =
        std::max(50, yaml["DebugViewer"]["perf_sample_ms"].as<int>(500));
    dv::DebugViewer viewer(viewer_cfg);
    viewer.loadBatch({});  // empty -> live push mode (holds window, drains queues)

    // Capture spdlog output into the viewer's scrolling log panel.
    spdlog::default_logger()->sinks().push_back(viewer.logSink());

    // Wall-clock reference for the "Profiling" table timestamps.
    const auto profile_t0 = std::chrono::steady_clock::now();

    // Count of fused poses forwarded to the viewer (logged at shutdown).
    std::atomic<std::size_t> fused_pushed{0};
    // Count of lag-window corrections forwarded to the viewer (one per
    // keyframe smoother update after the display anchor exists).
    std::atomic<std::size_t> fused_corrections{0};

    // ── PIPELINE-THREAD state ────────────────────────────────────────────────
    // Everything below is written and read ONLY inside on_frame_processed /
    // on_vo_data, which SystemManager emits back to back on the same thread for
    // one frame — so no lock is needed (the old driver's map_mutex guarded a
    // callback that fired on a different thread; that split is gone).

    // Image + telemetry of the frame in flight, carried from on_frame_processed
    // to on_vo_data (the VO channel has the pose but not the picture).
    cv::Mat               frame_image;
    sensor::TelemetryData frame_telemetry;
    bool                  frame_has_telemetry = false;

    // Latest keyframe map-point cloud (VO world, metres).
    std::vector<Vec3_t> latest_map_points;
    bool                map_dirty = false;

    // Alignment accumulation state.
    std::vector<Eigen::Vector3d> align_src;   // p_cam in the VO world frame
    std::vector<Eigen::Vector3d> align_dst;   // matching groundtruth ENU
    // Pending estimates buffered until the alignment freezes. The FULL T_wc is
    // kept (not just the position) so the flushed poses carry roll/pitch/yaw
    // for the viewer's "EstOdom" pose-axes rendering.
    std::vector<std::pair<int, Eigen::Matrix4d>,
                Eigen::aligned_allocator<std::pair<int, Eigen::Matrix4d>>>
        est_buffer;
    std::vector<std::pair<int, Eigen::Vector3d>> lost_buffer; // pending LOST markers
    Eigen::Matrix4d T_align  = Eigen::Matrix4d::Identity();
    bool   aligned           = false;
    bool   gt_origin_set     = false;
    double gt_o_lat = 0.0, gt_o_lon = 0.0, gt_o_alt = 0.0;

    // HUD bookkeeping: last plotted VO-world position, accumulated metric path
    // length, and the held course-over-ground heading.
    Eigen::Vector3d last_pos            = Eigen::Vector3d::Zero();
    bool            has_last_pos        = false;
    double          trajectory_length_m = 0.0;
    double          last_cog_heading_deg = 0.0;

    // ── Fused-trajectory display anchor ──────────────────────────────────────
    // The fused pose is metric ENU but anchored at (0, 0, agl_0) with a constant
    // unknown yaw offset (the mount azimuth θ) — by design in F1. For display we
    // only ANCHOR-TRANSLATE: capture the FIRST fused position f0 and the
    // groundtruth ENU position g0 current at that moment, then push
    // g0 + (f - f0). The residual constant yaw offset between the fused and GT
    // lines is EXPECTED (θ is unobservable until VPR/F2) — do NOT rotate it
    // away; seeing it is diagnostic. Guarded by fused_mutex: the pipeline thread
    // publishes latest_gt_enu while the fusion thread reads it (async fusion).
    std::mutex      fused_mutex;
    bool            latest_gt_valid = false;
    Eigen::Vector3d latest_gt_enu   = Eigen::Vector3d::Zero();
    bool            fused_f0_set    = false;
    bool            fused_g0_set    = false;
    Eigen::Vector3d fused_f0        = Eigen::Vector3d::Zero();
    Eigen::Vector3d fused_g0        = Eigen::Vector3d::Zero();
    // Fused poses arriving before any groundtruth exists are buffered until g0
    // can be captured, then flushed anchor-translated (position + orientation;
    // the frame id rides along for the viewer's correction splicing).
    std::vector<std::tuple<uint64_t, Eigen::Vector3d, Eigen::Quaterniond>>
        fused_pending;
    // Newest fused position in FUSION ENU — the altitude an absolute-fix marker
    // is drawn at (a fix is horizontal-only, so it has no Z of its own).
    Eigen::Vector3d latest_fused_pos = Eigen::Vector3d::Zero();
    bool            latest_fused_set = false;

    // ── "Error (m)" window feed: EXACT frame-id pairing ──────────────────────
    // The error must compare the fused pose of frame k against the groundtruth
    // of frame k. The display anchor deliberately uses the PREVIOUS frame's GT
    // (staged_gt_*, see below), so reusing that here would fold one frame of
    // motion into every sample — several metres per frame at these speeds. This
    // slot therefore carries THIS frame's GT plus its frame id, and the fusion
    // callback only emits a sample when the ids match. Guarded by fused_mutex
    // (same producer/consumer threads as the anchor state).
    Eigen::Vector3d gt_exact_enu      = Eigen::Vector3d::Zero();
    uint64_t        gt_exact_frame_id = 0;
    bool            gt_exact_valid    = false;

    // ── ONE-FRAME-DEFERRED groundtruth publication ───────────────────────────
    // The old driver pushed the frame to fusion BEFORE it computed that frame's
    // groundtruth, so the fusion callback of frame k always saw the GT of frame
    // k-1 — and that is what picks g0, the anchor of the whole fused display
    // line. SystemManager emits on_vo_data BEFORE on_fusion_result, so the GT
    // is staged here and published at the START of the next frame instead. It
    // reproduces the old anchor exactly; publishing eagerly would shift the
    // entire fused line by one frame of motion.
    Eigen::Vector3d staged_gt_enu   = Eigen::Vector3d::Zero();
    bool            staged_gt_valid = false;

    // Gate-B sink: every position handed to pushFusedPose, in push order.
    // Opened only when UAVLOC_VIEWER_DUMP is set; written under fused_mutex,
    // which the fusion callback already holds around every push.
    std::ofstream fused_dump;
    if (!dump_path.empty()) {
        fused_dump.open(dump_path);
        if (fused_dump.is_open()) {
            fused_dump << std::setprecision(DUMP_PRECISION);
            fused_dump << "frame_id,pred_x,pred_y,pred_z\n";
            spdlog::info("test_vo_viewer: UAVLOC_VIEWER_DUMP='{}' — the fused "
                         "display poses of this run are written there", dump_path);
        } else {
            spdlog::error("test_vo_viewer: cannot open pose dump '{}'", dump_path);
        }
    }
    auto dump_fused = [&](uint64_t fid, const Eigen::Vector3d& p) {
        if (fused_dump.is_open()) {
            fused_dump << fid << ',' << p.x() << ',' << p.y() << ',' << p.z() << '\n';
        }
    };

    // Feed the viewer's "Error (m)" window: |predicted - groundtruth| for the
    // SAME frame, in metres. `pred` is the position already handed to
    // pushFusedPose (i.e. in the groundtruth ENU frame through the g0 + (f - f0)
    // anchor translation), so the two are directly comparable.
    //
    // MUST be called with fused_mutex held — it reads the gt_exact_* slot the
    // pipeline thread writes.
    //
    // Frames whose groundtruth id does not match are SKIPPED rather than paired
    // approximately: that includes every pose flushed out of fused_pending
    // (emitted before the first GT existed). A skipped frame is a missing bar in
    // the histogram, which is honest; a mis-paired one would be a wrong bar.
    //
    // ⚠ This error is measured against TELEMETRY groundtruth, and it still
    // contains the unresolved mount-azimuth offset θ (F1: θ is prior-only), so
    // it is an upper bound, not a 4-DoF-aligned ATE. With UAVLOC_FAKE_FIX=1 the
    // very same telemetry also generates the absolute fixes, so the number is
    // then CONTAMINATED — it may not be quoted as system accuracy.
    // Plumbing counters only (how many samples reached the window, how many
    // frames had no groundtruth of the same id) — NOT an accuracy figure, so no
    // error statistic is logged here. Read them to confirm the feed works on a
    // headless run, where the window itself is never drawn.
    std::size_t error_samples_pushed  = 0;
    std::size_t error_samples_skipped = 0;
    auto push_error_sample = [&](uint64_t fid, const Eigen::Vector3d& pred) {
        if (!gt_exact_valid || gt_exact_frame_id != fid) {
            ++error_samples_skipped;
            return;
        }
        ++error_samples_pushed;
        const Eigen::Vector3d d = pred - gt_exact_enu;
        dv::ErrorSample s;
        s.frame_id = static_cast<int>(fid);
        s.err_2d_m = static_cast<float>(d.head<2>().norm());
        s.err_3d_m = static_cast<float>(d.norm());
        viewer.pushErrorSample(s);
    };

    // Build the "Profiling" table snapshot from the merged profiler stats
    // (mean = total_ms / count) and hand it to the viewer (latest-wins).
    auto push_profile_snapshot = [&](int frame_id) {
        namespace ut = uavloc::util;
        const auto stats = ut::Profiler::snapshot_total();
        const double frame_total_ms =
            stats[static_cast<std::size_t>(ut::ProfileStage::PROCESS_FRAME_TOTAL)].total_ms;
        dv::ProfileSnapshot snap;
        snap.t_sec = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - profile_t0).count();
        snap.frame_id = frame_id;
        for (std::size_t i = 0; i < stats.size(); ++i) {
            const ut::StageStat& s = stats[i];
            if (s.count == 0) {
                continue;  // stage never ran — keeps the table compact
            }
            dv::ProfileRow row;
            row.name    = ut::to_string(static_cast<ut::ProfileStage>(i));
            row.mean_ms = static_cast<float>(s.total_ms / static_cast<double>(s.count));
            row.last_ms = static_cast<float>(s.last_ms);
            row.max_ms  = static_cast<float>(s.max_ms);
            row.count   = s.count;
            row.percent = (frame_total_ms > 0.0)
                ? static_cast<float>(100.0 * s.total_ms / frame_total_ms) : 0.0f;
            snap.rows.push_back(std::move(row));
        }
        viewer.pushProfile(snap);
    };

    // Transform a VO-world position through the frozen alignment and push it
    // as a discrete LOST-state marker (red, "Lost state poses" checkbox).
    auto push_lost = [&](int fid, const Eigen::Vector3d& p_world) {
        const Eigen::Vector3d est =
            (T_align * p_world.homogeneous()).hnormalized();
        dv::InferredPose lp;
        lp.frame_id = fid;
        lp.x = est.x(); lp.y = est.y(); lp.z = est.z();
        viewer.pushLostPose(lp);
    };

    auto push_estimate = [&](int fid, const Eigen::Matrix4d& T_wc) {
        const Eigen::Vector3d p_world = T_wc.block<3, 1>(0, 3);
        const Eigen::Vector3d est =
            (T_align * p_world.homogeneous()).hnormalized();
        dv::InferredPose ep;
        ep.frame_id = fid;
        ep.x = est.x(); ep.y = est.y(); ep.z = est.z();
        // Orientation for the viewer's "EstOdom" pose axes: rotate the VO
        // camera rotation into ENU through the frozen alignment, then extract
        // ZYX (yaw, pitch, roll) — the same convention the viewer's gizmo
        // helper composes back as Rz(yaw) * Ry(pitch) * Rx(roll).
        const Eigen::Matrix3d R_enu =
            T_align.block<3, 3>(0, 0) * T_wc.block<3, 3>(0, 0);
        const Eigen::Vector3d ypr = R_enu.eulerAngles(2, 1, 0);
        ep.yaw_deg   = ypr.x() * DEG_PER_RAD;
        ep.pitch_deg = ypr.y() * DEG_PER_RAD;
        ep.roll_deg  = ypr.z() * DEG_PER_RAD;
        viewer.pushPose(ep);
    };
#endif  // UAVLOC_WITH_DEBUG_VIEWER

    // ── channel 1: image + telemetry of the frame just processed ─────────────
    sys.callbacks().on_frame_processed.add(
        [&](double /*t_msec*/, const core::FrameProcessed& fp) {
            const std::size_t index = ++frames_processed;
            if (max_frames_env > 0 && index == max_frames_env) {
                cap_last_frame_id.store(fp.frame_id);
                cap_reached.store(true);
            }
            if (beyond_cap_index(index)) {
                return;
            }
#ifdef UAVLOC_WITH_DEBUG_VIEWER
            frame_image         = fp.image;  // cv::Mat is refcounted
            frame_telemetry     = fp.telemetry;
            frame_has_telemetry = fp.has_telemetry;

            // Stream every `video_stride` frame into the viewer's video panel.
            if (index % static_cast<std::size_t>(video_stride) == 0) {
                viewer.pushFrame(fp.image);
            }
#endif
        });

    // ── channel 2: the VO payload — the bulk of the visualization ────────────
    sys.callbacks().on_vo_data.add([&](double /*t_msec*/, const vo::VOData& data) {
        const vo::VOResult& r = data.result;
        final_state.store(static_cast<int>(r.state));
        if (beyond_cap_index(frames_processed.load())) {
            return;
        }
        if (r.has_pose) {
            ++pose_successes;
        }
#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // Publish the PREVIOUS frame's groundtruth (see the staging note).
        if (staged_gt_valid) {
            std::lock_guard<std::mutex> lk(fused_mutex);
            latest_gt_enu   = staged_gt_enu;
            latest_gt_valid = true;
            staged_gt_valid = false;
        }

        if (data.map_updated && !data.map_points.empty()) {
            latest_map_points = data.map_points;
            map_dirty         = true;
        }

        // Refresh the "Profiling" table (opt-in; no-op cost when disabled).
        if (profile_enabled) {
            push_profile_snapshot(static_cast<int>(r.frame_id));
        }

        // ── LOST markers: mark the last valid position — the point where
        //    tracking broke — so the marker sits on the estimate polyline.
        //    Push through T_align once aligned, otherwise buffer. ────────────
        if (r.state == vo::VOTrackingState::LOST && has_last_pos) {
            if (aligned) {
                push_lost(static_cast<int>(r.frame_id), last_pos);
            } else {
                lost_buffer.emplace_back(static_cast<int>(r.frame_id), last_pos);
            }
        }

        // ── HUD bookkeeping + per-frame metrics (has_pose frames) ───────────
        if (r.has_pose) {
            const Eigen::Vector3d p = r.T_wc.block<3, 1>(0, 3);
            // Path length: sum of consecutive GLOBAL position deltas. The
            // increment is identity on the first step of a (re-)initialised
            // map; at those boundaries the position can jump, so skip them.
            const bool reinit_boundary =
                r.T_prev_curr.isApprox(Eigen::Matrix4d::Identity(), 1e-9);
            const Eigen::Vector3d dp = has_last_pos
                ? Eigen::Vector3d(p - last_pos) : Eigen::Vector3d::Zero();
            if (has_last_pos && !reinit_boundary) {
                trajectory_length_m += dp.norm();
                // VO course-over-ground heading (ENU, 0 = North CW): rotate
                // the per-frame displacement into ENU through the frozen
                // Umeyama alignment (the old test used geo.R_enu_w() here).
                // Held when the baseline is too small to trust the direction.
                if (aligned && dp.norm() >= hud_min_baseline_m) {
                    const Eigen::Vector3d dp_enu =
                        T_align.block<3, 3>(0, 0) * dp;
                    last_cog_heading_deg = wrap360(
                        std::atan2(dp_enu.x(), dp_enu.y()) * DEG_PER_RAD);
                }
            }
            last_pos     = p;
            has_last_pos = true;

            // Per-frame metrics: inliers/landmarks line charts + HUD scalars.
            const float hud_heading_tel =
                frame_has_telemetry
                    ? static_cast<float>(wrap360(
                          frame_telemetry.heading_deg +
                          frame_telemetry.gimbal_pan_deg))
                    : 0.0f;
            dv::FrameMetrics fm;
            fm.frame_id        = static_cast<int>(r.frame_id);
            fm.inliers         = static_cast<float>(r.num_inliers);
            fm.landmarks       = static_cast<float>(r.num_landmarks);
            fm.heading_deg     = static_cast<float>(last_cog_heading_deg);
            fm.heading_tel_deg = hud_heading_tel;
            fm.distance_m      = static_cast<float>(trajectory_length_m);
            viewer.pushMetrics(fm);

            // Tracking sub-window: overlay the tracked landmark keypoints on
            // the frame and annotate the counts (same stride as the plain
            // video panel).
            if (frames_processed.load() % static_cast<std::size_t>(video_stride) == 0 &&
                !frame_image.empty()) {
                cv::Mat overlay = frame_image.clone();
                for (const Eigen::Vector2d& obs : r.tracked_observations) {
                    cv::circle(overlay,
                               cv::Point(static_cast<int>(std::lround(obs.x())),
                                         static_cast<int>(std::lround(obs.y()))),
                               TRACK_KP_RADIUS_PX,
                               cv::Scalar(0, 255, 0),  // green (BGR)
                               TRACK_KP_THICKNESS, cv::LINE_AA);
                }
                const std::string label =
                    "Landmarks: " + std::to_string(r.num_landmarks) +
                    "  tracked: " + std::to_string(r.tracked_observations.size());
                cv::putText(overlay, label,
                            cv::Point(TRACK_TEXT_X_PX, TRACK_TEXT_Y_PX),
                            cv::FONT_HERSHEY_SIMPLEX, TRACK_TEXT_SCALE,
                            cv::Scalar(0, 255, 255), TRACK_TEXT_THICKNESS,
                            cv::LINE_AA);
                viewer.pushTrackingFrame(overlay);
            }
        }

        // ── Overlay: estimate vs groundtruth (TRACKING frames only) ─────────
        if (r.has_pose && r.state == vo::VOTrackingState::TRACKING &&
            frame_has_telemetry && frame_telemetry.valid &&
            frame_telemetry.altitude_m > 0.0) {
            const sensor::TelemetryData& t = frame_telemetry;
            if (!gt_origin_set) {
                gt_o_lat      = t.latitude_deg;
                gt_o_lon      = t.longitude_deg;
                gt_o_alt      = t.altitude_m;
                gt_origin_set = true;
            }
            const dv::ENUPoint e = dv::gps_to_enu(
                t.latitude_deg, t.longitude_deg, t.altitude_m,
                gt_o_lat, gt_o_lon, gt_o_alt);
            const Eigen::Vector3d gt_enu(e.e, e.n, e.u);
            const Eigen::Vector3d p_cam = r.T_wc.block<3, 1>(0, 3);

            // Groundtruth is independent of alignment — push it every frame.
            dv::InferredPose gp;
            gp.frame_id = static_cast<int>(r.frame_id);
            gp.x = gt_enu.x(); gp.y = gt_enu.y(); gp.z = gt_enu.z();
            viewer.pushGroundtruthPose(gp);

            // Stage it for the fusion callback of the NEXT frame (see above).
            staged_gt_enu   = gt_enu;
            staged_gt_valid = true;

            // Publish THIS frame's GT for the error window. Separate from the
            // staged/anchor path on purpose: the anchor wants the previous
            // frame, the error wants this one. SystemManager emits on_vo_data
            // before on_fusion_result for the same frame, so the fusion
            // callback below finds a matching id.
            {
                std::lock_guard<std::mutex> lk(fused_mutex);
                gt_exact_enu      = gt_enu;
                gt_exact_frame_id = r.frame_id;
                gt_exact_valid    = true;
            }

            if (!aligned) {
                // Accumulate correspondences; buffer estimates until the frozen
                // transform is known, then flush them aligned.
                align_src.push_back(p_cam);
                align_dst.push_back(gt_enu);
                est_buffer.emplace_back(static_cast<int>(r.frame_id), r.T_wc);

                if (static_cast<int>(align_src.size()) >= alignment_window &&
                    (alignment_min_spread_m <= 0.0 ||
                     horizontalSpreadM(align_dst) >= alignment_min_spread_m)) {
                    const std::size_t n_corr = align_src.size();
                    Eigen::Matrix3Xd S(3, n_corr);
                    Eigen::Matrix3Xd D(3, n_corr);
                    for (std::size_t i = 0; i < n_corr; ++i) {
                        S.col(static_cast<Eigen::Index>(i)) = align_src[i];
                        D.col(static_cast<Eigen::Index>(i)) = align_dst[i];
                    }
                    // Diagnostic reference ONLY: the previous free-3D-rotation
                    // Umeyama (scale fixed = 1). On this near-collinear camera
                    // window its roll about the flight axis is degenerate — it
                    // is computed solely for the before/after tilt log below.
                    const Eigen::Matrix4d T_umeyama3d =
                        Eigen::umeyama(S, D, /*with_scaling=*/false);

                    // The latest keyframe map-point cloud (VO world, metres) —
                    // the landmark carpet defines the ground plane.
                    const std::vector<Eigen::Vector3d> plane_pts(
                        latest_map_points.begin(), latest_map_points.end());

                    Eigen::Vector3d plane_c, plane_n;
                    if (fitPlane(plane_pts, plane_c, plane_n)) {
                        // 1) VO-side "up": landmark-plane normal, signed from
                        //    the plane centroid TOWARD the mean camera position
                        //    of the window (the camera flies above the ground).
                        Eigen::Vector3d cam_mean = Eigen::Vector3d::Zero();
                        for (const Eigen::Vector3d& s : align_src) {
                            cam_mean += s;
                        }
                        cam_mean /= static_cast<double>(n_corr);
                        Eigen::Vector3d up_vo = plane_n;
                        if (up_vo.dot(cam_mean - plane_c) < 0.0) {
                            up_vo = -up_vo;
                        }

                        // 2) R0: rotate up_vo onto ENU Up — kills roll/pitch.
                        const Eigen::Matrix3d R0 =
                            Eigen::Quaterniond::FromTwoVectors(
                                up_vo, Eigen::Vector3d::UnitZ())
                                .toRotationMatrix();

                        // 3) Yaw about Up + translation: 2D Procrustes (no
                        //    scale) on the horizontal components of the R0-
                        //    levelled window. With C = sum(d_c * s'_c^T) the
                        //    optimal yaw is atan2(C10 - C01, C00 + C11).
                        Eigen::Vector3d s_mean = Eigen::Vector3d::Zero();
                        Eigen::Vector3d d_mean = Eigen::Vector3d::Zero();
                        std::vector<Eigen::Vector3d> s_lev(n_corr);
                        for (std::size_t i = 0; i < n_corr; ++i) {
                            s_lev[i] = R0 * align_src[i];
                            s_mean += s_lev[i];
                            d_mean += align_dst[i];
                        }
                        s_mean /= static_cast<double>(n_corr);
                        d_mean /= static_cast<double>(n_corr);
                        Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
                        for (std::size_t i = 0; i < n_corr; ++i) {
                            const Eigen::Vector2d sc =
                                (s_lev[i] - s_mean).head<2>();
                            const Eigen::Vector2d dc =
                                (align_dst[i] - d_mean).head<2>();
                            C += dc * sc.transpose();
                        }
                        const double yaw = std::atan2(C(1, 0) - C(0, 1),
                                                      C(0, 0) + C(1, 1));
                        const Eigen::Matrix3d R_yaw =
                            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                .toRotationMatrix();

                        // 4) T_align = [R_yaw*R0 | t]; t from the centroids
                        //    (E,N via the 2D fit; U = mean altitude offset —
                        //    yaw does not change the Up component).
                        T_align.setIdentity();
                        T_align.block<3, 3>(0, 0) = R_yaw * R0;
                        T_align.block<3, 1>(0, 3) = d_mean - R_yaw * s_mean;

                        // 5) Quantitative tilt check (headless-verifiable):
                        //    re-fit the plane on the ALIGNED landmarks.
                        spdlog::info("alignment: landmark-plane tilt after "
                                     "align = {:.2f} deg (4-DoF gravity-aligned, "
                                     "yaw {:.2f} deg, {} plane points)",
                                     alignedPlaneTiltDeg(plane_pts, T_align),
                                     yaw * DEG_PER_RAD, plane_pts.size());
                        spdlog::info("alignment: OLD free-3D umeyama would give "
                                     "landmark-plane tilt = {:.2f} deg "
                                     "(degenerate roll, for comparison only)",
                                     alignedPlaneTiltDeg(plane_pts, T_umeyama3d));
                    } else {
                        spdlog::warn("alignment: no landmark snapshot to fit "
                                     "the ground plane ({} pts) — falling back "
                                     "to free 3D umeyama (roll unconstrained)",
                                     plane_pts.size());
                        T_align = T_umeyama3d;
                    }
                    aligned = true;
                    spdlog::info("alignment frozen at frame {} over {} TRACKING "
                                 "correspondences", r.frame_id, n_corr);
                    for (const auto& fp : est_buffer) {
                        push_estimate(fp.first, fp.second);
                    }
                    est_buffer.clear();
                    // Flush LOST markers gathered before alignment froze.
                    for (const auto& fp : lost_buffer) {
                        push_lost(fp.first, fp.second);
                    }
                    lost_buffer.clear();
                }
            } else {
                push_estimate(static_cast<int>(r.frame_id), r.T_wc);
            }
        }

        // ── Map-point cloud: push the latest keyframe cloud through the frozen
        //    alignment (only once the transform exists). ────────────────────
        if (aligned && map_dirty) {
            map_dirty = false;
            std::vector<Eigen::Vector3f> pts_enu;
            pts_enu.reserve(latest_map_points.size());
            for (const Vec3_t& p : latest_map_points) {
                const Eigen::Vector3d en =
                    (T_align * p.homogeneous()).hnormalized();
                pts_enu.emplace_back(static_cast<float>(en.x()),
                                     static_cast<float>(en.y()),
                                     static_cast<float>(en.z()));
            }
            if (!pts_enu.empty()) {
                viewer.pushMapPoints(pts_enu);
            }
        }
#endif  // UAVLOC_WITH_DEBUG_VIEWER
    });

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // ── channel 3: the fused pose ────────────────────────────────────────────
    // Fires on the FUSION thread in async mode — the DebugViewer push methods
    // are thread-safe. The orientation is passed through as-is: the constant
    // yaw offset θ (mount azimuth) baked into T_enu_c is EXPECTED — do NOT
    // correct it; seeing it on the fused pose axes is diagnostic.
    sys.callbacks().on_fusion_result.add(
        [&](double /*t_msec*/, const fusion::FusionResult& fres) {
            if (!fres.has_pose || beyond_cap_frame(fres.frame_id)) {
                return;
            }
            const Eigen::Vector3d    f = fres.T_enu_c.translation();
            const Eigen::Quaterniond q(fres.T_enu_c.rotation());
            std::lock_guard<std::mutex> lk(fused_mutex);
            latest_fused_pos = f;
            latest_fused_set = true;
            if (!fused_f0_set) {
                fused_f0     = f;
                fused_f0_set = true;
            }
            if (!fused_g0_set) {
                if (!latest_gt_valid) {
                    // no GT yet — buffer until g0 exists
                    fused_pending.emplace_back(fres.frame_id, f, q);
                    return;
                }
                fused_g0     = latest_gt_enu;
                fused_g0_set = true;
                for (const auto& [fid, fp, fq] : fused_pending) {
                    const Eigen::Vector3d d = fused_g0 + (fp - fused_f0);
                    viewer.pushFusedPose(d.cast<float>(), fq.cast<float>(), fid);
                    dump_fused(fid, d);
                    ++fused_pushed;
                    // Offered, not forced: these predate the first groundtruth,
                    // so the id will not match and they land in the skipped
                    // counter instead of becoming mis-paired samples.
                    push_error_sample(fid, d);
                }
                fused_pending.clear();
            }
            const Eigen::Vector3d d = fused_g0 + (f - fused_f0);
            viewer.pushFusedPose(d.cast<float>(), q.cast<float>(), fres.frame_id);
            dump_fused(fres.frame_id, d);
            ++fused_pushed;
            push_error_sample(fres.frame_id, d);
        });

    // ── channel 4: the smoothed lag window ───────────────────────────────────
    // Keyframe smoother update: redraw the lag window with the corrected poses.
    // SystemManager emits it right after the fusion result it belongs to, on the
    // same thread, and only when the graph was actually updated. Corrections
    // before the first GT anchor are meaningless — skipped, exactly as the old
    // driver skipped them via its pending buffer. Every lag pose goes through
    // the SAME anchor mapping as the live pushes: g0 + (p - f0), quaternion
    // passthrough.
    sys.callbacks().on_lag_window.add(
        [&](const std::vector<fusion::FusionLagPose>& window) {
            std::lock_guard<std::mutex> lk(fused_mutex);
            if (!fused_g0_set) {
                return;
            }
            std::vector<dv::FusedCorrection> corrected;
            corrected.reserve(window.size());
            for (const auto& lp : window) {
                const Eigen::Vector3d    p = lp.T_enu_c.translation();
                const Eigen::Quaterniond lq(lp.T_enu_c.rotation());
                const Eigen::Vector3d    dp = fused_g0 + (p - fused_f0);
                corrected.push_back({static_cast<uint64_t>(lp.frame_id),
                                     dp.cast<float>(), lq.cast<float>()});
            }
            if (!corrected.empty()) {
                viewer.pushFusedCorrection(corrected);
                ++fused_corrections;
            }
        });
#endif  // UAVLOC_WITH_DEBUG_VIEWER

    // ── OPTIONAL fake absolute-position producer (M1), OFF by default ────────
    // Enabled by UAVLOC_FAKE_FIX=1 and steered by the same UAVLOC_FIX_* names
    // tests/test_full_flight.cpp uses. Its point HERE is to make the absolute
    // fixes VISIBLE: yellow markers where the back-end accepted a measurement,
    // purple where it rejected it (or has not judged it yet), plus a counter
    // line and a permanent red contamination banner in the viewer's main panel.
    //
    // ⚠ Every marker is manufactured from GROUNDTRUTH. That is why the banner
    // exists and why it cannot be switched off.
    const bool fix_enabled = env_int("UAVLOC_FAKE_FIX", 0) != 0;

    // Marker bookkeeping. Written by the generation callback (pipeline thread)
    // and by the supervisor thread (verdict polling), hence its own mutex.
    std::mutex                   anchor_mutex;
    std::vector<AnchorFixRecord> anchor_fixes;
    std::deque<std::size_t>      anchor_pending;   // indices, in emission order
    std::size_t                  anchor_generated = 0;
    std::size_t                  anchor_accepted  = 0;
    std::size_t                  anchor_rejected  = 0;
    //! Re-anchor fixes, kept apart from the cadence ones everywhere they are
    //! shown: merging them would make it impossible to see whether the
    //! re-anchor path fired at all.
    std::size_t                  anchor_reinit_generated = 0;
    std::size_t                  anchor_reinit_accepted  = 0;
    bool                         anchor_dirty     = false;

    anchor::FakeAnchor* fake_anchor = nullptr;
    if (fix_enabled) {
        const anchor::FakeAnchorConfig fix_cfg =
            fake_anchor_config(yaml, reader_cfg);
        if (fix_cfg.telemetry.csv_path.empty()) {
            spdlog::error("test_vo_viewer: UAVLOC_FAKE_FIX=1 but '{}' configures "
                          "no VideoReader.DroneTelemetry.csv_path — the fake "
                          "anchor has no groundtruth to read", config_path);
            return 1;
        }
        auto fa = std::make_unique<anchor::FakeAnchor>(fix_cfg);
        fa->setGenerationCallback([&](const anchor::FakeFixRecord& rec) {
            AnchorFixRecord r;
            r.pos_fusion.head<2>() = rec.fix.xy_enu;
            r.from_reinit          = rec.from_reinit;
#ifdef UAVLOC_WITH_DEBUG_VIEWER
            {
                // A fix is horizontal-only; draw it at the altitude the
                // back-end currently believes in, so the marker sits ON the
                // fused line instead of on the ground plane.
                std::lock_guard<std::mutex> lk(fused_mutex);
                if (latest_fused_set) {
                    r.pos_fusion.z() = latest_fused_pos.z();
                }
            }
#endif
            std::lock_guard<std::mutex> lk(anchor_mutex);
            anchor_fixes.push_back(r);
            anchor_pending.push_back(anchor_fixes.size() - 1);
            ++anchor_generated;
            if (r.from_reinit) ++anchor_reinit_generated;
            anchor_dirty = true;
        });
        fake_anchor = fa.get();
        sys.setAnchor(std::move(fa));
        spdlog::warn("test_vo_viewer: FAKE ABSOLUTE FIXES ENABLED — the fix "
                     "markers are generated FROM GROUNDTRUTH; this run shows the "
                     "back-end consuming absolute positions, NOT the accuracy of "
                     "a real system");
        spdlog::info("  fake fix: sigma={:.1f} m, every {} KFs, outlier_rate={:.2f} "
                     "[{:.0f}, {:.0f}] m, latency={} KFs, seed={}, csv='{}'",
                     fix_cfg.sigma_m, fix_cfg.every_kf, fix_cfg.outlier_rate,
                     fix_cfg.outlier_min_m, fix_cfg.outlier_max_m,
                     fix_cfg.latency_kf, fix_cfg.seed, fix_cfg.telemetry.csv_path);
    }

    //! Poll the back-end's verdicts and repaint the markers. Called from the
    //! SUPERVISOR thread, never from a SystemManager callback (design §R-c: a
    //! subscriber must not call back into the manager).
    fusion::FusionFixStats anchor_prev_stats{};
    auto anchor_poll = [&]() {
        if (fake_anchor == nullptr) {
            return;
        }
        const fusion::FusionFixStats st = sys.fixStats();
        std::lock_guard<std::mutex> lk(anchor_mutex);
        // Verdicts arrive as counter DELTAS, attributed to the oldest
        // unresolved fixes in emission order — the module consumes them FIFO.
        auto take = [&](unsigned long long n, bool accepted) {
            for (unsigned long long i = 0; i < n && !anchor_pending.empty(); ++i) {
                const std::size_t idx = anchor_pending.front();
                anchor_pending.pop_front();
                anchor_fixes[idx].resolved = true;
                anchor_fixes[idx].accepted = accepted;
                if (accepted) ++anchor_accepted; else ++anchor_rejected;
                if (accepted && anchor_fixes[idx].from_reinit) {
                    ++anchor_reinit_accepted;
                }
                anchor_dirty = true;
            }
        };
        take(st.applied - anchor_prev_stats.applied, true);
        // Everything that is not "applied" is a rejection as far as the picture
        // is concerned; the log and test_full_flight's CSV carry the exact
        // reason, which a 3D marker cannot show anyway.
        take((st.gated          - anchor_prev_stats.gated) +
             (st.low_confidence - anchor_prev_stats.low_confidence) +
             (st.age_expired    - anchor_prev_stats.age_expired) +
             (st.unmatched      - anchor_prev_stats.unmatched) +
             (st.marginalized   - anchor_prev_stats.marginalized) +
             (st.queue_dropped  - anchor_prev_stats.queue_dropped) +
             (st.no_graph       - anchor_prev_stats.no_graph), false);
        anchor_prev_stats = st;

        if (!anchor_dirty) {
            return;
        }
        anchor_dirty = false;
#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // Same display mapping as the fused line — g0 + (p - f0) — so a marker
        // lands where it belongs relative to the other trajectories.
        std::vector<dv::AnchorFixMarker> markers;
        {
            std::lock_guard<std::mutex> lk_fused(fused_mutex);
            if (!fused_f0_set || !fused_g0_set) {
                anchor_dirty = true;  // no display anchor yet — retry next poll
                return;
            }
            markers.reserve(anchor_fixes.size());
            for (const AnchorFixRecord& r : anchor_fixes) {
                const Eigen::Vector3d d = fused_g0 + (r.pos_fusion - fused_f0);
                markers.push_back({d.cast<float>(), r.resolved && r.accepted,
                                   r.from_reinit});
            }
        }
        viewer.setAnchorFixes(markers);
        viewer.setAnchorOverlay(
            FAKE_ANCHOR_BANNER,
            "Anchor fixes  generated " + std::to_string(anchor_generated) +
                " | accepted " + std::to_string(anchor_accepted) +
                " | rejected " + std::to_string(anchor_rejected) +
                " | pending "  + std::to_string(anchor_pending.size()) +
                "   ||  re-anchor (square) " +
                std::to_string(anchor_reinit_generated) + " | accepted " +
                std::to_string(anchor_reinit_accepted));
#endif
    };

    if (!sys.setup()) {
        spdlog::error("test_vo_viewer: SystemManager setup failed");
        return 1;
    }

    // True once SystemManager::start() has succeeded — the first Start brings
    // the system up, every later one only resumes the reader. Atomic because it
    // is written on the main thread before run() and on the render thread from
    // inside the button handler.
    std::atomic<bool> system_started{false};

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // ── The Start/Stop bridge, and it lives HERE on purpose ──────────────────
    // The viewer publishes a click and knows nothing else; this driver owns the
    // SystemManager and performs the transition. uavloc_debug_viewer therefore
    // still links no uavloc target (R4). The handler runs on the viewer's render
    // thread — never on the source's reading thread, which is the one caller
    // pauseSource() would have to refuse — and returns the VERDICT, so a refused
    // request leaves the button label where it was instead of lying.
    viewer.setRunControl([&sys, &system_started](bool want_run) {
        if (!want_run) {
            return sys.pauseSource();
        }
        if (system_started.load()) {
            return sys.resumeSource();
        }
        if (!sys.start()) {
            return false;
        }
        system_started.store(true);
        return true;
    });
#endif

    if (autostart) {
        if (!sys.start()) {
            spdlog::error("test_vo_viewer: SystemManager start failed");
            return 1;
        }
        system_started.store(true);
        spdlog::info("test_vo_viewer: run mode = AUTO-START ({})", autostart_reason);
#ifdef UAVLOC_WITH_DEBUG_VIEWER
        viewer.setRunState(dv::DebugViewer::RunState::RUNNING);
#endif
    } else {
        spdlog::info("test_vo_viewer: run mode = WAIT FOR THE VIEWER'S 'Start' "
                     "BUTTON (a DISPLAY is present and neither UAVLOC_VIEWER_DUMP "
                     "nor UAVLOC_VIEWER_AUTOSTART=1 is set). Start/Stop then "
                     "pauses and resumes the input as often as you like.");
    }

    // Supervisor: the only thing that decides WHEN the data is over. It stops
    // the system (which cuts the source, drains and joins) but leaves the
    // viewer window alone — the old driver's worker did exactly this when it
    // ran out of frames.
    std::thread supervisor([&]() {
        while (!stop_requested.load()) {
            if (max_frames_env > 0 && frames_processed.load() >= max_frames_env) {
                break;
            }
            if (sys.stats().source_ended) {
                break;
            }
            anchor_poll();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(SUPERVISOR_POLL_MS));
        }
        sys.stop();
        // Final sweep: the last keyframe update resolves whatever was still in
        // flight, and the picture must show those verdicts too.
        anchor_poll();
#ifdef UAVLOC_WITH_DEBUG_VIEWER
        // The button must not still offer "Stop" once the run is over. It goes
        // back to "Start"; pressing it then hits resumeSource() on a system that
        // sys.stop() has shut down, which is refused and logged — the viewer
        // reports the refusal instead of pretending the run resumed.
        viewer.setRunState(dv::DebugViewer::RunState::PAUSED);
#endif
        spdlog::info("pipeline finished ({} frames) — viewer stays open, "
                     "close the window to exit", frames_processed.load());
    });

#ifdef UAVLOC_WITH_DEBUG_VIEWER
    // Performance sampler: a tiny dedicated thread that every `perf_sample_ms`
    // takes a util::SysMonitor sample (CPU % + RSS MB from /proc, measured
    // inside the util module) and pushes a PerfSample into the viewer's
    // "Performance" streaming charts. Producer-side measurement — the viewer
    // only plots (module decoupling). Terminates on stop_requested.
    std::atomic<std::size_t> perf_samples{0};
    std::thread perf_sampler([&]() {
        uavloc::util::SysMonitor mon;
        while (!stop_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(perf_sample_ms));
            uavloc::util::SysStats s;
            if (mon.sample(s)) {
                viewer.pushPerf(dv::PerfSample{s.t_sec, s.cpu_percent, s.rss_mb});
                ++perf_samples;
            }
        }
    });

    // Viewer owns the main thread (GL singleton); the pipeline runs on the
    // source's streaming thread. With a display, run() blocks until the user
    // closes the window — whether mid-run (the flag makes the supervisor tear
    // the system down) or after the data ended (the supervisor already did).
    // Headless, run() returns at once and the join below simply waits for the
    // pipeline to finish on its own.
    viewer.run();              // blocking on a display; returns immediately headless
    if (viewer.hadDisplay()) {
        stop_requested.store(true);
    }
    supervisor.join();
    stop_requested.store(true);  // sole sampler-exit signal on the headless path
    perf_sampler.join();
    spdlog::info("perf sampler: {} samples", perf_samples.load());

    if (fused_dump.is_open()) {
        fused_dump.close();
        spdlog::info("test_vo_viewer: fused pose dump written to '{}'", dump_path);
    }
    spdlog::info("fused poses pushed: {}", fused_pushed.load());
    spdlog::info("fused corrections applied: {}", fused_corrections.load());
    // Feed check for the "Error (m)" window (plumbing, not accuracy): skipped
    // frames are the ones with no groundtruth of the SAME id — mostly the poses
    // flushed out of fused_pending before the first GT existed.
    spdlog::info("error samples pushed: {} (skipped, no same-frame GT: {})",
                 error_samples_pushed, error_samples_skipped);
#else
    supervisor.join();
    stop_requested.store(true);
#endif

    // Idempotent: the supervisor already stopped it in every normal path.
    sys.stop();

    if (fake_anchor != nullptr) {
        const anchor::FakeAnchorStats ast = fake_anchor->stats();
        const fusion::FusionFixStats  fst = sys.fixStats();
        spdlog::warn("test_vo_viewer: the absolute fixes above were GENERATED "
                     "FROM GROUNDTRUTH — this run shows the back-end consuming "
                     "absolute positions, not a real system");
        spdlog::info("anchor: requests={} generated={} ({} outliers) emitted={} | "
                     "skipped: cadence={} no_telemetry={} no_groundtruth={} "
                     "no_origin={}",
                     ast.requested, ast.generated, ast.outliers, ast.emitted,
                     ast.skipped_cadence, ast.skipped_no_telemetry,
                     ast.skipped_no_groundtruth, ast.skipped_no_origin);
        spdlog::info("anchor markers: generated={} accepted={} rejected={} "
                     "(back-end: injected={} applied={} gated={} low_confidence={} "
                     "age_expired={} unmatched={} marginalized={} no_graph={})",
                     anchor_generated, anchor_accepted, anchor_rejected,
                     fst.injected, fst.applied, fst.gated, fst.low_confidence,
                     fst.age_expired, fst.unmatched, fst.marginalized,
                     fst.no_graph);
        // Re-anchor breakdown on its own line (square markers in the viewer).
        spdlog::info("anchor re-anchor: requests={} generated={} accepted={} "
                     "throttled={} | cadence fixes={}",
                     ast.reinit_requested, anchor_reinit_generated,
                     anchor_reinit_accepted, ast.reinit_throttled,
                     anchor_generated - anchor_reinit_generated);
    }

    spdlog::info("test_vo_viewer: frames_processed={} pose_successes={} final_state={}",
                 frames_processed.load(), pose_successes.load(), final_state.load());
    return 0;
}
