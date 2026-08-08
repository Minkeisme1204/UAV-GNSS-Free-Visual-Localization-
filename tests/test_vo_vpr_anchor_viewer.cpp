// test_vo_vpr_anchor_viewer — the WHOLE system in one run, with a live 3D view:
// core::SystemManager (VO + fusion) + sensor::VideoDataSource +
// debug_viewer::DebugViewer + an ABSOLUTE-POSITION producer.
//
// This is the sibling of tests/test_vo_viewer.cpp: same assembly, same run
// policy, same pose dump — plus the one piece that closes the loop, the
// anchor::AnchorInterface producer that supplies absolute position.
//
// ── ⚠ THE PRODUCER IS FAKE — THE NUMBERS ARE CONTAMINATED ───────────────────
// VPR does not exist yet (Phase 2), so the producer here is anchor::FakeAnchor,
// which does NOT recognise a place: it READS THE GROUNDTRUTH and adds noise.
// Every error figure from a run of this driver measures the BACK-END's ability
// to consume absolute positions and NOTHING about the accuracy of a real
// system. That sentence must travel with any number taken from here
// (.claude/rules/reporting.md), which is why the viewer shows a permanent red
// banner (enableAnchorFixTracking below) — a screenshot must not be able to
// leave without the caveat.
//
// When the real VPR producer exists, only the PRODUCER changes: it is handed to
// sys.setAnchor() through the same anchor::AnchorInterface, and neither the
// fusion back-end nor core::SystemManager needs a line of change. That is the
// whole point of the request/response seam.
//
// The target is only built with -DENABLE_VIEWER=ON (and Iridescence found), so
// the viewer is ALWAYS present: this file carries no #ifdef.
//
// ── No environment variables ────────────────────────────────────────────────
// The fake producer is configured by the named constants below, not by env vars
// and not by a YAML section, so a run is described completely by "this binary +
// this config". Only SPDLOG_LEVEL is honoured (it changes nothing measured).
// The groundtruth CSV and its frame-id offset are NOT constants — they are the
// mission's own data and come from the config's VideoReader section.
//
// ── Output ──────────────────────────────────────────────────────────────────
//   * eval::ANCHOR_VIEWER_DUMP_PATH — one row per fused pose
//     (`frame_id,pred_x,pred_y,pred_z`), the DISPLAY-anchored position the
//     viewer draws. A SEPARATE file from test_vo_viewer's dump on purpose: this
//     run injects absolute fixes, so it is a different experiment;
//   * a final report of anchor::FakeAnchorStats (producer side),
//     debug_viewer::AnchorFixCounters (what was drawn and how it was judged)
//     and fusion::FusionFixStats (back-end verdicts).
// The per-fix lifecycle is logged by the library as anchor[REQ] → anchor[EMIT]
// → anchor[APPLY], correlated by timestamp; every reject path prints its reason.
//
// Soft-skips (returns 0) when the gitignored dataset video is unavailable.
//
// ── Start/Stop vs. auto-start, and the frame cap ────────────────────────────
// Both come from eval::viewer_run_policy(), exactly as in test_vo_viewer:
// headless ⇒ a self-starting run capped at eval::PARITY_MAX_FRAMES (so the two
// viewer drivers cost the same wall time and fit the same ctest TIMEOUT; what
// this driver exercises — the request → emit → apply path — happens per
// keyframe, not per flight); with a DISPLAY ⇒ an uncapped session that waits
// for the Start button.
//
// Usage (from build/):
//     ./tests/test_vo_vpr_anchor_viewer [config.yaml]
// Run with a DISPLAY to open the live window; headless it starts by itself.
#include "uavloc/anchor/fake_anchor.h"
#include "uavloc/core/system_config.h"
#include "uavloc/core/system_manager.h"
#include "uavloc/debug_viewer/debug_viewer.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/util/scoped_timer.h"

#include "driver_common.h"

#include <Eigen/Core>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <cstddef>
#include <fstream>
#include <memory>
#include <string>

#ifndef UAVLOC_MISSION_CONFIG_PATH
#define UAVLOC_MISSION_CONFIG_PATH "config/uavloc_yenbai800m_newvo.yaml"   // fallback; CMake injects the real path
#endif

namespace uavloc {
namespace {

const std::string DEFAULT_CONFIG_PATH = UAVLOC_MISSION_CONFIG_PATH;

//! Prefix of this driver's log lines (and of the shared helpers' lines).
constexpr const char* DRIVER_NAME = "test_vo_vpr_anchor_viewer";

//! Per-frame stage profiling (util::Profiler → the viewer's "Profiling" table).
//! OFF, as in test_vo_viewer: a read-only instrument that still costs time on
//! the pipeline thread. Flip it to true and rebuild to inspect a run.
constexpr bool PROFILE_ENABLED = false;

//! The sentence that must stay on screen for as long as fake fixes are being
//! injected. Not a log line: a screenshot of the window has to carry it.
const char* const FAKE_ANCHOR_BANNER =
    "FAKE ANCHOR DANG BAT - cac diem 'fix' sinh tu GROUNDTRUTH, "
    "KHONG phai hieu nang cua he thong that.";

// ── Fake-producer configuration — every value is anchor::FakeAnchorConfig's
// own default (include/uavloc/anchor/fake_anchor.h). They are repeated here as
// named constants, not left implicit, because a driver that reports fix counts
// must state the experiment it ran; changing one here is a deliberate act that
// shows up in a diff.

//! Standard deviation [m] of the Gaussian noise added to a good fix, per axis,
//! and what the emitted AbsoluteFix declares in its covariance.
//! ⚠ 20 m is the struct default, NOT a value chosen from a measurement on this
//! dataset. The measured law (.docs/reports/m1_acceptance.md §4) is that a fix
//! only carries information while its sigma is below the drift accumulated
//! since the last accepted fix — on a 200-frame run the drift is small, so this
//! sigma may well be too large to help. That is a property to MEASURE with this
//! driver, not one to pre-empt by tuning the constant.
constexpr double FIX_SIGMA_M = 20.0;

//! Duty cycle: at most one fix every N keyframe requests. A real producer runs
//! on keyframes and cannot run on all of them.
constexpr int FIX_EVERY_KF = 20;

//! Anti-flooding limit on the REINIT bypass, in requests. 0 = every VO re-init
//! is allowed to skip the duty cycle. Kept at 0 because no measurement exists
//! to justify a non-zero value, and a non-zero one silently drops measurements.
constexpr int FIX_REINIT_MIN_KF = 0;

//! Probability that a fix is a gross outlier instead of a noisy-but-correct
//! measurement, and the magnitude band [m] of that corruption. 0 = never: this
//! driver's job is to show the nominal loop end to end. Outlier rejection has
//! its own dedicated test (tests/test_anchor.cpp).
constexpr double FIX_OUTLIER_RATE  = 0.0;
constexpr double FIX_OUTLIER_MIN_M = 200.0;
constexpr double FIX_OUTLIER_MAX_M = 1000.0;

//! Hold a produced fix back for N further requests before emitting it (the fix
//! keeps its original timestamp, so the smoother must reach back into the lag
//! window). 0 = emit at once — the latency path is exercised by test_anchor.
constexpr int FIX_LATENCY_KF = 0;

//! Seed of the noise/outlier RNG. Fixed, so the same binary on the same data
//! manufactures the same fixes byte for byte — this driver is a repeatable
//! experiment, not a random one.
constexpr unsigned int FIX_SEED = 42u;

}  // namespace
}  // namespace uavloc

int main(int argc, char** argv) {
    using namespace uavloc;
    namespace dv = uavloc::debug_viewer;

    spdlog::cfg::load_env_levels();

    const std::string config_path =
        (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    util::Profiler::set_enabled(PROFILE_ENABLED);

    // Mission → run mode → source: shared verbatim with test_vo_viewer, so the
    // only difference between the two runs is the anchor attached below.
    eval::MissionSetup mission = eval::load_viewer_mission(config_path);
    if (!mission.loaded) {
        return 1;
    }
    auto source = eval::open_viewer_source(mission, DRIVER_NAME);
    if (!source) {
        return 0;  // soft-skip: the gitignored dataset is absent
    }

    core::SystemManager sys(mission.system);
    sys.attachSource(std::move(source));

    // ── Driver-side accounting only (the display counts separately) ──────────
    std::atomic<std::size_t> frames_processed{0};
    std::atomic<std::size_t> pose_successes{0};
    std::atomic<int>         final_state{
        static_cast<int>(vo::VOTrackingState::NOT_INITIALIZED)};

    // Who starts the run AND how long it runs — one decision, shared verbatim
    // with test_vo_viewer (eval::viewer_run_policy): headless ⇒ a capped,
    // self-starting regression run; with a display ⇒ an uncapped session the
    // operator drives. 0 = no cap.
    const eval::RunPolicy policy = eval::viewer_run_policy();
    const std::size_t     max_frames = policy.max_frames;

    sys.callbacks().on_frame_processed.add(
        [&](double /*t_msec*/, const core::FrameProcessed& /*fp*/) {
            ++frames_processed;
        });
    sys.callbacks().on_vo_data.add([&](double /*t_msec*/, const vo::VOData& data) {
        final_state.store(static_cast<int>(data.result.state));
        if (max_frames > 0 && frames_processed.load() > max_frames) {
            return;
        }
        if (data.result.has_pose) {
            ++pose_successes;
        }
    });

    std::ofstream fused_dump =
        eval::open_pose_dump(eval::ANCHOR_VIEWER_DUMP_PATH, DRIVER_NAME);

    dv::DebugViewer::Config viewer_cfg =
        dv::DebugViewer::Config::fromYaml(mission.yaml["DebugViewer"]);
    viewer_cfg.autostart        = policy.autostart;
    viewer_cfg.autostart_reason = policy.reason;
    viewer_cfg.max_frames       = max_frames;

    dv::DebugViewer viewer(viewer_cfg);
    viewer.loadBatch({});  // empty -> live push mode

    spdlog::default_logger()->sinks().push_back(viewer.logSink());

    if (fused_dump.is_open()) {
        viewer.setFusedPoseSink(
            [&fused_dump](unsigned int fid, const Eigen::Vector3d& p) {
                fused_dump << fid << ',' << p.x() << ',' << p.y() << ','
                           << p.z() << '\n';
            });
    }

    viewer.attach(sys);

    // ── The absolute-position producer ──────────────────────────────────────
    // Building it is system assembly and stays in the application; every
    // generated fix is forwarded to the viewer, which draws it, polls the
    // back-end's verdict and keeps the counters. No graph work happens on the
    // producer thread — SystemManager's result callback only hands the fix to
    // the back-end.
    anchor::FakeAnchorConfig fix_cfg;
    // Mission data, not a tunable: the fake producer reads exactly the drone log
    // the pipeline itself is fed, with the same frame-id offset.
    fix_cfg.telemetry         = mission.reader.drone_telemetry;
    fix_cfg.frame_id_offset   =
        static_cast<long long>(mission.reader.telemetry_frame_id_offset);
    fix_cfg.sigma_m           = FIX_SIGMA_M;
    fix_cfg.every_kf          = FIX_EVERY_KF;
    fix_cfg.reinit_min_kf_gap = FIX_REINIT_MIN_KF;
    fix_cfg.outlier_rate      = FIX_OUTLIER_RATE;
    fix_cfg.outlier_min_m     = FIX_OUTLIER_MIN_M;
    fix_cfg.outlier_max_m     = FIX_OUTLIER_MAX_M;
    fix_cfg.latency_kf        = FIX_LATENCY_KF;
    fix_cfg.seed              = FIX_SEED;
    // SYNCHRONOUS: an asynchronous producer would make the instant a fix enters
    // the graph depend on thread scheduling, and this driver has to be
    // repeatable.
    fix_cfg.mode              = anchor::FakeAnchorMode::SYNCHRONOUS;

    if (fix_cfg.telemetry.csv_path.empty()) {
        spdlog::error("{}: '{}' configures no VideoReader.DroneTelemetry."
                      "csv_path — the fake anchor has no groundtruth to read",
                      DRIVER_NAME, config_path);
        return 1;
    }

    auto fake_anchor_owned = std::make_unique<anchor::FakeAnchor>(fix_cfg);
    anchor::FakeAnchor* fake_anchor = fake_anchor_owned.get();
    // The banner is a REQUIREMENT, not decoration (.claude/rules/reporting.md).
    viewer.enableAnchorFixTracking(FAKE_ANCHOR_BANNER);
    fake_anchor->setGenerationCallback([&viewer](const anchor::FakeFixRecord& rec) {
        viewer.pushAnchorFix(rec.fix.xy_enu, rec.from_reinit);
    });
    sys.setAnchor(std::move(fake_anchor_owned));

    spdlog::warn("{}: FAKE ABSOLUTE FIXES ENABLED — the fix markers are "
                 "generated FROM GROUNDTRUTH; this run shows the back-end "
                 "consuming absolute positions, NOT the accuracy of a real "
                 "system", DRIVER_NAME);
    spdlog::info("  fake fix: sigma={:.1f} m, every {} KFs, outlier_rate={:.2f} "
                 "[{:.0f}, {:.0f}] m, latency={} KFs, reinit_min_kf={}, seed={}, "
                 "csv='{}'",
                 fix_cfg.sigma_m, fix_cfg.every_kf, fix_cfg.outlier_rate,
                 fix_cfg.outlier_min_m, fix_cfg.outlier_max_m,
                 fix_cfg.latency_kf, fix_cfg.reinit_min_kf_gap, fix_cfg.seed,
                 fix_cfg.telemetry.csv_path);

    if (!sys.setup()) {
        spdlog::error("{}: SystemManager setup failed", DRIVER_NAME);
        return 1;
    }

    // The viewer owns the main thread AND the run (auto-start, supervisor,
    // sampler); headless it returns when the pipeline is done.
    viewer.run();

    // IDLE == the auto-start was REFUSED (a started run ends FINISHED, set by
    // the supervisor); the only way to tell a failed start from a finished run.
    if (policy.autostart && viewer.runState() == dv::DebugViewer::RunState::IDLE) {
        spdlog::error("{}: SystemManager start failed", DRIVER_NAME);
        return 1;
    }

    sys.stop();  // idempotent: the supervisor already stopped it normally

    if (fused_dump.is_open()) {
        fused_dump.close();
        spdlog::info("{}: fused pose dump written to '{}'", DRIVER_NAME,
                     eval::ANCHOR_VIEWER_DUMP_PATH);
    }

    // ── Report: producer, display and back-end counters ─────────────────────
    const anchor::FakeAnchorStats  ast = fake_anchor->stats();
    const dv::AnchorFixCounters    afc = viewer.anchorFixCounters();
    const fusion::FusionFixStats   fst = sys.fixStats();
    spdlog::warn("{}: the absolute fixes above were GENERATED FROM GROUNDTRUTH "
                 "— this run shows the back-end consuming absolute positions, "
                 "not a real system", DRIVER_NAME);
    spdlog::info("anchor: requests={} generated={} ({} outliers) emitted={} | "
                 "skipped: cadence={} no_telemetry={} no_groundtruth={} "
                 "no_origin={}",
                 ast.requested, ast.generated, ast.outliers, ast.emitted,
                 ast.skipped_cadence, ast.skipped_no_telemetry,
                 ast.skipped_no_groundtruth, ast.skipped_no_origin);
    spdlog::info("anchor markers: generated={} accepted={} rejected={} "
                 "pending={} (back-end: injected={} applied={} gated={} "
                 "low_confidence={} age_expired={} unmatched={} marginalized={} "
                 "no_graph={})",
                 afc.generated, afc.accepted, afc.rejected, afc.pending,
                 fst.injected, fst.applied, fst.gated, fst.low_confidence,
                 fst.age_expired, fst.unmatched, fst.marginalized, fst.no_graph);
    // Re-anchor breakdown on its own line (square markers in the viewer).
    spdlog::info("anchor re-anchor: requests={} generated={} accepted={} "
                 "throttled={} | cadence fixes={}",
                 ast.reinit_requested, afc.reinit_generated,
                 afc.reinit_accepted, ast.reinit_throttled,
                 afc.generated - afc.reinit_generated);
    // Not a failure — a FINDING that has to be visible in the log, because a run
    // that applied nothing measured nothing about the back-end consuming fixes.
    if (fst.applied == 0) {
        spdlog::warn("{}: NO absolute fix reached the graph in this run "
                     "(applied=0). The run is valid as a smoke test but says "
                     "NOTHING about the effect of absolute positioning — report "
                     "it as such", DRIVER_NAME);
    }

    spdlog::info("{}: frames_processed={} pose_successes={} final_state={}",
                 DRIVER_NAME, frames_processed.load(), pose_successes.load(),
                 final_state.load());
    return 0;
}
