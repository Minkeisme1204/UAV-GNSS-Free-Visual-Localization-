// test_vo_viewer — WORKED EXAMPLE of how to drive uavloc, with a live 3D view.
//
// Read this file as the reference assembly of the system. It does four things
// and nothing else:
//   1. load one mission YAML  → sensor::VideoReaderConfig + core::SystemConfig;
//   2. build core::SystemManager and attach a push-style sensor::VideoDataSource
//      (the source owns its reading thread; the manager owns the pipeline);
//   3. decide the RUN-MODE POLICY (auto-start vs. wait for the Start button) —
//      only the application knows whether anybody can press a button;
//   4. hand the manager to debug_viewer::DebugViewer::attach() and let
//      DebugViewer::run() own the main thread and the run, then report.
//
// ⚠ NO ABSOLUTE POSITION HERE. This driver runs VO + fusion + viewer only; no
// anchor::AnchorInterface is attached, so the fused trajectory is pure dead
// reckoning. For the whole-system flow WITH absolute fixes use the sibling
// driver tests/test_vo_vpr_anchor_viewer.cpp.
//
// All display logic — the four channel callbacks, the 4-DoF alignment, the
// display anchor, the overlays, the supervisor and the perf sampler — lives in
// the viewer module, not here. Design + contracts:
// `.docs/designs/viewer_module_design.md`.
//
// The target is only built with -DENABLE_VIEWER=ON (and Iridescence found), so
// the viewer is ALWAYS present: this file carries no #ifdef and no headless
// fallback path.
//
// ── No environment variables ────────────────────────────────────────────────
// What this driver measures is fixed by named constants (the frame cap and the
// dump path in tests/driver_common.h, the profiling switch below), so a run is
// described completely by "this binary + this config". Only SPDLOG_LEVEL, which
// belongs to the logging library and changes nothing that is measured, is still
// honoured.
//
// ── The pose dump — the cross-driver regression gate ────────────────────────
// The run ALWAYS writes one row per fused pose (`frame_id,pred_x,pred_y,pred_z`)
// to eval::VIEWER_DUMP_PATH: the DISPLAY-anchored position g0 + (f - f0), i.e.
// exactly what the viewer draws, taken from DebugViewer::FusedPoseSink.
// tests/test_driver_parity.cpp compares that sequence with test_full_flight's;
// the two differ by a CONSTANT (the display anchor), which is the property that
// gate asserts.
//
// ── Start/Stop vs. auto-start, and the frame cap ────────────────────────────
// Both come from eval::viewer_run_policy(), which answers one question — is
// this a regression run or a visual session? Headless ⇒ the run starts by
// itself (nobody could click) and stops at eval::PARITY_MAX_FRAMES. With a
// DISPLAY ⇒ it waits for the viewer's Start button and is NOT capped: a cap
// would end the session behind the operator's back and leave a Start button
// that can no longer start anything.
//
// Soft-skips (returns 0) when the gitignored dataset video is unavailable.
//
// Usage (from build/):
//     ./tests/test_vo_viewer [config.yaml]
// Run with a DISPLAY to open the live window (trajectory + groundtruth + cloud).
#include "uavloc/core/system_config.h"
#include "uavloc/core/system_manager.h"
#include "uavloc/debug_viewer/debug_viewer.h"
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
constexpr const char* DRIVER_NAME = "test_vo_viewer";

//! Per-frame stage profiling (util::Profiler → the viewer's "Profiling" table).
//! OFF: it is a read-only instrument, but it costs time on the pipeline thread
//! and nothing in this driver's job needs it. Flip it to true and rebuild to
//! look at the stage breakdown of a run.
constexpr bool PROFILE_ENABLED = false;

}  // namespace
}  // namespace uavloc

int main(int argc, char** argv) {
    using namespace uavloc;
    namespace dv = uavloc::debug_viewer;

    // Honour SPDLOG_LEVEL for a verbose per-frame run.
    spdlog::cfg::load_env_levels();

    const std::string config_path =
        (argc > 1) ? std::string(argv[1]) : DEFAULT_CONFIG_PATH;

    util::Profiler::set_enabled(PROFILE_ENABLED);

    // Mission → run mode → source. All three steps are shared verbatim with
    // test_vo_vpr_anchor_viewer, so the two drivers cannot drift apart.
    eval::MissionSetup mission = eval::load_viewer_mission(config_path);
    if (!mission.loaded) {
        return 1;
    }
    auto source = eval::open_viewer_source(mission, DRIVER_NAME);
    if (!source) {
        return 0;  // soft-skip: the gitignored dataset is absent
    }

    // The whole pipeline behind one object; the source is OWNED by it, so the
    // start/stop ordering is enforced there and not re-invented here (§4.5).
    core::SystemManager sys(mission.system);
    sys.attachSource(std::move(source));

    // ── Driver-side accounting only (the display counts separately) ──────────
    std::atomic<std::size_t> frames_processed{0};
    std::atomic<std::size_t> pose_successes{0};
    // Tracking state of the last processed frame (== VOModule::get_state()).
    std::atomic<int> final_state{
        static_cast<int>(vo::VOTrackingState::NOT_INITIALIZED)};

    // Who starts the run AND how long it runs: one decision, taken once (see
    // eval::viewer_run_policy). Headless ⇒ a capped, self-starting regression
    // run; with a display ⇒ an uncapped session the operator drives.
    const eval::RunPolicy policy = eval::viewer_run_policy();
    // 0 = no cap. Below the cap, everything past the cap-th frame is ignored so
    // the summary reports the capped run and not the few frames the
    // supervisor's poll period let slip through.
    const std::size_t max_frames = policy.max_frames;

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

    // ── The pose dump — written on every run, no switch ──────────────────────
    std::ofstream fused_dump =
        eval::open_pose_dump(eval::VIEWER_DUMP_PATH, DRIVER_NAME);

    // The whole "DebugViewer:" section is parsed by the viewer library itself
    // (window/panel keys plus the overlay parameters: alignment window and
    // spread gate, HUD baseline, perf sampler period). The three run-policy
    // fields are NOT in the YAML — they are this application's decision.
    dv::DebugViewer::Config viewer_cfg =
        dv::DebugViewer::Config::fromYaml(mission.yaml["DebugViewer"]);
    viewer_cfg.autostart        = policy.autostart;
    viewer_cfg.autostart_reason = policy.reason;
    viewer_cfg.max_frames       = max_frames;

    dv::DebugViewer viewer(viewer_cfg);
    viewer.loadBatch({});  // empty -> live push mode (holds window, drains queues)

    // Capture spdlog output into the viewer's scrolling log panel.
    spdlog::default_logger()->sinks().push_back(viewer.logSink());

    // Dump source: every position the viewer draws on the fused line, in push
    // order. The viewer calls the sink from inside the same critical section it
    // already holds around every push, so the order in the file is the order on
    // the screen — no lock is needed here.
    if (fused_dump.is_open()) {
        viewer.setFusedPoseSink(
            [&fused_dump](unsigned int fid, const Eigen::Vector3d& p) {
                fused_dump << fid << ',' << p.x() << ',' << p.y() << ','
                           << p.z() << '\n';
            });
    }

    // Subscribes to the four display channels, installs the Start/Stop control,
    // and makes run() the owner of the run (auto-start + supervisor + sampler).
    viewer.attach(sys);

    if (!sys.setup()) {
        spdlog::error("{}: SystemManager setup failed", DRIVER_NAME);
        return 1;
    }

    // Viewer owns the main thread (GL singleton) AND the run: it performs the
    // configured auto-start, supervises the end of data (or the frame cap) and
    // joins its two service threads before returning. With a display it blocks
    // until the user closes the window; headless it returns as soon as the
    // pipeline has finished.
    viewer.run();

    // An auto-start that was REFUSED leaves the button state at IDLE (a started
    // run ends FINISHED, set by the supervisor; a user-paused one PAUSED). That
    // is the only way this driver can still tell a failed start from a
    // completed run.
    if (policy.autostart && viewer.runState() == dv::DebugViewer::RunState::IDLE) {
        spdlog::error("{}: SystemManager start failed", DRIVER_NAME);
        return 1;
    }

    // Idempotent: the supervisor already stopped it in every normal path.
    sys.stop();

    // Closed only after stop(), so no producer can still be writing.
    if (fused_dump.is_open()) {
        fused_dump.close();
        spdlog::info("{}: fused pose dump written to '{}'", DRIVER_NAME,
                     eval::VIEWER_DUMP_PATH);
    }

    spdlog::info("{}: frames_processed={} pose_successes={} final_state={}",
                 DRIVER_NAME, frames_processed.load(), pose_successes.load(),
                 final_state.load());
    return 0;
}
