#pragma once

// driver_common — the settings the evaluation DRIVERS must agree on.
//
// tests/eval_common.h is the shared SCORING ruler; this is the shared INPUT
// side. Two drivers that score with the same maths but run the pipeline in
// different modes still produce incomparable numbers, and until now they did:
// test_full_flight forced VO async + the local-BA handshake while
// test_vo_viewer ran whatever the YAML said, and forced the synchronous
// fusion/input path only inside its UAVLOC_VIEWER_DUMP branch. The equality of
// their pose sequences was therefore an observation, not a property.
//
// Everything here is deliberately NOT product API: it configures an
// experiment, so it lives under tests/ and is never installed.

#include "uavloc/anchor/fake_anchor.h"
#include "uavloc/core/system_config.h"
#include "uavloc/sensor/video_data_source.h"
#include "uavloc/sensor/video_reader.h"

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <fstream>
#include <memory>
#include <string>

namespace uavloc {
namespace eval {

// ── Constants shared by the drivers ─────────────────────────────────────────
// The two viewer drivers take NO environment variable at all: what they measure
// is fixed by the constants below, so a run is described completely by its
// binary + config. tests/test_driver_parity.cpp therefore cannot pass a cap or a
// dump path to tests/test_vo_viewer.cpp any more — it reads THESE names, which
// is why they live here and not in either driver.

//! Frames fed to a viewer driver, and to test_full_flight when
//! test_driver_parity compares the two. Small enough for the ctest TIMEOUTs
//! (~40 s per run on ds3, measured 2026-08-08), large enough that VO has
//! initialised, lost tracking and re-initialised at least once on ds3 — the
//! paths where the two drivers could actually diverge.
constexpr int PARITY_MAX_FRAMES = 200;

//! Frame cap of an INTERACTIVE viewer session: none. 0 is the "run to end of
//! stream" value of DebugViewer::Config::max_frames, and it is spelled out here
//! because it is a DECISION, not an absent value — see viewer_run_policy().
constexpr std::size_t INTERACTIVE_MAX_FRAMES = 0;

//! Where tests/test_vo_viewer.cpp writes its fused pose sequence, ALWAYS (there
//! is no switch): `frame_id,pred_x,pred_y,pred_z`, one row per fused pose.
//! test_driver_parity joins that file with test_full_flight's CSV.
constexpr const char* VIEWER_DUMP_PATH = "test_vo_viewer_dump.csv";

//! Same file for tests/test_vo_vpr_anchor_viewer.cpp. A SEPARATE name on
//! purpose: that driver injects absolute fixes, so its poses are a different
//! experiment and must never overwrite the parity gate's input.
constexpr const char* ANCHOR_VIEWER_DUMP_PATH =
    "test_vo_vpr_anchor_viewer_dump.csv";

//! Digits written to a pose dump. MUST equal test_full_flight's CSV_PRECISION:
//! test_driver_parity joins the two files, so a different digit count would
//! show up as a spurious per-frame spread.
constexpr int DUMP_PRECISION = 12;

//! `name` as an int, or `fallback` when it is unset or empty.
int env_int(const char* name, int fallback);

//! `name` as a double, or `fallback` when it is unset or empty.
double env_double(const char* name, double fallback);

//! The run mode every offline evaluation driver uses, forced over whatever the
//! mission YAML says:
//!   * vo.async_enabled = true, vo.wait_for_local_bundle_adjustment = true —
//!     the mapping thread runs, but tracking blocks per keyframe until local BA
//!     finishes, so the frame sequence stays deterministic;
//!   * fusion.async_enabled = false — the FusionResult observed for a frame
//!     belongs to exactly that frame;
//!   * async_input = false — the pipeline runs inline on the feeding thread.
//! `publish_images` is NOT touched: whether the image rides along on the debug
//! channel is a per-driver need (the viewer draws it, the CSV driver does not)
//! and it does not change the computed poses.
void force_offline_run_mode(core::SystemConfig& cfg);

//! Builds the fake-anchor configuration in three layers, each overriding the
//! previous one:
//!   1. the defaults in anchor::FakeAnchorConfig;
//!   2. the mission YAML node `FakeAnchor:` (absent ⇒ nothing changes);
//!   3. the UAVLOC_FIX_* environment variables — env always wins, so an
//!      experiment matrix can be driven from the command line without editing
//!      any config file.
//! The groundtruth CSV and its frame-id offset default to the mission
//! VideoReader section (the fake producer reads exactly the log the pipeline is
//! fed); a `FakeAnchor.DroneTelemetry` / `FakeAnchor.frame_id_offset` key
//! overrides that.
//! The producer mode is forced SYNCHRONOUS: an asynchronous one would make the
//! instant a fix enters the graph depend on thread scheduling, and both drivers
//! are used as regression gates.
//! Logs where every_kf / sigma_m / seed came from, so a run's log alone answers
//! "was that value a default, the YAML, or my command line?".
anchor::FakeAnchorConfig fake_anchor_config(
    const YAML::Node& root, const sensor::VideoReaderConfig& reader_cfg);

// ── Assembly shared by the two VIEWER drivers ───────────────────────────────
// tests/test_vo_viewer.cpp and tests/test_vo_vpr_anchor_viewer.cpp differ in
// exactly one thing — whether an absolute-position producer is attached. Every
// other step (parse the mission, force the run mode, open the video, decide the
// run policy, open the pose dump) must be IDENTICAL or the two runs would not
// be comparable, so it lives here instead of being copied.

//! One mission config, parsed into everything a viewer driver needs.
//! `loaded == false` means the file could not be read or parsed — a real
//! failure, not a missing dataset (that is what open_viewer_source reports).
struct MissionSetup {
    bool                      loaded = false;
    YAML::Node                yaml;
    sensor::VideoReaderConfig reader;
    core::SystemConfig        system;
};

//! Loads `config_path`, parses the VideoReader + System sections, then applies
//! the two overrides every viewer driver needs:
//!   * `system.publish_images = true` — the viewer draws the frame and the
//!     tracking overlay, so the image must ride along on the debug channel (off
//!     by default, viewer design R-e). It changes no computed pose;
//!   * force_offline_run_mode() — the same deterministic run mode
//!     test_full_flight uses, so the pose sequences stay comparable.
//! Logs the failure itself; the caller only has to check `loaded`.
MissionSetup load_viewer_mission(const std::string& config_path);

//! Opens the mission video and wraps it in a push-style source.
//! Returns nullptr when the (gitignored) video cannot be opened — the caller
//! must then SOFT-SKIP with return 0. Opening here rather than inside
//! VideoDataSource is what keeps a missing dataset a skip instead of a start()
//! failure. `driver_name` only prefixes the log lines.
std::unique_ptr<sensor::VideoDataSource> open_viewer_source(
    const MissionSetup& mission, const char* driver_name);

//! The run-mode POLICY of a viewer driver: who starts the pipeline, and how
//! much data the run consumes. The two fields are NOT independent settings —
//! they are the two consequences of one question, see viewer_run_policy().
struct RunPolicy {
    bool        autostart  = true;
    //! DebugViewer::Config::max_frames for this run; 0 = until end of stream.
    std::size_t max_frames = static_cast<std::size_t>(PARITY_MAX_FRAMES);
    std::string reason;
};

//! Answers ONE question — "is this a regression run or a visual session?" — and
//! returns BOTH decisions that follow from it. They must be decided together:
//! splitting them is exactly how the frame cap of the regression run leaked
//! into interactive sessions (2026-08-08), where the supervisor hit it after
//! 200 frames, stopped the system, and left a "Start" button that could no
//! longer start anything.
//!
//!   headless (no DISPLAY/WAYLAND_DISPLAY) ⇒ REGRESSION run:
//!       autostart = true — nobody can press a button that was never drawn, so
//!       a headless run (ctest, the parity gate) must start by itself or hang;
//!       max_frames = PARITY_MAX_FRAMES — a gate has to be bounded and has to
//!       compare against the same length every time.
//!   a display                            ⇒ INTERACTIVE session:
//!       autostart = false — starting on its own would rob the operator of the
//!       only moment they can set the view up;
//!       max_frames = INTERACTIVE_MAX_FRAMES (0) — the operator, not a
//!       constant, decides when a visual session has shown enough.
//!
//! Reading DISPLAY here is environment DETECTION, not an experiment switch:
//! it answers "can a window exist on this machine", which is not something a
//! run of the drivers may otherwise configure (both take no environment
//! variable at all — see the note at the top of this file).
//! The DISPLAY/WAYLAND_DISPLAY test matches DebugViewer::run()'s own headless
//! test exactly (set AND non-empty — `DISPLAY=` is headless), or the driver
//! would wait for a button in a window that was never opened.
RunPolicy viewer_run_policy();

//! Opens a fused-pose dump at `path` and writes the header
//! `frame_id,pred_x,pred_y,pred_z` with DUMP_PRECISION digits. The returned
//! stream is closed (i.e. `!is_open()`) when the file could not be created —
//! that is logged as an error but is not fatal: the run itself is still valid.
std::ofstream open_pose_dump(const std::string& path, const char* driver_name);

} // namespace eval
} // namespace uavloc
