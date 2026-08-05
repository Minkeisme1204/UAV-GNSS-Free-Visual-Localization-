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
#include "uavloc/sensor/video_reader.h"

#include <yaml-cpp/yaml.h>

namespace uavloc {
namespace eval {

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

} // namespace eval
} // namespace uavloc
