# CLAUDE.md — UAV Localization (uavloc)

## Project Summary

Bachelor thesis: GPS-denied UAV localization using nadir imagery.
Fuses **Visual Odometry (VO)** and **Visual Place Recognition (VPR)** via a GTSAM factor graph.
All code compiles into **`libuavloc.so`** under the `uavloc` C++ namespace.

**Where the project stands:** VO (`new_vo`) and the GTSAM back-end (`fusion`) are
implemented and measured; `core::SystemManager` packages them into one system object.
**VPR is not built yet** — it is the Phase 2 target, and until it exists the absolute
position fix is supplied by the M1 *fake anchor* (groundtruth-derived, therefore a
contaminated measurement — see `.docs/reports/m1_fake_anchor.md`).

---

## Repository Layout

```
uav_localization/
├── CMakeLists.txt
├── include/uavloc/<module>/    # public headers
├── src/<module>/               # implementations + per-module CMakeLists.txt
├── tests/                      # test executables
├── apps/                       # INDEPENDENT build — never add_subdirectory from root
├── config/                     # YAML mission configs
├── data/                       # imagery / datasets (gitignored)
├── .local/                     # local third-party installs (Iridescence, GLFW)
├── .docs/                      # all Markdown docs + reference PDFs
│   ├── designs/                # module design documents
│   ├── reports/                # phase / milestone reports (Phase1, Phase2 roadmap, M0, M1, S1–S4)
│   ├── theory/                 # theory notes and derivations
│   ├── papers/                 # reference PDFs
│   └── bug_rp/                 # bug root-cause and fix-schedule notes
├── .claude/rules/              # coding, cmake, docs, constraints, reporting, agent_permissions
└── .thirdparty/                # read-only vendored code
    ├── stella_vslam/           #   SLAM reference (new_vo is a port of it)
    └── g2o/                    #   prebuilt; new_vo/optimize links core + stuff
```

> `.claude/rules/reporting.md` is a **hard project rule on data integrity**: no number
> may be written that does not come from a real, reproducible measurement; label
> statements `[đo]` / `[suy ra]` / `[giả thuyết]`; never tune a threshold to make a
> regression match a baseline — stop and report instead.

---

## Modules

Status below is what `src/CMakeLists.txt` actually builds — a commented-out
`add_subdirectory` means the module is **not** in `libuavloc.so`, whatever headers
exist on disk.

| Module         | In build | Status | Key files |
|----------------|----------|--------|-----------|
| `sensor`       | yes | Active | `data_interface.h`, `data_source_interface.h`, `video_data_source.h`, `stream_types.h`, `frame_data.h`, `telemetry_data.h`, `video_reader.h`, `camera_streamer.h`, `camera_model.h`, `geo_reference.h`, `sensor_factory.h` |
| `core`         | yes | Active — **`SystemManager`** (whole-system façade) | `system_manager.h`, `system_config.h`, `system_types.h`, `extrapolator.h` |
| `new_vo`       | yes | Active — the VO in use (stella_vslam port, pure-VO) | `vo_module.h`, `vo_config.h` + subdirs `camera/ data/ feature/ initialize/ match/ module/ optimize/ solve/` |
| `fusion`       | yes | Active — GTSAM fixed-lag smoother (links `gtsam` PRIVATE) | `fusion_module.h`, `fusion_config.h`, `fusion_data.h` |
| `util`         | yes | Active | `callback_slot.h`, `threadsafe_queue.h`, `scoped_timer.h`, `angle.h`, `converter.h`, `yaml.h`, `sys_monitor.h`, … |
| `debug_viewer` | separate lib | Active (`libuavloc_debug_viewer.a`, never linked into `libuavloc`) | `debug_viewer.h`, `debug_viewer_callbacks.h`, `callback_slot.h`, `telemetry_record.h`, `telemetry_csv_reader.h`, `trajectory_viewer.h` |
| `anchor`       | yes | Active (M1) — **request/response seam** for absolute position, kcb-`satcom`-shaped | `anchor_interface.h` (`requestFix`/`setResultCallback`), `anchor_query.h`, `absolute_fix.h`, `fake_anchor.h` |
| `vpr`          | **no** | Stub — Phase 2 target | — |
| `vo` (legacy)  | **no** | **Removed** from the working tree → `src/vo.zip`, `include/uavloc/vo.zip` (2026-07-13) | — |

---

## Sensor Module (implemented)

### Data types
- **`FrameData`** — image + metadata + optional `TelemetryData`; `has_telemetry` flag
- **`TelemetryData`** — heading, pitch, roll, lat/lon, altitude, speed, climb; synced from CSV
- **`FrameStatus`** — `OK | END_OF_STREAM | EMPTY_FRAME | CAMERA_DISCONNECTED | TIMEOUT | ERROR`

### Sources (both implement `DataInterface`)
- **`VideoReader`** — reads `.mp4/.avi/.mkv`; optional telemetry CSV sync by `frame_id` (exact) or `timestamp_msec` (nearest-neighbour fallback)
- **`CameraStreamer`** — live USB/RTSP/GStreamer camera; V4L2/FFMPEG/GSTREAMER backends

### Support classes
- **`CameraModel`** — intrinsics (`Eigen::Matrix3d K`), distortion (`Eigen::Matrix<double,5,1>`), undistort/normalize helpers
- **`TelemetryCsvReader`** — loads CSV, binary-search sync by timestamp, O(1) sync by frame_id
- **`SensorFactory`** — creates `VideoReader` or `CameraStreamer` from `SensorConfig`

### Config pattern
Every config struct has `static XxxConfig fromYaml(const YAML::Node& node)`.
Root YAML key matches the class name: `VideoReader:`, `CameraStreamer:`, `Camera:`, `Sensor:`.

### Telemetry CSV format
Header row required; only `timestamp_msec` is mandatory:
```
timestamp_msec,frame_id,heading_deg,pitch_deg,roll_deg,latitude_deg,longitude_deg,altitude_m,speed_mps,climb_mps
```

---

## Core Module — `SystemManager` (implemented, S1–S9)

`core::SystemManager` is the whole algorithm side of uavloc behind one object:
it owns `new_vo::VOModule` + `fusion::FusionModule`, keeps their thread structure
unchanged, and publishes results. **It deliberately does not own the sensor** —
sensors are independent producers that push in.

### Two ways data enters
- **Push (`onFrame`)** — the caller hands over a whole `sensor::FrameData`. Simple;
  used by the offline evaluation drivers.
- **Channels (kcb-style, 4 typed streams)** — `onAttitude` / `onGimbal` / `onGnss`
  buffer their samples in `core::Extrapolator`; `onImage` fires **last** and triggers
  the processing cycle, reading the other three back at the image timestamp.
  `sensor::VideoDataSource` publishes in exactly that order.
  `attachSource(std::unique_ptr<DataSourceInterface>)` wires a source in; the source
  runs its own reading thread.

Both paths must produce **bit-identical** output for the same input — this is the
standing regression gate (`UAVLOC_INPUT_MODE=frame|channels` in `test_full_flight`).

### Output — `SystemCallbacks` (`callbacks()`)
Subscription is per-instance `util::CallbackSlot`; **no static/global bus**.
`on_localization`, `on_vo_data`, `on_fusion_result`, `on_lag_window`,
`on_frame_processed`, `on_stats`.

### Lifecycle
`setup()` → `start()` → (data flows) → `stop()`. `stop()` is ordered against in-flight
`onFrame` calls by a `std::shared_mutex` gate. `pushAbsoluteFix()` injects an
`anchor::AbsoluteFix` (M1 fake anchor / future VPR).

### Config
`core::SystemConfig::fromYaml` — one file, sub-nodes per module. Missing `Camera:`
**fails loud** (a silent default would let `fx = fy = 0` reach the pipeline).
Design doc: `.docs/designs/system_manager_design.md`.

---

## Anchor Module — absolute position (M1 done, VPR is M3–M5)

`anchor::AnchorInterface` is the **request/response seam** where absolute position enters the
system, shaped after kcb_slam's `SatcomInterface`:

```cpp
virtual bool requestFix(const AnchorQuery&) = 0;          // non-blocking; false = producer busy
virtual void setResultCallback(ResultCallback) = 0;       // void(const AbsoluteFix&)
```

`SystemManager` sends a request **at every keyframe** and **at every VO re-init**
(`AnchorRequestReason::REINIT`, which bypasses the cadence gate). The result callback does exactly
one thing: hand the fix to `fusion_->push_absolute_fix()`. **No graph work on the producer thread** —
the same discipline kcb keeps.

`fusion` never learns where a fix came from, so swapping `FakeAnchor` for real VPR needs **no
back-end change**. `AbsoluteFix` is X/Y only — the vertical is already carried by `AglFactor`.

**`FakeAnchor` reads groundtruth directly** (approved: it becomes dead code once VPR exists). The
ENU origin is **pushed down by `SystemManager`** via `setEnuOrigin()` at the geo-anchoring instant —
the producer must never read an origin from config, or the two would drift apart silently.

Lifecycle logging: `anchor[REQ]` → `anchor[EMIT]` → `anchor[APPLY]`, correlated by `ts`. **Every
reject path prints an APPLY line with its reason** (`GATED / AGE_EXPIRED / UNMATCHED /
MARGINALIZED / LOW_CONFIDENCE`) — the M1 lesson was that 84 % of fixes were being dropped at
`debug` level where nobody saw them.

### The measured law that constrains VPR
> **An absolute fix only carries new information when its σ is smaller than the drift accumulated
> since the last accepted fix.** Below that, injecting it only adds noise.

Measured: ds6 at 800 frames (drift ≈ 19 m) got **worse** with σ = 20 m and better with σ = 5 m;
YenBai over the full 22.98 km (drift ≈ 1 990 m) improved 15× with σ = 5 m. See
`.docs/reports/m1_acceptance.md` §4.

---

## Util Module

`callback_slot.h` (signature-specialised `CallbackSlot<void(Args...)>`; copies the
subscriber list under lock, then invokes **unlocked**, so a subscriber may re-enter),
`threadsafe_queue.h` (BLOCK / DROP_OLDEST), `scoped_timer.h` (the `Profiler` used by
the M0 stage profiling), plus `angle/converter/yaml/sys_monitor` helpers.
`transforms.h` is still a draft.

---

## VO Module — ⚠ LEGACY, NOT IN THE BUILD

> **This section describes `src/vo/`, which was removed from the working tree on
> 2026-07-13** (`src/vo.zip`, `include/uavloc/vo.zip`) and is commented out in
> `src/CMakeLists.txt`. The VO actually running is **`new_vo`** — a port of
> stella_vslam stripped to pure VO (no marker / BoW / loop-closing / relocalisation),
> public entry points `include/uavloc/new_vo/vo_module.h` + `vo_config.h`, namespace
> `uavloc::vo`. The text below is kept as design history; do not treat its file paths
> as current.

> **Current state (on disk, 2026-07-11).** The full pipeline is implemented and the
> header layout has been reorganised. All VO **data types** (structs/enums) live in a
> single **`include/uavloc/vo/common.h`**; **`ProjectionMatcher` + `Tracker`** (with
> their configs) are bundled into public **`include/uavloc/vo/tracking.h`**. The old
> `vo_data.h`, `projection_matcher.h`, `src/vo/tracker.h`, `src/vo/local_map_types.h`,
> and `frame_sequence.{h,cpp}` have been **removed**. Config structs stay next to their
> owning class. `.claude/rules/coding.md` carries a VO exemption allowing common.h /
> tracking.h to expose internal types (`Landmark`, `Keyframe`, `KeyframePacket`, `Tracker`).

### Header layout (current)
- **`include/uavloc/vo/common.h`** — every data type: `FeatureSet`, `MatchesData`,
  `VOPoseData`, `VOResult`, `Observation`, `Landmark` (non-copyable — holds
  `std::atomic<int> num_visible/num_found`), `Keyframe`, `LandmarkSnapshot`,
  `KeyframePacket`, and the enums `VOTrackingState` (`NOT_INITIALIZED | INITIALIZED |
  TRACKING | LOST | FAILED`), `VOFailureReason`, `DescriptorType`, `FeatureStatus`,
  `MatchStatus`, `FeatureBackend`, `PoseStatus`, `TrackStatus`, `InitStatus`, `LandmarkState`.
- **`include/uavloc/vo/tracking.h`** — `ProjectionMatcher` (+`ProjectionMatcherConfig`)
  and `Tracker` (+`TrackerConfig`); forward-declares `LocalMap`/`LocalMapper` (held by
  ref), includes `common.h` + `local_optimizer.h`.
- Other public headers: `feature_detector.h`, `pose_estimator.h`, `local_optimizer.h`,
  `vo_module.h`. Private (`src/vo/`): `local_map.h`, `local_mapper.h`, `initializer.h`,
  `frame_queue.h`.

### Public API
- **`VOModule`** — pimpl (`include/uavloc/vo/vo_module.h`). **Synchronous default**:
  `VOResult processFrame(const sensor::FrameData&)` — deterministic, bit-for-bit
  reproducible (used by all tests/baseline). **Opt-in async mode** (`VOConfig::async_enabled`):
  `start()` / `stop()` spawn the tracking + local-mapping threads; `bool pushFrame(FrameData)`
  is non-blocking drop-oldest; results delivered via `setVOResultCallback()` (fires per
  frame, in frame order, on the tracking thread — **non-deterministic**). Also
  `setStatusCallback()` / `setKeyframeCallback()`.
- **`VOResult`** — per-frame: `frame_id`, `timestamp_msec`, `state` (`VOTrackingState`),
  `T_wc` (GLOBAL camera→world), `T_prev_curr` (GLOBAL increment), `has_pose`, `is_keyframe`,
  `num_keypoints`, `num_matches`, `num_inliers`, `num_landmarks`, `inlier_ratio`,
  `tracked_observations`. (`num_matches`/`inlier_ratio` are filled in BOTH init and tracking.)
- **`VOConfig`** — `VOConfig::fromYaml(node["VO"])`; embeds `FeatureDetectorConfig`,
  `ProjectionMatcherConfig`, `PoseEstimatorConfig`, plus `async_enabled`,
  `frame_queue_capacity`, `frame_queue_pop_timeout_ms`, `off_nadir_source`, `reinit_refresh_frames`,
  and a raw `YAML::Node vo_node` used to build the private Tracker/Initializer/LocalMapper configs.

### Pipeline (current, two threads in async mode)
```
FrameQueue → Tracking thread: detect(ORB) → [NOT_INITIALIZED] Initializer (homography-first
             two-view, parallax-gated, metric seed from altitude)  |  [TRACKING] project
             LocalMap landmarks → ProjectionMatcher → solvePnPRansac → PoseOptimizer
             (motion-only BA) → keyframe gate → submit KeyframePacket
                                                       │
Local Mapping thread ←──── KeyframePacket ────────────┘
             insert KF + observations → triangulateNewLandmarks (1-ref) → cullLandmarks
```
In synchronous mode the same steps run inline on the calling thread (no threads spawned).

### Internal components (`src/vo/`)
- **`VOModule::Impl`** — state machine (`NOT_INITIALIZED→INITIALIZED→TRACKING`, LOST weld/re-init),
  `T_world_anchor` welding, `metricScaleFromTelemetry`, async thread orchestration.
- **`Initializer`** — holds first frame as fixed reference, accumulates parallax, `tryInitialize`
  → homography pose (metric scale from altitude) → `cv::triangulatePoints` → seeds `LocalMap`.
- **`Tracker`** — constant-velocity motion model → project landmarks → `ProjectionMatcher` →
  `cv::solvePnPRansac` → `PoseOptimizer`; keyframe gate (SVO depth-normalized `d/depth`).
- **`LocalMapper`** — runs on the mapping thread; `triangulateNewLandmarks` + `cullLandmarks`;
  owns its own `ProjectionMatcher`. `KeyframePacket` is the thread-boundary hand-off.
- **`LocalMap`** — `unordered_map` of `Landmark`/`Keyframe` (shared_ptr, id-keyed), guarded by
  `map_mutex_`; atomic id counters; `snapshotLandmarks()` for lock-free tracking reads;
  `cullLandmarks()` marks bad if `obs_count < 2` or `num_found/num_visible < 0.25`.
- **`PoseOptimizer`** (`local_optimizer.{h,cpp}`) — motion-only BA, Eigen-only (hand se3 exp,
  Huber + per-frame MAD σ, landmarks fixed). Not g2o.
- **`FrameQueue`** — bounded, drop-oldest, mutex+CV (async input queue).

### Pending (design target, NOT yet on disk)
- **Phase B — covisibility graph** (K1/K2 local-map selection, multi-ref triangulation).
- **Phase C — multi-keyframe local Bundle Adjustment** (solver g2o-vs-GTSAM undecided).
- **Per-point triangulation-angle gate** — currently absent; triangulation is gated only by
  aggregate median PIXEL disparity (proxy) + per-point cheirality + reprojection, so
  low-parallax landmarks (~0.086°/frame at 800 m) still enter the map.

### SE3 convention
- `T_wc` = camera-to-world transform; first frame is identity.
- `T_cw = T_wc.inverse()`.
- `T_prev_curr = T_wc_curr * T_wc_prev.inverse()` (world-frame increment).
- `res.T_wc` is always the last valid pose; `res.has_pose` gates whether the result should be consumed.

### RNG seeding
OpenCV RNG is seeded to fixed values before every RANSAC call for reproducibility:
- H thread: `0xABCD1234`
- F thread: `0x5678EF00`
- PnP:      `0xDEADBEEF`

### Config
YAML key `VO:` in the mission config file (e.g. `config/uavloc_yenbai800m.yaml`).
Load via `VOConfig::fromYaml(node["VO"])`. All fields have defaults; missing keys never throw.
Key fields: `num_features`, `scale_levels`, `scale_factor`, `ini_fast_threshold`, `min_fast_threshold`, `min_parallax_deg`, `min_triangulated_pts`, `max_reproj_error_px`, `ransac_iterations`, `min_inliers_to_track`, `pose_opt_max_reproj_error_px`, `search_radius`, `kf_min_frame_gap`, `kf_min_inlier_ratio`, `kf_min_tracked_landmarks`, `frame_queue_capacity`, `undistort_image`, `descriptor_dist_threshold`, `lowe_ratio_threshold`, `ransac_reproj_threshold`, `pnp_iterations`, `pnp_confidence`, `frame_queue_pop_timeout_ms`.

### Known limitations
- Only **motion-only BA** exists (`PoseOptimizer`, landmarks fixed); no multi-keyframe local BA yet (Phase C).
- No per-point triangulation-angle gate (see Pending above); low parallax (~0.086°/frame @800 m) yields noisy landmark depth → Z-drift.
- Monocular scale is inherently ambiguous; metric scale seeded from altitude at init but drifts — a proper fix belongs in the `fusion` back-end (altitude prior + scale state), not front-end rescale (that was tried and failed).
- Async mode is non-deterministic by design; deterministic baseline requires the synchronous `processFrame` path.

---

## Debug Viewer Module (implemented)

Independent visualization library (`libuavloc_debug_viewer.a`). Does **not** link against `libuavloc` — communicates exclusively through its own data types. Requires Iridescence (installed in `.local/`); the whole module is omitted from the build if Iridescence is not found.

### Public API (`include/uavloc/debug_viewer/`)
- **`TelemetryRecord`** — data interface struct: `frame_id`, `latitude`, `longitude`, `altitude_m`, `roll_deg`, `pitch_deg`, `yaw_deg`, `ground_speed`
- **`load_telemetry_csv(path)`** — parses the drone telemetry CSV; skips GPS-not-locked rows; returns `std::vector<TelemetryRecord>`
- **`DebugViewer`** — main feature class (pimpl). Streams a video, decodes the per-frame barcode, builds the live 3D ENU groundtruth trajectory, and shows the current frame in a sub-window. `loadDirectory()` (auto-finds the `.csv` + video), `loadBatch()`, thread-safe `pushRecord()` (for an externally-computed/inferred trajectory), blocking `run()`, and `stop()`.
- **`DebugViewerCallbacks`** / **`CallbackSlot<>`** — thread-safe (mutex-guarded) publish/subscribe slots for algorithm↔viewer communication: `on_telemetry(TelemetryRecord)`, `on_status(string,string)`, `on_metric(double,float,string)`. See `.docs/designs/debug_viewer_communication.md`.
- **`TrajectoryViewer`** — lighter pimpl viewer for a pre-loaded record set; `setRecords()` converts GPS→ENU and builds drawables; `spin()` / `spinOnce()`.

### Barcode-driven groundtruth (YenBai 800 m)
Each video frame carries a **CODE-128** barcode whose payload is a zero-padded 7-digit `imageId` (NOT the telemetry itself). `DebugViewer` decodes it (zbar, `src/debug_viewer/barcode_decoder.{h,cpp}`), looks up the full `TelemetryRecord` from the CSV by `imageId`, converts GPS→ENU, and grows the trajectory. Headless-safe: when no `DISPLAY`/`WAYLAND_DISPLAY` is present (or GL init throws) it skips rendering, still decodes + fires callbacks, and exits cleanly.

### Config (`DebugViewer::Config`)
All fields have defaults; construct with `{}` to use them:
- `window_title`, `window_w`, `window_h` — window properties
- `max_trajectory_pts` — cap on stored points
- `frame_stride` — process 1 of every N frames (skipped frames use `cap.grab()`, no decode); higher = faster traversal
- `vector_every_n`, `arrow_scale` — heading arrow cadence/length (metres)
- `coord_frame_every_n`, `coord_frame_scale` — UAV orientation gizmos (metres; `0` disables)
- `display_scale` — metres→render-unit multiplier so a multi-km path fits on screen (e.g. `0.01` ⇒ 1 unit = 100 m)
- `vertical_scale` — extra multiplier on the Up/altitude axis only (`>1` exaggerate, `<1`/`0` flatten relative to the horizontal plane)

### GPS → ENU conversion
Flat-earth formula (`src/debug_viewer/gps_to_enu.h`) anchored at the first record. Scale constants for ~21° N: `M_LAT = 111319.5 m/deg`, `m_lon = 111319.5 × cos(lat₀) ≈ 103 700 m/deg`. Sufficient for trajectories ≤ 5 km; no external dependency.

### Telemetry CSV format (YenBai drone log)
Header row required (57 columns). Key columns (0-based): `0` imageId, `21` roll, `22` pitch, `23` yaw, `26` groundSpeed, `30` sensorLatitude, `31` sensorLongitude, `32` sensorAltitude. The CSV is the full flight log (`imageId` 1…~91741); the sample `.mkv` is a cut whose first barcode is `imageId 55166`.

### Dependencies
| Library | Purpose |
|---------|---------|
| Iridescence | OpenGL + ImGui 3D viewer (`guik::LightViewer`, `glk::create_texture`) |
| zbar | Per-frame barcode decoding (`pkg_check_modules` → `PkgConfig::ZBAR`) |
| OpenCV 4.x | `cv::VideoCapture`, image ops |
| Eigen 3.x | ENU math and pose transforms |
| spdlog | Logging |

Iridescence is installed under `.local/` (gitignored). CMake finds it via `find_package(iridescence QUIET PATHS .local/lib/cmake/iridescence)`.

---

## Tests and Demo

Test sources live in `tests/`; targets land in `build/tests/`. Targets whose
source is absent during the refactor (`test_vo`, `vo_demo`, and the
Iridescence-gated `demo_debug_viewer` / `debug_viewer_decode`) are guarded with
`if(EXISTS ...)` and silently skipped.

`tests/eval_common.{h,cpp}` (→ `uavloc_test_eval`, tests-only STATIC lib) holds the
**shared scoring ruler**: `percentile` (nearest-rank), `least_squares_slope`,
`aligned_rms_2d` (4-DoF yaw+translation fit), `dump_profile`. Two drivers must score
with the same code or their numbers cannot be compared — changing a body there
silently invalidates every p95/RMS already published.

### Current core tests (registered with `ctest`)
| Target | Covers |
|---|---|
| `test_system_manager` | `SystemManager` lifecycle, callbacks, push path |
| `test_extrapolator` | per-channel buffers, linear interp, **SLERP** for angles, exact-timestamp verbatim |
| `test_system_config` | YAML load; missing `Camera:` must throw |
| `test_util_callback` | `CallbackSlot` re-entrancy + thread safety |
| `test_data_source` | `VideoDataSource` streaming thread, channel firing order, EOS |
| `test_fusion_smoke`, `test_fusion_factors` | GTSAM graph + the custom factors |
| `test_anchor` | `AnchorInterface`/`FakeAnchor`: cadence, REINIT bypass, data gates, seed reproducibility, both delivery modes |
| `test_driver_parity` | **cross-driver gate**: `test_full_flight` and `test_vo_viewer` must produce the same trajectory (per-frame difference constant to ≤ 1e-6 m — the viewer dump is display-anchored, so only the offset may differ) |
| `test_full_flight` | offline evaluation driver — 37-column CSV, the ATE/p95/slope numbers in the reports |
| `test_vo_viewer` | `SystemManager` + `DebugViewer` live overlay; `UAVLOC_VIEWER_DUMP` gives a deterministic headless pose dump. ⚠ **hangs under `ctest` when `DISPLAY` is set** — it waits for the window to close; run headless |

### Measurement discipline — four traps that have already cost real time

1. **`use_fixed_seed: true` must be in the mission YAML.** The code default is `false`
   (`src/new_vo/module/vo_config.cpp:29`), which seeds the initializer's H/F RANSAC from
   `std::random_device` — **two runs of the same config then diverge at the frame where VO
   initialises**. `config/uavloc_yenbai800m.yaml` was missing it, which silently invalidated two
   whole measurement matrices before anyone noticed. All 5 configs now set it; verify before
   trusting any comparison.
2. **Frame cap is the env var `UAVLOC_VO_MAXFRAMES`, not a positional argument.** Passing it as
   `argv[3]` is silently ignored and you get a full-flight run.
3. **`pgrep test_full_flight` never matches** — Linux truncates `comm` to 15 chars, so the pattern
   must be `test_full_fligh`. Checking with the full name reports "nothing running" while six
   processes are.
4. **Give every run its own output path.** A run killed by a timeout can leave the child alive,
   still writing — two writers on one CSV produce a file that looks fine and compares wrong.

### `test_video_reader`
Opens a video via `VideoReader` (config-driven) and reads frames.

### `test_adjacent_matching`
Loads two adjacent frames via `VideoReader`, runs the ORB `IFeatureDetector` on
each, matches them with `ProjectionMatcher` (no prior), logs keypoint/match
stats, and writes a `cv::drawMatches` visualization to `adjacent_matches.png`
(headless-safe).

```bash
# From build/
./tests/test_adjacent_matching [config.yaml]
```

### `test_vo_pipeline`
Runs the full two-view VO pipeline end-to-end (`VideoReader` → ORB
`IFeatureDetector` → `ProjectionMatcher` → `PoseEstimator`). Anchors the first
valid frame as the world origin, chains each recovered relative pose onto `T_wc`
(`T_wc = T_wc * T_prev_curr`), and writes an accumulated top-down trajectory
image `vo_trajectory.png`. Monocular translation is up-to-scale, so the plotted
trajectory shape is qualitative, not metric. Headless-safe (no GUI).

```bash
# From build/
./tests/test_vo_pipeline [config.yaml]   # default: config/uavloc_yenbai800m.yaml
```

### `test_vo_debug`
Compact diagnostic: runs the first 20 valid frames through the packaged
`VOModule::processFrame` (shares the same config as `test_vo_pipeline`), logs a full
per-frame block (state, keypoints/matches/inliers/landmarks, `inlier_ratio`, `T_wc` and
`T_prev_curr` translation, telemetry), and writes a top-down trajectory image
`vo_debug_trajectory.png` (world X–Y, altitude dropped, heading arrows from `T_wc` rotation).
Headless, synchronous.

```bash
# From build/
./tests/test_vo_debug [config.yaml] [max_frames]   # default: config/uavloc_yenbai800m.yaml, 20
```

### `demo_debug_viewer`
The barcode-driven groundtruth trajectory viewer. Scans a directory for the
`.csv` + video, streams the video, decodes each frame's CODE-128 barcode →
`imageId` → CSV telemetry, and grows the live 3D ENU trajectory with a video
sub-window. Exits cleanly headless (no `DISPLAY`).

```bash
# From build/
./tests/demo_debug_viewer [directory | video_path]   # default: data/YenBai800m
```

### `test_iridescence`
Minimal Iridescence smoke test (points, polyline, coord axes, ImGui panel). Built only when Iridescence is found.

---

## CMake Hierarchy

```
src/<module>/CMakeLists.txt   →  target_sources(uavloc PRIVATE ...)
src/CMakeLists.txt            →  add_library(uavloc SHARED) + add_subdirectory per module
src/debug_viewer/CMakeLists.txt → STATIC lib uavloc_debug_viewer (Iridescence, independent)
CMakeLists.txt (root)         →  find_package deps + add_subdirectory(src) + add_subdirectory(tests)
                                 + conditional add_subdirectory(src/debug_viewer) if Iridescence found
apps/CMakeLists.txt           →  independent: find_package(uavloc REQUIRED)
```

Activate a new module by uncommenting its `add_subdirectory` line in `src/CMakeLists.txt`.
Currently enabled: `sensor`, `util`, `new_vo`, `fusion`, `core`, `anchor`. Still commented out:
`vo` (legacy, zipped), `vpr`, `utils`.
`find_package(GTSAM)` is already in root (`fusion` links `gtsam` PRIVATE);
`find_package(onnxruntime)` is still needed before enabling `vpr`.

**R4 (hard constraint):** `libuavloc` must never link `uavloc_debug_viewer`. The bridge
between algorithm and viewer lives in the driver (`tests/test_vo_viewer.cpp`), which
links both. Verify with
`nm -DC build/lib/libuavloc.so | grep -c debug_viewer` → must be `0`.

---

## Dependencies

| Library      | Linked on   | Purpose |
|--------------|-------------|---------|
| OpenCV 4.x   | `uavloc` PUBLIC | Image I/O, VideoCapture, geometry |
| Eigen 3.x    | `uavloc` PUBLIC | Matrix math, camera K, SE3 |
| spdlog       | `uavloc` PUBLIC | Logging |
| yaml-cpp     | `uavloc` PUBLIC | Config parsing (`yaml-cpp::yaml-cpp`) |
| Iridescence  | `uavloc_debug_viewer` PUBLIC | OpenGL/ImGui 3D viewer (optional; module skipped if absent) |
| zbar         | `uavloc_debug_viewer` PRIVATE | Per-frame barcode decoding (`PkgConfig::ZBAR`) |
| GTSAM 4.x    | `uavloc` PRIVATE (via `src/fusion/`) | Factor graph optimization — `IncrementalFixedLagSmoother` |
| g2o          | `uavloc` (via `new_vo/optimize`) | VO optimisation. **Prebuilt in-tree** at `.thirdparty/g2o/build`; only `core` + `stuff` are imported (`g2o::core`, `g2o::stuff`) — no `find_package` |
| ONNXRuntime  | future `vpr`    | Neural descriptor inference |
| stella_vslam | reference only  | Vendored, read-only |

---

## Building

```bash
# Library + tests
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run video reader test
./tests/test_video_reader

# Install, then build apps
cmake --install . --prefix /opt/uavloc
cd ../../apps && mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/opt/uavloc && make -j$(nproc)
```

`compile_commands.json` is generated automatically in `build/` — point clangd at it to fix IDE include errors.
