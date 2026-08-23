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
│   ├── README.md               # index of every document (maintained by hand)
│   ├── designs/                # module design documents — how the system SHOULD work
│   ├── reports/                # phase / milestone reports (Phase1, Phase2 roadmap, M0, M1, S1–S9)
│   ├── theory/                 # theory notes and derivations (code-version independent)
│   ├── datasets/               # input-data descriptions (YenBai, MUN-FRL, HoaLac)
│   ├── related_work/           # analyses of other people's systems
│   │   ├── kcb_slam/           #   the GPS-denied UAV system used as design counterpart
│   │   ├── orb_slam/           #   ORB-SLAM study notes
│   │   └── svo/                #   SVO Pro study notes
│   ├── thesis/                 # drafts destined for the thesis manuscript
│   └── papers/                 # reference PDFs
│       # (bug_rp/ removed 2026-08-06 — all three notes were about the legacy `vo`
│       #  module and the deleted tests/vo_demo.cpp; archived in legacy_docs_archive_*.zip)
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
| `debug_viewer` | conditional | Active — a module **inside** `libuavloc` since 2026-08-08, gated on `ENABLE_VIEWER` (ON for development, **OFF for deployment**; see revised R4) | `debug_viewer.h`, `debug_viewer_callbacks.h`, `callback_slot.h`, `telemetry_record.h`, `telemetry_csv_reader.h`, `trajectory_viewer.h`, `trajectory_aligner.h` |
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
YAML key `VO:` in the mission config file (e.g. `config/uavloc_yenbai800m_newvo.yaml`).
Load via `VOConfig::fromYaml(node["VO"])`. All fields have defaults; missing keys never throw.
Key fields: `num_features`, `scale_levels`, `scale_factor`, `ini_fast_threshold`, `min_fast_threshold`, `min_parallax_deg`, `min_triangulated_pts`, `max_reproj_error_px`, `ransac_iterations`, `min_inliers_to_track`, `pose_opt_max_reproj_error_px`, `search_radius`, `kf_min_frame_gap`, `kf_min_inlier_ratio`, `kf_min_tracked_landmarks`, `frame_queue_capacity`, `undistort_image`, `descriptor_dist_threshold`, `lowe_ratio_threshold`, `ransac_reproj_threshold`, `pnp_iterations`, `pnp_confidence`, `frame_queue_pop_timeout_ms`.

### Known limitations
- Only **motion-only BA** exists (`PoseOptimizer`, landmarks fixed); no multi-keyframe local BA yet (Phase C).
- No per-point triangulation-angle gate (see Pending above); low parallax (~0.086°/frame @800 m) yields noisy landmark depth → Z-drift.
- Monocular scale is inherently ambiguous; metric scale seeded from altitude at init but drifts — a proper fix belongs in the `fusion` back-end (altitude prior + scale state), not front-end rescale (that was tried and failed).
- Async mode is non-deterministic by design; deterministic baseline requires the synchronous `processFrame` path.

---

## Debug Viewer Module (implemented)

Visualization module **of `libuavloc`** (since 2026-08-08 — it used to be the independent static
library `libuavloc_debug_viewer.a`; see the revised R4 under "CMake Hierarchy"). It still talks to
the algorithm side only through its own data types, so it depends on no `uavloc` algorithm class.
Compiled in only when `ENABLE_VIEWER=ON` (development default) **and** Iridescence is found
(installed in `.local/`); a deployment build (`-DENABLE_VIEWER=OFF`) omits it entirely and the
resulting `libuavloc.so` carries no GL/GLFW/zbar dependency.

Design doc: **`.docs/designs/viewer_module_design.md`** — the three layers
(`Config::fromYaml` → `TrajectoryAligner` → `DebugViewer`+`SystemBridge`), the
`attach()/run()/stop()` contract (and why `run()` must own the calling thread), the four
callback channels and their emission order, and the distinction that is easiest to get
wrong: **`T_align` is the ESTIMATION layer** (4-DoF, frozen on purpose so VO drift stays
visible) while **`g₀ + (f − f₀)` is the DISPLAY layer** (pure translation between two
different ENU origins).

`SystemBridge` (`src/debug_viewer/debug_viewer_bridge.{h,cpp}`, module-private) is what turns
`core::SystemManager` events into what the viewer draws, and since S4 also owns the run
(auto-start + end-of-data supervisor + perf sampler). `DebugViewer::attach(sys)` /
`run()` / `stop()` is the whole public seam — a driver builds the system and hands it over.

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
source is absent (`test_vo`, `vo_demo`, and the viewer-gated
`demo_debug_viewer` / `debug_viewer_decode`) are guarded with `if(EXISTS ...)`
and silently skipped. `demo_debug_viewer`, `debug_viewer_decode`, `test_viewer_align`,
**`test_vo_viewer`, `test_vo_vpr_anchor_viewer` and `test_driver_parity`** are additionally
gated on `ENABLE_VIEWER AND Iridescence_FOUND` — with `-DENABLE_VIEWER=OFF` they are not
built (ctest goes 19 → 15 registered tests).

> **Parity is an ON-ONLY gate (decided 2026-08-08).** The two viewer drivers lost their
> `#ifdef UAVLOC_WITH_DEBUG_VIEWER` fallback branch: a driver whose whole subject is the live
> view has nothing to do without it, so at OFF neither is built and `test_driver_parity` — which
> reads `test_vo_viewer`'s dump — cannot run either. Accepted deliberately; recorded here and in
> `.docs/designs/viewer_module_design.md` §7.

> **The two viewer drivers take NO environment variable** (2026-08-08). Frame cap and dump paths
> are named constants in `tests/driver_common.h` (`PARITY_MAX_FRAMES = 200`, `VIEWER_DUMP_PATH`,
> `ANCHOR_VIEWER_DUMP_PATH`); `test_driver_parity` reads the SAME constants instead of passing
> env to the viewer (it still passes `UAVLOC_VO_MAXFRAMES` to `test_full_flight`). Profiling is a
> `constexpr bool` in each driver, default `false`. Both drivers always write their dump.
> Because the dump path is compiled in, `test_vo_viewer` and `test_driver_parity` share a ctest
> `RESOURCE_LOCK "viewer_pose_dump"` — two writers on one CSV compare wrong.

> **Auto-start AND the frame cap are ONE decision** (`eval::viewer_run_policy()`, fixed
> 2026-08-08). It answers "regression run or interactive session?" and returns both:
> headless ⇒ `autostart = true, max_frames = PARITY_MAX_FRAMES (200)`; a `DISPLAY` ⇒
> `autostart = false, max_frames = INTERACTIVE_MAX_FRAMES (0 = uncapped)`. Reading `DISPLAY`
> there is environment *detection*, the only env read left in the two drivers. Deciding the two
> separately is what broke interactive runs: the 200-frame cap fired, the supervisor called
> `sys.stop()`, and the button still said "Start" while `resumeSource()` could only be refused.
> Half two of that fix is the terminal `DebugViewer::RunState::FINISHED` — after the supervisor
> shuts the pipeline down the button is drawn **disabled and labelled "Finished"**, never
> "Start". The drivers still use `runState() == IDLE` to detect a REFUSED auto-start.
> Covered by `test_viewer_run_policy` (the GUI half cannot be exercised headless).

`tests/eval_common.{h,cpp}` (→ `uavloc_test_eval`, tests-only STATIC lib) holds the
**shared scoring ruler**: `percentile` (nearest-rank), `least_squares_slope`,
`aligned_rms_2d` (4-DoF yaw+translation fit), `dump_profile`. Two drivers must score
with the same code or their numbers cannot be compared — changing a body there
silently invalidates every p95/RMS already published.

`tests/driver_common.{h,cpp}` (same library) is the **shared INPUT side**: `force_offline_run_mode`,
`fake_anchor_config`, the constants above, and — since 2026-08-08 — the assembly the two viewer
drivers share verbatim: `load_viewer_mission` (parse + `publish_images` + forced run mode),
`open_viewer_source` (open the video, soft-skip when absent), `viewer_run_policy` (headless ⇒
auto-start) and `open_pose_dump`. The two drivers then differ in exactly one thing: whether an
`anchor::AnchorInterface` producer is attached.

### Test I/O contract — three rules, no exceptions

Broken independently in the past; each one produced a test that looked fine and
measured nothing. Verified clean 2026-08-06 **[đo]**.

1. **No path literal in a `.cpp`.** The mission config is injected by CMake as
   `target_compile_definitions(<target> PRIVATE UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/<x>.yaml")`
   and read through an `#ifndef`-guarded macro; `argv[1]` still overrides it. A hard-coded
   `/home/<user>/…/config/…` makes the test unrunnable for anyone else and invisible to a
   config rename — that is how 12 tests came to point at a config that had been deleted.
   Check with `grep -rn "/home/.*config/" tests/*.cpp` → must be empty. Dataset paths under
   `data/` are exempt (gitignored, user-managed).
2. **Datasets are optional: soft-skip, never fail.** `data/` is gitignored, so a test whose
   video/CSV is missing must `spdlog::warn(... SKIPPED)` and `return 0`. Reserve non-zero
   for a genuine gate failure — otherwise a missing dataset is indistinguishable from a bug.
3. **Headless is the default; every run is bounded.** Never call highgui unconditionally —
   gate on (`DISPLAY` ‖ `WAYLAND_DISPLAY`) **and** the test's own `UAVLOC_*_SHOW` env var,
   which ctest pins to `0`. Every `add_test` needs a `TIMEOUT`, and any full-stream loop
   needs a frame cap. `test_video_reader` had none of this: it aborted on a GTK init failure
   headless, and with a display it would have run all 17 952 frames and hung ctest forever.

### Acceptance criteria — what each registered test actually gates

Every test uses the same idiom: a local `check(bool, const char*)` lambda that sets
`rc = 1` and logs `FAIL: <what>`, ending in `PASS` + `return rc`. **No `assert`** — asserts
vanish in `NDEBUG` Release builds, which is how this project builds.

**Baseline 2026-08-07 [đo]: `15/15 passed` headless** (`cd build && unset DISPLAY WAYLAND_DISPLAY; ctest -j2`, 55.7 s).
**Baseline 2026-08-08 (S5a) [đo]: `17/17 passed` headless with `ENABLE_VIEWER=ON`** (108.7 s,
serial) and **`15/15 passed` with `ENABLE_VIEWER=OFF`** in a separate `build_off/` (73.8 s);
the four viewer-gated targets were not registered at OFF.
**Baseline 2026-08-08 (S6) [đo], after `test_driver_parity` was un-gated: `17/17 passed`
at ON (108.5 s, serial) and `16/16 passed` at OFF (106.2 s, serial)**, both with
`env -u DISPLAY -u WAYLAND_DISPLAY ctest --output-on-failure`. Three targets
(`demo_debug_viewer`, `debug_viewer_decode`, `test_viewer_align`) remain unregistered at OFF.
**Baseline 2026-08-08 (V1–V4: env vars removed, `test_vo_vpr_anchor_viewer` added, the viewer
drivers + parity moved behind `ENABLE_VIEWER`) [đo]: `18/18 passed` at ON (126.6 s, serial) and
`14/14 passed` at OFF (58.1 s, serial)**, same command. Six targets are unregistered at OFF
(`demo_debug_viewer`, `debug_viewer_decode`, `test_viewer_align`, `test_vo_viewer`,
`test_vo_vpr_anchor_viewer`, `test_driver_parity`). Pose-sequence regression re-verified the
same day: `test_vo_viewer`'s 193-row dump is **byte-identical** to `head -n 193
build/tests/s0_GOLDEN.csv` (md5 `6d8b4df1ab4d21030b1bcabd21b04b37` on both).
**Baseline 2026-08-08 (V5: the frame cap moved into `eval::viewer_run_policy()`, terminal
`RunState::FINISHED`, new `test_viewer_run_policy`) [đo]: `19/19 passed` at ON (127.3 s, serial)
and `15/15 passed` at OFF (58.0 s, serial)**, `env -u DISPLAY -u WAYLAND_DISPLAY ctest`.
Pose-sequence regression re-verified the same day: the 193-row dump still `cmp`s silently
against `head -n 193 build/tests/s0_GOLDEN.csv` (headless is still capped at 200).

### Current core tests (registered with `ctest`)
They take their config from `UAVLOC_MISSION_CONFIG_PATH` unless the Input column says
otherwise; `argv[1]` overrides everywhere. 20 registered at `ENABLE_VIEWER=ON`, 16 at OFF.

| Target | Input | Gates (acceptance criteria) |
|---|---|---|
| `test_system_manager` | munfrl_dataset3 | lifecycle, callbacks, push path; dataset checks soft-skip |
| `test_system_config` | munfrl | YAML load; **missing `Camera:` must throw** |
| `test_extrapolator` | none (pure unit) | per-channel buffers, linear interp, **SLERP** for angles, exact-timestamp verbatim |
| `test_util_callback` | none (pure unit) | `CallbackSlot` re-entrancy + thread safety |
| `test_data_source` | munfrl_dataset3 | `VideoDataSource` streaming thread, channel firing order, EOS |
| `test_anchor` | none (writes own CSV) | `AnchorInterface`/`FakeAnchor`: cadence, REINIT bypass, data gates, seed reproducibility, both delivery modes |
| `test_fusion_smoke`, `test_fusion_factors` | synthetic | GTSAM graph + the custom factors (10 numbered checks) |
| `test_video_reader` | yenbai800m_newvo | fps/frame-count > 0; frame `valid` + non-empty + **constant image size**; `frame_id` strictly increasing, `timestamp_msec` non-decreasing; telemetry arrives when the config enables it. Cap `UAVLOC_VR_MAXFRAMES` (100), GUI gated by `UAVLOC_VR_SHOW` |
| `test_drone_telemetry` | yenbai800m_newvo → `VideoReader.DroneTelemetry` node | `size() > 0`; `byFrameId` round-trip; **every mapped field finite** (no NaN/inf); `offNadirDeg() ∈ [0, 90]`; `toTelemetryData()` preserves altitude/speed; records ordered by `frame_id`. `argv[2]` overrides csv_path only |
| `test_vo_frame_loader` | yenbai800m_newvo | `data::Frame` well-formed: keypoints > 0, descriptor rows == keypoint count, bearings match, grid built, id/timestamp pass through |
| `test_vo_pipeline_new` | yenbai800m_newvo | `new_vo` end-to-end via `VOModule`; `UAVLOC_VO_MAXFRAMES`, `UAVLOC_VO_FULL` |
| `test_vo_viewer` | yenbai800m_newvo | `SystemManager` + `DebugViewer` live overlay, **no anchor** (pure VO+fusion). Always writes the deterministic pose dump `test_vo_viewer_dump.csv`; headless the run auto-starts and is capped at 200 frames, with a `DISPLAY` it waits for Start and is **uncapped** (`eval::viewer_run_policy()`). ⚠ it **hangs when `DISPLAY` is set** — the window stays open until closed; ctest pins `DISPLAY=`/`WAYLAND_DISPLAY=`. Only with `ENABLE_VIEWER=ON` |
| `test_vo_vpr_anchor_viewer` | yenbai800m_newvo | the **whole flow**: `SystemManager` + `VideoDataSource` + `DebugViewer` + an absolute-position producer (M1 `anchor::FakeAnchor`; real VPR is Phase 2 and swaps the producer only). Gates that the lifecycle `anchor[REQ]` → `anchor[EMIT]` → `anchor[APPLY]` runs end to end and reports `FakeAnchorStats` + `AnchorFixCounters` + `FusionFixStats`; `applied == 0` is logged as a finding, not hidden. ⚠ the fixes are manufactured **from groundtruth** — the viewer shows a permanent red banner and the driver repeats it in the log. Dump: `test_vo_vpr_anchor_viewer_dump.csv`. Only with `ENABLE_VIEWER=ON` |
| `test_driver_parity` | munfrl_dataset3 | **cross-driver gate**: `test_full_flight` and `test_vo_viewer` must produce the same trajectory (per-frame difference constant to ≤ 1e-6 m — the offset itself may differ). **ON-only since 2026-08-08** (it reads the viewer's dump, which does not exist at OFF). At ON the viewer dump is display-anchored (`g₀ + (f − f₀)`) and the constant is that anchor: dx/dy mean 0.000000 m, dz mean −0.023800 m, worst spread 9.000e−14 m **[đo 2026-08-08, 159 common frames]** |
| `test_viewer_align` | none (pure unit) | `debug_viewer::TrajectoryAligner` 4-DoF fit. Only with `ENABLE_VIEWER=ON` |
| `test_rate_estimator` | none (pure unit) | `core::RateEstimator`, the trailing-window rate behind `SystemStats::fps_windowed`: steady 10 Hz reads ~10 fps and reproduces the pipeline time fed in; **a stall decays to EXACTLY 0** (the property `fps_processed` cannot provide); the warm-up denominator is `min(window, elapsed)`, not the full window; a burst is capped at `count / window_sec`; one frame with zero elapsed time does not divide by zero. Synthetic time points only — no sleeps, no dataset. Registered in **both** configurations |
| `test_viewer_run_policy` | none (pure unit) | `eval::viewer_run_policy()`: `DISPLAY`/`WAYLAND_DISPLAY` set in-process, 5 cases (neither / X11 / Wayland / both / **both set but empty**); asserts the `autostart` + `max_frames` PAIR — headless ⇒ (true, 200), a display ⇒ (false, 0). Registered in **both** configurations (the predicate lives in `uavloc_test_eval`) |
| `test_build_hygiene` | none (source tree + `libuavloc.so`) | the invariant that replaced R4 — see "CMake Hierarchy". K1/K2/K2b in both configurations, K3/K4 at OFF, K5 at ON. Plain shell + binutils, headless |
| `test_ts_reader` | HoaLac `.ts` (dataset path, soft-skips) | MISB 0601 KLV pass + `VideoReader` open; `UAVLOC_TS_MAXFRAMES=100`, `UAVLOC_TS_SHOW=0` |

**Built but deliberately NOT registered** (need a dataset and run long — invoke manually):
`test_full_flight` (offline evaluation driver → 37-column CSV, the ATE/p95/slope numbers in the
reports; default config `uavloc_yenbai500m.yaml`), `ts_klv_to_csv`, `demo_debug_viewer`.

> **`test_fusion_offline` retired 2026-08-07** → `tests/retired_tests.zip`. It was a second
> offline evaluation driver that did **not** link `uavloc_test_eval` and emitted its own CSV
> schema (`vo_x,vo_y,vo_z,fx,fy,fz…`) instead of the 37-column one, so its numbers could never
> be compared with `test_full_flight`'s — exactly the trap the shared-ruler note above warns
> about. `test_full_flight` supersedes it: same pipeline, driven through `core::SystemManager`,
> scored with the shared ruler. **There is now exactly one offline evaluation driver.**

### Measurement discipline — four traps that have already cost real time

1. **`use_fixed_seed: true` must be in the mission YAML.** The code default is `false`
   (`src/new_vo/module/vo_config.cpp:29`), which seeds the initializer's H/F RANSAC from
   `std::random_device` — **two runs of the same config then diverge at the frame where VO
   initialises**. `config/uavloc_yenbai800m.yaml` was missing it, which silently invalidated two
   whole measurement matrices before anyone noticed. (That file was deleted on 2026-08-03; its
   successor is `config/uavloc_yenbai800m_newvo.yaml`.) All 5 configs on disk set it as of
   2026-08-06 **[đo]** — `grep -n use_fixed_seed config/*.yaml`; verify before trusting any comparison.
2. **Frame cap is the env var `UAVLOC_VO_MAXFRAMES`, not a positional argument.** Passing it as
   `argv[3]` is silently ignored and you get a full-flight run.
3. **`pgrep test_full_flight` never matches** — Linux truncates `comm` to 15 chars, so the pattern
   must be `test_full_fligh`. Checking with the full name reports "nothing running" while six
   processes are.
4. **Give every run its own output path.** A run killed by a timeout can leave the child alive,
   still writing — two writers on one CSV produce a file that looks fine and compares wrong.

> **Removed 2026-08-06.** `test_adjacent_matching`, `test_vo_pipeline` and `test_vo_debug`
> drove the **legacy `vo` module**, whose headers went to `include/uavloc/vo.zip` on
> 2026-07-13. Their CMake guard (`if(EXISTS include/uavloc/vo/vo_module.h)`) had been
> false ever since, so the three sources sat in `tests/` for three weeks without ever
> being compiled while this file still documented how to run them. Sources archived to
> `tests/retired_tests.zip`. Their `new_vo` successors are `test_vo_frame_loader` and
> `test_vo_pipeline_new`.

> **Removed 2026-08-07.** `test_vo_debug_new` and `test_iridescence`, also → `tests/retired_tests.zip`.
> `test_vo_debug_new` drove the **same synchronous `process_frame` path** as `test_vo_pipeline_new`
> over the same API: three of its five gates were verbatim duplicates, it lacked
> `pipeline_new`'s `max_landmarks > 0`, and its two unique gates covered the
> `add_data_out_callback` channel that `test_system_manager` / `test_full_flight` / `test_vo_viewer`
> already exercise — the remaining 425 lines drew a trajectory PNG `test_vo_pipeline_new` also
> produces. `test_iridescence` tested no uavloc code at all (a third-party smoke test, never
> registered); when Iridescence breaks, `test_vo_viewer` and `demo_debug_viewer` fail to build,
> which surfaces it anyway.
>
> **`test_vo_pipeline_new` is now the only registered ctest covering `new_vo` end-to-end**
> (`test_full_flight` is deliberately unregistered) — do not retire it without a replacement.

### `demo_debug_viewer`
The barcode-driven groundtruth trajectory viewer. Scans a directory for the
`.csv` + video, streams the video, decodes each frame's CODE-128 barcode →
`imageId` → CSV telemetry, and grows the live 3D ENU trajectory with a video
sub-window. Exits cleanly headless (no `DISPLAY`).

```bash
# From build/
./tests/demo_debug_viewer [directory | video_path]   # default: data/YenBai800m
```

---

## CMake Hierarchy

```
src/<module>/CMakeLists.txt   →  target_sources(uavloc PRIVATE ...)
src/CMakeLists.txt            →  add_library(uavloc SHARED) + add_subdirectory per module
                                 (debug_viewer included, gated on ENABLE_VIEWER + Iridescence)
CMakeLists.txt (root)         →  find_package deps + option(ENABLE_VIEWER)
                                 + add_subdirectory(src) + add_subdirectory(tests)
apps/CMakeLists.txt           →  independent: find_package(uavloc REQUIRED)
```

Activate a new module by uncommenting its `add_subdirectory` line in `src/CMakeLists.txt`.
Currently enabled: `sensor`, `util`, `new_vo`, `fusion`, `core`, `anchor`, and
`debug_viewer` (conditional). Still commented out: `vo` (legacy, zipped), `vpr`, `utils`.
`find_package(GTSAM)` is already in root (`fusion` links `gtsam` PRIVATE);
`find_package(onnxruntime)` is still needed before enabling `vpr`.

### R4 (hard constraint) — revised 2026-08-08, deliberately

> **Superseded rule (kept for the record):** *"`libuavloc` must never link
> `uavloc_debug_viewer`; the bridge lives in the driver `tests/test_vo_viewer.cpp`, which links
> both. Verify with `nm -DC build/lib/libuavloc.so | grep -c debug_viewer` → must be `0`."*
> That rule described the old layout, where the viewer was the independent static library
> `uavloc_debug_viewer.a`. It no longer holds and must not be re-applied.

**Why it changed.** The separate library forced the same public types to exist twice (the viewer
carried its own `callback_slot.h`, its own telemetry record, its own ENU maths) and made every
consumer link two products with two RPATHs. The intent behind R4 was never "two libraries" — it
was **"no GL in the flight binary"**. That intent is now enforced by a build switch instead of by
a link boundary.

**The rule now.**
- `debug_viewer` is a **module of `libuavloc`**, compiled in via
  `option(ENABLE_VIEWER "Build the debug viewer into libuavloc" ON)`. ON is the *development*
  default; the module is additionally gated on Iridescence being found.
- **Deployment builds MUST configure `-DENABLE_VIEWER=OFF`** — `apps/` and anything running on the
  UAV. There is no exception: an ON build pulls `libiridescence`, GL/GLFW and `libzbar` into
  `libuavloc.so`.
- **The replacement invariant (this is the load-bearing part):** the flag `ENABLE_VIEWER` and the
  macro `UAVLOC_WITH_DEBUG_VIEWER` **must not appear in any file under `src/` or
  `include/uavloc/` other than `src/debug_viewer/` and `include/uavloc/debug_viewer/`** — with the
  single build-system exception of the module gate in `src/CMakeLists.txt`, which the CMake
  hierarchy rule forces to live there. No `libuavloc` translation unit may therefore compile
  differently under the flag, so **the ABI of `libuavloc` does not diverge between the two
  configurations**. Only `tests/` may read the macro — and since 2026-08-08 **no test does**: the
  viewer drivers are gated in `tests/CMakeLists.txt` instead, so their sources contain no `#ifdef`
  at all.
- **Verify with** the ctest target `test_build_hygiene` (`tests/build_hygiene.sh`), which runs in
  both configurations: K1 no viewer header included outside the module · K2/K2b the switch is
  invisible to library sources and defined only by the module · K3 (OFF) `nm -DC libuavloc.so |
  grep -c "debug_viewer\|DebugViewer"` = 0 · K4 (OFF) no `libGL\.|libGLX|libEGL|glfw|iridescence|
  imgui|zbar` in `objdump -p … NEEDED` · K5 (ON) the symbol count is **non-zero**, proving the flag
  is not a no-op.
  ⚠ K4's pattern is anchored on purpose: a case-insensitive `libGL` also matches `libglib-2.0` and
  has produced a false failure once already.

---

## Dependencies

| Library      | Linked on   | Purpose |
|--------------|-------------|---------|
| OpenCV 4.x   | `uavloc` PUBLIC | Image I/O, VideoCapture, geometry |
| Eigen 3.x    | `uavloc` PUBLIC | Matrix math, camera K, SE3 |
| spdlog       | `uavloc` PUBLIC | Logging |
| yaml-cpp     | `uavloc` PUBLIC | Config parsing (`yaml-cpp::yaml-cpp`) |
| Iridescence  | `uavloc` PRIVATE (via `src/debug_viewer/`, only when `ENABLE_VIEWER=ON`) | OpenGL/ImGui 3D viewer (optional; module skipped if absent or if the flag is OFF) |
| zbar         | `uavloc` PRIVATE (via `src/debug_viewer/`, only when `ENABLE_VIEWER=ON`) | Per-frame barcode decoding (`PkgConfig::ZBAR`) |
| GTSAM 4.x    | `uavloc` PRIVATE (via `src/fusion/`) | Factor graph optimization — `IncrementalFixedLagSmoother` |
| g2o          | `uavloc` (via `new_vo/optimize`) | VO optimisation. **Prebuilt in-tree** at `.thirdparty/g2o/build`; only `core` + `stuff` are imported (`g2o::core`, `g2o::stuff`) — no `find_package` |
| ONNXRuntime  | future `vpr`    | Neural descriptor inference |
| stella_vslam | reference only  | Vendored, read-only |

---

## Building

```bash
# Development build — viewer compiled in (ENABLE_VIEWER defaults to ON)
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run video reader test
./tests/test_video_reader

# DEPLOYMENT build — mandatory for apps/ and anything running on the UAV.
# Keeps GL/GLFW/zbar out of libuavloc.so; see the revised R4.
cmake -S . -B build_off -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=OFF
cmake --build build_off -j$(nproc)

# Install, then build apps
cmake --install build_off --prefix /opt/uavloc
cd apps && mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/opt/uavloc && make -j$(nproc)
```

`compile_commands.json` is generated automatically in `build/` — point clangd at it to fix IDE include errors.
