# CLAUDE.md — UAV Localization (uavloc)

## Project Summary

Bachelor thesis: GPS-denied UAV localization using nadir imagery.
Fuses **Visual Odometry (VO)** and **Visual Place Recognition (VPR)** via a GTSAM factor graph.
All code compiles into **`libuavloc.so`** under the `uavloc` C++ namespace.

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
│   └── bug_rp/                 # bug root-cause and fix-schedule notes
├── .claude/rules/              # coding, cmake, docs, constraints, agent_permissions
└── .thirdparty/stella_vslam/   # read-only vendored SLAM reference
```

---

## Modules

| Module         | Status | Key files |
|----------------|--------|-----------|
| `sensor`       | Active | `data_interface.h`, `frame_data.h`, `telemetry_data.h`, `video_reader.h`, `camera_streamer.h`, `camera_model.h`, `sensor_factory.h` |
| `vo`           | Active (refactor in progress) | `vo_data.h`, `feature_detector.h`, `projection_matcher.h`, `pose_estimator.h`, `frame_sequence.h` (placeholder: `local_optimizer.h`) |
| `debug_viewer` | Active | `debug_viewer.h`, `debug_viewer_callbacks.h`, `callback_slot.h`, `telemetry_record.h`, `telemetry_csv_reader.h`, `trajectory_viewer.h` |
| `vpr`          | Stub   | — |
| `fusion`       | Stub   | — (needs GTSAM) |
| `anchor`       | Stub   | — |
| `core`         | Stub   | — |
| `utils`        | Stub   | `transforms.h` (work-in-progress draft; not yet compilable / not wired into the build) |

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

## VO Module (refactor in progress)

> **Current state (on disk).** The module is being rebuilt around small,
> independently-testable stages. The pipeline/Public-API described further below
> is the **design target** (see `.docs/designs/vo_design.md`); several of those
> classes (`VOModule`, `Initializer`, `PoseOptimizer`, `LocalMap`, …) are not yet
> present. What exists today:
>
> - **`vo_data.h`** — shared types: `FeatureSet` (`keypoints`, `descriptors`,
>   `DescriptorType`), `MatchesData` (`matches`, `inliers`, `inlier_mask`,
>   `num_matches`, `num_inliers`, `inlier_ratio`), `VOPoseData`
>   (`R_L_C`, `t_L_C`, `T_prev_curr`), and the status enums `FeatureStatus`,
>   `MatchStatus`, `VOTrackingState` (`NOT_INITIALIZED`, `INITIALIZED`,
>   `TRACKING`, `LOST`, `FAILED`), and `VOFailureReason` (`NONE`,
>   `NOT_ENOUGH_FEATURES`, `NOT_ENOUGH_MATCHES`, `NOT_ENOUGH_INLIERS`,
>   `POSE_ESTIMATION_FAILED`). (`PoseStatus` lives in `pose_estimator.h`,
>   not `vo_data.h`.)
> - **`feature_detector.h`** — `IFeatureDetector` + `createFeatureDetector()`;
>   ORB (`ORB_OPENCV`) and `FAST_ONLY` backends; `FeatureDetectorConfig::fromYaml`.
> - **`projection_matcher.h`** — `ProjectionMatcher` (Hamming BF + Lowe ratio +
>   orientation/optional GMS filters); fills `matches`, leaves inliers for the
>   pose stage; `ProjectionMatcherConfig::fromYaml`.
> - **`pose_estimator.h`** — `PoseEstimator`: consumes `MatchesData` and recovers
>   the relative pose **homography-first** via `cv::findHomography` (RANSAC) +
>   `cv::decomposeHomographyMat` with a nadir-aware solution selector (ground
>   normal ≈ optical axis). Writes inliers back into `MatchesData`, outputs
>   `VOPoseData`; translation is **up-to-scale** (monocular). Optional
>   essential-matrix fallback for non-planar/high-parallax. `PoseEstimatorConfig::fromYaml`
>   (YAML key `PoseEstimator:` under `VO:`). Implementation (`pose_estimator.cpp`,
>   318 LOC) is complete: homography RANSAC seeded `0xABCD1234`, a nadir-aware
>   `cv::decomposeHomographyMat` solution selector, inlier writeback into
>   `MatchesData`, and an essential-matrix fallback seeded `0x5678EF00`.
> - **`frame_sequence.h`** — frame buffering for two-view processing.
> - Placeholder (empty header): `local_optimizer.h`. (`motion_estimator.h` has
>   been removed.)
>
> Current two-view data flow: `FeatureDetector → ProjectionMatcher → PoseEstimator`.
> NOTE: vo headers currently use `#ifndef` guards except `pose_estimator.h`, which
> follows the `#pragma once` coding rule.

### Public API (design target)
- **`VOModule`** — pimpl class; `start()` / `stop()` control the tracking thread; `pushFrame(FrameData)` enqueues a frame (returns `false` on drop); `currentState()` returns `TrackingState`; three callback setters: `setVOResultCallback()`, `setKeyframeCallback()`, `setStatusCallback()`.
- **`VOResultCallback`** — `std::function<void(const VOResult&)>` fired on every processed frame.
- **`KeyframeCallback`** — `std::function<void(const KeyframePacket&)>` fired when a new keyframe is selected; feeds VPR downstream.
- **`StatusCallback`** — `std::function<void(TrackingState, const std::string&)>` fired on state transitions.

### Key data types
- **`TrackingState`** — `enum class`: `NOT_INITIALIZED | INITIALIZING | TRACKING | LOST` (the enum actually implemented on disk is `VOTrackingState` in `vo_data.h`, with members `NOT_INITIALIZED | INITIALIZED | TRACKING | LOST | FAILED` — do not confuse the two)
- **`VOResult`** — per-frame result: `frame_id`, `timestamp_msec`, `state`, `T_wc` (Eigen::Matrix4d), `T_prev_curr`, `has_pose`, `is_keyframe`, `num_keypoints`, `num_matches`, `num_inliers`, `inlier_ratio`, `mean_reproj_error`, `tracking_confidence`, `translation_from_last_kf`, `rotation_from_last_kf`
- **`VOConfig`** — all tunable parameters loaded from YAML key `VO:` via `VOConfig::fromYaml(node)`; includes feature, RANSAC, keyframe, queue, and matching fields
- **`KeyframePacket`** — carries keyframe image and pose to VPR

### Pipeline overview (design target)
```
FrameQueue → Preprocessor (undistort) → FeatureExtractor (ORB)
  ├─ [NOT_INITIALIZED] Initializer: parallel H/F RANSAC → triangulation → seed map
  └─ [TRACKING] ProjectionMatcher → PnP (solvePnPRansac) → PoseOptimizer
                → KeyframeManager (insertion policy)
                → LocalMap (cullLandmarks on keyframe)
```

### Internal components (design target — `src/vo/`)
- **`VOModuleImpl`** — tracking thread, state machine, full pipeline orchestration
- **`FrameQueue`** — bounded MPSC queue; drop-oldest policy; configurable capacity
- **`Initializer`** — parallel H/F RANSAC on two threads; nadir-aware H decomposition (surface-normal tiebreaker); triangulates seed map
- **`MotionModel`** — constant-velocity SE3 twist; `predict(T_wc_last, frame_gap)` scales twist by gap
- **`PoseOptimizer`** — `cv::solvePnPRansac` placeholder (TODO: replace with g2o BA)
- **`ProjectionMatcher`** — landmark projection + Hamming match + Lowe ratio test + `kp_assigned[]` deduplication guard
- **`LocalMap`** — landmark/keyframe store; `cullLandmarks()` marks bad if `obs_count < 2` or `num_found/num_visible < 0.25`
- **`KeyframeManager`** — insertion policy: frame gap, inlier ratio, and tracked-landmark count thresholds
- **`Preprocessor`** — optional per-frame undistortion using `CameraModel`
- **`FeatureSet`** (header-only) — internal struct: `keypoints`, `descriptors`, `normalized_pts`
- **`Landmark`** (header-only) — internal struct: `id`, `pos_w`, `descriptor`, `obs_count`, `num_visible`, `num_found`, `bad`

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
- `PoseOptimizer` calls `cv::solvePnPRansac` directly; a proper g2o bundle-adjustment back-end is not yet integrated.
- Monocular scale is inherently ambiguous; metric scale recovery requires altitude from telemetry or stereo.

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
Add `find_package(GTSAM)` / `find_package(onnxruntime)` in root before enabling `fusion` / `vpr`.

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
| GTSAM 4.x    | future `fusion` | Factor graph optimization |
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
