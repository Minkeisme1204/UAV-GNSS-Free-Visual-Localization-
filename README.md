# UAV-GNSS-Free-Visual-Localization

Bachelor thesis project: **UAV localization without GNSS, from nadir imagery.**

A UAV flying with a calibrated downward-looking camera and an attitude/altitude telemetry
stream has to answer one question when GNSS is unavailable: *where am I, in geographic
coordinates?* This repository is the algorithm side of that answer. It combines

* **Visual Odometry (VO)** — a metric-scaled monocular front-end that tracks the motion
  between frames, and
* **Visual Place Recognition (VPR)** — an absolute position measurement obtained by
  matching the live frame against a georeferenced basemap,

inside a **GTSAM factor graph** (`IncrementalFixedLagSmoother`) that produces one fused
latitude / longitude / AGL estimate per frame, with an uncertainty attached.

Everything compiles into a single shared library **`libuavloc.so`**, namespace `uavloc`,
C++17.

---

## Project status

Read this section before trusting any output of this repository.

| Part | State |
|---|---|
| `sensor` — video + telemetry input, camera model, georeferencing | implemented |
| `new_vo` — pure VO front-end (a port of `stella_vslam`, stripped of marker / BoW / loop closing / relocalization) | implemented, measured |
| `fusion` — GTSAM fixed-lag smoother back-end | implemented, measured |
| `core::SystemManager` — the whole pipeline behind one object | implemented |
| `anchor` — the request/response seam where absolute position enters | implemented |
| `debug_viewer` — live 3D view, built only with `-DENABLE_VIEWER=ON` | implemented |
| **`vpr` — real place recognition** | **NOT BUILT — Phase 2 target** |

> ### ⚠ The absolute position measurement is currently FAKE
>
> Because VPR does not exist yet, the absolute position producer in this repository is
> `anchor::FakeAnchor`. It does **not** recognise a place: it **reads the groundtruth log
> and adds Gaussian noise**. Any error figure produced by a run that has the anchor
> attached is therefore **contaminated** — it measures the back-end's ability to *consume*
> absolute measurements, and says **nothing** about the accuracy of a real system. This
> caveat must travel with every number taken from such a run
> (`.claude/rules/reporting.md`).
>
> When the real VPR producer exists, only the **producer** changes: it is handed to
> `SystemManager::setAnchor()` through the same `anchor::AnchorInterface`, and neither the
> fusion back-end nor `SystemManager` needs a line of change.

For measured results (accuracy, timing, profiling), see the reports under `.docs/reports/`
rather than this file — no numbers are duplicated here.

---

## Architecture at a glance

```
              ┌──────────────────────── libuavloc.so ────────────────────────┐
              │                                                              │
 video +      │   sensor::VideoDataSource                                    │
 telemetry ──►│      │ image / attitude / gimbal / gnss channels             │
 (or live     │      ▼                                                       │
  camera)     │   core::SystemManager ─── core::Extrapolator (time sync)     │
              │      │                                                       │
              │      │  frame                                                │
              │      ▼                                                       │
              │   new_vo::VOModule ──── relative pose, keyframes, landmarks  │
              │      │                                                       │
              │      ▼                                                       │
              │   fusion::FusionModule  (GTSAM IncrementalFixedLagSmoother)  │
              │      ▲                                                       │
              │      │ absolute XY fix                                       │
              │   anchor::AnchorInterface                                    │
              │      └── anchor::FakeAnchor   (M1, groundtruth-driven)       │
              │      └── vpr::…               (Phase 2, not built)           │
              │      │                                                       │
              │      ▼                                                       │
              │   SystemCallbacks ──► core::LocalizationOutput               │
              │                        (lat / lon / AGL + 1σ radius)         │
              │                                                              │
              │   debug_viewer::DebugViewer  — subscribes to the same        │
              │      callbacks; compiled in only with ENABLE_VIEWER=ON       │
              └──────────────────────────────────────────────────────────────┘
```

`SystemManager` deliberately does **not** own the sensor: sensors are independent producers
that push into it. Output is delivered through per-instance `util::CallbackSlot`
subscriptions (`on_localization`, `on_vo_data`, `on_fusion_result`, `on_lag_window`,
`on_frame_processed`, `on_stats`) — there is no global bus.

---

## Dependencies

| Library | Required | Notes |
|---|---|---|
| Eigen 3 | yes | `find_package(Eigen3 REQUIRED NO_MODULE)` |
| OpenCV 4.x | yes | image I/O, `VideoCapture`, geometry |
| spdlog | yes | all runtime logging |
| yaml-cpp | yes | mission configuration |
| Threads | yes | VO tracking / local-mapping threads |
| GTSAM 4.x | yes | the fusion back-end; linked `PRIVATE`, so the public API stays GTSAM-free |
| g2o | yes | **prebuilt in-tree** at `.thirdparty/g2o/build`; only `g2o::core` and `g2o::stuff` are imported as `IMPORTED` targets — there is **no** `find_package(g2o)` |
| Iridescence + GLFW | optional | `find_package(... QUIET)` against `.local/`; without it the viewer module and its drivers are not built |
| zbar | optional | `pkg_check_modules`; viewer only, decodes the per-frame groundtruth barcode |
| onnxruntime | not yet | commented out in the root `CMakeLists.txt`, reserved for `vpr` |

## Build

Development build (viewer on):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=ON
cmake --build build -j"$(nproc)"
```

Deployment build — **mandatory for anything that flies**:

```bash
cmake -B build_off -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=OFF
cmake --build build_off -j"$(nproc)"
```

With `ENABLE_VIEWER=OFF`, `libuavloc.so` carries no viewer symbol and pulls in no
GL / GLFW / Iridescence / zbar dependency. That invariant is not a convention — it is
checked by the `test_build_hygiene` test, which also verifies that the flag is not a no-op
and that no viewer header is included from outside `src/debug_viewer/` and
`include/uavloc/debug_viewer/`:

```bash
cd build_off && ctest -R test_build_hygiene --output-on-failure
```

### CMake options

| Option | Default | Meaning |
|---|---|---|
| `BUILD_TESTING` | `ON` | build the test executables under `<build>/tests/` |
| `ENABLE_VIEWER` | `ON` | compile `debug_viewer` into `libuavloc`; set `OFF` for deployment |
| `USE_SSE_ORB` | `OFF` | opt-in SSE3 fast path in the ORB extractor (x86 hosts only) |

`compile_commands.json` is generated in the build directory — point clangd at it.

---

## Configuration

A run is described completely by **one binary + one mission YAML**. The configs used on the
development machine live in `config/` (gitignored, user-maintained):

```
uavloc_yenbai800m_newvo.yaml     # Yen Bai, nadir, ~800 m AGL   (default for both viewer drivers)
uavloc_yenbai500m.yaml           # Yen Bai, ~500 m AGL
uavloc_munfrl_dataset3.yaml      # MUN-FRL Bell 412, dataset 3
uavloc_munfrl.yaml               # MUN-FRL Bell 412, dataset 6
uavloc_hoalac.yaml               # Hoa Lac
```

Each file carries one sub-node per module (`Camera:`, `VideoReader:`, `VO:`, `Fusion:`,
`DebugViewer:`, …) and is loaded through `core::SystemConfig::fromYaml`. A missing
`Camera:` node **fails loud** on purpose — a silent default would let `fx = fy = 0` reach
the pipeline.

The dataset paths inside a config (`VideoReader.video_path` and
`VideoReader.DroneTelemetry.csv_path`) are absolute on the machine that wrote them; adjust
them to your own layout.

The default config path of every test is injected by CMake
(`UAVLOC_MISSION_CONFIG_PATH`); passing `argv[1]` overrides it.

---

## Data

`data/` is gitignored — no imagery or flight log ships with the code. Tests that need a
dataset **soft-skip (return 0)** when the video cannot be opened, so a missing dataset is
not a test failure.

Expected layout: one directory per flight, holding the video and its telemetry CSV, e.g.

```
data/
├── YenBai800m/                 cut_2025-07-30_800m-1x.mkv + .csv
├── YenBai500m/
├── munfrl_bell412_dataset3/
├── munfrl_bell412_dataset6/
└── HoaLac/
```

The Yen Bai flight logs are 57-column drone logs; the column-to-field mapping is declared
in the config's `VideoReader.DroneTelemetry.columns` list, so no column index is hard-coded
in the source.

---

## Running the two main drivers

Both are built only with `-DENABLE_VIEWER=ON` (and Iridescence found). Both take **no
environment variable**: everything they measure is a named constant in the source
(`tests/driver_common.h` and the driver itself), so a run is fully described by
"this binary + this config". Only `SPDLOG_LEVEL` is honoured, and it changes nothing that
is measured.

### Run policy — headless vs. with a display

The same rule applies to both drivers, and you will meet it immediately:

| Environment | Behaviour |
|---|---|
| **no `DISPLAY` / `WAYLAND_DISPLAY`** | **regression run**: starts by itself, capped at `eval::PARITY_MAX_FRAMES = 200` frames, writes its pose dump, exits |
| **a display is present** | **interactive session**: opens the window and **waits for the Start button**; **no frame cap** — it runs to the end of the stream |

The Start/Stop button is a **pause switch**: it pauses and resumes the input source; the
video is never rewound. When the stream ends, the button becomes `Finished` and is
disabled — that state is terminal, the pipeline has been shut down for good.

> ⚠ `DebugViewer::run()` owns the main thread and blocks until the window is closed. Do not
> run these two drivers under `ctest` with a display attached — their ctest entries pin
> `DISPLAY=;WAYLAND_DISPLAY=` for exactly that reason.

### 1. `test_vo_viewer` — VO + fusion + viewer, **no absolute position**

This is the reference assembly of the system, and the cleanest thing to read first: load one
mission YAML, build `SystemManager`, attach a `VideoDataSource`, decide the run policy, hand
the manager to `DebugViewer::attach()` / `run()`. No `anchor::AnchorInterface` is attached,
so the fused trajectory is **pure dead reckoning**.

```bash
cd build
./tests/test_vo_viewer                                   # default mission config
./tests/test_vo_viewer ../config/uavloc_munfrl_dataset3.yaml
```

Output: `test_vo_viewer_dump.csv` in the working directory, one row per fused pose,
`frame_id,pred_x,pred_y,pred_z` — the display-anchored ENU position the viewer draws.
`test_driver_parity` joins that file with `test_full_flight`'s CSV and asserts the two
sequences differ only by a constant offset (the display anchor).

### 2. `test_vo_vpr_anchor_viewer` — the **whole system flow**

Same assembly, same run policy, same dump format, plus the one piece that closes the loop:
an absolute-position producer attached through `anchor::AnchorInterface`. Until VPR exists
that producer is `anchor::FakeAnchor`.

```bash
cd build
./tests/test_vo_vpr_anchor_viewer
./tests/test_vo_vpr_anchor_viewer ../config/uavloc_yenbai500m.yaml
```

> ### ⚠ CONTAMINATED MEASUREMENT — read before reporting anything from this run
>
> `anchor::FakeAnchor` **reads the groundtruth CSV and adds noise**. It does not look at the
> image. Every error number this driver can produce measures the **back-end consuming
> absolute positions**, not the accuracy of a real system. The viewer shows a permanent red
> banner while fake fixes are being injected, so a screenshot cannot leave without the
> caveat, and the driver repeats the warning in the log before and after the run.

The fake producer is configured by named constants in
`tests/test_vo_vpr_anchor_viewer.cpp`, each equal to the corresponding
`anchor::FakeAnchorConfig` default — **none of them was tuned**:

| Constant | Value | Meaning |
|---|---|---|
| `FIX_SIGMA_M` | `20.0` | per-axis noise σ [m], also what the emitted fix declares |
| `FIX_EVERY_KF` | `20` | duty cycle: at most one fix every N keyframe requests |
| `FIX_REINIT_MIN_KF` | `0` | throttle on the VO-re-init bypass; 0 = every re-init may skip the cadence |
| `FIX_OUTLIER_RATE` | `0.0` | probability of a gross outlier (outlier rejection is tested in `test_anchor`) |
| `FIX_OUTLIER_MIN_M` / `FIX_OUTLIER_MAX_M` | `200.0` / `1000.0` | magnitude band of that corruption [m] |
| `FIX_LATENCY_KF` | `0` | requests a fix is held back before being emitted |
| `FIX_SEED` | `42` | RNG seed — the same binary on the same data manufactures the same fixes |
| producer mode | `SYNCHRONOUS` | an async producer would make the instant a fix enters the graph depend on thread scheduling |

The groundtruth CSV and its frame-id offset are **not** constants: they are mission data,
taken from the config's `VideoReader` section.

Output:

* `test_vo_vpr_anchor_viewer_dump.csv` — same format as above, **a separate file on
  purpose**: this run injects absolute fixes, so it is a different experiment and must
  never overwrite the parity gate's input;
* a final report of `anchor::FakeAnchorStats` (producer side),
  `debug_viewer::AnchorFixCounters` (what was drawn and how it was judged) and
  `fusion::FusionFixStats` (back-end verdicts);
* the per-fix lifecycle logged by the library as `anchor[REQ]` → `anchor[EMIT]` →
  `anchor[APPLY]`, correlated by timestamp. **Every reject path prints its reason**
  (`GATED / AGE_EXPIRED / UNMATCHED / MARGINALIZED / LOW_CONFIDENCE`).

If `applied = 0`, the driver says so explicitly: the run is still a valid smoke test, but it
measured nothing about the effect of absolute positioning and must be reported as such.

---

## Running the full test suite

Always run headless — two of the tests open a window and block until it is closed:

```bash
cd build && env -u DISPLAY -u WAYLAND_DISPLAY ctest --output-on-failure
```

`ctest -N` lists **19 tests** in the `ENABLE_VIEWER=ON` configuration and **15** in the
`OFF` one (measured 2026-08-08). The four that disappear at `OFF` are the viewer-gated
ones: `test_viewer_align`, `test_vo_viewer`, `test_vo_vpr_anchor_viewer` and the
cross-driver gate `test_driver_parity`.

Tests that need a dataset soft-skip when it is absent, so a clone without `data/` still
passes.

---

## Documentation

| Where | What |
|---|---|
| `CLAUDE.md` | the working map of the repository: module status, conventions, measurement traps |
| `.claude/rules/` | the project's hard rules — `coding.md`, `cmake.md`, `docs.md`, `constraints.md`, and `reporting.md` (data integrity: no number may be published that does not come from a real, reproducible measurement) |
| `.docs/README.md` | manually maintained index of every design document, report and theory note |
| `.docs/designs/` | how the system is *supposed* to work (e.g. `system_manager_design.md`, `fusion_factor_graph.md`, `viewer_module_design.md`) |
| `.docs/reports/` | what was measured on a given date (e.g. `Phase1_final_release.md`, `m1_acceptance.md`, `profiling_baseline.md`) |

`.docs/` is gitignored and is therefore not part of a clone.

---

## License

Apache License 2.0 — see [LICENSE](LICENSE).

This repository is the software artefact of a bachelor thesis (HUST). The datasets it is
evaluated on are not redistributed here.
