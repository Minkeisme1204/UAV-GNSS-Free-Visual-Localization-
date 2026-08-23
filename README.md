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
| `anchor` — the request/response seam where absolute position enters | implemented; **two** producers behind it — `anchor::FakeAnchor` (M1, groundtruth-driven) and `anchor::VprAnchor` (real, image-driven) |
| `debug_viewer` — live 3D view, built only with `-DENABLE_VIEWER=ON` | implemented |
| `vpr` — real place recognition | implemented as a thin adapter (`src/vpr/`) over the external `satvpr_core` library; **built conditionally**, only when `SATVPR_ROOT` points at a valid satVPR checkout; **not wired into `core::SystemManager`** |

> ### ⚠ A real VPR producer exists — but the default absolute measurement is still FAKE
>
> `vpr::VprModule` and `anchor::VprAnchor` are **real and they run**: image → DINO ViT-S/8
> (ONNX) → VLAD → PCA whitening → retrieval inside a `.vprdb`, out comes an
> `anchor::AbsoluteFix`. That is not a placeholder.
>
> What is **not** done is the wiring: `core::SystemManager` holds no reference to
> `VprAnchor` at all (`grep -r "VprAnchor\|VprModule" src/core include/uavloc/core` → 0
> hits). The real producer is therefore attached **by a driver, explicitly**, and no driver
> reaches for it by default: `test_vo_viewer` attaches no anchor at all,
> `test_vo_vpr_anchor_viewer` runs `--anchor fake` unless told `--anchor vpr` (its ctest
> entry passes `fake`), and `test_full_flight` needs `UAVLOC_VPR_FIX=1`.
>
> So the old caveat still holds, unchanged, for every number taken from a default run:
> `anchor::FakeAnchor` does **not** recognise a place, it **reads the groundtruth log and
> adds Gaussian noise**. Such a figure is **contaminated** — it measures the back-end's
> ability to *consume* absolute measurements and says **nothing** about the accuracy of a
> real system. The caveat must travel with the number (`.claude/rules/reporting.md`).
>
> One obstacle is known and worth stating before you try to swap the two: `VprAnchor` is
> **asynchronous-only** (DINO inference cannot run inline on the pipeline thread; there is
> no equivalent of `FakeAnchorMode::SYNCHRONOUS`), while `tests/driver_common.cpp` forces
> the fake producer to `SYNCHRONOUS` precisely to keep a run deterministic. Attaching
> `VprAnchor` to those drivers therefore breaks the bit-identical regression gate.
>
> The design point still stands: only the **producer** changes. It is handed to
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
              │      ├── anchor::FakeAnchor   (M1, groundtruth-driven —      │
              │      │                       what every driver uses today)   │
              │      └── anchor::VprAnchor ← vpr::VprModule ← satvpr_core    │
              │                            (real; not wired into             │
              │                             SystemManager yet)               │
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
| onnxruntime | conditional | `find_package(onnxruntime REQUIRED)` at `CMakeLists.txt:44` — but that line only runs when `EXISTS "${SATVPR_ROOT}/CMakeLists.txt"`, so it is a **conditional** dependency of the `vpr` module, not a missing one |
| `satvpr_core` | conditional | the VPR library itself (DINOv2 + VLAD + PCA + tile index). **Not a submodule**: it is pulled in with `add_subdirectory(${SATVPR_ROOT})`. `SATVPR_ROOT` defaults to `.thirdparty/satvpr`, **which is not present in this repository**, so by default `vpr` is not built and CMake only emits a `message(WARNING)` |

## Build

Development build (viewer on):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_VIEWER=ON
cmake --build build -j"$(nproc)"
```

Development build with the VPR module (needs a satVPR checkout):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DSATVPR_ROOT=/path/to/satVPR
cmake --build build -j"$(nproc)"
```

Without `-DSATVPR_ROOT=`, `UAVLOC_HAVE_VPR` is `FALSE`: uavloc still builds normally, but
it has no `vpr` module and the four `test_vpr*` tests are not registered. That is
**deliberate** — the `EXISTS` gate at `CMakeLists.txt:43` is what lets a checkout without
the submodule configure and build at all.

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

The same test carries gate **K6** for the VPR module: `libuavloc.so` must export **zero**
`satvpr::` or `Ort::` symbols. That needs **both** mechanisms, and both are load-bearing —
`--exclude-libs,libsatvpr_core.a` hides what comes from the archive, and
`-fvisibility-inlines-hidden` on `vpr_module.cpp` hides the satVPR inline functions
instantiated into uavloc's own object. The compile option must be set **with
`TARGET_DIRECTORY uavloc`**: source-file properties are directory-scoped, and the `uavloc`
target is defined in `src/`, not in `src/vpr/` — without it the command is silently a
no-op. The first wiring leaked 56 `satvpr::` and 12 `Ort::` symbols.

### CMake options

| Option | Default | Meaning |
|---|---|---|
| `BUILD_TESTING` | `ON` | build the test executables under `<build>/tests/` |
| `ENABLE_VIEWER` | `ON` | compile `debug_viewer` into `libuavloc`; set `OFF` for deployment |
| `USE_SSE_ORB` | `OFF` | opt-in SSE3 fast path in the ORB extractor (x86 hosts only) |
| `SATVPR_ROOT` | `${PROJECT_SOURCE_DIR}/.thirdparty/satvpr` | path to the satVPR checkout; decides whether `vpr` is built. A **CACHE PATH**, not an `option()` |

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

Two sub-nodes are **not** read by `core::SystemConfig`, which only knows `Camera:`,
`VO:` and `Fusion:`. `VPR:` and `Anchor:` are parsed by `vpr::VprConfig::fromYaml` and
`anchor::VprAnchorConfig::fromYaml`, which the **driver** calls itself — the same shape
`tests/driver_common.cpp` uses for `anchor::FakeAnchorConfig`. Both nodes hold absolute
artifact paths belonging to the machine that wrote them, and `uavloc_hoalac.yaml` has
neither node.

> ⚠ Because the `.as<T>(default)` pattern never throws, pointing either `fromYaml` at a
> config whose node is missing — or that still carries the older placeholder keys the two
> schemas replaced — yields a **silently wrong** configuration rather than an error; an
> empty `database_path` is the first symptom you will actually see. The `test_vpr*` tests
> sidestep the question entirely: they do **not** read these files, taking their artifact
> paths from a CMake macro derived from `SATVPR_ROOT`.

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

## Dataset tooling — `scripts/`

Standalone Python utilities that prepare a dataset. None of them imports `libuavloc`, and
none of them writes to its input, so they are safe to point at a read-only source archive.
They need the extra stack `pip install rasterio pillow pyproj numpy` (plus `contextily` and
`requests` for the downloader).

Two of them build the georeferenced basemap that VPR will eventually match against:

| Script | What it does |
|---|---|
| `collect_satellite.py` | **Downloads** a basemap covering a trajectory — an XYZ tile provider via contextily, an ArcGIS MapServer `/export`, or an ImageServer `/exportImage` — and writes it as a GeoTIFF (`_3857`, optionally `_utm`). Extent, zoom and output path can all come from a mission YAML (`SatelliteMap:` + `VideoReader.DroneTelemetry`). |
| `stitch_tiles.py` | **Mosaics an XYZ tile pyramid that is already on disk** (`<z>/<x>/<y>.png`) into one `_3857` + one `_utm` GeoTIFF per zoom level. Georeferencing comes from the tile indices, not from an estimate; the mosaic is written one tile-row at a time, so a 23808 × 15616 map never has to fit in RAM. |

```bash
# Download: extent, zoom and output all taken from the mission YAML
python3 scripts/collect_satellite.py --config config/uavloc_yenbai800m.yaml

# Mosaic tiles already fetched: print the grid/GSD/bbox table, write nothing
python3 scripts/stitch_tiles.py --tiles /path/to/tiles --zooms 19 20 21 \
    --out data/munfrl_bell412_dataset6/map --name bell412_dataset6 --dry-run

# ... then do it for real, with the UTM zone pinned
python3 scripts/stitch_tiles.py --tiles /path/to/tiles --zooms 19 20 21 \
    --out data/munfrl_bell412_dataset6/map --name bell412_dataset6 --utm-epsg 32618
```

`stitch_tiles.py` records where a map came from in the GeoTIFF tags (`TILE_SOURCE_DIR`,
`TILE_ZOOM`, `TILE_COUNT_*`, `GSD_M_PER_PX`, `CREATED_UTC`) and reserves
`SOURCE_PROVENANCE` for the imagery provider, capture date and licence — which a bare tile
tree cannot tell you. Pass `--provenance "..."` once you know it; until then the tag holds
a placeholder saying so, because an unattributed basemap cannot be cited in the thesis.
By default it also builds `[2,4,8,16]` overviews into every file; `--no-overviews` leaves
each `.tif` holding a single full-resolution image (~1/3 smaller, slower to pan in a GIS —
`gdaladdo -r average <file> 2 4 8 16` adds the pyramid back).

Every file it writes is audited for **orphaned blocks** (sum of `TileByteCounts` vs. size on
disk) and a file more than 10 % dead is reported on stderr — the failure that once left a
7.71 GB GeoTIFF holding 3.34 GB of pixels. `--gdal-cachemax-mb N` (default 4096) sets the
GDAL block cache used while writing; it trades RAM for I/O, but correctness no longer
depends on it, because the UTM reprojection warps all bands in a single pass.

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

## The VPR track

`vpr::VprModule` is a thin adapter over `satvpr_core`: image → DINO ViT-S/8 (ONNX) → VLAD →
PCA whitening → retrieval inside a `.vprdb` tile index, optionally followed by a fine stage
(SuperPoint + LightGlue + homography) that turns the winning tile into a metric position.
`anchor::VprAnchor` wraps it behind `anchor::AnchorInterface` and is the **only** place in
the track that knows about ENU — `VprModule` speaks lat/lon and nothing else.

Four tests cover it, registered only when `UAVLOC_HAVE_VPR` is true:

| Test | What it does |
|---|---|
| `test_vpr` | the `vpr` module in isolation: parse the config, `setup()` must **refuse** a missing artifact and an input size that disagrees with the ONNX static shape, then one real query on one real image |
| `test_vpr_anchor` | `VprAnchor` judged against the `AnchorInterface` contract: lifecycle, cadence gate, REINIT bypass, single slot (no queue), and the `AbsoluteFix` invariants — symmetric positive-definite covariance, timestamp preserved, callback delivered on another thread |
| `test_vpr_fine` | the wiring of the fine stage: a v2 `.vprdb` enables it, a v1 database disables it **without failing**, and the returned position stays inside the reranked tile |
| `test_vpr_flow` | **a measurement driver, not a gate**: runs N queries from `query_latlon.csv`, prints the position error against groundtruth, how well the covariance is calibrated, and a five-stage latency breakdown. It returns non-zero only if `setup()` / `start()` fail — it contains **no pass/fail threshold** |

Every one of them locates its artifacts through `SATVPR_ROOT`, injected by CMake as a macro
(`SATVPR_DATA_ROOT`, `DEFAULT_DATASET_DIR`) — never from a mission YAML. Four artifacts are
mandatory: the backbone `.onnx`, the VLAD codebook `.bin` (with its `.json` sidecar beside
it), `pca.yml`, and the `.vprdb`. If any of them is absent the test **soft-skips
(`return 0`)**, per the project's dataset rule.

> ⚠ Known hole: `test_vpr`, `test_vpr_anchor` and `test_vpr_fine` hard-code
> `device = "cuda"` and do **not** soft-skip when there is no GPU / CUDA execution
> provider. On a machine that has all the artifacts but no GPU they **fail** instead of
> skipping. (`test_vpr_flow` is the exception — it takes `--device`.)

> ⚠ Whenever the fine stage is not in play — no keypoint/matcher model configured, a v1
> database, or fewer inliers than `min_inliers` — the returned position is the **rank-1 tile
> centre**, so the error is floored by the tile-grid quantisation. And the covariance and
> confidence the fine stage attaches are explicitly **uncalibrated** (a measured
> approximation, not a noise model), which is why `max_uncalibrated_confidence` caps them.

---

## Offline evaluation and profiling

`test_full_flight` is the **only** offline evaluation driver. It is built but
**deliberately not registered with ctest** — it needs a dataset and runs long. Unlike the
two viewer drivers, it *does* read environment variables:

```bash
cd build
UAVLOC_PROFILE=1 ./tests/test_full_flight ../config/uavloc_yenbai800m_newvo.yaml tests/<run>.csv
```

* `UAVLOC_PROFILE=1` enables the per-stage profiler (off by default). Alongside the
  trajectory CSV it then writes `profile_<stem>.csv` and prints a summary table. The
  profiler only reads clocks; it never changes the path the code takes.
* `UAVLOC_VO_MAXFRAMES` caps the number of frames (`0` / unset = run to the end).
  ⚠ It is an **environment variable, not `argv[3]`** — passed on the command line it is
  silently ignored and you get a full-flight run.
* `UAVLOC_INPUT_MODE=frame|channels` selects which front door of `SystemManager` the run
  uses; the two modes must produce a bit-identical CSV.
* `UAVLOC_FAKE_FIX=1` attaches `anchor::FakeAnchor` (contaminated — see the warning at the
  top), `UAVLOC_VPR_FIX=1` attaches the real `anchor::VprAnchor`. They are mutually
  exclusive and both default to off, in which case no anchor is attached at all.

Always give a run its own output path: a process killed by a timeout can leave a child
alive and still writing, and two writers on one CSV produce a file that looks fine and
compares wrong.

> ⚠ `pgrep test_full_flight` **never matches** — Linux truncates `comm` to 15 characters,
> so the pattern must be `pgrep test_full_fligh`. Checking with the full name reports
> "nothing running" while several processes are.

---

## Running the full test suite

Always run headless — two of the tests open a window and block until it is closed:

```bash
cd build && env -u DISPLAY -u WAYLAND_DISPLAY ctest --output-on-failure
```

`ctest -N` lists **24 tests** with `ENABLE_VIEWER=ON` *and* a valid `SATVPR_ROOT`, and
**16** with `ENABLE_VIEWER=OFF` and no satVPR checkout (both measured 2026-08-23). The
eight-test difference comes from two independent gates: four tests need the viewer
(`test_viewer_align`, `test_vo_viewer`, `test_vo_vpr_anchor_viewer` and the cross-driver
gate `test_driver_parity`) and four need VPR (`test_vpr`, `test_vpr_anchor`,
`test_vpr_fine`, `test_vpr_flow`). The two mixed configurations hold **20 tests each** —
that pair is *derived from the CMake gates, not measured*.

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
| `CURRENT_STATE_VPR.md` | repo root, gitignored — a short hand-over note for the vpr/anchor work stream; the full hand-over document lives in the satVPR repository as `CURRENT_STATE.md` |

`.docs/` is gitignored and is therefore not part of a clone.

---

## License

Apache License 2.0 — see [LICENSE](LICENSE).

This repository is the software artefact of a bachelor thesis (HUST). The datasets it is
evaluated on are not redistributed here.
