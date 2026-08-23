# All test executables land in <build>/tests/
set(TEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/tests)

# ── shared evaluation helpers (S8) ────────────────────────────────────────────
# Scoring maths (percentile / least-squares slope / 4-DoF aligned RMS) and the
# stage-profiler dump, extracted VERBATIM from test_full_flight.cpp so every
# driver scores with the same ruler. Deliberately a tests-only static library:
# these are evaluation helpers, not product API, so they must not become
# installed headers of libuavloc.
#
# driver_common.cpp (S9) is the INPUT-side counterpart: the forced run mode and
# the layered fake-anchor configuration both evaluation drivers must share. Same
# reasoning, same library — a driver that scores with the shared ruler but runs
# the pipeline in its own mode still produces incomparable numbers.
add_library(uavloc_test_eval STATIC eval_common.cpp driver_common.cpp)
target_link_libraries(uavloc_test_eval PUBLIC uavloc)
target_include_directories(uavloc_test_eval PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ── sensor ────────────────────────────────────────────────────────────────────
# Reads a mission config (path from CMake, argv[1] overrides) and soft-skips when
# its gitignored video is absent. UAVLOC_VR_SHOW=0 keeps ctest headless even on a
# machine with a display — without it the GUI path blocks until the window closes.
add_executable(test_video_reader test_video_reader.cpp)
target_link_libraries(test_video_reader PRIVATE uavloc)
target_compile_definitions(test_video_reader PRIVATE
    UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_yenbai800m_newvo.yaml"
)
set_target_properties(test_video_reader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME test_video_reader COMMAND test_video_reader)
set_tests_properties(test_video_reader PROPERTIES
    ENVIRONMENT "UAVLOC_VR_SHOW=0"
    TIMEOUT 120
)

# ── drone-log telemetry parser ────────────────────────────────────────────────
# Takes its DroneTelemetry node straight out of the mission config (path from
# CMake, argv[1] overrides; argv[2] overrides just the csv_path), so no dataset
# path is written in the test source. Soft-skips when the CSV is absent.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_drone_telemetry.cpp)
    add_executable(test_drone_telemetry test_drone_telemetry.cpp)
    target_link_libraries(test_drone_telemetry PRIVATE uavloc)
    target_compile_definitions(test_drone_telemetry PRIVATE
        UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_yenbai800m_newvo.yaml"
    )
    set_target_properties(test_drone_telemetry PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_drone_telemetry COMMAND test_drone_telemetry)
    set_tests_properties(test_drone_telemetry PROPERTIES TIMEOUT 120)
endif()

# ── push-style data source (S6a: DataSourceInterface + VideoDataSource) ─────
# Checks 2-8 use an in-test DataInterface double; checks 1-4 only READ a mission
# config (path from CMake, argv[1] overrides) and soft-skip when its gitignored
# video is absent. Multi-threaded — every internal wait carries its own
# deadline, the ctest TIMEOUT below is only the outer safety net.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_data_source.cpp)
    add_executable(test_data_source test_data_source.cpp)
    target_link_libraries(test_data_source PRIVATE uavloc)
    target_compile_definitions(test_data_source PRIVATE
        UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_munfrl_dataset3.yaml"
    )
    set_target_properties(test_data_source PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_data_source COMMAND test_data_source)
    set_tests_properties(test_data_source PROPERTIES TIMEOUT 180)
endif()

# ── util pub/sub primitives (S5b: CallbackSlot + ThreadsafeQueue<T>) ─────────
# Header-only templates, but the test still links uavloc for spdlog/Threads and
# the public include path. Renamed from test_util_pubsub at S5b, when
# util::Publisher<T> was replaced by util::CallbackSlot<void(Args...)>.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_util_callback.cpp)
    add_executable(test_util_callback test_util_callback.cpp)
    target_link_libraries(test_util_callback PRIVATE uavloc)
    set_target_properties(test_util_callback PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_util_callback COMMAND test_util_callback)
    # Multi-threaded test: every internal wait has its own deadline, this is
    # only the outer safety net.
    set_tests_properties(test_util_callback PROPERTIES TIMEOUT 120)
endif()

# ── core system config (S2: SystemConfig::fromYaml) ──────────────────────────
# In-memory YAML for the default/parsing checks; the delegation check only
# READS an existing mission config, whose path comes from CMake (argv[1]
# overrides) so the test source carries no path.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_system_config.cpp)
    add_executable(test_system_config test_system_config.cpp)
    target_link_libraries(test_system_config PRIVATE uavloc)
    target_compile_definitions(test_system_config PRIVATE
        UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_munfrl.yaml"
    )
    set_target_properties(test_system_config PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_system_config COMMAND test_system_config)
endif()

# ── core system manager (S3 sync, S5 async, S6c four channels) ───────────────
# The synthetic checks build their YAML in memory; the dataset checks (7, 15,
# 19, 20) only READ a mission config (path from CMake, argv[1] overrides) and
# soft-skip when its gitignored video is absent. Multi-threaded — every internal
# wait carries its own deadline, the ctest TIMEOUT below is only the outer
# safety net.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_system_manager.cpp)
    add_executable(test_system_manager test_system_manager.cpp)
    target_link_libraries(test_system_manager PRIVATE uavloc)
    target_compile_definitions(test_system_manager PRIVATE
        UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_munfrl_dataset3.yaml"
    )
    set_target_properties(test_system_manager PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_system_manager COMMAND test_system_manager)
    set_tests_properties(test_system_manager PROPERTIES TIMEOUT 900)
endif()

# ── core extrapolator (S6b: 3-channel time buffer, SLERP + exact-match rule) ─
# Pure unit test: no dataset, no config, nothing under config/ is read. Includes
# a multi-threaded check whose internal wait carries its own deadline; the ctest
# TIMEOUT below is only the outer safety net.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_extrapolator.cpp)
    add_executable(test_extrapolator test_extrapolator.cpp)
    target_link_libraries(test_extrapolator PRIVATE uavloc)
    set_target_properties(test_extrapolator PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_extrapolator COMMAND test_extrapolator)
    set_tests_properties(test_extrapolator PROPERTIES TIMEOUT 120)
endif()

# ── vo ────────────────────────────────────────────────────────────────────────
# Guarded: source is part of an in-progress refactor and may be absent.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo.cpp)
    add_executable(test_vo test_vo.cpp)
    target_link_libraries(test_vo PRIVATE uavloc)
    set_target_properties(test_vo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_vo COMMAND test_vo)
endif()

# ── new_vo frame loader ───────────────────────────────────────────────────────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo_frame_loader.cpp)
    add_executable(test_vo_frame_loader test_vo_frame_loader.cpp)
    target_link_libraries(test_vo_frame_loader PRIVATE uavloc)
    target_compile_definitions(test_vo_frame_loader PRIVATE
        UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_yenbai800m_newvo.yaml"
    )
    set_target_properties(test_vo_frame_loader PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_vo_frame_loader COMMAND test_vo_frame_loader)
endif()

# ── new_vo end-to-end pipeline (VOModule) ─────────────────────────────────────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo_pipeline_new.cpp)
    add_executable(test_vo_pipeline_new test_vo_pipeline_new.cpp)
    target_link_libraries(test_vo_pipeline_new PRIVATE uavloc)
    target_compile_definitions(test_vo_pipeline_new PRIVATE
        UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_yenbai800m_newvo.yaml"
    )
    set_target_properties(test_vo_pipeline_new PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_vo_pipeline_new COMMAND test_vo_pipeline_new)
endif()

# ── viewer alignment maths (debug_viewer::TrajectoryAligner) ────────────────
# Pure synthetic unit test: no dataset, no GL. TrajectoryAligner only exists in
# libuavloc when the viewer module is compiled in, so the test is gated on the
# same condition as the module — with ENABLE_VIEWER=OFF it is not built.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_viewer_align.cpp
   AND ENABLE_VIEWER AND Iridescence_FOUND)
    add_executable(test_viewer_align test_viewer_align.cpp)
    target_link_libraries(test_viewer_align PRIVATE uavloc)
    set_target_properties(test_viewer_align PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_viewer_align COMMAND test_viewer_align)
    set_tests_properties(test_viewer_align PROPERTIES TIMEOUT 60)
endif()

# ── viewer RUN POLICY (eval::viewer_run_policy) ─────────────────────────────
# Pure unit test of the predicate that decides "regression run or interactive
# session?" — it manipulates DISPLAY/WAYLAND_DISPLAY in-process and asserts the
# autostart + frame-cap PAIR. NOT gated on ENABLE_VIEWER: the predicate lives in
# uavloc_test_eval (driver_common.cpp), which is built in both configurations,
# and the GUI branch it decides can never be exercised on a headless machine —
# checking the decision is the only coverage available.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_viewer_run_policy.cpp)
    add_executable(test_viewer_run_policy test_viewer_run_policy.cpp)
    target_link_libraries(test_viewer_run_policy PRIVATE uavloc uavloc_test_eval)
    set_target_properties(test_viewer_run_policy PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_viewer_run_policy COMMAND test_viewer_run_policy)
    # No dataset, no threads: seconds at most. The timeout is the safety net.
    set_tests_properties(test_viewer_run_policy PROPERTIES TIMEOUT 60)
endif()

# Pure unit test of core::RateEstimator, the trailing-window throughput
# estimator behind SystemStats::fps_windowed. NOT gated on ENABLE_VIEWER: the
# estimator lives in core, and the property that matters — a stalled pipeline
# decaying to 0 instead of freezing — is arithmetic over synthetic time points,
# with no dataset, no threads and no real sleeps.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_rate_estimator.cpp)
    add_executable(test_rate_estimator test_rate_estimator.cpp)
    target_link_libraries(test_rate_estimator PRIVATE uavloc)
    set_target_properties(test_rate_estimator PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_rate_estimator COMMAND test_rate_estimator)
    # Synthetic time points only: milliseconds of work. The timeout is a net.
    set_tests_properties(test_rate_estimator PROPERTIES TIMEOUT 60)
endif()

# ── anchor (M1: AnchorInterface + the groundtruth-driven FakeAnchor) ─────────
# Self-contained: the test WRITES its own synthetic groundtruth CSV into the
# working directory and deletes it again, so it needs no dataset. Multi-threaded
# in the ASYNC-mode check — the producer's own stop() drains and joins, the
# ctest TIMEOUT is only the outer safety net.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_anchor.cpp)
    add_executable(test_anchor test_anchor.cpp)
    target_link_libraries(test_anchor PRIVATE uavloc)
    set_target_properties(test_anchor PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_anchor COMMAND test_anchor)
    set_tests_properties(test_anchor PROPERTIES TIMEOUT 120)
endif()

# ── fusion smoke (F1 skeleton: threading + GTSAM linkage, synthetic input) ────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_fusion_smoke.cpp)
    add_executable(test_fusion_smoke test_fusion_smoke.cpp)
    target_link_libraries(test_fusion_smoke PRIVATE uavloc)
    set_target_properties(test_fusion_smoke PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_fusion_smoke COMMAND test_fusion_smoke)
endif()

# ── fusion factors (F1: drift-recovery on synthetic corrupted VO) ────────────
# Includes the private src/fusion/factors.h (attitude chain + yaw helpers), so
# it needs the src include dir and a direct gtsam link (gtsam is PRIVATE on
# uavloc by design — the public API stays gtsam-free).
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_fusion_factors.cpp)
    add_executable(test_fusion_factors test_fusion_factors.cpp)
    target_link_libraries(test_fusion_factors PRIVATE uavloc gtsam)
    target_include_directories(test_fusion_factors PRIVATE ${CMAKE_SOURCE_DIR}/src)
    set_target_properties(test_fusion_factors PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_fusion_factors COMMAND test_fusion_factors)
endif()

# ── full-flight evaluation driver (groundtruth-vs-fused CSV + FPS) ───────────
# Forces VO async + wait_for_local_bundle_adjustment and fusion sync; logs
# per-frame groundtruth ENU, fused prediction, tracked landmarks, coordinate
# errors and processing time. Needs src/debug_viewer for the header-only
# gps_to_enu.h (no viewer lib dependency). Not a ctest (needs the dataset
# video and runs long); invoke manually, optionally with UAVLOC_VO_MAXFRAMES.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_full_flight.cpp)
    add_executable(test_full_flight test_full_flight.cpp)
    target_link_libraries(test_full_flight PRIVATE uavloc uavloc_test_eval)
    target_compile_definitions(test_full_flight PRIVATE
        UAVLOC_HAVE_VPR=$<BOOL:${UAVLOC_HAVE_VPR}>)
    target_include_directories(test_full_flight PRIVATE ${CMAKE_SOURCE_DIR}/src/debug_viewer)
    target_compile_definitions(test_full_flight PRIVATE
        UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_yenbai500m.yaml"
    )
    set_target_properties(test_full_flight PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
endif()

# ── VIEWER-GATED DRIVERS: the two live-view drivers + the parity gate ────────
# All three are built ONLY when the viewer is compiled in. The two drivers have
# no headless fallback branch any more (no #ifdef in their sources): a driver
# whose whole subject is the live view has nothing to do without it.
#
# Consequence, accepted deliberately (2026-08-08): test_driver_parity is an
# ON-ONLY gate. It compares test_full_flight with test_vo_viewer's dump, and
# that dump no longer exists at OFF. Documented in CLAUDE.md and in
# `.docs/designs/viewer_module_design.md` §7.
#
# Both drivers take NO environment variable: the frame cap and the dump paths
# are named constants in tests/driver_common.h, which test_driver_parity reads
# too. That is why nothing but the headless pinning is passed in ENVIRONMENT.
if(ENABLE_VIEWER AND Iridescence_FOUND)
    # ── VO + fusion + viewer, NO absolute position ───────────────────────────
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo_viewer.cpp)
        add_executable(test_vo_viewer test_vo_viewer.cpp)
        # uavloc_test_eval carries driver_common.{h,cpp}: the forced run mode,
        # the shared assembly helpers and the constants above.
        target_link_libraries(test_vo_viewer PRIVATE uavloc uavloc_test_eval)
        target_compile_definitions(test_vo_viewer PRIVATE
            UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_yenbai800m_newvo.yaml"
        )
        set_target_properties(test_vo_viewer PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
        )
        # Headless capped smoke (the cap is compiled in). DISPLAY must be empty:
        # with a window the run blocks until the user closes it.
        add_test(NAME test_vo_viewer COMMAND test_vo_viewer)
        # RESOURCE_LOCK: the dump path is a compiled-in constant, so this test
        # and test_driver_parity (which spawns the same binary and then READS
        # that file) write the very same name. Two writers on one CSV produce a
        # file that looks fine and compares wrong — ctest must not run them
        # concurrently.
        set_tests_properties(test_vo_viewer PROPERTIES
            ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
            RESOURCE_LOCK "viewer_pose_dump"
            TIMEOUT 300
        )
    endif()

    # ── the WHOLE flow: VO + fusion + viewer + absolute fixes (fake anchor) ──
    # ⚠ the fixes are manufactured from groundtruth (M1); the driver prints and
    # displays that caveat. Replaced by real VPR in Phase 2 without touching the
    # back-end.
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo_vpr_anchor_viewer.cpp)
        add_executable(test_vo_vpr_anchor_viewer test_vo_vpr_anchor_viewer.cpp)
        target_link_libraries(test_vo_vpr_anchor_viewer PRIVATE uavloc uavloc_test_eval)
        # UAVLOC_HAVE_VPR là biến CMake, không phải macro — driver cần nó ở dạng
        # macro để rẽ nhánh `--anchor vpr` mà vẫn build được khi module VPR tắt.
        target_compile_definitions(test_vo_vpr_anchor_viewer PRIVATE
            UAVLOC_MISSION_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_yenbai800m_newvo.yaml"
            UAVLOC_HAVE_VPR=$<BOOL:${UAVLOC_HAVE_VPR}>
        )
        set_target_properties(test_vo_vpr_anchor_viewer PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
        )
        # ctest chạy nhánh FAKE: nó tái lập được và không đòi artifact của satVPR.
        # Nhánh thật chạy tay: --anchor vpr.
        add_test(NAME test_vo_vpr_anchor_viewer
                 COMMAND test_vo_vpr_anchor_viewer --anchor fake)
        set_tests_properties(test_vo_vpr_anchor_viewer PROPERTIES
            ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
            TIMEOUT 300
        )
    endif()

    # ── cross-driver parity gate (S9) ────────────────────────────────────────
    # Spawns test_full_flight and test_vo_viewer on the SAME config and cap and
    # checks that their pose sequences differ by a CONSTANT offset only (the
    # viewer's display anchor). Registered separately from test_vo_viewer's own
    # smoke entry. DISPLAY/WAYLAND_DISPLAY are cleared in ENVIRONMENT and again
    # in the spawned command line: with a display the viewer opens a live window
    # and blocks until it is closed.
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_driver_parity.cpp
       AND TARGET test_full_flight AND TARGET test_vo_viewer)
        add_executable(test_driver_parity test_driver_parity.cpp)
        # uavloc_test_eval only for driver_common.h's constants (the cap and the
        # viewer's dump path) — both sides must read the same names.
        target_link_libraries(test_driver_parity PRIVATE uavloc uavloc_test_eval)
        target_compile_definitions(test_driver_parity PRIVATE
            UAVLOC_PARITY_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/uavloc_munfrl_dataset3.yaml"
            UAVLOC_FULL_FLIGHT_EXE="$<TARGET_FILE:test_full_flight>"
            UAVLOC_VO_VIEWER_EXE="$<TARGET_FILE:test_vo_viewer>"
        )
        add_dependencies(test_driver_parity test_full_flight test_vo_viewer)
        set_target_properties(test_driver_parity PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
        )
        add_test(NAME test_driver_parity COMMAND test_driver_parity)
        # Two full 200-frame pipeline runs back to back (~40 s each on the
        # reference machine); the timeout is the outer safety net.
        set_tests_properties(test_driver_parity PROPERTIES
            ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
            RESOURCE_LOCK "viewer_pose_dump"
            TIMEOUT 900
        )
    endif()
endif()

# ── MPEG-TS reader: H.264 video + MISB ST 0601 KLV telemetry ─────────────────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_ts_reader.cpp)
    add_executable(test_ts_reader test_ts_reader.cpp)
    target_link_libraries(test_ts_reader PRIVATE uavloc)
    # KLV phase needs libav demuxing; without it the test builds video-only.
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBAV IMPORTED_TARGET libavformat libavcodec libavutil)
    endif()
    if(LIBAV_FOUND)
        target_link_libraries(test_ts_reader PRIVATE PkgConfig::LIBAV)
        target_compile_definitions(test_ts_reader PRIVATE UAVLOC_WITH_LIBAV)
    endif()
    set_target_properties(test_ts_reader PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_ts_reader COMMAND test_ts_reader)
    set_tests_properties(test_ts_reader PROPERTIES
        # UAVLOC_TS_SHOW=0 keeps ctest headless even on machines with a display.
        ENVIRONMENT "UAVLOC_TS_MAXFRAMES=100;UAVLOC_TS_SHOW=0"
        TIMEOUT 120
    )

    # ── ts_klv_to_csv — converter tool: .ts MISB 0601 KLV → telemetry CSV ────
    # A tool, not a test (no add_test). Shares tests/klv_misb0601.h with
    # test_ts_reader and requires libav for demuxing, so it is only built when
    # the LIBAV pkg-config block above succeeded.
    if(LIBAV_FOUND AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ts_klv_to_csv.cpp)
        add_executable(ts_klv_to_csv ts_klv_to_csv.cpp)
        target_link_libraries(ts_klv_to_csv PRIVATE
            spdlog::spdlog_header_only PkgConfig::LIBAV)
        target_compile_definitions(ts_klv_to_csv PRIVATE UAVLOC_WITH_LIBAV)
        set_target_properties(ts_klv_to_csv PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
        )
    endif()
endif()

# ── vo demo ───────────────────────────────────────────────────────────────────
# Guarded: source is part of an in-progress refactor and may be absent.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vo_demo.cpp)
    add_executable(vo_demo vo_demo.cpp)
    target_link_libraries(vo_demo PRIVATE uavloc)
    set_target_properties(vo_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
endif()

# ── viewer demos — deliberately SKIPPED when the viewer is not compiled in ──
# Both are GUI tools whose whole subject is the viewer, so with
# ENABLE_VIEWER=OFF (or no Iridescence) they are simply not built.
if(ENABLE_VIEWER AND Iridescence_FOUND)
    # Guarded: sources are part of an in-progress refactor and may be absent.
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo_debug_viewer.cpp)
        add_executable(demo_debug_viewer demo_debug_viewer.cpp)
        target_link_libraries(demo_debug_viewer PRIVATE uavloc)
        set_target_properties(demo_debug_viewer PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
        )
    endif()

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/debug_viewer_decode.cpp)
        add_executable(debug_viewer_decode debug_viewer_decode.cpp)
        target_include_directories(debug_viewer_decode PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(debug_viewer_decode PRIVATE uavloc zbar)
        set_target_properties(debug_viewer_decode PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
        )
    endif()
endif()

# ── build hygiene (S5a) — the invariant that replaced R4 ────────────────────
# Checks that the viewer stays confined to its module and that ENABLE_VIEWER
# really changes what lands in libuavloc.so. Pure shell + binutils, headless.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/build_hygiene.sh)
    # The effective state, not the raw option: ENABLE_VIEWER=ON without
    # Iridescence still produces a viewer-free libuavloc.
    if(ENABLE_VIEWER AND Iridescence_FOUND)
        set(UAVLOC_VIEWER_STATE ON)
    else()
        set(UAVLOC_VIEWER_STATE OFF)
    endif()
    add_test(NAME test_build_hygiene
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build_hygiene.sh
                ${CMAKE_SOURCE_DIR}
                ${UAVLOC_VIEWER_STATE}
                $<TARGET_FILE:uavloc>
    )
    set_tests_properties(test_build_hygiene PROPERTIES TIMEOUT 120)
endif()

# ── vpr: chỉ kiểm phần GHÉP, số học có bộ parity riêng ở repo satvpr_core ──────
if(UAVLOC_HAVE_VPR AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vpr.cpp)
    add_executable(test_vpr test_vpr.cpp)
    target_link_libraries(test_vpr PRIVATE uavloc yaml-cpp::yaml-cpp ${OpenCV_LIBS})
    target_compile_definitions(test_vpr PRIVATE SATVPR_DATA_ROOT="${SATVPR_ROOT}")
    set_target_properties(test_vpr PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR})
    add_test(NAME test_vpr COMMAND test_vpr)
    set_tests_properties(test_vpr PROPERTIES TIMEOUT 300)
endif()

if(UAVLOC_HAVE_VPR AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vpr_anchor.cpp)
    add_executable(test_vpr_anchor test_vpr_anchor.cpp)
    target_link_libraries(test_vpr_anchor PRIVATE uavloc yaml-cpp::yaml-cpp ${OpenCV_LIBS})
    target_compile_definitions(test_vpr_anchor PRIVATE SATVPR_DATA_ROOT="${SATVPR_ROOT}")
    set_target_properties(test_vpr_anchor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR})
    add_test(NAME test_vpr_anchor COMMAND test_vpr_anchor)
    set_tests_properties(test_vpr_anchor PROPERTIES TIMEOUT 300)
endif()

if(UAVLOC_HAVE_VPR AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vpr_fine.cpp)
    add_executable(test_vpr_fine test_vpr_fine.cpp)
    target_link_libraries(test_vpr_fine PRIVATE uavloc yaml-cpp::yaml-cpp ${OpenCV_LIBS})
    target_compile_definitions(test_vpr_fine PRIVATE SATVPR_DATA_ROOT="${SATVPR_ROOT}")
    set_target_properties(test_vpr_fine PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR})
    # yenbai_800m: bộ DUY NHẤT đã nâng lên .vprdb v2 (có kho ảnh ô). mun_ds6 còn
    # v1 nên khớp tinh sẽ tắt — nhánh đó có test riêng trong chính binary này.
    add_test(NAME test_vpr_fine COMMAND test_vpr_fine yenbai_800m 30)
    set_tests_properties(test_vpr_fine PROPERTIES TIMEOUT 600)
endif()

# ── test_vpr_flow: driver ĐO trên dữ liệu thật, không phải cổng pass/fail ─────
# Mặc định trỏ vào bộ dữ liệu của satvpr_core; mọi thứ ghi đè được bằng tham số
# dòng lệnh, nên chạy được trên bất kỳ eval set nào có đủ query_latlon.csv,
# queries/, pca.yml và <tên>.vprdb. Thiếu dữ liệu -> soft-skip.
if(UAVLOC_HAVE_VPR AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vpr_flow.cpp)
    add_executable(test_vpr_flow test_vpr_flow.cpp)
    target_link_libraries(test_vpr_flow PRIVATE uavloc ${OpenCV_LIBS})
    target_compile_definitions(test_vpr_flow PRIVATE
        DEFAULT_DATASET_DIR="${SATVPR_ROOT}/dataset/eval/mun_ds6"
        DEFAULT_WEIGHTS_DIR="${SATVPR_ROOT}/.weights")
    set_target_properties(test_vpr_flow PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR})
    add_test(NAME test_vpr_flow COMMAND test_vpr_flow --queries 20 --yaw-offset 180)
    set_tests_properties(test_vpr_flow PROPERTIES TIMEOUT 600)
endif()
