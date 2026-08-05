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
add_executable(test_video_reader test_video_reader.cpp)
target_link_libraries(test_video_reader PRIVATE uavloc)
set_target_properties(test_video_reader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME test_video_reader COMMAND test_video_reader)

# ── drone-log telemetry parser ────────────────────────────────────────────────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_drone_telemetry.cpp)
    add_executable(test_drone_telemetry test_drone_telemetry.cpp)
    target_link_libraries(test_drone_telemetry PRIVATE uavloc)
    set_target_properties(test_drone_telemetry PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_drone_telemetry COMMAND test_drone_telemetry)
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
    set_target_properties(test_vo_frame_loader PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_vo_frame_loader COMMAND test_vo_frame_loader)
endif()

# ── new_vo end-to-end pipeline (VOModule) ─────────────────────────────────────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo_pipeline_new.cpp)
    add_executable(test_vo_pipeline_new test_vo_pipeline_new.cpp)
    target_link_libraries(test_vo_pipeline_new PRIVATE uavloc)
    set_target_properties(test_vo_pipeline_new PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    add_test(NAME test_vo_pipeline_new COMMAND test_vo_pipeline_new)
endif()

# ── new_vo debug driver (VOData publish callback + top-down trajectory) ───────
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo_debug_new.cpp)
    add_executable(test_vo_debug_new test_vo_debug_new.cpp)
    target_link_libraries(test_vo_debug_new PRIVATE uavloc)
    set_target_properties(test_vo_debug_new PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    # Optional live 3D viewer (trajectory + map-point cloud). Only when
    # Iridescence and the debug_viewer library are available. Linking both uavloc
    # and uavloc_debug_viewer in one exe is valid — they are independent units.
    if(Iridescence_FOUND AND TARGET uavloc_debug_viewer)
        target_link_libraries(test_vo_debug_new PRIVATE uavloc_debug_viewer)
        target_compile_definitions(test_vo_debug_new PRIVATE UAVLOC_WITH_DEBUG_VIEWER)
        # gps_to_enu.h lives in src/debug_viewer (header-only helper).
        target_include_directories(test_vo_debug_new PRIVATE ${CMAKE_SOURCE_DIR}/src/debug_viewer)
        target_link_options(test_vo_debug_new PRIVATE "-Wl,--disable-new-dtags")
        set_target_properties(test_vo_debug_new PROPERTIES
            BUILD_RPATH   "${CMAKE_SOURCE_DIR}/.local/lib"
            INSTALL_RPATH "${CMAKE_SOURCE_DIR}/.local/lib"
        )
    endif()
    add_test(NAME test_vo_debug_new COMMAND test_vo_debug_new)
endif()

# ── new_vo LIVE 3D viewer driver (trajectory + groundtruth + map cloud) ───────
# Manual/demo tool: the VO pipeline runs on a worker thread while DebugViewer::run()
# owns the main thread (live window). Not registered as a blocking ctest that runs
# to END_OF_STREAM; a headless capped smoke (UAVLOC_VO_MAXFRAMES) is added instead.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_vo_viewer.cpp)
    add_executable(test_vo_viewer test_vo_viewer.cpp)
    # uavloc_test_eval carries driver_common.{h,cpp}: the forced run mode and the
    # fake-anchor configuration this driver must share with test_full_flight.
    target_link_libraries(test_vo_viewer PRIVATE uavloc uavloc_test_eval)
    set_target_properties(test_vo_viewer PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
    # Optional live 3D viewer. Only when Iridescence and the debug_viewer library
    # are available. Linking both uavloc and uavloc_debug_viewer in one exe is
    # valid — they are independent code units.
    if(Iridescence_FOUND AND TARGET uavloc_debug_viewer)
        target_link_libraries(test_vo_viewer PRIVATE uavloc_debug_viewer)
        target_compile_definitions(test_vo_viewer PRIVATE UAVLOC_WITH_DEBUG_VIEWER)
        # gps_to_enu.h lives in src/debug_viewer (header-only helper).
        target_include_directories(test_vo_viewer PRIVATE ${CMAKE_SOURCE_DIR}/src/debug_viewer)
        target_link_options(test_vo_viewer PRIVATE "-Wl,--disable-new-dtags")
        set_target_properties(test_vo_viewer PROPERTIES
            BUILD_RPATH   "${CMAKE_SOURCE_DIR}/.local/lib"
            INSTALL_RPATH "${CMAKE_SOURCE_DIR}/.local/lib"
        )
    endif()
    # Headless capped smoke only — never a blocking run-to-END_OF_STREAM ctest.
    add_test(NAME test_vo_viewer COMMAND test_vo_viewer)
    set_tests_properties(test_vo_viewer PROPERTIES
        ENVIRONMENT "UAVLOC_VO_MAXFRAMES=200"
        TIMEOUT 120
    )
endif()

# The old vo module (include/uavloc/vo) was removed from the working tree; the
# three tests below depend on its public headers, so they are guarded and skip
# while it is absent. They auto-return if the vo headers are restored.
if(EXISTS ${CMAKE_SOURCE_DIR}/include/uavloc/vo/vo_module.h)

# ── adjacent matching ─────────────────────────────────────────────────────────
add_executable(test_adjacent_matching test_adjacent_matching.cpp)
target_link_libraries(test_adjacent_matching PRIVATE uavloc)
set_target_properties(test_adjacent_matching PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME test_adjacent_matching COMMAND test_adjacent_matching)

# ── vo pipeline trajectory ────────────────────────────────────────────────────
add_executable(test_vo_pipeline test_vo_pipeline.cpp)
target_link_libraries(test_vo_pipeline PRIVATE uavloc)
# Drives the pipeline entirely through the public vo::VOModule — no internal
# src/vo headers needed anymore.
set_target_properties(test_vo_pipeline PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
# Optional debug-viewer overlay (estimate vs groundtruth). Only when Iridescence
# and the debug_viewer library are available. Linking both uavloc and
# uavloc_debug_viewer in one exe is valid — they are independent code units.
if(Iridescence_FOUND AND TARGET uavloc_debug_viewer)
    target_link_libraries(test_vo_pipeline PRIVATE uavloc_debug_viewer)
    target_compile_definitions(test_vo_pipeline PRIVATE UAVLOC_WITH_DEBUG_VIEWER)
    # gps_to_enu.h lives in src/debug_viewer (header-only helper).
    target_include_directories(test_vo_pipeline PRIVATE ${CMAKE_SOURCE_DIR}/src/debug_viewer)
    target_link_options(test_vo_pipeline PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(test_vo_pipeline PROPERTIES
        BUILD_RPATH   "${CMAKE_SOURCE_DIR}/.local/lib"
        INSTALL_RPATH "${CMAKE_SOURCE_DIR}/.local/lib"
    )
endif()
add_test(NAME test_vo_pipeline COMMAND test_vo_pipeline)

# ── vo debug (first-N-frames per-frame log) ─────────────────────────────────────
# Drives the pipeline through the public vo::VOModule and logs a detailed block
# per frame for the first N valid frames — a compact diagnostic driver.
add_executable(test_vo_debug test_vo_debug.cpp)
target_link_libraries(test_vo_debug PRIVATE uavloc)
set_target_properties(test_vo_debug PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME test_vo_debug COMMAND test_vo_debug)

endif()  # old vo module present

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

# ── fusion offline evaluation driver (real pipeline → per-frame CSV) ─────────
# Runs VideoReader → new_vo VOModule → FusionModule (forced synchronous) on a
# mission config and dumps a per-frame CSV for offline error analysis. Public
# API only — links like test_vo_pipeline_new. Not a ctest (needs the dataset
# video and runs long); invoke manually with UAVLOC_VO_MAXFRAMES.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_fusion_offline.cpp)
    add_executable(test_fusion_offline test_fusion_offline.cpp)
    target_link_libraries(test_fusion_offline PRIVATE uavloc)
    set_target_properties(test_fusion_offline PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
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
    target_include_directories(test_full_flight PRIVATE ${CMAKE_SOURCE_DIR}/src/debug_viewer)
    set_target_properties(test_full_flight PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
    )
endif()

# ── cross-driver parity gate (S9) ────────────────────────────────────────────
# Spawns test_full_flight and test_vo_viewer on the SAME config/cap and checks
# that their pose sequences differ by a CONSTANT offset only (the viewer's
# display anchor). Registered separately from test_vo_viewer's own smoke entry,
# which is left untouched. DISPLAY/WAYLAND_DISPLAY are cleared in ENVIRONMENT
# and again in the spawned command line: with a display the viewer opens a live
# window and blocks until it is closed.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_driver_parity.cpp
   AND TARGET test_full_flight AND TARGET test_vo_viewer)
    add_executable(test_driver_parity test_driver_parity.cpp)
    target_link_libraries(test_driver_parity PRIVATE uavloc)
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
    # Two full 200-frame pipeline runs back to back (~40 s each on the reference
    # machine); the timeout is the outer safety net, not an expectation.
    set_tests_properties(test_driver_parity PROPERTIES
        ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="
        TIMEOUT 900
    )
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

# ── test_iridescence — only needs Iridescence (no debug_viewer target required) ──
if(Iridescence_FOUND)
    add_executable(test_iridescence test_iridescence.cpp)
    target_link_libraries(test_iridescence PRIVATE Iridescence::iridescence Eigen3::Eigen)
    target_link_options(test_iridescence PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(test_iridescence PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
        BUILD_RPATH  "${CMAKE_SOURCE_DIR}/.local/lib"
        INSTALL_RPATH "${CMAKE_SOURCE_DIR}/.local/lib"
    )
endif()

# ── demo_debug_viewer — needs both Iridescence and the debug_viewer library ──
if(Iridescence_FOUND AND TARGET uavloc_debug_viewer)
    # Guarded: sources are part of an in-progress refactor and may be absent.
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo_debug_viewer.cpp)
        add_executable(demo_debug_viewer demo_debug_viewer.cpp)
        target_link_libraries(demo_debug_viewer PRIVATE uavloc_debug_viewer spdlog::spdlog_header_only)
        target_link_options(demo_debug_viewer PRIVATE "-Wl,--disable-new-dtags")
        set_target_properties(demo_debug_viewer PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
            BUILD_RPATH  "${CMAKE_SOURCE_DIR}/.local/lib"
            INSTALL_RPATH "${CMAKE_SOURCE_DIR}/.local/lib"
        )
    endif()

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/debug_viewer_decode.cpp)
        add_executable(debug_viewer_decode debug_viewer_decode.cpp)
        target_include_directories(debug_viewer_decode PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(debug_viewer_decode PRIVATE uavloc_debug_viewer ${OpenCV_LIBS} zbar)
        target_link_options(debug_viewer_decode PRIVATE "-Wl,--disable-new-dtags")
        set_target_properties(debug_viewer_decode PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
            BUILD_RPATH  "${CMAKE_SOURCE_DIR}/.local/lib"
            INSTALL_RPATH "${CMAKE_SOURCE_DIR}/.local/lib"
        )
    endif()
endif()
