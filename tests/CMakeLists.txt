# All test executables land in <build>/tests/
set(TEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/tests)

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
    target_link_libraries(test_vo_viewer PRIVATE uavloc)
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
