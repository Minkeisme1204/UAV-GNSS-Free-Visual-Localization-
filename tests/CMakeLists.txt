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
