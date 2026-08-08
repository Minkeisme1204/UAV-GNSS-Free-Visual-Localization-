# debug_viewer — a MODULE OF libuavloc (changed 2026-08-08, S5a; it used to be
# the independent STATIC library uavloc_debug_viewer under the old R4).
# Reached only when ENABLE_VIEWER is ON and Iridescence was found, so every
# GL/GLFW/zbar dependency below is confined to development builds; a deployment
# build (-DENABLE_VIEWER=OFF) never enters this file.
#
# Invariant that replaces old R4: neither ENABLE_VIEWER nor
# UAVLOC_WITH_DEBUG_VIEWER may appear anywhere under src/ or include/uavloc/
# outside this module's two directories — that keeps libuavloc's ABI identical
# in both configurations. Checked by tests/build_hygiene.sh (test_build_hygiene).

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

target_sources(uavloc PRIVATE
    barcode_decoder.cpp
    debug_viewer.cpp
    debug_viewer_bridge.cpp
    debug_viewer_callbacks.cpp
    telemetry_csv_reader.cpp
    trajectory_aligner.cpp
    trajectory_viewer.cpp
    viewer_config.cpp
)

# Module-private headers (barcode_decoder.h, debug_viewer_bridge.h,
# gps_to_enu.h, log_ring_sink.h).
target_include_directories(uavloc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# PRIVATE: no public debug_viewer header includes an Iridescence or zbar header
# (DebugViewer/TrajectoryViewer are pimpl), so consumers never need them.
# Eigen / spdlog / yaml-cpp / OpenCV are already PUBLIC on uavloc.
target_link_libraries(uavloc PRIVATE
    Iridescence::iridescence
    PkgConfig::ZBAR
)

# Tells consumers (currently only tests/) that the viewer is compiled in. It is
# deliberately NOT referenced by any uavloc source — see the invariant above.
target_compile_definitions(uavloc PUBLIC UAVLOC_WITH_DEBUG_VIEWER)

# Iridescence and its bundled GLFW live in .local/lib. --disable-new-dtags emits
# DT_RPATH instead of DT_RUNPATH: RUNPATH is not transitive, so it would resolve
# libiridescence.so but not the libraries libiridescence.so itself pulls from
# .local/lib.
target_link_options(uavloc PRIVATE "-Wl,--disable-new-dtags")
set_property(TARGET uavloc APPEND PROPERTY BUILD_RPATH   "${CMAKE_SOURCE_DIR}/.local/lib")
set_property(TARGET uavloc APPEND PROPERTY INSTALL_RPATH "${CMAKE_SOURCE_DIR}/.local/lib")
