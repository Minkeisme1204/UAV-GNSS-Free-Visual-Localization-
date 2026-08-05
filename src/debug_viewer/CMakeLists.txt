# debug_viewer is an INDEPENDENT static library: it does NOT link libuavloc.
# It decodes the per-frame barcode (zbar), builds the GPS->ENU groundtruth
# trajectory, and renders it live via Iridescence.

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

add_library(uavloc_debug_viewer STATIC
    barcode_decoder.cpp
    debug_viewer.cpp
    debug_viewer_callbacks.cpp
    telemetry_csv_reader.cpp
    trajectory_viewer.cpp
)

target_include_directories(uavloc_debug_viewer
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(uavloc_debug_viewer
    PUBLIC
        Iridescence::iridescence
        Eigen3::Eigen
        # PUBLIC since S5b: the public header callback_slot.h aliases the
        # header-only util::CallbackSlot, which logs subscriber exceptions with
        # spdlog, so every consumer TU needs spdlog's headers. spdlog lives in
        # .local/, not on a system include path. This is a HEADER-ONLY target —
        # uavloc_debug_viewer still links no uavloc target (R4).
        spdlog::spdlog_header_only
    PRIVATE
        ${OpenCV_LIBS}
        PkgConfig::ZBAR
)

target_include_directories(uavloc_debug_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})

target_compile_features(uavloc_debug_viewer PUBLIC cxx_std_17)
