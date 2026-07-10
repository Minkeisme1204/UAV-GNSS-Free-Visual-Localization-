add_library(uavloc SHARED)

# PUBLIC so downstream targets (tests, apps) inherit include paths
target_include_directories(uavloc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(uavloc PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    spdlog::spdlog_header_only
    yaml-cpp::yaml-cpp
    Threads::Threads
)

# ── Modules ───────────────────────────────────────────────────────────────────
add_subdirectory(sensor)

# Uncomment each line as the module gains its first source file:
add_subdirectory(vo)
# add_subdirectory(vpr)
# add_subdirectory(fusion)   # also link GTSAM in root CMakeLists first
# add_subdirectory(anchor)
# add_subdirectory(core)
# add_subdirectory(utils)
