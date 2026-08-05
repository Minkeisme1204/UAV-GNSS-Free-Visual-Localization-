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
# add_subdirectory(vo)   # old vo module removed from the working tree (src/vo → vo.zip,
#                        # include/uavloc/vo deleted); superseded by new_vo. Re-enable
#                        # only if the old vo sources+headers are restored.
add_subdirectory(util)
add_subdirectory(new_vo)   # VoFrameLoader subset enabled (feature/orb + data/ + camera adapter)
# add_subdirectory(vpr)
add_subdirectory(fusion)   # GTSAM back-end (links gtsam PRIVATE in src/fusion/)
add_subdirectory(anchor)   # AnchorInterface + the M1 groundtruth FakeAnchor
add_subdirectory(core)     # SystemConfig (S2); SystemManager lands in S3

# add_subdirectory(utils)
