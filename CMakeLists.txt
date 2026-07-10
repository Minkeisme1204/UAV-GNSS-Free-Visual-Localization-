cmake_minimum_required(VERSION 3.16)
project(uavloc VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # generates compile_commands.json for clangd/IDEs

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ── Dependencies ──────────────────────────────────────────────────────────────
list(PREPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/.local")

list(APPEND CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${PROJECT_SOURCE_DIR}/.local/lib/cmake
    ${PROJECT_SOURCE_DIR}/.local/share/cmake)

find_package(Eigen3   REQUIRED NO_MODULE)
find_package(OpenCV   REQUIRED)
find_package(spdlog   REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads  REQUIRED)   # VO Tracking / Local Mapping threads
# yaml-cpp 0.7 exports the unnamespaced target "yaml-cpp"; create the
# namespaced alias so code can use yaml-cpp::yaml-cpp unconditionally.
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# Uncomment when the corresponding module is implemented:
# find_package(GTSAM        REQUIRED)   # fusion module
# find_package(onnxruntime  REQUIRED)   # vpr module

# ── Optional: debug_viewer (requires Iridescence) ─────────────────────────────
# Iridescence and its bundled GLFW are installed under .local/.
# glfw3 must be found first so that the "glfw" target exists when
# iridescence-targets.cmake validates its interface link libraries.
find_package(glfw3 QUIET
    PATHS "${PROJECT_SOURCE_DIR}/.local/lib/cmake/glfw3"
    NO_DEFAULT_PATH)
find_package(iridescence QUIET
    PATHS "${PROJECT_SOURCE_DIR}/.local/lib/cmake/iridescence"
    NO_DEFAULT_PATH)
if(iridescence_FOUND)
    set(Iridescence_FOUND TRUE)
    message(STATUS "Found Iridescence: ${PROJECT_SOURCE_DIR}/.local")
else()
    message(WARNING "Iridescence not found — debug_viewer module will not be built")
endif()

# ── Build options ─────────────────────────────────────────────────────────────
option(BUILD_TESTING "Build test programs" ON)

# ── Library ───────────────────────────────────────────────────────────────────
add_subdirectory(src)

if(Iridescence_FOUND)
    add_subdirectory(src/debug_viewer)
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
include(GNUInstallDirs)

install(TARGETS uavloc
    EXPORT  uavlocTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT uavlocTargets
    FILE      uavlocTargets.cmake
    NAMESPACE uavloc::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uavloc
)
