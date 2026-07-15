# CMake Rules

## Hierarchy — strictly inner to outer, never skip a level

```
src/<module>/CMakeLists.txt   → OBJECT library or source list for that module
src/CMakeLists.txt            → add_subdirectory per module, assembles uavloc target
CMakeLists.txt (root)         → find_package deps, add_subdirectory(src), add_subdirectory(tests)
apps/CMakeLists.txt           → INDEPENDENT — never included from root via add_subdirectory
```

## Target naming
- Library: `uavloc`
- Tests: `test_<module>` (e.g. `test_sensor`, `test_vpr`)
- Apps: freeform, defined entirely within `apps/`

## Required find_package calls in root CMakeLists
```cmake
find_package(Eigen3     REQUIRED)
find_package(OpenCV     REQUIRED)
find_package(GTSAM      REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(spdlog     REQUIRED)
find_package(yaml-cpp   REQUIRED)
```

## Hard rules
- `apps/` is a separate build ecosystem — never add it via `add_subdirectory` from the root.
- Each module owns its own `CMakeLists.txt`; do not define module sources in the parent list.
