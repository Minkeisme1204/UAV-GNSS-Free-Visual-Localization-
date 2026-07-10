# C++ Coding Rules

## Namespace
- Every symbol in the library must be inside `namespace uavloc { }`.
- Sub-namespaces are allowed (e.g. `namespace uavloc::vpr { }`) but keep them shallow.
- Never use `using namespace` in any header file.

## Headers
- Public API headers live in `include/uavloc/<module>/`.
- Implementation-only helpers live in `src/<module>/` and are never installed.
- Use `#pragma once` — not `#ifndef` include guards.
- Correct OpenCV include: `#include <opencv2/opencv.hpp>` (not `cv2/opencv.hpp`).
- Do not put implementation code in public headers unless it is a template.

## Language and style
- C++17 minimum.
- Naming: `PascalCase` classes, `snake_case` functions/variables, `UPPER_SNAKE_CASE` constants/enums.
- No raw pointers for ownership — use `std::unique_ptr` / `std::shared_ptr`.
- Enum classes only — never plain `enum`.
- Every class/struct definition ends with a semicolon.

## Logging
- Use `spdlog` throughout. Never use `std::cout` for runtime messages.
- `spdlog::debug` for internals, `spdlog::info` for state changes, `spdlog::warn`/`spdlog::error` for recoverable/fatal issues.

## Configuration
- All runtime parameters are loaded from YAML files via `yaml-cpp`.
- No hard-coded magic numbers in source — put them in the config file and load at startup.
- YAML loading follows the `node["key"].as<T>(default)` pattern so missing keys never throw.

## Data types — images vs. matrices
- **Images** (raw pixels, decoded frames, masks): always `cv::Mat`. This includes anything read from a camera, file, or produced by an OpenCV pipeline.
- **Matrix computation and declarations** (poses, rotations, translations, covariance, Jacobians, linear algebra): always Eigen types (e.g. `Eigen::Matrix3d`, `Eigen::Vector3d`, `Eigen::Isometry3d`). Never use `cv::Mat` for numeric computation.
- Conversions between the two are allowed only at the seam where an OpenCV algorithm produces a result that must enter numeric computation — use `cv::cv2eigen` / `cv::eigen2cv` from `<opencv2/core/eigen.hpp>`.

## Third-party
- Do not modify anything under `.thirdparty/` — treat it as read-only vendored code.
