# Hard Constraints

Things that must never happen regardless of task context.

- **No `.thirdparty/` edits** — stella_vslam and other vendored sources are read-only references.
- **No `apps/` in root CMake** — `apps/` is an independent build; never add it via `add_subdirectory` from the root `CMakeLists.txt`.
- **No `using namespace` in headers** — causes namespace pollution for every translation unit that includes the header.
- **No implementation in public headers** — unless the code is a template. Logic belongs in `src/<module>/`.
- **No magic numbers in source** — all tunable values go in the YAML config and are loaded at runtime.
- **No `std::cout` for runtime messages** — use `spdlog` at the appropriate log level.
- **No binary assets in git** — model weights, imagery, and datasets go in `data/` which is gitignored.
