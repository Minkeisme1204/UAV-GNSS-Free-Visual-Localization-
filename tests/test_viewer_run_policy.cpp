// test_viewer_run_policy — unit test for eval::viewer_run_policy(), the single
// predicate both viewer drivers ask: "is this a REGRESSION run or an
// INTERACTIVE session?".
//
// Why this test exists: the two decisions that follow from that question —
// who starts the pipeline (autostart) and how much data it consumes
// (max_frames) — were once taken in two different places. The regression run's
// 200-frame cap then leaked into interactive sessions, where the supervisor hit
// it, stopped the system, and left a "Start" button that could no longer start
// anything (2026-08-08). Nothing on a headless CI machine can exercise the GUI
// branch, but the DECISION is pure and can be checked here.
//
// Checks (the environment is manipulated in-process with setenv/unsetenv):
//   P1  neither DISPLAY nor WAYLAND_DISPLAY set        -> headless / regression
//   P2  DISPLAY set, WAYLAND_DISPLAY unset             -> interactive
//   P3  WAYLAND_DISPLAY set, DISPLAY unset             -> interactive
//   P4  both set                                       -> interactive
//   P5  both SET BUT EMPTY (`DISPLAY=`)                -> headless / regression
//       — this is the one DebugViewer::run() also treats as headless; if the
//       two tests disagreed, a driver would wait for a button in a window that
//       was never opened.
// In every case both fields are asserted TOGETHER: a regression run must be
// (autostart = true, max_frames = PARITY_MAX_FRAMES) and an interactive one
// (autostart = false, max_frames = INTERACTIVE_MAX_FRAMES = 0).
//
// Pure unit test: no dataset, no config, no GL, no pipeline.

#include "driver_common.h"

#include <spdlog/spdlog.h>

#include <cstddef>
#include <cstdlib>

namespace {

int g_rc = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        g_rc = 1;
        spdlog::error("FAIL: {}", what);
    }
}

//! The two decisions of a headless run, asserted as a pair on purpose.
void check_regression(const uavloc::eval::RunPolicy& p, const char* what) {
    check(p.autostart, what);
    check(p.max_frames ==
              static_cast<std::size_t>(uavloc::eval::PARITY_MAX_FRAMES),
          what);
}

//! The two decisions of an interactive session, likewise as a pair.
void check_interactive(const uavloc::eval::RunPolicy& p, const char* what) {
    check(!p.autostart, what);
    check(p.max_frames == uavloc::eval::INTERACTIVE_MAX_FRAMES, what);
    check(p.max_frames == 0, what);  // 0 is what the viewer reads as "no cap"
}

//! Non-empty values only have to be non-empty; their content is never parsed.
constexpr const char* X11_DISPLAY_VALUE    = ":0";
constexpr const char* WAYLAND_SOCKET_VALUE = "wayland-0";

void set_display(const char* display, const char* wayland) {
    if (display != nullptr) {
        setenv("DISPLAY", display, 1);
    } else {
        unsetenv("DISPLAY");
    }
    if (wayland != nullptr) {
        setenv("WAYLAND_DISPLAY", wayland, 1);
    } else {
        unsetenv("WAYLAND_DISPLAY");
    }
}

} // namespace

int main() {
    using namespace uavloc;
    spdlog::set_level(spdlog::level::info);

    // P1 — nothing set: the ctest / parity-gate situation.
    set_display(nullptr, nullptr);
    check_regression(eval::viewer_run_policy(),
                     "P1: no DISPLAY and no WAYLAND_DISPLAY must be a capped, "
                     "self-starting regression run");

    // P2 — X11 only.
    set_display(X11_DISPLAY_VALUE, nullptr);
    check_interactive(eval::viewer_run_policy(),
                      "P2: DISPLAY set must be an uncapped interactive session");

    // P3 — Wayland only.
    set_display(nullptr, WAYLAND_SOCKET_VALUE);
    check_interactive(eval::viewer_run_policy(),
                      "P3: WAYLAND_DISPLAY set must be an uncapped interactive "
                      "session");

    // P4 — both (an XWayland session).
    set_display(X11_DISPLAY_VALUE, WAYLAND_SOCKET_VALUE);
    check_interactive(eval::viewer_run_policy(),
                      "P4: both display variables set must be an uncapped "
                      "interactive session");

    // P5 — both set but EMPTY. This is how ctest pins the viewer drivers
    // headless (ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY="), and it is what
    // DebugViewer::run() treats as headless too.
    set_display("", "");
    check_regression(eval::viewer_run_policy(),
                     "P5: an EMPTY DISPLAY/WAYLAND_DISPLAY must count as "
                     "headless, exactly as DebugViewer::run() does");

    if (g_rc != 0) {
        spdlog::error("test_viewer_run_policy: FAIL");
        return g_rc;
    }
    spdlog::info("test_viewer_run_policy: PASS");
    return g_rc;
}
