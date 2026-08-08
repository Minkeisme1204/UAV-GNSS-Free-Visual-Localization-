#!/usr/bin/env bash
# build_hygiene.sh — the invariant that replaced R4 (S5a, 2026-08-08).
#
# Since debug_viewer became a module of libuavloc, "libuavloc must not link the
# viewer" is no longer the guarantee. What guarantees a viewer-free flight
# binary instead is:
#   * the viewer's code and headers stay inside their two directories, and
#   * ENABLE_VIEWER / UAVLOC_WITH_DEBUG_VIEWER are never read by uavloc sources,
#     so libuavloc's ABI cannot diverge between the two configurations, and
#   * with ENABLE_VIEWER=OFF the shipped .so really carries no viewer symbol and
#     no GL/GLFW/zbar dependency, while with ON it really does (proving the flag
#     is not a no-op).
#
# Usage:  build_hygiene.sh <repo_root> <ON|OFF> <path/to/libuavloc.so>
# Exit 0 only when every applicable check passes; every failure is printed.

set -u

if [ "$#" -ne 3 ]; then
    echo "usage: $0 <repo_root> <ON|OFF> <libuavloc.so>" >&2
    exit 2
fi

REPO="$1"
VIEWER_STATE="$2"
LIB="$3"

VIEWER_SRC_DIR="src/debug_viewer/"
VIEWER_INC_DIR="include/uavloc/debug_viewer/"

failures=0

fail() {
    echo "FAIL $1: $2"
    failures=$((failures + 1))
}

pass() {
    echo "PASS $1: $2"
}

if [ ! -d "$REPO/src" ] || [ ! -d "$REPO/include/uavloc" ]; then
    echo "FAIL setup: '$REPO' does not look like the uavloc repo root" >&2
    exit 2
fi

cd "$REPO" || exit 2

# ── K1: nothing outside the viewer module includes a viewer header ───────────
# A viewer include in a normal module would drag GL into the flight build.
k1=$(grep -rn "#include.*debug_viewer" include/uavloc src \
        --include='*.h' --include='*.hpp' --include='*.cpp' 2>/dev/null \
     | grep -v "^${VIEWER_SRC_DIR}" \
     | grep -v "^${VIEWER_INC_DIR}")
if [ -n "$k1" ]; then
    fail K1 "viewer headers included from outside the viewer module:"
    printf '%s\n' "$k1" | sed 's/^/        /'
else
    pass K1 "no viewer header is included outside ${VIEWER_SRC_DIR} / ${VIEWER_INC_DIR}"
fi

# ── K2: the switch is invisible to library code (anti-ABI-divergence) ────────
# The most important check: if any uavloc TRANSLATION UNIT compiled differently
# under the flag, libuavloc's ABI would depend on ENABLE_VIEWER and an ON-built
# consumer could silently mismatch an OFF-built library. Only compiled sources
# and headers carry ABI, so only those are scanned — the one CMake reference the
# design requires (the module gate in src/CMakeLists.txt, which .claude/rules/
# cmake.md forces to live there) is covered by K2b instead.
k2=$(grep -rn "ENABLE_VIEWER\|UAVLOC_WITH_DEBUG_VIEWER" src include \
        --include='*.h' --include='*.hpp' --include='*.cpp' 2>/dev/null \
     | grep -v "^${VIEWER_SRC_DIR}" \
     | grep -v "^${VIEWER_INC_DIR}")
if [ -n "$k2" ]; then
    fail K2 "the viewer switch is referenced by library sources outside the viewer module (ABI may diverge):"
    printf '%s\n' "$k2" | sed 's/^/        /'
else
    pass K2 "no source/header outside the viewer module reads ENABLE_VIEWER / UAVLOC_WITH_DEBUG_VIEWER"
fi

# ── K2b: only the viewer module may DEFINE the macro ─────────────────────────
# Keeps the build system from switching the macro on somewhere else, which would
# reintroduce exactly the divergence K2 forbids.
k2b=$(grep -rn "UAVLOC_WITH_DEBUG_VIEWER" src include \
         --include='CMakeLists.txt' --include='*.cmake' 2>/dev/null \
      | grep -v "^${VIEWER_SRC_DIR}" \
      | grep -v "^${VIEWER_INC_DIR}")
if [ -n "$k2b" ]; then
    fail K2b "UAVLOC_WITH_DEBUG_VIEWER is defined outside the viewer module's CMakeLists:"
    printf '%s\n' "$k2b" | sed 's/^/        /'
else
    pass K2b "only ${VIEWER_SRC_DIR}CMakeLists.txt defines UAVLOC_WITH_DEBUG_VIEWER"
fi

if [ ! -f "$LIB" ]; then
    fail setup "library not found: $LIB"
    echo "build_hygiene: $failures check(s) failed"
    exit 1
fi

nm_count=$(nm -DC "$LIB" 2>/dev/null | grep -c "debug_viewer\|DebugViewer")

if [ "$VIEWER_STATE" = "OFF" ]; then
    # ── K3: no viewer symbol survives in the deployment library ─────────────
    if [ "$nm_count" -ne 0 ]; then
        fail K3 "libuavloc.so exports $nm_count viewer symbol(s) with ENABLE_VIEWER=OFF"
    else
        pass K3 "no debug_viewer/DebugViewer symbol in $(basename "$LIB")"
    fi

    # ── K4: no GUI shared-object dependency ────────────────────────────────
    # The pattern is anchored on purpose: a case-insensitive 'libGL' also
    # matches libglib-2.0, which produced a false failure once already.
    k4=$(objdump -p "$LIB" 2>/dev/null | awk '/NEEDED/{print $2}' \
         | grep -E 'libGL\.|libGLX|libEGL|glfw|iridescence|imgui|zbar')
    if [ -n "$k4" ]; then
        fail K4 "GUI libraries are still NEEDED by libuavloc.so:"
        printf '%s\n' "$k4" | sed 's/^/        /'
    else
        pass K4 "no GL/GLFW/iridescence/imgui/zbar in NEEDED"
    fi
    echo "SKIP K5: only meaningful with ENABLE_VIEWER=ON"
else
    # ── K5: the flag is not a no-op ────────────────────────────────────────
    # Without this, K3/K4 would be trivially green forever if the module ever
    # stopped being compiled at all.
    if [ "$nm_count" -eq 0 ]; then
        fail K5 "ENABLE_VIEWER=ON but libuavloc.so exports no viewer symbol — the flag has no effect"
    else
        pass K5 "libuavloc.so exports $nm_count viewer symbol(s)"
    fi
    echo "SKIP K3/K4: only meaningful with ENABLE_VIEWER=OFF"
fi

if [ "$failures" -ne 0 ]; then
    echo "build_hygiene: $failures check(s) failed"
    exit 1
fi

echo "build_hygiene: all checks passed (ENABLE_VIEWER=$VIEWER_STATE)"
exit 0
