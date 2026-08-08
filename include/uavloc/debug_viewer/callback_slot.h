#pragma once

// debug_viewer::CallbackSlot is now an ALIAS of util::CallbackSlot (S5b,
// .docs/designs/system_manager_design.md §3.7). The viewer keeps its own
// spelling — every call site and DebugViewerCallbacks stay unchanged — but the
// shared implementation fixes the two defects of the old local copy: it no
// longer holds the mutex while invoking a subscriber (add() from inside a
// callback used to deadlock) and it supports remove(id).
//
// util/callback_slot.h is a header-only template, so including it does NOT make
// uavloc_debug_viewer depend on libuavloc: the library still links no uavloc
// target (see src/debug_viewer/CMakeLists.txt).

#include <uavloc/util/callback_slot.h>

namespace uavloc::debug_viewer {

template <typename Signature>
using CallbackSlot = util::CallbackSlot<Signature>;

} // namespace uavloc::debug_viewer
