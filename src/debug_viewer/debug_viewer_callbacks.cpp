#include <uavloc/debug_viewer/debug_viewer_callbacks.h>

namespace uavloc::debug_viewer {

// Definitions for the static callback slots declared in the header.
CallbackSlot<void(const TelemetryRecord&)>              DebugViewerCallbacks::on_telemetry;
CallbackSlot<void(const std::string&, const std::string&)> DebugViewerCallbacks::on_status;
CallbackSlot<void(double, float, const std::string&)>   DebugViewerCallbacks::on_metric;

} // namespace uavloc::debug_viewer
