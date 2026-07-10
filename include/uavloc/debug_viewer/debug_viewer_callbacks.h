#pragma once
#include <uavloc/debug_viewer/callback_slot.h>
#include <uavloc/debug_viewer/telemetry_record.h>
#include <string>

namespace uavloc::debug_viewer {

struct DebugViewerCallbacks {
    // Fired per telemetry record (from barcode decode or CSV)
    static CallbackSlot<void(const TelemetryRecord&)> on_telemetry;

    // Fired for status line updates (module name → status string)
    static CallbackSlot<void(const std::string&, const std::string&)> on_status;

    // Fired for named scalar metrics (timestamp, value, name)
    static CallbackSlot<void(double, float, const std::string&)> on_metric;
};

} // namespace uavloc::debug_viewer
