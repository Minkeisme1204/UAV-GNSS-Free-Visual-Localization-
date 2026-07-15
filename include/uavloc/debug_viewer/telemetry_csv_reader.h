#pragma once

#include <uavloc/debug_viewer/telemetry_record.h>
#include <string>
#include <vector>

namespace uavloc::debug_viewer {

std::vector<TelemetryRecord> load_telemetry_csv(const std::string& path);

} // namespace uavloc::debug_viewer
