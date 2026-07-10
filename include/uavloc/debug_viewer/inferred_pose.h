#pragma once

namespace uavloc::debug_viewer {

// A single metric pose sample already expressed in the viewer's display frame
// (local ENU, metres). Unlike TelemetryRecord, this carries NO GPS — the point
// is fed straight into the polyline without a GPS->ENU round-trip. Used by the
// metric pushPose()/pushGroundtruthPose() API so externally-computed
// trajectories (e.g. VO estimates) can be overlaid directly.
struct InferredPose {
    int    frame_id = 0;
    double x = 0.0, y = 0.0, z = 0.0;                 // E, N, U (metres)
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;  // optional, for gizmos
};

} // namespace uavloc::debug_viewer
