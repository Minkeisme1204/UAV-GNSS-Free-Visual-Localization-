#pragma once

// ============================================================
// VoFrameLoader — sensor→Frame bridge for the new_vo pipeline.
//
// Turns a raw sensor::FrameData (image + metadata) into a data::Frame, the
// entry object of the new_vo pipeline. In stella_vslam this glue lived in
// system::create_monocular_frame(); that class was NOT ported, so this small
// bridge reuses the already-ported feature/data functions to reproduce the
// exact same wiring (ORB extract → undistort → bearings → grid → Frame ctor).
//
// SCOPE: this class ONLY builds the Frame. It performs no initialisation,
// tracking, or Keyframe decisions — those belong to the orchestrator
// (see .docs/designs/vo_design_v3.md). It reuses existing functions only; no
// new algorithm and no state machine.
//
// TELEMETRY: data::Frame (faithful stella port) has NO telemetry field, so the
// loader intentionally does NOT attach sensor::TelemetryData to the Frame. A
// higher layer keeps telemetry in parallel, keyed by frame_id. Do not widen
// data::Frame here.
//
// BUILT: this file is part of the enabled new_vo subset (feature/orb + data/ +
// camera adapter). The camera seam is a real implementation
// (camera::PerspectiveCamera) and the ORB PascalCase naming is reconciled.
// See .docs/designs/new_vo_port_notes.md.
// ============================================================

#include "uavloc/sensor/frame_data.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/frame_observation.h"
#include "uavloc/new_vo/data/common.h"
#include "uavloc/new_vo/feature/orb/orb_extractor.h"
#include "uavloc/new_vo/feature/orb/orb_params.h"
#include "uavloc/new_vo/camera/base.h"

#include <vector>

#include <opencv2/opencv.hpp>

namespace uavloc {
namespace vo {

//! Builds a data::Frame from a raw sensor::FrameData, reusing the ported
//! ORB extractor + camera geometry + keypoint-grid helpers.
class VoFrameLoader {
public:
    VoFrameLoader() = delete;

    //! Constructor.
    //! @param camera        borrowed camera model (shared with keyframes/map)
    //! @param OrbParams    borrowed ORB scale-pyramid params (passed to Frame ctor)
    //! @param min_area      ORB min node area ("min_size" in stella; default 800)
    //! @param desc_type     descriptor backend (ORB by default)
    //! @param mask_rects    optional feature mask rectangles
    //! @param num_grid_cols keypoint-grid columns (stella preprocessing default 64)
    //! @param num_grid_rows keypoint-grid rows (stella preprocessing default 48)
    VoFrameLoader(camera::Base* camera,
                    feature::OrbParams* orb_params,
                    unsigned int min_area = 800,
                    feature::DescriptorType desc_type = feature::DescriptorType::ORB,
                    const std::vector<std::vector<float>>& mask_rects = {},
                    unsigned int num_grid_cols = 64,
                    unsigned int num_grid_rows = 48);

    //! Build a monocular data::Frame from a raw sensor Frame.
    //! Mirrors stella system::create_monocular_frame (fiducial detection dropped).
    data::Frame load(const sensor::FrameData& fd);

private:
    //! ORB feature extractor — owned, constructed once (avoids per-frame alloc).
    feature::OrbExtractor orb_extractor_;

    //! Camera model — borrowed (concrete camera::PerspectiveCamera at the call site).
    camera::Base* camera_ = nullptr;

    //! ORB scale-pyramid params — borrowed (handed to the data::Frame ctor).
    feature::OrbParams* orb_params_ = nullptr;

    //! keypoint-grid dimensions used to accelerate projection matching.
    unsigned int num_grid_cols_;
    unsigned int num_grid_rows_;
};

} // namespace vo
} // namespace uavloc
