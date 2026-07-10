#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include "uavloc/sensor/frame_data.h"
#include "uavloc/vo/vo_data.h"

#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace uavloc::vo {

// Selects the concrete detector built by createFeatureDetector().
enum class FeatureBackend {
    ORB_OPENCV,   // default: keypoints + binary descriptors (matching-ready)
    FAST_ONLY     // lighter: keypoints only (no descriptor) for KLT/point tracking
};

// Tunable detector parameters. All fields have Jetson-oriented defaults so the
// detector can be constructed with FeatureDetectorConfig{} and overridden from
// YAML. No magic numbers should appear in the implementation logic.
struct FeatureDetectorConfig {
    FeatureBackend backend = FeatureBackend::ORB_OPENCV;
    int   max_keypoints   = 1000;   // Jetson phase-1: 500..1000
    int   min_keypoints   = 80;     // below this -> NOT_ENOUGH_FEATURES
    int   pyramid_levels  = 8;      // Jetson: 4..8
    float scale_factor    = 1.2f;
    int   fast_threshold  = 20;     // Jetson: 15..30
    int   min_fast_threshold = 7;   // fallback for sparse images
    int   edge_threshold  = 31;
    int   wta_k           = 2;
    int   patch_size      = 31;
    bool  use_grid        = true;   // distribute keypoints across a grid
    int   grid_cols       = 0;      // 0 = auto
    int   grid_rows       = 0;

    // Build a config from a YAML node; missing keys fall back to defaults.
    static FeatureDetectorConfig fromYaml(const YAML::Node& node);
};

// Abstract feature-detection interface. Concrete backends live in src/vo/.
//
// detect() takes the source frame by const reference and writes into the caller
// supplied FeatureSet. Implementations must not copy or clone the image header
// and must not retain it beyond the call.
class IFeatureDetector {
public:
    virtual ~IFeatureDetector() = default;

    // Detect keypoints (and descriptors, depending on backend) for the frame.
    // The result is written into output; the returned status mirrors
    // output.status for convenient call-site branching.
    virtual FeatureStatus detect(const sensor::FrameData& frame, FeatureSet& output) = 0;

    // The configuration this detector was built with.
    virtual const FeatureDetectorConfig& config() const = 0;

    // Human-readable backend name for logging.
    virtual std::string name() const = 0;
};

// Builds the concrete detector selected by config.backend.
std::unique_ptr<IFeatureDetector> createFeatureDetector(const FeatureDetectorConfig& config);

}  // namespace uavloc::vo

#endif // FEATURE_DETECTOR_H
