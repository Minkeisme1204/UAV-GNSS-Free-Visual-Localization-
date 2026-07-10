#include "uavloc/vo/feature_detector.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <spdlog/spdlog.h>

namespace uavloc::vo {

// ---------------------------------------------------------------------------
// FeatureSet
// ---------------------------------------------------------------------------
void FeatureSet::clear() {
    keypoints.clear();
    descriptors.release();
    descriptor_type    = DescriptorType::NONE;
    status             = FeatureStatus::OK;
    processing_time_ms = 0.0;
    meta               = FeatureFrameMeta{};
}

// ---------------------------------------------------------------------------
// FeatureDetectorConfig
// ---------------------------------------------------------------------------
FeatureDetectorConfig FeatureDetectorConfig::fromYaml(const YAML::Node& node) {
    FeatureDetectorConfig cfg;  // start from defaults

    if (!node) {
        return cfg;
    }

    // backend is a case-insensitive string; default keeps cfg.backend.
    if (node["backend"]) {
        std::string backend_str = node["backend"].as<std::string>("");
        std::transform(backend_str.begin(), backend_str.end(), backend_str.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        if (backend_str == "FAST_ONLY") {
            cfg.backend = FeatureBackend::FAST_ONLY;
        } else if (backend_str == "ORB_OPENCV") {
            cfg.backend = FeatureBackend::ORB_OPENCV;
        }
        // unknown values silently retain the default backend
    }

    // Only the high-level knobs are read from YAML; the ORB-internal tuning
    // parameters (scale_factor, pyramid_levels, fast_threshold,
    // min_fast_threshold, edge_threshold, wta_k, patch_size) stay hardcoded at
    // their struct defaults so there are fewer params to set by hand.
    cfg.max_keypoints = node["max_keypoints"].as<int>(cfg.max_keypoints);
    cfg.min_keypoints = node["min_keypoints"].as<int>(cfg.min_keypoints);
    cfg.use_grid      = node["use_grid"].as<bool>(cfg.use_grid);
    cfg.grid_cols     = node["grid_cols"].as<int>(cfg.grid_cols);
    cfg.grid_rows     = node["grid_rows"].as<int>(cfg.grid_rows);

    return cfg;
}

namespace {

// Target side length (in pixels) of one auto-grid cell when grid_cols/grid_rows
// are left at 0. Picked so a 1080p frame yields roughly a 16x9 grid.
constexpr int AUTO_GRID_CELL_PX = 120;

// Convert the source frame's image to a single-channel view usable by OpenCV
// detectors. Single-channel images are returned as-is (no copy); 3-channel
// images are converted into the caller-provided scratch Mat.
const cv::Mat& asGray(const cv::Mat& src, cv::Mat& gray_scratch) {
    if (src.channels() == 3) {
        cv::cvtColor(src, gray_scratch, cv::COLOR_BGR2GRAY);
        return gray_scratch;
    }
    return src;
}

// Distribute keypoints across a grid_cols x grid_rows grid, retaining the
// strongest (highest response) keypoints per cell so the survivors are spread
// across the frame and the total is capped near max_keypoints.
//
// When grid_cols/grid_rows are 0, a grid is auto-derived from the image size
// and AUTO_GRID_CELL_PX. This must run before ORB descriptor computation so
// descriptor rows stay aligned with the retained keypoints.
void distributeOnGrid(std::vector<cv::KeyPoint>& keypoints,
                      const cv::Size& image_size,
                      const FeatureDetectorConfig& cfg) {
    if (keypoints.empty()) {
        return;
    }

    int cols = cfg.grid_cols;
    int rows = cfg.grid_rows;
    if (cols <= 0) {
        cols = std::max(1, image_size.width / AUTO_GRID_CELL_PX);
    }
    if (rows <= 0) {
        rows = std::max(1, image_size.height / AUTO_GRID_CELL_PX);
    }

    const int num_cells = cols * rows;
    if (num_cells <= 1) {
        return;  // nothing to distribute over
    }

    const float cell_w = static_cast<float>(image_size.width)  / static_cast<float>(cols);
    const float cell_h = static_cast<float>(image_size.height) / static_cast<float>(rows);

    // Per-cell cap so the grand total stays near max_keypoints; at least one.
    const int per_cell_cap = std::max(1, cfg.max_keypoints / num_cells);

    // Bucket keypoints by cell index.
    std::unordered_map<int, std::vector<cv::KeyPoint>> buckets;
    buckets.reserve(static_cast<size_t>(num_cells));
    for (const cv::KeyPoint& kp : keypoints) {
        int cx = static_cast<int>(kp.pt.x / cell_w);
        int cy = static_cast<int>(kp.pt.y / cell_h);
        cx = std::min(std::max(cx, 0), cols - 1);
        cy = std::min(std::max(cy, 0), rows - 1);
        buckets[cy * cols + cx].push_back(kp);
    }

    std::vector<cv::KeyPoint> kept;
    kept.reserve(keypoints.size());
    for (auto& entry : buckets) {
        std::vector<cv::KeyPoint>& cell = entry.second;
        if (static_cast<int>(cell.size()) > per_cell_cap) {
            std::nth_element(cell.begin(), cell.begin() + per_cell_cap, cell.end(),
                             [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                                 return a.response > b.response;
                             });
            cell.resize(static_cast<size_t>(per_cell_cap));
        }
        kept.insert(kept.end(), cell.begin(), cell.end());
    }

    keypoints.swap(kept);
}

double elapsedMs(const std::chrono::steady_clock::time_point& start) {
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// Shared front-half of detect(): clear, stamp meta, validate, grayscale.
// Returns true when detection should proceed; sets output.status and returns
// false on early exits (empty image).
bool prepareDetect(const sensor::FrameData& frame, FeatureSet& output,
                   cv::Mat& gray_out) {
    output.clear();
    output.meta = FeatureFrameMeta{frame.frame_id, frame.timestamp_msec};

    if (!frame.HasImage()) {
        output.status = FeatureStatus::EMPTY_IMAGE;
        spdlog::warn("FeatureDetector: empty image for frame {}", frame.frame_id);
        return false;
    }

    gray_out = asGray(frame.image, gray_out);
    return true;
}

// Apply the min-keypoint threshold and finalise status; keypoints already set.
void finalizeStatus(FeatureSet& output, const FeatureDetectorConfig& cfg) {
    if (output.keypoints.size() < static_cast<size_t>(cfg.min_keypoints)) {
        output.status = FeatureStatus::NOT_ENOUGH_FEATURES;
        spdlog::warn("FeatureDetector: frame {} only {} keypoints (min {})",
                     output.meta.frame_id, output.keypoints.size(), cfg.min_keypoints);
    } else {
        output.status = FeatureStatus::OK;
    }
}

// ---------------------------------------------------------------------------
// OrbDetector — keypoints + binary descriptors
// ---------------------------------------------------------------------------
class OrbDetector : public IFeatureDetector {
public:
    explicit OrbDetector(const FeatureDetectorConfig& config)
        : config_(config) {
        orb_ = cv::ORB::create(
            config_.max_keypoints,
            config_.scale_factor,
            config_.pyramid_levels,
            config_.edge_threshold,
            /*firstLevel=*/0,
            config_.wta_k,
            cv::ORB::HARRIS_SCORE,
            config_.patch_size,
            config_.fast_threshold);
    }

    FeatureStatus detect(const sensor::FrameData& frame, FeatureSet& output) override {
        const auto t_start = std::chrono::steady_clock::now();

        cv::Mat gray;
        if (!prepareDetect(frame, output, gray)) {
            output.processing_time_ms = elapsedMs(t_start);
            return output.status;
        }

        if (config_.use_grid) {
            // Detect, grid-prune, then compute descriptors on survivors so the
            // descriptor rows stay aligned with the retained keypoints.
            orb_->detect(gray, output.keypoints);
            distributeOnGrid(output.keypoints, gray.size(), config_);
            orb_->compute(gray, output.keypoints, output.descriptors);
        } else {
            orb_->detectAndCompute(gray, cv::noArray(), output.keypoints, output.descriptors);
        }
        output.descriptor_type = DescriptorType::ORB;

        finalizeStatus(output, config_);
        output.processing_time_ms = elapsedMs(t_start);
        return output.status;
    }

    const FeatureDetectorConfig& config() const override { return config_; }
    std::string name() const override { return "ORB_OPENCV"; }

private:
    FeatureDetectorConfig config_;
    cv::Ptr<cv::ORB> orb_;
};

// ---------------------------------------------------------------------------
// FastDetector — keypoints only, no descriptors
// ---------------------------------------------------------------------------
class FastDetector : public IFeatureDetector {
public:
    explicit FastDetector(const FeatureDetectorConfig& config)
        : config_(config) {
        fast_ = cv::FastFeatureDetector::create(config_.fast_threshold);
    }

    FeatureStatus detect(const sensor::FrameData& frame, FeatureSet& output) override {
        const auto t_start = std::chrono::steady_clock::now();

        cv::Mat gray;
        if (!prepareDetect(frame, output, gray)) {
            output.processing_time_ms = elapsedMs(t_start);
            return output.status;
        }

        fast_->detect(gray, output.keypoints);
        if (config_.use_grid) {
            distributeOnGrid(output.keypoints, gray.size(), config_);
        }
        output.descriptor_type = DescriptorType::NONE;  // keypoints only

        finalizeStatus(output, config_);
        output.processing_time_ms = elapsedMs(t_start);
        return output.status;
    }

    const FeatureDetectorConfig& config() const override { return config_; }
    std::string name() const override { return "FAST_ONLY"; }

private:
    FeatureDetectorConfig config_;
    cv::Ptr<cv::FastFeatureDetector> fast_;
};

}  // namespace

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
std::unique_ptr<IFeatureDetector> createFeatureDetector(const FeatureDetectorConfig& config) {
    switch (config.backend) {
        case FeatureBackend::FAST_ONLY:
            spdlog::info("createFeatureDetector: backend=FAST_ONLY (keypoints only)");
            return std::make_unique<FastDetector>(config);
        case FeatureBackend::ORB_OPENCV:
        default:
            spdlog::info("createFeatureDetector: backend=ORB_OPENCV (keypoints + descriptors)");
            return std::make_unique<OrbDetector>(config);
    }
}

}  // namespace uavloc::vo
