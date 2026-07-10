// VOModule implementation — owns every VO stage and the full state machine.
//
// The orchestration here was lifted verbatim (behaviour-preserving) from the
// former tests/test_vo_pipeline.cpp driver loop: NOT_INITIALIZED -> INITIALIZED
// via the Initializer (parallax accumulation), TRACKING via the Tracker, and
// LOST -> re-initialise with a global anchor weld so the trajectory stays
// continuous across map resets.

#include "uavloc/vo/vo_module.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <Eigen/LU>  // Matrix4d::inverse()
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <spdlog/spdlog.h>

#include "frame_queue.h"  // internal (src/vo): async frame buffer
#include "initializer.h"  // internal (src/vo): Chặng 1A local-map bootstrap
#include "local_map.h"
#include "local_mapper.h" // internal (src/vo): Local Mapping stage
#include "tracker.h"      // internal (src/vo): Chặng 1B tracking-vs-map

namespace uavloc::vo {

namespace {

// Degrees -> radians for the off-nadir angle (pure unit conversion, not tunable).
const double RAD_PER_DEG = M_PI / 180.0;

// Gimbal tilt is measured from the horizontal plane (~90° ⇒ nadir). Only trust
// the gimbal for the off-nadir angle inside this plausible range; outside it the
// value is missing/garbage and the airframe attitude is used instead.
const double GIMBAL_TILT_MIN_DEG = 10.0;
const double GIMBAL_TILT_MAX_DEG = 170.0;

// Lower bound on cos(off_nadir) so a near-90° tilt cannot blow the slant range up.
const double MIN_COS_OFF_NADIR = 1e-3;

// Compute the metres-per-unit translation scale from a frame's telemetry. For a
// near-nadir camera the slant range to the ground is d = altitude_m / cos(off_nadir).
// The off-nadir angle comes from the gimbal tilt (|90 - tilt|) when `source` is
// "gimbal" (or "auto" and the tilt is in range), otherwise from the airframe
// attitude (acos(cos(roll)·cos(pitch))). Returns -1 (keep monocular up-to-scale)
// when telemetry is absent / invalid.
double metricScaleFromTelemetry(const sensor::FrameData& frame,
                                const std::string&       source) {
    if (!frame.has_telemetry || frame.telemetry.altitude_m <= 0.0) {
        return -1.0;
    }
    const sensor::TelemetryData& tel = frame.telemetry;

    const bool tilt_in_range = tel.gimbal_tilt_deg >= GIMBAL_TILT_MIN_DEG &&
                               tel.gimbal_tilt_deg <= GIMBAL_TILT_MAX_DEG;
    const bool use_gimbal =
        (source == "gimbal" || source == "auto") && tilt_in_range;

    double cos_off_nadir;
    if (use_gimbal) {
        const double off_nadir = std::abs(90.0 - tel.gimbal_tilt_deg) * RAD_PER_DEG;
        cos_off_nadir = std::cos(off_nadir);
    } else {
        const double roll  = tel.roll_deg  * RAD_PER_DEG;
        const double pitch = tel.pitch_deg * RAD_PER_DEG;
        cos_off_nadir = std::cos(roll) * std::cos(pitch);
    }
    cos_off_nadir = std::max(cos_off_nadir, MIN_COS_OFF_NADIR);
    return tel.altitude_m / cos_off_nadir;
}

// Undistort the keypoint pixel coordinates of a FeatureSet in place so the
// homography (which uses the pinhole K) sees rectified pixels. No-op when the
// model carries zero distortion. Descriptors are untouched.
void undistortFeatures(const sensor::CameraModel& camera, FeatureSet& features) {
    if (features.keypoints.empty()) {
        return;
    }
    if (camera.distCoeffs().isZero(0.0)) {
        return;  // no lens distortion configured — undistortion is identity (no-op)
    }
    std::vector<cv::Point2f> pts;
    pts.reserve(features.keypoints.size());
    for (const cv::KeyPoint& kp : features.keypoints) {
        pts.push_back(kp.pt);
    }
    cv::Mat K_cv, D_cv;
    cv::eigen2cv(camera.K(), K_cv);
    cv::eigen2cv(camera.distCoeffs(), D_cv);
    std::vector<cv::Point2f> undist;
    cv::undistortPoints(pts, undist, K_cv, D_cv, cv::noArray(), K_cv);
    for (std::size_t i = 0; i < features.keypoints.size() && i < undist.size(); ++i) {
        features.keypoints[i].pt = undist[i];
    }
}

}  // namespace

// ── VOConfig ────────────────────────────────────────────────────────────────
VOConfig VOConfig::fromYaml(const YAML::Node& vo_node) {
    VOConfig cfg;
    cfg.detector  = FeatureDetectorConfig::fromYaml(vo_node);
    cfg.matcher   = ProjectionMatcherConfig::fromYaml(vo_node);
    // PoseEstimator: is an optional nested block; fromYaml falls back to defaults
    // when the node is missing (never throws).
    cfg.estimator = PoseEstimatorConfig::fromYaml(vo_node["PoseEstimator"]);
    cfg.reinit_refresh_frames =
        vo_node["reinit_refresh_frames"].as<int>(cfg.reinit_refresh_frames);
    cfg.off_nadir_source =
        vo_node["off_nadir_source"].as<std::string>(cfg.off_nadir_source);
    cfg.async_enabled =
        vo_node["async_enabled"].as<bool>(cfg.async_enabled);
    cfg.frame_queue_capacity =
        vo_node["frame_queue_capacity"].as<int>(cfg.frame_queue_capacity);
    cfg.frame_queue_pop_timeout_ms =
        vo_node["frame_queue_pop_timeout_ms"].as<int>(cfg.frame_queue_pop_timeout_ms);
    cfg.vo_node = vo_node;
    return cfg;
}

// ── Impl ──────────────────────────────────────────────────────────────────────
struct VOModule::Impl {
    VOConfig            config;
    sensor::CameraModel camera;  // owned copy so the stage references stay valid

    std::unique_ptr<IFeatureDetector> detector;
    std::unique_ptr<ProjectionMatcher> matcher;
    std::unique_ptr<PoseEstimator>     estimator;
    std::unique_ptr<Initializer>       initializer;  // Chặng 1A
    std::unique_ptr<LocalMapper>       mapper;       // Local Mapping stage
    std::unique_ptr<Tracker>           tracker;      // Chặng 1B (shares the LocalMap)

    // ── Async plumbing (unused in the default synchronous mode) ───────────────
    std::unique_ptr<FrameQueue>        frame_queue;
    std::thread                        tracking_thread;
    std::atomic<bool>                  running{false};
    VOModule::VOResultCallback         vo_result_cb;

    // State machine flags (mirrors the former PipelineContext).
    bool initialized       = false;  // NOT_INITIALIZED -> INITIALIZED
    bool init_ref_set      = false;  // first frame latched as the fixed reference
    int  reinit_attempts   = 0;
    bool just_reinitialised = false; // first tracked step after (re-)init

    // Accumulated camera->world pose IN THE CURRENT LOCAL MAP (identity at each
    // local-map start) and the global anchor that welds successive local maps:
    //   T_wc_global = T_world_anchor * T_wc.
    Eigen::Matrix4d T_wc            = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_world_anchor  = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_wc_global_prev = Eigen::Matrix4d::Identity();
    bool            have_prev_global = false;

    VOTrackingState state = VOTrackingState::NOT_INITIALIZED;

    VOModule::StatusCallback   status_cb;
    VOModule::KeyframeCallback keyframe_cb;

    Impl(const VOConfig& cfg, const sensor::CameraModel& cam)
        : config(cfg), camera(cam) {
        detector  = createFeatureDetector(config.detector);
        matcher   = std::make_unique<ProjectionMatcher>(config.matcher);
        estimator = std::make_unique<PoseEstimator>(config.estimator, camera);

        // Initializer / Tracker reuse the existing "VO:" keys (no new YAML keys).
        const InitializerConfig init_cfg = InitializerConfig::fromYaml(config.vo_node);
        initializer = std::make_unique<Initializer>(
            init_cfg, *matcher, *estimator, camera);

        // The LocalMapper grows the SAME LocalMap the Initializer seeds. It owns
        // its own ProjectionMatcher (built from the same config) so it never shares
        // matcher state with the Tracker across threads.
        const LocalMapperConfig mapper_cfg =
            LocalMapperConfig::fromYaml(config.vo_node);
        mapper = std::make_unique<LocalMapper>(
            mapper_cfg, config.matcher, camera, initializer->localMap());

        // The Tracker operates on the SAME LocalMap the Initializer seeds; that
        // map persists across reset() (cleared, not reallocated), so binding the
        // reference once here is safe across re-inits.
        const TrackerConfig tracker_cfg = TrackerConfig::fromYaml(config.vo_node);
        tracker = std::make_unique<Tracker>(
            tracker_cfg, *matcher, camera, initializer->localMap(), *mapper);

        spdlog::info("VOModule: feature detector backend: {}", detector->name());
    }

    ~Impl() { stopAsync(); }

    void startAsync();
    void stopAsync();
    void trackingLoop();

    void setState(VOTrackingState s, const std::string& msg) {
        if (s != state) {
            state = s;
            if (status_cb) {
                status_cb(s, msg);
            }
        }
    }

    int currentLandmarks() const {
        return initialized
                   ? static_cast<int>(initializer->localMap().numLandmarks())
                   : 0;
    }

    VOResult processFrame(const sensor::FrameData& frame);
};

VOResult VOModule::Impl::processFrame(const sensor::FrameData& frame) {
    VOResult res;
    res.frame_id       = static_cast<int>(frame.frame_id);
    res.timestamp_msec = frame.timestamp_msec;
    res.state          = state;

    // ── Feature extraction ────────────────────────────────────────────────────
    FeatureSet features;
    const FeatureStatus feat_status = detector->detect(frame, features);
    if (feat_status != FeatureStatus::OK || !features.hasDescriptors()) {
        spdlog::warn("Feature extraction failed on frame {} — status={}",
                     frame.frame_id, static_cast<int>(feat_status));
        res.num_landmarks = currentLandmarks();
        return res;  // has_pose = false
    }
    res.num_keypoints = static_cast<int>(features.keypoints.size());

    // Undistort keypoint coordinates before any geometry (no-op when the camera
    // carries zero distortion, as in the YenBai config).
    undistortFeatures(camera, features);

    // ── NOT_INITIALIZED -> INITIALIZED ────────────────────────────────────────
    if (!initialized) {
        const double init_scale =
            metricScaleFromTelemetry(frame, config.off_nadir_source);
        if (!init_ref_set) {
            // Latch the first frame as the FIXED reference and wait.
            initializer->reset(features, frame.telemetry);
            init_ref_set    = true;
            reinit_attempts = 0;
            setState(VOTrackingState::NOT_INITIALIZED, "holding initial reference");
            res.state = state;
            return res;
        }

        const InitStatus istat =
            initializer->tryInitialize(features, frame.telemetry, init_scale);
        if (istat != InitStatus::SUCCESS) {
            // NOT_READY / FAILED — keep accumulating, but slide the reference
            // forward if it has gone stale.
            if (++reinit_attempts >= config.reinit_refresh_frames) {
                initializer->reset(features, frame.telemetry);
                reinit_attempts = 0;
            }
            setState(VOTrackingState::NOT_INITIALIZED, "accumulating parallax");
            res.state = state;
            return res;
        }

        // SUCCESS: anchor the world at KF0 and place the camera at KF1.
        const double agl = (frame.has_telemetry ? frame.telemetry.altitude_m : 0.0);
        spdlog::info("Initialized at frame {}: {} landmarks, median depth "
                     "{:.1f} m (AGL ~{:.1f} m)",
                     frame.frame_id, initializer->numSeeded(),
                     initializer->medianSeedDepth(), agl);

        T_wc = initializer->T_prev_curr();  // KF1 camera->world pose
        const Eigen::Matrix4d T_wc_global = T_world_anchor * T_wc;

        // Hand the seeded local map to the Tracker (KF1 is the highest kf id).
        const uint64_t last_kf_id = initializer->localMap().numKeyframes() - 1;
        tracker->start(T_wc, last_kf_id);
        initialized        = true;
        reinit_attempts    = 0;
        just_reinitialised = true;  // do not chain a step across the new origin

        res.state         = VOTrackingState::INITIALIZED;
        res.T_wc          = T_wc_global;
        res.T_prev_curr   = Eigen::Matrix4d::Identity();
        res.has_pose      = true;
        res.is_keyframe   = true;
        res.num_matches   = initializer->numMatches();
        res.num_inliers   = initializer->numSeeded();
        res.inlier_ratio  = res.num_matches > 0
                                ? static_cast<double>(res.num_inliers) /
                                      static_cast<double>(res.num_matches)
                                : 0.0;
        res.num_landmarks = static_cast<int>(initializer->localMap().numLandmarks());

        T_wc_global_prev = T_wc_global;
        have_prev_global = true;

        setState(VOTrackingState::INITIALIZED, "local map seeded");
        if (keyframe_cb) {
            keyframe_cb(res);
        }
        return res;
    }

    // ── TRACKING vs local map ────────────────────────────────────────────────
    Eigen::Matrix4d T_wc_curr = T_wc;
    const TrackStatus tstat = tracker->track(
        features, frame.frame_id, frame.timestamp_msec, frame.telemetry, T_wc_curr);

    if (tstat == TrackStatus::LOST) {
        spdlog::warn("Tracking LOST at frame {} ({} prior inliers) — "
                     "re-initialising from this frame",
                     frame.frame_id, tracker->lastTrackedInliers());
        // Weld the running anchor with the last trusted GLOBAL pose BEFORE
        // dropping the map so the next local map continues seamlessly.
        T_world_anchor = T_world_anchor * T_wc;
        T_wc           = Eigen::Matrix4d::Identity();
        initializer->reset(features, frame.telemetry);
        initialized      = false;
        init_ref_set     = true;
        have_prev_global = false;

        res.state       = VOTrackingState::LOST;
        res.num_inliers = tracker->lastTrackedInliers();
        setState(VOTrackingState::LOST, "tracking lost; re-initialising");
        res.state = state;
        return res;  // has_pose = false
    }

    const bool is_kf = (tstat == TrackStatus::KEYFRAME_INSERTED);

    // GLOBAL pose + GLOBAL inter-frame increment. The increment is identity on the
    // first tracked step after a (re-)init so it contributes no spurious motion.
    const Eigen::Matrix4d T_wc_global = T_world_anchor * T_wc_curr;
    if (!just_reinitialised && have_prev_global) {
        res.T_prev_curr = T_wc_global * T_wc_global_prev.inverse();
    }
    just_reinitialised = false;

    T_wc             = T_wc_curr;
    T_wc_global_prev = T_wc_global;
    have_prev_global = true;

    res.state         = VOTrackingState::TRACKING;
    res.T_wc          = T_wc_global;
    res.has_pose      = true;
    res.is_keyframe   = is_kf;
    res.num_matches   = tracker->lastTrackedMatches();
    res.num_inliers   = tracker->lastTrackedInliers();
    res.inlier_ratio  = res.num_matches > 0
                            ? static_cast<double>(res.num_inliers) /
                                  static_cast<double>(res.num_matches)
                            : 0.0;
    res.num_landmarks = static_cast<int>(initializer->localMap().numLandmarks());
    // Read-only snapshot of the tracked-landmark pixel observations for the
    // debug-viewer overlay (does not alter VO behaviour).
    res.tracked_observations = tracker->lastObservations();

    setState(VOTrackingState::TRACKING, "tracking");
    if (is_kf && keyframe_cb) {
        keyframe_cb(res);
    }
    return res;
}

// ── Async orchestration ─────────────────────────────────────────────────────
void VOModule::Impl::startAsync() {
    if (running.load()) {
        return;  // already running
    }
    frame_queue = std::make_unique<FrameQueue>(
        config.frame_queue_capacity, config.frame_queue_pop_timeout_ms);
    mapper->start();       // Local Mapping thread
    tracker->setAsync(true);
    running.store(true);
    tracking_thread = std::thread([this] { trackingLoop(); });
    spdlog::info("VOModule: async mode started (queue capacity {}, pop timeout {} ms)",
                 config.frame_queue_capacity, config.frame_queue_pop_timeout_ms);
}

void VOModule::Impl::stopAsync() {
    if (!running.load()) {
        return;
    }
    running.store(false);
    if (frame_queue) {
        frame_queue->stop();  // unblock the Tracking thread's pop()
    }
    if (tracking_thread.joinable()) {
        tracking_thread.join();
    }
    mapper->stop();           // drain + join the Local Mapping thread
    tracker->setAsync(false);
    spdlog::info("VOModule: async mode stopped");
}

void VOModule::Impl::trackingLoop() {
    sensor::FrameData frame;
    while (running.load()) {
        if (!frame_queue->pop(frame)) {
            continue;  // timeout / stopped — re-check running
        }
        const VOResult r = processFrame(frame);
        if (vo_result_cb) {
            vo_result_cb(r);  // single ordered consumer, on this Tracking thread
        }
    }
}

// ── VOModule ──────────────────────────────────────────────────────────────────
VOModule::VOModule(const VOConfig& config, const sensor::CameraModel& camera)
    : impl_(std::make_unique<Impl>(config, camera)) {}

VOModule::~VOModule() = default;

VOResult VOModule::processFrame(const sensor::FrameData& frame) {
    return impl_->processFrame(frame);
}

void VOModule::start() { impl_->startAsync(); }

bool VOModule::pushFrame(const sensor::FrameData& frame) {
    if (!impl_->frame_queue) {
        return false;  // start() not called — nothing to enqueue into
    }
    return impl_->frame_queue->push(frame);
}

void VOModule::stop() { impl_->stopAsync(); }

VOTrackingState VOModule::state() const { return impl_->state; }

int VOModule::numLandmarks() const { return impl_->currentLandmarks(); }

void VOModule::setStatusCallback(StatusCallback cb) {
    impl_->status_cb = std::move(cb);
}

void VOModule::setKeyframeCallback(KeyframeCallback cb) {
    impl_->keyframe_cb = std::move(cb);
}

void VOModule::setVOResultCallback(VOResultCallback cb) {
    impl_->vo_result_cb = std::move(cb);
}

}  // namespace uavloc::vo
