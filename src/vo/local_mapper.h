#pragma once

// LocalMapper — the map-growing half of the Tracking / Local-Mapping split.
//
// The Tracker (Tracking thread) decides WHEN a keyframe is inserted, allocates
// its id, and advances its own motion state immediately. The heavy map work —
// attaching landmark observations, triangulating new landmarks against the
// reference keyframe, inserting the keyframe, and culling stale landmarks — is
// packaged into a KeyframePacket and handed to the LocalMapper.
//
// Two execution modes share the exact same processKeyframe() code path:
//   - Synchronous (default): submitKeyframe() runs processKeyframe() inline on
//     the calling (Tracking) thread, so the pipeline stays deterministic and
//     bit-for-bit identical to the pre-refactor single-thread implementation.
//   - Asynchronous (start()/stop()): a dedicated Local Mapping thread drains a
//     FIFO keyframe queue, so heavy mapping never blocks per-frame tracking.
//     Async mode is intentionally NOT deterministic (see the multi-thread design).
//
// SE3 convention (CLAUDE.md): T_wc is camera->world; T_cw = T_wc.inverse().

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "uavloc/sensor/camera_model.h"
#include "uavloc/sensor/telemetry_data.h"
#include "uavloc/vo/projection_matcher.h"
#include "uavloc/vo/vo_data.h"  // FeatureSet

#include "local_map.h"

namespace uavloc::vo {

// Tunable parameters for the LocalMapper. Reuses the existing "VO:" YAML keys.
struct LocalMapperConfig {
    // Max per-point reprojection error (px) for a newly triangulated landmark to
    // be kept. YAML: VO.max_reproj_error_px.
    double max_reproj_error_px = 4.0;

    // Build from the mission "VO:" node; missing keys fall back to defaults.
    static LocalMapperConfig fromYaml(const YAML::Node& vo_node);
};

// Everything the Local Mapping stage needs to promote one tracked frame into a
// keyframe and grow the map. Produced by the Tracker at keyframe time; all data
// is copied by value so the packet is self-contained across a thread boundary.
struct KeyframePacket {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t kf_id          = 0;   // pre-allocated keyframe id (LocalMap::nextKeyframeId)
    uint64_t ref_kf_id      = 0;   // reference keyframe for triangulation (old last_kf)
    uint64_t frame_id       = 0;   // originating FrameData::frame_id
    double   timestamp_msec = 0.0;

    Eigen::Matrix4d       T_wc = Eigen::Matrix4d::Identity();  // camera->world
    FeatureSet            features;                             // current frame features
    sensor::TelemetryData telemetry;

    int    num_inliers  = 0;   // tracked (PnP/BA) inliers this frame
    double inlier_ratio = 0.0;

    // Per-correspondence arrays (parallel): the tracked landmark id, the current
    // keypoint index it matched, and whether it survived as a PnP/BA inlier.
    std::vector<uint64_t> match_lm_ids;
    std::vector<int>      match_curr_kp;
    std::vector<char>     is_inlier;  // 1 = inlier (std::vector<bool> avoided)
};

// Grows a shared LocalMap from KeyframePackets. Owns its own ProjectionMatcher
// (deterministic given the config) so it never shares matcher state with the
// Tracker across threads.
class LocalMapper {
public:
    LocalMapper(const LocalMapperConfig&       config,
                const ProjectionMatcherConfig& matcher_config,
                const sensor::CameraModel&     camera,
                LocalMap&                      local_map);
    ~LocalMapper();

    LocalMapper(const LocalMapper&)            = delete;
    LocalMapper& operator=(const LocalMapper&) = delete;

    // Hand a keyframe to the mapper. In async mode (thread running) the packet is
    // enqueued and processed on the Local Mapping thread; otherwise it is
    // processed inline on the caller's thread (synchronous, deterministic).
    void submitKeyframe(KeyframePacket packet);

    // Start / stop the Local Mapping thread (async mode). stop() drains any
    // pending packets so the map is complete before returning.
    void start();
    void stop();

    bool isRunning() const { return running_.load(); }

private:
    // The full keyframe-insertion body: attach observations, triangulate new
    // landmarks against the reference keyframe, insert the keyframe, cull stale
    // landmarks. Runs inline (sync) or on the Local Mapping thread (async).
    void processKeyframe(const KeyframePacket& pkt);

    // Triangulate landmarks for current keypoints not yet tied to a landmark, by
    // matching the reference keyframe against the current frame. Returns the
    // number of new landmarks created.
    int triangulateNewLandmarks(const Keyframe&        ref_kf,
                                const FeatureSet&      curr,
                                Keyframe&              new_kf,
                                const Eigen::Matrix4d& T_wc_curr);

    // Mark as BAD any landmark whose found/visible ratio has fallen below the
    // map-hygiene threshold. Returns the number culled.
    int cullLandmarks();

    // Local Mapping thread loop (async).
    void run();

    LocalMapperConfig          config_;
    ProjectionMatcher          matcher_;   // owned; not shared with the Tracker
    const sensor::CameraModel& camera_;
    LocalMap&                  local_map_;

    // Async plumbing.
    std::deque<KeyframePacket> queue_;
    std::mutex                 queue_mutex_;
    std::condition_variable    queue_cv_;
    std::thread                thread_;
    std::atomic<bool>          running_{false};
    bool                       stop_flag_ = false;  // guarded by queue_mutex_
};

}  // namespace uavloc::vo
