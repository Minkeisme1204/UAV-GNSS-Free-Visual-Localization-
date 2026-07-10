#pragma once

// LocalMap — the sole owner of the VO local map's keyframes and landmarks.
//
// Ownership is exclusive via std::shared_ptr stored in two id-keyed maps; ids
// are handed out by monotonically increasing counters. Keyframe <-> Landmark
// links are resolved by id through getKeyframe() / getLandmark(), never by
// raw or cross pointers (see vo_keyframe_landmark_design.md §C.2).
//
// Thread-safety (Phase A of the Tracking / Local-Mapping split): every method
// that touches the container state takes map_mutex_, so the Tracking thread
// (reads) and the Local Mapping thread (mutations) never touch the maps
// concurrently. The Tracking hot path must NOT iterate the live containers;
// it takes a value snapshot via snapshotLandmarks() and works on the copy.

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "local_map_types.h"

namespace uavloc::vo {

// A value snapshot of one landmark for the Tracking thread. The fields the
// visibility / PnP path needs are copied under the map lock so tracking never
// dereferences the live containers, while `lm` keeps the owning shared_ptr so
// the per-frame visibility counter can still be bumped atomically.
struct LandmarkSnapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::shared_ptr<Landmark> lm;                       // live object (atomic counters)
    uint64_t                  id     = 0;
    Eigen::Vector3d           pos_w  = Eigen::Vector3d::Zero();
    cv::Mat                   descriptor;               // shallow ref (immutable after creation)
    bool                      is_bad = false;
};

class LocalMap {
public:
    using KeyframeMap = std::unordered_map<uint64_t, std::shared_ptr<Keyframe>>;
    using LandmarkMap = std::unordered_map<uint64_t, std::shared_ptr<Landmark>>;

    LocalMap() = default;

    // Monotonic id allocators. Call these to stamp a Keyframe / Landmark id
    // before adding it, so the cross-links agree with the stored id. Atomic so
    // the Tracking thread (keyframe ids) and Local Mapping thread (landmark ids)
    // can allocate without the map lock.
    uint64_t nextKeyframeId() { return next_keyframe_id_++; }
    uint64_t nextLandmarkId() { return next_landmark_id_++; }

    // Insert. The shared_ptr's ->id must already be set (typically via the
    // next*Id() allocators above).
    void addKeyframe(const std::shared_ptr<Keyframe>& kf);
    void addLandmark(const std::shared_ptr<Landmark>& lm);

    // Lookup by id; returns nullptr when absent.
    std::shared_ptr<Keyframe> getKeyframe(uint64_t id) const;
    std::shared_ptr<Landmark> getLandmark(uint64_t id) const;

    // Value snapshot of all landmarks in native container order, taken under the
    // map lock. The Tracking thread iterates the returned vector instead of the
    // live map so a concurrent Local-Mapping mutation cannot invalidate iterators
    // or rehash mid-iteration. Order is deterministic (landmarks are never erased,
    // only marked BAD), so the sync pipeline is bit-for-bit unchanged.
    std::vector<LandmarkSnapshot> snapshotLandmarks() const;

    // NOTE: these expose the live containers by reference and are therefore NOT
    // thread-safe to iterate while the Local Mapping thread may mutate the map —
    // use snapshotLandmarks() from the Tracking thread. Kept for single-threaded
    // / internal Local-Mapping-thread use.
    const KeyframeMap& keyframes() const { return keyframes_; }
    const LandmarkMap& landmarks() const { return landmarks_; }

    size_t numKeyframes() const;
    size_t numLandmarks() const;

    // Re-anchor the metric scale of the whole local map by a similarity scalar s
    // around the world origin (KF0): every landmark position and every keyframe
    // translation is multiplied by s (rotations untouched). Reprojection is
    // invariant under this uniform scaling, so the inlier set is preserved while
    // the absolute scale is pinned to the altitude prior (Fix #1, see
    // vo_lowparallax_fix_design.md §B.1 Phương án A). BAD landmarks are scaled too
    // for consistency.
    void rescale(double s);

    void clear();

private:
    KeyframeMap keyframes_;
    LandmarkMap landmarks_;

    // Guards keyframes_ / landmarks_ across the Tracking and Local Mapping threads.
    mutable std::mutex map_mutex_;

    std::atomic<uint64_t> next_keyframe_id_{0};
    std::atomic<uint64_t> next_landmark_id_{0};
};

}  // namespace uavloc::vo
