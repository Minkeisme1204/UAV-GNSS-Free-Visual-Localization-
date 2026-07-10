#include "local_map.h"

#include <spdlog/spdlog.h>

namespace uavloc::vo {

void LocalMap::addKeyframe(const std::shared_ptr<Keyframe>& kf) {
    if (!kf) {
        spdlog::warn("LocalMap::addKeyframe — ignoring null keyframe");
        return;
    }
    std::lock_guard<std::mutex> lk(map_mutex_);
    keyframes_[kf->id] = kf;
}

void LocalMap::addLandmark(const std::shared_ptr<Landmark>& lm) {
    if (!lm) {
        spdlog::warn("LocalMap::addLandmark — ignoring null landmark");
        return;
    }
    std::lock_guard<std::mutex> lk(map_mutex_);
    landmarks_[lm->id] = lm;
}

std::shared_ptr<Keyframe> LocalMap::getKeyframe(uint64_t id) const {
    std::lock_guard<std::mutex> lk(map_mutex_);
    auto it = keyframes_.find(id);
    return (it != keyframes_.end()) ? it->second : nullptr;
}

std::shared_ptr<Landmark> LocalMap::getLandmark(uint64_t id) const {
    std::lock_guard<std::mutex> lk(map_mutex_);
    auto it = landmarks_.find(id);
    return (it != landmarks_.end()) ? it->second : nullptr;
}

std::vector<LandmarkSnapshot> LocalMap::snapshotLandmarks() const {
    std::lock_guard<std::mutex> lk(map_mutex_);
    std::vector<LandmarkSnapshot> out;
    out.reserve(landmarks_.size());
    for (const auto& kv : landmarks_) {  // native container order (deterministic)
        const std::shared_ptr<Landmark>& lm = kv.second;
        if (!lm) {
            continue;
        }
        LandmarkSnapshot s;
        s.lm         = lm;
        s.id         = lm->id;
        s.pos_w      = lm->pos_w;
        s.descriptor = lm->descriptor;  // shallow ref; ORB descriptor is immutable
        s.is_bad     = lm->isBad();
        out.push_back(std::move(s));
    }
    return out;
}

size_t LocalMap::numKeyframes() const {
    std::lock_guard<std::mutex> lk(map_mutex_);
    return keyframes_.size();
}

size_t LocalMap::numLandmarks() const {
    std::lock_guard<std::mutex> lk(map_mutex_);
    return landmarks_.size();
}

void LocalMap::rescale(double s) {
    if (s <= 0.0 || s == 1.0) {
        return;  // non-positive or no-op scale: nothing to do
    }
    std::lock_guard<std::mutex> lk(map_mutex_);
    for (auto& kv : landmarks_) {
        if (kv.second) {
            kv.second->pos_w *= s;  // scale ALL landmarks (incl. BAD) for consistency
        }
    }
    for (auto& kv : keyframes_) {
        if (kv.second) {
            kv.second->T_wc.block<3, 1>(0, 3) *= s;  // translation only; R untouched
        }
    }
}

void LocalMap::clear() {
    std::lock_guard<std::mutex> lk(map_mutex_);
    keyframes_.clear();
    landmarks_.clear();
    next_keyframe_id_.store(0);
    next_landmark_id_.store(0);
}

}  // namespace uavloc::vo
