#pragma once

// LocalMapper — synchronous local-mapping step for the new_vo pipeline.
//
// Runs inline (no thread) right after a new keyframe is inserted. Mirrors the
// per-keyframe body of stella_vslam::mapping_module (store keyframe -> cull
// invalid landmarks -> triangulate new landmarks against covisibilities ->
// local bundle adjustment -> cull redundant keyframes), trimmed to pure VO
// (no BoW, no landmark fusion). See .docs/designs/vo_design_v3.md §6.

#include "uavloc/new_vo/vo_config.h"

#include <functional>
#include <memory>
#include <unordered_map>

namespace uavloc {
namespace vo {

namespace data {
class Keyframe;
class Landmark;
class MapDatabase;
} // namespace data

namespace module {
class LocalMapCleaner;
} // namespace module

namespace optimize {
class LocalBundleAdjuster;
} // namespace optimize

namespace module {

class LocalMapper {
public:
    //! Landmarks replaced during fusion: old landmark -> surviving landmark.
    using ReplacedLandmarks =
        std::unordered_map<std::shared_ptr<data::Landmark>, std::shared_ptr<data::Landmark>>;

    LocalMapper() = delete;

    LocalMapper(data::MapDatabase* map_db, const VOConfig& config);

    ~LocalMapper();

    //! Extend the map with a freshly inserted keyframe (synchronous).
    //! Landmarks merged away during fusion are reported in replaced_lms so the
    //! caller can remap stale references (e.g. in the tracker's last frame).
    //! force_stop_flag (may be null) aborts the local bundle adjustment early.
    //! skip_local_ba (async backpressure): skip local BA when the mapping queue
    //! is backed up so it can drain. abort_landmark_gen (async backpressure): a
    //! predicate polled between covisibility pairs to interrupt landmark
    //! generation when a newer keyframe is queued. Both default to no-ops so the
    //! synchronous baseline is bit-for-bit unchanged.
    void map(const std::shared_ptr<data::Keyframe>& cur_keyfrm,
             ReplacedLandmarks& replaced_lms,
             bool* force_stop_flag = nullptr,
             bool skip_local_ba = false,
             std::function<bool()> abort_landmark_gen = {});

    //! Drop all internal per-keyframe scratch state (the LocalMapCleaner's
    //! pending "fresh landmark" queue). Must only be called while no map() is in
    //! flight. Used on a map reset (LOST -> re-init) so the cleaner never
    //! dereferences landmarks/keyframes that MapDatabase::clear() has erased.
    void reset();

private:
    //! Register the keyframe: queue its landmarks for redundancy checks, update
    //! covisibility connections, and add it to the map database.
    void store_new_keyframe(const std::shared_ptr<data::Keyframe>& cur_keyfrm);

    //! Triangulate new landmarks between the keyframe and each covisibility.
    //! abort_landmark_gen (may be empty) is polled between covisibility pairs to
    //! interrupt generation when a newer keyframe is queued (async backpressure).
    void create_new_landmarks(const std::shared_ptr<data::Keyframe>& cur_keyfrm,
                              const std::function<bool()>& abort_landmark_gen);

    //! Merge duplicate landmarks between the keyframe and its covisibilities.
    void fuse_landmark_duplication(const std::shared_ptr<data::Keyframe>& cur_keyfrm,
                                   ReplacedLandmarks& replaced_lms);

    void triangulate_with_two_keyframes(const std::shared_ptr<data::Keyframe>& keyfrm_1,
                                        const std::shared_ptr<data::Keyframe>& keyfrm_2,
                                        const std::vector<std::pair<unsigned int, unsigned int>>& matches);

    data::MapDatabase* map_db_ = nullptr;

    std::unique_ptr<module::LocalMapCleaner>        local_map_cleaner_;
    std::unique_ptr<optimize::LocalBundleAdjuster>  local_bundle_adjuster_;

    //! Erase temporal keyframes older than num_temporal_keyframes_ keyframes
    //! (mirrors stella mapping_module erase_temporal_keyframes /
    //! num_temporal_keyframes; bounded-VO sliding window).
    const bool         erase_temporal_keyframes_;
    const unsigned int num_temporal_keyframes_;

    const unsigned int num_covisibilities_for_landmark_generation_;
    const unsigned int num_covisibilities_for_landmark_fusion_;
    const float        parallax_deg_thr_;
    const float        residual_rad_thr_;
    const double       baseline_dist_thr_ratio_;

    //! Env-gated (UAVLOC_VO_DIAG) diagnostic logging switch, read once at ctor.
    const bool         diag_enabled_;
};

} // namespace module
} // namespace vo
} // namespace uavloc
