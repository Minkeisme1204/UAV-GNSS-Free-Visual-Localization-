#include "uavloc/new_vo/vo_module.h"

#include "uavloc/new_vo/camera/perspective_camera.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/data/map_database.h"
#include "uavloc/new_vo/feature/orb/orb_params.h"
#include "uavloc/new_vo/match/projection.h"
#include "uavloc/new_vo/module/frame_tracker.h"
#include "uavloc/new_vo/module/initializer.h"
#include "uavloc/new_vo/module/keyframe_inserter.h"
#include "uavloc/new_vo/module/local_map_updater.h"
#include "uavloc/new_vo/module/local_mapper.h"
#include "uavloc/new_vo/module/mapping_module.h"
#include "uavloc/new_vo/module/vo_frame_loader.h"
#include "uavloc/new_vo/optimize/pose_optimizer.h"
#include "uavloc/new_vo/optimize/pose_optimizer_factory.h"

#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {

class VOModule::Impl {
public:
    explicit Impl(const VOConfig& config)
        : config_(config),
          diag_enabled_(std::getenv("UAVLOC_VO_DIAG") != nullptr),
          camera_model_(config.camera_intrinsics),
          camera_(camera_model_),
          orb_params_(config.orb_name, config.orb_scale_factor, config.orb_num_levels,
                      config.orb_ini_fast_thr, config.orb_min_fast_thr),
          map_db_(config.min_num_shared_lms),
          loader_(&camera_, &orb_params_, config.orb_min_area,
                  feature::DescriptorType::ORB, {}, config.num_grid_cols, config.num_grid_rows),
          pose_optimizer_(std::shared_ptr<optimize::PoseOptimizer>(
              optimize::PoseOptimizerFactory::create(config.vo_node))),
          initializer_(&map_db_, config.vo_node),
          tracker_(&camera_, pose_optimizer_, config.frame_tracker_num_matches_thr,
                   config.use_fixed_seed, config.frame_tracker_margin),
          local_map_updater_(config.max_num_local_keyfrms),
          mapping_module_(&map_db_, config),
          keyframe_inserter_(config.vo_node),
          proj_matcher_(config.projection_lowe_ratio, true) {
        keyframe_inserter_.set_mapping_module(&mapping_module_);
        mapping_module_.start();
        if (config_.temporal_mapping_enabled) {
            // Mirrors stella system::enable_temporal_mapping(): snapshot
            // next_keyframe_id_ as the fixed-keyframe threshold. With an empty
            // map this is 0 (tracking-side temporal logic stays inert); the
            // mapping-side erase block still bounds the keyframe window.
            map_db_.set_fixed_keyframe_id_threshold();
        }
    }

    ~Impl() {
        mapping_module_.stop();
    }

    VOResult process_frame(const sensor::FrameData& fd) {
        // Keep telemetry in parallel, keyed by frame_id (data::Frame has no
        // telemetry field — see VoFrameLoader design note).
        if (fd.has_telemetry) {
            telem_[fd.frame_id] = fd.telemetry;
        }

        data::Frame curr_frm = loader_.load(fd);

        VOResult res;
        res.frame_id       = curr_frm.id_;
        res.timestamp_msec = curr_frm.timestamp_;
        res.num_keypoints  = static_cast<int>(curr_frm.frm_obs_.undist_keypts_.size());

        switch (state_) {
            case VOTrackingState::NOT_INITIALIZED:
                process_not_initialized(curr_frm, res);
                break;
            case VOTrackingState::TRACKING:
                process_tracking(curr_frm, res);
                break;
            case VOTrackingState::LOST:
                process_lost(curr_frm, res);
                break;
        }

        res.state         = state_;
        res.num_landmarks = static_cast<int>(map_db_.get_num_landmarks());
        finalize_result(curr_frm, res);
        publish_data_out(res);
        return res;
    }

    VOTrackingState get_state() const { return state_; }
    data::MapDatabase* get_map_database() { return &map_db_; }

    void add_data_out_callback(std::function<void(const VOData&)> cb) {
        if (cb) {
            data_out_callbacks_.push_back(std::move(cb));
        }
    }

    void clear_data_out_callbacks() { data_out_callbacks_.clear(); }

private:
    //-----------------------------------------
    // state-machine steps

    void process_not_initialized(data::Frame& curr_frm, VOResult& res) {
        initializer_.initialize(camera::SetupType::Monocular, curr_frm);

        if (initializer_.get_state() == module::InitializerState::Wrong) {
            reset_map();
            return;
        }
        if (initializer_.get_state() != module::InitializerState::Succeeded) {
            return;
        }

        // Seeded map exists: two keyframes + landmarks; curr_frm has its pose,
        // ref_keyfrm_ and 2D-3D associations set by the Initializer.
        seed_metric_scale(curr_frm);

        state_ = VOTrackingState::TRACKING;
        twist_is_valid_ = false;
        velocity_ = Mat44_t::Identity();
        set_last_frame(curr_frm);

        res.is_keyframe   = true;
        res.num_landmarks = static_cast<int>(map_db_.get_num_landmarks());
        spdlog::info("VOModule: initialized at frame {} with {} landmarks",
                     curr_frm.id_, map_db_.get_num_landmarks());
    }

    void process_tracking(data::Frame& curr_frm, VOResult& res) {
        // Apply any landmark fusion replacements produced by the async mapping
        // thread so tracking never dereferences a merged-away landmark.
        MappingModule::ReplacedLandmarks pending_replaced;
        mapping_module_.drain_replaced_lms(pending_replaced);
        apply_replaced_lms(last_frm_, pending_replaced);

        // Update the last frame pose from its reference keyframe (mapping may
        // have optimized it) and inherit its reference keyframe.
        update_last_frame();
        curr_frm.ref_keyfrm_ = last_frm_.ref_keyfrm_;

        bool succeeded = track_current_frame(curr_frm);

        // Temporal mapping (mirrors stella tracking_module.cc:229-238):
        // keyframes with id >= fixed_keyframe_id_threshold are "temporal". When
        // any entered the local map, run a second local-map pass without
        // temporal-majority landmarks. Threshold == 0 (default) disables all of
        // this — both guards below are false and the old path is taken.
        const unsigned int fixed_keyframe_id_threshold = map_db_.get_fixed_keyframe_id_threshold();
        unsigned int num_temporal_keyfrms = 0;

        unsigned int num_tracked_lms = 0;
        unsigned int num_reliable_lms = 0;
        const unsigned int min_num_obs_thr = (3 <= map_db_.get_num_keyframes()) ? 3 : 2;
        if (succeeded) {
            succeeded = track_local_map(curr_frm, num_tracked_lms, num_reliable_lms,
                                        num_temporal_keyfrms, min_num_obs_thr,
                                        fixed_keyframe_id_threshold, res);
        }
        if (fixed_keyframe_id_threshold > 0 && succeeded && num_temporal_keyfrms > 0) {
            succeeded = track_local_map_without_temporal_keyframes(
                curr_frm, num_tracked_lms, num_reliable_lms, min_num_obs_thr,
                fixed_keyframe_id_threshold, res);
        }

        if (succeeded) {
            update_motion_model(curr_frm);
        }
        map_db_.update_frame_statistics(curr_frm, !succeeded);

        res.num_inliers = static_cast<int>(num_tracked_lms);

        // Keyframe gate.
        if (succeeded && curr_frm.ref_keyfrm_
            && keyframe_inserter_.new_keyframe_is_needed(&map_db_, curr_frm, num_tracked_lms,
                                                         num_reliable_lms, *curr_frm.ref_keyfrm_, min_num_obs_thr)) {
            const auto prev_kf = curr_frm.ref_keyfrm_;
            keyframe_inserter_.insert_new_keyframe(&map_db_, curr_frm);
            res.is_keyframe = true;
            if (diag_enabled_) {
                const double baseline_since_prev_kf =
                    prev_kf ? (curr_frm.get_trans_wc() - prev_kf->get_trans_wc()).norm() : 0.0;
                spdlog::info("DIAG: KF insert frame={} baseline_since_prev_kf={} num_tracked_lms={}",
                             curr_frm.id_, baseline_since_prev_kf, num_tracked_lms);
            }
            on_new_keyframe(curr_frm);
        }

        if (succeeded) {
            state_ = VOTrackingState::TRACKING;
            set_last_frame(curr_frm);
        }
        else {
            state_ = VOTrackingState::LOST;
            spdlog::info("VOModule: tracking lost at frame {}", curr_frm.id_);
        }
    }

    //! Accumulate the segment-to-global weld transform from the last valid
    //! segment-local pose, so that after re-init (new segment anchored at the
    //! origin) the OUTPUT poses/map points continue from the pre-LOST position.
    //! Called once per loss, at the top of process_lost() (before reset_map()
    //! wipes last_frm_). Internal (segment-local) state is never touched.
    void update_weld_on_lost() {
        if (!config_.weld_on_reinit || !last_frm_.pose_is_valid()) {
            return;
        }
        Mat44_t T_wc_last_local = Mat44_t::Identity();
        T_wc_last_local.block<3, 3>(0, 0) = last_frm_.get_rot_wc();
        T_wc_last_local.block<3, 1>(0, 3) = last_frm_.get_trans_wc();
        T_weld_ = T_weld_ * T_wc_last_local;
        const Vec3_t t = T_weld_.block<3, 1>(0, 3);
        spdlog::info("VOModule: welded re-init anchor at t=({:.2f}, {:.2f}, {:.2f})",
                     t.x(), t.y(), t.z());
    }

    //! Synchronous local mapping after a new keyframe is inserted. Landmarks
    //! merged away during fusion are remapped in the current frame so the next
    //! frame never dereferences a replaced/dead landmark.
    void on_new_keyframe(data::Frame& curr_frm) {
        MappingModule::ReplacedLandmarks replaced_lms;
        mapping_module_.submit(curr_frm.ref_keyfrm_, replaced_lms);
        // Synchronous: fusion replacements are available immediately. Async:
        // replaced_lms is empty here and drained at the next tracking step.
        apply_replaced_lms(curr_frm, replaced_lms);
    }

    //! Remap landmark references in frm according to the fusion replacements
    //! (mirrors stella_vslam::tracking_module::replace_landmarks_in_last_frm).
    static void apply_replaced_lms(data::Frame& frm,
                                   const module::LocalMapper::ReplacedLandmarks& replaced_lms) {
        if (replaced_lms.empty()) {
            return;
        }
        for (unsigned int idx = 0; idx < frm.frm_obs_.undist_keypts_.size(); ++idx) {
            const auto lm = frm.get_landmark(idx);
            if (!lm) {
                continue;
            }
            const auto it = replaced_lms.find(lm);
            if (it == replaced_lms.end()) {
                continue;
            }
            const auto replaced_lm = it->second;
            if (frm.has_landmark(replaced_lm)) {
                frm.erase_landmark(replaced_lm);
            }
            frm.add_landmark(replaced_lm, idx);
        }
    }

    bool track_current_frame(data::Frame& curr_frm) {
        bool succeeded = false;
        if (twist_is_valid_) {
            succeeded = tracker_.motion_based_track(curr_frm, last_frm_, velocity_);
        }
        if (!succeeded && curr_frm.ref_keyfrm_) {
            succeeded = tracker_.robust_match_based_track(curr_frm, last_frm_, curr_frm.ref_keyfrm_);
        }
        return succeeded;
    }

    bool track_local_map(data::Frame& curr_frm,
                         unsigned int& num_tracked_lms,
                         unsigned int& num_reliable_lms,
                         unsigned int& num_temporal_keyfrms,
                         const unsigned int min_num_obs_thr,
                         const unsigned int fixed_keyframe_id_threshold,
                         VOResult& res) {
        if (!update_local_map(curr_frm, fixed_keyframe_id_threshold, num_temporal_keyfrms)) {
            return false;
        }
        // Deviation from stella (pre-existing port behavior): the first-pass
        // search result is not used as a failure signal, keeping the default
        // (threshold == 0) path byte-identical to the previous implementation.
        search_local_landmarks(curr_frm, fixed_keyframe_id_threshold);
        return optimize_current_frame_with_local_map(curr_frm, num_tracked_lms, num_reliable_lms, min_num_obs_thr, res);
    }

    //! Second local-map pass excluding temporal-majority landmarks (mirrors
    //! stella tracking_module::track_local_map_without_temporal_keyframes).
    //! Only reachable when fixed_keyframe_id_threshold > 0.
    bool track_local_map_without_temporal_keyframes(data::Frame& curr_frm,
                                                    unsigned int& num_tracked_lms,
                                                    unsigned int& num_reliable_lms,
                                                    const unsigned int min_num_obs_thr,
                                                    const unsigned int fixed_keyframe_id_threshold,
                                                    VOResult& res) {
        bool succeeded = search_local_landmarks(curr_frm, fixed_keyframe_id_threshold);

        if (config_.enable_temporal_keyframe_only_tracking && !succeeded) {
            spdlog::debug("VOModule: temporal keyframe only tracking (frame={})", curr_frm.id_);
            return true;
        }

        if (succeeded) {
            succeeded = optimize_current_frame_with_local_map(curr_frm, num_tracked_lms,
                                                              num_reliable_lms, min_num_obs_thr, res);
        }

        if (!succeeded) {
            spdlog::info("VOModule: local map tracking (without temporal keyframes) failed at frame {}",
                         curr_frm.id_);
        }
        return succeeded;
    }

    void update_last_frame() {
        auto last_ref_keyfrm = last_frm_.ref_keyfrm_;
        if (!last_ref_keyfrm) {
            return;
        }
        last_frm_.set_pose_cw(last_cam_pose_from_ref_keyfrm_ * last_ref_keyfrm->get_pose_cw());
    }

    bool update_local_map(data::Frame& curr_frm,
                          const unsigned int fixed_keyframe_id_threshold,
                          unsigned int& num_temporal_keyfrms) {
        // Drop associations to landmarks that are being erased.
        for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
            const auto& lm = curr_frm.get_landmark(idx);
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                curr_frm.erase_landmark_with_index(idx);
            }
        }

        // Temporal-aware overload (stella tracking_module.cc:517); identical to
        // the plain overload when the threshold is 0.
        if (!local_map_updater_.acquire_local_map(curr_frm.get_landmarks(),
                                                  fixed_keyframe_id_threshold,
                                                  num_temporal_keyfrms)) {
            return false;
        }
        local_landmarks_ = local_map_updater_.get_local_landmarks();
        const auto nearest_covisibility = local_map_updater_.get_nearest_covisibility();
        if (nearest_covisibility) {
            curr_frm.ref_keyfrm_ = nearest_covisibility;
        }
        map_db_.set_local_landmarks(local_landmarks_);
        return true;
    }

    //! Returns whether at least one local landmark is a projection candidate
    //! (stella found_proj_candidate). The return value is consumed only by the
    //! non-temporal second pass; the first pass ignores it (pre-existing port
    //! deviation, keeps the default path byte-identical).
    bool search_local_landmarks(data::Frame& curr_frm, const unsigned int fixed_keyframe_id_threshold) {
        std::unordered_set<unsigned int> curr_landmark_ids;
        for (const auto& lm : curr_frm.get_landmarks()) {
            if (!lm || lm->will_be_erased()) {
                continue;
            }
            curr_landmark_ids.insert(lm->id_);
            lm->increase_num_observable();
        }

        bool found_proj_candidate = false;
        Vec2_t reproj;
        float x_right = 0.0f;
        unsigned int pred_scale_level = 0;
        eigen_alloc_unord_map<unsigned int, Vec2_t> lm_to_reproj;
        std::unordered_map<unsigned int, float> lm_to_x_right;
        std::unordered_map<unsigned int, unsigned int> lm_to_scale;
        for (const auto& lm : local_landmarks_) {
            if (!lm || lm->will_be_erased()) {
                continue;
            }
            if (curr_landmark_ids.count(lm->id_)) {
                continue;
            }
            // Temporal mapping (stella tracking_module.cc:567-581): skip
            // landmarks observed mostly from temporal keyframes.
            if (fixed_keyframe_id_threshold > 0) {
                const auto observations = lm->get_observations();
                unsigned int temporal_observations = 0;
                for (const auto& obs : observations) {
                    const auto keyfrm = obs.first.lock();
                    if (keyfrm && keyfrm->id_ >= fixed_keyframe_id_threshold) {
                        ++temporal_observations;
                    }
                }
                // Mirrors the stella_vslam constant (tracking_module.cc:576).
                const double temporal_ratio_thr = 0.5;
                const double temporal_ratio = observations.empty()
                                                  ? 0.0
                                                  : static_cast<double>(temporal_observations)
                                                        / static_cast<double>(observations.size());
                if (temporal_ratio > temporal_ratio_thr) {
                    continue;
                }
            }
            if (curr_frm.can_observe(lm, 0.5f, reproj, x_right, pred_scale_level)) {
                lm_to_reproj[lm->id_] = reproj;
                lm_to_x_right[lm->id_] = x_right;
                lm_to_scale[lm->id_] = pred_scale_level;
                lm->increase_num_observable();
                found_proj_candidate = true;
            }
        }

        proj_matcher_.match_frame_and_landmarks(curr_frm, local_landmarks_, lm_to_reproj,
                                                lm_to_x_right, lm_to_scale,
                                                config_.local_map_projection_margin);
        return found_proj_candidate;
    }

    bool optimize_current_frame_with_local_map(data::Frame& curr_frm,
                                               unsigned int& num_tracked_lms,
                                               unsigned int& num_reliable_lms,
                                               const unsigned int min_num_obs_thr,
                                               VOResult& res) {
        Mat44_t optimized_pose;
        std::vector<bool> outlier_flags;
        pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
        curr_frm.set_pose_cw(optimized_pose);

        for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
            if (idx < outlier_flags.size() && outlier_flags.at(idx)) {
                curr_frm.erase_landmark_with_index(idx);
            }
        }

        num_tracked_lms = 0;
        num_reliable_lms = 0;
        res.tracked_observations.clear();
        for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
            const auto& lm = curr_frm.get_landmark(idx);
            if (!lm || lm->will_be_erased()) {
                continue;
            }
            if (0 < min_num_obs_thr && min_num_obs_thr <= lm->num_observations()) {
                ++num_reliable_lms;
            }
            ++num_tracked_lms;
            lm->increase_num_observed();
            // Undistorted pixel position of this tracked keypoint, exported for
            // visualization (additive; no effect on the tracking logic).
            const auto& kp = curr_frm.frm_obs_.undist_keypts_[idx];
            res.tracked_observations.emplace_back(static_cast<double>(kp.pt.x),
                                                  static_cast<double>(kp.pt.y));
        }

        res.num_matches = static_cast<int>(num_tracked_lms);
        const int denom = std::max(1, res.num_keypoints);
        res.inlier_ratio = static_cast<double>(num_tracked_lms) / static_cast<double>(denom);

        if (diag_enabled_) {
            spdlog::info("DIAG: track frame={} num_tracked_lms={} local_lms={}",
                         curr_frm.id_, num_tracked_lms, local_landmarks_.size());
        }

        if (num_tracked_lms < config_.min_inliers_to_track) {
            if (diag_enabled_) {
                spdlog::info("DIAG: LOST frame={} num_tracked_lms={} min_inliers_to_track={}",
                             curr_frm.id_, num_tracked_lms, config_.min_inliers_to_track);
            }
            spdlog::debug("VOModule: local map tracking failed: {} tracked < {}",
                          num_tracked_lms, config_.min_inliers_to_track);
            return false;
        }
        return true;
    }

    void update_motion_model(const data::Frame& curr_frm) {
        if (last_frm_.pose_is_valid()) {
            Mat44_t last_frm_cam_pose_wc = Mat44_t::Identity();
            last_frm_cam_pose_wc.block<3, 3>(0, 0) = last_frm_.get_rot_wc();
            last_frm_cam_pose_wc.block<3, 1>(0, 3) = last_frm_.get_trans_wc();
            twist_is_valid_ = true;
            velocity_ = curr_frm.get_pose_cw() * last_frm_cam_pose_wc;
        }
        else {
            twist_is_valid_ = false;
            velocity_ = Mat44_t::Identity();
        }
    }

    void process_lost(data::Frame& /*curr_frm*/, VOResult& /*res*/) {
        // Capture the weld anchor exactly once per loss, before reset_map()
        // wipes last_frm_. Deliberately NOT done at the TRACKING -> LOST
        // transition itself so that the LOST frame's own (rejected) pose is
        // still finalized with the old segment's weld — updating there would
        // double-apply the weld to that frame's published pose.
        update_weld_on_lost();
        // Pure VO: no relocalization — restart initialization.
        reset_map();
        state_ = VOTrackingState::NOT_INITIALIZED;
    }

    //! Tear down the map on LOST -> re-init. In ASYNC mode this first blocks
    //! until the mapping thread is idle (pause_and_wait), so map_db_.clear()
    //! can never race an in-flight LocalMapper::map() on the keyframes/landmarks
    //! being erased. In SYNC mode pause_and_wait()/resume() are no-ops, so this
    //! is behaviourally identical to the previous inline reset (deterministic).
    //! Also drops tracking-side references (local_landmarks_, last_frm_) that
    //! pointed into the cleared map so the next frame cannot dereference freed
    //! landmarks.
    void reset_map() {
        // ASYNC: block until the mapping thread is idle so map_db_.clear() cannot
        // race an in-flight LocalMapper::map(); then drop the mapper's stale
        // scratch state (fresh-landmark queue + fusion replacements) that would
        // otherwise dereference just-erased keyframes/landmarks. SYNC: all three
        // MappingModule calls are no-ops, so this is behaviourally identical to
        // the previous inline reset (deterministic baseline unchanged).
        mapping_module_.pause_and_wait(); // no-op in sync
        mapping_module_.reset_state();    // no-op in sync
        initializer_.reset();
        map_db_.clear();
        if (config_.temporal_mapping_enabled) {
            // Re-arm temporal mapping after the reset. Deviation from stella:
            // there enable_temporal_mapping() is an app-level one-shot call and a
            // reset silently drops the threshold; our VOModule owns the whole
            // session, so LOST -> re-init keeps temporal mapping enabled.
            // (clear() zeroed next_keyframe_id_, so the threshold is 0 again —
            // behaviorally identical for a pure-VO empty-map restart.)
            map_db_.set_fixed_keyframe_id_threshold();
        }
        // Drop tracking-side references into the now-cleared map so the next
        // frame cannot dereference freed landmarks.
        local_landmarks_.clear();
        last_frm_ = data::Frame();
        mapping_module_.resume();         // no-op effect in sync
        twist_is_valid_ = false;
        velocity_ = Mat44_t::Identity();
    }

    //-----------------------------------------
    // helpers

    void set_last_frame(const data::Frame& curr_frm) {
        last_frm_ = curr_frm;
        if (curr_frm.pose_is_valid() && curr_frm.ref_keyfrm_) {
            last_cam_pose_from_ref_keyfrm_ = curr_frm.get_pose_cw() * curr_frm.ref_keyfrm_->get_pose_wc();
        }
    }

    void finalize_result(const data::Frame& curr_frm, VOResult& res) {
        if (curr_frm.pose_is_valid()) {
            // Output-side weld: publish poses in the welded global frame while
            // the internal pipeline keeps operating on segment-local poses.
            // prev_T_wc_ / last_valid_T_wc_ are stored welded, so T_prev_curr
            // stays consistent between consecutive welded poses. When
            // weld_on_reinit is false the local pose is used untouched
            // (bit-identical deterministic baseline).
            const Mat44_t T_wc = config_.weld_on_reinit
                                     ? Mat44_t(T_weld_ * curr_frm.get_pose_wc())
                                     : curr_frm.get_pose_wc();
            res.has_pose = true;
            res.T_wc     = T_wc;
            if (has_prev_pose_) {
                res.T_prev_curr = T_wc * prev_T_wc_.inverse();
            }
            prev_T_wc_    = T_wc;
            has_prev_pose_ = true;
            last_valid_T_wc_ = T_wc;
        }
        else {
            res.has_pose = false;
            res.T_wc     = last_valid_T_wc_;
        }
    }

    //! Build a VOData payload for this frame and invoke every registered
    //! data-out callback in registration order. Map points are gathered only on
    //! keyframes (when the map changed) to keep the per-frame cost low.
    void publish_data_out(const VOResult& res) {
        if (data_out_callbacks_.empty()) {
            return;
        }
        VOData data;
        data.result = res;
        if (res.is_keyframe) {
            data.map_updated = true;
            for (const auto& lm : map_db_.get_all_landmarks()) {
                if (!lm || lm->will_be_erased()) {
                    continue;
                }
                // Output-side weld: map points leave the module in the welded
                // global frame (the internal map stays segment-local).
                Vec3_t pos = lm->get_pos_in_world();
                if (config_.weld_on_reinit) {
                    pos = T_weld_.block<3, 3>(0, 0) * pos + T_weld_.block<3, 1>(0, 3);
                }
                data.map_points.push_back(pos);
            }
        }
        for (const auto& cb : data_out_callbacks_) {
            cb(data);
        }
    }

    //! Rescale the seeded map to metric using the altitude/AGL prior. The
    //! Initializer normalizes the median depth to 1.0, so the metric scale is
    //! simply the target slant range. Rescales keyframe translations and
    //! landmark positions about the world origin (reprojection-invariant).
    void seed_metric_scale(data::Frame& curr_frm) {
        if (!config_.enable_metric_scale) {
            return;
        }
        auto it = telem_.find(curr_frm.id_);
        if (it == telem_.end() || !it->second.valid) {
            spdlog::debug("VOModule: no telemetry for frame {} — skip metric scale", curr_frm.id_);
            return;
        }
        const double altitude = it->second.altitude_m;
        if (altitude <= 0.0) {
            return;
        }

        // Off-nadir angle from the gimbal tilt (tilt ~90 deg = nadir). Fall back
        // to a nadir assumption when the tilt is implausible.
        double off_nadir_rad = 0.0;
        const double tilt = it->second.gimbal_tilt_deg;
        if (tilt > 45.0 && tilt <= 90.0) {
            off_nadir_rad = (90.0 - tilt) * M_PI / 180.0;
        }
        const double cos_off = std::cos(off_nadir_rad);
        if (cos_off <= 1e-6) {
            return;
        }
        const double target_depth = altitude / cos_off;

        const auto ref_keyfrm = curr_frm.ref_keyfrm_;
        if (!ref_keyfrm) {
            return;
        }
        const float median_depth = ref_keyfrm->compute_median_depth(true);
        if (!(median_depth > 0.0f)) {
            return;
        }
        const double scale = target_depth / static_cast<double>(median_depth);
        if (!std::isfinite(scale) || scale <= 0.0) {
            return;
        }

        // Scale all keyframe translations.
        for (const auto& keyfrm : map_db_.get_all_keyframes()) {
            if (!keyfrm) {
                continue;
            }
            Mat44_t pose_cw = keyfrm->get_pose_cw();
            pose_cw.block<3, 1>(0, 3) *= scale;
            keyfrm->set_pose_cw(pose_cw);
        }
        // Scale all landmark positions.
        for (const auto& lm : map_db_.get_all_landmarks()) {
            if (!lm) {
                continue;
            }
            lm->set_pos_in_world(lm->get_pos_in_world() * scale);
            lm->update_mean_normal_and_obs_scale_variance();
        }

        // Refresh the current frame pose from its (rescaled) reference keyframe.
        curr_frm.set_pose_cw(ref_keyfrm->get_pose_cw());
        spdlog::info("VOModule: metric scale seeded (scale={:.3f}, target_depth={:.1f} m)",
                     scale, target_depth);
    }

    //-----------------------------------------
    // owned components

    VOConfig config_;

    //! Env-gated (UAVLOC_VO_DIAG) diagnostic logging switch, read once at ctor.
    const bool diag_enabled_;

    sensor::CameraModel        camera_model_;
    camera::PerspectiveCamera  camera_;
    feature::OrbParams         orb_params_;
    data::MapDatabase          map_db_;
    VoFrameLoader              loader_;

    std::shared_ptr<optimize::PoseOptimizer> pose_optimizer_;
    module::Initializer        initializer_;
    module::FrameTracker       tracker_;
    module::LocalMapUpdater    local_map_updater_;
    MappingModule              mapping_module_;
    module::KeyframeInserter   keyframe_inserter_;
    match::Projection          proj_matcher_;

    std::vector<std::shared_ptr<data::Landmark>> local_landmarks_;

    //-----------------------------------------
    // state

    VOTrackingState state_ = VOTrackingState::NOT_INITIALIZED;
    data::Frame     last_frm_;
    Mat44_t         velocity_ = Mat44_t::Identity();
    bool            twist_is_valid_ = false;
    Mat44_t         last_cam_pose_from_ref_keyfrm_ = Mat44_t::Identity();

    Mat44_t prev_T_wc_       = Mat44_t::Identity();
    bool    has_prev_pose_   = false;
    Mat44_t last_valid_T_wc_ = Mat44_t::Identity();

    //! Accumulated segment-to-global weld transform (weld-on-reinit). Updated
    //! only at TRACKING -> LOST transitions when config_.weld_on_reinit is
    //! enabled; stays identity forever otherwise. Applied output-side only.
    Mat44_t T_weld_ = Mat44_t::Identity();

    std::unordered_map<uint64_t, sensor::TelemetryData> telem_;

    //! Data-out subscribers, invoked once per frame at the end of process_frame.
    std::vector<std::function<void(const VOData&)>> data_out_callbacks_;
};

//-----------------------------------------
// VOModule pimpl forwarding

VOModule::VOModule(const VOConfig& config)
    : impl_(new Impl(config)) {}

VOModule::~VOModule() = default;

VOResult VOModule::process_frame(const sensor::FrameData& fd) {
    return impl_->process_frame(fd);
}

void VOModule::add_data_out_callback(std::function<void(const VOData&)> cb) {
    impl_->add_data_out_callback(std::move(cb));
}

void VOModule::clear_data_out_callbacks() {
    impl_->clear_data_out_callbacks();
}

VOTrackingState VOModule::get_state() const {
    return impl_->get_state();
}

data::MapDatabase* VOModule::get_map_database() const {
    return impl_->get_map_database();
}

} // namespace vo
} // namespace uavloc
