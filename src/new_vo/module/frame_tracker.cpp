#include "uavloc/new_vo/camera/base.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/match/projection.h"
#include "uavloc/new_vo/match/robust.h"
#include "uavloc/new_vo/module/frame_tracker.h"
#include "uavloc/new_vo/optimize/pose_optimizer_g2o.h"
#include "uavloc/util/scoped_timer.h"

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace module {

FrameTracker::FrameTracker(camera::Base* camera, const std::shared_ptr<optimize::PoseOptimizer>& pose_optimizer,
                             const unsigned int num_matches_thr, bool use_fixed_seed, float margin)
    : camera_(camera), num_matches_thr_(num_matches_thr), use_fixed_seed_(use_fixed_seed), margin_(margin), pose_optimizer_(pose_optimizer) {}

bool FrameTracker::motion_based_track(data::Frame& curr_frm, const data::Frame& last_frm, const Mat44_t& velocity) const {
    match::Projection projection_matcher(0.9, true);

    // Set the initial pose by using the motion model
    curr_frm.set_pose_cw(velocity * last_frm.get_pose_cw());

    // Initialize the 2D-3D matches
    curr_frm.erase_landmarks();

    // Reproject the 3D points observed in the last Frame and find 2D-3D matches
    unsigned int num_matches = 0;
    {
        util::ScopedTimer _t(util::ProfileStage::TRACK_MATCH);
        num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, margin_);
    }

    if (num_matches < num_matches_thr_) {
        // Increment the margin, and search again
        curr_frm.erase_landmarks();
        util::ScopedTimer _t(util::ProfileStage::TRACK_MATCH);
        num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, 2 * margin_);
    }

    if (num_matches < num_matches_thr_) {
        spdlog::debug("motion based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // Pose optimization
    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    {
        util::ScopedTimer _t(util::ProfileStage::TRACK_POSE_OPT);
        pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
    }
    curr_frm.set_pose_cw(optimized_pose);

    // Discard the outliers
    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        spdlog::debug("motion based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }
    else {
        return true;
    }
}

bool FrameTracker::robust_match_based_track(data::Frame& curr_frm, const data::Frame& last_frm, const std::shared_ptr<data::Keyframe>& ref_keyfrm) const {
    match::Robust robust_matcher(0.8, true);

    // Search 2D-2D matches between the ref keyframes and the current Frame
    // to acquire 2D-3D matches between the Frame keypoints and 3D points observed in the ref Keyframe
    std::vector<std::shared_ptr<data::Landmark>> matched_lms_in_curr;
    unsigned int num_matches = 0;
    {
        util::ScopedTimer _t(util::ProfileStage::TRACK_MATCH);
        num_matches = robust_matcher.match_frame_and_keyframe(curr_frm, ref_keyfrm, matched_lms_in_curr, use_fixed_seed_);
    }

    if (num_matches < num_matches_thr_) {
        spdlog::debug("robust match based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // Update the 2D-3D matches
    curr_frm.set_landmarks(matched_lms_in_curr);

    // Pose optimization
    // The initial value is the pose of the previous Frame
    curr_frm.set_pose_cw(last_frm.get_pose_cw());
    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    {
        util::ScopedTimer _t(util::ProfileStage::TRACK_POSE_OPT);
        pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
    }
    curr_frm.set_pose_cw(optimized_pose);

    // Discard the outliers
    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        spdlog::debug("robust match based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }
    else {
        return true;
    }
}

unsigned int FrameTracker::discard_outliers(const std::vector<bool>& outlier_flags, data::Frame& curr_frm) const {
    unsigned int num_valid_matches = 0;

    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
        if (curr_frm.get_landmark(idx) == nullptr) {
            continue;
        }

        if (outlier_flags.at(idx)) {
            curr_frm.erase_landmark_with_index(idx);
        }
        else {
            ++num_valid_matches;
        }
    }

    return num_valid_matches;
}

} // namespace module
}} // namespace vo // namespace uavloc
