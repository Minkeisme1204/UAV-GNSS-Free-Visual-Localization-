#pragma once

#include "uavloc/common/type.h"
// TODO(port): camera/ not cloned — see .docs/designs/new_vo_port_notes.md
#include "uavloc/new_vo/camera/base.h"
// NOTE(port): reuses the already-ported ORB feature header. Its type is named
// feature::OrbParams (PascalCase) whereas the mirrored data/ code below still
// references feature::OrbParams — reconcile at wiring. See port-notes.
#include "uavloc/new_vo/feature/orb/orb_params.h"
// TODO(port): util/ not cloned — see .docs/designs/new_vo_port_notes.md
#include "uavloc/util/converter.h"
#include "uavloc/new_vo/data/frame_observation.h"

#include <vector>
#include <atomic>
#include <memory>
#include <unordered_set>

#include <Eigen/Core>

namespace uavloc {
namespace vo {

namespace camera {
class Base;
} // namespace camera

namespace feature {
class OrbExtractor;
struct OrbParams;
} // namespace feature

namespace data {

class Keyframe;
class Landmark;

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame() = default;

    bool operator==(const Frame& frm) { return this->id_ == frm.id_; }
    bool operator!=(const Frame& frm) { return !(*this == frm); }

    /**
     * Constructor for monocular Frame
     * @param frame_id
     * @param timestamp
     * @param camera
     * @param OrbParams
     * @param frm_obs
     */
    Frame(const unsigned int frame_id, const double timestamp, camera::Base* camera, feature::OrbParams* orb_params,
          const FrameObservation frm_obs);

    /**
     * Set camera pose and refresh rotation and translation
     * @param pose_cw
     */
    void set_pose_cw(const Mat44_t& pose_cw);

    /**
     * Get camera pose
     */
    Mat44_t get_pose_cw() const;

    /**
     * Get the inverse of the camera pose
     */
    Mat44_t get_pose_wc() const;

    /**
     * Get camera center
     * @return
     */
    Vec3_t get_trans_wc() const;

    /**
     * Get inverse of rotation
     * @return
     */
    Mat33_t get_rot_wc() const;

    /**
     * Get the translation of the camera pose
     */
    Vec3_t get_trans_cw() const {
        return trans_cw_;
    }

    /**
     * Get the rotation of the camera pose
     */
    Mat33_t get_rot_cw() const {
        return rot_cw_;
    }

    /**
     * Invalidate pose
     */
    void invalidate_pose() {
        pose_is_valid_ = false;
    }

    /**
     * Return true if pose is valid
     */
    bool pose_is_valid() const {
        return pose_is_valid_;
    }

    /**
     * Check observability of the Landmark
     */
    bool can_observe(const std::shared_ptr<Landmark>& lm, const float ray_cos_thr,
                     Vec2_t& reproj, float& x_right, unsigned int& pred_scale_level) const;

    bool has_landmark(const std::shared_ptr<Landmark>& lm) const;

    void add_landmark(const std::shared_ptr<Landmark>&, const unsigned int idx);

    std::shared_ptr<Landmark> get_landmark(const unsigned int idx) const;

    void erase_landmark_with_index(const unsigned int idx);

    void erase_landmark(const std::shared_ptr<Landmark>& lm);

    std::vector<std::shared_ptr<Landmark>> get_landmarks() const;

    void erase_landmarks();

    void set_landmarks(const std::vector<std::shared_ptr<Landmark>>& landmarks);

    /**
     * Get keypoint indices in the cell which reference point is located
     * @param ref_x
     * @param ref_y
     * @param margin
     * @param min_level
     * @param max_level
     * @return
     */
    std::vector<unsigned int> get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level = -1, const int max_level = -1) const;

    /**
     * Perform stereo triangulation of the keypoint
     * @param idx
     * @return
     */
    Vec3_t triangulate_stereo(const unsigned int idx) const;

    //! current Frame ID
    unsigned int id_;

    //! timestamp
    double timestamp_;

    //! camera model
    camera::Base* camera_ = nullptr;

    //! ORB scale pyramid information
    const feature::OrbParams* orb_params_ = nullptr;

    //! constant observations
    FrameObservation frm_obs_;

    //! reference Keyframe for tracking
    std::shared_ptr<Keyframe> ref_keyfrm_ = nullptr;

private:
    //! landmarks, whose nullptr indicates no-association
    std::vector<std::shared_ptr<Landmark>> landmarks_;
    std::unordered_map<std::shared_ptr<Landmark>, unsigned int> landmarks_idx_map_;

    //! camera pose: world -> camera
    bool pose_is_valid_ = false;
    Mat44_t pose_cw_;

    //! Camera pose
    //! rotation: world -> camera
    Mat33_t rot_cw_;
    //! translation: world -> camera
    Vec3_t trans_cw_;
    //! rotation: camera -> world
    Mat33_t rot_wc_;
    //! translation: camera -> world
    Vec3_t trans_wc_;
};

} // namespace data
}} // namespace vo // namespace uavloc
