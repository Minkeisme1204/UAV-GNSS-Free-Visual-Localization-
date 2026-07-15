#pragma once

#include "uavloc/common/type.h"
// TODO(port): camera/ not cloned — see .docs/designs/new_vo_port_notes.md
#include "uavloc/new_vo/camera/base.h"
// NOTE(port): reuses the already-ported ORB feature header (feature::OrbParams,
// PascalCase). Mirrored data/ code below still names feature::OrbParams —
// reconcile at wiring. See .docs/designs/new_vo_port_notes.md
#include "uavloc/new_vo/feature/orb/orb_params.h"
#include "uavloc/new_vo/data/graph_node.h"
#include "uavloc/new_vo/data/frame_observation.h"

#include <set>
#include <mutex>
#include <atomic>
#include <memory>

#include <nlohmann/json_fwd.hpp>

// sqlite3 forward declarations (persistence signatures kept, bodies cut — the
// <sqlite3.h> include was dropped to avoid the sqlite3 dependency).
// TODO(port): serialization cut — see .docs/designs/new_vo_port_notes.md
typedef struct sqlite3 sqlite3;
typedef struct sqlite3_stmt sqlite3_stmt;

namespace uavloc {
namespace vo {

namespace camera {
class Base;
} // namespace camera

namespace data {

class Frame;
class Landmark;
class MapDatabase;
class CameraDatabase;
class OrbParamsDatabase;

class Keyframe : public std::enable_shared_from_this<Keyframe> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor for building from a Frame
     */
    explicit Keyframe(unsigned int id, const Frame& frm);

    /**
     * Constructor for map loading
     * (NOTE: some variables must be recomputed after the construction. See the definition.)
     */
    Keyframe(const unsigned int id,
             const double timestamp, const Mat44_t& pose_cw, camera::Base* camera,
             const feature::OrbParams* orb_params, const FrameObservation& frm_obs);
    virtual ~Keyframe();

    // Factory method for create Keyframe
    static std::shared_ptr<Keyframe> make_keyframe(unsigned int id, const Frame& frm);
    static std::shared_ptr<Keyframe> make_keyframe(
        const unsigned int id,
        const double timestamp, const Mat44_t& pose_cw, camera::Base* camera,
        const feature::OrbParams* orb_params, const FrameObservation& frm_obs);
    static std::shared_ptr<Keyframe> from_stmt(sqlite3_stmt* stmt,
                                               CameraDatabase* cam_db,
                                               OrbParamsDatabase* orb_params_db,
                                               unsigned int next_keyframe_id);

    // operator overrides
    bool operator==(const Keyframe& keyfrm) const { return id_ == keyfrm.id_; }
    bool operator!=(const Keyframe& keyfrm) const { return !(*this == keyfrm); }
    bool operator<(const Keyframe& keyfrm) const { return id_ < keyfrm.id_; }
    bool operator<=(const Keyframe& keyfrm) const { return id_ <= keyfrm.id_; }
    bool operator>(const Keyframe& keyfrm) const { return id_ > keyfrm.id_; }
    bool operator>=(const Keyframe& keyfrm) const { return id_ >= keyfrm.id_; }

    /**
     * Encode this Keyframe information as JSON
     */
    nlohmann::json to_json() const;

    /**
     * Save this Keyframe information to db
     */
    static std::vector<std::pair<std::string, std::string>> columns() {
        return std::vector<std::pair<std::string, std::string>>{
            {"src_frm_id", "INTEGER"}, // removed
            {"ts", "REAL"},
            {"cam", "BLOB"},
            {"OrbParams", "BLOB"},
            {"pose_cw", "BLOB"},
            {"n_keypts", "INTEGER"},
            {"undist_keypts", "BLOB"},
            {"x_rights", "BLOB"},
            {"depths", "BLOB"},
            {"descs", "BLOB"}};
    };
    bool bind_to_stmt(sqlite3* db, sqlite3_stmt* stmt) const;

    //-----------------------------------------
    // camera pose

    /**
     * Set camera pose
     */
    void set_pose_cw(const Mat44_t& pose_cw);

    /**
     * Get the camera pose
     */
    Mat44_t get_pose_cw() const;

    /**
     * Get the inverse of the camera pose
     */
    Mat44_t get_pose_wc() const;

    /**
     * Get the camera center
     */
    Vec3_t get_trans_wc() const;

    /**
     * Get the rotation of the camera pose
     */
    Mat33_t get_rot_cw() const;

    /**
     * Get the translation of the camera pose
     */
    Vec3_t get_trans_cw() const;

    //-----------------------------------------
    // features and observations

    /**
     * Add a Landmark observed by myself at keypoint idx
     */
    void add_landmark(std::shared_ptr<Landmark> lm, const unsigned int idx);

    /**
     * Erase a Landmark observed by myself at keypoint idx
     */
    void erase_landmark_with_index(const unsigned int idx);

    /**
     * Erase a Landmark
     */
    void erase_landmark(const std::shared_ptr<Landmark>& lm);

    /**
     * Update all of the landmarks
     */
    void update_landmarks();

    /**
     * Get all of the landmarks
     * (NOTE: including nullptr)
     */
    std::vector<std::shared_ptr<Landmark>> get_landmarks() const;

    /**
     * Get the valid landmarks
     */
    std::set<std::shared_ptr<Landmark>> get_valid_landmarks() const;

    /**
     * Get the number of tracked landmarks which have observers equal to or greater than the threshold
     */
    unsigned int get_num_tracked_landmarks(const unsigned int min_num_obs_thr) const;

    /**
     * Get the Landmark associated keypoint idx
     */
    std::shared_ptr<Landmark>& get_landmark(const unsigned int idx);

    /**
     * Get the keypoint indices in the cell which reference point is located
     */
    std::vector<unsigned int> get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin,
                                                    const int min_level = -1, const int max_level = -1) const;

    /**
     * Triangulate the keypoint using the disparity
     */
    Vec3_t triangulate_stereo(const unsigned int idx) const;

    /**
     * Compute median of depths
     */
    float compute_median_depth(const bool abs = false) const;

    /**
     * Compute median of distances
     */
    float compute_median_distance() const;

    /**
     * Whether or not the camera setting is capable of obtaining depth information
     */
    bool depth_is_available() const;

    //-----------------------------------------
    // flags

    /**
     * Set this Keyframe as non-erasable
     */
    void set_not_to_be_erased();

    /**
     * Set this Keyframe as erasable
     */
    void set_to_be_erased();

    /**
     * Erase this Keyframe
     */
    void prepare_for_erasing(MapDatabase* map_db);

    /**
     * Whether this Keyframe will be erased shortly or not
     */
    bool will_be_erased();

    //-----------------------------------------
    // meta information

    //! Keyframe ID
    unsigned int id_;

    //! timestamp in seconds
    const double timestamp_;

    //-----------------------------------------
    // camera parameters

    //! camera model
    camera::Base* camera_;

    //-----------------------------------------
    // feature extraction parameters

    //! ORB feature extraction model
    const feature::OrbParams* orb_params_;

    //-----------------------------------------
    // constant observations

    FrameObservation frm_obs_;

    //-----------------------------------------
    // covisibility graph

    //! graph node
    std::unique_ptr<GraphNode> graph_node_ = nullptr;

private:
    //-----------------------------------------
    // camera pose

    //! need mutex for access to poses
    mutable std::mutex mtx_pose_;
    //! camera pose from the world to the current
    Mat44_t pose_cw_;
    //! camera pose from the current to the world
    Mat44_t pose_wc_;
    //! camera center
    Vec3_t trans_wc_;

    //-----------------------------------------
    // observations

    //! need mutex for access to Landmark observations
    mutable std::mutex mtx_observations_;
    //! observed landmarks
    std::vector<std::shared_ptr<Landmark>> landmarks_;

    //-----------------------------------------
    // flags

    //! flag which indicates this Keyframe is erasable or not
    std::atomic<bool> cannot_be_erased_{false};

    //! flag which indicates this Keyframe will be erased
    std::atomic<bool> will_be_erased_{false};
};

} // namespace data
}} // namespace vo // namespace uavloc
