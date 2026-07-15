#pragma once

#include "uavloc/common/type.h"
// TODO(port): data/FrameStatistics not cloned (publish/eval only) — stubbed.
// See .docs/designs/new_vo_port_notes.md
#include "uavloc/new_vo/data/frame_statistics.h"

#include <mutex>
#include <atomic>
#include <vector>
#include <unordered_map>
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
class Keyframe;
class Landmark;
class CameraDatabase;
class OrbParamsDatabase;

class MapDatabase {
public:
    /**
     * Constructor
     */
    MapDatabase(unsigned int min_num_shared_lms);

    /**
     * Destructor
     */
    ~MapDatabase();

    /**
     * Set fixed_keyframe_id_threshold
     */
    void set_fixed_keyframe_id_threshold();

    /**
     * Get fixed_keyframe_id_threshold
     */
    unsigned int get_fixed_keyframe_id_threshold();

    /**
     * Add Keyframe to the database
     * @param keyfrm
     */
    void add_keyframe(const std::shared_ptr<Keyframe>& keyfrm);

    /**
     * Erase Keyframe from the database
     * @param keyfrm
     */
    void erase_keyframe(const std::shared_ptr<Keyframe>& keyfrm);

    /**
     * Get Keyframe from the database
     * @param id
     */
    std::shared_ptr<Keyframe> get_keyframe(unsigned int id) const;

    /**
     * Add Landmark to the database
     * @param lm
     */
    void add_landmark(std::shared_ptr<Landmark>& lm);

    /**
     * Erase Landmark from the database
     * @param id
     */
    void erase_landmark(unsigned int id);

    /**
     * Get Landmark from the database
     * @param id
     */
    std::shared_ptr<Landmark> get_landmark(unsigned int id) const;

    /**
     * Set local landmarks
     * @param local_lms
     */
    void set_local_landmarks(const std::vector<std::shared_ptr<Landmark>>& local_lms);

    /**
     * Get local landmarks
     * @return
     */
    std::vector<std::shared_ptr<Landmark>> get_local_landmarks() const;

    /**
     * Get all of the keyframes in the database
     * NOTE: Access multiple spanning trees. Used only to read and write databases.
     * @return
     */
    std::vector<std::shared_ptr<Keyframe>> get_all_keyframes() const;

    /**
     * Get closest keyframes to a given 2d pose
     * @param pose Given 2d pose
     * @param normal_vector normal vector of plane
     * @param distance_threshold Maximum distance where close keyframes could be found
     * @param angle_threshold Maximum angle between given pose and close keyframes
     * @return Vector closest keyframes
     */
    std::vector<std::shared_ptr<Keyframe>> get_close_keyframes_2d(const Mat44_t& pose_cw,
                                                                  const Vec3_t& normal_vector,
                                                                  const double distance_threshold,
                                                                  const double angle_threshold) const;

    /**
     * Get closest keyframes to a given pose
     * @param pose Given pose
     * @param distance_threshold Maximum distance where close keyframes could be found
     * @param angle_threshold Maximum angle between given pose and close keyframes
     * @return Vector closest keyframes
     */
    std::vector<std::shared_ptr<Keyframe>> get_close_keyframes(const Mat44_t& pose_cw,
                                                               const double distance_threshold,
                                                               const double angle_threshold) const;

    /**
     * Get the number of keyframes
     * @return
     */
    unsigned get_num_keyframes() const;

    /**
     * Get all of the landmarks in the database
     * @return
     */
    std::vector<std::shared_ptr<Landmark>> get_all_landmarks() const;

    /**
     * Get the last Keyframe added to the database
     * @return shared pointer to the last Keyframe added to the database
     */
    std::shared_ptr<Keyframe> get_last_inserted_keyframe() const;

    /**
     * Add spanning root
     */
    void add_spanning_root(std::shared_ptr<Keyframe>& Keyframe);

    /**
     * Get spanning roots
     */
    std::vector<std::shared_ptr<Keyframe>> get_spanning_roots();

    /**
     * Get the number of landmarks
     * @return
     */
    unsigned int get_num_landmarks() const;

    /**
     * Get minimum threshold for covisibility graph connection
     * @return minimum threshold for covisibility graph connection
     */
    unsigned int get_min_num_shared_lms() const;

    /**
     * Update Frame statistics
     * @param frm
     * @param is_lost
     */
    void update_frame_statistics(const data::Frame& frm, const bool is_lost) {
        std::lock_guard<std::mutex> lock(mtx_map_access_);
        frm_stats_.update_frame_statistics(frm, is_lost);
    }

    /**
     * Replace a Keyframe which will be erased in Frame statistics
     * @param old_keyfrm
     * @param new_keyfrm
     */
    void replace_reference_keyframe(const std::shared_ptr<data::Keyframe>& old_keyfrm, const std::shared_ptr<data::Keyframe>& new_keyfrm) {
        std::lock_guard<std::mutex> lock(mtx_map_access_);
        frm_stats_.replace_reference_keyframe(old_keyfrm, new_keyfrm);
    }

    /**
     * Get Frame statistics
     * @return
     */
    FrameStatistics get_frame_statistics() const {
        std::lock_guard<std::mutex> lock(mtx_map_access_);
        return frm_stats_;
    }

    /**
     * Clear the database
     */
    void clear();

    /**
     * Load keyframes and landmarks from JSON
     * @param cam_db
     * @param orb_params_db
     * @param json_keyfrms
     * @param json_landmarks
     */
    void from_json(CameraDatabase* cam_db, OrbParamsDatabase* orb_params_db,
                   const nlohmann::json& json_keyfrms, const nlohmann::json& json_landmarks);

    /**
     * Dump keyframes and landmarks as JSON
     * @param json_keyfrms
     * @param json_landmarks
     */
    void to_json(nlohmann::json& json_keyfrms, nlohmann::json& json_landmarks) const;

    /**
     * Load keyframes and landmarks from database
     */
    bool from_db(sqlite3* db,
                 CameraDatabase* cam_db,
                 OrbParamsDatabase* orb_params_db);

    /**
     * Dump keyframes and landmarks to database
     */
    bool to_db(sqlite3* db) const;

    //! mutex for locking ALL access to the database
    //! (NOTE: cannot used in MapDatabase class)
    static std::mutex mtx_database_;

    //! next ID
    std::atomic<unsigned int> next_keyframe_id_{0};
    std::atomic<unsigned int> next_landmark_id_{0};

private:
    /**
     * Decode JSON and register Keyframe information to the map database
     * (NOTE: objects which are not constructed yet will be set as nullptr)
     * @param cam_db
     * @param orb_params_db
     * @param id
     * @param json_keyfrm
     */
    void register_keyframe(CameraDatabase* cam_db, OrbParamsDatabase* orb_params_db,
                           const unsigned int id, const nlohmann::json& json_keyfrm);

    /**
     * Decode JSON and register Landmark information to the map database
     * (NOTE: objects which are not constructed yet will be set as nullptr)
     * @param id
     * @param json_landmark
     */
    void register_landmark(const unsigned int id, const nlohmann::json& json_landmark);

    /**
     * Decode JSON and register essential graph information
     * (NOTE: Keyframe database must be completely constructed before calling this function)
     * @param id
     * @param json_keyfrm
     */
    void register_graph(const unsigned int id, const nlohmann::json& json_keyfrm);

    /**
     * Decode JSON and register Keyframe-Landmark associations
     * (NOTE: Keyframe and Landmark database must be completely constructed before calling this function)
     * @param keyfrm_id
     * @param json_keyfrm
     */
    void register_association(const unsigned int keyfrm_id, const nlohmann::json& json_keyfrm);

    bool load_keyframes_from_db(sqlite3* db,
                                const std::string& table_name,
                                CameraDatabase* cam_db,
                                OrbParamsDatabase* orb_params_db);
    bool load_landmarks_from_db(sqlite3* db, const std::string& table_name);
    void load_association_from_stmt(sqlite3_stmt* stmt);
    bool load_associations_from_db(sqlite3* db, const std::string& table_name);
    bool save_keyframes_to_db(sqlite3* db, const std::string& table_name) const;
    bool save_landmarks_to_db(sqlite3* db, const std::string& table_name) const;
    static std::vector<std::pair<std::string, std::string>> association_columns() {
        return std::vector<std::pair<std::string, std::string>>{
            {"lm_ids", "BLOB"},
            {"span_parent", "INTEGER"},
            {"n_spanning_children", "INTEGER"},
            {"spanning_children", "BLOB"},
            {"n_loop_edges", "INTEGER"},
            {"loop_edges", "BLOB"}};
    };
    bool bind_association_to_stmt(sqlite3_stmt* stmt,
                                  const std::shared_ptr<Keyframe>& keyfrm) const;
    bool save_associations_to_db(sqlite3* db, const std::string& table_name) const;

    //! mutex for mutual exclusion controll between class methods
    mutable std::mutex mtx_map_access_;

    //-----------------------------------------
    // Keyframe and Landmark database

    //! IDs and keyframes
    std::unordered_map<unsigned int, std::shared_ptr<Keyframe>> keyframes_;
    //! IDs and landmarks
    std::unordered_map<unsigned int, std::shared_ptr<Landmark>> landmarks_;

    //! spanning roots
    std::vector<std::shared_ptr<Keyframe>> spanning_roots_;

    //! The last Keyframe added to the database
    std::shared_ptr<Keyframe> last_inserted_keyfrm_ = nullptr;

    //! local landmarks
    std::vector<std::shared_ptr<Landmark>> local_landmarks_;

    //! keyframes with id less than or equal to fixed_keyframe_id_threshold are not optimized
    unsigned int fixed_keyframe_id_threshold_ = 0;

    //-----------------------------------------
    // parameters for global/local mapping (optimization)

    //! minimum threshold for covisibility graph connection
    const unsigned int min_num_shared_lms_ = 15;

    //-----------------------------------------
    // Frame statistics for odometry evaluation

    //! Frame statistics
    FrameStatistics frm_stats_;
};

} // namespace data
}} // namespace vo // namespace uavloc
