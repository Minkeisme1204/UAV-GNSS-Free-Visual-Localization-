#pragma once

#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/initialize/base.h"

#include <memory>

namespace uavloc {
namespace vo {

class Config;

namespace data {
class Frame;
class MapDatabase;
} // namespace data

namespace module {

// Initializer state
enum class InitializerState {
    NotReady,
    Initializing,
    Wrong,
    Succeeded
};

class Initializer {
public:
    Initializer() = delete;

    //! Constructor
    Initializer(data::MapDatabase* map_db,
                const YAML::Node& yaml_node);

    //! Destructor
    ~Initializer();

    //! Reset Initializer
    void reset();

    //! Get initialization state
    InitializerState get_state() const;

    //! Get the initial Frame ID which succeeded in initialization
    unsigned int get_initial_frame_id() const;

    //! Get the initial Frame stamp which succeeded in initialization
    double get_initial_frame_timestamp() const;

    //! Get whether to use a fixed seed for RANSAC
    bool get_use_fixed_seed() const;

    //! Initialize with the current Frame
    bool initialize(const camera::SetupType setup_type,
                    data::Frame& curr_frm);

private:
    //! map database
    data::MapDatabase* map_db_ = nullptr;
    //! Initializer status
    InitializerState state_ = InitializerState::NotReady;

    //! ID of Frame used for initialization (will be set after succeeded)
    unsigned int init_frm_id_ = 0;
    //! timestamp of Frame used for initialization (will be set after succeeded)
    double init_frm_stamp_ = 0.0;

    //-----------------------------------------
    // parameters

    //! max number of iterations of RANSAC (only for monocular Initializer)
    const unsigned int num_ransac_iters_;
    //! min number of valid pts (It should be greater than or equal to min_num_triangulated_)
    const unsigned int min_num_valid_pts_;
    //! min number of triangulated pts
    const unsigned int min_num_triangulated_pts_;
    //! min parallax (only for monocular Initializer)
    const float parallax_deg_thr_;
    //! reprojection error threshold (only for monocular Initializer)
    const float reproj_err_thr_;
    //! max number of iterations of BA (only for monocular Initializer)
    const unsigned int num_ba_iters_;
    //! initial scaling factor (only for monocular Initializer)
    const float scaling_factor_;
    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_;
    //! Gain threshold (for g2o)
    const float gain_threshold_;
    //! Verbosity (for g2o)
    const bool verbose_;

    //-----------------------------------------
    // for monocular camera model

    //! Create Initializer for monocular
    void create_initializer(data::Frame& curr_frm);

    //! Try to initialize a map with monocular camera setup
    bool try_initialize_for_monocular(data::Frame& curr_frm);

    //! Create an initial map with monocular camera setup
    bool create_map_for_monocular(data::Frame& curr_frm);

    //! Scaling up or down a initial map
    void scale_map(const std::shared_ptr<data::Keyframe>& init_keyfrm, const std::shared_ptr<data::Keyframe>& curr_keyfrm, const double scale);

    //! Initializer for monocular
    std::unique_ptr<initialize::Base> initializer_ = nullptr;
    //! initial Frame
    data::Frame init_frm_;
    //! coordinates of previously matched points to perform area-based matching
    std::vector<cv::Point2f> prev_matched_coords_;
    //! initial matching indices (index: idx of initial Frame, value: idx of current Frame)
    std::vector<int> init_matches_;

    //-----------------------------------------
    // for stereo or RGBD camera model

    //! Try to initialize a map with stereo or RGBD camera setup
    bool try_initialize_for_stereo(data::Frame& curr_frm);

    //! Create an initial map with stereo or RGBD camera setup
    bool create_map_for_stereo(data::Frame& curr_frm);
};

} // namespace module
}} // namespace vo // namespace uavloc

