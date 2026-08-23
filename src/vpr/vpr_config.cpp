#include "uavloc/vpr/vpr_config.h"

namespace uavloc::vpr {

VprConfig VprConfig::fromYaml(const YAML::Node& node) {
    VprConfig c;
    if (!node) {

        return c;   // node vắng mặt -> mặc định; setup() sẽ báo thiếu artifact
    }
    c.model_path      = node["model_path"].as<std::string>(c.model_path);
    c.vocab_bin_path  = node["vocab_bin_path"].as<std::string>(c.vocab_bin_path);
    c.pca_path        = node["pca_path"].as<std::string>(c.pca_path);
    c.database_path   = node["database_path"].as<std::string>(c.database_path);

    c.input_height    = node["input_height"].as<int>(c.input_height);
    c.input_width     = node["input_width"].as<int>(c.input_width);
    c.patch_size      = node["patch_size"].as<int>(c.patch_size);

    c.yaw_offset_deg  = node["yaw_offset_deg"].as<double>(c.yaw_offset_deg);
    c.use_gimbal_pan  = node["use_gimbal_pan"].as<bool>(c.use_gimbal_pan);

    c.top_k           = node["top_k"].as<int>(c.top_k);
    c.search_radius_m = node["search_radius_m"].as<double>(c.search_radius_m);

    c.device           = node["device"].as<std::string>(c.device);
    c.intra_op_threads = node["intra_op_threads"].as<int>(c.intra_op_threads);

    c.keypoint_model_path = node["keypoint_model_path"].as<std::string>(c.keypoint_model_path);
    c.matcher_model_path  = node["matcher_model_path"].as<std::string>(c.matcher_model_path);
    c.fine_net_size       = node["fine_net_size"].as<int>(c.fine_net_size);
    c.fine_max_keypoints  = node["fine_max_keypoints"].as<int>(c.fine_max_keypoints);
    c.bev_gsd_m           = node["bev_gsd_m"].as<double>(c.bev_gsd_m);
    c.bev_px              = node["bev_px"].as<int>(c.bev_px);
    c.fine_top_k          = node["fine_top_k"].as<int>(c.fine_top_k);
    c.ransac_px           = node["ransac_px"].as<double>(c.ransac_px);
    c.ransac_iters        = node["ransac_iters"].as<int>(c.ransac_iters);
    c.ransac_seed         = node["ransac_seed"].as<int>(c.ransac_seed);
    c.min_inliers         = node["min_inliers"].as<int>(c.min_inliers);
    return c;
}

} // namespace uavloc::vpr
