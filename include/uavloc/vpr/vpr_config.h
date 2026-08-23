#pragma once

// Cấu hình module vpr, đọc từ node `VPR:` của file mission YAML.
//
// ⚠ KHÔNG đi qua core::SystemConfig. SystemConfig chỉ có ba sub-config (vo,
// fusion, camera) và không đọc node nào của vpr/anchor. Module này được dựng ở
// tầng application/driver rồi gắn vào, đúng khuôn mà tests/driver_common.cpp làm
// với anchor::FakeAnchorConfig.
//
// Node `VPR:` đã tồn tại sẵn trong config/uavloc_*.yaml như placeholder chưa ai
// đọc (model_path / input_width / match_threshold / top_k). Các khoá dưới đây
// THAY THẾ bộ đó; khoá cũ bị bỏ qua lặng lẽ theo mẫu `.as<T>(default)`.

#include <yaml-cpp/yaml.h>

#include <string>

namespace uavloc::vpr {

struct VprConfig {
    //! ── Artifact. Đường dẫn tuyệt đối; thiếu file thì setup() trả false.
    std::string model_path;       //!< .onnx của backbone
    std::string vocab_bin_path;   //!< codebook VLAD, float32 thô
    std::string pca_path;         //!< pca.yml (cv::FileStorage), CÓ whitening
    std::string database_path;    //!< .vprdb

    //! ── Tiền xử lý. Phải khớp shape tĩnh của .onnx, lệch thì setup() từ chối
    //! chứ không tự resize: vị trí embedding đã gấp thành hằng số trong đồ thị.
    int input_height = 224;
    int input_width  = 298;
    int patch_size   = 8;

    //! Hằng số lắp đặt camera [độ], cộng vào góc căn Bắc.
    //! ⚠ Với các log MUN-FRL, giá trị 180 CHÍNH LÀ gimbal_pan_deg (camera quay về
    //! đuôi). Bật use_gimbal_pan đồng thời để yaw_offset_deg = 180 là đếm hai lần.
    double yaw_offset_deg = 0.0;
    //! Cộng gimbal_pan_deg của telemetry vào góc căn Bắc. Đúng cho chế độ sản
    //! phẩm; tắt khi tái lập số liệu của một eval set đã khai yaw_offset_deg.
    bool use_gimbal_pan = true;

    //! ── Truy hồi.
    int    top_k = 10;
    double search_radius_m = 500.0;

    //! ── Thiết bị. "cuda" hoặc "cpu".
    std::string device = "cuda";
    //! 0 = để ONNX Runtime tự chọn. Đặt 1 khi cần kết quả tất định.
    int intra_op_threads = 0;

    //! Đọc theo mẫu `node["key"].as<T>(default)` — khoá thiếu không bao giờ throw.
    // ── Khớp tinh (tuỳ chọn) ─────────────────────────────────────────────────
    // Bỏ trống `keypoint_model_path` hoặc `matcher_model_path` ⇒ tắt hẳn tầng
    // khớp tinh; `VprModule::has_fine()` trả false và `localize()` lùi về tâm ô.

    std::string keypoint_model_path;   //!< superpoint.onnx
    std::string matcher_model_path;    //!< superpoint_lightglue.onnx

    //! Cạnh ảnh vuông đưa vào SuperPoint/LightGlue.
    int fine_net_size = 512;
    //! Giữ lại bao nhiêu điểm mỗi ảnh. Đo được: 512 điểm cho LightGlue 5,7 ms,
    //! 1024 điểm 8,8 ms — gấp rưỡi thời gian.
    int fine_max_keypoints = 512;

    //! Canvas BEV: 508 px x 0,781 m/px = 397 m, xấp xỉ vệt phủ của một ô 399 m.
    double bev_gsd_m = 0.781;
    int    bev_px = 508;

    //! Số ô đem khớp. Đo được: top-3 cho kết quả BẰNG top-5 (trung vị 98,8 m cả
    //! hai), top-1 tụt xuống 128,1 m. Hạ xuống 3 cắt ~19 ms mà không mất gì.
    int fine_top_k = 5;

    double ransac_px = 5.0;
    int    ransac_iters = 5000;
    //! ⚠ RANSAC là thuật toán NGẪU NHIÊN dùng RNG toàn cục của OpenCV. Ghim hạt
    //! giống để hai lần bay cùng dữ liệu ra cùng kết quả.
    int    ransac_seed = 42;

    //! Dưới ngần này inlier thì coi khớp tinh thất bại và lùi về tâm ô. Đo được
    //! trên Yên Bái: 70,9 % số query vượt ngưỡng 15.
    int    min_inliers = 15;

    static VprConfig fromYaml(const YAML::Node& node);
};

} // namespace uavloc::vpr
