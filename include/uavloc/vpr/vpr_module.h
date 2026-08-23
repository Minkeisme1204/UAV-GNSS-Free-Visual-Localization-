#pragma once

// VprModule — truy hồi ảnh cho uavloc: khung hình -> danh sách ô bản đồ ứng viên.
//
// THUẦN: đồng bộ, một luồng, không thread, không ENU, không GTSAM. Luồng, nhịp,
// chuyển ENU, cov và confidence thuộc về anchor::VprAnchor.
//
// Header này CỐ Ý chỉ để lọt cv::Mat, yaml-cpp, std và kiểu của chính uavloc:
// pimpl giữ ONNX Runtime và satvpr:: nằm hoàn toàn trong .cpp, nên chúng không
// rò vào interface đã export của libuavloc.so (tests/build_hygiene.sh kiểm).
//
// Một instance KHÔNG thread-safe. Chủ sở hữu chỉ được chạm vào nó từ một luồng.

#include "uavloc/vpr/vpr_config.h"
#include "uavloc/vpr/vpr_types.h"

#include <opencv2/core/mat.hpp>

#include <memory>
#include <string>

namespace uavloc::vpr {

//! Tư thế và nội tại cho một lần khớp tinh. POD thuần — không kiểu `satvpr::`
//! nào lọt qua ranh giới này (cổng K6 của build_hygiene.sh gác điều đó).
struct FineQueryInput {
    //! Tư thế thân máy bay [độ]. ⚠ `roll`/`pitch` là thứ nắn BEV cần mà bản
    //! trước không có: chúng đã nằm sẵn trong `sensor::TelemetryData`, chỉ chưa
    //! được chép sang `anchor::AnchorQuery`.
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;

    //! Gimbal [độ]. ⚠ `gimbal_tilt_deg` đo TỪ MẶT PHẲNG NGANG: 90 = nhìn thẳng
    //! xuống, KHÔNG phải 0.
    double gimbal_pan_deg = 0.0;
    double gimbal_tilt_deg = 90.0;

    //! Cao độ TRÊN MẶT ĐẤT [m] — `anchor::AnchorQuery::agl_m`, đã là AGL.
    double height_agl_m = 0.0;

    //! Nội tại của khung THÔ. Lấy từ `core::SystemConfig::camera`; không cần
    //! FOV theo khung vì trên bộ Yên Bái `fov = 62` ở cả 237 khung cho
    //! `fx = 1065,1`, trùng `Camera: fx: 1065.138` đến bốn chữ số.
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
};

class VprModule {
public:
    VprModule();
    ~VprModule();
    VprModule(const VprModule&) = delete;
    VprModule& operator=(const VprModule&) = delete;

    //! Nạp mô hình, codebook, PCA và cơ sở dữ liệu ô. Không sinh thread, không
    //! chặn trên thiết bị. Trả false kèm thông báo ở last_error() khi thiếu
    //! artifact hoặc khi cơ sở dữ liệu không thuộc về PCA đang nạp.
    bool setup(const VprConfig& config);
    bool is_ready() const;
    const std::string& last_error() const;

    //! Góc căn Bắc mà retrieve() sẽ dùng, theo cấu hình:
    //!   yaw_deg + (use_gimbal_pan ? gimbal_pan_deg : 0) + yaw_offset_deg
    double align_degrees(double yaw_deg, double gimbal_pan_deg) const;

    //! Truy hồi cho một khung hình.
    //!
    //! `bgr_frame` là khung THÔ (BGR uint8) — mọi phép xoay, cắt, resize do module
    //! tự làm; đưa vào ảnh đã tiền xử lý sẽ cho kết quả sai.
    //! `align_deg` là góc căn Bắc đã cộng đủ (dùng align_degrees()).
    //! `prior_lat/lon` là tâm cổng lọc không gian.
    //!
    //! ⚠ Bán kính lấy từ config. Ở uavloc tâm cổng là ước lượng ĐÃ TRÔI của
    //! fusion, khác bản Python vốn đặt tâm ở vị trí thật của query — nên số
    //! Recall đo offline KHÔNG chuyển thẳng sang được.
    RetrievalOutcome retrieve(const cv::Mat& bgr_frame, double align_deg,
                              double prior_lat, double prior_lon);

    //! Thời gian từng tầng của lần retrieve() gần nhất.
    const VprStageTiming& last_timing() const;

    //! Cơ sở dữ liệu có kho ảnh ô không (`.vprdb` v2).
    //!
    //! Khớp tinh phải chạy SuperPoint trên chính ô bản đồ, mà bản C++ không có
    //! GDAL nên không mở được GeoTIFF nguồn — pixel của ô phải nằm sẵn trong DB.
    //! false ⇒ `localize()` luôn trả `FinePose::valid == false` với toạ độ tâm ô,
    //! tức đúng hành vi của bản chưa có khớp tinh. Nâng cấp bằng:
    //!     python -m scripts.export_vprdb --set <tên> --upgrade
    bool has_fine() const;

    //! Truy hồi RỒI khớp tinh, trả thẳng toạ độ.
    //!
    //! Làm trọn chuỗi: nắn BEV từ tư thế -> truy hồi trên canvas -> top-K ->
    //! khớp SuperPoint+LightGlue với từng ô -> rerank theo inlier -> đẩy NADIR
    //! qua homography -> lat/lon.
    //!
    //! ⚠ `bgr_frame` là khung THÔ. Module tự nắn; đưa vào ảnh đã xoay là xoay
    //! hai lần.
    //! ⚠ KHÔNG gọi song song: đường khớp tinh dùng đệm tái dùng. `VprAnchor` có
    //! khe đơn và một worker duy nhất nên hợp đồng này thoả.
    //! `retrieval` != nullptr ⇒ nhận luôn kết quả truy hồi của chính lần chạy
    //! này. Cần vì tầng trên vẫn dùng top-K để dựng cov khi khớp tinh không đạt,
    //! mà gọi `retrieve()` lần nữa là chạy lại cả DINO+VLAD+PCA — 6,2 ms phí.
    FinePose localize(const cv::Mat& bgr_frame, const FineQueryInput& in,
                      double prior_lat, double prior_lon,
                      RetrievalOutcome* retrieval = nullptr);

    //! Thời gian từng tầng của lần localize() gần nhất.
    const FineStageTiming& last_fine_timing() const;

    //! Số ô trong cơ sở dữ liệu, và eval set đã dựng ra nó (chẩn đoán).
    long long database_size() const;
    const std::string& database_name() const;

    //! Bước lưới giữa hai tâm ô liền kề [m], và bề rộng vệt phủ của một ô [m].
    //!
    //! Cần cho tầng trên: ước lượng vị trí là TÂM ô hạng 1, mà vị trí thật rải
    //! đều trong một ô cạnh `s` ⇒ phương sai sàn `s²/12`. Đây là sàn CỨNG của
    //! truy hồi thuần, không mẹo nào hạ được — chỉ khớp tinh mới hạ.
    double tile_stride_m() const;
    double tile_footprint_m() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::vpr
