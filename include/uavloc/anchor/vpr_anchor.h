#pragma once

// VprAnchor — nhà sản xuất vị trí tuyệt đối THẬT, thay FakeAnchor đằng sau đúng
// AnchorInterface. `fusion` không biết fix đến từ đâu, nên việc thay này không
// đụng một dòng nào của back-end.
//
// Đây là nơi DUY NHẤT của track VPR biết tới hệ ENU: uavloc::vpr::VprModule chỉ
// nói lat/lon (.docs/reports/Phase2_roadmap.md §4).
//
// ⚠ BẮT BUỘC BẤT ĐỒNG BỘ. Suy luận DINO không thể chạy inline trên luồng
// pipeline. Hệ quả (core::SystemManager::onAnchorFix, system_manager.cpp:1113):
// callback của producer bất đồng bộ đi qua pushAbsoluteFix() và LẤY
// shared_lock(gate_mutex_) — nên callback phải cực rẻ và tuyệt đối không được
// giữ khoá nội bộ nào khi gọi.

#include "uavloc/anchor/anchor_interface.h"
#include "uavloc/sensor/geo_reference.h"
#include "uavloc/vpr/vpr_module.h"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace uavloc::anchor {

struct VprAnchorConfig {
    vpr::VprConfig vpr;

    //! Nhịp: nhiều nhất một truy vấn mỗi N yêu cầu KEYFRAME. Nhịp là việc của
    //! NHÀ SẢN XUẤT (anchor_interface.h:71-80), không phải của caller.
    int every_kf = 5;
    //! Chống lụt cho nhánh REINIT bypass; 0 = không giới hạn, đúng lý lẽ ở
    //! fake_anchor.h — chưa có phép đo nào biện minh cho một số khác.
    int reinit_min_kf_gap = 0;

    //! Bán kính tìm kiếm suy từ cov_pred khi prediction_valid:
    //!   r = clamp(prior_sigma_scale * sqrt(lambda_max(cov_pred)), min, max)
    double prior_sigma_scale = 3.0;
    double radius_min_m      = 300.0;
    double radius_max_m      = 5000.0;
    //! Khi !prediction_valid: không có tiên nghiệm nên quét rộng.
    double radius_no_prior_m = 5000.0;

    //! Cổng dữ liệu. Chuỗi này giả định ảnh NADIR; nghiêng quá ngưỡng thì từ chối.
    //!
    //! ⚠ Đây là góc LỆCH KHỎI NADIR, tức `|90 - gimbal_tilt_deg|` — vì
    //! `gimbal_tilt_deg` đo từ MẶT PHẲNG NGANG và ~90 mới là nhìn thẳng xuống
    //! (telemetry_data.h:26). Đo trên Yên Bái: tilt trung vị 84,2 ⇒ lệch nadir
    //! 5,8 độ; MUN-FRL để tilt = 90 hằng số ⇒ lệch 0.
    //! ⚠ [giả thuyết] — chưa có phép đo nào chốt giá trị 25. Để rộng và ghi log.
    double max_tilt_deg = 25.0;

    //! Trần của confidence khi CHƯA hiệu chuẩn (C7 mới có đường cong reliability).
    //! Giữ thấp để một fix chưa hiệu chuẩn không bao giờ tự nhận là đáng tin.
    double max_uncalibrated_confidence = 0.5;

    // ── Khớp tinh: cov và confidence suy từ SỐ INLIER ────────────────────────
    // Chỉ dùng khi `FinePose::valid` — tức homography đạt ngưỡng. Không đạt thì
    // vị trí vẫn là tâm ô và công thức lượng tử lưới cũ mới đúng.
    //
    // ⚠ CHƯA HIỆU CHUẨN. Ba số dưới đây lấy TỪ PHÂN BỐ SAI SỐ ĐO ĐƯỢC trên 237
    // query Yên Bái, không phải từ một mô hình nhiễu: trung vị sai số 74 m ứng
    // với trung vị 32 inlier, p90 inlier là 58. Đường cong reliability thật là
    // sản phẩm của C7; tới lúc đó đây chỉ là một xấp xỉ CÓ CƠ SỞ ĐO, và
    // `max_uncalibrated_confidence` vẫn chặn trần.

    //! σ [m] tại `fine_sigma_ref_inliers` inlier.
    double fine_sigma_ref_m = 74.0;
    //! Số inlier ứng với σ trên. Trung vị đo được.
    double fine_sigma_ref_inliers = 32.0;
    //! Sàn σ: dưới mức này thì ta đang tự nhận chính xác hơn cả những gì đã đo.
    double fine_sigma_min_m = 15.0;

    //! Số inlier để confidence chạm trần. p90 đo được là 58.
    double fine_confidence_full_inliers = 58.0;

    //! Nội tại camera của khung THÔ. Driver lấy từ `core::SystemConfig::camera`.
    //! Bốn double thay vì `sensor::CameraIntrinsics` để header này khỏi kéo thêm
    //! phụ thuộc. fx <= 0 ⇒ khớp tinh không chạy được và anchor lùi về tâm ô.
    double camera_fx = 0.0, camera_fy = 0.0, camera_cx = 0.0, camera_cy = 0.0;
    //! Fix dưới ngưỡng này không phát đi. 0 = phát hết, để
    //! fusion::FusionConfig::fix_min_confidence quyết định.
    double min_confidence_emit = 0.0;

    static VprAnchorConfig fromYaml(const YAML::Node& node);
};

//! Bộ đếm kết quả, đặt tên theo khuôn FakeAnchorStats để driver in chung được.
//! requested = accepted + dropped_busy + dropped_not_running.
struct VprAnchorStats {
    unsigned long long requested = 0;
    unsigned long long accepted  = 0;
    unsigned long long dropped_busy = 0;
    unsigned long long dropped_not_running = 0;

    unsigned long long skipped_cadence      = 0;
    unsigned long long skipped_no_telemetry = 0;
    unsigned long long skipped_no_origin    = 0;
    unsigned long long skipped_tilt         = 0;

    unsigned long long retrieved      = 0;   //!< truy hồi trả về ứng viên
    unsigned long long unmatched      = 0;   //!< không ứng viên nào qua cổng
    unsigned long long low_confidence = 0;   //!< bị chặn bởi min_confidence_emit
    unsigned long long emitted        = 0;

    unsigned long long reinit_requested = 0;
    unsigned long long reinit_bypassed  = 0;
    unsigned long long reinit_throttled = 0;

    double last_query_ms = 0.0;
    double p50_query_ms  = 0.0;
};

class VprAnchor final : public AnchorInterface {
public:
    explicit VprAnchor(VprAnchorConfig cfg);
    ~VprAnchor() override;
    VprAnchor(const VprAnchor&) = delete;
    VprAnchor& operator=(const VprAnchor&) = delete;

    bool setup() override;
    bool start() override;
    void stop() override;                                //!< idempotent; join xong mới trả về
    void setResultCallback(ResultCallback cb) override;
    void setEnuOrigin(double lat0_deg, double lon0_deg) override;
    bool requestFix(const AnchorQuery& q) override;      //!< KHÔNG bao giờ chặn

    VprAnchorStats stats() const;

    //! Móc QUAN SÁT, gọi ngay trước khi fix đi ra `setResultCallback`.
    //!
    //! Tồn tại vì driver có viewer cần đánh dấu vị trí fix, mà `AnchorInterface`
    //! không có móc nào cho việc đó và `SystemManager::setup()` thì GHI ĐÈ
    //! `setResultCallback` (system_manager.cpp:246) — nên bọc callback kết quả
    //! là không được. `FakeAnchor::setGenerationCallback` giải cùng bài toán này
    //! theo cùng cách: một móc chỉ có ở lớp con.
    //!
    //! ⚠ Gọi trên LUỒNG WORKER, trong lúc anchor đang chạy. Đừng chặn ở đây.
    void setEmitCallback(std::function<void(const AbsoluteFix&)> cb);

    //! Thời gian từng tầng của lần truy hồi gần nhất — dùng cho hồ sơ hiệu năng.
    vpr::VprStageTiming last_stage_timing() const;

private:
    void worker_loop();
    void run_once(const AnchorQuery& q);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::anchor
