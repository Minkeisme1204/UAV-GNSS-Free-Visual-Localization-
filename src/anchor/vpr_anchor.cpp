#include "uavloc/anchor/vpr_anchor.h"

#include <spdlog/spdlog.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace uavloc::anchor {
namespace {

//! Sàn cho trị riêng nhỏ nhất của cov [m²].
//!
//! fusion::push_absolute_fix validate isUsableCovariance() và DROP LẶNG LẼ khi
//! không đạt (fusion_module.cpp:149-157). Một cov suy biến vì thế biến mất không
//! dấu vết — nên ép SPD ở đây, không hy vọng phía dưới tha.
constexpr double MIN_COV_EIGENVALUE_M2 = 1.0;

double median_of(std::deque<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

} // namespace

VprAnchorConfig VprAnchorConfig::fromYaml(const YAML::Node& node) {
    VprAnchorConfig c;
    if (!node) return c;
    c.vpr = vpr::VprConfig::fromYaml(node["VPR"] ? node["VPR"] : node);
    c.every_kf          = node["every_kf"].as<int>(c.every_kf);
    c.reinit_min_kf_gap = node["reinit_min_kf_gap"].as<int>(c.reinit_min_kf_gap);
    c.prior_sigma_scale = node["prior_sigma_scale"].as<double>(c.prior_sigma_scale);
    c.radius_min_m      = node["radius_min_m"].as<double>(c.radius_min_m);
    c.radius_max_m      = node["radius_max_m"].as<double>(c.radius_max_m);
    c.radius_no_prior_m = node["radius_no_prior_m"].as<double>(c.radius_no_prior_m);
    c.max_tilt_deg      = node["max_tilt_deg"].as<double>(c.max_tilt_deg);
    c.max_uncalibrated_confidence =
        node["max_uncalibrated_confidence"].as<double>(c.max_uncalibrated_confidence);
    c.min_confidence_emit = node["min_confidence_emit"].as<double>(c.min_confidence_emit);
    return c;
}

struct VprAnchor::Impl {
    VprAnchorConfig cfg;
    vpr::VprModule  vpr;

    sensor::GeoReferencer geo;
    std::atomic<bool> origin_set{false};

    // MỘT KHE DUY NHẤT, không hàng đợi. Một query cũ là vô giá trị: hàng đợi chỉ
    // để dành sẵn những phép đo đã hết hạn trước khi tới lượt được xử lý.
    mutable std::mutex      slot_mutex;
    std::condition_variable slot_cv;
    std::optional<AnchorQuery> pending;

    std::atomic<bool> running{false};
    std::atomic<bool> busy{false};
    bool ready = false;

    mutable std::mutex cb_mutex;
    ResultCallback     result_cb;
    std::function<void(const AbsoluteFix&)> emit_cb;

    std::thread worker;

    mutable std::mutex stats_mutex;
    VprAnchorStats     stats;
    std::deque<double> query_ms;
    vpr::VprStageTiming last_stage;

    int kf_counter = 0;
    int last_emit_seq = -1000000;
    int last_reinit_seq = -1000000;
    bool have_reinit = false;
};

VprAnchor::VprAnchor(VprAnchorConfig cfg) : impl_(std::make_unique<Impl>()) {
    impl_->cfg = std::move(cfg);
}

VprAnchor::~VprAnchor() { stop(); }

bool VprAnchor::setup() {
    // Không sinh thread, không chặn trên thiết bị — hợp đồng anchor_interface.h:33.
    impl_->ready = impl_->vpr.setup(impl_->cfg.vpr);
    if (!impl_->ready) {
        spdlog::error("anchor[VPR] setup thất bại: {}", impl_->vpr.last_error());
        return false;
    }
    spdlog::info("anchor[VPR] sẵn sàng — {} ô ('{}'), bước lưới {:.1f} m, "
                 "vệt phủ {:.1f} m, nhịp mỗi {} keyframe",
                 impl_->vpr.database_size(), impl_->vpr.database_name(),
                 impl_->vpr.tile_stride_m(), impl_->vpr.tile_footprint_m(),
                 impl_->cfg.every_kf);
    return true;
}

bool VprAnchor::start() {
    if (!impl_->ready) {
        spdlog::error("anchor[VPR] start() trước khi setup() thành công");
        return false;
    }
    if (impl_->running.exchange(true)) return true;   // đã chạy
    impl_->worker = std::thread([this] { worker_loop(); });
    return true;
}

void VprAnchor::stop() {
    if (!impl_->running.exchange(false)) {
        if (impl_->worker.joinable()) impl_->worker.join();
        return;
    }
    impl_->slot_cv.notify_all();
    if (impl_->worker.joinable()) impl_->worker.join();
    // Sau điểm này KHÔNG còn callback nào đang bay — đó là thứ cho phép
    // SystemManager tháo consumer an toàn (anchor_interface.h:42-45).
    const auto s = stats();
    spdlog::info("anchor[VPR] dừng — yêu cầu {}, nhận {}, phát {} | bỏ: nhịp {}, "
                 "bận {}, chưa có gốc {}, thiếu telemetry {}, nghiêng {}",
                 s.requested, s.accepted, s.emitted, s.skipped_cadence, s.dropped_busy,
                 s.skipped_no_origin, s.skipped_no_telemetry, s.skipped_tilt);
    spdlog::info("anchor[VPR] REINIT: yêu cầu {}, bypass {}, bị chặn {} | "
                 "truy vấn trung vị {:.1f} ms",
                 s.reinit_requested, s.reinit_bypassed, s.reinit_throttled, s.p50_query_ms);
}

void VprAnchor::setResultCallback(ResultCallback cb) {
    std::lock_guard<std::mutex> lk(impl_->cb_mutex);
    impl_->result_cb = std::move(cb);
}

void VprAnchor::setEnuOrigin(double lat0_deg, double lon0_deg) {
    // Đến GIỮA CHỪNG lúc chạy, tại fused pose đầu tiên trùng telemetry khả dụng
    // (system_manager.cpp:1088). Trước thời điểm đó producer phải im lặng.
    impl_->geo.init(lat0_deg, lon0_deg, 0.0, 0.0);
    impl_->origin_set.store(true);
    spdlog::info("anchor[VPR] nhận gốc ENU: {:.6f}, {:.6f}", lat0_deg, lon0_deg);
}

bool VprAnchor::requestFix(const AnchorQuery& q) {
    {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.requested;
        if (q.reason == AnchorRequestReason::REINIT) ++impl_->stats.reinit_requested;
    }
    if (!impl_->running.load()) {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.dropped_not_running;
        return false;
    }

    ++impl_->kf_counter;
    const int seq = impl_->kf_counter;

    // ── Cổng nhịp, và nhánh REINIT ───────────────────────────────────────────
    bool bypass = false;
    if (q.reason == AnchorRequestReason::REINIT) {
        const int gap = impl_->cfg.reinit_min_kf_gap;
        if (gap > 0 && impl_->have_reinit && (seq - impl_->last_reinit_seq) < gap) {
            // Bị chống lụt chặn -> RƠI XUỐNG thành yêu cầu nhịp thường, KHÔNG vứt.
            // Cùng ngữ nghĩa với FakeAnchor để hai producer so được với nhau.
            std::lock_guard<std::mutex> lk(impl_->stats_mutex);
            ++impl_->stats.reinit_throttled;
        } else {
            bypass = true;
        }
    }
    if (!bypass && (seq - impl_->last_emit_seq) < impl_->cfg.every_kf) {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.skipped_cadence;
        return true;   // "nhận" nhưng không tới lượt — không phải lỗi
    }

    // ── Cổng dữ liệu. Áp dụng NHƯ NHAU cho REINIT: bypass chỉ tha cổng nhịp ──
    // agl_m <= 0 nghĩa là khung không có telemetry dùng được (anchor_query.h:73).
    // Không telemetry -> không yaw -> north_align xoay bừa -> descriptor vô nghĩa.
    if (!(q.agl_m > 0.0)) {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.skipped_no_telemetry;
        return true;
    }
    if (!impl_->origin_set.load()) {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.skipped_no_origin;
        return true;
    }
    // ⚠ `gimbal_tilt_deg` đo TỪ MẶT PHẲNG NGANG, ~90 = nadir (telemetry_data.h:26),
    // nên góc lệch nadir là |90 - tilt|, KHÔNG phải |tilt|. Bản đầu viết
    // `std::abs(q.gimbal_tilt_deg) > max_tilt_deg` và vì thế loại 100 % số khung
    // trên mọi bộ dữ liệu thật (Yên Bái tilt ≈ 84, MUN-FRL tilt = 90 hằng số).
    // Không lộ ra vì test duy nhất chạm tới lớp này đặt tilt = 0.
    if (std::abs(90.0 - q.gimbal_tilt_deg) > impl_->cfg.max_tilt_deg) {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.skipped_tilt;
        return true;
    }

    // ── Khe đơn ──────────────────────────────────────────────────────────────
    // "Bận" = worker đang chạy HOẶC khe đã có người chờ. Kiểm mỗi cờ `busy` là
    // chưa đủ: worker chỉ đặt nó khi NHẬN việc, nên các yêu cầu dồn dập tới trước
    // lúc worker thức sẽ ghi đè `pending` trong im lặng mà vẫn được báo là "đã
    // nhận" — caller không có cách nào biết phép đo của mình đã bị vứt.
    {
        std::lock_guard<std::mutex> lk(impl_->slot_mutex);
        if (impl_->busy.load() || impl_->pending.has_value()) {
            std::lock_guard<std::mutex> sl(impl_->stats_mutex);
            ++impl_->stats.dropped_busy;
            return false;   // "không nhận", đúng nghĩa anchor_interface.h:76
        }
        // cv::Mat đếm tham chiếu, nhưng khung hình thuộc về pipeline và có thể bị
        // ghi đè trước khi worker chạm tới. Nhịp đã lọc nên clone chỉ xảy ra thưa.
        AnchorQuery copy = q;
        copy.image = q.image.clone();
        impl_->pending = std::move(copy);
    }
    impl_->last_emit_seq = seq;
    if (bypass) {
        impl_->last_reinit_seq = seq;
        impl_->have_reinit = true;
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.reinit_bypassed;
    }
    {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.accepted;
    }
    impl_->slot_cv.notify_one();
    return true;
}

void VprAnchor::worker_loop() {
    while (true) {
        AnchorQuery q;
        {
            std::unique_lock<std::mutex> lk(impl_->slot_mutex);
            impl_->slot_cv.wait(lk, [this] {
                return impl_->pending.has_value() || !impl_->running.load();
            });
            if (!impl_->running.load() && !impl_->pending.has_value()) return;
            if (!impl_->pending.has_value()) continue;
            q = std::move(*impl_->pending);
            impl_->pending.reset();
        }
        impl_->busy.store(true);
        run_once(q);
        impl_->busy.store(false);
    }
}

void VprAnchor::run_once(const AnchorQuery& q) {
    const auto t0 = std::chrono::steady_clock::now();

    // Bán kính từ hiệp phương sai của fusion. Tiên nghiệm CHỈ dùng để chọn vùng
    // tìm, KHÔNG BAO GIỜ để chấm điểm kết quả (absolute_fix.h:36-37).
    double radius = impl_->cfg.radius_no_prior_m;
    if (q.prediction_valid) {
        const double lam = q.cov_pred.eigenvalues().real().maxCoeff();
        radius = std::clamp(impl_->cfg.prior_sigma_scale * std::sqrt(std::max(lam, 0.0)),
                            impl_->cfg.radius_min_m, impl_->cfg.radius_max_m);
    }
    impl_->cfg.vpr.search_radius_m = radius;

    const sensor::LatLonAlt prior =
        impl_->geo.latlon_from_enu(Eigen::Vector3d(q.xy_enu_pred.x(), q.xy_enu_pred.y(), 0.0));
    const double align = impl_->vpr.align_degrees(q.yaw_deg, q.gimbal_pan_deg);

    // Khớp tinh nếu có: nó làm trọn nắn BEV -> truy hồi -> khớp -> toạ độ, và
    // trả luôn kết quả truy hồi để khỏi chạy lại DINO+VLAD+PCA lần nữa.
    // Thiếu nội tại camera thì không nắn BEV được ⇒ đi đường truy hồi thuần.
    const bool use_fine = impl_->vpr.has_fine() && impl_->cfg.camera_fx > 0.0;
    vpr::RetrievalOutcome out;
    vpr::FinePose fine;
    if (use_fine) {
        vpr::FineQueryInput in;
        in.roll_deg  = q.roll_deg;
        in.pitch_deg = q.pitch_deg;
        in.yaw_deg   = q.yaw_deg;
        in.gimbal_pan_deg  = q.gimbal_pan_deg;
        in.gimbal_tilt_deg = q.gimbal_tilt_deg;
        in.height_agl_m = q.agl_m;
        in.fx = impl_->cfg.camera_fx;
        in.fy = impl_->cfg.camera_fy;
        in.cx = impl_->cfg.camera_cx;
        in.cy = impl_->cfg.camera_cy;
        fine = impl_->vpr.localize(q.image, in, prior.lat, prior.lon, &out);
    } else {
        out = impl_->vpr.retrieve(q.image, align, prior.lat, prior.lon);
    }

    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        impl_->stats.last_query_ms = ms;
        impl_->query_ms.push_back(ms);
        if (impl_->query_ms.size() > 200) impl_->query_ms.pop_front();
        impl_->stats.p50_query_ms = median_of(impl_->query_ms);
        impl_->last_stage = impl_->vpr.last_timing();
        if (out.valid) ++impl_->stats.retrieved; else ++impl_->stats.unmatched;
    }
    if (!out.valid) {
        spdlog::debug("anchor[VPR] frame={} không có ứng viên nào qua cổng", q.frame_id);
        return;
    }

    // ── Vị trí: TÂM Ô HẠNG 1 ─────────────────────────────────────────────────
    // Không dùng trọng tâm có trọng số của top-K: đo được trên bộ eval, cách đó
    // cho kết quả XẤU HƠN. Top-K chỉ dùng để dựng cov.
    const auto& top = out.matches.front();
    // Khớp tinh đạt ngưỡng ⇒ toạ độ LIÊN TỤC trong ô; không đạt ⇒ tâm ô hạng 1,
    // đúng hành vi của bản chưa có tầng tinh. `FinePose` mang sẵn cả hai trường
    // hợp nên ở đây không phải rẽ nhánh.
    const double fix_lat = fine.tile_id >= 0 ? fine.latitude : top.latitude;
    const double fix_lon = fine.tile_id >= 0 ? fine.longitude : top.longitude;
    const Eigen::Vector3d p = impl_->geo.enu(fix_lat, fix_lon, 0.0);

    // ── cov: hai nguồn, cả hai đều là HÌNH HỌC, không phải điểm số ───────────
    const double s = impl_->vpr.tile_stride_m();
    Eigen::Matrix2d cov = (s * s / 12.0) * Eigen::Matrix2d::Identity();   // sàn lượng tử lưới

    // ⚠ Sàn `s²/12` là phương sai của phân bố ĐỀU trên một ô lưới — nó đúng khi
    // vị trí LÀ tâm ô, và vô nghĩa khi khớp tinh đã cho một điểm liên tục. Khi
    // đó σ suy từ SỐ INLIER: đó là đại lượng duy nhất ở tầng này tương quan với
    // sai số. Luật 1/√n là cách một ước lượng từ n phép đo độc lập co lại.
    if (fine.valid && fine.num_inliers > 0) {
        const double sigma = std::clamp(
            impl_->cfg.fine_sigma_ref_m *
                std::sqrt(impl_->cfg.fine_sigma_ref_inliers /
                          static_cast<double>(fine.num_inliers)),
            impl_->cfg.fine_sigma_min_m, s / std::sqrt(12.0));
        cov = (sigma * sigma) * Eigen::Matrix2d::Identity();
    } else if (out.matches.size() > 2) {
        Eigen::Matrix2d scatter = Eigen::Matrix2d::Zero();
        double wsum = 0.0;
        for (const auto& m : out.matches) {
            const Eigen::Vector3d e = impl_->geo.enu(m.latitude, m.longitude, 0.0);
            const Eigen::Vector2d d(e.x() - p.x(), e.y() - p.y());
            const double w = std::max(0.0, static_cast<double>(m.similarity));
            scatter += w * d * d.transpose();
            wsum += w;
        }
        if (wsum > 0.0) cov += scatter / wsum;
    }
    // Ép SPD: đối xứng hoá rồi kẹp trị riêng nhỏ nhất. Thiếu bước này thì
    // fusion drop LẶNG LẼ và không ai biết fix đã biến mất ở đâu.
    cov = 0.5 * (cov + cov.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    Eigen::Vector2d lam = es.eigenvalues();
    lam = lam.cwiseMax(MIN_COV_EIGENVALUE_M2);
    cov = es.eigenvectors() * lam.asDiagonal() * es.eigenvectors().transpose();

    // ── confidence: đặc trưng PHÂN BIỆT, KHÔNG phải cosine ───────────────────
    // Cosine không phải xác suất và không đơn điệu theo sai số: ảnh nhiều kết cấu
    // cho cosine cao với MỌI ô, ảnh đồng nhất cho cosine thấp với cả ô đúng — tức
    // nó tin cậy nhất đúng ở nơi bài toán khó nhất (absolute_fix.h:44-49).
    const double footprint = std::max(impl_->vpr.tile_footprint_m(), 1.0);
    double best_far = -1.0;      // ứng viên tốt nhất cách hạng 1 > 2 vệt phủ
    int    near_count = 0;       // số ứng viên trong 1 vệt phủ quanh hạng 1
    for (std::size_t i = 1; i < out.matches.size(); ++i) {
        const Eigen::Vector3d e = impl_->geo.enu(out.matches[i].latitude,
                                                 out.matches[i].longitude, 0.0);
        const double d = std::hypot(e.x() - p.x(), e.y() - p.y());
        if (d > 2.0 * footprint) best_far = std::max(best_far, static_cast<double>(out.matches[i].similarity));
        else if (d <= footprint) ++near_count;
    }
    const double margin = (best_far < 0.0) ? 0.0
                                           : std::clamp(top.similarity - best_far, 0.0, 1.0);
    const double consistency =
        static_cast<double>(near_count) / std::max<std::size_t>(1, out.matches.size() - 1);

    // ⚠ CHƯA HIỆU CHUẨN. Đường cong reliability là sản phẩm của C7; tới lúc đó
    // giá trị này bị chặn trên để một fix chưa hiệu chuẩn không tự nhận đáng tin.
    // Khớp tinh đạt ⇒ số inlier là chỉ báo TỐT HƠN margin/consistency: hai cái
    // đó đo độ PHÂN BIỆT của truy hồi, không nói gì về việc homography có khớp
    // hay không. Vẫn chặn trần vì cả hai đường đều chưa hiệu chuẩn.
    const double raw_conf =
        (fine.valid && impl_->cfg.fine_confidence_full_inliers > impl_->cfg.vpr.min_inliers)
            ? std::clamp((static_cast<double>(fine.num_inliers) -
                          static_cast<double>(impl_->cfg.vpr.min_inliers)) /
                             (impl_->cfg.fine_confidence_full_inliers -
                              static_cast<double>(impl_->cfg.vpr.min_inliers)),
                         0.0, 1.0)
            : 0.5 * (std::min(margin / 0.10, 1.0) + consistency);
    const double confidence = std::min(impl_->cfg.max_uncalibrated_confidence, raw_conf);

    if (confidence < impl_->cfg.min_confidence_emit) {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.low_confidence;
        spdlog::info("anchor[APPLY] frame={} ts={:.1f} result=LOW_CONFIDENCE conf={:.3f}",
                     q.frame_id, q.timestamp_msec, confidence);
        return;
    }

    AbsoluteFix fix;
    fix.timestamp_msec = q.timestamp_msec;   // KHÔNG đổi: đây là khoá nối về X(k)
    fix.xy_enu = Eigen::Vector2d(p.x(), p.y());
    fix.cov = cov;
    fix.confidence = confidence;
    fix.valid = true;

    // anchor[EMIT] là dòng DUY NHẤT producer sở hữu: anchor[REQ] do SystemManager
    // phát, anchor[APPLY] do fusion phát. Ghi sigma ở đây để khi fix biến mất
    // trong fusion còn truy được.
    spdlog::info("anchor[EMIT] frame={} ts={:.1f} xy_enu=[{:.2f}, {:.2f}] "
                 "sigma=[{:.1f}, {:.1f}] conf={:.3f} margin={:.3f} consist={:.2f} "
                 "tile={} sim={:.3f} gated={}/{} latency_ms={:.1f} "
                 "fine={} inl={} rank={}",
                 q.frame_id, q.timestamp_msec, fix.xy_enu.x(), fix.xy_enu.y(),
                 std::sqrt(cov(0, 0)), std::sqrt(cov(1, 1)), confidence, margin,
                 consistency, top.tile_id, top.similarity, out.num_gated, out.num_total, ms,
                 fine.valid ? "yes" : "no", fine.num_inliers, fine.rank);

    {
        std::lock_guard<std::mutex> lk(impl_->stats_mutex);
        ++impl_->stats.emitted;
    }
    // Gọi callback NGOÀI mọi khoá: nó đi qua pushAbsoluteFix() và sẽ lấy
    // shared_lock(gate_mutex_) của SystemManager.
    ResultCallback cb;
    std::function<void(const AbsoluteFix&)> emit;
    {
        std::lock_guard<std::mutex> lk(impl_->cb_mutex);
        cb = impl_->result_cb;
        emit = impl_->emit_cb;
    }
    // Móc quan sát TRƯỚC consumer thật: nếu fusion từ chối fix, viewer vẫn thấy
    // được nó đã được phát ra — đó chính là lúc cần nhìn nhất.
    if (emit) emit(fix);
    if (cb) cb(fix);
}

vpr::VprStageTiming VprAnchor::last_stage_timing() const {
    std::lock_guard<std::mutex> lk(impl_->stats_mutex);
    return impl_->last_stage;
}

void VprAnchor::setEmitCallback(std::function<void(const AbsoluteFix&)> cb) {
    std::lock_guard<std::mutex> lk(impl_->cb_mutex);
    impl_->emit_cb = std::move(cb);
}

VprAnchorStats VprAnchor::stats() const {
    std::lock_guard<std::mutex> lk(impl_->stats_mutex);
    return impl_->stats;
}

} // namespace uavloc::anchor
