// test_vpr_anchor — VprAnchor sau đúng AnchorInterface.
//
// Theo khuôn tests/test_anchor.cpp: dựng producer ĐỘC LẬP, không SystemManager,
// tự gọi setup/start/setEnuOrigin/requestFix/stop. Số học đã có bộ parity riêng
// ở satvpr_core; ở đây kiểm HÀNH VI HỆ THỐNG — cổng, luồng, và các bất biến mà
// nếu sai thì fix sẽ biến mất lặng lẽ trong fusion.

#include "uavloc/anchor/vpr_anchor.h"

#include <Eigen/Eigenvalues>
#include <opencv2/imgcodecs.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

namespace {

int g_failed = 0;

bool check(bool ok, const std::string& name, const std::string& detail = "") {
    std::printf("  [%s] %s%s%s\n", ok ? " OK " : "FAIL", name.c_str(),
                detail.empty() ? "" : " — ", detail.c_str());
    if (!ok) ++g_failed;
    return ok;
}

bool exists(const std::string& p) {
    struct stat st {};
    return !p.empty() && ::stat(p.c_str(), &st) == 0;
}

uavloc::anchor::AnchorQuery make_query(unsigned id, double ts, const cv::Mat& img) {
    uavloc::anchor::AnchorQuery q;
    q.frame_id = id;
    q.timestamp_msec = ts;
    q.image = img;
    q.agl_m = 60.0;
    q.yaw_deg = 67.2153;
    q.gimbal_pan_deg = 180.0;
    // ⚠ 90 = NHÌN THẲNG XUỐNG. `gimbal_tilt_deg` đo từ mặt phẳng ngang
    // (telemetry_data.h:26), nên khung nadir bình thường là 90, không phải 0.
    // Bản đầu để 0 và vì thế che mất một lỗi thật: cổng nghiêng dùng |tilt|
    // thay vì |90 - tilt|, tức loại 100 % khung trên mọi bộ dữ liệu thật.
    q.gimbal_tilt_deg = 90.0;
    q.reason = uavloc::anchor::AnchorRequestReason::KEYFRAME;
    return q;
}

} // namespace

int main() {
    const std::string root = SATVPR_DATA_ROOT;
    const std::string d = root + "/dataset/eval/mun_ds6";

    uavloc::anchor::VprAnchorConfig cfg;
    cfg.vpr.model_path     = root + "/.weights/dino/dino_vits8_layer9_norm.onnx";
    cfg.vpr.vocab_bin_path = root + "/.weights/vlad/vocab_b2_flighttrack_all.bin";
    cfg.vpr.pca_path       = d + "/pca.yml";
    cfg.vpr.database_path  = d + "/mun_ds6.vprdb";
    cfg.vpr.yaw_offset_deg = 0.0;
    cfg.vpr.use_gimbal_pan = true;      // 67.2153 + 180 = góc căn Bắc của MUN
    cfg.vpr.device         = "cuda";
    cfg.every_kf           = 3;
    cfg.radius_no_prior_m  = 500.0;

    for (const auto& p : {cfg.vpr.model_path, cfg.vpr.vocab_bin_path,
                          cfg.vpr.pca_path, cfg.vpr.database_path}) {
        if (!exists(p)) { std::printf("BỎ QUA: thiếu %s\n", p.c_str()); return 0; }
    }
    const cv::Mat img = cv::imread(d + "/queries/00000.jpg", cv::IMREAD_COLOR);
    if (img.empty()) { std::printf("BỎ QUA: thiếu ảnh query\n"); return 0; }

    // Gốc ENU và vị trí thật của query 0 (query_latlon.csv dòng đầu).
    const double lat0 = 45.3231215, lon0 = -75.6674457;

    uavloc::anchor::VprAnchor anchor(cfg);

    // ── 1. Vòng đời ──────────────────────────────────────────────────────────
    check(!anchor.start(), "start() trước setup() bị từ chối");
    if (!check(anchor.setup(), "setup()")) return 1;

    std::mutex m;
    std::vector<uavloc::anchor::AbsoluteFix> fixes;
    std::atomic<std::thread::id> cb_thread{};
    anchor.setResultCallback([&](const uavloc::anchor::AbsoluteFix& f) {
        cb_thread.store(std::this_thread::get_id());
        std::lock_guard<std::mutex> lk(m);
        fixes.push_back(f);
    });

    check(!anchor.requestFix(make_query(0, 1000.0, img)), "requestFix trước start() -> false");
    check(anchor.start(), "start()");

    // ── 2. Chưa có gốc ENU thì TUYỆT ĐỐI im lặng ────────────────────────────
    // setEnuOrigin đến giữa chừng lúc chạy (system_manager.cpp:1088), nên producer
    // phải sống được qua nhiều keyframe đầu mà không phát gì.
    for (unsigned i = 1; i <= 9; ++i) anchor.requestFix(make_query(i, 1000.0 + i, img));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    {
        std::lock_guard<std::mutex> lk(m);
        check(fixes.empty(), "không phát fix nào khi chưa có gốc ENU");
    }
    check(anchor.stats().skipped_no_origin > 0, "đếm được skipped_no_origin",
          std::to_string(anchor.stats().skipped_no_origin));

    anchor.setEnuOrigin(lat0, lon0);

    // ── 3. Cổng dữ liệu ──────────────────────────────────────────────────────
    {
        auto q = make_query(100, 2000.0, img);
        q.agl_m = 0.0;                     // không có telemetry dùng được
        const auto before = anchor.stats().skipped_no_telemetry;
        anchor.requestFix(q);
        check(anchor.stats().skipped_no_telemetry == before + 1,
              "agl_m <= 0 bị chặn, không xoay bừa");

        // 60 từ mặt phẳng ngang = lệch nadir 30 độ > ngưỡng 25 ⇒ phải chặn.
        auto q2 = make_query(101, 2001.0, img);
        q2.gimbal_tilt_deg = 60.0;
        const auto b2 = anchor.stats().skipped_tilt;
        anchor.requestFix(q2);
        check(anchor.stats().skipped_tilt == b2 + 1,
              "lệch nadir 30 độ bị chặn");

        // TEST ÂM của chính cổng đó: khung NADIR phải ĐI QUA. Thiếu vế này thì
        // một cổng chặn tất cả vẫn xanh — đúng điều đã xảy ra với bản dùng
        // |tilt| thay vì |90 - tilt|.
        auto q3 = make_query(102, 2002.0, img);
        q3.gimbal_tilt_deg = 90.0;         // nhìn thẳng xuống
        const auto b3 = anchor.stats().skipped_tilt;
        anchor.requestFix(q3);
        check(anchor.stats().skipped_tilt == b3,
              "khung NADIR (tilt 90) KHÔNG bị cổng nghiêng chặn");

        // Và một khung nghiêng như thật của Yên Bái: tilt 84,2 ⇒ lệch 5,8 độ.
        auto q4 = make_query(103, 2003.0, img);
        q4.gimbal_tilt_deg = 84.2;
        const auto b4 = anchor.stats().skipped_tilt;
        anchor.requestFix(q4);
        check(anchor.stats().skipped_tilt == b4,
              "khung nghiêng thật của Yên Bái (tilt 84,2) đi qua được");
    }

    // ── 4. Nhịp ──────────────────────────────────────────────────────────────
    {
        const auto before = anchor.stats();
        for (unsigned i = 200; i < 200 + 9; ++i) {
            anchor.requestFix(make_query(i, 3000.0 + i, img));
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
        }
        const auto after = anchor.stats();
        const auto accepted = after.accepted - before.accepted;
        const auto skipped  = after.skipped_cadence - before.skipped_cadence;
        const auto busy     = after.dropped_busy - before.dropped_busy;
        // Kỳ vọng suy từ nguyên lý, không gõ cứng: 9 yêu cầu cách nhau 120 ms,
        // nhịp 3 -> nhận 3; phần còn lại phải rơi vào nhịp hoặc bận, không mất đi đâu.
        check(accepted == 3, "nhịp mỗi 3 keyframe -> nhận đúng 3",
              std::to_string(accepted));
        check(accepted + skipped + busy == 9, "9 yêu cầu được kế toán đủ",
              "nhận " + std::to_string(accepted) + " + nhịp " + std::to_string(skipped) +
              " + bận " + std::to_string(busy));
    }

    // ── 5. REINIT bỏ qua nhịp ────────────────────────────────────────────────
    {
        anchor.requestFix(make_query(300, 4000.0, img));   // đặt lại mốc nhịp
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        const auto before = anchor.stats();
        auto q = make_query(301, 4001.0, img);
        q.reason = uavloc::anchor::AnchorRequestReason::REINIT;
        anchor.requestFix(q);                              // ngay sát, nhịp sẽ chặn
        const auto after = anchor.stats();
        check(after.reinit_bypassed == before.reinit_bypassed + 1 &&
              after.skipped_cadence == before.skipped_cadence,
              "REINIT bỏ qua cổng nhịp");
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

    // ── 6. Khe đơn: bận thì trả false, không xếp hàng ────────────────────────
    {
        const auto before = anchor.stats().dropped_busy;
        for (unsigned i = 400; i < 406; ++i) {
            auto q = make_query(i, 5000.0 + i, img);
            q.reason = uavloc::anchor::AnchorRequestReason::REINIT;   // bỏ qua nhịp
            anchor.requestFix(q);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        check(anchor.stats().dropped_busy > before,
              "yêu cầu dồn dập -> dropped_busy, không xếp hàng",
              std::to_string(anchor.stats().dropped_busy - before));
    }

    // ── 7. Bất biến của fix — sai là fusion DROP LẶNG LẼ ────────────────────
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    {
        std::lock_guard<std::mutex> lk(m);
        if (!check(!fixes.empty(), "đã phát được fix", std::to_string(fixes.size()))) return 1;
        bool ts_ok = true, finite = true, spd = true, conf_ok = true, near = true;
        for (const auto& f : fixes) {
            ts_ok  &= (f.timestamp_msec > 0.0);
            finite &= f.xy_enu.allFinite() && f.cov.allFinite();
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(f.cov);
            spd &= (es.eigenvalues().minCoeff() > 0.0) &&
                   ((f.cov - f.cov.transpose()).cwiseAbs().maxCoeff() < 1e-12);
            conf_ok &= (f.confidence >= 0.0 && f.confidence <= cfg.max_uncalibrated_confidence);
            near &= (f.xy_enu.norm() < 2000.0);
        }
        check(ts_ok, "timestamp giữ nguyên, không bị đặt lại");
        check(finite, "xy_enu và cov hữu hạn");
        check(spd, "cov đối xứng xác định dương (fusion drop lặng lẽ nếu không)");
        check(conf_ok, "confidence trong [0, trần chưa hiệu chuẩn]",
              std::to_string(fixes.front().confidence));
        check(near, "vị trí nằm gần gốc ENU (hệ toạ độ đúng)",
              std::to_string(fixes.front().xy_enu.norm()) + " m");
        check(cb_thread.load() != std::this_thread::get_id(),
              "callback chạy trên LUỒNG KHÁC (producer bất đồng bộ)");
        const double sigma = std::sqrt(fixes.front().cov(0, 0));
        std::printf("        sigma_x = %.1f m (sàn lượng tử lưới đơn thuần "
                    "= stride/sqrt(12))\n", sigma);
    }

    // ── 8. stop() idempotent, không còn callback đang bay ───────────────────
    const auto before_stop = anchor.stats().emitted;
    anchor.stop();
    anchor.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    check(anchor.stats().emitted == before_stop, "sau stop() không còn fix nào bay ra");
    check(!anchor.requestFix(make_query(999, 9000.0, img)), "requestFix sau stop() -> false");

    const auto s = anchor.stats();
    std::printf("\n  yêu cầu %llu | nhận %llu | phát %llu | truy vấn trung vị %.0f ms\n",
                s.requested, s.accepted, s.emitted, s.p50_query_ms);
    std::printf("\n%s — %d hỏng\n", g_failed == 0 ? "ĐẠT" : "HỎNG", g_failed);
    return g_failed == 0 ? 0 : 1;
}
