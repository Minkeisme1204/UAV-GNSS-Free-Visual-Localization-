// test_vpr_flow — chạy một dòng chảy truy vấn thật qua anchor::VprAnchor và đo.
//
// Khác các test cổng: đây KHÔNG kiểm bất biến mà chạy đúng kịch bản vận hành —
// nạp một lần, rồi N khung hình thật đi qua worker thread — và đo ba thứ mà chỉ
// dữ liệu thật mới trả lời được:
//
//   1. SAI SỐ VỊ TRÍ so với groundtruth, trong chính hệ ENU mà fusion tiêu thụ;
//   2. HIỆU CHUẨN của cov — tỉ lệ sai số nằm trong 2σ, kỳ vọng ~95% nếu cov
//      trung thực. Đây là số quan trọng nhất: cov quá bi quan thì fusion đánh
//      trọng số thấp và ta vứt đi thông tin, cov quá lạc quan thì một fix sai
//      kéo lệch cả quỹ đạo;
//   3. ĐỘ TRỄ từng tầng ở trạng thái ổn định.
//
// Chạy được trên bất kỳ bộ dữ liệu nào có đủ: query_latlon.csv, queries/,
// pca.yml, <tên>.vprdb. Thiếu thì SOFT-SKIP (trả 0), đúng khuôn của repo.
//
// Ví dụ:
//   test_vpr_flow --dataset /duong/dan/eval/mun_ds6 --weights /duong/dan/.weights \
//                 --yaw-offset 180 --queries 20 --csv /tmp/flow.csv

#include "uavloc/anchor/vpr_anchor.h"
#include "uavloc/sensor/geo_reference.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace {

struct Row { int id = 0; double lat = 0, lon = 0, yaw = 0, alt = 0; };

bool exists(const std::string& p) {
    struct stat st {};
    return !p.empty() && ::stat(p.c_str(), &st) == 0;
}

std::string basename_of(std::string p) {
    while (!p.empty() && p.back() == '/') p.pop_back();
    const auto s = p.find_last_of('/');
    return s == std::string::npos ? p : p.substr(s + 1);
}

//! query_latlon.csv — cột bắt buộc: Query_id, Latitude, Longitude, Yaw.
//! Altitude_m là tuỳ chọn; thiếu thì dùng --agl.
std::vector<Row> load_queries(const std::string& path, int limit, double default_agl) {
    std::vector<Row> out;
    std::ifstream f(path);
    if (!f) return out;

    std::string line;
    if (!std::getline(f, line)) return out;
    std::vector<std::string> cols;
    {
        std::stringstream ss(line);
        std::string c;
        while (std::getline(ss, c, ',')) {
            while (!c.empty() && (c.back() == '\r' || c.back() == ' ')) c.pop_back();
            cols.push_back(c);
        }
    }
    const auto idx = [&](const char* name) -> int {
        for (std::size_t i = 0; i < cols.size(); ++i) if (cols[i] == name) return static_cast<int>(i);
        return -1;
    };
    const int i_id = idx("Query_id"), i_lat = idx("Latitude"), i_lon = idx("Longitude");
    const int i_yaw = idx("Yaw"), i_alt = idx("Altitude_m");
    if (i_id < 0 || i_lat < 0 || i_lon < 0 || i_yaw < 0) return out;

    while (std::getline(f, line) && (limit <= 0 || static_cast<int>(out.size()) < limit)) {
        std::vector<std::string> v;
        std::stringstream ss(line);
        std::string c;
        while (std::getline(ss, c, ',')) v.push_back(c);
        if (static_cast<int>(v.size()) <= std::max({i_id, i_lat, i_lon, i_yaw})) continue;
        Row r;
        r.id  = std::stoi(v[static_cast<std::size_t>(i_id)]);
        r.lat = std::stod(v[static_cast<std::size_t>(i_lat)]);
        r.lon = std::stod(v[static_cast<std::size_t>(i_lon)]);
        r.yaw = std::stod(v[static_cast<std::size_t>(i_yaw)]);
        r.alt = (i_alt >= 0 && static_cast<int>(v.size()) > i_alt)
                    ? std::stod(v[static_cast<std::size_t>(i_alt)]) : default_agl;
        out.push_back(r);
    }
    return out;
}

double quantile(std::vector<double> v, double q) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(q * static_cast<double>(v.size() - 1))];
}

const char* arg_of(int argc, char** argv, const char* key, const char* fallback) {
    for (int i = 1; i + 1 < argc; ++i) if (std::strcmp(argv[i], key) == 0) return argv[i + 1];
    return fallback;
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    const std::string dataset = arg_of(argc, argv, "--dataset", DEFAULT_DATASET_DIR);
    const std::string weights = arg_of(argc, argv, "--weights", DEFAULT_WEIGHTS_DIR);
    const std::string vocab   = arg_of(argc, argv, "--vocab", "vocab_b2_flighttrack_all.bin");
    const std::string model   = arg_of(argc, argv, "--model", "dino_vits8_layer9_norm.onnx");
    const std::string device  = arg_of(argc, argv, "--device", "cuda");
    const std::string csv_out = arg_of(argc, argv, "--csv", "");
    const int    n_queries    = std::atoi(arg_of(argc, argv, "--queries", "20"));
    const double yaw_offset   = std::atof(arg_of(argc, argv, "--yaw-offset", "0"));
    // Nội tại camera cho tầng khớp tinh. Mặc định là bộ Yên Bái (fov 62 độ trên
    // khung 1280x720 ⇒ fx 1065,138 — trùng Camera.fx trong YAML của uavloc).
    // Đặt --camera-fx 0 để TẮT khớp tinh và đo lại đường tâm ô.
    const double camera_fx    = std::atof(arg_of(argc, argv, "--camera-fx", "1065.138"));
    const double camera_cx    = std::atof(arg_of(argc, argv, "--camera-cx", "640"));
    const double camera_cy    = std::atof(arg_of(argc, argv, "--camera-cy", "360"));
    const double radius_m     = std::atof(arg_of(argc, argv, "--radius", "500"));
    const int    every_kf     = std::atoi(arg_of(argc, argv, "--every-kf", "1"));
    const double default_agl  = std::atof(arg_of(argc, argv, "--agl", "100"));
    //! Tiên nghiệm lệch đi ngần này mét — mô phỏng ước lượng ĐÃ TRÔI của fusion.
    //! 0 = trường hợp lạc quan (tâm cổng ở đúng vị trí thật), giống bộ eval Python.
    const double prior_bias_m = std::atof(arg_of(argc, argv, "--prior-bias", "0"));

    const std::string name = basename_of(dataset);
    uavloc::anchor::VprAnchorConfig cfg;
    cfg.vpr.model_path     = weights + "/dino/" + model;
    cfg.vpr.vocab_bin_path = weights + "/vlad/" + vocab;
    cfg.vpr.pca_path       = dataset + "/pca.yml";
    cfg.vpr.database_path  = dataset + "/" + name + ".vprdb";
    cfg.vpr.yaw_offset_deg = yaw_offset;
    // ⚠ yaw_offset chỉ được cộng MỘT nơi. Module sở hữu nó (align_degrees), nên
    // driver truyền q.yaw_deg THÔ. Cộng ở cả hai chỗ là đếm hai lần — đã đo:
    // với bộ MUN, yaw+360 làm sai số trung vị nhảy từ 25,8 m lên 163,5 m.
    cfg.vpr.use_gimbal_pan = false;   // gimbal_pan đã nằm trong yaw_offset của MUN
    cfg.vpr.device         = device;

    // ── Khớp tinh ────────────────────────────────────────────────────────────
    // Chỉ bật khi có ĐỦ ba thứ: hai model ONNX, DB v2 (có kho ảnh ô) và nội tại
    // camera. Thiếu bất kỳ cái nào thì anchor lùi về tâm ô — và đó vẫn là một
    // phép đo hợp lệ, chỉ là của đường cũ.
    cfg.vpr.keypoint_model_path = weights + "/superpoint.onnx";
    cfg.vpr.matcher_model_path  = weights + "/superpoint_lightglue.onnx";
    // Nội tại của bộ đang đo. Yên Bái: fov 62 độ trên khung 1280x720 ⇒ fx 1065,1,
    // trùng Camera.fx trong uavloc_yenbai800m_newvo.yaml. MUN dùng bộ khác nhưng
    // DB của nó còn là v1 nên khớp tinh không chạy — không ảnh hưởng.
    cfg.camera_fx = cfg.camera_fy = camera_fx;
    cfg.camera_cx = camera_cx;
    cfg.camera_cy = camera_cy;
    cfg.every_kf           = every_kf;
    cfg.radius_no_prior_m  = radius_m;
    cfg.radius_max_m       = radius_m;

    if (!exists(cfg.vpr.keypoint_model_path) || !exists(cfg.vpr.matcher_model_path)) {
        std::printf("  (thiếu model khớp tinh — chạy ở chế độ tâm ô)\n");
        cfg.vpr.keypoint_model_path.clear();
        cfg.vpr.matcher_model_path.clear();
    }

    for (const auto& p : {cfg.vpr.model_path, cfg.vpr.vocab_bin_path,
                          cfg.vpr.pca_path, cfg.vpr.database_path}) {
        if (!exists(p)) { std::printf("BỎ QUA: thiếu %s\n", p.c_str()); return 0; }
    }
    const auto rows = load_queries(dataset + "/query_latlon.csv", n_queries, default_agl);
    if (rows.empty()) {
        std::printf("BỎ QUA: không đọc được %s/query_latlon.csv (cần cột "
                    "Query_id, Latitude, Longitude, Yaw)\n", dataset.c_str());
        return 0;
    }

    uavloc::anchor::VprAnchor anchor(cfg);
    const auto t_setup = std::chrono::steady_clock::now();
    if (!anchor.setup()) { std::printf("setup thất bại\n"); return 1; }
    const double setup_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_setup).count();

    std::mutex mtx;
    std::condition_variable cv;
    std::vector<uavloc::anchor::AbsoluteFix> got;
    anchor.setResultCallback([&](const uavloc::anchor::AbsoluteFix& f) {
        std::lock_guard<std::mutex> lk(mtx);
        got.push_back(f);
        cv.notify_all();
    });
    if (!anchor.start()) { std::printf("start thất bại\n"); return 1; }

    // Gốc ENU = vị trí thật của truy vấn đầu. Trong hệ thống thật, SystemManager
    // đẩy gốc xuống tại fused pose đầu tiên trùng telemetry khả dụng.
    anchor.setEnuOrigin(rows[0].lat, rows[0].lon);
    uavloc::sensor::GeoReferencer geo;
    geo.init(rows[0].lat, rows[0].lon, 0.0, 0.0);

    std::printf("\n%s | %zu truy vấn | setup %.0f ms | tiên nghiệm lệch %.0f m\n\n",
                name.c_str(), rows.size(), setup_ms, prior_bias_m);
    std::printf("%5s %9s %9s %8s %8s %8s\n", "q", "sai_số", "sigma", "conf", "trễ_ms", "trong_2σ");
    std::printf("%s\n", std::string(56, '-').c_str());

    std::ofstream csv;
    if (!csv_out.empty()) {
        csv.open(csv_out);
        csv << "query_id,err_m,sigma_m,confidence,latency_ms,inside_2sigma\n";
    }

    std::vector<double> errs, sigmas, confs, lats;
    std::vector<double> t_pre, t_bb, t_agg, t_prj, t_srch;
    int emitted = 0, refused = 0, no_fix = 0;
    const auto t_flow = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < rows.size(); ++i) {
        const Row& r = rows[i];
        char fname[32];
        std::snprintf(fname, sizeof(fname), "%05d.jpg", r.id);
        const cv::Mat img = cv::imread(dataset + "/queries/" + fname, cv::IMREAD_COLOR);
        if (img.empty()) continue;

        uavloc::anchor::AnchorQuery q;
        q.frame_id        = static_cast<unsigned>(r.id);
        q.timestamp_msec  = 1000.0 * static_cast<double>(i);
        q.image           = img;
        q.agl_m           = r.alt;
        q.yaw_deg         = r.yaw;      // THÔ — module cộng yaw_offset
        q.gimbal_pan_deg  = 0.0;
        // 90 = nhìn thẳng xuống. Bộ MUN-FRL để gimbal_tilt_deg = 90 hằng số
        // (không có gimbal thật), Yên Bái trung vị 84,2 — cả hai đều là ảnh
        // nadir. Bản đầu để 0 và vì thế né mất cổng nghiêng hoàn toàn.
        q.gimbal_tilt_deg = 90.0;

        const Eigen::Vector3d p_true = geo.enu(r.lat, r.lon, 0.0);
        const Eigen::Vector2d xy_true(p_true.x(), p_true.y());
        q.xy_enu_pred = xy_true + Eigen::Vector2d(prior_bias_m, 0.0);
        q.cov_pred = Eigen::Matrix2d::Identity() * ((radius_m / 3.0) * (radius_m / 3.0));
        q.prediction_valid = true;

        const std::size_t before = got.size();
        if (!anchor.requestFix(q)) { ++refused; continue; }

        std::unique_lock<std::mutex> lk(mtx);
        const bool ok = cv.wait_for(lk, std::chrono::seconds(10),
                                    [&] { return got.size() > before; });
        if (!ok) { ++no_fix; continue; }
        const auto f = got.back();
        lk.unlock();

        const double err   = (f.xy_enu - xy_true).norm();
        const double sigma = std::sqrt(0.5 * (f.cov(0, 0) + f.cov(1, 1)));
        const bool inside  = err <= 2.0 * sigma;
        const double lat_ms = anchor.stats().last_query_ms;
        errs.push_back(err); sigmas.push_back(sigma);
        confs.push_back(f.confidence); lats.push_back(lat_ms);
        const auto& st = anchor.last_stage_timing();
        t_pre.push_back(st.preprocess); t_bb.push_back(st.backbone);
        t_agg.push_back(st.aggregate);  t_prj.push_back(st.project);
        t_srch.push_back(st.search);
        ++emitted;

        std::printf("%5d %9.1f %9.1f %8.3f %8.1f %8s\n", r.id, err, sigma,
                    f.confidence, lat_ms, inside ? "có" : "KHÔNG");
        if (csv) csv << r.id << ',' << err << ',' << sigma << ',' << f.confidence
                     << ',' << lat_ms << ',' << (inside ? 1 : 0) << '\n';
    }
    const double flow_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_flow).count();
    anchor.stop();

    std::printf("%s\n", std::string(56, '-').c_str());
    std::printf("phát %d/%zu fix | từ chối %d | không có fix %d | %.0f ms tổng\n",
                emitted, rows.size(), refused, no_fix, flow_ms);
    if (errs.empty()) { std::printf("\nkhông có fix nào để thống kê\n"); return 0; }

    int inside = 0;
    for (std::size_t i = 0; i < errs.size(); ++i) if (errs[i] <= 2.0 * sigmas[i]) ++inside;

    std::printf("\nSAI SỐ VỊ TRÍ so với groundtruth\n");
    std::printf("  trung vị %.1f m | p90 %.1f m | max %.1f m\n",
                quantile(errs, 0.5), quantile(errs, 0.9), quantile(errs, 1.0));

    std::printf("\nHIỆU CHUẨN cov — sai số nằm trong 2σ: %d/%zu = %.0f%%\n",
                inside, errs.size(), 100.0 * inside / static_cast<double>(errs.size()));
    const double ratio = quantile(sigmas, 0.5) / std::max(quantile(errs, 0.5), 1e-9);
    std::printf("  σ trung vị %.1f m / sai số trung vị %.1f m = %.1fx", 
                quantile(sigmas, 0.5), quantile(errs, 0.5), ratio);
    if (ratio > 2.0)      std::printf("  -> cov QUÁ BI QUAN: fusion sẽ đánh trọng số thấp\n");
    else if (ratio < 0.5) std::printf("  -> cov QUÁ LẠC QUAN: một fix sai sẽ kéo lệch quỹ đạo\n");
    else                  std::printf("  -> hợp lý\n");

    std::printf("\nconfidence: trung vị %.3f | min %.3f | max %.3f  [CHƯA HIỆU CHUẨN]\n",
                quantile(confs, 0.5), quantile(confs, 0.0), quantile(confs, 1.0));
    std::printf("độ trễ truy vấn: trung vị %.1f ms | p90 %.1f ms\n",
                quantile(lats, 0.5), quantile(lats, 0.9));
    std::printf("  phân rã (trung vị): tiền xử lý %.1f | backbone %.1f | VLAD %.2f | "
                "PCA %.2f | tìm kiếm %.3f ms\n",
                quantile(t_pre, 0.5), quantile(t_bb, 0.5), quantile(t_agg, 0.5),
                quantile(t_prj, 0.5), quantile(t_srch, 0.5));
    if (!csv_out.empty()) std::printf("\n-> %s\n", csv_out.c_str());
    return 0;
}
