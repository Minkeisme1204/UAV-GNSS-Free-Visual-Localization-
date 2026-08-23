// test_vpr — module vpr chạy được từ trong libuavloc.
//
// Đây KHÔNG phải test parity: mọi cổng số học nằm ở bộ test của satvpr_core
// (ctest -L parity ở repo satVPR). Test này chỉ khẳng định phần GHÉP đúng:
// nạp được artifact, từ chối đúng lúc, và một khung hình thật ra được lat/lon.
//
// Soft-skip (trả 0) khi thiếu artifact: một checkout không có dataset vẫn phải
// chạy ctest được.

#include "uavloc/vpr/vpr_module.h"

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <sys/stat.h>

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

} // namespace

int main() {
    const std::string root = SATVPR_DATA_ROOT;
    const std::string set = "mun_ds6";
    const std::string d = root + "/dataset/eval/" + set;

    uavloc::vpr::VprConfig cfg;
    cfg.model_path      = root + "/.weights/dino/dino_vits8_layer9_norm.onnx";
    cfg.vocab_bin_path  = root + "/.weights/vlad/vocab_b2_flighttrack_all.bin";
    cfg.pca_path        = d + "/pca.yml";
    cfg.database_path   = d + "/" + set + ".vprdb";
    cfg.input_height    = 224;
    cfg.input_width     = 298;
    cfg.yaw_offset_deg  = 180.0;   // MUN: gimbal quay về đuôi
    cfg.use_gimbal_pan  = false;   // ...nên KHÔNG cộng gimbal_pan lần nữa
    cfg.top_k           = 10;
    cfg.search_radius_m = 500.0;
    cfg.device          = "cuda";

    for (const auto& p : {cfg.model_path, cfg.vocab_bin_path, cfg.pca_path, cfg.database_path}) {
        if (!exists(p)) {
            std::printf("BỎ QUA: thiếu artifact %s\n", p.c_str());
            return 0;
        }
    }

    // ── 1. Cấu hình từ YAML theo mẫu .as<T>(default) ─────────────────────────
    {
        const YAML::Node n = YAML::Load("VPR:\n  top_k: 7\n  device: cpu\n");
        const auto c = uavloc::vpr::VprConfig::fromYaml(n["VPR"]);
        check(c.top_k == 7 && c.device == "cpu", "fromYaml đọc được khoá có mặt");
        check(c.input_height == 224 && c.patch_size == 8,
              "khoá vắng mặt lấy mặc định, không throw");
        const auto empty = uavloc::vpr::VprConfig::fromYaml(YAML::Node());
        check(empty.top_k == 10, "node VPR vắng hẳn cũng không throw");
    }

    // ── 2. Từ chối đúng lúc ──────────────────────────────────────────────────
    {
        uavloc::vpr::VprModule m;
        auto bad = cfg;
        bad.database_path = d + "/khong-ton-tai.vprdb";
        check(!m.setup(bad) && !m.is_ready(), "setup() từ chối khi thiếu DB");
        check(!m.last_error().empty(), "có thông báo lỗi", m.last_error());

        auto wrong_size = cfg;
        wrong_size.input_height = 256;   // .onnx có shape TĨNH 224x296
        uavloc::vpr::VprModule m2;
        check(!m2.setup(wrong_size), "setup() từ chối khi input_size lệch đồ thị ONNX",
              m2.last_error().substr(0, 90));
    }

    // ── 3. Nạp thật rồi chạy một khung hình ──────────────────────────────────
    uavloc::vpr::VprModule vpr;
    if (!check(vpr.setup(cfg), "setup() thành công", vpr.last_error())) return 1;
    check(vpr.is_ready(), "is_ready()");
    check(vpr.database_size() > 0, "DB có ô",
          std::to_string(vpr.database_size()) + " ô, '" + vpr.database_name() + "'");

    // Góc căn Bắc: use_gimbal_pan = false nên gimbal_pan bị bỏ qua.
    check(std::abs(vpr.align_degrees(67.2153, 180.0) - (67.2153 + 180.0)) < 1e-9,
          "align_degrees không đếm gimbal_pan hai lần");

    const cv::Mat frame = cv::imread(d + "/queries/00000.jpg", cv::IMREAD_COLOR);
    if (!check(!frame.empty(), "đọc được khung hình")) return 1;

    // Vị trí tiên nghiệm lấy từ query_latlon.csv dòng đầu của mun_ds6.
    const double prior_lat = 45.3231215, prior_lon = -75.6674457;
    const auto out = vpr.retrieve(frame, vpr.align_degrees(67.2153, 0.0),
                                  prior_lat, prior_lon);

    check(out.valid, "retrieve() trả kết quả");
    check(static_cast<int>(out.matches.size()) == cfg.top_k, "đủ top-K",
          std::to_string(out.matches.size()));
    check(out.num_gated > 0 && out.num_gated <= out.num_total, "cổng không gian hợp lệ",
          std::to_string(out.num_gated) + "/" + std::to_string(out.num_total) + " ô");

    if (!out.matches.empty()) {
        const auto& top = out.matches.front();
        check(top.similarity > 0.0F && top.similarity <= 1.0F, "similarity trong [0,1]",
              std::to_string(top.similarity));
        check(std::abs(top.latitude - prior_lat) < 0.01 &&
              std::abs(top.longitude - prior_lon) < 0.01,
              "ô hạng 1 nằm gần tiên nghiệm",
              std::to_string(top.latitude) + ", " + std::to_string(top.longitude));
        for (std::size_t i = 1; i < out.matches.size(); ++i) {
            if (out.matches[i].similarity > out.matches[i - 1].similarity) {
                check(false, "xếp hạng giảm dần"); break;
            }
        }
        check(true, "xếp hạng giảm dần");
    }

    const auto& t = vpr.last_timing();
    std::printf("\n  thời gian: tiền xử lý %.1f | backbone %.1f | VLAD %.2f | "
                "PCA %.2f | tìm kiếm %.3f ms  (tổng %.1f ms)\n",
                t.preprocess, t.backbone, t.aggregate, t.project, t.search, t.total());

    std::printf("\n%s — %d hỏng\n", g_failed == 0 ? "ĐẠT" : "HỎNG", g_failed);
    return g_failed == 0 ? 0 : 1;
}
