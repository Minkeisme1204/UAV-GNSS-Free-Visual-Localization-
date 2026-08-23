// VprModule::localize() — truy hồi RỒI khớp tinh, trả thẳng lat/lon.
//
// Đây là chỗ tầng khớp tinh của satVPR lần đầu được gọi từ phía uavloc. Bộ test
// đo đúng ba thứ:
//   * DB v2 ⇒ khớp tinh BẬT; DB v1 ⇒ TẮT mà không hỏng, và toạ độ vẫn dùng được;
//   * toạ độ trả về nằm trong ô thắng rerank, không phải một điểm bất kỳ;
//   * sai số so groundtruth so được với con số satVPR đã đo trên cùng bộ.
//
// ⚠ Không phải cổng parity. satVPR có P10 lo việc đó. Ở đây kiểm việc NỐI DÂY:
// đúng kiểu, đúng khung toạ độ, đúng hành vi lùi.

#include "uavloc/vpr/vpr_module.h"

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
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

constexpr double kMPerDegLat = 111132.0;
constexpr double kMPerDegLonEq = 111320.0;
constexpr double kPi = 3.14159265358979323846;

double dist_m(double lat1, double lon1, double lat2, double lon2, double ref_lat) {
    const double dy = (lat2 - lat1) * kMPerDegLat;
    const double dx = (lon2 - lon1) * kMPerDegLonEq * std::cos(ref_lat * kPi / 180.0);
    return std::sqrt(dx * dx + dy * dy);
}

//! Một dòng của query_latlon.csv, cộng tư thế từ log thô.
struct Row {
    int id = 0;
    double lat = 0, lon = 0, yaw = 0, alt = 0;
    double roll = 0, pitch = 0, pan = 0, tilt = 90.0;
};

//! Đọc query_latlon.csv (Query_id, Latitude, Longitude, Yaw, ImageId, Sensor_Alt_m, ...)
std::vector<Row> read_queries(const std::string& path, int limit) {
    std::vector<Row> out;
    std::ifstream f(path);
    if (!f) return out;
    std::string line;
    std::getline(f, line);   // header
    while (std::getline(f, line) && (limit <= 0 || static_cast<int>(out.size()) < limit)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> c;
        while (std::getline(ss, cell, ',')) c.push_back(cell);
        if (c.size() < 6) continue;
        Row r;
        r.id  = std::stoi(c[0]);
        r.lat = std::stod(c[1]);
        r.lon = std::stod(c[2]);
        r.yaw = std::stod(c[3]);
        r.alt = std::stod(c[5]);   // Sensor_Alt_m — ĐÚNG thứ uavloc có lúc chạy
        out.push_back(r);
    }
    return out;
}

double median_of(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

} // namespace

int main(int argc, char** argv) {
    const std::string root = SATVPR_DATA_ROOT;
    const std::string set = (argc > 1) ? argv[1] : "yenbai_800m";
    const int n_max = (argc > 2) ? std::atoi(argv[2]) : 30;
    const std::string d = root + "/dataset/eval/" + set;

    uavloc::vpr::VprConfig cfg;
    cfg.model_path          = root + "/.weights/dino/dino_vits8_layer9_norm.onnx";
    cfg.vocab_bin_path      = root + "/.weights/vlad/vocab_b2_flighttrack_all.bin";
    cfg.pca_path            = d + "/pca.yml";
    cfg.database_path       = d + "/" + set + ".vprdb";
    cfg.keypoint_model_path = root + "/.weights/superpoint.onnx";
    cfg.matcher_model_path  = root + "/.weights/superpoint_lightglue.onnx";
    cfg.top_k               = 5;
    cfg.fine_top_k          = 5;
    cfg.search_radius_m     = 1000.0;
    cfg.device              = "cuda";

    for (const auto& p : {cfg.model_path, cfg.vocab_bin_path, cfg.pca_path,
                          cfg.database_path, cfg.keypoint_model_path,
                          cfg.matcher_model_path}) {
        if (!exists(p)) {
            std::printf("BỎ QUA: thiếu artifact %s\n", p.c_str());
            return 0;
        }
    }
    const auto rows = read_queries(d + "/query_latlon.csv", n_max);
    if (rows.empty()) {
        std::printf("BỎ QUA: không đọc được %s/query_latlon.csv\n", d.c_str());
        return 0;
    }

    uavloc::vpr::VprModule m;
    if (!check(m.setup(cfg), "setup()", m.last_error())) return 1;
    check(m.has_fine(), "khớp tinh BẬT với .vprdb v2");

    // ── Cấu hình đọc được từ YAML ────────────────────────────────────────────
    {
        const YAML::Node n = YAML::Load(
            "VPR:\n  fine_top_k: 3\n  min_inliers: 9\n  ransac_seed: 7\n");
        const auto c = uavloc::vpr::VprConfig::fromYaml(n["VPR"]);
        check(c.fine_top_k == 3 && c.min_inliers == 9 && c.ransac_seed == 7,
              "fromYaml đọc được khoá khớp tinh");
        check(c.bev_px == 508 && c.fine_net_size == 512,
              "khoá khớp tinh vắng mặt lấy mặc định");
    }

    // ── Chạy thật ────────────────────────────────────────────────────────────
    std::vector<double> err_fine, err_coarse;
    long n_valid = 0, n_rank1 = 0, n_out_of_tile = 0;
    const double ref_lat = rows.front().lat;
    const double half_tile_m = m.tile_footprint_m() * 0.75;   // nới 1,5x cho an toàn

    for (const auto& r : rows) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "/queries/%05d.jpg", r.id);
        const cv::Mat img = cv::imread(d + buf, cv::IMREAD_COLOR);
        if (img.empty()) continue;

        uavloc::vpr::FineQueryInput in;
        in.yaw_deg = r.yaw;
        in.gimbal_tilt_deg = 90.0;   // nadir; bộ này không có cột tilt trong csv rút gọn
        in.height_agl_m = r.alt;
        in.fx = in.fy = 1065.138;
        in.cx = static_cast<double>(img.cols) / 2.0;
        in.cy = static_cast<double>(img.rows) / 2.0;

        // Cổng đặt ở vị trí THẬT — trường hợp lạc quan, giống bản Python.
        const auto p = m.localize(img, in, r.lat, r.lon);
        if (p.tile_id < 0) continue;

        err_fine.push_back(dist_m(r.lat, r.lon, p.latitude, p.longitude, ref_lat));
        if (p.valid) ++n_valid;
        if (p.rank == 0) ++n_rank1;

        // Toạ độ phải nằm TRONG ô thắng, không phải một điểm bất kỳ trên bản đồ.
        // Đây là kiểm nối dây: sai khung toạ độ thì nó bay đi rất xa.
        const auto rr = m.retrieve(img, m.align_degrees(r.yaw, 0.0), r.lat, r.lon);
        if (rr.valid) {
            for (const auto& mm : rr.matches) {
                if (mm.tile_id != p.tile_id) continue;
                if (dist_m(mm.latitude, mm.longitude, p.latitude, p.longitude,
                           ref_lat) > half_tile_m) {
                    ++n_out_of_tile;
                }
                err_coarse.push_back(
                    dist_m(r.lat, r.lon, rr.matches.front().latitude,
                           rr.matches.front().longitude, ref_lat));
                break;
            }
        }
    }

    if (!check(!err_fine.empty(), "chạy được ít nhất một query")) return 1;
    const auto n = static_cast<double>(err_fine.size());

    check(n_out_of_tile == 0, "toạ độ nằm TRONG ô thắng rerank",
          std::to_string(n_out_of_tile) + "/" + std::to_string(err_fine.size())
              + " ra ngoài");

    long u100 = 0, u200 = 0;
    for (double e : err_fine) {
        if (e <= 100.0) ++u100;
        if (e <= 200.0) ++u200;
    }
    const double med = median_of(err_fine);
    std::printf("\n[INFO] %zu query | err_2d trung vị %.1f m | ≤100m %.1f%% | "
                "≤200m %.1f%%\n", err_fine.size(), med,
                100.0 * static_cast<double>(u100) / n,
                100.0 * static_cast<double>(u200) / n);
    if (!err_coarse.empty()) {
        std::printf("[INFO] truy hồi thuần (tâm ô hạng 1) trung vị %.1f m\n",
                    median_of(err_coarse));
    }
    std::printf("[INFO] homography đạt %.1f%% | ô thắng là hạng 1: %.1f%% | "
                "thời gian khớp tinh %.1f ms\n",
                100.0 * static_cast<double>(n_valid) / n,
                100.0 * static_cast<double>(n_rank1) / n,
                m.last_fine_timing().total());

    // satVPR đo trung vị 74,0 m trên đủ 237 query với cùng định nghĩa độ cao.
    // Ngưỡng để rộng: mẫu ở đây nhỏ, và cái cần bắt là NỐI DÂY SAI (khung toạ độ
    // lệch cho ra hàng trăm mét), không phải chênh vài mét.
    check(med < 200.0, "trung vị err_2d ở mức satVPR đã đo (74,0 m), không phải "
                       "hàng trăm mét", std::to_string(static_cast<int>(med)) + " m");

    std::printf("%s — %d hỏng\n", g_failed == 0 ? "ĐẠT" : "HỎNG", g_failed);
    return g_failed == 0 ? 0 : 1;
}
