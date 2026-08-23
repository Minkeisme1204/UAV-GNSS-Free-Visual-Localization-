#include "uavloc/vpr/vpr_module.h"

// satvpr:: và ONNX Runtime CHỈ xuất hiện trong file này. Header công khai dùng
// pimpl nên chúng không rò vào interface đã export của libuavloc.so.
#include <satvpr/geom/bev.h>
#include <satvpr/pipeline/fine_localizer.h>
#include <satvpr/pipeline/place_recognizer.h>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>

namespace uavloc::vpr {
namespace {

satvpr::ExecutionProvider parse_provider(std::string device) {
    std::transform(device.begin(), device.end(), device.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (device == "cuda" || device == "gpu") return satvpr::ExecutionProvider::CUDA;
    if (device == "tensorrt" || device == "trt") return satvpr::ExecutionProvider::TENSORRT;
    return satvpr::ExecutionProvider::CPU;
}

satvpr::KeypointProvider parse_kpt_provider(std::string device) {
    std::transform(device.begin(), device.end(), device.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (device == "cuda" || device == "gpu") return satvpr::KeypointProvider::CUDA;
    if (device == "tensorrt" || device == "trt") return satvpr::KeypointProvider::TENSORRT;
    return satvpr::KeypointProvider::CPU;
}

} // namespace

struct VprModule::Impl {
    VprConfig cfg;
    // ⚠ THỨ TỰ KHAI BÁO CÓ Ý NGHĨA: `fine` giữ tham chiếu tới `core->database()`,
    // mà thành viên huỷ NGƯỢC thứ tự khai báo — nên `fine` phải đứng SAU `core`
    // để nó chết trước.
    std::unique_ptr<satvpr::PlaceRecognizer> core;
    std::unique_ptr<satvpr::FineLocalizer>   fine;
    VprStageTiming timing;
    FineStageTiming fine_timing;
    std::string error;
    std::string db_name;
    long long   db_size = 0;

    // Đệm tái dùng cho đường khớp tinh. Đây là lý do `localize()` KHÔNG gọi song
    // song được — hợp đồng đã ghi trong header.
    cv::Mat canvas;
    std::vector<std::int64_t> top_ids;
};

VprModule::VprModule() : impl_(std::make_unique<Impl>()) {}
VprModule::~VprModule() = default;

bool VprModule::setup(const VprConfig& config) {
    impl_->cfg = config;
    impl_->error.clear();

    satvpr::PlaceRecognizerConfig c;
    c.preprocess.input_h        = config.input_height;
    c.preprocess.input_w        = config.input_width;
    c.preprocess.patch_size     = config.patch_size;
    c.preprocess.yaw_offset_deg = config.yaw_offset_deg;
    c.preprocess.use_gimbal_pan = config.use_gimbal_pan;
    c.backbone.model_path       = config.model_path;
    c.backbone.provider         = parse_provider(config.device);
    c.backbone.intra_op_threads = config.intra_op_threads;
    // Lệch input_size là hỏng IM LẶNG: vị trí embedding đã gấp thành hằng số
    // trong đồ thị ONNX. Khai kỳ vọng để satvpr từ chối thay vì tự resize.
    c.backbone.expect_h         = config.input_height;
    c.backbone.expect_w         = config.input_width - (config.input_width % config.patch_size);
    c.vocab_bin_path            = config.vocab_bin_path;
    c.pca_yaml_path             = config.pca_path;
    c.database_path             = config.database_path;
    c.top_k                     = config.top_k;
    c.search_radius_m           = config.search_radius_m;

    auto made = satvpr::PlaceRecognizer::create(c);
    if (!made) {
        impl_->error = made.status().str();
        spdlog::error("vpr: setup thất bại — {}", impl_->error);
        impl_->core.reset();
        return false;
    }
    impl_->core = made.take();
    impl_->db_size = static_cast<long long>(impl_->core->db_header().num_tiles);
    impl_->db_name = impl_->core->db_header().eval_set;
    // ── Khớp tinh: TUỲ CHỌN ──────────────────────────────────────────────────
    // Thiếu model hoặc DB là v1 ⇒ tắt hẳn tầng này và nói rõ vì sao. KHÔNG làm
    // setup() thất bại: truy hồi vẫn chạy được, và một DB cũ vẫn phải dùng được.
    impl_->fine.reset();
    if (config.keypoint_model_path.empty() || config.matcher_model_path.empty()) {
        spdlog::info("vpr: khớp tinh TẮT — chưa khai keypoint_model_path/"
                     "matcher_model_path; vị trí sẽ là tâm ô hạng 1");
    } else if (!impl_->core->database().has_images()) {
        spdlog::warn("vpr: khớp tinh TẮT — '{}' là .vprdb v1, không có kho ảnh ô. "
                     "Nâng cấp: python -m scripts.export_vprdb --set {} --upgrade",
                     config.database_path, impl_->db_name);
    } else {
        satvpr::FineLocalizerConfig f;
        f.keypoint.model_path      = config.keypoint_model_path;
        f.keypoint.net_size        = config.fine_net_size;
        f.keypoint.max_keypoints   = config.fine_max_keypoints;
        f.keypoint.provider        = parse_kpt_provider(config.device);
        f.keypoint.intra_op_threads = config.intra_op_threads;
        f.matcher.model_path       = config.matcher_model_path;
        f.matcher.provider         = parse_kpt_provider(config.device);
        f.matcher.intra_op_threads = config.intra_op_threads;
        f.bev.out_gsd_m            = config.bev_gsd_m;
        f.bev.out_px               = config.bev_px;
        f.top_k_match              = config.fine_top_k;
        f.ransac_px                = config.ransac_px;
        f.ransac_iters             = config.ransac_iters;
        f.ransac_seed              = config.ransac_seed;
        f.min_inliers              = config.min_inliers;

        auto fmade = satvpr::FineLocalizer::create(f, impl_->core->database(),
                                                   config.database_path);
        if (!fmade) {
            // Không làm setup() hỏng: mất khớp tinh vẫn còn truy hồi.
            spdlog::error("vpr: dựng khớp tinh thất bại, chạy tiếp ở chế độ tâm ô "
                          "— {}", fmade.status().str());
        } else {
            impl_->fine = fmade.take();
        }
    }

    spdlog::info("vpr: sẵn sàng — {} ô ('{}'), backbone {} trên {} | khớp tinh: {}",
                 impl_->db_size, impl_->db_name, config.model_path, config.device,
                 impl_->fine ? "BẬT" : "tắt");
    return true;
}

bool VprModule::is_ready() const { return impl_->core != nullptr; }
const std::string& VprModule::last_error() const { return impl_->error; }

double VprModule::align_degrees(double yaw_deg, double gimbal_pan_deg) const {
    return yaw_deg + (impl_->cfg.use_gimbal_pan ? gimbal_pan_deg : 0.0) +
           impl_->cfg.yaw_offset_deg;
}

bool VprModule::has_fine() const { return impl_->fine != nullptr; }

const FineStageTiming& VprModule::last_fine_timing() const { return impl_->fine_timing; }

FinePose VprModule::localize(const cv::Mat& bgr_frame, const FineQueryInput& in,
                             double prior_lat, double prior_lon,
                             RetrievalOutcome* retrieval) {
    FinePose out;
    if (retrieval != nullptr) *retrieval = RetrievalOutcome{};
    impl_->fine_timing = FineStageTiming{};
    if (impl_->core == nullptr) {
        impl_->error = "chưa setup()";
        return out;
    }

    // ── 1. Nắn BEV ───────────────────────────────────────────────────────────
    // Canvas ra ĐÃ hướng Bắc (yaw nằm trong ma trận xoay), nên bước truy hồi
    // dưới đây phải đặt `is_tile = true`: gọi north_align lần nữa là xoay hai lần.
    const cv::Matx33d K(in.fx, 0.0, in.cx, 0.0, in.fy, in.cy, 0.0, 0.0, 1.0);
    const cv::Matx33d rot = satvpr::camera_rotation(in.roll_deg, in.pitch_deg,
                                                    in.yaw_deg, in.gimbal_pan_deg,
                                                    in.gimbal_tilt_deg);
    satvpr::BevConfig bev_cfg;
    bev_cfg.out_gsd_m = impl_->cfg.bev_gsd_m;
    bev_cfg.out_px    = impl_->cfg.bev_px;

    satvpr::BevResult bev;
    const satvpr::Status bst = satvpr::warp_bev(bgr_frame, rot, K, in.height_agl_m,
                                                bev_cfg, &impl_->canvas, &bev);
    if (!bst.ok()) {
        impl_->error = bst.str();
        spdlog::warn("vpr: nắn BEV thất bại — {}", impl_->error);
        return out;
    }
    out.coverage = bev.coverage;

    // ── 2. Truy hồi trên canvas ──────────────────────────────────────────────
    satvpr::PlaceRecognizer::Query q;
    q.bgr_frame = &impl_->canvas;
    q.prior_lat = prior_lat;
    q.prior_lon = prior_lon;
    q.use_gate  = true;
    q.is_tile   = true;   // canvas đã hướng Bắc — xem ghi chú ở bước 1

    satvpr::RetrievalResult res;
    satvpr::PlaceRecognizer::Timing t;
    const satvpr::Status rst = impl_->core->run(q, &res, &t);
    if (!rst.ok() || res.candidates.empty()) {
        impl_->error = rst.ok() ? "không có ứng viên nào qua cổng" : rst.str();
        return out;
    }
    impl_->timing.preprocess = t.preprocess;
    impl_->timing.backbone   = t.backbone;
    impl_->timing.aggregate  = t.vlad;
    impl_->timing.project    = t.pca;
    impl_->timing.search     = t.search;

    if (retrieval != nullptr) {
        retrieval->num_gated = res.num_gated;
        retrieval->num_total = res.num_total;
        retrieval->matches.reserve(res.candidates.size());
        for (const auto& c : res.candidates) {
            PlaceMatch pm;
            pm.tile_id    = c.tile_id;
            pm.similarity = c.similarity;
            pm.latitude   = c.center_lat;
            pm.longitude  = c.center_lon;
            retrieval->matches.push_back(pm);
        }
        retrieval->valid = true;
    }

    // ── 3. Khớp tinh, hoặc lùi về tâm ô ──────────────────────────────────────
    if (impl_->fine == nullptr) {
        // DB v1 hoặc chưa khai model: hành vi ĐÚNG BẰNG bản chưa có khớp tinh.
        out.tile_id   = res.candidates.front().tile_id;
        out.rank      = 0;
        out.latitude  = res.candidates.front().center_lat;
        out.longitude = res.candidates.front().center_lon;
        out.valid     = false;
        return out;
    }

    impl_->top_ids.clear();
    impl_->top_ids.reserve(res.candidates.size());
    for (const auto& c : res.candidates) impl_->top_ids.push_back(c.tile_id);

    satvpr::FineQuery fq;
    fq.bgr_frame = &bgr_frame;
    fq.K = K;
    fq.height_m = in.height_agl_m;
    fq.roll_deg = in.roll_deg;
    fq.pitch_deg = in.pitch_deg;
    fq.yaw_deg = in.yaw_deg;
    fq.pan_deg = in.gimbal_pan_deg;
    fq.tilt_deg = in.gimbal_tilt_deg;
    fq.tile_ids = impl_->top_ids.data();
    fq.num_tiles = static_cast<std::int32_t>(impl_->top_ids.size());

    satvpr::FineResult fr;
    satvpr::FineTiming ft;
    const satvpr::Status fst = impl_->fine->localize(fq, &fr, &ft);
    if (!fst.ok()) {
        impl_->error = fst.str();
        spdlog::warn("vpr: khớp tinh thất bại — {}", impl_->error);
        // Vẫn trả tâm ô hạng 1: một lỗi ở tầng tinh không được làm mất luôn kết
        // quả truy hồi vốn đã có.
        out.tile_id   = res.candidates.front().tile_id;
        out.rank      = 0;
        out.latitude  = res.candidates.front().center_lat;
        out.longitude = res.candidates.front().center_lon;
        out.valid     = false;
        return out;
    }

    impl_->fine_timing.bev      = ft.bev;
    impl_->fine_timing.decode   = ft.decode;
    impl_->fine_timing.keypoint = ft.keypoint;
    impl_->fine_timing.match    = ft.match;
    impl_->fine_timing.ransac   = ft.ransac;

    out.valid       = fr.valid;
    out.latitude    = fr.lat;
    out.longitude   = fr.lon;
    out.tile_id     = fr.tile_id;
    out.rank        = fr.rank;
    out.num_matches = fr.n_match;
    out.num_inliers = fr.n_inlier;
    return out;
}

RetrievalOutcome VprModule::retrieve(const cv::Mat& bgr_frame, double align_deg,
                                     double prior_lat, double prior_lon) {
    RetrievalOutcome out;
    impl_->timing = VprStageTiming{};
    if (impl_->core == nullptr) {
        impl_->error = "chưa setup()";
        return out;
    }

    satvpr::PlaceRecognizer::Query q;
    q.bgr_frame = &bgr_frame;
    q.align_deg = align_deg;
    q.prior_lat = prior_lat;
    q.prior_lon = prior_lon;
    q.use_gate  = true;

    satvpr::RetrievalResult res;
    satvpr::PlaceRecognizer::Timing t;
    const satvpr::Status st = impl_->core->run(q, &res, &t);
    if (!st.ok()) {
        impl_->error = st.str();
        spdlog::warn("vpr: retrieve thất bại — {}", impl_->error);
        return out;
    }

    impl_->timing.preprocess = t.preprocess;
    impl_->timing.backbone   = t.backbone;
    impl_->timing.aggregate  = t.vlad;
    impl_->timing.project    = t.pca;
    impl_->timing.search     = t.search;

    out.num_gated = res.num_gated;
    out.num_total = res.num_total;
    out.matches.reserve(res.candidates.size());
    for (const auto& c : res.candidates) {
        PlaceMatch m;
        m.tile_id    = c.tile_id;
        m.similarity = c.similarity;
        m.latitude   = c.center_lat;
        m.longitude  = c.center_lon;
        out.matches.push_back(m);
    }
    out.valid = !out.matches.empty();
    return out;
}

const VprStageTiming& VprModule::last_timing() const { return impl_->timing; }
long long VprModule::database_size() const { return impl_->db_size; }

double VprModule::tile_stride_m() const {
    if (impl_->core == nullptr) return 0.0;
    const auto& h = impl_->core->db_header();
    return static_cast<double>(h.tile_stride) * h.gsd_m;
}

double VprModule::tile_footprint_m() const {
    if (impl_->core == nullptr) return 0.0;
    const auto& h = impl_->core->db_header();
    return static_cast<double>(h.tile_size) * h.gsd_m;
}
const std::string& VprModule::database_name() const { return impl_->db_name; }

} // namespace uavloc::vpr
