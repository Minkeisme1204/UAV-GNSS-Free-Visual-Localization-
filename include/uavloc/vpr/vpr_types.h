#pragma once

// Kiểu dữ liệu qua ranh giới của module vpr.
//
// vpr TRẢ VỀ LAT/LON, không bao giờ trả ENU. `anchor` là nơi DUY NHẤT đổi
// pose <-> toạ độ địa lý (.docs/reports/Phase2_roadmap.md §4), nên mọi phép đổi
// sang khung ENU của fusion nằm ở anchor::VprAnchor, không nằm ở đây.

#include <cstdint>
#include <vector>

namespace uavloc::vpr {

//! Một ô bản đồ ứng viên, đã xếp hạng theo độ tương đồng giảm dần.
struct PlaceMatch {
    std::int64_t tile_id = -1;
    //! Cosine giữa descriptor truy vấn và descriptor ô, trong [-1, 1].
    //!
    //! ⚠ KHÔNG dùng giá trị này làm anchor::AbsoluteFix::confidence. Nó không
    //! phải xác suất và không đơn điệu theo sai số: ảnh nhiều kết cấu cho cosine
    //! cao với MỌI ô, ảnh đồng nhất cho cosine thấp với cả ô đúng. Đo được trên
    //! mun_ds6: nhiễu 1 LSB làm 9,9% truy vấn đổi top-1, và khi đổi thì đổi sang
    //! nơi cách trung vị 162 m. confidence phải dựng từ đặc trưng PHÂN BIỆT
    //! (margin, spatial consistency) và phải được hiệu chuẩn.
    float  similarity = 0.0F;
    double latitude   = 0.0;
    double longitude  = 0.0;
};

//! Kết quả một lần truy hồi.
struct RetrievalOutcome {
    std::vector<PlaceMatch> matches;   //!< rỗng nghĩa là không có ứng viên nào
    //! Số ô lọt cổng không gian / tổng số ô trong cơ sở dữ liệu.
    std::int32_t num_gated = 0;
    std::int32_t num_total = 0;
    bool valid = false;
};

//! Thời gian từng tầng của lần truy hồi gần nhất [ms]. Dùng cho hồ sơ hiệu năng
//! (M0 của dự án đã ghi thành bài học: đo trước, tối ưu sau).
struct VprStageTiming {
    double preprocess = 0.0;
    double backbone   = 0.0;
    double aggregate  = 0.0;
    double project    = 0.0;
    double search     = 0.0;
    double total() const { return preprocess + backbone + aggregate + project + search; }
};

//! Vị trí sau KHỚP TINH — kết quả của tầng SuperPoint+LightGlue+homography.
//!
//! Truy hồi chỉ trả lời "ô nào", mà câu trả lời đó RỜI RẠC nên toạ độ chỉ có thể
//! là tâm ô. Tầng khớp tinh biến các cặp điểm thành một phép biến đổi liên tục,
//! nhờ đó nadir rơi vào một pixel bất kỳ trong ô. Đo trên 237 query Yên Bái:
//! trung vị 139,3 -> 77,1 m, tỉ lệ <=100 m từ 30,4 lên 63,3 %.
struct FinePose {
    //! false = khớp tinh KHÔNG đạt ngưỡng inlier và đã lùi về tâm ô. `latitude`
    //! và `longitude` VẪN dùng được — chúng là tâm ô thắng rerank, tức đúng bằng
    //! hành vi của bản chưa có khớp tinh. Tầng trên không phải tự đoán.
    bool valid = false;

    double latitude = 0.0;
    double longitude = 0.0;

    std::int64_t tile_id = -1;
    //! Hạng của ô THẮNG RERANK trong danh sách truy hồi, tính từ 0.
    //! ⚠ Đo được: chỉ 43,9 % số query có ô thắng là hạng 1. Rerank không phải
    //! tinh chỉnh — hơn nửa số lần, ô đúng không nằm ở đầu danh sách truy hồi.
    std::int32_t rank = -1;

    std::int32_t num_matches = 0;
    //! Số inlier của homography. ĐÂY là đại lượng duy nhất ở tầng này tương quan
    //! với sai số, nên nó là thứ nên dùng để dựng cov và confidence — không phải
    //! cosine của truy hồi.
    std::int32_t num_inliers = 0;

    //! Phần canvas BEV thực sự được khung nhìn thấy, [0, 1]. Canvas phần lớn là
    //! nền trắng thì khớp tệ vì lý do chẳng liên quan tới việc đúng chỗ.
    double coverage = 0.0;
};

//! Thời gian từng tầng của lần localize() gần nhất [ms].
//!
//! ⚠ Đo trên RTX 5090, 237 query, top-5: RANSAC chiếm 37 % và là tầng DUY NHẤT
//! có phương sai đáng kể (152,8 so với <=0,1 của các tầng mạng). Nó phụ thuộc
//! dữ liệu vì số cặp khớp dao động 13-175.
struct FineStageTiming {
    double bev = 0.0;
    double decode = 0.0;
    double keypoint = 0.0;
    double match = 0.0;
    double ransac = 0.0;
    double total() const { return bev + decode + keypoint + match + ransac; }
};

} // namespace uavloc::vpr
