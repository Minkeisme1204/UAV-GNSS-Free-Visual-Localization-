# Data Integrity Rules — báo cáo và số liệu

Áp dụng cho **mọi** báo cáo, tài liệu, bảng biểu và phát biểu về kết quả trong dự án này —
kể cả trả lời trong hội thoại, không riêng file `.md`.

## Nguyên tắc gốc

> **Không con số nào được viết ra nếu nó không đến từ một phép đo có thật, tái lập được.**

Thà thiếu số còn hơn có số sai. Một bảng có ô "chưa đo" là trung thực; một bảng đầy số ước
lượng là vô dụng và nguy hiểm hơn không có gì.

## Cấm tuyệt đối

- **Không bịa số.** Không ước lượng, nội suy, làm tròn "cho đẹp", hay điền số từ trí nhớ về một
  lần chạy trước. Nếu chưa chạy thì ghi **"chưa đo"**.
- **Không tái sử dụng số cũ như số mới.** Mỗi bảng phải ghi **ngày chạy** và **cấu hình**; khi mã
  hoặc config đã đổi thì số cũ chỉ được dùng nếu nói rõ đó là số của phiên bản nào.
- **Không "sửa cho khớp".** Nếu phép kiểm hồi quy lệch so với baseline, phải **dừng và báo cáo**,
  không được chỉnh ngưỡng/tham số cho tới khi số liệu vừa mắt.
- **Không công bố kết luận vượt quá thứ phép đo cho phép.** Xem "Ranh giới suy luận" bên dưới.

## Bắt buộc

- **Dẫn nguồn cho mọi con số**: tên file log/CSV trong `build/tests/`, hoặc lệnh tái lập. Người đọc
  phải chạy lại được.
- **Ghi rõ điều kiện đo**: dataset, số khung, cấu hình, chế độ chạy (sync/async), ngày, có chạy
  song song hay không.
- **Phân biệt ba loại phát biểu** và ghi nhãn khi không hiển nhiên:
  | Nhãn | Nghĩa |
  |---|---|
  | **[đo]** | lấy trực tiếp từ log/CSV của một lần chạy có thật |
  | **[suy ra]** | tính từ đại lượng đã đo, nêu rõ công thức |
  | **[giả thuyết]** | chưa có phép đo phân xử — phải kèm phép đo dự kiến để kiểm chứng |
- **Số liệu ngoài** (bài báo, thư viện, dataset) phải có trích dẫn kiểm chứng được, và ghi rõ đã
  xác minh hay lấy từ khảo sát cũ.
- **Công bố sai số kèm phân bố**, không rút gọn thành một số: tối thiểu **trung vị · p95 · max**,
  và nói rõ RMS là RMS (ATE) chứ không phải trung bình.
- **Khi phép đo bị nhiễm** (ví dụ phép đo tuyệt đối sinh từ groundtruth), phải ghi cảnh báo **ngay
  tại bảng số liệu**, không giấu xuống cuối tài liệu.

## Ranh giới suy luận — lỗi nguy hiểm nhất

Không được biến "**A không xảy ra trong thí nghiệm này**" thành "**A không có tác dụng**" khi thí
nghiệm không đủ điều kiện để kết luận.

> Ví dụ có thật (M1, 2026-08-01): trên YenBai, 84 % phép đo tuyệt đối bị cổng lọc loại trước khi
> vào đồ thị. Phát biểu **đúng** là "cổng lọc hiệu chỉnh sai làm vô hiệu hoá phép đo tuyệt đối".
> Phát biểu **sai** là "định vị tuyệt đối không cải thiện được sai số trên YenBai" — thí nghiệm
> chưa từng kiểm tra điều đó.

Khi một thí nghiệm không hợp lệ (mẫu rỗng, cấu hình sai, đường dữ liệu bị chặn), **ghi rõ là vô
hiệu** thay vì báo cáo con số thu được.

## Tự đính chính

Khi phát hiện một số đã công bố là sai hoặc bị hiểu sai phạm vi, phải **đính chính tại chỗ** trong
tài liệu gốc (giữ lại dấu vết bản cũ), không im lặng sửa số.

> Ví dụ có thật (M0 → 2026-08-01): con số "11 860 keypoint mỗi khung hình" đo bằng công cụ chỉ đọc
> khung hình đầu tiên ⇒ không phải trung bình cả chuyến bay. Đã ghi khối đính chính trong
> `profiling_baseline.md` §5 thay vì lặng lẽ xoá.

## Nghiệm thu

Mỗi milestone phải có bảng đối chiếu **tiêu chí ↔ số đo ↔ đạt/chưa đạt**, trong đó mục chưa đạt
ghi rõ **nguyên nhân** và **phép đo cần làm tiếp**. Không được đánh dấu đạt bằng lập luận.
