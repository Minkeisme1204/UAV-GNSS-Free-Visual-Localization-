# Documentation Rules

- All design documents, architecture notes, and API references go into `.docs/` as Markdown (`.md`) files.
- Reference PDFs stay in `.docs/papers/`.
- Do not create Markdown files outside `.docs/` except for `README.md` and `CLAUDE.md`.
- File naming: `snake_case_topic.md` (e.g. `.docs/designs/fusion_factor_graph.md`).
- Do not commit binary data, model weights, or large datasets — use `data/` (gitignored) or document download steps in `.docs/`.
- Every new document must be added to `.docs/README.md` (the manually-maintained index).

## Where does a document belong?

| Thư mục | Câu hỏi quyết định | Vòng đời |
|---|---|---|
| `designs/` | Hệ **nên** hoạt động thế nào? | Sống lâu, sửa tại chỗ khi mã đổi |
| `reports/` | Ngày X đã đo/đạt được cái gì? | Bất biến sau khi viết; sai thì đính chính, không viết lại |
| `theory/` | Suy dẫn toán/vật lý, không gắn phiên bản mã | Gần như vĩnh viễn |
| `datasets/` | Mô tả dữ liệu vào, không mô tả mã | Đổi khi có dataset mới |
| `related_work/` | Phân tích hệ của người khác | Đóng băng sau khi khảo sát xong |
| `thesis/` | Văn bản sẽ vào quyển thuyết minh | Bản thảo, sửa liên tục |
| `papers/` | PDF gốc của bên thứ ba | Chỉ thêm |

**Ranh giới đắt nhất — `designs/` ↔ `reports/`:** nếu tài liệu có dòng "Ngày đo"/"Ngày chạy"
kèm một bảng số thì nó là **report** (bất biến — sai thì viết khối đính chính, không sửa số tại
chỗ). Nếu nó mô tả trạng thái hiện hành của hệ và sẽ được sửa khi mã đổi thì nó là **design**.
