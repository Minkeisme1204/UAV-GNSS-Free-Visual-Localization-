# vpr module — truy hồi ảnh. Toàn bộ số học nằm ở satvpr::core; hai file ở đây
# chỉ là lớp adapter cho uavloc (đọc YAML theo mẫu của repo, kiểu dữ liệu riêng).
target_sources(uavloc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/vpr_config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vpr_module.cpp
)

# PRIVATE — cùng lý do gtsam là PRIVATE ở src/fusion/: header công khai
# include/uavloc/vpr/ dùng pimpl và chỉ để lọt cv::Mat / yaml-cpp / std.
target_link_libraries(uavloc PRIVATE satvpr::core)

# ⚠ PRIVATE của CMake KHÔNG ẩn symbol — nó chỉ chặn lan truyền include và link
# dependency sang target khác. Kiểm chứng: bản đầu tiên để lọt 56 symbol satvpr::
# và 12 symbol Ort:: vào bảng symbol động của libuavloc.so.
#
# --exclude-libs ẩn mọi symbol đến từ một archive tĩnh. Chỉ nhắm đúng
# libsatvpr_core.a để không đụng các thư viện khác.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(uavloc PRIVATE "LINKER:--exclude-libs,libsatvpr_core.a")
endif()

# --exclude-libs chỉ với tới symbol đến TỪ archive. Hàm inline của satvpr được
# thực thể hoá trong chính vpr_module.cpp (object của uavloc) nên vẫn lọt ra dưới
# dạng symbol yếu — đo được 3 cái: Status::str() và dtor ngầm của
# PlaceRecognizerConfig.
#
# -fvisibility-inlines-hidden ẩn đúng nhóm đó mà KHÔNG đụng hàm non-inline, nên
# uavloc::vpr::VprModule::* vẫn được export. Đặt ở mức FILE, không mức target:
# áp cho cả libuavloc sẽ đổi hành vi của các module đã ổn định.
# ⚠ TARGET_DIRECTORY là bắt buộc: thuộc tính source file có phạm vi THƯ MỤC, mà
# target `uavloc` được định nghĩa ở src/ chứ không phải src/vpr/. Thiếu nó thì
# lệnh này im lặng không có tác dụng — đã kiểm: vẫn còn đúng 3 symbol yếu.
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/vpr_module.cpp
    TARGET_DIRECTORY uavloc
    PROPERTIES COMPILE_OPTIONS "-fvisibility-inlines-hidden")
