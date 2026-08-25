# Cài đặt dependencies & build `uavloc`

Tài liệu này liệt kê **toàn bộ** thư viện cần để build `libuavloc.so` và các test.
Mỗi thư viện có một mục riêng, ghi rõ **nguồn cài** (apt hay build từ source), lệnh CLI đầy đủ,
và các lưu ý đặc thù cho máy tham chiếu.

> Nguồn dữ liệu: tài liệu được dựng lại từ `build/CMakeCache.txt`, `build/compile_commands.json`
> và link line thực tế của `build/src/CMakeFiles/uavloc.dir/link.txt` trên máy tham chiếu.

---

## 0. Cấu hình máy tham chiếu

| Thành phần | Giá trị trên máy đã build thành công |
| --- | --- |
| OS | Ubuntu 22.04.5 LTS (jammy), kernel 6.8.0 |
| CPU | AMD Ryzen 9950X (x86-64, có AVX-512) |
| GPU | NVIDIA GeForce RTX 5090 — compute capability **12.0 (sm_120)** |
| NVIDIA driver | 580.173.02 |
| CUDA Toolkit | 12.8 (`/usr/local/cuda`) |
| cuDNN | 9.15.0 |
| Compiler | gcc / g++ 11.4.0 (mặc định của Ubuntu 22.04) |
| CMake | 3.28.3 tại `/usr/local/bin/cmake` (apt chỉ có 3.22.1) |
| C++ standard | C++17 |

**Lưu ý đặc thù phần cứng:** RTX 5090 là `sm_120`. Mọi thư viện có kernel CUDA
(OpenCV, ONNX Runtime, PyTorch cho phần Python) **phải** được build/cài với arch 12.0.
Binary CUDA build cho arch cũ sẽ cài được nhưng crash ở lời gọi CUDA đầu tiên.

---

## 1. Sơ đồ cài đặt

Dự án dùng **3 prefix** khác nhau. Đây là điểm dễ nhầm nhất:

| Prefix | Chứa gì |
| --- | --- |
| `/usr/local` | OpenCV, GTSAM, ONNX Runtime |
| `<repo>/.local` | spdlog, Iridescence, GLFW |
| `<repo>/.thirdparty/g2o/build/lib` | g2o — **dùng thẳng từ build tree, KHÔNG install** |
| `~/.projects/satVPR` | satVPR — repo riêng, `add_subdirectory` vào uavloc |

### Bảng nguồn cài

| Thư viện | Version trên máy tham chiếu | Nguồn |
| --- | --- | --- |
| Eigen3 | 3.4.0 | **apt** `libeigen3-dev` |
| Boost | 1.74.0 | **apt** `libboost-all-dev` |
| Intel TBB | 2021.5.0 | **apt** `libtbb-dev` |
| yaml-cpp | 0.7.0 | **apt** `libyaml-cpp-dev` |
| nlohmann/json | 3.10.5 | **apt** `nlohmann-json3-dev` |
| ZBar | 0.23.92 | **apt** `libzbar-dev` |
| FFmpeg (libav*) | 4.4.2 | **apt** `libavcodec/format/util/swscale-dev` |
| OpenGL / GLX / X11 | hệ thống | **apt** |
| libpng / libjpeg / zlib | hệ thống | **apt** |
| SuiteSparse | 5.10.1 | **apt** `libsuitesparse-dev` |
| CUDA Toolkit | 12.8 | **apt** (repo NVIDIA) |
| cuDNN | 9.15.0 | **apt** (repo NVIDIA) |
| CMake | 3.28.3 | **source** (hoặc repo Kitware) |
| **OpenCV + contrib** | 4.10.0, CUDA ON | **SOURCE** → `/usr/local` |
| **ONNX Runtime** | 1.23.2, CUDA EP | **SOURCE** → `/usr/local` |
| **GTSAM** | 4.3a1 (`adfd9c15`) | **SOURCE** → `/usr/local` |
| **g2o** | `502c4077` (20241228+52) | **SOURCE** → build tree in-place |
| **spdlog** | v1.17.0 (`d5275e33`) | **SOURCE** → `<repo>/.local` |
| **Iridescence** | v1.0.0+ (`a3d11ff`) | **SOURCE** → `<repo>/.local` |
| **GLFW** | 3.4 | **SOURCE** — Iridescence tự tải, cài vào `.local` |
| **satVPR** | `4e1f143` | **SOURCE** — repo riêng, build in-tree |

**KHÔNG phải dependency của uavloc** (có trên máy nhưng do việc khác để lại):
`stella_vslam`, `fbow`, `ceres-solver`, `faiss`, `easy_profiler`, và bản `libg2o_*.so`
nằm ở `/usr/local/lib`. Link line của `libuavloc.so` không tham chiếu cái nào trong số đó.

---

## 2. Chuẩn bị

```bash
sudo apt update
sudo apt install -y build-essential git pkg-config curl wget unzip ca-certificates
```

Đặt biến môi trường dùng chung cho cả tài liệu:

```bash
export UAVLOC=$HOME/Desktop/Workplace/HUST/uav_localization   # đổi cho đúng máy bạn
export SRC=$HOME/sources                                       # nơi để source thư viện
mkdir -p "$SRC" "$UAVLOC/.thirdparty" "$UAVLOC/.local"
```

---

## 3. CMake ≥ 3.28 (source hoặc repo Kitware)

Ubuntu 22.04 chỉ có CMake 3.22.1. Dự án khai báo `cmake_minimum_required(VERSION 3.16)`
nên 3.22 về lý thuyết đủ, nhưng máy tham chiếu dùng **3.28.3** và ONNX Runtime 1.23 yêu cầu
CMake mới. Dùng repo Kitware cho nhanh:

```bash
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc \
  | sudo gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] \
https://apt.kitware.com/ubuntu/ jammy main" \
  | sudo tee /etc/apt/sources.list.d/kitware.list
sudo apt update && sudo apt install -y cmake
cmake --version    # kỳ vọng >= 3.28
```

Hoặc build từ source đúng như máy tham chiếu:

```bash
cd "$SRC"
wget https://github.com/Kitware/CMake/releases/download/v3.28.3/cmake-3.28.3.tar.gz
tar xf cmake-3.28.3.tar.gz && cd cmake-3.28.3
./bootstrap --parallel=$(nproc) && make -j$(nproc)
sudo make install          # -> /usr/local/bin/cmake
hash -r
```

---

## 4. NVIDIA driver + CUDA 12.8 + cuDNN 9.15 (apt, repo NVIDIA)

Bắt buộc cho OpenCV-CUDA và ONNX Runtime CUDA EP.

```bash
# Driver (RTX 5090 cần >= 570; máy tham chiếu dùng 580 open)
sudo apt install -y nvidia-driver-580-open
# reboot sau khi cài driver
```

```bash
# CUDA Toolkit 12.8
cd "$SRC"
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-8
```

```bash
# cuDNN 9.15 cho CUDA 12
sudo apt install -y cudnn9-cuda-12 libcudnn9-dev-cuda-12
```

Thêm vào `~/.bashrc`:

```bash
export PATH=/usr/local/cuda/bin:$PATH
export CUDA_HOME=/usr/local/cuda
```

Kiểm tra:

```bash
nvidia-smi                      # thấy RTX 5090, driver 580.x
nvcc --version                  # release 12.8
ls /usr/local/cuda/lib64/libcudart.so
```

**Lưu ý:** `libcudnn` trên máy tham chiếu nằm ở `/usr/lib/x86_64-linux-gnu`, không phải
trong `/usr/local/cuda`. Đường dẫn này cần cho tham số `--cudnn_home` của ONNX Runtime.

---

## 5. Toàn bộ gói apt

Chạy một lần, gộp cả dependency trực tiếp của uavloc lẫn dependency để build OpenCV/GTSAM/g2o.

```bash
sudo apt install -y \
  libeigen3-dev \
  libboost-all-dev \
  libtbb-dev \
  libyaml-cpp-dev \
  nlohmann-json3-dev \
  libzbar-dev \
  libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
  libgl1-mesa-dev libglu1-mesa-dev \
  libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
  libpng-dev libjpeg-dev zlib1g-dev \
  libsuitesparse-dev \
  libgtk-3-dev \
  libtiff-dev libwebp-dev libopenexr-dev libopenjp2-7-dev \
  libdc1394-dev libv4l-dev \
  libfreetype-dev libharfbuzz-dev libhdf5-dev \
  libopenblas-dev libatlas-base-dev \
  libglm-dev libglfw3-dev \
  python3-dev python3-numpy python3-venv
```

### 5.1 Ghi chú từng gói apt

- **`libeigen3-dev` (3.4.0)** — headers ở `/usr/include/eigen3`, config CMake ở
  `/usr/share/eigen3/cmake`. Dùng bởi uavloc, g2o, Iridescence, satVPR.
  GTSAM thì **không** dùng bản này (xem §8).
- **`libboost-all-dev` (1.74.0)** — GTSAM cần `serialization, filesystem, thread, atomic,
  date_time, regex, timer, chrono`. Cài `-all-dev` cho gọn.
- **`libtbb-dev` (2021.5.0)** — bắt buộc vì GTSAM được build với `GTSAM_WITH_TBB=ON`
  và `GTSAM_DEFAULT_ALLOCATOR=TBB`. Nếu thiếu, GTSAM sẽ build ra ABI khác và uavloc link hỏng.
- **`libyaml-cpp-dev` (0.7.0)** — đọc file `config/*.yaml`. Link tới
  `/usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.7.0`.
- **`nlohmann-json3-dev` (3.10.5)** — trên Ubuntu 22.04 gói này **không có CMake package
  config dùng được theo cách thông thường**; satVPR tìm nó bằng `find_path(NLOHMANN_JSON_INCLUDE_DIR
  nlohmann/json.hpp)` và ra `/usr/include`. Không cần làm gì thêm.
- **`libzbar-dev`** — dùng bởi `src/debug_viewer/barcode_decoder.cpp` (giải barcode đồng bộ
  frame). Tìm qua `pkg-config zbar`.
- **`libav*-dev` (FFmpeg 4.4.2)** — bật define `UAVLOC_WITH_LIBAV` cho `sensor/video_reader.cpp`
  (đọc MPEG-TS + KLV). Tìm qua `pkg-config libavformat libavcodec libavutil`.
  Nếu thiếu, CMake vẫn chạy nhưng `UAVLOC_WITH_LIBAV` tắt và mất khả năng đọc dataset `.ts`.
- **`libsuitesparse-dev`** — cho `G2O_USE_CHOLMOD` / `G2O_USE_CSPARSE`.
  Trên máy tham chiếu g2o **không** tìm thấy CHOLMOD/CSparse (`CHOLMOD_DIR-NOTFOUND`) và vẫn build
  ổn, vì uavloc chỉ dùng `g2o_core` + `g2o_stuff` với solver Eigen. Gói này là tùy chọn an toàn.
- **`libglm-dev`, `libglfw3-dev`** — nếu cài trước, Iridescence sẽ dùng bản hệ thống thay vì
  tự tải về. Xem cảnh báo ở §10.
- Nhóm `libgtk-3-dev / libtiff / libwebp / libopenexr / libopenjp2 / libdc1394 / libv4l /
  libfreetype / libharfbuzz / libhdf5 / openblas / atlas` — **chỉ cần để build OpenCV** với đầy đủ
  module như máy tham chiếu (highgui GTK3, freetype, hdf, videoio v4l2).

---

## 6. OpenCV 4.10.0 + opencv_contrib — **BUILD TỪ SOURCE**

Không dùng `libopencv-dev` của apt: bản apt là 4.5.4, không có module CUDA và không có
`opencv_contrib` (`xfeatures2d`, `ximgproc`, `wechat_qrcode`, `freetype`, `hdf`…) mà dự án link tới.

```bash
cd "$SRC"
git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv.git
git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv_contrib.git

cd "$SRC/opencv"
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DOPENCV_EXTRA_MODULES_PATH="$SRC/opencv_contrib/modules" \
  -DWITH_CUDA=ON \
  -DWITH_CUDNN=ON \
  -DOPENCV_DNN_CUDA=ON \
  -DCUDA_ARCH_BIN=12.0 \
  -DCUDA_ARCH_PTX= \
  -DWITH_CUBLAS=ON \
  -DWITH_CUFFT=ON \
  -DWITH_FFMPEG=ON \
  -DWITH_GTK=ON \
  -DWITH_V4L=ON \
  -DWITH_OPENGL=OFF \
  -DWITH_GSTREAMER=OFF \
  -DWITH_TBB=OFF \
  -DBUILD_opencv_python3=ON \
  -DBUILD_opencv_world=OFF \
  -DOPENCV_ENABLE_NONFREE=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_DOCS=OFF \
  -DCPU_BASELINE=SSE3 \
  -DCPU_DISPATCH=SSE4_1,SSE4_2,AVX,FP16,AVX2,AVX512_SKX

make -j"$(nproc)"
sudo make install
sudo ldconfig
```

Kiểm tra:

```bash
opencv_version                    # 4.10.0
opencv_version -v | grep -A2 "NVIDIA CUDA"   # phải thấy: YES (ver 12.8, CUFFT CUBLAS)
opencv_version -v | grep "GPU arch"          # phải thấy: 120
ls /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake
```

### Lưu ý
- **`CUDA_ARCH_BIN=12.0` là bắt buộc cho RTX 5090.** Đặt sai (ví dụ 8.6) thì build xong
  nhưng mọi hàm `cv::cuda::*` sẽ báo lỗi no kernel image.
- Để `CUDA_ARCH_PTX` rỗng để không sinh PTX — build nhanh hơn nhiều, đúng như máy tham chiếu.
- Thời gian build: **1.5–3 giờ**, tốn ~30 GB đĩa. Cần ≥16 GB RAM cho `-j$(nproc)`;
  nếu OOM thì giảm còn `-j8`.
- `OPENCV_ENABLE_NONFREE=OFF` — máy tham chiếu ghi `Non-free algorithms: NO`. Giữ nguyên.
- Nếu máy không có GPU NVIDIA: đặt `-DWITH_CUDA=OFF -DWITH_CUDNN=OFF -DOPENCV_DNN_CUDA=OFF`.
  `libuavloc.so` vẫn link được vì nó không gọi trực tiếp API CUDA của OpenCV,
  nhưng khi đó phần VPR chạy CPU sẽ chậm.

---

## 7. ONNX Runtime 1.23.2 (CUDA EP) — **BUILD TỪ SOURCE**

Dùng bởi satVPR: DINOv2 backbone, SuperPoint, LightGlue.
Bản prebuilt trên GitHub không kèm CMake package config `onnxruntimeConfig.cmake`
mà `find_package(onnxruntime REQUIRED)` của satVPR cần → phải build từ source.

```bash
cd "$SRC"
git clone --recursive --branch v1.23.2 --depth 1 https://github.com/microsoft/onnxruntime.git
cd onnxruntime

./build.sh \
  --config Release \
  --build_shared_lib \
  --parallel "$(nproc)" \
  --use_cuda \
  --cuda_home /usr/local/cuda \
  --cudnn_home /usr/lib/x86_64-linux-gnu \
  --skip_tests \
  --cmake_extra_defines \
      CMAKE_INSTALL_PREFIX=/usr/local \
      CMAKE_CUDA_ARCHITECTURES=120 \
      onnxruntime_ENABLE_PYTHON=OFF

sudo cmake --install build/Linux/Release
sudo ldconfig
```

Kiểm tra:

```bash
ls /usr/local/lib/libonnxruntime.so.1.23.2
ls /usr/local/lib/libonnxruntime_providers_cuda.so
ls /usr/local/lib/cmake/onnxruntime/onnxruntimeConfig.cmake
```

### Lưu ý
- `--cudnn_home` là `/usr/lib/x86_64-linux-gnu` (nơi apt đặt cuDNN), **không phải**
  `/usr/local/cuda`. Sai chỗ này sẽ lỗi ở bước configure.
- `CMAKE_CUDA_ARCHITECTURES=120` cho RTX 5090.
- `libonnxruntime_providers_cuda.so` được dlopen lúc runtime, không có trong link line.
  Phải nằm trong đường tìm của loader (`/usr/local/lib` đã có trong `ld.so.conf`).
- Build mất **1–2 giờ**.

---

---

## 7B. Cài bản ONNX Runtime **thấp hơn** 1.23.2

Mục này dành cho ba tình huống: (1) không muốn chờ 1–2 giờ build source, (2) cài trên máy
**không phải** RTX 5090 (GPU cũ hơn, CUDA cũ hơn), (3) chỉ cần chạy CPU. Máy tham chiếu vẫn
là 1.23.2 — mọi con số kết quả trong `.docs/` sinh ra với bản đó.

### 7B.1 Sàn phiên bản — quyết định bởi chính 3 model ONNX

Đọc trực tiếp từ file `.onnx` trong `.weights/` **[đo được]**:

| Model | `ir_version` | opset | Producer |
| --- | :--: | :--: | --- |
| `dino/dino_vits8_layer9_norm.onnx` | 8 | **16** | pytorch 2.7.1 |
| `superpoint.onnx` | 8 | **16** | pytorch 2.0.1 |
| `superpoint_lightglue.onnx` | 8 | **16** | pytorch 2.0.1 |

Tự kiểm lại `ir_version` không cần cài gói `onnx`:

```bash
for m in "$HOME/.projects/satVPR"/.weights/dino/*.onnx "$HOME/.projects/satVPR"/.weights/*.onnx; do
  python3 -c "b=open('$m','rb').read(2); print('$m'.split('/')[-1], '-> ir_version', b[1])"
done
```

⇒ ONNX Runtime chỉ cần **hỗ trợ IR 8 + opset 16**, tức mọi bản **≥ 1.12** đều nạp được model.
Nếu bản ORT quá cũ, lỗi hiện ra rất rõ ràng lúc `Ort::Session` khởi tạo:
`Unsupported model IR version: 8, max supported IR version: ...`.

**Mã nguồn cũng không đòi bản mới.** satVPR chỉ dùng API C++ cổ điển — `Ort::Env`,
`Ort::Session`, `Ort::SessionOptions`, `Ort::Value`, `Ort::MemoryInfo`, `Ort::RunOptions`,
`Ort::AllocatorWithDefaultOptions`, `Ort::Exception` — và struct CUDA **kiểu cũ**
`OrtCUDAProviderOptions` (không phải `...OptionsV2`). Toàn bộ đã ổn định từ ORT 1.10.
`CMakeLists.txt` của satVPR gọi `find_package(onnxruntime REQUIRED)` **không kèm ràng buộc
version**, nên không có sàn nào từ phía build.

### 7B.2 Trần phiên bản — quyết định bởi GPU

| GPU đích | Ràng buộc | Khuyến nghị |
| --- | --- | --- |
| **RTX 5090 / Blackwell (sm_120)** | cần CUDA 12.8; kernel CUTLASS/flash-attention của ORT đời cũ chưa biên dịch được cho sm_120 | **đừng hạ dưới 1.22**; bản prebuilt ≤ 1.21 **không** có kernel sm_120 |
| Ada / Ampere (sm_89, sm_86) | CUDA 12.x + cuDNN 9 (ORT ≥ 1.19) hoặc cuDNN 8 (ORT ≤ 1.18) | 1.18–1.20 đều dùng được |
| Không có GPU | — | tarball CPU, không cần CUDA/cuDNN |

> ⚠ **Chưa kiểm trên máy tham chiếu.** Bảng này là ràng buộc suy từ tài liệu ORT/CUDA, không
> phải kết quả build thật ở đây. Triệu chứng khi chọn sai: `no kernel image is available for
> execution` ngay ở lời gọi CUDA đầu tiên (xem §15).

### 7B.3 Cách A — dùng tarball prebuilt + **tự viết CMake config** (~2 phút)

§7 nói bản prebuilt "không dùng được" vì thiếu `onnxruntimeConfig.cmake`. Điều đó đúng, nhưng
**file đó tự viết được** — nó chỉ khai báo một imported target. Đây là cách nhanh nhất để có
một bản thấp hơn.

```bash
ORT_VER=1.20.1                       # đổi thành version muốn cài
ORT_PKG=onnxruntime-linux-x64-gpu-${ORT_VER}     # bản CPU: bỏ '-gpu-' -> onnxruntime-linux-x64-${ORT_VER}

cd "$SRC"
wget "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VER}/${ORT_PKG}.tgz"
tar xzf "${ORT_PKG}.tgz"

# 1. Chép header + thư viện vào /usr/local
sudo cp -r "${ORT_PKG}/include/." /usr/local/include/
sudo cp -P  "${ORT_PKG}/lib/"*.so* /usr/local/lib/

# 2. Tạo symlink soname (tarball đôi khi thiếu)
cd /usr/local/lib
sudo ln -sf "libonnxruntime.so.${ORT_VER}" libonnxruntime.so.1
sudo ln -sf libonnxruntime.so.1            libonnxruntime.so
sudo ldconfig
```

Rồi sinh CMake package config:

```bash
sudo mkdir -p /usr/local/lib/cmake/onnxruntime

sudo tee /usr/local/lib/cmake/onnxruntime/onnxruntimeConfig.cmake >/dev/null <<'EOF'
get_filename_component(_ort_prefix "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION             "${_ort_prefix}/lib/libonnxruntime.so"
  IMPORTED_SONAME               "libonnxruntime.so.1"
  INTERFACE_INCLUDE_DIRECTORIES "${_ort_prefix}/include")

set(onnxruntime_INCLUDE_DIRS "${_ort_prefix}/include")
set(onnxruntime_LIBRARIES    onnxruntime::onnxruntime)
set(onnxruntime_FOUND TRUE)
EOF

sudo tee /usr/local/lib/cmake/onnxruntime/onnxruntimeConfigVersion.cmake >/dev/null <<EOF
set(PACKAGE_VERSION "${ORT_VER}")
if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if(PACKAGE_FIND_VERSION STREQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
EOF
```

Tên target `onnxruntime::onnxruntime` **phải giữ nguyên** — đó chính là tên
`target_link_libraries(satvpr_core PRIVATE onnxruntime::onnxruntime)` đang dùng.

**Ba khác biệt so với bản build từ source (§7):**

- Tarball GPU đã kèm sẵn `libonnxruntime_providers_cuda.so` và
  `libonnxruntime_providers_shared.so` — chép cả hai, nếu không CUDA EP sẽ im lặng rơi về CPU.
- Prebuilt biên dịch cho **một tập kiến trúc CUDA cố định**; nếu GPU của bạn không nằm trong đó
  thì hỏng lúc chạy chứ không phải lúc build.
- Tarball GPU của ORT ≥ 1.19 cần **cuDNN 9**; bản ≤ 1.18 cần **cuDNN 8**. Cài nhầm nhánh cuDNN
  sẽ lỗi `libcudnn.so.X: cannot open shared object file`.

### 7B.4 Cách B — build từ source ở tag thấp hơn

Giữ nguyên toàn bộ lệnh của §7, **chỉ đổi tag**:

```bash
ORT_VER=1.20.1
git clone --recursive --branch v${ORT_VER} --depth 1 https://github.com/microsoft/onnxruntime.git
```

Ba điều cần chỉnh theo version:

- `CMAKE_CUDA_ARCHITECTURES` — đặt đúng compute capability của GPU đích (`86`, `89`, `120`…),
  không copy nguyên `120` nếu máy không phải Blackwell.
- `--cudnn_home` — trỏ tới nhánh cuDNN mà version đó yêu cầu (xem 7B.3).
- Bản ORT càng cũ càng kén compiler: nếu gặp lỗi biên dịch CUDA khó hiểu với gcc 11, thử
  `--allow_running_as_root` bỏ qua và **ưu tiên Cách A** thay vì đi sâu sửa build.

### 7B.5 Hạ version khi máy **đã có** 1.23.2

Phải dọn bản cũ trước, nếu không loader sẽ vẫn thấy file cũ:

```bash
sudo rm -f /usr/local/lib/libonnxruntime.so*                 \
           /usr/local/lib/libonnxruntime_providers_*.so
sudo rm -rf /usr/local/lib/cmake/onnxruntime                 \
            /usr/local/include/onnxruntime_*.h               \
            /usr/local/include/cpu_provider_factory.h
sudo ldconfig
```

Rồi cài lại theo 7B.3 hoặc 7B.4, và **build lại từ đầu**:

```bash
rm -rf "$UAVLOC/build" "$HOME/.projects/satVPR/build"
# rồi configure + build lại theo §13
```

> `SONAME` của mọi bản 1.x đều là `libonnxruntime.so.1` **[đo được trên 1.23.2]**, nên link line
> cũ về mặt kỹ thuật vẫn giải được. Dù vậy **vẫn nên xoá `build/`**: header ORT nằm trong
> dependency đã cache của CMake, và đổi header mà không cấu hình lại là kiểu lỗi hỏng-im-lặng.

### 7B.6 Kiểm chứng sau khi cài

```bash
ls -l /usr/local/lib/libonnxruntime.so*          # realname phải mang đúng version vừa cài
ls    /usr/local/lib/libonnxruntime_providers_cuda.so
ls    /usr/local/lib/cmake/onnxruntime/onnxruntimeConfig.cmake
ldconfig -p | grep onnxruntime
```

Kiểm ở mức chạy thật — nạp đủ cả ba model và đi qua CUDA EP:

```bash
cd "$UAVLOC/build"
ctest -R vpr --output-on-failure          # cần data/ và config/*.yaml, xem §13
```

Nếu bản ORT quá cũ so với model, test hỏng ngay lúc tạo session với thông báo
`Unsupported model IR version`. Nếu sai kiến trúc CUDA, test chạy tới lời gọi CUDA đầu tiên rồi
báo `no kernel image is available for execution`.

⚠ **Hạ version rồi thì đừng trộn số liệu.** Mọi kết quả trong `.docs/reports/` chạy trên
1.23.2; ORT khác version có thể đổi thứ tự phép toán dấu phẩy động ⇒ số inlier và chuỗi re-init
lệch đi. Nếu báo cáo bằng số của bản khác, phải ghi rõ version ngay tại bảng.

## 8. GTSAM 4.3a1 — **BUILD TỪ SOURCE**

Không dùng gói apt/PPA: dự án bám nhánh `develop` (commit `adfd9c15`) và link
`libgtsam.so.4.3a1`.

```bash
cd "$UAVLOC/.thirdparty"
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout adfd9c153c7edf93e00eea04f89b105db8f83e26

mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_SHARED_LIBS=ON \
  -DGTSAM_WITH_TBB=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=OFF \
  -DGTSAM_USE_SYSTEM_METIS=OFF \
  -DGTSAM_ENABLE_BOOST_SERIALIZATION=ON \
  -DGTSAM_USE_BOOST_FEATURES=ON \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_METIS_EXECUTABLES=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_QUATERNIONS=OFF \
  -DGTSAM_POSE3_EXPMAP=ON \
  -DGTSAM_ROT3_EXPMAP=ON \
  -DGTSAM_TANGENT_PREINTEGRATION=ON \
  -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=ON

make -j"$(nproc)"
sudo make install
sudo ldconfig
```

Kiểm tra:

```bash
ls /usr/local/lib/libgtsam.so.4.3a1
ls /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake
ls /usr/local/lib/libmetis-gtsam.so /usr/local/lib/libcephes-gtsam.so
```

### Lưu ý — đây là mục dễ sai nhất
- **`GTSAM_USE_SYSTEM_EIGEN=OFF`**: GTSAM cài Eigen bản riêng của nó vào
  `/usr/local/include/gtsam/3rdparty/Eigen`. Đây là cấu hình đã build thành công.
  Đổi sang `ON` sẽ tạo ra hai bản Eigen khác nhau trong cùng chương trình → sai lệch
  alignment và lỗi runtime khó truy.
- **`GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`**: bắt buộc để `-march` của GTSAM khớp với uavloc.
  Bật lên là nguồn gốc kinh điển của segfault trong Eigen do lệch alignment.
- **`GTSAM_WITH_TBB=ON`** kéo theo `GTSAM_DEFAULT_ALLOCATOR=TBB`. Phải cài `libtbb-dev`
  **trước** khi configure GTSAM.
- GTSAM cài kèm `libmetis-gtsam.so` và `libcephes-gtsam.so` vào `/usr/local/lib` —
  cả hai đều có trong link line của `libuavloc.so`. Đừng xóa.
- `GTSAM_BUILD_WITH_CCACHE` mặc định ON nhưng máy tham chiếu không cài ccache; tùy chọn này
  tự no-op. Muốn build nhanh hơn thì `sudo apt install ccache`.
- Build mất **30–60 phút**.

---

## 9. g2o — **BUILD TỪ SOURCE, KHÔNG INSTALL**

uavloc link **thẳng vào build tree**: `.thirdparty/g2o/build/lib/libg2o_core.so`
và `libg2o_stuff.so`, include từ `.thirdparty/g2o` + `.thirdparty/g2o/build`.
Vì vậy **không được `make install`** và **không được xóa thư mục build**.

```bash
cd "$UAVLOC/.thirdparty"
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 502c40774af0bdd404678e8eb12ff08ee5d246ec

mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DG2O_USE_OPENMP=OFF \
  -DG2O_USE_LOGGING=OFF \
  -DG2O_BUILD_APPS=OFF \
  -DG2O_BUILD_EXAMPLES=OFF \
  -DBUILD_UNITTESTS=OFF \
  -DG2O_BUILD_SBA_TYPES=ON \
  -DG2O_BUILD_ICP_TYPES=ON \
  -DG2O_BUILD_DATA_TYPES=ON

make -j"$(nproc)"
# KHÔNG chạy make install
```

Kiểm tra:

```bash
ls "$UAVLOC/.thirdparty/g2o/build/lib/libg2o_core.so"
ls "$UAVLOC/.thirdparty/g2o/build/lib/libg2o_stuff.so"
ls "$UAVLOC/.thirdparty/g2o/build/g2o/config.h"     # header sinh ra lúc build
```

### Lưu ý
- **`BUILD_WITH_MARCH_NATIVE=OFF`** — cùng lý do như GTSAM.
- **`G2O_USE_LOGGING=OFF`** — khuyến nghị mạnh, khác với máy tham chiếu.
  Trên máy tham chiếu tùy chọn này để ON và `find_package(spdlog 1.6 QUIET)` đã **vô tình
  bắt trúng spdlog của miniconda**:

  ```
  libspdlog.so.1.11 => /home/<user>/miniconda3/lib/libspdlog.so.1.11
  libfmt.so.9       => /home/<user>/miniconda3/lib/libfmt.so.9
  ```

  Hậu quả: `libuavloc.so` phụ thuộc runtime vào miniconda. Xóa/di chuyển miniconda là
  chương trình chết. Đặt `G2O_USE_LOGGING=OFF` để cắt hẳn nhánh này.
  Nếu vẫn muốn để ON, hãy `conda deactivate` và bỏ conda khỏi `PATH`/`CMAKE_PREFIX_PATH`
  trước khi configure g2o.
- `G2O_INSTALL_CMAKE_CONFIG` để mặc định OFF; uavloc không dùng `find_package(g2o)`.
- Nếu `/usr/local/lib` đã có `libg2o_*.so.0.2` từ lần cài trước (ví dụ khi build
  stella_vslam), **xóa hoặc bỏ qua** — chúng không được dùng và dễ gây nhầm khi debug.
- Build mất **5–15 phút**.

---

## 10. spdlog v1.17.0 — **BUILD TỪ SOURCE** → `<repo>/.local`

Không dùng `libspdlog-dev` của apt (bản 1.9, quá cũ).
uavloc dùng spdlog ở chế độ **header-only** (define `SPDLOG_FWRITE_UNLOCKED` xuất hiện trong
mọi TU nhưng không có `libspdlog` trong link line), tuy vậy vẫn cần bước install để có
package config.

```bash
cd "$UAVLOC/.thirdparty"
git clone --branch v1.x https://github.com/gabime/spdlog.git
cd spdlog
git checkout d5275e3343c4dca4feb9fa772199181509f2a657

mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$UAVLOC/.local" \
  -DSPDLOG_ENABLE_PCH=OFF
make -j"$(nproc)"
make install        # KHÔNG sudo — cài vào .local trong repo
```

Kiểm tra:

```bash
ls "$UAVLOC/.local/lib/cmake/spdlog/spdlogConfig.cmake"
ls "$UAVLOC/.local/include/spdlog/spdlog.h"
```

---

## 11. Iridescence — **BUILD TỪ SOURCE** → `<repo>/.local`

Thư viện viewer OpenGL, dùng cho module `debug_viewer` (`ENABLE_VIEWER=ON`).
Kéo theo GLFW và GLM.

```bash
cd "$UAVLOC/.thirdparty"
git clone --recursive https://github.com/koide3/iridescence.git
cd iridescence
git checkout a3d11ffe9fc01c216856aa3b40396b1d8fd64b05
git submodule update --init --recursive

mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$UAVLOC/.local" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_EXT_TESTS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF
make -j"$(nproc)"
make install        # KHÔNG sudo
```

Kiểm tra:

```bash
ls "$UAVLOC/.local/lib/libiridescence.so.1.0.1"
ls "$UAVLOC/.local/lib/cmake/iridescence/"
ls "$UAVLOC/.local/lib/libglfw.so.3.4"           # xem lưu ý bên dưới
```

### Lưu ý
- `--recursive` là bắt buộc: `thirdparty/` chứa `gl3w`, `imgui`, `ImGuizmo`, `implot`,
  `portable-file-dialogs`, `rapidhash`.
- **GLFW**: Iridescence chạy `find_package(glfw3)`; nếu không thấy, nó tự
  `FetchContent` GLFW **3.4** rồi build và cài vào cùng prefix.
  Trên máy tham chiếu đây chính là điều đã xảy ra — `.local/lib/libglfw.so.3.4` và
  `.local/lib/cmake/glfw3/` là do Iridescence sinh ra, dù apt đã có `libglfw3-dev` 3.3.6.
  Cả hai đường đều chạy được; chỉ cần biết `glfw3_DIR` mà uavloc dùng là
  `$UAVLOC/.local/lib/cmake/glfw3`.
- **GLM**: tương tự, nếu không có `libglm-dev` thì tải GLM 1.0.1.
  Hai bước FetchContent này **cần mạng** lúc configure.
- `assimp` là `find_package(... QUIET)` — bỏ qua được, máy tham chiếu không cài.
- `BUILD_WITH_MARCH_NATIVE=OFF` — giữ nguyên cho khớp ABI.

---

## 12. satVPR — repo riêng, build in-tree

`libsatvpr_core.a` được `add_subdirectory` vào uavloc và nhúng tĩnh vào `libuavloc.so`
(có `-Wl,--exclude-libs,libsatvpr_core.a` trong link line). Đây là repo tách rời.

```bash
mkdir -p "$HOME/.projects" && cd "$HOME/.projects"
git clone https://github.com/Minkeisme1204/Satellite-domain-Visual-Place-Recognition.git satVPR
cd satVPR
git checkout 4e1f143
```

Không cần build riêng — uavloc sẽ `add_subdirectory` nó qua biến `SATVPR_ROOT`.
Nếu muốn build/test độc lập:

```bash
cd "$HOME/.projects/satVPR"
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSATVPR_BUILD_TESTS=OFF
cmake --build build -j"$(nproc)"
```

### Lưu ý
- Dependency của satVPR: **OpenCV** (core, imgproc, imgcodecs, calib3d), **Eigen3**,
  **ONNX Runtime**, **nlohmann/json**. Đều đã cài ở các bước trên.
- `SATVPR_NATIVE_ARCH=ON` (mặc định) thêm `-O3 -march=native` **chỉ cho target satvpr_core**
  (PRIVATE, không lan sang uavloc). Nếu build để deploy sang máy CPU khác:
  `-DSATVPR_NATIVE_ARCH=OFF`.
- **`SATVPR_BUILD_TESTS=OFF` khi build trong uavloc** — test của satVPR cần `dataset/`
  và `.weights/` (model ONNX) không có trong repo.
- Model weights ONNX (DINOv2 / SuperPoint / LightGlue) nằm ở `~/.projects/satVPR/.weights`
  và không đi kèm git. Đây là **dữ liệu runtime**, không phải dependency build —
  thiếu chúng vẫn build được, chỉ không chạy được test VPR.
- Phần Python của satVPR (nếu cần chạy script sinh vocabulary/PCA) dùng venv riêng
  và bắt buộc PyTorch bản cu128 cho sm_120:
  ```bash
  cd "$HOME/.projects/satVPR"
  python3 -m venv .env
  .env/bin/pip install torch==2.11.0+cu128 torchvision==0.26.0+cu128 \
      --index-url https://download.pytorch.org/whl/cu128
  .env/bin/pip install -r requirements.txt
  ```

---

## 13. Build `uavloc`

```bash
cd "$UAVLOC"
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$UAVLOC/.local;/usr/local" \
  -DSATVPR_ROOT="$HOME/.projects/satVPR" \
  -DSATVPR_BUILD_TESTS=OFF \
  -DSATVPR_NATIVE_ARCH=ON \
  -DENABLE_VIEWER=ON \
  -DUSE_SSE_ORB=OFF \
  -DBUILD_TESTING=ON

cmake --build build -j"$(nproc)"
```

Kết quả:

```bash
ls build/lib/libuavloc.so
ls build/lib/libsatvpr_core.a
ls build/tests/           # test_full_flight, test_vo_viewer, test_vpr, ...
```

### Các option của dự án

| Option | Giá trị tham chiếu | Ý nghĩa |
| --- | --- | --- |
| `CMAKE_BUILD_TYPE` | `Release` | `-O3 -DNDEBUG` |
| `ENABLE_VIEWER` | `ON` | Bật `debug_viewer` (cần Iridescence). Đặt `OFF` để build headless, khi đó bỏ được §11 |
| `USE_SSE_ORB` | `OFF` | Dùng đường ORB tổng quát thay vì nhánh SSE của stella_vslam. **Giữ OFF** — bật lên đổi kết quả số học |
| `BUILD_TESTING` | `ON` | Build `tests/` và đăng ký CTest |
| `SATVPR_ROOT` | `~/.projects/satVPR` | Đường dẫn tới repo satVPR |
| `SATVPR_BUILD_TESTS` | `OFF` | Tắt test riêng của satVPR |
| `SATVPR_NATIVE_ARCH` | `ON` | `-march=native` cho riêng satvpr_core |

### Define sinh ra lúc build (tham khảo, không cần đặt tay)

- `UAVLOC_WITH_DEBUG_VIEWER` — khi `ENABLE_VIEWER=ON`
- `UAVLOC_WITH_LIBAV` — khi `pkg-config` tìm thấy FFmpeg
- `UAVLOC_HAVE_VPR=1` — khi satVPR + ONNX Runtime sẵn sàng
- `SATVPR_DATA_ROOT`, `UAVLOC_MISSION_CONFIG_PATH` — đường dẫn tuyệt đối nhúng vào test.
  **Vì vậy build tree không di chuyển được**: đổi chỗ repo thì phải configure lại từ đầu.

### Chạy test

```bash
cd "$UAVLOC/build"
ctest --output-on-failure
```

Phần lớn test cần dataset ở `data/` và config ở `config/*.yaml`; chúng không nằm trong
repo và sẽ fail nếu thiếu. Build thành công không phụ thuộc vào chúng.

---

## 14. Bảng kiểm tra nhanh sau khi cài

```bash
cmake --version                                   | head -1
gcc --version                                     | head -1
nvcc --version                                    | tail -2
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
opencv_version
opencv_version -v | grep -E "NVIDIA CUDA|GPU arch"
ls /usr/local/lib/libgtsam.so.4.3a1
ls /usr/local/lib/libonnxruntime.so.1.23.2
ls "$UAVLOC/.thirdparty/g2o/build/lib/libg2o_core.so"
ls "$UAVLOC/.local/lib/libiridescence.so.1.0.1"
ls "$UAVLOC/.local/lib/cmake/spdlog"
pkg-config --modversion yaml-cpp zbar libavformat
```

Sau khi build xong, kiểm tra không có phụ thuộc lạ:

```bash
ldd build/lib/libuavloc.so | grep -iE "not found|conda"
# Kỳ vọng: không in ra gì.
```

---

## 15. Xử lý sự cố

| Triệu chứng | Nguyên nhân | Cách xử lý |
| --- | --- | --- |
| `Could NOT find iridescence` / `spdlog` | Thiếu `.local` trong prefix path | Thêm `-DCMAKE_PREFIX_PATH="$UAVLOC/.local;/usr/local"` |
| `undefined reference to g2o::...` | Đã `make install` g2o hoặc xóa build tree | Build lại g2o, giữ nguyên `.thirdparty/g2o/build/` |
| Segfault trong Eigen/GTSAM ngay lúc chạy | Lệch `-march=native` giữa các thư viện | Build lại GTSAM/g2o/Iridescence với `MARCH_NATIVE=OFF` |
| `libspdlog.so.1.11 => .../miniconda3/...` | g2o bắt nhầm spdlog của conda | Build lại g2o với `-DG2O_USE_LOGGING=OFF` |
| `no kernel image is available for execution` | OpenCV/ORT build sai CUDA arch | Build lại với arch `12.0` / `CMAKE_CUDA_ARCHITECTURES=120` |
| `find_package(onnxruntime)` thất bại | Dùng bản prebuilt, thiếu CMake config | Build từ source theo §7, **hoặc** tự sinh config theo §7B.3 |
| `Unsupported model IR version: 8` | ORT quá cũ so với model (`.onnx` là IR 8 / opset 16) | Dùng ORT ≥ 1.12 — xem §7B.1 |
| CUDA EP im lặng rơi về CPU | Thiếu `libonnxruntime_providers_cuda.so` cạnh `libonnxruntime.so` | Chép đủ `libonnxruntime_providers_*.so` (§7B.3) rồi `sudo ldconfig` |
| `libgtsam.so.4.3a1: cannot open shared object file` | Chưa chạy `ldconfig` | `sudo ldconfig` |
| Không đọc được video `.ts` | Thiếu `libav*-dev` lúc configure | Cài FFmpeg dev rồi **xóa `build/` và configure lại** |
| GTSAM link lỗi liên quan TBB | GTSAM build khi chưa có `libtbb-dev` | Cài `libtbb-dev`, xóa `.thirdparty/gtsam/build`, build lại |
| Test fail vì không tìm thấy config/dataset | Đường dẫn tuyệt đối nhúng lúc configure | Configure lại sau khi đặt đúng `data/`, `config/` |

---

## 16. Tổng thời gian và dung lượng

| Bước | Thời gian | Đĩa |
| --- | --- | --- |
| apt + CUDA + cuDNN | ~20 phút | ~10 GB |
| OpenCV + contrib | 1.5–3 giờ | ~30 GB (build), ~1 GB (installed) |
| ONNX Runtime | 1–2 giờ | ~15 GB |
| GTSAM | 30–60 phút | ~5 GB |
| g2o | 5–15 phút | ~1 GB |
| spdlog | < 2 phút | nhỏ |
| Iridescence + GLFW | 5–10 phút | ~1 GB |
| uavloc | 5–15 phút | ~2 GB |

Tổng: **4–7 giờ**, cần ~60 GB đĩa trống trong quá trình build.
Dọn `build/` của OpenCV và ONNX Runtime sau khi cài xong để lấy lại ~45 GB.
