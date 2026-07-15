#include "barcode_decoder.h"

#include <zbar.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <string>

namespace uavloc::debug_viewer {

namespace {

// zbar pixel format for an 8-bit single channel grayscale buffer.
constexpr char ZBAR_GRAY_FORMAT[] = "Y800";

// Emit a decode-failure warning at most once per this many consecutive misses
// so a barcode-free stretch of video does not flood the log.
constexpr std::uint64_t WARN_EVERY_N_FAILURES = 60;

} // anonymous namespace

struct BarcodeDecoder::Impl {
    zbar::ImageScanner scanner;
    std::uint64_t      fail_count = 0;

    Impl() {
        // Enable every supported symbology; the embedded code is CODE-128 but
        // enabling all keeps the decoder robust to format changes.
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    }
};

BarcodeDecoder::BarcodeDecoder() : impl_(std::make_unique<Impl>()) {}

BarcodeDecoder::~BarcodeDecoder() = default;

bool BarcodeDecoder::decode(const cv::Mat& gray, int& out_image_id) {
    if (gray.empty() || gray.type() != CV_8UC1) {
        spdlog::error("BarcodeDecoder::decode: expected non-empty CV_8UC1 image");
        return false;
    }

    // zbar::Image wraps the buffer without owning it; gray must outlive the scan.
    zbar::Image image(gray.cols, gray.rows, ZBAR_GRAY_FORMAT,
                      gray.data, static_cast<unsigned long>(gray.cols) * gray.rows);

    const int n = impl_->scanner.scan(image);
    if (n <= 0) {
        if ((impl_->fail_count++ % WARN_EVERY_N_FAILURES) == 0) {
            spdlog::warn("BarcodeDecoder: no barcode found (failure #{})", impl_->fail_count);
        }
        return false;
    }

    // Take the first symbol whose payload parses as an integer image id.
    for (auto sym = image.symbol_begin(); sym != image.symbol_end(); ++sym) {
        const std::string payload = sym->get_data();
        try {
            std::size_t consumed = 0;
            const int id = std::stoi(payload, &consumed);
            if (consumed == payload.size()) {
                out_image_id = id;
                impl_->fail_count = 0;
                return true;
            }
        } catch (const std::exception&) {
            // Non-numeric payload — keep scanning remaining symbols.
        }
        spdlog::debug("BarcodeDecoder: ignoring non-numeric payload [{}]", payload);
    }

    if ((impl_->fail_count++ % WARN_EVERY_N_FAILURES) == 0) {
        spdlog::warn("BarcodeDecoder: barcode(s) found but none numeric (failure #{})",
                     impl_->fail_count);
    }
    return false;
}

} // namespace uavloc::debug_viewer
