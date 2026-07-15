#pragma once

#include <opencv2/opencv.hpp>
#include <memory>

namespace uavloc::debug_viewer {

// Decodes the per-frame CODE-128 barcode embedded in the YenBai nadir video.
//
// Discovered format (see .docs note): each frame carries a single CODE-128
// symbol near the top edge whose payload is a zero-padded decimal string equal
// to the telemetry CSV `imageId` of that frame (e.g. "0055166"). The barcode
// therefore does NOT carry lat/lon/yaw directly — those are looked up from the
// CSV by the decoded id. This class only extracts that integer id.
class BarcodeDecoder {
public:
    BarcodeDecoder();
    ~BarcodeDecoder();

    BarcodeDecoder(const BarcodeDecoder&)            = delete;
    BarcodeDecoder& operator=(const BarcodeDecoder&) = delete;

    // Scans a grayscale (CV_8UC1) image for the frame-id barcode.
    // On success sets out_image_id to the decoded integer and returns true.
    // Returns false (rate-limited spdlog::warn) when no valid symbol is found.
    bool decode(const cv::Mat& gray, int& out_image_id);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::debug_viewer
