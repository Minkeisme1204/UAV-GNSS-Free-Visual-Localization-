#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <cv2/opencv.hpp>
#include <vector> 
#include <spdlog/spdlog.h>
#include <string.h>

namespace uavloc {
    cv::Mat NormalizeImage(const cv::Mat& image) {
        if (image.empty()) {
            spdlog::error("Input image is empty.");
            return cv::Mat();
        }
        
        cv::Mat normalized;
        image.convertTo(normalized, CV_32F, 1.0 / 255.0);
        return normalized;
    }

    cv::Mat ResizeImage(const cv::Mat& image, ) {
        if (image.empty()) {
            spdlog::error("Input image is empty.");
            return cv::Mat();
        }
        
        cv::Mat resized;
        cv::resize(image, resized, new_size);
        return resized;
    }

}

#endif 