#ifndef PERCEPTION_PKG_IMAGE_UTILS_HPP
#define PERCEPTION_PKG_IMAGE_UTILS_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>

namespace perception_pkg {

// HSV thresholding for white lane markings
// Python: lower=[0, 0, 200], upper=[180, 70, 255]
cv::Mat thresholdWhite(const cv::Mat& hsv);

// HSV thresholding for white lane markings (lane_tracking version)
// Python: lower=[0, 0, 200], upper=[180, 40, 255]
cv::Mat thresholdWhiteTracking(const cv::Mat& hsv);

// HSV thresholding for yellow center line
// Python: lower=[15, 80, 80], upper=[40, 255, 255]
cv::Mat thresholdYellow(const cv::Mat& hsv);

// HSV thresholding for yellow center line (lane_tracking version)
// Python: lower=[15, 80, 80], upper=[35, 255, 255]
cv::Mat thresholdYellowTracking(const cv::Mat& hsv);

// Extract ROI from frame starting at roi_y_ratio from top
// Returns: (roi_color, roi_y_start)
std::pair<cv::Mat, int> extractROI(const cv::Mat& frame, float roi_y_ratio);

// Apply CLAHE histogram equalization
cv::Mat applyCLAHE(const cv::Mat& gray, float clip_limit = 2.0,
                   cv::Size tile_size = cv::Size(8, 8));

// Preprocess for lane detection (CLAHE + blur + Canny)
cv::Mat preprocessForLaneDetection(const cv::Mat& roi_color,
                                   int gaussian_kernel,
                                   int canny_low, int canny_high);

// Apply morphological closing
cv::Mat morphClose(const cv::Mat& mask, int kernel_size = 3, int iterations = 2);

// Clamp value to range
template<typename T>
T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_IMAGE_UTILS_HPP
