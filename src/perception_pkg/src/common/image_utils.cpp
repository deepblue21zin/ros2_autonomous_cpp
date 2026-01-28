#include "perception_pkg/common/image_utils.hpp"

namespace perception_pkg {

cv::Mat thresholdWhite(const cv::Mat& hsv) {
    cv::Mat mask;
    // lane_marking_node: lower=[0, 0, 200], upper=[180, 70, 255]
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 70, 255), mask);
    return mask;
}

cv::Mat thresholdWhiteTracking(const cv::Mat& hsv) {
    cv::Mat mask;
    // lane_tracking_node: lower=[0, 0, 200], upper=[180, 40, 255]
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 40, 255), mask);
    return mask;
}

cv::Mat thresholdYellow(const cv::Mat& hsv) {
    cv::Mat mask;
    // lane_marking_node: lower=[15, 80, 80], upper=[40, 255, 255]
    cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(40, 255, 255), mask);
    return mask;
}

cv::Mat thresholdYellowTracking(const cv::Mat& hsv) {
    cv::Mat mask;
    // lane_tracking_node: lower=[15, 80, 80], upper=[35, 255, 255]
    cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(35, 255, 255), mask);
    return mask;
}

std::pair<cv::Mat, int> extractROI(const cv::Mat& frame, float roi_y_ratio) {
    int h = frame.rows;
    int y_start = static_cast<int>(h * roi_y_ratio);
    cv::Mat roi = frame(cv::Range(y_start, h), cv::Range::all()).clone();
    return {roi, y_start};
}

cv::Mat applyCLAHE(const cv::Mat& gray, float clip_limit, cv::Size tile_size) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clip_limit, tile_size);
    cv::Mat result;
    clahe->apply(gray, result);
    return result;
}

cv::Mat preprocessForLaneDetection(const cv::Mat& roi_color,
                                   int gaussian_kernel,
                                   int canny_low, int canny_high) {
    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(roi_color, hsv, cv::COLOR_BGR2HSV);

    // Threshold white and yellow
    cv::Mat mask_white = thresholdWhiteTracking(hsv);
    cv::Mat mask_yellow = thresholdYellowTracking(hsv);

    // Combine masks
    cv::Mat mask;
    cv::bitwise_or(mask_white, mask_yellow, mask);

    // Apply mask to image
    cv::Mat masked;
    cv::bitwise_and(roi_color, roi_color, masked, mask);

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);

    // Apply CLAHE
    gray = applyCLAHE(gray, 2.0, cv::Size(8, 8));

    // Gaussian blur
    cv::Mat blur;
    cv::GaussianBlur(gray, blur, cv::Size(gaussian_kernel, gaussian_kernel), 0);

    // Canny edge detection
    cv::Mat edges;
    cv::Canny(blur, edges, canny_low, canny_high);

    return edges;
}

cv::Mat morphClose(const cv::Mat& mask, int kernel_size, int iterations) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(kernel_size, kernel_size));
    cv::Mat result;
    cv::morphologyEx(mask, result, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), iterations);
    return result;
}

}  // namespace perception_pkg
