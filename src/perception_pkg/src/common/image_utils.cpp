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
    // 흰색 차선 검출 - HSV 파라미터 가이드:
    // cv::Scalar(H, S, V):
    //   H (Hue/색상):        0-180 범위
    //   S (Saturation/채도): 0-255 (0=무채색, 255=순수 색상)
    //   V (Value/밝기):      0-255 (0=검정, 255=흰색)
    //
    // 조명이 어두울 때: S와 V 범위를 넓게 설정
    cv::inRange(hsv,
                cv::Scalar(0, 0, 120),    // lower: H=0,   S=0,  V=120 (어두운 회색도 포함)
                cv::Scalar(180, 60, 255), // upper: H=180, S=60, V=255 (약간 채도 있어도 허용)
                mask);
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

    // Threshold white only
    cv::Mat mask = thresholdWhiteTracking(hsv);

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

cv::Mat preprocessForSlidingWindow(const cv::Mat& roi_color, int gaussian_kernel) {
    // HSV 변환
    cv::Mat hsv;
    cv::cvtColor(roi_color, hsv, cv::COLOR_BGR2HSV);

    // V 채널 기반 동적 임계값 계산 (통창 햇빛 대응)
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    const cv::Mat& v_channel = channels[2];

    // 80th 퍼센타일: 히스토그램으로 빠르게 계산
    int v_hist[256] = {};
    for (int r = 0; r < v_channel.rows; r++) {
        const uchar* row = v_channel.ptr<uchar>(r);
        for (int c = 0; c < v_channel.cols; c++) {
            v_hist[row[c]]++;
        }
    }
    int total = v_channel.rows * v_channel.cols;
    int target = static_cast<int>(total * 0.80);
    int cumulative = 0;
    int v_p80 = 200;
    for (int i = 0; i < 256; i++) {
        cumulative += v_hist[i];
        if (cumulative >= target) {
            v_p80 = i;
            break;
        }
    }

    // V 하한 = 80th 퍼센타일 - 30, 범위 [80, 200]
    int v_lower = std::max(80, std::min(200, v_p80 - 30));

    cv::Mat mask;
    cv::inRange(hsv,
                cv::Scalar(0, 10, v_lower),
                cv::Scalar(180, 60, 255),
                mask);

    // 노이즈 제거: 가우시안 블러 → 모폴로지 닫기 (점선 gap 연결)
    cv::GaussianBlur(mask, mask, cv::Size(gaussian_kernel, gaussian_kernel), 0);
    mask = morphClose(mask, 5, 2);

    return mask;
}

cv::Mat preprocessForSlidingWindowGrayscale(const cv::Mat& roi_color, int gaussian_kernel, int threshold) {
    // Grayscale 변환
    cv::Mat gray;
    cv::cvtColor(roi_color, gray, cv::COLOR_BGR2GRAY);

    // Otsu 자동 임계값 (햇빛/그림자 대응)
    // threshold 파라미터는 Otsu 하한으로만 사용 (너무 낮은 임계값 방지)
    cv::Mat binary;
    double otsu_val = cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Otsu 결과가 너무 낮으면 고정 threshold 사용 (최소 보장)
    if (otsu_val < static_cast<double>(threshold)) {
        cv::threshold(gray, binary, threshold, 255, cv::THRESH_BINARY);
    }

    // 노이즈 제거: 가우시안 블러 → 모폴로지 닫기 (점선 gap 연결)
    cv::GaussianBlur(binary, binary, cv::Size(gaussian_kernel, gaussian_kernel), 0);
    cv::Mat mask = morphClose(binary, 5, 2);

    return mask;
}

cv::Mat morphClose(const cv::Mat& mask, int kernel_size, int iterations) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(kernel_size, kernel_size));
    cv::Mat result;
    cv::morphologyEx(mask, result, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), iterations);
    return result;
}

}  // namespace perception_pkg
