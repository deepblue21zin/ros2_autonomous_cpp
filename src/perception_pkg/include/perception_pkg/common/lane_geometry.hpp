#ifndef PERCEPTION_PKG_LANE_GEOMETRY_HPP
#define PERCEPTION_PKG_LANE_GEOMETRY_HPP

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <vector>

namespace perception_pkg {

// Polynomial fitting: x = a*y^2 + b*y + c (quadratic)
// Returns coefficients [a, b, c]
Eigen::Vector3d fitPolynomial2D(const std::vector<double>& y_points,
                                const std::vector<double>& x_points);

// Linear polynomial fitting: x = a*y + b
// Returns coefficients [a, b]
Eigen::Vector2d fitPolynomial1D(const std::vector<double>& y_points,
                                const std::vector<double>& x_points);

// Evaluate quadratic polynomial at y
double evaluatePolynomial2D(const Eigen::Vector3d& coeffs, double y);

// Evaluate linear polynomial at y
double evaluatePolynomial1D(const Eigen::Vector2d& coeffs, double y);

// RANSAC + 2차 다항식 피팅 결과
struct RansacFitResult {
    Eigen::Vector3d coeffs;                // [a, b, c] for x = a*y^2 + b*y + c
    bool valid;
    std::vector<cv::Point> curve_points;   // 곡선 렌더링용 포인트
    int x_bottom;                          // ROI 하단 x 좌표
};

// RANSAC + 2차 다항식으로 차선 피팅
RansacFitResult fitPolynomial2DRansac(
    const std::vector<double>& y_points,
    const std::vector<double>& x_points,
    int y_bottom, int y_top,
    int ransac_iters = 60,
    double inlier_thresh = 12.0);

// Hough 세그먼트를 따라 일정 간격으로 포인트 샘플링
void sampleLinePoints(int x1, int y1, int x2, int y2,
                      std::vector<double>& out_x, std::vector<double>& out_y,
                      double step = 5.0);

// Compute histogram for sliding window
std::vector<int> computeHistogram(const cv::Mat& binary_mask, int start_row = -1);

// Find peak in histogram
int findHistogramPeak(const std::vector<int>& histogram, int start_col = 0, int end_col = -1);

// Sliding window lane detection
struct SlidingWindowResult {
    std::vector<int> lane_x;
    std::vector<int> lane_y;
    bool valid;
};

SlidingWindowResult slidingWindowSearch(const cv::Mat& binary_mask,
                                        int start_x,
                                        int nwindows,
                                        int window_margin,
                                        int minpix);

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_LANE_GEOMETRY_HPP
