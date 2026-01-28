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
