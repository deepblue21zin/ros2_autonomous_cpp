#include "perception_pkg/common/lane_geometry.hpp"
#include <numeric>
#include <algorithm>
#include <random>
#include <cmath>

namespace perception_pkg {

Eigen::Vector3d fitPolynomial2D(const std::vector<double>& y_points,
                                const std::vector<double>& x_points) {
    const int n = static_cast<int>(y_points.size());
    if (n < 3) {
        return Eigen::Vector3d::Zero();
    }

    // Build Vandermonde matrix for quadratic fit
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        double y = y_points[i];
        A(i, 0) = y * y;  // y^2
        A(i, 1) = y;      // y
        A(i, 2) = 1.0;    // constant
        b(i) = x_points[i];
    }

    // Solve least squares: A * coeffs = b
    Eigen::Vector3d coeffs = A.colPivHouseholderQr().solve(b);
    return coeffs;  // [a, b, c] where x = a*y^2 + b*y + c
}

Eigen::Vector2d fitPolynomial1D(const std::vector<double>& y_points,
                                const std::vector<double>& x_points) {
    const int n = static_cast<int>(y_points.size());
    if (n < 2) {
        return Eigen::Vector2d::Zero();
    }

    // Build matrix for linear fit
    Eigen::MatrixXd A(n, 2);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        A(i, 0) = y_points[i];  // y
        A(i, 1) = 1.0;          // constant
        b(i) = x_points[i];
    }

    // Solve least squares
    Eigen::Vector2d coeffs = A.colPivHouseholderQr().solve(b);
    return coeffs;  // [a, b] where x = a*y + b
}

double evaluatePolynomial2D(const Eigen::Vector3d& coeffs, double y) {
    return coeffs(0) * y * y + coeffs(1) * y + coeffs(2);
}

double evaluatePolynomial1D(const Eigen::Vector2d& coeffs, double y) {
    return coeffs(0) * y + coeffs(1);
}

void sampleLinePoints(int x1, int y1, int x2, int y2,
                      std::vector<double>& out_x, std::vector<double>& out_y,
                      double step) {
    double length = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    int n_points = std::max(static_cast<int>(length / step), 2);

    for (int i = 0; i < n_points; ++i) {
        double t = static_cast<double>(i) / (n_points - 1);
        out_x.push_back(x1 + t * (x2 - x1));
        out_y.push_back(y1 + t * (y2 - y1));
    }
}

RansacFitResult fitPolynomial2DRansac(
    const std::vector<double>& y_points,
    const std::vector<double>& x_points,
    int y_bottom, int y_top,
    int ransac_iters,
    double inlier_thresh) {

    RansacFitResult result;
    result.valid = false;
    result.x_bottom = 0;

    const int n = static_cast<int>(y_points.size());
    const int min_samples = 3;  // degree + 1
    if (n < min_samples * 2) return result;

    Eigen::Vector3d best_coef = Eigen::Vector3d::Zero();
    int best_inlier_count = 0;

    std::mt19937 rng(42);

    for (int iter = 0; iter < ransac_iters; ++iter) {
        // 랜덤 3개 포인트 선택
        std::vector<int> indices(n);
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), rng);

        std::vector<double> sample_y(min_samples), sample_x(min_samples);
        for (int i = 0; i < min_samples; ++i) {
            sample_y[i] = y_points[indices[i]];
            sample_x[i] = x_points[indices[i]];
        }

        Eigen::Vector3d coef = fitPolynomial2D(sample_y, sample_x);

        // 인라이어 카운트
        int inlier_count = 0;
        for (int i = 0; i < n; ++i) {
            double x_pred = evaluatePolynomial2D(coef, y_points[i]);
            if (std::abs(x_points[i] - x_pred) < inlier_thresh) {
                inlier_count++;
            }
        }

        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_coef = coef;
        }
    }

    // inlier가 전체의 30% 미만이면 노이즈 피팅으로 판단 → reject
    int min_inliers = std::max(min_samples, n * 3 / 10);
    if (best_inlier_count < min_inliers) return result;

    // 인라이어만으로 재피팅
    std::vector<double> inlier_y, inlier_x;
    for (int i = 0; i < n; ++i) {
        double x_pred = evaluatePolynomial2D(best_coef, y_points[i]);
        if (std::abs(x_points[i] - x_pred) < inlier_thresh) {
            inlier_y.push_back(y_points[i]);
            inlier_x.push_back(x_points[i]);
        }
    }

    if (static_cast<int>(inlier_y.size()) < min_inliers) return result;

    // 수직 커버리지 체크: inlier가 ROI 높이의 40% 이상을 커버해야 유효
    double y_min_inlier = *std::min_element(inlier_y.begin(), inlier_y.end());
    double y_max_inlier = *std::max_element(inlier_y.begin(), inlier_y.end());
    double roi_height = static_cast<double>(y_bottom - y_top);
    if (roi_height > 0 && (y_max_inlier - y_min_inlier) < roi_height * 0.4) return result;

    Eigen::Vector3d final_coef = fitPolynomial2D(inlier_y, inlier_x);

    // 곡선 포인트 생성
    const int n_pts = 30;
    for (int i = 0; i < n_pts; ++i) {
        double y = y_top + static_cast<double>(y_bottom - y_top) * i / (n_pts - 1);
        double x = evaluatePolynomial2D(final_coef, y);
        result.curve_points.push_back(cv::Point(static_cast<int>(x), static_cast<int>(y)));
    }

    result.coeffs = final_coef;
    result.x_bottom = static_cast<int>(evaluatePolynomial2D(final_coef, y_bottom));
    result.valid = true;

    return result;
}

std::vector<int> computeHistogram(const cv::Mat& binary_mask, int start_row) {
    int h = binary_mask.rows;
    int w = binary_mask.cols;

    if (start_row < 0) {
        start_row = h / 2;
    }

    std::vector<int> histogram(w, 0);

    for (int x = 0; x < w; ++x) {
        for (int y = start_row; y < h; ++y) {
            if (binary_mask.at<uchar>(y, x) > 0) {
                histogram[x]++;
            }
        }
    }

    return histogram;
}

std::vector<int> computeHistogramFromPoints(const std::vector<cv::Point>& nonzero,
                                             int width, int start_row) {
    std::vector<int> histogram(width, 0);
    for (const auto& pt : nonzero) {
        if (pt.y >= start_row && pt.x >= 0 && pt.x < width) {
            histogram[pt.x]++;
        }
    }
    return histogram;
}

int findHistogramPeak(const std::vector<int>& histogram, int start_col, int end_col) {
    if (histogram.empty()) return 0;

    if (end_col < 0) {
        end_col = static_cast<int>(histogram.size());
    }

    int max_val = 0;
    int max_idx = start_col;

    for (int i = start_col; i < end_col && i < static_cast<int>(histogram.size()); ++i) {
        if (histogram[i] > max_val) {
            max_val = histogram[i];
            max_idx = i;
        }
    }

    return max_idx;
}

SlidingWindowResult slidingWindowSearch(int h, int w,
                                        const std::vector<cv::Point>& nonzero,
                                        int start_x,
                                        int nwindows,
                                        int window_margin,
                                        int minpix) {
    SlidingWindowResult result;

    if (nonzero.empty()) return result;

    // y-row 인덱스 구축: row_start[y] = nonzero에서 pt.y >= y인 첫 인덱스
    // cv::findNonZero 출력은 래스터 순서(y 오름차순)이므로 정렬 불필요
    const int n = static_cast<int>(nonzero.size());
    std::vector<int> row_start(h + 1, n);
    for (int i = n - 1; i >= 0; --i) {
        int y = nonzero[i].y;
        if (y >= 0 && y < h) {
            row_start[y] = i;
        }
    }
    // 역방향 전파: row_start[y] = min(row_start[y], row_start[y+1])
    for (int r = h - 1; r >= 0; --r) {
        row_start[r] = std::min(row_start[r], row_start[r + 1]);
    }

    int window_height = h / std::max(nwindows, 1);
    int x_current = start_x;

    for (int window = 0; window < nwindows; ++window) {
        int win_y_low = std::max(0, h - (window + 1) * window_height);
        int win_y_high = std::min(h, h - window * window_height);
        int win_x_low = std::max(0, x_current - window_margin);
        int win_x_high = std::min(w, x_current + window_margin);

        std::vector<int> good_x;

        // 해당 y-범위 버킷만 순회 (전체 nonzero 대신)
        int start_idx = row_start[win_y_low];
        int end_idx = row_start[win_y_high];
        for (int i = start_idx; i < end_idx; ++i) {
            const auto& pt = nonzero[i];
            if (pt.x >= win_x_low && pt.x < win_x_high) {
                result.lane_x.push_back(pt.x);
                result.lane_y.push_back(pt.y);
                good_x.push_back(pt.x);
            }
        }

        if (static_cast<int>(good_x.size()) > minpix) {
            double sum = std::accumulate(good_x.begin(), good_x.end(), 0.0);
            x_current = static_cast<int>(sum / good_x.size());
        }
    }

    result.valid = result.lane_x.size() >= 50;
    return result;
}

SlidingWindowResult searchAroundPoly(const std::vector<cv::Point>& nonzero,
                                     const Eigen::Vector3d& prev_coeffs,
                                     int margin) {
    SlidingWindowResult result;

    if (nonzero.empty()) return result;

    for (const auto& pt : nonzero) {
        double x_pred = evaluatePolynomial2D(prev_coeffs, static_cast<double>(pt.y));
        if (std::abs(pt.x - x_pred) < margin) {
            result.lane_x.push_back(pt.x);
            result.lane_y.push_back(pt.y);
        }
    }

    result.valid = result.lane_x.size() >= 50;
    return result;
}

}  // namespace perception_pkg
