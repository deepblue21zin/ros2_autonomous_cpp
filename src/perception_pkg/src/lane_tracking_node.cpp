#include "perception_pkg/lane_tracking_node.hpp"
#include "perception_pkg/common/image_utils.hpp"
#include "perception_pkg/common/lane_geometry.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <numeric>

namespace perception_pkg {

LaneTrackingNode::LaneTrackingNode()
    : Node("lane_tracking_node") {
    // Declare and load parameters
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("kp", 0.6);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("debug", false);
    this->declare_parameter("roi_y_ratio", 0.55);
    this->declare_parameter("use_grayscale_threshold", false);
    this->declare_parameter("gaussian_kernel", 5);
    this->declare_parameter("avg_window", 5);
    this->declare_parameter("lane_width_ratio", 0.55);
    this->declare_parameter("max_steer_delta", 0.15);
    this->declare_parameter("lookahead_min_ratio", 0.15);
    this->declare_parameter("lookahead_max_ratio", 0.50);
    this->declare_parameter("lookahead_curvature_k", 50.0);
    this->declare_parameter("coeff_ema_alpha", 0.4);
    this->declare_parameter("max_lost_frames", 15);
    this->declare_parameter("max_single_lane_hold", 10);
    this->declare_parameter("crosswalk_density_threshold", 0.30);
    this->declare_parameter("crosswalk_density_max", 0.50);
    this->declare_parameter("max_crosswalk_frames", 60);
    this->declare_parameter("search_around_margin", 60);
    this->declare_parameter("search_around_fallback", 3);
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    use_grayscale_threshold_ = this->get_parameter("use_grayscale_threshold").as_bool();
    gaussian_kernel_ = this->get_parameter("gaussian_kernel").as_int();
    avg_window_ = this->get_parameter("avg_window").as_int();
    lane_width_ratio_ = this->get_parameter("lane_width_ratio").as_double();
    max_steer_delta_ = this->get_parameter("max_steer_delta").as_double();
    lookahead_min_ratio_ = this->get_parameter("lookahead_min_ratio").as_double();
    lookahead_max_ratio_ = this->get_parameter("lookahead_max_ratio").as_double();
    lookahead_curvature_k_ = this->get_parameter("lookahead_curvature_k").as_double();
    coeff_ema_alpha_ = this->get_parameter("coeff_ema_alpha").as_double();
    max_lost_frames_ = this->get_parameter("max_lost_frames").as_int();
    max_single_lane_hold_ = this->get_parameter("max_single_lane_hold").as_int();
    crosswalk_density_threshold_ = this->get_parameter("crosswalk_density_threshold").as_double();
    crosswalk_density_max_ = this->get_parameter("crosswalk_density_max").as_double();
    max_crosswalk_frames_ = this->get_parameter("max_crosswalk_frames").as_int();
    search_around_margin_ = this->get_parameter("search_around_margin").as_int();
    search_around_fallback_ = this->get_parameter("search_around_fallback").as_int();
    prev_steering_ = 0.0;
    prev_smooth_offset_ = 0.0;
    prev_center_x_ = 0.0;
    lost_count_ = 0;
    has_prev_center_ = false;
    crosswalk_hold_count_ = 0;
    tracked_lane_width_ = 0.0;
    has_tracked_width_ = false;
    single_lane_hold_count_ = 0;
    left_poly_miss_count_ = 0;
    right_poly_miss_count_ = 0;
    cached_v_lower_ = 180;  // 초기값: 기본 하한
    v_lower_update_counter_ = 0;
    overlay_frame_count_ = 0;

    // QoS for low latency: best_effort to skip old frames
    auto qos_sensor = rclcpp::QoS(1).best_effort();

    // Setup subscribers (raw image only)
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, qos_sensor,
        std::bind(&LaneTrackingNode::imageCallback, this, std::placeholders::_1));

    // Subscribe to stop line for crosswalk filtering
    stop_line_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/perception/stop_line", 10,
        std::bind(&LaneTrackingNode::stopLineCallback, this, std::placeholders::_1));

    // Setup publishers
    offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lane/center_offset", 10);
    steer_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lane/steering_angle", 10);
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_overlay", qos_sensor);

    RCLCPP_INFO(this->get_logger(), "LaneTrackingNode initialized");
    RCLCPP_INFO(this->get_logger(), "  camera_topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  kp: %.2f, kd: %.2f", kp_, kd_);
    RCLCPP_INFO(this->get_logger(), "  roi_y_ratio: %.2f", roi_y_ratio_);
}

void LaneTrackingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv::Mat bgr_frame;
        const std::string& enc = msg->encoding;

        if (enc == "yuv422_yuy2" || enc == "yuyv") {
            // YUV422 YUYV → BGR (cv_bridge가 지원하지 않는 포맷)
            cv::Mat yuv(msg->height, msg->width, CV_8UC2,
                        const_cast<uint8_t*>(msg->data.data()), msg->step);
            cv::cvtColor(yuv, bgr_frame, cv::COLOR_YUV2BGR_YUY2);
        } else if (enc == "uyvy") {
            cv::Mat yuv(msg->height, msg->width, CV_8UC2,
                        const_cast<uint8_t*>(msg->data.data()), msg->step);
            cv::cvtColor(yuv, bgr_frame, cv::COLOR_YUV2BGR_UYVY);
        } else {
            // bgr8, rgb8, mono8 등 - cv_bridge가 처리
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            bgr_frame = cv_ptr->image;
        }

        if (!bgr_frame.empty()) {
            handleFrame(bgr_frame, msg->header);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void LaneTrackingNode::stopLineCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Parse stop line message: [x1, y1, x2, y2, distance_m]
    if (msg->data.size() >= 5) {
        current_stop_line_ = StopLine(
            msg->data[0], msg->data[1],
            msg->data[2], msg->data[3],
            msg->data[4]
        );
    } else {
        // No valid stop line
        current_stop_line_ = StopLine();
    }
}

void LaneTrackingNode::handleFrame(const cv::Mat& frame, const std_msgs::msg::Header& header) {
    // Update parameters dynamically
    updateParams();

    // Extract ROI
    auto [roi_color, roi_y] = extractROI(frame, roi_y_ratio_);

    // Preprocess: 전처리 방식 선택 (Sliding Window용)
    cv::Mat binary_mask;
    if (use_grayscale_threshold_) {
        // Grayscale Binary Threshold 방식
        binary_mask = preprocessForSlidingWindowGrayscale(roi_color, gaussian_kernel_, 120);
    } else {
        // HSV Threshold 방식 (기본값) - v_lower 5프레임마다 갱신
        if (v_lower_update_counter_ % 5 == 0) {
            cached_v_lower_ = computeAdaptiveVLower(roi_color);
        }
        v_lower_update_counter_++;
        binary_mask = preprocessForSlidingWindow(roi_color, gaussian_kernel_, cached_v_lower_);
    }

    // Detect lane center (Sliding Window + RANSAC)
    // debug=false 시 overlay 생성 스킵, 10Hz throttle
    bool should_draw_overlay = debug_ && (overlay_frame_count_ % 1 == 0);
    auto [lane_center, roi_overlay] = detectLaneCenter(binary_mask, roi_color, roi_y, should_draw_overlay);
    overlay_frame_count_++;

    // Compute steering
    auto [steering, offset_norm] = computeSteering(lane_center, frame.cols);

    // Publish results
    std_msgs::msg::Float32 offset_msg;
    offset_msg.data = static_cast<float>(offset_norm);
    offset_pub_->publish(offset_msg);

    std_msgs::msg::Float32 steer_msg;
    steer_msg.data = static_cast<float>(steering);
    steer_pub_->publish(steer_msg);

    // Publish overlay (debug=true && throttled)
    if (should_draw_overlay) {
        cv::Mat overlay = composeOverlay(frame, roi_overlay, roi_y, lane_center);
        sensor_msgs::msg::Image::SharedPtr overlay_msg =
            cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
        overlay_pub_->publish(*overlay_msg);
    }
}

void LaneTrackingNode::updateParams() {
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    search_around_margin_ = static_cast<int>(std::max(1L, this->get_parameter("search_around_margin").as_int()));
    search_around_fallback_ = static_cast<int>(std::max(1L, this->get_parameter("search_around_fallback").as_int()));
    lookahead_min_ratio_ = std::clamp(this->get_parameter("lookahead_min_ratio").as_double(), 0.05, 0.5);
    lookahead_max_ratio_ = std::clamp(this->get_parameter("lookahead_max_ratio").as_double(), 0.1, 1.0);
    lookahead_min_ratio_ = std::min(lookahead_min_ratio_, lookahead_max_ratio_);  // min ≤ max 보장
    lookahead_curvature_k_ = std::max(0.0, this->get_parameter("lookahead_curvature_k").as_double());
    coeff_ema_alpha_ = std::clamp(this->get_parameter("coeff_ema_alpha").as_double(), 0.0, 1.0);
}

std::pair<double, cv::Mat> LaneTrackingNode::detectLaneCenter(
    const cv::Mat& binary_mask, const cv::Mat& roi_color, int roi_y,
    bool draw_overlay) {

    cv::Mat overlay;
    if (draw_overlay) {
        overlay = roi_color.clone();
    }
    int h = binary_mask.rows;
    int w = binary_mask.cols;

    // 횡단보도 영역 마스킹 (해당 영역 픽셀을 0으로 채워서 무시)
    cv::Mat mask = binary_mask.clone();
    if (current_stop_line_.valid) {
        float cx1 = current_stop_line_.x1;
        float cy1 = current_stop_line_.y1 - roi_y;
        float cx2 = current_stop_line_.x2;
        float cy2 = current_stop_line_.y2 - roi_y;
        float mx = (cx2 - cx1) * 0.1f;
        float my = (cy2 - cy1) * 0.1f;

        cv::rectangle(mask,
                      cv::Point(static_cast<int>(cx1 + mx), static_cast<int>(cy1 + my)),
                      cv::Point(static_cast<int>(cx2 - mx), static_cast<int>(cy2 - my)),
                      cv::Scalar(0), -1);

        if (draw_overlay) {
            cv::rectangle(overlay,
                         cv::Point(static_cast<int>(cx1), static_cast<int>(cy1)),
                         cv::Point(static_cast<int>(cx2), static_cast<int>(cy2)),
                         cv::Scalar(0, 0, 255), 2);
        }
    }

    // findNonZero 1회 호출 — 이후 모든 검색 + 밀도 계산에서 재사용
    std::vector<cv::Point> nonzero;
    cv::findNonZero(mask, nonzero);
    double white_density = static_cast<double>(nonzero.size()) / (h * w);

    // 디버그: 밀도 값 항상 표시 (임계값 튜닝용)
    if (draw_overlay) {
        char density_buf[64];
        snprintf(density_buf, sizeof(density_buf), "density: %.1f%%", white_density * 100.0);
        cv::putText(overlay, density_buf,
                    cv::Point(10, h - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(200, 200, 200), 1);
    }

    if (white_density > crosswalk_density_threshold_ && white_density < crosswalk_density_max_) {
        // 횡단보도 감지 (밴드: threshold < density < max) → 이전 center 유지
        double center_x;
        if (has_prev_center_ && crosswalk_hold_count_ < max_crosswalk_frames_) {
            center_x = prev_center_x_;
            crosswalk_hold_count_++;
        } else {
            center_x = static_cast<double>(w) / 2.0;
        }

        if (draw_overlay) {
            cv::putText(overlay,
                        "CROSSWALK HOLD (" + std::to_string(crosswalk_hold_count_) + ")",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 0, 255), 2);
            cv::circle(overlay, cv::Point(static_cast<int>(center_x), h / 2),
                       8, cv::Scalar(255, 0, 255), -1);
        }
        return {center_x, overlay};
    }

    // 횡단보도 종료 → hold 카운터 리셋, 정상 sliding window 재획득
    crosswalk_hold_count_ = 0;

    // 1. Search-around-poly 우선, 실패 시 히스토그램 기반 full search
    SlidingWindowResult left_sw, right_sw;
    bool left_used_search_around = false;
    bool right_used_search_around = false;

    // 좌측 차선: 이전 polynomial이 있고 연속 실패가 적으면 search-around-poly
    if (prev_left_coeffs_.has_value() && left_poly_miss_count_ < search_around_fallback_) {
        left_sw = searchAroundPoly(nonzero, prev_left_coeffs_.value(), search_around_margin_);
        left_used_search_around = true;
    }
    // 우측 차선
    if (prev_right_coeffs_.has_value() && right_poly_miss_count_ < search_around_fallback_) {
        right_sw = searchAroundPoly(nonzero, prev_right_coeffs_.value(), search_around_margin_);
        right_used_search_around = true;
    }

    // Fallback: search-around-poly 실패 시 히스토그램 기반 full search
    // 히스토그램 1회만 계산 후 캐시 (좌/우 모두 사용 가능)
    std::vector<int> histogram;
    if (!left_sw.valid || !right_sw.valid) {
        histogram = computeHistogramFromPoints(nonzero, w, h / 2);
        int midpoint = w / 2;
        if (!left_sw.valid) {
            int left_base = findHistogramPeak(histogram, 0, midpoint);
            left_sw = slidingWindowSearch(h, w, nonzero, left_base, 9, 50, 50);
            left_used_search_around = false;
        }
        if (!right_sw.valid) {
            int right_base = findHistogramPeak(histogram, midpoint, w);
            right_sw = slidingWindowSearch(h, w, nonzero, right_base, 9, 50, 50);
            right_used_search_around = false;
        }
    }

    // 3. RANSAC + 2차 다항식 피팅
    int y_bottom = h - 1;
    int y_top = static_cast<int>(h * 0.3);

    // 동적 lookahead: 곡률이 크면 가까이(반응성), 작으면 멀리(안정성)
    double curvature = 0.0;
    if (prev_left_coeffs_.has_value())
        curvature = std::max(curvature, std::abs(2.0 * prev_left_coeffs_.value()(0)));
    if (prev_right_coeffs_.has_value())
        curvature = std::max(curvature, std::abs(2.0 * prev_right_coeffs_.value()(0)));

    double effective_ratio = lookahead_max_ratio_ / (1.0 + lookahead_curvature_k_ * curvature);
    effective_ratio = std::clamp(effective_ratio, lookahead_min_ratio_, lookahead_max_ratio_);

    int y_lookahead = y_bottom - static_cast<int>((y_bottom - y_top) * effective_ratio);

    bool left_detected = false;
    bool right_detected = false;
    Eigen::Vector3d left_coeffs, right_coeffs;

    if (left_sw.valid) {
        std::vector<double> ly(left_sw.lane_y.begin(), left_sw.lane_y.end());
        std::vector<double> lx(left_sw.lane_x.begin(), left_sw.lane_x.end());
        RansacFitResult left_fit = fitPolynomial2DRansac(ly, lx, y_bottom, y_top);
        if (left_fit.valid) {
            left_detected = true;
            left_coeffs = left_fit.coeffs;
            if (draw_overlay) {
                cv::polylines(overlay, left_fit.curve_points, false,
                              cv::Scalar(255, 0, 0), 3);
            }
        }
    }

    if (right_sw.valid) {
        std::vector<double> ry(right_sw.lane_y.begin(), right_sw.lane_y.end());
        std::vector<double> rx(right_sw.lane_x.begin(), right_sw.lane_x.end());
        RansacFitResult right_fit = fitPolynomial2DRansac(ry, rx, y_bottom, y_top);
        if (right_fit.valid) {
            right_detected = true;
            right_coeffs = right_fit.coeffs;
            if (draw_overlay) {
                cv::polylines(overlay, right_fit.curve_points, false,
                              cv::Scalar(0, 0, 255), 3);
            }
        }
    }

    // RANSAC 실패 + search-around-poly 출처일 때만 histogram full search 재시도
    bool left_need_retry = !left_detected && left_used_search_around;
    bool right_need_retry = !right_detected && right_used_search_around;

    if (left_need_retry || right_need_retry) {
        // 히스토그램 재사용: 이미 계산되어 있으면 재활용
        if (histogram.empty()) {
            histogram = computeHistogramFromPoints(nonzero, w, h / 2);
        }
        int midpoint = w / 2;

        if (left_need_retry) {
            int left_base = findHistogramPeak(histogram, 0, midpoint);
            SlidingWindowResult left_retry = slidingWindowSearch(h, w, nonzero, left_base, 9, 50, 50);
            if (left_retry.valid) {
                std::vector<double> ly(left_retry.lane_y.begin(), left_retry.lane_y.end());
                std::vector<double> lx(left_retry.lane_x.begin(), left_retry.lane_x.end());
                RansacFitResult left_fit = fitPolynomial2DRansac(ly, lx, y_bottom, y_top);
                if (left_fit.valid) {
                    left_detected = true;
                    left_coeffs = left_fit.coeffs;
                    if (draw_overlay) {
                        cv::polylines(overlay, left_fit.curve_points, false,
                                      cv::Scalar(255, 0, 0), 3);
                    }
                }
            }
        }
        if (right_need_retry) {
            int right_base = findHistogramPeak(histogram, midpoint, w);
            SlidingWindowResult right_retry = slidingWindowSearch(h, w, nonzero, right_base, 9, 50, 50);
            if (right_retry.valid) {
                std::vector<double> ry(right_retry.lane_y.begin(), right_retry.lane_y.end());
                std::vector<double> rx(right_retry.lane_x.begin(), right_retry.lane_x.end());
                RansacFitResult right_fit = fitPolynomial2DRansac(ry, rx, y_bottom, y_top);
                if (right_fit.valid) {
                    right_detected = true;
                    right_coeffs = right_fit.coeffs;
                    if (draw_overlay) {
                        cv::polylines(overlay, right_fit.curve_points, false,
                                      cv::Scalar(0, 0, 255), 3);
                    }
                }
            }
        }
    }

    // 교차 게이트: 양쪽 polynomial이 교차/근접하면 신뢰도 낮은 쪽 폐기
    if (left_detected && right_detected) {
        double lx_bot  = evaluatePolynomial2D(left_coeffs, y_bottom);
        double rx_bot  = evaluatePolynomial2D(right_coeffs, y_bottom);
        double lx_look = evaluatePolynomial2D(left_coeffs, y_lookahead);
        double rx_look = evaluatePolynomial2D(right_coeffs, y_lookahead);
        double lx_top  = evaluatePolynomial2D(left_coeffs, y_top);
        double rx_top  = evaluatePolynomial2D(right_coeffs, y_top);

        double min_gap = w * 0.10;  // 최소 간격 (이미지 폭의 10%)
        if ((rx_bot - lx_bot) < min_gap ||
            (rx_look - lx_look) < min_gap ||
            (rx_top - lx_top) < min_gap) {
            // 이전 프레임 추적 이력이 있는 쪽 유지, 없는 쪽 폐기
            bool left_has_prev = prev_left_coeffs_.has_value();
            bool right_has_prev = prev_right_coeffs_.has_value();

            if (left_has_prev && !right_has_prev) {
                right_detected = false;
            } else if (right_has_prev && !left_has_prev) {
                left_detected = false;
            } else {
                // 둘 다 추적 중이거나 둘 다 새로운 경우: 양쪽 폐기 → HOLD
                left_detected = false;
                right_detected = false;
            }
        }
    }

    // search-around-poly 상태 갱신: 검출 성공 시 polynomial 저장, 실패 시 miss count 증가
    if (left_detected) {
        // EMA 평활: search-around-poly 기준점 안정화 (다음 프레임 탐색 영역 평활화)
        if (prev_left_coeffs_.has_value()) {
            prev_left_coeffs_ = coeff_ema_alpha_ * left_coeffs
                              + (1.0 - coeff_ema_alpha_) * prev_left_coeffs_.value();
        } else {
            prev_left_coeffs_ = left_coeffs;
        }
        left_poly_miss_count_ = 0;
    } else {
        left_poly_miss_count_++;
        if (left_poly_miss_count_ >= search_around_fallback_) {
            prev_left_coeffs_.reset();  // 연속 실패 → polynomial 초기화
        }
    }
    if (right_detected) {
        if (prev_right_coeffs_.has_value()) {
            prev_right_coeffs_ = coeff_ema_alpha_ * right_coeffs
                               + (1.0 - coeff_ema_alpha_) * prev_right_coeffs_.value();
        } else {
            prev_right_coeffs_ = right_coeffs;
        }
        right_poly_miss_count_ = 0;
    } else {
        right_poly_miss_count_++;
        if (right_poly_miss_count_ >= search_around_fallback_) {
            prev_right_coeffs_.reset();
        }
    }

    // 4. Pure Pursuit: lookahead point 기반 차선 중심 계산
    double center_x;
    bool lane_found = false;
    int fallback_lane_width = static_cast<int>(w * lane_width_ratio_);

    if (left_detected && right_detected) {
        double left_x = evaluatePolynomial2D(left_coeffs, y_lookahead);
        double right_x = evaluatePolynomial2D(right_coeffs, y_lookahead);
        center_x = (left_x + right_x) / 2.0;
        lane_found = true;
        single_lane_hold_count_ = 0;

        // EMA로 실측 차선 폭 추적 (양쪽 보일 때만 갱신)
        double measured_width = std::abs(right_x - left_x);
        if (measured_width > w * 0.2 && measured_width < w * 0.9) {
            if (has_tracked_width_) {
                tracked_lane_width_ = 0.8 * tracked_lane_width_ + 0.2 * measured_width;
            } else {
                tracked_lane_width_ = measured_width;
                has_tracked_width_ = true;
            }
        }
    } else if (right_detected || left_detected) {
        single_lane_hold_count_++;
        // 단일 차선 감지: 이전 center가 있고 hold 제한 이내이면 Hold
        if (has_prev_center_ && single_lane_hold_count_ <= max_single_lane_hold_) {
            center_x = prev_center_x_;
            lane_found = true;
        } else {
            // hold 초과 또는 이전 center 없음: 가상 센터 재계산
            double est_width = has_tracked_width_
                ? tracked_lane_width_
                : static_cast<double>(fallback_lane_width);

            if (right_detected) {
                double right_x = evaluatePolynomial2D(right_coeffs, y_lookahead);
                center_x = right_x - est_width / 2.0;
            } else {
                double left_x = evaluatePolynomial2D(left_coeffs, y_lookahead);
                center_x = left_x + est_width / 2.0;
            }
            lane_found = true;
        }
    }

    // 5. Predict+Hold: 양쪽 차선 모두 소실 시 이전 center_x 유지
    if (lane_found) {
        center_x = clamp(center_x, 0.0, static_cast<double>(w - 1));
        prev_center_x_ = center_x;
        lost_count_ = 0;
        has_prev_center_ = true;
    } else if (has_prev_center_ && lost_count_ < max_lost_frames_) {
        center_x = prev_center_x_;
        lost_count_++;
    } else {
        center_x = static_cast<double>(w) / 2.0;
    }

    // 시각화: lookahead point (노란색), 소실 중이면 빨간 테두리
    if (draw_overlay) {
        cv::circle(overlay, cv::Point(static_cast<int>(center_x), y_lookahead),
                   6, cv::Scalar(0, 255, 255), -1);
        if (!lane_found) {
            cv::circle(overlay, cv::Point(static_cast<int>(center_x), y_lookahead),
                       10, cv::Scalar(0, 0, 255), 2);
        }
    }

    return {center_x, overlay};
}

std::pair<double, double> LaneTrackingNode::computeSteering(double lane_center, int width) {
    double img_center = static_cast<double>(width) / 2.0;
    double offset = lane_center - img_center;

    // Normalize to [-1, 1]
    double offset_norm = clamp(offset / img_center, -1.0, 1.0);

    // Moving average filter
    offset_buffer_.push_back(offset_norm);
    if (static_cast<int>(offset_buffer_.size()) > avg_window_) {
        offset_buffer_.pop_front();
    }

    double sum = std::accumulate(offset_buffer_.begin(), offset_buffer_.end(), 0.0);
    double smooth_offset = sum / offset_buffer_.size();

    // PD control with sign inversion
    double d_offset = smooth_offset - prev_smooth_offset_;
    prev_smooth_offset_ = smooth_offset;
    double steering = clamp(kp_ * (-smooth_offset) + kd_ * (-d_offset), -1.0, 1.0);

    // Rate limit: 프레임 간 급격한 조향 변화 방지
    double delta = steering - prev_steering_;
    if (delta > max_steer_delta_) {
        steering = prev_steering_ + max_steer_delta_;
    } else if (delta < -max_steer_delta_) {
        steering = prev_steering_ - max_steer_delta_;
    }
    prev_steering_ = steering;

    return {steering, smooth_offset};
}

cv::Mat LaneTrackingNode::composeOverlay(const cv::Mat& frame, const cv::Mat& roi_overlay,
                                          int roi_y, double lane_center) {
    cv::Mat overlay = frame.clone();

    // Blend ROI overlay
    cv::Mat roi_region = overlay(cv::Range(roi_y, overlay.rows), cv::Range::all());
    cv::addWeighted(roi_overlay, 0.7, roi_region, 0.3, 0, roi_region);

    // Draw image center line (yellow)
    int img_center = frame.cols / 2;
    cv::line(overlay, cv::Point(img_center, roi_y), cv::Point(img_center, frame.rows),
             cv::Scalar(0, 255, 255), 2);

    // Draw lane center line (green)
    int lane_x = static_cast<int>(lane_center);
    cv::line(overlay, cv::Point(lane_x, roi_y), cv::Point(lane_x, frame.rows),
             cv::Scalar(0, 255, 0), 2);

    return overlay;
}

}  // namespace perception_pkg

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<perception_pkg::LaneTrackingNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lane_tracking_node"),
                     "Exception in lane_tracking_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
