#include "perception_pkg/lane_tracking_node.hpp"
#include "perception_pkg/common/image_utils.hpp"
#include "perception_pkg/common/lane_geometry.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <numeric>

namespace perception_pkg {

LaneTrackingNode::LaneTrackingNode()
    : Node("lane_tracking_node") {
    // Declare and load parameters
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("use_compressed", false);
    this->declare_parameter("kp", 0.6);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("debug", false);
    this->declare_parameter("roi_y_ratio", 0.55);
    this->declare_parameter("use_grayscale_threshold", false);
    this->declare_parameter("gaussian_kernel", 5);
    this->declare_parameter("avg_window", 5);
    this->declare_parameter("lane_width_ratio", 0.55);
    this->declare_parameter("max_steer_delta", 0.15);
    this->declare_parameter("lookahead_ratio", 0.4);
    this->declare_parameter("max_lost_frames", 15);
    this->declare_parameter("crosswalk_density_threshold", 0.15);
    this->declare_parameter("max_crosswalk_frames", 60);

    camera_topic_ = this->get_parameter("camera_topic").as_string();
    use_compressed_ = this->get_parameter("use_compressed").as_bool();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    use_grayscale_threshold_ = this->get_parameter("use_grayscale_threshold").as_bool();
    gaussian_kernel_ = this->get_parameter("gaussian_kernel").as_int();
    avg_window_ = this->get_parameter("avg_window").as_int();
    lane_width_ratio_ = this->get_parameter("lane_width_ratio").as_double();
    max_steer_delta_ = this->get_parameter("max_steer_delta").as_double();
    lookahead_ratio_ = this->get_parameter("lookahead_ratio").as_double();
    max_lost_frames_ = this->get_parameter("max_lost_frames").as_int();
    crosswalk_density_threshold_ = this->get_parameter("crosswalk_density_threshold").as_double();
    max_crosswalk_frames_ = this->get_parameter("max_crosswalk_frames").as_int();
    prev_steering_ = 0.0;
    prev_smooth_offset_ = 0.0;
    prev_center_x_ = 0.0;
    lost_count_ = 0;
    has_prev_center_ = false;
    crosswalk_hold_count_ = 0;

    // QoS for low latency: best_effort to skip old frames
    auto qos_sensor = rclcpp::QoS(1).best_effort();

    // Setup subscribers
    if (use_compressed_) {
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            camera_topic_ + "/compressed", qos_sensor,
            std::bind(&LaneTrackingNode::compressedCallback, this, std::placeholders::_1));
    } else {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, qos_sensor,
            std::bind(&LaneTrackingNode::imageCallback, this, std::placeholders::_1));
    }

    // Subscribe to stop line for crosswalk filtering
    stop_line_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/perception/stop_line", 10,
        std::bind(&LaneTrackingNode::stopLineCallback, this, std::placeholders::_1));

    // Setup publishers
    offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lane/center_offset", 10);
    steer_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lane/steering_angle", 10);
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_overlay", 1);

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

void LaneTrackingNode::compressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (!frame.empty()) {
            std_msgs::msg::Header header;
            header.stamp = msg->header.stamp;
            header.frame_id = msg->header.frame_id;
            handleFrame(frame, header);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image decode error: %s", e.what());
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
        // HSV Threshold 방식 (기본값)
        binary_mask = preprocessForSlidingWindow(roi_color, gaussian_kernel_);
    }

    // Detect lane center (Sliding Window + RANSAC)
    auto [lane_center, roi_overlay] = detectLaneCenter(binary_mask, roi_color, roi_y);

    // Compute steering
    auto [steering, offset_norm] = computeSteering(lane_center, frame.cols);

    // Publish results
    std_msgs::msg::Float32 offset_msg;
    offset_msg.data = static_cast<float>(offset_norm);
    offset_pub_->publish(offset_msg);

    std_msgs::msg::Float32 steer_msg;
    steer_msg.data = static_cast<float>(steering);
    steer_pub_->publish(steer_msg);

    // Publish overlay
    if (debug_) {
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
}

std::pair<double, cv::Mat> LaneTrackingNode::detectLaneCenter(
    const cv::Mat& binary_mask, const cv::Mat& roi_color, int roi_y) {

    cv::Mat overlay = roi_color.clone();
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

        if (debug_) {
            cv::rectangle(overlay,
                         cv::Point(static_cast<int>(cx1), static_cast<int>(cy1)),
                         cv::Point(static_cast<int>(cx2), static_cast<int>(cy2)),
                         cv::Scalar(0, 0, 255), 2);
        }
    }

    // 횡단보도 Predict+Hold: 흰 픽셀 밀도로 횡단보도 감지
    double white_density = static_cast<double>(cv::countNonZero(mask)) / (h * w);
    if (white_density > crosswalk_density_threshold_) {
        // 횡단보도 감지 → lane 업데이트 중단, 이전 center 유지
        double center_x;
        if (has_prev_center_ && crosswalk_hold_count_ < max_crosswalk_frames_) {
            center_x = prev_center_x_;
            crosswalk_hold_count_++;
        } else {
            center_x = static_cast<double>(w) / 2.0;
        }

        if (debug_) {
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

    // 1. 히스토그램: 하단 50%에서 좌/우 차선 시작점 찾기
    std::vector<int> histogram = computeHistogram(mask, h / 2);
    int midpoint = w / 2;
    int left_base = findHistogramPeak(histogram, 0, midpoint);
    int right_base = findHistogramPeak(histogram, midpoint, w);

    // 2. Sliding Window 탐색
    SlidingWindowResult left_sw = slidingWindowSearch(mask, left_base, 9, 50, 50);
    SlidingWindowResult right_sw = slidingWindowSearch(mask, right_base, 9, 50, 50);

    // 3. RANSAC + 2차 다항식 피팅
    int y_bottom = h - 1;
    int y_top = static_cast<int>(h * 0.3);
    // Pure Pursuit: lookahead 지점 (ROI 높이의 lookahead_ratio_ 만큼 위)
    int y_lookahead = y_bottom - static_cast<int>((y_bottom - y_top) * lookahead_ratio_);

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
            cv::polylines(overlay, left_fit.curve_points, false,
                          cv::Scalar(255, 0, 0), 3);
        }
    }

    if (right_sw.valid) {
        std::vector<double> ry(right_sw.lane_y.begin(), right_sw.lane_y.end());
        std::vector<double> rx(right_sw.lane_x.begin(), right_sw.lane_x.end());
        RansacFitResult right_fit = fitPolynomial2DRansac(ry, rx, y_bottom, y_top);
        if (right_fit.valid) {
            right_detected = true;
            right_coeffs = right_fit.coeffs;
            cv::polylines(overlay, right_fit.curve_points, false,
                          cv::Scalar(0, 0, 255), 3);
        }
    }

    // 4. Pure Pursuit: lookahead point 기반 차선 중심 계산
    double center_x;
    bool lane_found = false;
    int estimated_lane_width = static_cast<int>(w * lane_width_ratio_);

    if (left_detected && right_detected) {
        double left_x = evaluatePolynomial2D(left_coeffs, y_lookahead);
        double right_x = evaluatePolynomial2D(right_coeffs, y_lookahead);
        center_x = (left_x + right_x) / 2.0;
        lane_found = true;
    } else if (right_detected) {
        double right_x = evaluatePolynomial2D(right_coeffs, y_lookahead);
        center_x = right_x - estimated_lane_width / 2.0;
        lane_found = true;
    } else if (left_detected) {
        double left_x = evaluatePolynomial2D(left_coeffs, y_lookahead);
        center_x = left_x + estimated_lane_width / 2.0;
        lane_found = true;
    }

    // 5. Predict+Hold: 차선 소실 시 이전 center_x 유지
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
    cv::circle(overlay, cv::Point(static_cast<int>(center_x), y_lookahead),
               6, cv::Scalar(0, 255, 255), -1);
    if (!lane_found) {
        cv::circle(overlay, cv::Point(static_cast<int>(center_x), y_lookahead),
                   10, cv::Scalar(0, 0, 255), 2);
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
