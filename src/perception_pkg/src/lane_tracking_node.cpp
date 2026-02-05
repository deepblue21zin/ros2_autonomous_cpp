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
    this->declare_parameter("debug", false);
    this->declare_parameter("roi_y_ratio", 0.55);
    this->declare_parameter("canny_low", 50);
    this->declare_parameter("canny_high", 150);
    this->declare_parameter("gaussian_kernel", 5);
    this->declare_parameter("avg_window", 5);
    this->declare_parameter("hough_threshold", 50);
    this->declare_parameter("hough_min_length", 30);
    this->declare_parameter("hough_max_gap", 80);
    this->declare_parameter("slope_min", 0.3);
    this->declare_parameter("slope_max", 2.5);
    this->declare_parameter("lane_x_margin", 0.40);
    this->declare_parameter("lane_y_bottom", 0.50);

    camera_topic_ = this->get_parameter("camera_topic").as_string();
    use_compressed_ = this->get_parameter("use_compressed").as_bool();
    kp_ = this->get_parameter("kp").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    canny_low_ = this->get_parameter("canny_low").as_int();
    canny_high_ = this->get_parameter("canny_high").as_int();
    gaussian_kernel_ = this->get_parameter("gaussian_kernel").as_int();
    avg_window_ = this->get_parameter("avg_window").as_int();
    hough_threshold_ = this->get_parameter("hough_threshold").as_int();
    hough_min_length_ = this->get_parameter("hough_min_length").as_int();
    hough_max_gap_ = this->get_parameter("hough_max_gap").as_int();
    slope_min_ = this->get_parameter("slope_min").as_double();
    slope_max_ = this->get_parameter("slope_max").as_double();
    lane_x_margin_ = this->get_parameter("lane_x_margin").as_double();
    lane_y_bottom_ = this->get_parameter("lane_y_bottom").as_double();

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
    RCLCPP_INFO(this->get_logger(), "  kp: %.2f", kp_);
    RCLCPP_INFO(this->get_logger(), "  roi_y_ratio: %.2f", roi_y_ratio_);
}

void LaneTrackingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        handleFrame(cv_ptr->image, msg->header);
    } catch (const cv_bridge::Exception& e) {
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

    // Preprocess for lane detection
    cv::Mat edges = preprocessForLaneDetection(roi_color, gaussian_kernel_,
                                                canny_low_, canny_high_);

    // Detect lane center
    auto [lane_center, roi_overlay] = detectLaneCenter(edges, roi_color, roi_y);

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
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    hough_threshold_ = this->get_parameter("hough_threshold").as_int();
    hough_min_length_ = this->get_parameter("hough_min_length").as_int();
    hough_max_gap_ = this->get_parameter("hough_max_gap").as_int();
    slope_min_ = this->get_parameter("slope_min").as_double();
    slope_max_ = this->get_parameter("slope_max").as_double();
    lane_x_margin_ = this->get_parameter("lane_x_margin").as_double();
    lane_y_bottom_ = this->get_parameter("lane_y_bottom").as_double();
}

std::pair<double, cv::Mat> LaneTrackingNode::detectLaneCenter(
    const cv::Mat& edges, const cv::Mat& roi_color, int roi_y) {

    cv::Mat overlay = roi_color.clone();
    int h = edges.rows;
    int w = edges.cols;

    // 디버그: 영역 경계선 시각화 (주차선 필터링 영역)
    if (debug_) {
        int left_x_max = static_cast<int>(w * lane_x_margin_);
        int right_x_min = static_cast<int>(w * (1.0 - lane_x_margin_));
        int y_bottom_threshold = static_cast<int>(h * (1.0 - lane_y_bottom_));

        // 좌/우 영역 경계 (노란색 수직선)
        cv::line(overlay, cv::Point(left_x_max, 0), cv::Point(left_x_max, h),
                 cv::Scalar(0, 255, 255), 1);
        cv::line(overlay, cv::Point(right_x_min, 0), cv::Point(right_x_min, h),
                 cv::Scalar(0, 255, 255), 1);
        // 하단 경계 (노란색 수평선)
        cv::line(overlay, cv::Point(0, y_bottom_threshold), cv::Point(w, y_bottom_threshold),
                 cv::Scalar(0, 255, 255), 1);
    }

    // Probabilistic Hough Line Transform
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180,
                    hough_threshold_, hough_min_length_, hough_max_gap_);

    // 길이 기반 샘플링된 포인트 수집
    std::vector<double> left_x, left_y, right_x, right_y;

    // Get crosswalk bbox for filtering (if valid)
    float crosswalk_x1 = 0, crosswalk_x2 = 0;
    float crosswalk_y1 = 0, crosswalk_y2 = 0;
    float bbox_x1 = 0, bbox_x2 = 0, bbox_y1 = 0, bbox_y2 = 0;
    bool has_crosswalk = current_stop_line_.valid;

    // Debug: check if no crosswalk detected
    if (!has_crosswalk && debug_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[lane] No stop line detected (current_stop_line_.valid=false)");
    }

    if (has_crosswalk) {
        crosswalk_x1 = current_stop_line_.x1;
        crosswalk_y1 = current_stop_line_.y1 - roi_y;  // Convert to ROI coordinates
        crosswalk_x2 = current_stop_line_.x2;
        crosswalk_y2 = current_stop_line_.y2 - roi_y;

        // Debug: log bbox coordinates
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[lane] Stop line detected: full_img(%.0f,%.0f-%.0f,%.0f) roi_y=%d → roi(%.0f,%.0f-%.0f,%.0f)",
                             current_stop_line_.x1, current_stop_line_.y1,
                             current_stop_line_.x2, current_stop_line_.y2,
                             roi_y, crosswalk_x1, crosswalk_y1, crosswalk_x2, crosswalk_y2);

        // bbox를 약간 축소 (10% 마진)하여 경계 부근의 차선은 통과
        float margin_x = (crosswalk_x2 - crosswalk_x1) * 0.1f;
        float margin_y = (crosswalk_y2 - crosswalk_y1) * 0.1f;
        bbox_x1 = crosswalk_x1 + margin_x;
        bbox_x2 = crosswalk_x2 - margin_x;
        bbox_y1 = crosswalk_y1 + margin_y;
        bbox_y2 = crosswalk_y2 - margin_y;

        // Visualize crosswalk bbox
        if (debug_) {
            // 원본 bbox (빨간색)
            cv::rectangle(overlay,
                         cv::Point(static_cast<int>(crosswalk_x1), static_cast<int>(crosswalk_y1)),
                         cv::Point(static_cast<int>(crosswalk_x2), static_cast<int>(crosswalk_y2)),
                         cv::Scalar(0, 0, 255), 2);
            // 축소된 필터링 bbox (보라색)
            cv::rectangle(overlay,
                         cv::Point(static_cast<int>(bbox_x1), static_cast<int>(bbox_y1)),
                         cv::Point(static_cast<int>(bbox_x2), static_cast<int>(bbox_y2)),
                         cv::Scalar(255, 0, 255), 2);

            // Check if bbox is outside ROI
            if (crosswalk_y1 < 0 || crosswalk_y2 < 0 ||
                crosswalk_y1 > h || crosswalk_y2 > h) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "[lane] Crosswalk bbox partially outside ROI! roi_h=%d bbox_y=(%.0f,%.0f)",
                                     h, crosswalk_y1, crosswalk_y2);
            }
        }
    }

    // Debug: count filtered lines
    int total_lines = static_cast<int>(lines.size());
    int filtered_lines = 0;

    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

        if (x2 == x1) continue;

        double slope = static_cast<double>(y2 - y1) / static_cast<double>(x2 - x1);

        // 기울기 필터: 최소값 (거의 수평) 및 최대값 (거의 수직) 제거
        if (std::abs(slope) < slope_min_) continue;
        if (std::abs(slope) > slope_max_) continue;  // 주차선 같은 수직선 제거

        // Y 좌표 하단 체크: 차선은 이미지 하단 영역을 지나야 함
        int y_bottom_threshold = static_cast<int>(h * (1.0 - lane_y_bottom_));
        if (y1 < y_bottom_threshold && y2 < y_bottom_threshold) {
            continue;  // 두 끝점 모두 상단에만 있으면 차선 아님 (주차선/차량)
        }

        // Filter out lines inside crosswalk bbox (횡단보도 필터링)
        if (has_crosswalk) {
            // 선의 중심점 계산
            float line_center_x = (x1 + x2) / 2.0f;
            float line_center_y = (y1 + y2) / 2.0f;

            // 선의 중심점이 축소된 bbox 안에 있으면 횡단보도로 판단
            bool line_center_in_crosswalk =
                (line_center_x >= bbox_x1 && line_center_x <= bbox_x2 &&
                 line_center_y >= bbox_y1 && line_center_y <= bbox_y2);

            // 또는 양 끝점이 모두 bbox 안에 있는 경우 (완전히 횡단보도 내부)
            bool both_ends_in_crosswalk =
                (x1 >= bbox_x1 && x1 <= bbox_x2 && y1 >= bbox_y1 && y1 <= bbox_y2) &&
                (x2 >= bbox_x1 && x2 <= bbox_x2 && y2 >= bbox_y1 && y2 <= bbox_y2);

            if (line_center_in_crosswalk || both_ends_in_crosswalk) {
                filtered_lines++;
                continue;  // Skip crosswalk lines
            }
        }

        double avg_x = (x1 + x2) / 2.0;

        // 양 끝 10% 영역 필터링 (주차선/트랙 바깥 제거)
        int edge_margin = static_cast<int>(w * 0.10);
        if (avg_x < edge_margin || avg_x > w - edge_margin) {
            continue;  // 이미지 양 끝은 주차선이므로 무시
        }

        // X 좌표 영역 + 기울기 기반 좌/우 분류 (중앙 영역 무시)
        int left_x_max = static_cast<int>(w * lane_x_margin_);          // 왼쪽 35% 영역
        int right_x_min = static_cast<int>(w * (1.0 - lane_x_margin_)); // 오른쪽 35% 영역

        if (slope < 0 && avg_x < left_x_max) {
            // 왼쪽 차선: 음의 기울기 + 왼쪽 영역
            sampleLinePoints(x1, y1, x2, y2, left_x, left_y);
        } else if (slope > 0 && avg_x > right_x_min) {
            // 오른쪽 차선: 양의 기울기 + 오른쪽 영역
            sampleLinePoints(x1, y1, x2, y2, right_x, right_y);
        } else {
            // 중앙 영역이거나 기울기 방향과 위치가 안 맞으면 무시
            continue;
        }

        if (debug_) {
            cv::line(overlay, cv::Point(x1, y1), cv::Point(x2, y2),
                     cv::Scalar(0, 255, 0), 2);
        }
    }

    // Debug: log filtering statistics
    if (has_crosswalk && debug_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[lane] Crosswalk filtering: total_lines=%d filtered=%d remaining=%d",
                             total_lines, filtered_lines, total_lines - filtered_lines);
    }

    int y_bottom = h - 1;
    int y_top = static_cast<int>(h * 0.3); // ROI 내에서 라인 그리기 범위 작아질수록 증가(상단의 30%까지 봄)

    std::vector<int> lane_positions;

    // RANSAC + 2차 다항식으로 차선 피팅
    RansacFitResult left_result = fitPolynomial2DRansac(left_y, left_x, y_bottom, y_top);
    RansacFitResult right_result = fitPolynomial2DRansac(right_y, right_x, y_bottom, y_top);

    if (left_result.valid) {
        lane_positions.push_back(left_result.x_bottom);
        // 곡선으로 차선 표시 (polylines)
        cv::polylines(overlay, left_result.curve_points, false,
                      cv::Scalar(255, 0, 0), 3);
    }

    if (right_result.valid) {
        lane_positions.push_back(right_result.x_bottom);
        cv::polylines(overlay, right_result.curve_points, false,
                      cv::Scalar(0, 0, 255), 3);
    }

    // Calculate center with single-lane fallback
    double center_x;
    bool left_detected = left_result.valid;
    bool right_detected = right_result.valid;

    if (left_detected && right_detected) {
        double sum = std::accumulate(lane_positions.begin(), lane_positions.end(), 0.0);
        center_x = sum / lane_positions.size();
    } else if (right_detected && !left_detected) {
        int estimated_lane_width = static_cast<int>(w * 0.35);
        center_x = lane_positions[0] - estimated_lane_width / 2;
        center_x = std::max(0.0, std::min(center_x, static_cast<double>(w - 1)));
    } else if (left_detected && !right_detected) {
        int estimated_lane_width = static_cast<int>(w * 0.35);
        center_x = lane_positions[0] + estimated_lane_width / 2;
        center_x = std::max(0.0, std::min(center_x, static_cast<double>(w - 1)));
    } else {
        center_x = static_cast<double>(w) / 2.0;
    }

    cv::circle(overlay, cv::Point(static_cast<int>(center_x), y_bottom),
               6, cv::Scalar(0, 255, 255), -1);

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

    // Proportional control with sign inversion
    double steering = clamp(kp_ * (-smooth_offset), -1.0, 1.0);

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
