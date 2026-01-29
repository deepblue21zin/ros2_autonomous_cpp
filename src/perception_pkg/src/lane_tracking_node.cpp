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

    camera_topic_ = this->get_parameter("camera_topic").as_string();
    use_compressed_ = this->get_parameter("use_compressed").as_bool();
    kp_ = this->get_parameter("kp").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    canny_low_ = this->get_parameter("canny_low").as_int();
    canny_high_ = this->get_parameter("canny_high").as_int();
    gaussian_kernel_ = this->get_parameter("gaussian_kernel").as_int();
    avg_window_ = this->get_parameter("avg_window").as_int();

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

void LaneTrackingNode::handleFrame(const cv::Mat& frame, const std_msgs::msg::Header& header) {
    // Update parameters dynamically
    updateParams();

    // Extract ROI
    auto [roi_color, roi_y] = extractROI(frame, roi_y_ratio_);

    // Preprocess for lane detection
    cv::Mat edges = preprocessForLaneDetection(roi_color, gaussian_kernel_,
                                                canny_low_, canny_high_);

    // Detect lane center
    auto [lane_center, roi_overlay] = detectLaneCenter(edges, roi_color);

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
        sensor_msgs::msg::Image::SharedPtr overlay_msg = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
        overlay_pub_->publish(*overlay_msg);
    }
}

void LaneTrackingNode::updateParams() {
    kp_ = this->get_parameter("kp").as_double();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
}

std::pair<double, cv::Mat> LaneTrackingNode::detectLaneCenter(
    const cv::Mat& edges, const cv::Mat& roi_color) {

    cv::Mat overlay = roi_color.clone();
    int h = edges.rows;
    int w = edges.cols;

    // Probabilistic Hough Line Transform
    // 파라미터 튜닝: 곡선/점선 차선 인식 개선
    // - threshold: 50→30 (곡선에서 직선 투표수 부족 문제 해결)
    // - minLineLength: 40→20 (짧은 점선 세그먼트 감지)
    // - maxLineGap: 50→100 (점선 간격 연결 개선)
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 100);

    std::vector<std::array<float, 4>> left_points;
    std::vector<std::array<float, 4>> right_points;

    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

        if (x2 == x1) continue;

        double slope = static_cast<double>(y2 - y1) / static_cast<double>(x2 - x1);

        // Reject nearly horizontal lines
        // 0.3→0.15로 완화: 급커브에서 더 많은 선 감지
        if (std::abs(slope) < 0.15) continue;

        if (slope < 0) {
            left_points.push_back({static_cast<float>(x1), static_cast<float>(y1),
                                   static_cast<float>(x2), static_cast<float>(y2)});
        } else {
            right_points.push_back({static_cast<float>(x1), static_cast<float>(y1),
                                    static_cast<float>(x2), static_cast<float>(y2)});
        }
    }

    int y_bottom = h - 1;
    int y_top = static_cast<int>(h * 0.6);

    std::vector<int> lane_positions;

    // Fit left lane
    if (!left_points.empty()) {
        std::vector<double> all_x, all_y;
        for (const auto& pts : left_points) {
            all_x.push_back(pts[0]);
            all_y.push_back(pts[1]);
            all_x.push_back(pts[2]);
            all_y.push_back(pts[3]);
        }
        Eigen::Vector2d coef = fitPolynomial1D(all_y, all_x);
        int lx_bottom = static_cast<int>(evaluatePolynomial1D(coef, y_bottom));
        int lx_top = static_cast<int>(evaluatePolynomial1D(coef, y_top));
        lane_positions.push_back(lx_bottom);

        // Draw left lane (blue)
        cv::line(overlay, cv::Point(lx_bottom, y_bottom), cv::Point(lx_top, y_top),
                 cv::Scalar(255, 0, 0), 3);
    }

    // Fit right lane
    if (!right_points.empty()) {
        std::vector<double> all_x, all_y;
        for (const auto& pts : right_points) {
            all_x.push_back(pts[0]);
            all_y.push_back(pts[1]);
            all_x.push_back(pts[2]);
            all_y.push_back(pts[3]);
        }
        Eigen::Vector2d coef = fitPolynomial1D(all_y, all_x);
        int rx_bottom = static_cast<int>(evaluatePolynomial1D(coef, y_bottom));
        int rx_top = static_cast<int>(evaluatePolynomial1D(coef, y_top));
        lane_positions.push_back(rx_bottom);

        // Draw right lane (red)
        cv::line(overlay, cv::Point(rx_bottom, y_bottom), cv::Point(rx_top, y_top),
                 cv::Scalar(0, 0, 255), 3);
    }

    // Calculate center
    double center_x;
    if (!lane_positions.empty()) {
        double sum = std::accumulate(lane_positions.begin(), lane_positions.end(), 0.0);
        center_x = sum / lane_positions.size();
    } else {
        center_x = static_cast<double>(w) / 2.0;
    }

    // Draw center point (green)
    cv::circle(overlay, cv::Point(static_cast<int>(center_x), y_bottom - 20),
               10, cv::Scalar(0, 255, 0), -1);

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
