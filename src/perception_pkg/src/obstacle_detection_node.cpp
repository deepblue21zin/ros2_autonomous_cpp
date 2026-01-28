#include "perception_pkg/obstacle_detection_node.hpp"
#include "perception_pkg/common/image_utils.hpp"
#include <opencv2/highgui.hpp>

namespace perception_pkg {

ObstacleDetectionNode::ObstacleDetectionNode()
    : Node("obstacle_detection_node") {
    // Declare and load parameters
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("use_compressed", false);
    this->declare_parameter("roi_y_ratio", 0.55);
    this->declare_parameter("band_ratio", 0.6);
    this->declare_parameter("obstacle_center_ratio", 0.2);
    this->declare_parameter("obstacle_square_ratio", 0.0);
    this->declare_parameter("min_area", 600.0);
    this->declare_parameter("min_height", 20.0);
    this->declare_parameter("saturation_thresh", 50);
    this->declare_parameter("value_thresh", 100);
    this->declare_parameter("blur_kernel", 5);
    this->declare_parameter("canny_low", 50);
    this->declare_parameter("canny_high", 150);
    this->declare_parameter("morph_kernel", 5);
    this->declare_parameter("bias_gain", 1.0);
    this->declare_parameter("min_bias_weight", 0.1);
    this->declare_parameter("publish_overlay", true);

    camera_topic_ = this->get_parameter("camera_topic").as_string();
    use_compressed_ = this->get_parameter("use_compressed").as_bool();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    band_ratio_ = this->get_parameter("band_ratio").as_double();
    center_ratio_ = this->get_parameter("obstacle_center_ratio").as_double();
    square_ratio_ = this->get_parameter("obstacle_square_ratio").as_double();
    min_area_ = this->get_parameter("min_area").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    saturation_thresh_ = this->get_parameter("saturation_thresh").as_int();
    value_thresh_ = this->get_parameter("value_thresh").as_int();
    blur_kernel_ = this->get_parameter("blur_kernel").as_int();
    canny_low_ = this->get_parameter("canny_low").as_int();
    canny_high_ = this->get_parameter("canny_high").as_int();
    morph_kernel_ = this->get_parameter("morph_kernel").as_int();
    bias_gain_ = this->get_parameter("bias_gain").as_double();
    min_bias_weight_ = this->get_parameter("min_bias_weight").as_double();
    publish_overlay_ = this->get_parameter("publish_overlay").as_bool();

    // QoS for low latency
    auto qos_sensor = rclcpp::QoS(1).best_effort();

    // Setup subscribers
    if (use_compressed_) {
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            camera_topic_ + "/compressed", qos_sensor,
            std::bind(&ObstacleDetectionNode::compressedCallback, this, std::placeholders::_1));
    } else {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, qos_sensor,
            std::bind(&ObstacleDetectionNode::imageCallback, this, std::placeholders::_1));
    }

    // Setup publishers
    detection_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/perception/obstacles_2d", 10);
    bias_pub_ = this->create_publisher<std_msgs::msg::Float32>("/perception/obstacle_bias", 10);
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/obstacle_overlay", 1);

    RCLCPP_INFO(this->get_logger(), "ObstacleDetectionNode initialized");
    RCLCPP_INFO(this->get_logger(), "  camera_topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  use_compressed: %s", use_compressed_ ? "true" : "false");
}

void ObstacleDetectionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        processFrame(cv_ptr->image, msg->header.stamp);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void ObstacleDetectionNode::compressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (!frame.empty()) {
            processFrame(frame, msg->header.stamp);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image decode error: %s", e.what());
    }
}

void ObstacleDetectionNode::processFrame(const cv::Mat& frame, const rclcpp::Time& stamp) {
    auto [detections, overlay] = detectObstacles(frame);

    // Publish detections
    publishDetections(detections);

    // Compute and publish bias
    double bias = computeBias(detections, frame.cols);
    std_msgs::msg::Float32 bias_msg;
    bias_msg.data = static_cast<float>(bias);
    bias_pub_->publish(bias_msg);

    // Publish overlay
    if (publish_overlay_ && !overlay.empty()) {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = "camera";
        sensor_msgs::msg::Image::SharedPtr overlay_msg = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
        overlay_pub_->publish(*overlay_msg);
    }
}

std::pair<std::vector<Detection>, cv::Mat> ObstacleDetectionNode::detectObstacles(const cv::Mat& frame) {
    std::vector<Detection> detections;
    cv::Mat overlay = frame.clone();

    int h = frame.rows;
    int w = frame.cols;

    // Calculate ROI
    int roi_y_start = static_cast<int>(h * roi_y_ratio_);
    int roi_height = static_cast<int>(h * band_ratio_);
    int roi_y_end = std::min(roi_y_start + roi_height, h);

    // Calculate horizontal ROI (center band)
    int center_x = w / 2;
    int half_width = static_cast<int>(w * center_ratio_ / 2);
    int roi_x_start = center_x - half_width;
    int roi_x_end = center_x + half_width;

    // Use square ROI if specified
    if (square_ratio_ > 0) {
        int side = static_cast<int>(w * square_ratio_);
        roi_x_start = center_x - side / 2;
        roi_x_end = center_x + side / 2;
        roi_y_start = h - side;
        roi_y_end = h;
    }

    // Clamp ROI bounds
    roi_x_start = std::max(0, roi_x_start);
    roi_x_end = std::min(w, roi_x_end);
    roi_y_start = std::max(0, roi_y_start);
    roi_y_end = std::min(h, roi_y_end);

    if (roi_x_end <= roi_x_start || roi_y_end <= roi_y_start) {
        return {detections, overlay};
    }

    // Extract ROI
    cv::Mat roi = frame(cv::Range(roi_y_start, roi_y_end),
                        cv::Range(roi_x_start, roi_x_end));

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // Create mask for dark colored objects (low saturation and value)
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 0),
                cv::Scalar(180, saturation_thresh_, value_thresh_), mask);

    // Apply Gaussian blur
    cv::Mat blurred;
    cv::GaussianBlur(mask, blurred, cv::Size(blur_kernel_, blur_kernel_), 0);

    // Canny edge detection
    cv::Mat edges;
    cv::Canny(blurred, edges, canny_low_, canny_high_);

    // Combine with mask
    cv::Mat combined;
    cv::bitwise_or(mask, edges, combined);

    // Morphological closing
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(morph_kernel_, morph_kernel_));
    cv::Mat closed;
    cv::morphologyEx(combined, closed, cv::MORPH_CLOSE, kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Process contours
    for (const auto& contour : contours) {
        cv::Rect bbox = cv::boundingRect(contour);
        double area = cv::contourArea(contour);

        // Filter by area and height
        if (area < min_area_ || bbox.height < min_height_) {
            continue;
        }

        // Convert to frame coordinates
        float x1 = static_cast<float>(bbox.x + roi_x_start);
        float y1 = static_cast<float>(bbox.y + roi_y_start);
        float x2 = static_cast<float>(bbox.x + bbox.width + roi_x_start);
        float y2 = static_cast<float>(bbox.y + bbox.height + roi_y_start);

        Detection det(x1, y1, x2, y2, static_cast<float>(area));
        detections.push_back(det);

        // Draw on overlay
        cv::rectangle(overlay, cv::Point(static_cast<int>(x1), static_cast<int>(y1)),
                      cv::Point(static_cast<int>(x2), static_cast<int>(y2)),
                      cv::Scalar(0, 0, 255), 2);
    }

    // Draw ROI on overlay
    cv::rectangle(overlay, cv::Point(roi_x_start, roi_y_start),
                  cv::Point(roi_x_end, roi_y_end), cv::Scalar(0, 255, 0), 1);

    return {detections, overlay};
}

double ObstacleDetectionNode::computeBias(const std::vector<Detection>& detections, int width) {
    if (detections.empty()) {
        return 0.0;
    }

    double total_weight = 0.0;
    double weighted_offset = 0.0;
    double center = static_cast<double>(width) / 2.0;

    for (const auto& det : detections) {
        double det_center = det.centerX();
        double offset = (det_center - center) / center;  // Normalized to [-1, 1]

        // Weight by area
        double weight = std::max(static_cast<double>(det.area()), min_bias_weight_);
        total_weight += weight;
        weighted_offset += offset * weight;
    }

    if (total_weight > 0) {
        return bias_gain_ * (weighted_offset / total_weight);
    }
    return 0.0;
}

void ObstacleDetectionNode::publishDetections(const std::vector<Detection>& detections) {
    std_msgs::msg::Float32MultiArray msg;

    // Layout: [x1, y1, x2, y2, score] per detection
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "detections";
    msg.layout.dim[0].size = detections.size();
    msg.layout.dim[0].stride = 5;
    msg.layout.dim[1].label = "fields";
    msg.layout.dim[1].size = 5;
    msg.layout.dim[1].stride = 1;

    for (const auto& det : detections) {
        msg.data.push_back(det.x1);
        msg.data.push_back(det.y1);
        msg.data.push_back(det.x2);
        msg.data.push_back(det.y2);
        msg.data.push_back(det.score);
    }

    detection_pub_->publish(msg);
}

}  // namespace perception_pkg

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<perception_pkg::ObstacleDetectionNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("obstacle_detection_node"),
                     "Exception in obstacle_detection_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
