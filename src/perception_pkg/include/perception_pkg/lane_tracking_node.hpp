#ifndef PERCEPTION_PKG_LANE_TRACKING_NODE_HPP
#define PERCEPTION_PKG_LANE_TRACKING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <deque>
#include "perception_pkg/common/detection_types.hpp"

namespace perception_pkg {

class LaneTrackingNode : public rclcpp::Node {
public:
    LaneTrackingNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stop_line_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr offset_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;

    // Parameters
    std::string camera_topic_;
    bool use_compressed_;
    double kp_;
    bool debug_;
    double roi_y_ratio_;
    int canny_low_;
    int canny_high_;
    int gaussian_kernel_;
    int avg_window_;
    int hough_threshold_;
    int hough_min_length_;
    int hough_max_gap_;
    double slope_min_;
    double slope_max_;
    double lane_x_margin_;
    double lane_y_bottom_;
    int min_lane_length_;

    // Moving average buffer
    std::deque<double> offset_buffer_;

    // Crosswalk filtering
    StopLine current_stop_line_;

    // Callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void stopLineCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // Processing
    void handleFrame(const cv::Mat& frame, const std_msgs::msg::Header& header);
    void updateParams();

    // Lane center detection using Hough transform
    std::pair<double, cv::Mat> detectLaneCenter(
        const cv::Mat& edges, const cv::Mat& roi_color, int roi_y);

    // Steering computation with moving average
    std::pair<double, double> computeSteering(double lane_center, int width);

    // Overlay composition
    cv::Mat composeOverlay(const cv::Mat& frame, const cv::Mat& roi_overlay,
                          int roi_y, double lane_center);
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_LANE_TRACKING_NODE_HPP
