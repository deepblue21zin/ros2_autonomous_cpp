#ifndef PERCEPTION_PKG_LANE_MARKING_NODE_HPP
#define PERCEPTION_PKG_LANE_MARKING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "perception_pkg/common/detection_types.hpp"

namespace perception_pkg {

class LaneMarkingNode : public rclcpp::Node {
public:
    LaneMarkingNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_light_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lane_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr stop_pub_;

    // Parameters
    std::string camera_topic_;
    bool use_compressed_;
    double roi_y_ratio_;
    int canny_low_;
    int canny_high_;
    double stop_roi_ratio_;
    int nwindows_;
    int window_margin_;
    int minpix_;
    double stop_line_pixels_per_meter_;
    double stop_line_min_aspect_;
    double stop_line_min_width_;
    bool use_traffic_light_gate_;
    std::vector<std::string> allowed_stop_states_;
    std::string current_light_state_;

    // Callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void trafficLightCallback(const std_msgs::msg::String::SharedPtr msg);

    // Core processing
    void handleFrame(const cv::Mat& frame);

    // Lane detection (sliding window + polynomial fitting)
    std::vector<LineSegment> detectLaneSegments(
        const cv::Mat& white_mask, const cv::Mat& yellow_mask, int roi_y);

    std::vector<LineSegment> fitLaneFromMask(
        const cv::Mat& mask, int roi_y,
        int expected_lanes, const std::vector<LineType>& type_ids);

    // Style estimation
    LineStyle estimateStyle(const cv::Mat& mask,
                           const std::vector<int>& xs,
                           const std::vector<int>& ys);

    // Stop line detection
    StopLine detectStopLine(const cv::Mat& frame, int roi_y);

    // Traffic light gating
    bool trafficLightAllowsStop() const;

    // Publishing
    void publishLaneSegments(const std::vector<LineSegment>& segments);
    void publishStopLine(const StopLine& stop_line);
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_LANE_MARKING_NODE_HPP
