#ifndef PERCEPTION_PKG_OBSTACLE_DETECTION_NODE_HPP
#define PERCEPTION_PKG_OBSTACLE_DETECTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "perception_pkg/common/detection_types.hpp"

namespace perception_pkg {

class ObstacleDetectionNode : public rclcpp::Node {
public:
    ObstacleDetectionNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bias_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;

    // Parameters
    std::string camera_topic_;
    bool use_compressed_;
    double roi_y_ratio_;
    double band_ratio_;
    double center_ratio_;
    double square_ratio_;
    double min_area_;
    double min_height_;
    int saturation_thresh_;
    int value_thresh_;
    int blur_kernel_;
    int canny_low_;
    int canny_high_;
    int morph_kernel_;
    double bias_gain_;
    double min_bias_weight_;
    bool publish_overlay_;

    // Callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // Processing
    void processFrame(const cv::Mat& frame, const rclcpp::Time& stamp);
    std::pair<std::vector<Detection>, cv::Mat> detectObstacles(const cv::Mat& frame);

    // Bias computation
    double computeBias(const std::vector<Detection>& detections, int width);

    // Publishing
    void publishDetections(const std::vector<Detection>& detections);
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_OBSTACLE_DETECTION_NODE_HPP
