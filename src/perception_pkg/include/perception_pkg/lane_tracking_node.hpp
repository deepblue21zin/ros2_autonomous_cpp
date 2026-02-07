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
    double kd_;
    bool debug_;
    double roi_y_ratio_;
    bool use_grayscale_threshold_;
    int gaussian_kernel_;
    int avg_window_;

    double lane_width_ratio_;
    double max_steer_delta_;
    double prev_steering_;
    double prev_smooth_offset_;
    double lookahead_ratio_;
    int max_lost_frames_;

    // Crosswalk Predict+Hold parameters
    double crosswalk_density_threshold_;
    double crosswalk_density_max_;
    int max_crosswalk_frames_;

    // Vehicle body masking parameters
    bool enable_vehicle_mask_;
    double vehicle_mask_bottom_left_x_;
    double vehicle_mask_bottom_right_x_;
    double vehicle_mask_top_left_x_;
    double vehicle_mask_top_right_x_;
    double vehicle_mask_top_y_;
    int vehicle_mask_num_side_points_;  // 측면 곡선 점 개수 (클수록 부드러움)
    int vehicle_mask_num_top_points_;   // 상단 원호 점 개수 (클수록 부드러움)
    double vehicle_mask_curve_depth_;   // 상단 곡선 깊이 (0.0=직선, 0.3=깊은 곡선)

    // Predict+Hold state
    double prev_center_x_;
    int lost_count_;
    bool has_prev_center_;
    int crosswalk_hold_count_;

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

    // Vehicle body masking (차량 본체 영역 제거)
    cv::Mat createVehicleMask(const cv::Mat& frame);

    // Overlay composition
    cv::Mat composeOverlay(const cv::Mat& frame, const cv::Mat& roi_overlay,
                          int roi_y, double lane_center);
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_LANE_TRACKING_NODE_HPP
