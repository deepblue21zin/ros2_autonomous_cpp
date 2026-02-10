#ifndef PERCEPTION_PKG_LANE_TRACKING_NODE_HPP
#define PERCEPTION_PKG_LANE_TRACKING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <deque>
#include <optional>
#include <Eigen/Dense>
#include "perception_pkg/common/detection_types.hpp"

namespace perception_pkg {

class LaneTrackingNode : public rclcpp::Node {
public:
    LaneTrackingNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stop_line_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr offset_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;

    // Parameters
    std::string camera_topic_;
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

    // Predict+Hold state
    double prev_center_x_;
    int lost_count_;
    bool has_prev_center_;
    int crosswalk_hold_count_;

    // Adaptive lane width tracking (EMA)
    double tracked_lane_width_;
    bool has_tracked_width_;
    int single_lane_hold_count_;
    int max_single_lane_hold_;  // 단일 차선 시 이전 center 유지 프레임 수

    // Moving average buffer
    std::deque<double> offset_buffer_;

    // Search-around-poly: 이전 프레임 polynomial 저장
    std::optional<Eigen::Vector3d> prev_left_coeffs_;
    std::optional<Eigen::Vector3d> prev_right_coeffs_;
    int search_around_margin_;
    int search_around_fallback_;  // search-around-poly 실패 시 full search로 전환할 연속 실패 횟수
    int left_poly_miss_count_;
    int right_poly_miss_count_;

    // 전처리 v_lower 캐시 (N프레임마다 갱신)
    int cached_v_lower_;
    int v_lower_update_counter_;

    // Overlay throttle (debug=true 시 10Hz 제한)
    int overlay_frame_count_;

    // Crosswalk filtering
    StopLine current_stop_line_;

    // Callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void stopLineCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // Processing
    void handleFrame(const cv::Mat& frame, const std_msgs::msg::Header& header);
    void updateParams();

    // Lane center detection (Sliding Window + RANSAC)
    // draw_overlay=false → overlay 생성/그리기 스킵 (fast path)
    std::pair<double, cv::Mat> detectLaneCenter(
        const cv::Mat& edges, const cv::Mat& roi_color, int roi_y,
        bool draw_overlay = true);

    // Steering computation with moving average
    std::pair<double, double> computeSteering(double lane_center, int width);

    // Overlay composition
    cv::Mat composeOverlay(const cv::Mat& frame, const cv::Mat& roi_overlay,
                          int roi_y, double lane_center);
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_LANE_TRACKING_NODE_HPP
