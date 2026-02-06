#ifndef DECISION_DECISION_NODE_HPP
#define DECISION_DECISION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <optional>

namespace decision {

class DecisionNode : public rclcpp::Node {
public:
    DecisionNode();

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lane_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_bias_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ultra_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stop_line_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double cruise_speed_;
    double max_steer_rad_;
    double soft_steer_rad_;
    double soft_steer_threshold_;
    double ultra_safe_m_;
    bool use_traffic_light_;
    bool stop_on_yellow_;
    double stop_line_stop_distance_;  // 정지선 정지 거리
    double lane_timeout_;
    bool use_obstacle_avoidance_;
    double obstacle_bias_weight_;
    bool test_mode_;  // 테스트 모드: 센서 없이 모터 구동 가능

    // Speed control parameters
    double min_speed_;
    double curve_factor_;
    double accel_rate_;          // m/s² 가속 제한
    double decel_rate_;          // m/s² 감속 제한
    double obstacle_d_stop_;     // 장애물 즉시정지 거리 (m)
    double obstacle_d_slow_;     // 장애물 감속 시작 거리 (m)
    double obstacle_k_;          // 장애물 감속 계수
    double obstacle_max_speed_;  // 장애물 감속 시 최대 속도
    int lane_lost_hold_frames_;  // 차선 소실 hold 프레임
    double lost_lane_speed_;     // 차선 소실 시 감속 속도

    // State
    double lane_steer_norm_;
    std::optional<rclcpp::Time> lane_stamp_;
    bool obstacle_;
    double obstacle_bias_;
    double ultra_min_;
    std::string traffic_state_;
    double stop_line_distance_;  // 정지선까지 거리 (m), -1=없음

    // Speed control state
    double current_cmd_speed_;   // 현재 출력 속도 (램핑용)
    rclcpp::Time last_timer_time_;
    int lane_lost_count_;        // 차선 소실 연속 프레임 수

    // Callbacks
    void laneCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void obstacleBiasCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void ultraCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void trafficCallback(const std_msgs::msg::String::SharedPtr msg);
    void stopLineCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void timerCallback();

    // Steering mapping
    double mapSteer(double steer_norm) const;

    // Clamp utility
    template<typename T>
    static T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
};

}  // namespace decision

#endif  // DECISION_DECISION_NODE_HPP
