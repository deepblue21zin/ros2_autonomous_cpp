#ifndef DECISION_DECISION_NODE_HPP
#define DECISION_DECISION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
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
    double lane_timeout_;
    bool use_obstacle_avoidance_;
    double obstacle_bias_weight_;

    // State
    double lane_steer_norm_;
    std::optional<rclcpp::Time> lane_stamp_;
    bool obstacle_;
    double obstacle_bias_;
    double ultra_min_;
    std::string traffic_state_;

    // Callbacks
    void laneCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void obstacleBiasCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void ultraCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void trafficCallback(const std_msgs::msg::String::SharedPtr msg);
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
