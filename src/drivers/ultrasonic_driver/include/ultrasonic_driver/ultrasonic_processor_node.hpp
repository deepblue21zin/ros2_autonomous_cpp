#ifndef ULTRASONIC_DRIVER_ULTRASONIC_PROCESSOR_NODE_HPP
#define ULTRASONIC_DRIVER_ULTRASONIC_PROCESSOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace ultrasonic_driver {

class UltrasonicProcessorNode : public rclcpp::Node {
public:
    UltrasonicProcessorNode();

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ranges_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obs_pub_;

    // Parameters
    double safe_distance_;
    std::string ranges_topic_;

    // Callback
    void rangesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};

}  // namespace ultrasonic_driver

#endif  // ULTRASONIC_DRIVER_ULTRASONIC_PROCESSOR_NODE_HPP
