#include "ultrasonic_driver/ultrasonic_processor_node.hpp"
#include <algorithm>
#include <limits>

namespace ultrasonic_driver {

UltrasonicProcessorNode::UltrasonicProcessorNode()
    : Node("ultrasonic_processor_node") {
    // Declare and load parameters
    this->declare_parameter("safe_distance_m", 0.2);
    this->declare_parameter("ranges_topic", "/ultrasonic/ranges");

    safe_distance_ = this->get_parameter("safe_distance_m").as_double();
    ranges_topic_ = this->get_parameter("ranges_topic").as_string();

    // Setup subscribers and publishers
    ranges_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        ranges_topic_, 10,
        std::bind(&UltrasonicProcessorNode::rangesCallback, this, std::placeholders::_1));
    
    min_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/ultrasonic/min_range", 10);
    
    obs_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/ultrasonic/obstacle", 10);

    RCLCPP_INFO(this->get_logger(), "UltrasonicProcessorNode initialized");
    RCLCPP_INFO(this->get_logger(), "  safe_distance: %.2f m", safe_distance_);
    RCLCPP_INFO(this->get_logger(), "  ranges_topic: %s", ranges_topic_.c_str());
}

void UltrasonicProcessorNode::rangesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Check if data is empty
    if (msg->data.empty()) {
        return;
    }

    // Need at least 3 values for front sensors (F, FL, FR)
    if (msg->data.size() < 3) {
        return;
    }

    // Extract front sensors only (indices 0, 1, 2)
    std::vector<float> front_values;
    for (size_t i = 0; i < 3 && i < msg->data.size(); ++i) {
        float v = msg->data[i];
        if (v > 0.0f) {
            front_values.push_back(v);
        }
    }

    // Calculate minimum range
    float min_range = 0.0f;
    if (!front_values.empty()) {
        min_range = *std::min_element(front_values.begin(), front_values.end());
    }

    // Detect obstacle: valid reading AND below safe distance
    bool has_obstacle = (min_range > 0.0f) && (min_range < static_cast<float>(safe_distance_));

    // Publish results
    std_msgs::msg::Float32 min_msg;
    min_msg.data = min_range;
    min_pub_->publish(min_msg);

    std_msgs::msg::Bool obs_msg;
    obs_msg.data = has_obstacle;
    obs_pub_->publish(obs_msg);
}

}  // namespace ultrasonic_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ultrasonic_driver::UltrasonicProcessorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ultrasonic_processor_node"),
                     "Exception in ultrasonic_processor_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
