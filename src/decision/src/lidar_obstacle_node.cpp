#include "decision/lidar_obstacle_node.hpp"
#include <optional>

namespace decision {

LidarObstacleNode::LidarObstacleNode()
    : Node("lidar_obstacle_node") {
    // Declare and load parameters
    this->declare_parameter("angle_min_deg", 350.0);
    this->declare_parameter("angle_max_deg", 10.0);
    this->declare_parameter("distance_m", 0.5);
    this->declare_parameter("scan_topic", "/scan");

    angle_min_deg_ = this->get_parameter("angle_min_deg").as_double();
    angle_max_deg_ = this->get_parameter("angle_max_deg").as_double();
    distance_m_ = this->get_parameter("distance_m").as_double();
    scan_topic_ = this->get_parameter("scan_topic").as_string();

    // Setup subscribers and publishers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&LidarObstacleNode::scanCallback, this, std::placeholders::_1));
    
    flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/perception/obstacle_flag", 10);
    
    dist_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/perception/obstacle_distance", 10);

    RCLCPP_INFO(this->get_logger(), "LidarObstacleNode initialized");
    RCLCPP_INFO(this->get_logger(), "  angle_min_deg: %.1f", angle_min_deg_);
    RCLCPP_INFO(this->get_logger(), "  angle_max_deg: %.1f", angle_max_deg_);
    RCLCPP_INFO(this->get_logger(), "  distance_m: %.2f", distance_m_);
    RCLCPP_INFO(this->get_logger(), "  scan_topic: %s", scan_topic_.c_str());
}

double LidarObstacleNode::normalizeAngle(double rad) {
    constexpr double TWO_PI = 2.0 * M_PI;
    while (rad < 0.0) {
        rad += TWO_PI;
    }
    while (rad >= TWO_PI) {
        rad -= TWO_PI;
    }
    return rad;
}

bool LidarObstacleNode::angleInWindow(double angle, double start, double end) {
    // Handle wraparound (e.g., 350 to 10 degrees)
    if (start <= end) {
        return angle >= start && angle <= end;
    }
    // Wraparound case
    return angle >= start || angle <= end;
}

void LidarObstacleNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Convert configuration to radians and normalize
    double start_rad = normalizeAngle(angle_min_deg_ * M_PI / 180.0);
    double end_rad = normalizeAngle(angle_max_deg_ * M_PI / 180.0);

    std::optional<double> min_dist;

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double distance = scan->ranges[i];

        // Skip invalid readings
        if (!std::isfinite(distance) || distance <= 0.0) {
            continue;
        }

        // Calculate and normalize angle
        double angle = normalizeAngle(scan->angle_min + i * scan->angle_increment);

        // Check if angle is within detection window
        if (!angleInWindow(angle, start_rad, end_rad)) {
            continue;
        }

        // Check if within detection distance
        if (distance <= distance_m_) {
            if (!min_dist.has_value() || distance < min_dist.value()) {
                min_dist = distance;
            }
        }
    }

    // Publish results
    std_msgs::msg::Bool flag_msg;
    flag_msg.data = min_dist.has_value();
    flag_pub_->publish(flag_msg);

    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = min_dist.value_or(0.0f);
    dist_pub_->publish(dist_msg);
}

}  // namespace decision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<decision::LidarObstacleNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lidar_obstacle_node"),
                     "Exception in lidar_obstacle_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
