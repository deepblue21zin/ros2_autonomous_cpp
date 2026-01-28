#ifndef DECISION_LIDAR_OBSTACLE_NODE_HPP
#define DECISION_LIDAR_OBSTACLE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

namespace decision {

class LidarObstacleNode : public rclcpp::Node {
public:
    LidarObstacleNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;

    // Parameters
    double angle_min_deg_;
    double angle_max_deg_;
    double distance_m_;
    std::string scan_topic_;

    // Callback
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Utility functions
    static double normalizeAngle(double rad);
    static bool angleInWindow(double angle, double start, double end);
};

}  // namespace decision

#endif  // DECISION_LIDAR_OBSTACLE_NODE_HPP
