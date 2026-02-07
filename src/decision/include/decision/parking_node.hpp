#ifndef DECISION_PARKING_NODE_HPP
#define DECISION_PARKING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>

namespace decision {

enum class ParkingState {
    IDLE,
    SEARCH,
    ALIGN,
    REVERSE_ENTER,
    STRAIGHTEN,
    HOLD,
    EXIT_FORWARD,
    SEEK_OUT,
    DONE
};

class ParkingNode : public rclcpp::Node {
public:
    ParkingNode();

private:
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr line_sub_;

    // Publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    // Timer (20Hz control loop)
    rclcpp::TimerBase::SharedPtr timer_;

    // ── State ──
    ParkingState state_;
    rclcpp::Time state_enter_time_;
    bool out_line_detected_;

    // Latest LiDAR scan cache
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    // Space detection state (SEARCH)
    bool wall_seen_;         // 측면에 벽/장애물 감지 이력
    int gap_point_count_;    // 연속 원거리 포인트 카운트

    // ALIGN state
    rclcpp::Time align_stop_time_;
    bool align_stopped_;

    // ── Parameters ──
    bool auto_start_;

    // SEARCH
    double search_speed_;
    double side_scan_min_deg_;
    double side_scan_max_deg_;
    double side_near_threshold_;
    double side_far_threshold_;
    int min_gap_points_;

    // ALIGN
    double overshoot_distance_;
    double align_duration_;

    // REVERSE_ENTER
    double reverse_speed_;
    double entry_steer_;
    double back_scan_min_deg_;
    double back_scan_max_deg_;
    double back_wall_target_;

    // STRAIGHTEN
    double straighten_speed_;
    double straighten_steer_kp_;
    double park_depth_;
    double left_scan_min_deg_;
    double left_scan_max_deg_;
    double right_scan_min_deg_;
    double right_scan_max_deg_;

    // HOLD
    double hold_duration_;

    // EXIT_FORWARD
    double exit_speed_;
    double exit_steer_;
    double exit_duration_;

    // SEEK_OUT
    double seek_speed_;
    double seek_out_wall_threshold_;      // 이 거리 이상이면 벽 감지 안 됨
    double seek_out_position_error_;     // 좌우 위치 차이 허용값 (m)

    // Safety
    double emergency_stop_dist_;

    // ── Callbacks ──
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lineCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void timerCallback();

    // ── State handlers ──
    void handleIdle();
    void handleSearch();
    void handleAlign();
    void handleReverseEnter();
    void handleStraighten();
    void handleHold();
    void handleExitForward();
    void handleSeekOut();

    // ── LiDAR utilities ──
    static double normalizeAngle(double rad);
    static bool angleInWindow(double angle, double start, double end);
    double getMinDistanceInWindow(double angle_min_deg, double angle_max_deg) const;
    double getMeanDistanceInWindow(double angle_min_deg, double angle_max_deg) const;
    int countPointsInWindow(double angle_min_deg, double angle_max_deg,
                            double min_range, double max_range) const;

    // ── Command helpers ──
    void publishCmd(double speed, double steer);
    void publishStop();
    void changeState(ParkingState new_state);
    const char* stateToString(ParkingState s) const;

    template<typename T>
    static T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
};

}  // namespace decision

#endif  // DECISION_PARKING_NODE_HPP
