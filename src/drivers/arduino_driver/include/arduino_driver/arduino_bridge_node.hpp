#ifndef ARDUINO_DRIVER_ARDUINO_BRIDGE_NODE_HPP
#define ARDUINO_DRIVER_ARDUINO_BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <string>
#include <map>
#include <array>
#include <thread>
#include <atomic>
#include <mutex>

namespace arduino_driver {

// Sensor order for ultrasonic data
const std::array<std::string, 6> SENSOR_ORDER = {"F", "FL", "FR", "R", "RL", "RR"};

// Steering thresholds (degrees)
constexpr double STEER_THRESHOLD_HARD = 15.0;
constexpr double STEER_THRESHOLD_SOFT = 5.0;

class ArduinoBridgeNode : public rclcpp::Node {
public:
    ArduinoBridgeNode();
    ~ArduinoBridgeNode();

private:
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ultra_pub_;

    // Parameters
    std::string port_;
    int baudrate_;
    std::string command_topic_;
    std::string ultrasonic_topic_;
    bool use_legacy_cmd_;
    double max_speed_mps_;
    double max_steer_deg_;
    double center_servo_deg_;

    // Serial communication (Boost.Asio)
    boost::asio::io_service io_service_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::thread read_thread_;
    std::atomic<bool> running_;
    std::mutex write_mutex_;

    // Last command for deduplication
    std::string last_cmd_;

    // Callbacks
    void cmdCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);

    // Serial initialization
    void initSerial();

    // Command conversion
    std::string toLegacyCommand(const ackermann_msgs::msg::AckermannDrive& msg);
    int speedToPwm(double speed);
    int steerToServo(double steering_angle);

    // Serial reading thread
    void readLoop();
    std::map<std::string, double> parseUltrasonicLine(const std::string& line);
};

}  // namespace arduino_driver

#endif  // ARDUINO_DRIVER_ARDUINO_BRIDGE_NODE_HPP
