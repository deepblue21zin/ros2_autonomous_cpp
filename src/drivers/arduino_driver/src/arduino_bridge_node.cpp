#include "arduino_driver/arduino_bridge_node.hpp"
#include "arduino_driver/serial_protocol.hpp"
#include <sstream>

namespace arduino_driver {

ArduinoBridgeNode::ArduinoBridgeNode()
    : Node("arduino_bridge_node"), running_(true) {
    // Declare and load parameters
    this->declare_parameter("port", "/dev/ttyACM0");
    this->declare_parameter("baudrate", 9600);
    this->declare_parameter("command_topic", "/arduino/cmd");
    this->declare_parameter("ultrasonic_topic", "/ultrasonic/ranges");
    this->declare_parameter("use_legacy_cmd", true);
    this->declare_parameter("max_speed_mps", 2.0);
    this->declare_parameter("max_steer_deg", 30.0);
    this->declare_parameter("center_servo_deg", 90.0);

    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    command_topic_ = this->get_parameter("command_topic").as_string();
    ultrasonic_topic_ = this->get_parameter("ultrasonic_topic").as_string();
    use_legacy_cmd_ = this->get_parameter("use_legacy_cmd").as_bool();
    max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();
    max_steer_deg_ = this->get_parameter("max_steer_deg").as_double();
    center_servo_deg_ = this->get_parameter("center_servo_deg").as_double();

    // Setup ROS interfaces
    cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        command_topic_, 10,
        std::bind(&ArduinoBridgeNode::cmdCallback, this, std::placeholders::_1));
    
    ultra_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        ultrasonic_topic_, 10);

    // Initialize serial port
    initSerial();

    // Start read thread
    read_thread_ = std::thread(&ArduinoBridgeNode::readLoop, this);

    RCLCPP_INFO(this->get_logger(), "ArduinoBridgeNode initialized");
    RCLCPP_INFO(this->get_logger(), "  port: %s", port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  baudrate: %d", baudrate_);
    RCLCPP_INFO(this->get_logger(), "  use_legacy_cmd: %s", use_legacy_cmd_ ? "true" : "false");
}

ArduinoBridgeNode::~ArduinoBridgeNode() {
    running_ = false;

    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }

    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void ArduinoBridgeNode::initSerial() {
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_service_);
        serial_port_->open(port_);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));

        RCLCPP_INFO(this->get_logger(), "Serial port opened: %s at %d baud", 
                    port_.c_str(), baudrate_);
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", 
                     port_.c_str(), e.what());
        throw;
    }
}

void ArduinoBridgeNode::cmdCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(write_mutex_);

    if (!serial_port_ || !serial_port_->is_open()) {
        return;
    }

    try {
        if (use_legacy_cmd_) {
            std::string cmd = toLegacyCommand(*msg);
            if (cmd != last_cmd_) {
                std::string payload = cmd + "\n";
                boost::asio::write(*serial_port_, boost::asio::buffer(payload));
                last_cmd_ = cmd;
            }
        } else {
            int pwm = speedToPwm(msg->speed);
            int servo = steerToServo(msg->steering_angle);
            std::ostringstream oss;
            oss << "V:" << pwm << ",S:" << servo << "\n";
            boost::asio::write(*serial_port_, boost::asio::buffer(oss.str()));
        }
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Serial write error: %s", e.what());
    }
}

std::string ArduinoBridgeNode::toLegacyCommand(const ackermann_msgs::msg::AckermannDrive& msg) {
    // Check for stop
    if (std::abs(msg.speed) < 1e-3) {
        return "S";
    }

    // Convert steering angle to degrees
    double steer_deg = radToDeg(msg.steering_angle);

    // Check for reverse
    if (msg.speed < 0) {
        return "B";
    }

    // Forward with steering
    if (steer_deg > STEER_THRESHOLD_HARD) {
        return "R";  // Hard right
    }
    if (steer_deg < -STEER_THRESHOLD_HARD) {
        return "L";  // Hard left
    }
    if (steer_deg > STEER_THRESHOLD_SOFT) {
        return "r";  // Soft right
    }
    if (steer_deg < -STEER_THRESHOLD_SOFT) {
        return "l";  // Soft left
    }

    return "F";  // Forward straight
}

int ArduinoBridgeNode::speedToPwm(double speed) {
    double clamped = clamp(std::abs(speed), 0.0, max_speed_mps_);
    double ratio = clamped / max_speed_mps_;
    return static_cast<int>(clamp(ratio * 255.0, 0.0, 255.0));
}

int ArduinoBridgeNode::steerToServo(double steering_angle) {
    double steer_deg = radToDeg(steering_angle);
    steer_deg = clamp(steer_deg, -max_steer_deg_, max_steer_deg_);
    return static_cast<int>(center_servo_deg_ + steer_deg);
}

void ArduinoBridgeNode::readLoop() {
    std::string buffer;
    char c;

    while (running_ && rclcpp::ok()) {
        try {
            if (!serial_port_ || !serial_port_->is_open()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            boost::asio::read(*serial_port_, boost::asio::buffer(&c, 1));

            if (c == '\n') {
                auto parsed = parseUltrasonicLine(buffer);
                if (!parsed.empty()) {
                    std_msgs::msg::Float32MultiArray msg;
                    msg.data.reserve(SENSOR_ORDER.size());

                    for (const auto& key : SENSOR_ORDER) {
                        auto it = parsed.find(key);
                        msg.data.push_back(it != parsed.end() ? static_cast<float>(it->second) : 0.0f);
                    }

                    ultra_pub_->publish(msg);
                }
                buffer.clear();
            } else if (c != '\r') {
                buffer += c;
                // Prevent buffer overflow
                if (buffer.size() > 1024) {
                    buffer = buffer.substr(512);
                }
            }
        } catch (const boost::system::system_error& e) {
            if (running_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "Serial read error: %s", e.what());
            }
        }
    }
}

std::map<std::string, double> ArduinoBridgeNode::parseUltrasonicLine(const std::string& line) {
    return parseUltrasonicData(line);
}

}  // namespace arduino_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<arduino_driver::ArduinoBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_bridge_node"), 
                     "Exception in arduino_bridge_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
