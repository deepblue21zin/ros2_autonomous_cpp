#include "decision/decision_node.hpp"

namespace decision {

DecisionNode::DecisionNode()
    : Node("decision_node"),
      lane_steer_norm_(0.0),
      obstacle_(false),
      obstacle_bias_(0.0),
      ultra_min_(0.0),
      traffic_state_("unknown") {
    // Declare and load parameters
    this->declare_parameter("cruise_speed_mps", 1.0);
    this->declare_parameter("max_steer_rad", 0.6);
    this->declare_parameter("soft_steer_rad", 0.3);
    this->declare_parameter("soft_steer_threshold", 0.35);
    this->declare_parameter("ultra_safe_distance_m", 0.2);
    this->declare_parameter("use_traffic_light", false);
    this->declare_parameter("stop_on_yellow", true);
    this->declare_parameter("lane_timeout_sec", 0.5);
    this->declare_parameter("use_obstacle_avoidance", true);
    this->declare_parameter("obstacle_bias_weight", 0.5);
    this->declare_parameter("test_mode", false);  // 테스트 모드: 센서 없이 모터 구동

    cruise_speed_ = this->get_parameter("cruise_speed_mps").as_double();
    max_steer_rad_ = this->get_parameter("max_steer_rad").as_double();
    soft_steer_rad_ = this->get_parameter("soft_steer_rad").as_double();
    soft_steer_threshold_ = this->get_parameter("soft_steer_threshold").as_double();
    ultra_safe_m_ = this->get_parameter("ultra_safe_distance_m").as_double();
    use_traffic_light_ = this->get_parameter("use_traffic_light").as_bool();
    stop_on_yellow_ = this->get_parameter("stop_on_yellow").as_bool();
    lane_timeout_ = this->get_parameter("lane_timeout_sec").as_double();
    use_obstacle_avoidance_ = this->get_parameter("use_obstacle_avoidance").as_bool();
    obstacle_bias_weight_ = this->get_parameter("obstacle_bias_weight").as_double();
    test_mode_ = this->get_parameter("test_mode").as_bool();

    // Setup subscribers
    lane_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/lane/steering_angle", 10,
        std::bind(&DecisionNode::laneCallback, this, std::placeholders::_1));
    
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/perception/obstacle_flag", 10,
        std::bind(&DecisionNode::obstacleCallback, this, std::placeholders::_1));
    
    obstacle_bias_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/perception/obstacle_bias", 10,
        std::bind(&DecisionNode::obstacleBiasCallback, this, std::placeholders::_1));
    
    ultra_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/ultrasonic/min_range", 10,
        std::bind(&DecisionNode::ultraCallback, this, std::placeholders::_1));
    
    traffic_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/perception/traffic_light_state", 10,
        std::bind(&DecisionNode::trafficCallback, this, std::placeholders::_1));

    // Setup publisher with reliable QoS for control commands (delay optimization!)
    auto qos_reliable = rclcpp::QoS(10).reliable();
    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
        "/decision/cmd", qos_reliable);

    // Setup timer (10Hz = 100ms) - CRITICAL FOR LOW LATENCY!
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DecisionNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "DecisionNode initialized (10Hz)");
    RCLCPP_INFO(this->get_logger(), "  cruise_speed: %.2f m/s", cruise_speed_);
    RCLCPP_INFO(this->get_logger(), "  max_steer_rad: %.2f", max_steer_rad_);
    RCLCPP_INFO(this->get_logger(), "  soft_steer_rad: %.2f", soft_steer_rad_);
    RCLCPP_INFO(this->get_logger(), "  ultra_safe_m: %.2f", ultra_safe_m_);
    RCLCPP_INFO(this->get_logger(), "  use_traffic_light: %s", use_traffic_light_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  lane_timeout: %.2f sec", lane_timeout_);
    RCLCPP_INFO(this->get_logger(), "  use_obstacle_avoidance: %s", use_obstacle_avoidance_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  obstacle_bias_weight: %.2f", obstacle_bias_weight_);
    RCLCPP_INFO(this->get_logger(), "  test_mode: %s", test_mode_ ? "true" : "false");

    if (test_mode_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ TEST MODE ENABLED - Sensors bypassed!");
    }
}

void DecisionNode::laneCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    lane_steer_norm_ = msg->data;
    lane_stamp_ = this->now();
}

void DecisionNode::obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    obstacle_ = msg->data;
}

void DecisionNode::obstacleBiasCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    obstacle_bias_ = msg->data;
}

void DecisionNode::ultraCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    ultra_min_ = msg->data;
}

void DecisionNode::trafficCallback(const std_msgs::msg::String::SharedPtr msg) {
    traffic_state_ = msg->data;
}

double DecisionNode::mapSteer(double steer_norm) const {
    // Linear mapping if soft_steer_threshold disabled
    if (soft_steer_threshold_ <= 0.0) {
        return steer_norm * max_steer_rad_;
    }

    double abs_steer = std::abs(steer_norm);
    double sign = (steer_norm >= 0.0) ? 1.0 : -1.0;

    // Soft steering zone
    if (abs_steer <= soft_steer_threshold_) {
        double scale = abs_steer / soft_steer_threshold_;
        return sign * scale * soft_steer_rad_;
    }

    // Full steering range
    return steer_norm * max_steer_rad_;
}

void DecisionNode::timerCallback() {
    ackermann_msgs::msg::AckermannDrive cmd;
    bool should_stop = false;

    // 테스트 모드: 모든 센서 체크 우회
    if (test_mode_) {
        // 테스트 모드에서는 기본 속도로 직진
        cmd.speed = cruise_speed_;
        cmd.steering_angle = mapSteer(lane_steer_norm_);
        cmd_pub_->publish(cmd);
        return;
    }

    // Check 1: LiDAR obstacle
    if (obstacle_) {
        should_stop = true;
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Stopping: LiDAR obstacle detected");
    }

    // Check 2: Ultrasonic safety
    if (!should_stop && ultra_min_ > 0.0 && ultra_min_ < ultra_safe_m_) {
        should_stop = true;
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Stopping: Ultrasonic obstacle at %.2f m", ultra_min_);
    }

    // Check 3: Traffic light (optional)
    if (!should_stop && use_traffic_light_) {
        if (traffic_state_ == "red") {
            should_stop = true;
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Stopping: Red traffic light");
        } else if (stop_on_yellow_ && traffic_state_ == "yellow") {
            should_stop = true;
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Stopping: Yellow traffic light");
        }
    }

    // Check 4: Lane availability
    if (!should_stop) {
        if (!lane_stamp_.has_value()) {
            should_stop = true;
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Stopping: No lane data received");
        } else {
            double age = (this->now() - lane_stamp_.value()).seconds();
            if (age > lane_timeout_) {
                should_stop = true;
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "Stopping: Lane data timeout (%.2f sec)", age);
            }
        }
    }

    // Generate command
    if (should_stop) {
        cmd.speed = 0.0;
        cmd.steering_angle = 0.0;
    } else {
        // Apply obstacle avoidance if enabled
        double steer_norm = lane_steer_norm_;
        if (use_obstacle_avoidance_) {
            steer_norm += obstacle_bias_ * obstacle_bias_weight_;
        }
        double steer_clamped = clamp(steer_norm, -1.0, 1.0);
        cmd.speed = cruise_speed_;
        cmd.steering_angle = mapSteer(steer_clamped);
    }

    cmd_pub_->publish(cmd);
}

}  // namespace decision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<decision::DecisionNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("decision_node"),
                     "Exception in decision_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
