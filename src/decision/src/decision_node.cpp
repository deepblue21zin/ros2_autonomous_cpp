#include "decision/decision_node.hpp"

namespace decision {

DecisionNode::DecisionNode()
    : Node("decision_node"),
      lane_steer_norm_(0.0),
      obstacle_(false),
      obstacle_bias_(0.0),
      ultra_min_(0.0),
      traffic_state_("unknown"),
      stop_line_distance_(-1.0) {
    // Declare and load parameters
    this->declare_parameter("cruise_speed_mps", 1.0);
    this->declare_parameter("max_steer_rad", 0.6);
    this->declare_parameter("soft_steer_rad", 0.3);
    this->declare_parameter("soft_steer_threshold", 0.35);
    this->declare_parameter("ultra_safe_distance_m", 0.0);  // 초음파 비활성화
    this->declare_parameter("use_traffic_light", false);
    this->declare_parameter("stop_on_yellow", true);
    this->declare_parameter("stop_line_stop_distance_m", 0.5);  // 정지선 정지 거리
    this->declare_parameter("lane_timeout_sec", 0.5);
    this->declare_parameter("use_obstacle_avoidance", true);
    this->declare_parameter("obstacle_bias_weight", 0.3);
    this->declare_parameter("test_mode", false);  // 테스트 모드: 센서 없이 모터 구동

    cruise_speed_ = this->get_parameter("cruise_speed_mps").as_double();
    max_steer_rad_ = this->get_parameter("max_steer_rad").as_double();
    soft_steer_rad_ = this->get_parameter("soft_steer_rad").as_double();
    soft_steer_threshold_ = this->get_parameter("soft_steer_threshold").as_double();
    ultra_safe_m_ = this->get_parameter("ultra_safe_distance_m").as_double();
    use_traffic_light_ = this->get_parameter("use_traffic_light").as_bool();
    stop_on_yellow_ = this->get_parameter("stop_on_yellow").as_bool();
    stop_line_stop_distance_ = this->get_parameter("stop_line_stop_distance_m").as_double();
    lane_timeout_ = this->get_parameter("lane_timeout_sec").as_double();
    use_obstacle_avoidance_ = this->get_parameter("use_obstacle_avoidance").as_bool();
    obstacle_bias_weight_ = this->get_parameter("obstacle_bias_weight").as_double();
    test_mode_ = this->get_parameter("test_mode").as_bool();

    // Setup subscribers
    lane_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/lane/steering_angle", 1,
        std::bind(&DecisionNode::laneCallback, this, std::placeholders::_1));

    obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/perception/obstacle_flag", 1,
        std::bind(&DecisionNode::obstacleCallback, this, std::placeholders::_1));

    obstacle_bias_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/perception/obstacle_bias", 1,
        std::bind(&DecisionNode::obstacleBiasCallback, this, std::placeholders::_1));

    ultra_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/ultrasonic/min_range", 1,
        std::bind(&DecisionNode::ultraCallback, this, std::placeholders::_1));

    traffic_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/perception/traffic_light_state", 1,
        std::bind(&DecisionNode::trafficCallback, this, std::placeholders::_1));

    stop_line_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/perception/stop_line", 1,
        std::bind(&DecisionNode::stopLineCallback, this, std::placeholders::_1));

    // Setup publisher
    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
        "/decision/cmd", 1);

    // Setup timer (20Hz = 50ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DecisionNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "[decision_unified] node started (test_mode=%s)",
                test_mode_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[decision_unified] obstacle_avoidance=%s bias_weight=%.2f",
                use_obstacle_avoidance_ ? "true" : "false", obstacle_bias_weight_);
    RCLCPP_INFO(this->get_logger(), "[decision_unified] control_hz=20.0");
    RCLCPP_INFO(this->get_logger(), "[decision_unified] traffic_light=%s stop_line_distance=%.2fm",
                use_traffic_light_ ? "true" : "false", stop_line_stop_distance_);
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

void DecisionNode::stopLineCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Parse stop line message: [x1, y1, x2, y2, distance_m]
    if (msg->data.size() >= 5) {
        stop_line_distance_ = msg->data[4];
    } else {
        stop_line_distance_ = -1.0;  // 정지선 없음
    }
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
    std::string reason = "";

    // 우선순위 1: 라이다 장애물 (위험 거리)
    if (obstacle_) {
        should_stop = true;
        reason = "lidar_obstacle";
    }

    // 우선순위 2: 초음파 전방 장애물 (위험 거리)
    if (!should_stop && ultra_min_ > 0.0 && ultra_min_ < ultra_safe_m_) {
        should_stop = true;
        char buf[64];
        snprintf(buf, sizeof(buf), "ultrasonic(%.2fm)", ultra_min_);
        reason = buf;
    }

    // 우선순위 3: 신호등 + 정지선 (선택적)
    if (!should_stop && use_traffic_light_) {
        // Option B: 빨강/노랑 + 정지선 가까움 → 정지, 초록 → 주행
        if (traffic_state_ == "red" || traffic_state_ == "yellow") {
            // 빨강/노랑 신호일 때 정지선 거리 확인
            if (stop_line_distance_ > 0.0 && stop_line_distance_ < stop_line_stop_distance_) {
                should_stop = true;
                char buf[128];
                snprintf(buf, sizeof(buf), "%s_light_at_stopline(%.2fm)",
                         traffic_state_.c_str(), stop_line_distance_);
                reason = buf;
            }
        }
        // 초록불이면 정지선 무시하고 주행 (명시적으로 처리 안 해도 should_stop=False 유지)
    }

    // 우선순위 4: 차선 유효성 확인
    if (!should_stop) {
        if (!lane_stamp_.has_value()) {
            should_stop = true;
            reason = "no_lane_data";
        } else {
            double age = (this->now() - lane_stamp_.value()).seconds();
            if (age > lane_timeout_) {
                should_stop = true;
                char buf[64];
                snprintf(buf, sizeof(buf), "lane_timeout(%.1fs)", age);
                reason = buf;
            }
        }
    }

    // 명령 생성
    if (should_stop) {
        cmd.speed = 0.0;
        cmd.steering_angle = 0.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[decision_unified] STOP: %s", reason.c_str());
    } else {
        // 주행: 차선 추종 + 장애물 회피
        double steer = clamp(lane_steer_norm_, -1.0, 1.0);

        // 카메라 장애물 회피 바이어스 적용
        if (use_obstacle_avoidance_ && std::abs(obstacle_bias_) > 0.01) {
            steer += obstacle_bias_ * obstacle_bias_weight_;
            steer = clamp(steer, -1.0, 1.0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "[decision_unified] AVOID: lane_steer=%.2f bias=%.2f → final=%.2f",
                                 lane_steer_norm_, obstacle_bias_, steer);
        }

        cmd.speed = cruise_speed_;
        cmd.steering_angle = mapSteer(steer);
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
