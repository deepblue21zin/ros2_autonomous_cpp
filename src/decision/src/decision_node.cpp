#include "decision/decision_node.hpp"

#include <thread>
#include <chrono>

namespace decision {

DecisionNode::DecisionNode()
    : Node("decision_node"),
      lane_steer_norm_(0.0),
      obstacle_(false),
      obstacle_bias_(0.0),
      ultra_min_(0.0),
      traffic_state_("unknown"),
      stop_line_distance_(-1.0),
      current_cmd_speed_(0.0),
      prev_steer_rad_(0.0),
      lane_lost_count_(0) {
    // Declare and load parameters
    this->declare_parameter("cruise_speed_mps", 9.0);
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

    // Speed control parameters
    this->declare_parameter("min_speed_mps", 5.0);
    this->declare_parameter("curve_factor", 0.4);
    this->declare_parameter("accel_rate_mps2", 0.5);
    this->declare_parameter("decel_rate_mps2", 1.0);
    this->declare_parameter("obstacle_d_stop_m", 0.5);
    this->declare_parameter("obstacle_d_slow_m", 1.5);
    this->declare_parameter("obstacle_k", 0.8);
    this->declare_parameter("obstacle_max_speed_mps", 0.6);
    this->declare_parameter("lane_lost_hold_frames", 45);
    this->declare_parameter("lost_lane_speed_mps", 0.3);
    this->declare_parameter("steer_rate_limit_rad_per_sec", 2.0);

    // Sensor timeout parameters
    this->declare_parameter("obstacle_timeout_sec", 0.5);
    this->declare_parameter("obstacle_bias_timeout_sec", 0.5);
    this->declare_parameter("ultra_timeout_sec", 1.0);
    this->declare_parameter("traffic_timeout_sec", 2.0);
    this->declare_parameter("stop_line_timeout_sec", 1.0);

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

    // Speed control
    min_speed_ = this->get_parameter("min_speed_mps").as_double();
    curve_factor_ = this->get_parameter("curve_factor").as_double();
    accel_rate_ = this->get_parameter("accel_rate_mps2").as_double();
    decel_rate_ = this->get_parameter("decel_rate_mps2").as_double();
    obstacle_d_stop_ = this->get_parameter("obstacle_d_stop_m").as_double();
    obstacle_d_slow_ = this->get_parameter("obstacle_d_slow_m").as_double();
    obstacle_k_ = this->get_parameter("obstacle_k").as_double();
    obstacle_max_speed_ = this->get_parameter("obstacle_max_speed_mps").as_double();
    lane_lost_hold_frames_ = this->get_parameter("lane_lost_hold_frames").as_int();
    lost_lane_speed_ = this->get_parameter("lost_lane_speed_mps").as_double();
    steer_rate_limit_ = this->get_parameter("steer_rate_limit_rad_per_sec").as_double();

    // Sensor timeouts
    obstacle_timeout_ = this->get_parameter("obstacle_timeout_sec").as_double();
    obstacle_bias_timeout_ = this->get_parameter("obstacle_bias_timeout_sec").as_double();
    ultra_timeout_ = this->get_parameter("ultra_timeout_sec").as_double();
    traffic_timeout_ = this->get_parameter("traffic_timeout_sec").as_double();
    stop_line_timeout_ = this->get_parameter("stop_line_timeout_sec").as_double();

    last_timer_time_ = this->now();

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

    // Setup publishers
    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
        "/decision/cmd", 1);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/decision/status", 1);

    // Setup timer (20Hz = 50ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DecisionNode::timerCallback, this));

    // Setup dynamic parameter callback (안전한 파라미터만 동적 변경 허용)
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DecisionNode::onParameterChange, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[decision] node started (test_mode=%s)",
                test_mode_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[decision] obstacle_avoidance=%s bias_weight=%.2f",
                use_obstacle_avoidance_ ? "true" : "false", obstacle_bias_weight_);
    RCLCPP_INFO(this->get_logger(), "[decision] steer_rate_limit=%.1f rad/s", steer_rate_limit_);
    RCLCPP_INFO(this->get_logger(), "[decision] sensor_timeouts: obstacle=%.1fs ultra=%.1fs traffic=%.1fs",
                obstacle_timeout_, ultra_timeout_, traffic_timeout_);
    RCLCPP_INFO(this->get_logger(), "[decision] traffic_light=%s stop_line_distance=%.2fm",
                use_traffic_light_ ? "true" : "false", stop_line_stop_distance_);
}

void DecisionNode::shutdown() {
    RCLCPP_WARN(this->get_logger(), "[decision] SHUTDOWN: Sending stop commands...");
    ackermann_msgs::msg::AckermannDrive stop_cmd;
    stop_cmd.speed = 0.0;
    stop_cmd.steering_angle = 0.0;
    for (int i = 0; i < 5; i++) {
        cmd_pub_->publish(stop_cmd);
        RCLCPP_INFO(this->get_logger(), "[decision] Stop command %d/5 sent", i + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_WARN(this->get_logger(), "[decision] SHUTDOWN: Stop commands completed.");
}

void DecisionNode::laneCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    lane_steer_norm_ = msg->data;
    lane_stamp_ = this->now();
}

void DecisionNode::obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    obstacle_ = msg->data;
    obstacle_stamp_ = this->now();
}

void DecisionNode::obstacleBiasCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    obstacle_bias_ = msg->data;
    obstacle_bias_stamp_ = this->now();
}

void DecisionNode::ultraCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    ultra_min_ = msg->data;
    ultra_stamp_ = this->now();
}

void DecisionNode::trafficCallback(const std_msgs::msg::String::SharedPtr msg) {
    traffic_state_ = msg->data;
    traffic_stamp_ = this->now();
}

void DecisionNode::stopLineCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Parse stop line message: [x1, y1, x2, y2, distance_m]
    if (msg->data.size() >= 5) {
        stop_line_distance_ = msg->data[4];
    } else {
        stop_line_distance_ = -1.0;  // 정지선 없음
    }
    stop_line_stamp_ = this->now();
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

void DecisionNode::checkSensorFreshness() {
    rclcpp::Time now = this->now();

    // Obstacle: stale → false (주행 계속)
    if (obstacle_stamp_.has_value()) {
        double age = (now - obstacle_stamp_.value()).seconds();
        if (age > obstacle_timeout_) {
            if (obstacle_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[decision] obstacle data stale (%.1fs) → reset to false", age);
            }
            obstacle_ = false;
        }
    }

    // Obstacle bias: stale → 0.0 (회피 비활성)
    if (obstacle_bias_stamp_.has_value()) {
        double age = (now - obstacle_bias_stamp_.value()).seconds();
        if (age > obstacle_bias_timeout_) {
            obstacle_bias_ = 0.0;
        }
    }

    // Ultrasonic: stale → 0.0 (무시됨, ultra_min_ > 0.0 조건에 의해)
    if (ultra_stamp_.has_value()) {
        double age = (now - ultra_stamp_.value()).seconds();
        if (age > ultra_timeout_) {
            ultra_min_ = 0.0;
        }
    }

    // Traffic light: stale → "unknown" (정지 안 함)
    if (traffic_stamp_.has_value()) {
        double age = (now - traffic_stamp_.value()).seconds();
        if (age > traffic_timeout_) {
            traffic_state_ = "unknown";
        }
    }

    // Stop line: stale → -1.0 (정지선 없음)
    if (stop_line_stamp_.has_value()) {
        double age = (now - stop_line_stamp_.value()).seconds();
        if (age > stop_line_timeout_) {
            stop_line_distance_ = -1.0;
        }
    }
}

rcl_interfaces::msg::SetParametersResult DecisionNode::onParameterChange(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        const auto& name = param.get_name();

        if (name == "cruise_speed_mps") {
            cruise_speed_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: cruise_speed=%.2f", cruise_speed_);
        } else if (name == "min_speed_mps") {
            min_speed_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: min_speed=%.2f", min_speed_);
        } else if (name == "curve_factor") {
            curve_factor_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: curve_factor=%.2f", curve_factor_);
        } else if (name == "obstacle_bias_weight") {
            obstacle_bias_weight_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: obstacle_bias_weight=%.2f", obstacle_bias_weight_);
        } else if (name == "steer_rate_limit_rad_per_sec") {
            steer_rate_limit_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: steer_rate_limit=%.2f", steer_rate_limit_);
        } else if (name == "accel_rate_mps2") {
            accel_rate_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: accel_rate=%.2f", accel_rate_);
        } else if (name == "decel_rate_mps2") {
            decel_rate_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: decel_rate=%.2f", decel_rate_);
        } else if (name == "lost_lane_speed_mps") {
            lost_lane_speed_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "[decision] param update: lost_lane_speed=%.2f", lost_lane_speed_);
        }
        // 안전 관련 파라미터는 동적 변경 거부
        else if (name == "max_steer_rad" || name == "ultra_safe_distance_m" ||
                 name == "use_traffic_light" || name == "test_mode") {
            result.successful = false;
            result.reason = "Safety parameter '" + name + "' cannot be changed at runtime";
            RCLCPP_WARN(this->get_logger(), "[decision] REJECTED param change: %s (safety)", name.c_str());
            return result;
        }
    }
    return result;
}

void DecisionNode::timerCallback() {
    // dt 계산 (실제 시간 기반)
    rclcpp::Time now = this->now();
    double dt = (now - last_timer_time_).seconds();
    last_timer_time_ = now;
    dt = clamp(dt, 0.001, 0.2);  // 이상치 방어 (1ms ~ 200ms)

    // === 센서 유효성 검사 (stale 데이터 리셋) ===
    checkSensorFreshness();

    ackermann_msgs::msg::AckermannDrive cmd;
    bool e_stop = false;
    std::string reason = "";
    std::string status = "";

    // === 긴급 정지 판정 (e-stop: 램핑 무시, 즉시 0) ===

    // 우선순위 1: 라이다 장애물 (위험 거리)
    if (obstacle_) {
        e_stop = true;
        reason = "lidar_obstacle";
    }

    // 우선순위 2: 초음파 전방 장애물 (위험 거리)
    if (!e_stop && ultra_min_ > 0.0 && ultra_min_ < ultra_safe_m_) {
        e_stop = true;
        char buf[64];
        snprintf(buf, sizeof(buf), "ultrasonic(%.2fm)", ultra_min_);
        reason = buf;
    }

    // 우선순위 3: 신호등 + 정지선 (선택적)
    if (!e_stop && use_traffic_light_) {
        if (traffic_state_ == "red" || traffic_state_ == "yellow") {
            if (stop_line_distance_ > 0.0 && stop_line_distance_ < stop_line_stop_distance_) {
                e_stop = true;
                char buf[128];
                snprintf(buf, sizeof(buf), "%s_light_at_stopline(%.2fm)",
                         traffic_state_.c_str(), stop_line_distance_);
                reason = buf;
            }
        }
    }

    // 긴급 정지 시 즉시 0 (램핑 무시)
    if (e_stop) {
        cmd.speed = 0.0;
        cmd.steering_angle = 0.0;
        current_cmd_speed_ = 0.0;
        prev_steer_rad_ = 0.0;
        status = "E_STOP reason=" + reason;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[decision] E-STOP: %s", reason.c_str());
        cmd_pub_->publish(cmd);

        // 상태 퍼블리시 (변경 시에만)
        if (status != last_status_) {
            std_msgs::msg::String status_msg;
            status_msg.data = status;
            status_pub_->publish(status_msg);
            last_status_ = status;
        }
        return;
    }

    // === 차선 유효성 + 차선 소실 감속 ===
    bool lane_valid = false;
    if (lane_stamp_.has_value()) {
        double age = (now - lane_stamp_.value()).seconds();
        if (age <= lane_timeout_) {
            lane_valid = true;
            lane_lost_count_ = 0;
        }
    }

    if (!lane_valid && !test_mode_) {
        lane_lost_count_++;
        if (lane_lost_count_ > lane_lost_hold_frames_) {
            // hold 초과 → 정지
            cmd.speed = 0.0;
            cmd.steering_angle = 0.0;
            current_cmd_speed_ = 0.0;
            prev_steer_rad_ = 0.0;
            status = "LANE_LOST count=" + std::to_string(lane_lost_count_) +
                     "/" + std::to_string(lane_lost_hold_frames_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[decision] STOP: lane_lost(%d frames)", lane_lost_count_);
            cmd_pub_->publish(cmd);

            if (status != last_status_) {
                std_msgs::msg::String status_msg;
                status_msg.data = status;
                status_pub_->publish(status_msg);
                last_status_ = status;
            }
            return;
        }
    }

    // === 조향 계산 ===
    double steer = clamp(lane_steer_norm_, -1.0, 1.0);

    // 카메라 장애물 회피 바이어스 적용
    if (use_obstacle_avoidance_ && std::abs(obstacle_bias_) > 0.01) {
        steer += obstacle_bias_ * obstacle_bias_weight_;
        steer = clamp(steer, -1.0, 1.0);
    }

    // === target_speed 결정 ===
    double target_speed = cruise_speed_;

    // 1) 커브 감속: |steer| 비례
    double steer_norm = std::abs(steer);
    double curve_speed = cruise_speed_ * (1.0 - curve_factor_ * steer_norm);
    target_speed = std::min(target_speed, curve_speed);

    // 2) 장애물 거리 감속 (초음파 piecewise)
    if (ultra_min_ > 0.0) {
        double d = ultra_min_;
        if (d <= obstacle_d_stop_) {
            target_speed = 0.0;
        } else if (d < obstacle_d_slow_) {
            double v_obstacle = obstacle_k_ * (d - obstacle_d_stop_);
            v_obstacle = clamp(v_obstacle, 0.0, obstacle_max_speed_);
            target_speed = std::min(target_speed, v_obstacle);
        }
    }

    // 3) 차선 소실 감속
    if (!lane_valid && !test_mode_ && lane_lost_count_ <= lane_lost_hold_frames_) {
        target_speed = std::min(target_speed, lost_lane_speed_);
    }

    // min_speed 적용 (NORMAL 상태에서만 - 위의 조건들에서 0으로 설정된 경우는 0 유지)
    if (target_speed > 0.0) {
        target_speed = std::max(target_speed, min_speed_);
    }

    // === dt 기반 속도 램핑 ===
    double dv_up = accel_rate_ * dt;
    double dv_dn = decel_rate_ * dt;
    double delta = target_speed - current_cmd_speed_;
    current_cmd_speed_ += clamp(delta, -dv_dn, dv_up);
    current_cmd_speed_ = clamp(current_cmd_speed_, 0.0, cruise_speed_);

    // === 조향 변화율 제한 ===
    double target_steer_rad = mapSteer(steer);
    if (steer_rate_limit_ > 0.0) {
        double max_steer_delta = steer_rate_limit_ * dt;
        double steer_delta = target_steer_rad - prev_steer_rad_;
        target_steer_rad = prev_steer_rad_ + clamp(steer_delta, -max_steer_delta, max_steer_delta);
    }
    prev_steer_rad_ = target_steer_rad;

    // === 출력 ===
    cmd.speed = current_cmd_speed_;
    cmd.steering_angle = target_steer_rad;
    cmd_pub_->publish(cmd);

    // === 상태 퍼블리시 (변경 시에만) ===
    char status_buf[256];
    if (!lane_valid && !test_mode_) {
        snprintf(status_buf, sizeof(status_buf),
                 "LANE_SLOW speed=%.2f steer=%.2f count=%d/%d",
                 current_cmd_speed_, target_steer_rad, lane_lost_count_, lane_lost_hold_frames_);
    } else {
        snprintf(status_buf, sizeof(status_buf),
                 "DRIVING speed=%.2f steer=%.2f",
                 current_cmd_speed_, target_steer_rad);
    }
    status = status_buf;
    if (status != last_status_) {
        std_msgs::msg::String status_msg;
        status_msg.data = status;
        status_pub_->publish(status_msg);
        last_status_ = status;
    }
}

}  // namespace decision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<decision::DecisionNode> node;
    try {
        node = std::make_shared<decision::DecisionNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("decision_node"),
                     "Exception in decision_node: %s", e.what());
    }

    // 종료 시 안전 정지 명령 전송
    if (node) {
        node->shutdown();
    }

    rclcpp::shutdown();
    return 0;
}
