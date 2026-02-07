#include "decision/parking_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace decision {

ParkingNode::ParkingNode()
    : Node("parking_node"),
      state_(ParkingState::IDLE),
      out_line_detected_(false),
      wall_seen_(false),
      gap_point_count_(0),
      align_stopped_(false) {

    // ── Declare parameters ──
    this->declare_parameter("auto_start", true);

    // SEARCH
    this->declare_parameter("search_speed_mps", 0.3);
    this->declare_parameter("side_scan_angle_min_deg", 80.0);
    this->declare_parameter("side_scan_angle_max_deg", 100.0);
    this->declare_parameter("side_near_threshold_m", 0.6);
    this->declare_parameter("side_far_threshold_m", 1.5);
    this->declare_parameter("min_gap_points", 5);

    // ALIGN
    this->declare_parameter("overshoot_distance_m", 0.3);
    this->declare_parameter("align_duration_sec", 1.5);

    // REVERSE_ENTER
    this->declare_parameter("reverse_speed_mps", -0.25);
    this->declare_parameter("entry_steer_rad", 0.4);
    this->declare_parameter("back_scan_angle_min_deg", 350.0);
    this->declare_parameter("back_scan_angle_max_deg", 10.0);
    this->declare_parameter("back_wall_target_m", 0.5);

    // STRAIGHTEN
    this->declare_parameter("straighten_speed_mps", -0.15);
    this->declare_parameter("straighten_steer_kp", 2.0);
    this->declare_parameter("park_depth_m", 0.25);
    this->declare_parameter("left_scan_angle_min_deg", 260.0);
    this->declare_parameter("left_scan_angle_max_deg", 280.0);
    this->declare_parameter("right_scan_angle_min_deg", 80.0);
    this->declare_parameter("right_scan_angle_max_deg", 100.0);

    // HOLD
    this->declare_parameter("hold_duration_sec", 3.0);

    // EXIT_FORWARD
    this->declare_parameter("exit_speed_mps", 0.3);
    this->declare_parameter("exit_steer_rad", -0.4);
    this->declare_parameter("exit_duration_sec", 2.5);

    // SEEK_OUT
    this->declare_parameter("seek_speed_mps", 0.3);

    // Safety
    this->declare_parameter("emergency_stop_distance_m", 0.10);

    // ── Load parameters ──
    auto_start_ = this->get_parameter("auto_start").as_bool();

    search_speed_ = this->get_parameter("search_speed_mps").as_double();
    side_scan_min_deg_ = this->get_parameter("side_scan_angle_min_deg").as_double();
    side_scan_max_deg_ = this->get_parameter("side_scan_angle_max_deg").as_double();
    side_near_threshold_ = this->get_parameter("side_near_threshold_m").as_double();
    side_far_threshold_ = this->get_parameter("side_far_threshold_m").as_double();
    min_gap_points_ = this->get_parameter("min_gap_points").as_int();

    overshoot_distance_ = this->get_parameter("overshoot_distance_m").as_double();
    align_duration_ = this->get_parameter("align_duration_sec").as_double();

    reverse_speed_ = this->get_parameter("reverse_speed_mps").as_double();
    entry_steer_ = this->get_parameter("entry_steer_rad").as_double();
    back_scan_min_deg_ = this->get_parameter("back_scan_angle_min_deg").as_double();
    back_scan_max_deg_ = this->get_parameter("back_scan_angle_max_deg").as_double();
    back_wall_target_ = this->get_parameter("back_wall_target_m").as_double();

    straighten_speed_ = this->get_parameter("straighten_speed_mps").as_double();
    straighten_steer_kp_ = this->get_parameter("straighten_steer_kp").as_double();
    park_depth_ = this->get_parameter("park_depth_m").as_double();
    left_scan_min_deg_ = this->get_parameter("left_scan_angle_min_deg").as_double();
    left_scan_max_deg_ = this->get_parameter("left_scan_angle_max_deg").as_double();
    right_scan_min_deg_ = this->get_parameter("right_scan_angle_min_deg").as_double();
    right_scan_max_deg_ = this->get_parameter("right_scan_angle_max_deg").as_double();

    hold_duration_ = this->get_parameter("hold_duration_sec").as_double();

    exit_speed_ = this->get_parameter("exit_speed_mps").as_double();
    exit_steer_ = this->get_parameter("exit_steer_rad").as_double();
    exit_duration_ = this->get_parameter("exit_duration_sec").as_double();

    seek_speed_ = this->get_parameter("seek_speed_mps").as_double();

    emergency_stop_dist_ = this->get_parameter("emergency_stop_distance_m").as_double();

    // ── Subscribers ──
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ParkingNode::scanCallback, this, std::placeholders::_1));

    line_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/parking/line_detected", 10,
        std::bind(&ParkingNode::lineCallback, this, std::placeholders::_1));

    // ── Publishers ──
    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
        "/decision/cmd", 1);

    state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/parking/state", 10);

    // ── Timer: 20Hz control loop ──
    timer_ = this->create_wall_timer(50ms,
        std::bind(&ParkingNode::timerCallback, this));

    state_enter_time_ = this->now();

    // Auto-start
    if (auto_start_) {
        changeState(ParkingState::SEARCH);
    }

    RCLCPP_INFO(this->get_logger(), "ParkingNode initialized");
    RCLCPP_INFO(this->get_logger(), "  auto_start: %s", auto_start_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  search_speed: %.2f m/s", search_speed_);
    RCLCPP_INFO(this->get_logger(), "  side_scan: %.0f~%.0f deg", side_scan_min_deg_, side_scan_max_deg_);
    RCLCPP_INFO(this->get_logger(), "  entry_steer: %.2f rad", entry_steer_);
    RCLCPP_INFO(this->get_logger(), "  hold_duration: %.1f sec", hold_duration_);
}

// ════════════════════════════════════════════════════
// Callbacks
// ════════════════════════════════════════════════════

void ParkingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
}

void ParkingNode::lineCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    out_line_detected_ = msg->data;
}

void ParkingNode::timerCallback() {
    // Publish current state for debug
    std_msgs::msg::String state_msg;
    state_msg.data = stateToString(state_);
    state_pub_->publish(state_msg);

    // No scan data yet → stay still
    if (!latest_scan_ && state_ != ParkingState::IDLE && state_ != ParkingState::DONE) {
        publishStop();
        return;
    }

    switch (state_) {
        case ParkingState::IDLE:          handleIdle(); break;
        case ParkingState::SEARCH:        handleSearch(); break;
        case ParkingState::ALIGN:         handleAlign(); break;
        case ParkingState::REVERSE_ENTER: handleReverseEnter(); break;
        case ParkingState::STRAIGHTEN:    handleStraighten(); break;
        case ParkingState::HOLD:          handleHold(); break;
        case ParkingState::EXIT_FORWARD:  handleExitForward(); break;
        case ParkingState::SEEK_OUT:      handleSeekOut(); break;
        case ParkingState::DONE:          publishStop(); break;
    }
}

// ════════════════════════════════════════════════════
// State handlers
// ════════════════════════════════════════════════════

void ParkingNode::handleIdle() {
    publishStop();
}

void ParkingNode::handleSearch() {
    // 저속 전진하면서 측면 스캔으로 주차 공간 탐색
    double side_min = getMinDistanceInWindow(side_scan_min_deg_, side_scan_max_deg_);

    if (side_min > 0.0 && side_min < side_near_threshold_) {
        // 벽/장애물 있음 → gap 카운트 리셋, 벽 이력 기록
        wall_seen_ = true;
        gap_point_count_ = 0;
    } else if (wall_seen_ && side_min > side_far_threshold_) {
        // 벽이 있다가 갑자기 멀어짐 → 빈 공간
        gap_point_count_++;

        if (gap_point_count_ >= min_gap_points_) {
            RCLCPP_INFO(this->get_logger(),
                "Space detected! gap_points=%d, side_dist=%.2f",
                gap_point_count_, side_min);
            changeState(ParkingState::ALIGN);
            return;
        }
    }

    publishCmd(search_speed_, 0.0);
}

void ParkingNode::handleAlign() {
    // 공간을 약간 지나친 후 정지 (overshoot)
    double elapsed = (this->now() - state_enter_time_).seconds();

    if (!align_stopped_) {
        // overshoot: align_duration 동안 저속 전진
        if (elapsed < align_duration_) {
            publishCmd(search_speed_ * 0.5, 0.0);
        } else {
            // 정지 후 잠시 대기
            publishStop();
            align_stopped_ = true;
            align_stop_time_ = this->now();
        }
    } else {
        // 정지 후 0.5초 대기 → REVERSE_ENTER
        if ((this->now() - align_stop_time_).seconds() > 0.5) {
            changeState(ParkingState::REVERSE_ENTER);
        } else {
            publishStop();
        }
    }
}

void ParkingNode::handleReverseEnter() {
    // 후진 + 조향으로 주차 공간 진입
    double back_dist = getMinDistanceInWindow(back_scan_min_deg_, back_scan_max_deg_);

    if (back_dist > 0.0 && back_dist < back_wall_target_) {
        // 뒤 벽에 충분히 가까워짐 → 정렬 단계
        RCLCPP_INFO(this->get_logger(),
            "Back wall reached: %.2f m (target: %.2f)", back_dist, back_wall_target_);
        changeState(ParkingState::STRAIGHTEN);
        return;
    }

    // 비상 정지 체크
    if (back_dist > 0.0 && back_dist < emergency_stop_dist_) {
        RCLCPP_WARN(this->get_logger(), "Emergency stop! back_dist=%.2f", back_dist);
        publishStop();
        changeState(ParkingState::STRAIGHTEN);
        return;
    }

    publishCmd(reverse_speed_, entry_steer_);
}

void ParkingNode::handleStraighten() {
    // 후진하면서 좌우 벽 거리로 중앙 정렬
    double back_dist = getMinDistanceInWindow(back_scan_min_deg_, back_scan_max_deg_);
    double left_dist = getMeanDistanceInWindow(left_scan_min_deg_, left_scan_max_deg_);
    double right_dist = getMeanDistanceInWindow(right_scan_min_deg_, right_scan_max_deg_);

    // 뒤 벽 도달 → HOLD
    if (back_dist > 0.0 && back_dist < park_depth_) {
        RCLCPP_INFO(this->get_logger(),
            "Parked! back=%.2f, left=%.2f, right=%.2f", back_dist, left_dist, right_dist);
        changeState(ParkingState::HOLD);
        return;
    }

    // 비상 정지
    if (back_dist > 0.0 && back_dist < emergency_stop_dist_) {
        RCLCPP_WARN(this->get_logger(), "Emergency stop in straighten!");
        changeState(ParkingState::HOLD);
        return;
    }

    // 좌우 벽 거리 차이로 P 제어 조향
    double steer = 0.0;
    if (left_dist > 0.0 && right_dist > 0.0) {
        double error = left_dist - right_dist;  // 양수 = 좌측이 더 멀다 → 좌로 조향
        steer = clamp(straighten_steer_kp_ * error, -0.3, 0.3);
    }

    publishCmd(straighten_speed_, steer);
}

void ParkingNode::handleHold() {
    publishStop();

    double elapsed = (this->now() - state_enter_time_).seconds();
    if (elapsed >= hold_duration_) {
        RCLCPP_INFO(this->get_logger(), "Hold complete (%.1f sec), exiting", elapsed);
        changeState(ParkingState::EXIT_FORWARD);
    }
}

void ParkingNode::handleExitForward() {
    // 전진 + 조향으로 주차 공간 탈출
    double elapsed = (this->now() - state_enter_time_).seconds();

    if (elapsed >= exit_duration_) {
        RCLCPP_INFO(this->get_logger(), "Exit maneuver complete, seeking OUT line");
        changeState(ParkingState::SEEK_OUT);
        return;
    }

    publishCmd(exit_speed_, exit_steer_);
}

void ParkingNode::handleSeekOut() {
    // OUT 라인 감지까지 직진
    if (out_line_detected_) {
        RCLCPP_INFO(this->get_logger(), "OUT line detected! Parking mission complete.");
        changeState(ParkingState::DONE);
        return;
    }

    publishCmd(seek_speed_, 0.0);
}

// ════════════════════════════════════════════════════
// LiDAR utilities
// ════════════════════════════════════════════════════

double ParkingNode::normalizeAngle(double rad) {
    constexpr double TWO_PI = 2.0 * M_PI;
    while (rad < 0.0) rad += TWO_PI;
    while (rad >= TWO_PI) rad -= TWO_PI;
    return rad;
}

bool ParkingNode::angleInWindow(double angle, double start, double end) {
    if (start <= end) {
        return angle >= start && angle <= end;
    }
    return angle >= start || angle <= end;
}

double ParkingNode::getMinDistanceInWindow(double angle_min_deg, double angle_max_deg) const {
    if (!latest_scan_) return -1.0;

    double start_rad = normalizeAngle(angle_min_deg * M_PI / 180.0);
    double end_rad = normalizeAngle(angle_max_deg * M_PI / 180.0);
    double min_dist = -1.0;

    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
        double d = latest_scan_->ranges[i];
        if (!std::isfinite(d) || d <= 0.0) continue;

        double angle = normalizeAngle(latest_scan_->angle_min + i * latest_scan_->angle_increment);
        if (!angleInWindow(angle, start_rad, end_rad)) continue;

        if (min_dist < 0.0 || d < min_dist) {
            min_dist = d;
        }
    }
    return min_dist;
}

double ParkingNode::getMeanDistanceInWindow(double angle_min_deg, double angle_max_deg) const {
    if (!latest_scan_) return -1.0;

    double start_rad = normalizeAngle(angle_min_deg * M_PI / 180.0);
    double end_rad = normalizeAngle(angle_max_deg * M_PI / 180.0);
    double sum = 0.0;
    int count = 0;

    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
        double d = latest_scan_->ranges[i];
        if (!std::isfinite(d) || d <= 0.0) continue;

        double angle = normalizeAngle(latest_scan_->angle_min + i * latest_scan_->angle_increment);
        if (!angleInWindow(angle, start_rad, end_rad)) continue;

        sum += d;
        count++;
    }
    return count > 0 ? sum / count : -1.0;
}

int ParkingNode::countPointsInWindow(double angle_min_deg, double angle_max_deg,
                                      double min_range, double max_range) const {
    if (!latest_scan_) return 0;

    double start_rad = normalizeAngle(angle_min_deg * M_PI / 180.0);
    double end_rad = normalizeAngle(angle_max_deg * M_PI / 180.0);
    int count = 0;

    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
        double d = latest_scan_->ranges[i];
        if (!std::isfinite(d) || d <= 0.0) continue;

        double angle = normalizeAngle(latest_scan_->angle_min + i * latest_scan_->angle_increment);
        if (!angleInWindow(angle, start_rad, end_rad)) continue;

        if (d >= min_range && d <= max_range) {
            count++;
        }
    }
    return count;
}

// ════════════════════════════════════════════════════
// Command helpers
// ════════════════════════════════════════════════════

void ParkingNode::publishCmd(double speed, double steer) {
    ackermann_msgs::msg::AckermannDrive msg;
    msg.speed = static_cast<float>(speed);
    msg.steering_angle = static_cast<float>(steer);
    cmd_pub_->publish(msg);
}

void ParkingNode::publishStop() {
    publishCmd(0.0, 0.0);
}

void ParkingNode::changeState(ParkingState new_state) {
    RCLCPP_INFO(this->get_logger(), "State: %s → %s",
                stateToString(state_), stateToString(new_state));
    state_ = new_state;
    state_enter_time_ = this->now();

    // Reset state-specific variables
    if (new_state == ParkingState::SEARCH) {
        wall_seen_ = false;
        gap_point_count_ = 0;
    } else if (new_state == ParkingState::ALIGN) {
        align_stopped_ = false;
    } else if (new_state == ParkingState::SEEK_OUT) {
        out_line_detected_ = false;
    }
}

const char* ParkingNode::stateToString(ParkingState s) const {
    switch (s) {
        case ParkingState::IDLE:          return "IDLE";
        case ParkingState::SEARCH:        return "SEARCH";
        case ParkingState::ALIGN:         return "ALIGN";
        case ParkingState::REVERSE_ENTER: return "REVERSE_ENTER";
        case ParkingState::STRAIGHTEN:    return "STRAIGHTEN";
        case ParkingState::HOLD:          return "HOLD";
        case ParkingState::EXIT_FORWARD:  return "EXIT_FORWARD";
        case ParkingState::SEEK_OUT:      return "SEEK_OUT";
        case ParkingState::DONE:          return "DONE";
        default:                          return "UNKNOWN";
    }
}

}  // namespace decision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<decision::ParkingNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("parking_node"),
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
