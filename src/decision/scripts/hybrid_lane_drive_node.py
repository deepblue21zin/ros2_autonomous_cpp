#!/usr/bin/env python3
"""HSV + YOLO 결합 차선 주행 노드 (차선 변경 지원).

HSV 차선 검출로 빠른 차선 추종을 수행하고,
YOLO로 실선/점선 판별 + 장애물 검출을 수행합니다.

상태 기계:
  - NORMAL: HSV 차선 중심 추종
  - LANE_CHANGE: 고정 조향으로 점선을 넘어 차선 변경
  - STABILIZE: HSV 재잠금 대기

입력:
  - /lane/center_offset (Float32): HSV 차선 중심 offset (-1.0 ~ 1.0)
  - /lane/steering_angle (Float32): HSV 차선 조향 (-1.0 ~ 1.0, 정규화)
  - /perception/drivable_mask (Image): YOLO 주행 가능 영역 마스크
  - /perception/lane_line_type (String): "left:TYPE,right:TYPE" (solid/dashed/none)
  - /perception/traffic_light_state (String): 신호등 상태

출력:
  - /decision/cmd (AckermannDrive): 조향각 + 속도 명령
  - /decision/state (String): 현재 상태
"""

from __future__ import annotations

from enum import Enum
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge


class DriveState(Enum):
    """주행 상태."""
    NORMAL = "normal"                # 정상 주행 (HSV 차선 추종)
    LANE_CHANGE = "lane_change"      # 차선 변경 중 (고정 조향)
    STABILIZE = "stabilize"          # HSV 재잠금 대기


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


class HybridLaneDriveNode(Node):
    """HSV + YOLO 결합 차선 주행 노드 (차선 변경 지원)."""

    def __init__(self) -> None:
        super().__init__('hybrid_lane_drive_node')

        # ── 파라미터 선언 ──
        # 속도
        self.declare_parameter('cruise_speed_mps', 0.55)
        self.declare_parameter('slow_speed_mps', 0.38)
        # 조향
        self.declare_parameter('max_steer_rad', 0.45)
        self.declare_parameter('kp', 1.8)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('steer_rate_limit_rad_per_sec', 2.0)
        self.declare_parameter('use_lane_steering_input', True)
        self.declare_parameter('lane_timeout_sec', 0.5)
        # 장애물 검출
        self.declare_parameter('obstacle_detect_threshold', 0.22)
        self.declare_parameter('obstacle_on_frames', 3)
        self.declare_parameter('obstacle_off_frames', 5)
        # 차선 변경
        self.declare_parameter('enable_lane_change', False)
        self.declare_parameter('lane_change_steer_rad', 0.4)
        self.declare_parameter('lane_change_duration_sec', 1.5)
        self.declare_parameter('stabilize_duration_sec', 0.5)
        # 신호등
        self.declare_parameter('use_traffic_light', True)
        # 제어
        self.declare_parameter('control_hz', 20.0)

        # ── 파라미터 로드 ──
        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.slow_speed = self.get_parameter('slow_speed_mps').value
        self.max_steer = self.get_parameter('max_steer_rad').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.steer_rate_limit = self.get_parameter('steer_rate_limit_rad_per_sec').value
        self.use_lane_steering_input = self.get_parameter('use_lane_steering_input').value
        self.lane_timeout = self.get_parameter('lane_timeout_sec').value
        self.obstacle_threshold = self.get_parameter('obstacle_detect_threshold').value
        self.obstacle_on_frames = int(self.get_parameter('obstacle_on_frames').value)
        self.obstacle_off_frames = int(self.get_parameter('obstacle_off_frames').value)
        self.enable_lane_change = self.get_parameter('enable_lane_change').value
        self.lane_change_steer = self.get_parameter('lane_change_steer_rad').value
        self.lane_change_duration = self.get_parameter('lane_change_duration_sec').value
        self.stabilize_duration = self.get_parameter('stabilize_duration_sec').value
        self.use_traffic_light = self.get_parameter('use_traffic_light').value
        control_hz = self.get_parameter('control_hz').value

        # ── 내부 상태 ──
        self.bridge = CvBridge()
        self.state = DriveState.NORMAL

        # HSV 차선 중심 offset
        self.lane_offset: float | None = None
        self.last_lane_time: Time | None = None
        # lane_tracking 계산 조향 (-1.0~1.0)
        self.lane_steer_norm: float | None = None
        self.last_lane_steer_time: Time | None = None

        # YOLO 장애물 정보
        self.has_obstacle = False
        self.obstacle_side = "none"  # "left", "right", "none"
        self.last_obstacle_time: Time | None = None
        self.obstacle_detect_count = 0
        self.obstacle_clear_count = 0

        # YOLO 차선 타입 정보
        self.left_line_type = "none"   # "solid", "dashed", "none"
        self.right_line_type = "none"

        # 신호등
        self.traffic_light_state = "unknown"

        # 차선 변경 상태
        self.change_direction = 0         # +1=왼쪽, -1=오른쪽
        self.state_start_time: Time | None = None

        # PD 제어
        self.prev_error = 0.0
        self.prev_cmd_steer = 0.0
        self.last_control_time = self.get_clock().now()

        # ── 퍼블리셔 ──
        self.cmd_pub = self.create_publisher(AckermannDrive, '/decision/cmd', 1)
        self.state_pub = self.create_publisher(String, '/decision/state', 1)

        # ── 서브스크라이버 ──
        self.create_subscription(
            Float32, '/lane/center_offset', self._lane_offset_cb, 1)
        self.create_subscription(
            Float32, '/lane/steering_angle', self._lane_steer_cb, 1)
        self.create_subscription(
            Image, '/perception/drivable_mask', self._drivable_mask_cb, 1)
        self.create_subscription(
            String, '/perception/traffic_light_state', self._traffic_light_cb, 1)
        self.create_subscription(
            String, '/perception/lane_line_type', self._lane_line_type_cb, 1)

        # ── 제어 타이머 ──
        period = 1.0 / max(control_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"[hybrid_lane_drive] 초기화 완료: "
            f"cruise={self.cruise_speed}m/s, slow={self.slow_speed}m/s, "
            f"lane_steer_input={self.use_lane_steering_input}, "
            f"lane_change={self.enable_lane_change}"
        )

    # ──────────────────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────────────────

    def _lane_offset_cb(self, msg: Float32) -> None:
        """HSV 차선 중심 offset 수신."""
        self.lane_offset = msg.data
        self.last_lane_time = self.get_clock().now()

    def _lane_steer_cb(self, msg: Float32) -> None:
        """lane_tracking 조향 수신 (-1.0~1.0)."""
        self.lane_steer_norm = float(_clamp(msg.data, -1.0, 1.0))
        self.last_lane_steer_time = self.get_clock().now()

    def _drivable_mask_cb(self, msg: Image) -> None:
        """YOLO drivable_mask 수신 → 장애물 검출."""
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self._detect_obstacle_from_mask(mask)
            self.last_obstacle_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"마스크 변환 실패: {e}")

    def _traffic_light_cb(self, msg: String) -> None:
        """신호등 상태 수신."""
        state = msg.data.strip().lower()
        state_alias = {
            "red_light": "red",
            "traffic_light_red": "red",
            "green_light": "green",
            "traffic_light_green": "green",
            "yellow_light": "yellow",
            "traffic_light_yellow": "yellow",
        }
        self.traffic_light_state = state_alias.get(state, state)

    def _lane_line_type_cb(self, msg: String) -> None:
        """차선 타입 수신. 형식: 'left:TYPE,right:TYPE'"""
        try:
            parts = msg.data.split(",")
            for part in parts:
                side, line_type = part.strip().split(":")
                if side == "left":
                    self.left_line_type = line_type
                elif side == "right":
                    self.right_line_type = line_type
        except Exception as e:
            self.get_logger().error(f"차선 타입 파싱 실패: {e}")

    # ──────────────────────────────────────────────────────────────
    # 장애물 검출
    # ──────────────────────────────────────────────────────────────

    def _detect_obstacle_from_mask(self, mask: np.ndarray) -> None:
        """YOLO drivable_mask에서 장애물 검출."""
        h, w = mask.shape[:2]

        # ROI: 중앙 60% 영역 (차선 앞쪽)
        roi_y1 = int(h * 0.4)
        roi_y2 = int(h * 0.8)
        roi = mask[roi_y1:roi_y2, :]

        # 좌/중/우 영역으로 나누기
        third = w // 3
        left_area = roi[:, :third]
        center_area = roi[:, third:2*third]
        right_area = roi[:, 2*third:]

        # 각 영역의 주행 가능 비율
        left_ratio = np.sum(left_area > 127) / max(left_area.size, 1)
        center_ratio = np.sum(center_area > 127) / max(center_area.size, 1)
        right_ratio = np.sum(right_area > 127) / max(right_area.size, 1)

        blocked = center_ratio < self.obstacle_threshold
        if blocked:
            self.obstacle_detect_count += 1
            self.obstacle_clear_count = 0
            if not self.has_obstacle and self.obstacle_detect_count >= self.obstacle_on_frames:
                self.has_obstacle = True
                if left_ratio > right_ratio:
                    self.obstacle_side = "left"
                else:
                    self.obstacle_side = "right"
        else:
            self.obstacle_clear_count += 1
            self.obstacle_detect_count = 0
            if self.has_obstacle and self.obstacle_clear_count >= self.obstacle_off_frames:
                self.has_obstacle = False
                self.obstacle_side = "none"

    # ──────────────────────────────────────────────────────────────
    # 차선 변경 방향 결정
    # ──────────────────────────────────────────────────────────────

    def _decide_lane_change_direction(self) -> int:
        """점선 방향으로 차선 변경 방향 결정.

        Returns:
            +1: 왼쪽으로 변경 (좌측 점선)
            -1: 오른쪽으로 변경 (우측 점선)
             0: 변경 불가 (점선 없음)
        """
        left_dashed = self.left_line_type == "dashed"
        right_dashed = self.right_line_type == "dashed"

        if left_dashed and not right_dashed:
            return 1   # 좌측 점선 → 왼쪽으로
        elif right_dashed and not left_dashed:
            return -1  # 우측 점선 → 오른쪽으로
        elif left_dashed and right_dashed:
            # 양쪽 점선 → 장애물 반대쪽으로
            if self.obstacle_side == "right":
                return 1   # 장애물 오른쪽 → 왼쪽으로
            else:
                return -1  # 장애물 왼쪽 → 오른쪽으로
        return 0  # 점선 없음

    # ──────────────────────────────────────────────────────────────
    # 상태 기계
    # ──────────────────────────────────────────────────────────────

    def _get_elapsed_in_state(self) -> float:
        """현재 상태 진입 후 경과 시간(초)."""
        if self.state_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

    def _transition_to(self, new_state: DriveState) -> None:
        """상태 전환."""
        old = self.state.value
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"[상태 전환] {old} → {new_state.value}")

    def _update_state(self) -> None:
        """주행 상태 업데이트."""
        if not self.enable_lane_change:
            if self.state != DriveState.NORMAL:
                self._transition_to(DriveState.NORMAL)
            return

        elapsed = self._get_elapsed_in_state()

        if self.state == DriveState.NORMAL:
            # 장애물 감지 → 점선 확인 → 차선 변경
            if self.has_obstacle:
                direction = self._decide_lane_change_direction()
                if direction != 0:
                    self.change_direction = direction
                    self._transition_to(DriveState.LANE_CHANGE)
                else:
                    self.get_logger().warn(
                        "장애물 감지했으나 점선 없음 → 감속 유지",
                        throttle_duration_sec=1.0)

        elif self.state == DriveState.LANE_CHANGE:
            # 차선 변경 완료 (시간 기반)
            if elapsed >= self.lane_change_duration:
                self._transition_to(DriveState.STABILIZE)

        elif self.state == DriveState.STABILIZE:
            # 안정화 완료 → NORMAL
            if elapsed >= self.stabilize_duration:
                self._transition_to(DriveState.NORMAL)

    # ──────────────────────────────────────────────────────────────
    # PD 제어
    # ──────────────────────────────────────────────────────────────

    def _compute_steering(self, target_offset: float, dt: float) -> float:
        """PD 제어로 조향각 계산."""
        error = -target_offset
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        steer = self.kp * error + self.kd * derivative
        self.prev_error = error
        steer = np.clip(steer, -self.max_steer, self.max_steer)
        return steer

    # ──────────────────────────────────────────────────────────────
    # 메인 제어 루프
    # ──────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """주기적 제어 루프."""
        cmd = AckermannDrive()
        cmd.speed = 0.0
        cmd.steering_angle = 0.0

        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        dt = _clamp(dt, 0.001, 0.2)

        # 1. HSV 차선 정보 타임아웃 체크
        if self.state != DriveState.LANE_CHANGE:
            if self.use_lane_steering_input:
                if self.lane_steer_norm is None or self.last_lane_steer_time is None:
                    self.get_logger().warn(
                        "STOP: no lane steering",
                        throttle_duration_sec=2.0)
                    self.prev_cmd_steer = 0.0
                    self.cmd_pub.publish(cmd)
                    return
                elapsed = (now - self.last_lane_steer_time).nanoseconds / 1e9
                if elapsed > self.lane_timeout:
                    self.get_logger().warn(
                        f"STOP: lane steering timeout ({elapsed:.1f}s)",
                        throttle_duration_sec=2.0)
                    self.prev_cmd_steer = 0.0
                    self.cmd_pub.publish(cmd)
                    return
            else:
                if self.lane_offset is None or self.last_lane_time is None:
                    self.get_logger().warn(
                        "STOP: no lane offset",
                        throttle_duration_sec=2.0)
                    self.prev_cmd_steer = 0.0
                    self.cmd_pub.publish(cmd)
                    return

                elapsed = (now - self.last_lane_time).nanoseconds / 1e9
                if elapsed > self.lane_timeout:
                    self.get_logger().warn(
                        f"STOP: lane timeout ({elapsed:.1f}s)",
                        throttle_duration_sec=2.0)
                    self.prev_cmd_steer = 0.0
                    self.cmd_pub.publish(cmd)
                    return

        # 2. 신호등 체크
        if self.use_traffic_light and self.traffic_light_state == "red":
            self.get_logger().info(
                "STOP: red light",
                throttle_duration_sec=1.0)
            self.prev_cmd_steer = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 3. 상태 업데이트
        self._update_state()

        # 4. 상태별 조향 + 속도 계산

        if self.state == DriveState.NORMAL:
            if self.use_lane_steering_input and self.lane_steer_norm is not None:
                steer = self.lane_steer_norm * self.max_steer
            elif self.lane_offset is not None:
                steer = self._compute_steering(self.lane_offset, dt)
            else:
                steer = 0.0
            # 장애물 있으면 감속
            if self.has_obstacle:
                speed = self.slow_speed
            else:
                speed = self.cruise_speed

        elif self.state == DriveState.LANE_CHANGE:
            # 고정 조향으로 차선 변경
            steer = self.lane_change_steer * self.change_direction
            speed = self.slow_speed
            self.prev_error = 0.0  # PD 리셋

        elif self.state == DriveState.STABILIZE:
            # HSV 차선 추종으로 안정화 (감속)
            if self.lane_offset is not None:
                steer = self._compute_steering(self.lane_offset, dt)
            else:
                steer = 0.0
            speed = self.slow_speed

        else:
            steer = 0.0
            speed = 0.0

        # 5. 조향 변화율 제한 (track 모드와 유사한 안정화)
        max_delta = self.steer_rate_limit * dt
        steer = _clamp(steer, self.prev_cmd_steer - max_delta, self.prev_cmd_steer + max_delta)
        steer = _clamp(steer, -self.max_steer, self.max_steer)
        self.prev_cmd_steer = steer

        # 5. 명령 발행
        cmd.speed = speed
        cmd.steering_angle = float(steer)
        self.cmd_pub.publish(cmd)

        # 6. 상태 발행
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        # 로그
        lane_off_str = f"{self.lane_offset:+.2f}" if self.lane_offset is not None else "N/A"
        lane_steer_str = f"{self.lane_steer_norm:+.2f}" if self.lane_steer_norm is not None else "N/A"
        self.get_logger().info(
            f"[{self.state.value}] offset={lane_off_str}, lane_steer={lane_steer_str}, "
            f"steer={steer:+.3f}rad, speed={speed:.2f}m/s, "
            f"lines=L:{self.left_line_type}/R:{self.right_line_type}, "
            f"obs={self.has_obstacle}",
            throttle_duration_sec=0.5
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridLaneDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
