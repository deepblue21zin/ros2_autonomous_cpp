#!/usr/bin/env python3
"""HSV + YOLO 결합 차선 주행 노드.

HSV 차선 검출로 빠른 차선 추종을 수행하고,
YOLO 장애물 검출로 장애물 회피를 수행합니다.

상태 기계:
  - NORMAL: HSV 차선 중심 추종
  - AVOIDING_LEFT: 장애물 감지 → 왼쪽으로 회피
  - AVOIDING_RIGHT: 장애물 감지 → 오른쪽으로 회피
  - RETURNING: 회피 완료 → 차선 중앙 복귀

입력:
  - /lane/center_offset (Float32): HSV 차선 중심 offset (-1.0 ~ 1.0)
  - /perception/obstacles (BoundingBoxArray 또는 Image): YOLO 장애물 정보
  - /perception/drivable_mask (Image): YOLO 주행 가능 영역 마스크

출력:
  - /decision/cmd (AckermannDrive): 조향각 + 속도 명령
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
    NORMAL = "normal"              # 정상 주행 (HSV 차선 추종)
    AVOIDING_LEFT = "avoiding_left"   # 장애물 회피 (왼쪽)
    AVOIDING_RIGHT = "avoiding_right"  # 장애물 회피 (오른쪽)
    RETURNING = "returning"        # 차선 중앙 복귀


class HybridLaneDriveNode(Node):
    """HSV + YOLO 결합 차선 주행 노드."""

    def __init__(self) -> None:
        super().__init__('hybrid_lane_drive_node')

        # ── 파라미터 선언 ──
        self.declare_parameter('cruise_speed_mps', 0.3)
        self.declare_parameter('slow_speed_mps', 0.2)
        self.declare_parameter('max_steer_rad', 0.45)
        self.declare_parameter('kp', 2.5)
        self.declare_parameter('kd', 0.6)
        self.declare_parameter('avoid_offset', 0.4)  # 회피 시 차선 변경 offset
        self.declare_parameter('return_threshold', 0.15)  # 중앙 복귀 판단 threshold
        self.declare_parameter('obstacle_detect_threshold', 0.3)  # 장애물 감지 임계값
        self.declare_parameter('use_traffic_light', True)
        self.declare_parameter('control_hz', 20.0)

        # ── 파라미터 로드 ──
        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.slow_speed = self.get_parameter('slow_speed_mps').value
        self.max_steer = self.get_parameter('max_steer_rad').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.avoid_offset = self.get_parameter('avoid_offset').value
        self.return_threshold = self.get_parameter('return_threshold').value
        self.obstacle_threshold = self.get_parameter('obstacle_detect_threshold').value
        self.use_traffic_light = self.get_parameter('use_traffic_light').value
        control_hz = self.get_parameter('control_hz').value

        # ── 내부 상태 ──
        self.bridge = CvBridge()
        self.state = DriveState.NORMAL

        # HSV 차선 중심 offset
        self.lane_offset: float | None = None
        self.last_lane_time: Time | None = None

        # YOLO 장애물 정보
        self.has_obstacle = False
        self.obstacle_side = "none"  # "left", "right", "none"
        self.last_obstacle_time: Time | None = None

        # 신호등
        self.traffic_light_state = "unknown"

        # PID 제어
        self.prev_error = 0.0
        self.integral = 0.0

        # ── 퍼블리셔 ──
        self.cmd_pub = self.create_publisher(AckermannDrive, '/decision/cmd', 1)
        self.state_pub = self.create_publisher(String, '/decision/state', 1)

        # ── 서브스크라이버 ──
        self.create_subscription(
            Float32, '/lane/center_offset', self._lane_offset_cb, 1)
        self.create_subscription(
            Image, '/perception/drivable_mask', self._drivable_mask_cb, 1)
        self.create_subscription(
            String, '/perception/traffic_light_state', self._traffic_light_cb, 1)

        # ── 제어 타이머 ──
        period = 1.0 / max(control_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"[hybrid_lane_drive] 초기화 완료: "
            f"cruise={self.cruise_speed}m/s, kp={self.kp}, kd={self.kd}"
        )

    # ──────────────────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────────────────

    def _lane_offset_cb(self, msg: Float32) -> None:
        """HSV 차선 중심 offset 수신."""
        self.lane_offset = msg.data
        self.last_lane_time = self.get_clock().now()

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
        self.traffic_light_state = msg.data

    # ──────────────────────────────────────────────────────────────
    # 장애물 검출
    # ──────────────────────────────────────────────────────────────

    def _detect_obstacle_from_mask(self, mask: np.ndarray) -> None:
        """YOLO drivable_mask에서 장애물 검출.

        마스크의 중앙 영역이 막혀있으면 장애물로 판단.
        좌우 중 어느 쪽이 더 열려있는지 확인.
        """
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

        # 중앙이 막혔으면 장애물
        if center_ratio < self.obstacle_threshold:
            self.has_obstacle = True
            # 좌/우 중 더 열린 쪽으로 회피
            if left_ratio > right_ratio:
                self.obstacle_side = "left"
            else:
                self.obstacle_side = "right"
        else:
            self.has_obstacle = False
            self.obstacle_side = "none"

    # ──────────────────────────────────────────────────────────────
    # 상태 기계
    # ──────────────────────────────────────────────────────────────

    def _update_state(self) -> None:
        """주행 상태 업데이트."""
        if self.state == DriveState.NORMAL:
            # 정상 주행 중 장애물 감지 → 회피 모드
            if self.has_obstacle:
                if self.obstacle_side == "left":
                    self.state = DriveState.AVOIDING_RIGHT  # 오른쪽으로 회피
                else:
                    self.state = DriveState.AVOIDING_LEFT   # 왼쪽으로 회피
                self.get_logger().info(
                    f"[hybrid_lane_drive] 장애물 감지 → {self.state.value}")

        elif self.state in (DriveState.AVOIDING_LEFT, DriveState.AVOIDING_RIGHT):
            # 회피 중 장애물 사라짐 → 복귀 모드
            if not self.has_obstacle:
                self.state = DriveState.RETURNING
                self.get_logger().info(
                    f"[hybrid_lane_drive] 장애물 통과 → RETURNING")

        elif self.state == DriveState.RETURNING:
            # 복귀 중 차선 중앙 도달 → 정상 모드
            if self.lane_offset is not None and abs(self.lane_offset) < self.return_threshold:
                self.state = DriveState.NORMAL
                self.get_logger().info(
                    f"[hybrid_lane_drive] 차선 복귀 완료 → NORMAL")

    # ──────────────────────────────────────────────────────────────
    # 목표 계산
    # ──────────────────────────────────────────────────────────────

    def _compute_target_offset(self) -> float | None:
        """상태에 따른 목표 offset 계산."""
        if self.lane_offset is None:
            return None

        if self.state == DriveState.NORMAL:
            # 정상: HSV 차선 중심 추종
            return self.lane_offset

        elif self.state == DriveState.AVOIDING_LEFT:
            # 왼쪽 회피: 차선 중심 + 회피 offset (왼쪽으로 이동)
            return self.lane_offset - self.avoid_offset

        elif self.state == DriveState.AVOIDING_RIGHT:
            # 오른쪽 회피: 차선 중심 + 회피 offset (오른쪽으로 이동)
            return self.lane_offset + self.avoid_offset

        elif self.state == DriveState.RETURNING:
            # 복귀: 차선 중심으로
            return self.lane_offset

        return self.lane_offset

    # ──────────────────────────────────────────────────────────────
    # PID 제어
    # ──────────────────────────────────────────────────────────────

    def _compute_steering(self, target_offset: float, dt: float) -> float:
        """PID 제어로 조향각 계산.

        Args:
            target_offset: 목표 offset (-1.0 ~ 1.0, 왼쪽이 음수)
            dt: 시간 간격 (초)

        Returns:
            조향각 (라디안, 왼쪽이 양수)
        """
        # 오류: offset이 양수면 오른쪽으로 치우침 → 왼쪽으로 회전 필요
        error = -target_offset

        # PID 계산
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        steer = self.kp * error + self.kd * derivative

        self.prev_error = error

        # 조향각 제한
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

        # 1. HSV 차선 정보 타임아웃 체크
        if self.lane_offset is None or self.last_lane_time is None:
            self.get_logger().warn(
                "STOP: no lane offset",
                throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(cmd)
            return

        elapsed = (now - self.last_lane_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.get_logger().warn(
                f"STOP: lane timeout ({elapsed:.1f}s)",
                throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(cmd)
            return

        # 2. 신호등 체크
        if self.use_traffic_light and self.traffic_light_state == "red":
            self.get_logger().info(
                "STOP: red light",
                throttle_duration_sec=1.0
            )
            self.cmd_pub.publish(cmd)
            return

        # 3. 상태 업데이트
        self._update_state()

        # 4. 목표 offset 계산
        target_offset = self._compute_target_offset()
        if target_offset is None:
            self.get_logger().warn(
                "STOP: no target offset",
                throttle_duration_sec=1.0
            )
            self.cmd_pub.publish(cmd)
            return

        # 5. PID 조향각 계산
        dt = 1.0 / 20.0
        steer = self._compute_steering(target_offset, dt)

        # 6. 속도 계산
        if self.state in (DriveState.AVOIDING_LEFT, DriveState.AVOIDING_RIGHT):
            speed = self.slow_speed  # 회피 시 감속
        else:
            speed = self.cruise_speed

        # 7. 명령 발행
        cmd.speed = speed
        cmd.steering_angle = steer
        self.cmd_pub.publish(cmd)

        # 8. 상태 발행
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        self.get_logger().info(
            f"[{self.state.value}] lane={self.lane_offset:+.2f}, "
            f"target={target_offset:+.2f}, steer={steer:+.3f}rad, speed={speed:.2f}m/s",
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
