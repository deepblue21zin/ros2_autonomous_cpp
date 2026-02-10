#!/usr/bin/env python3
"""HSV + YOLO 결합 차선 주행 노드 (차선 변경 지원).

HSV 차선 검출로 빠른 차선 추종을 수행하고,
YOLO로 실선/점선 판별 + 장애물 검출을 수행합니다.

상태 기계:
  - NORMAL: HSV 차선 중심 추종
  - LANE_CHANGE: 고정 조향으로 점선을 넘어 차선 변경
  - STABILIZE: HSV 재잠금 대기
  - LANE_RETURN: 장애물 통과 후 원래 차선으로 복귀

입력:
  - /lane/center_offset (Float32): HSV 차선 중심 offset (-1.0 ~ 1.0)
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
    LANE_RETURN = "lane_return"      # 원래 차선 복귀 중 (고정 조향)


class HybridLaneDriveNode(Node):
    """HSV + YOLO 결합 차선 주행 노드 (차선 변경 지원)."""

    def __init__(self) -> None:
        super().__init__('hybrid_lane_drive_node')

        # ── 파라미터 선언 ──
        # 속도
        self.declare_parameter('cruise_speed_mps', 0.3)
        self.declare_parameter('slow_speed_mps', 0.2)
        # 조향
        self.declare_parameter('max_steer_rad', 0.45)
        self.declare_parameter('kp', 2.5)
        self.declare_parameter('kd', 0.6)
        # 장애물 검출
        self.declare_parameter('obstacle_detect_threshold', 0.3)
        # 차선 변경
        self.declare_parameter('lane_change_steer_rad', 0.4)
        self.declare_parameter('lane_change_duration_sec', 1.5)
        self.declare_parameter('stabilize_duration_sec', 0.5)
        self.declare_parameter('lane_return_delay_sec', 2.0)
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
        self.obstacle_threshold = self.get_parameter('obstacle_detect_threshold').value
        self.lane_change_steer = self.get_parameter('lane_change_steer_rad').value
        self.lane_change_duration = self.get_parameter('lane_change_duration_sec').value
        self.stabilize_duration = self.get_parameter('stabilize_duration_sec').value
        self.lane_return_delay = self.get_parameter('lane_return_delay_sec').value
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

        # YOLO 차선 타입 정보
        self.left_line_type = "none"   # "solid", "dashed", "none"
        self.right_line_type = "none"

        # 신호등
        self.traffic_light_state = "unknown"

        # 차선 변경 상태
        self.in_changed_lane = False      # 현재 차선 변경된 상태인지
        self.change_direction = 0         # +1=왼쪽, -1=오른쪽
        self.state_start_time: Time | None = None
        self.obstacle_clear_time: Time | None = None  # 장애물 사라진 시간

        # PD 제어
        self.prev_error = 0.0

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
        self.create_subscription(
            String, '/perception/lane_line_type', self._lane_line_type_cb, 1)

        # ── 제어 타이머 ──
        period = 1.0 / max(control_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"[hybrid_lane_drive] 초기화 완료: "
            f"cruise={self.cruise_speed}m/s, kp={self.kp}, kd={self.kd}, "
            f"lane_change_steer={self.lane_change_steer}rad, "
            f"lane_change_dur={self.lane_change_duration}s"
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
        """YOLO drivable_mask에서 장애물 검출.

        마스크의 중앙 영역이 막혀있으면 장애물로 판단.
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
        prev_obstacle = self.has_obstacle
        if center_ratio < self.obstacle_threshold:
            self.has_obstacle = True
            if left_ratio > right_ratio:
                self.obstacle_side = "left"
            else:
                self.obstacle_side = "right"
        else:
            self.has_obstacle = False
            self.obstacle_side = "none"

        # 장애물이 방금 사라졌으면 시간 기록
        if prev_obstacle and not self.has_obstacle:
            self.obstacle_clear_time = self.get_clock().now()

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
        now = self.get_clock().now()
        elapsed = self._get_elapsed_in_state()

        if self.state == DriveState.NORMAL:
            if not self.in_changed_lane:
                # 정상 주행 중 장애물 감지 → 점선 확인 후 차선 변경
                if self.has_obstacle:
                    direction = self._decide_lane_change_direction()
                    if direction != 0:
                        self.change_direction = direction
                        self._transition_to(DriveState.LANE_CHANGE)
                    else:
                        self.get_logger().warn(
                            "장애물 감지했으나 점선 없음 → 감속 유지",
                            throttle_duration_sec=1.0)
            else:
                # 차선 변경 상태에서 장애물 통과 후 → 복귀 대기
                if not self.has_obstacle and self.obstacle_clear_time is not None:
                    clear_elapsed = (now - self.obstacle_clear_time).nanoseconds / 1e9
                    if clear_elapsed >= self.lane_return_delay:
                        # 점선 확인 후 복귀
                        direction = self._decide_lane_change_direction()
                        if direction != 0:
                            self.change_direction = direction
                            self._transition_to(DriveState.LANE_RETURN)
                        else:
                            # 점선 없어도 복귀 시도 (반대 방향)
                            self.change_direction = -self.change_direction
                            self._transition_to(DriveState.LANE_RETURN)

        elif self.state == DriveState.LANE_CHANGE:
            # 차선 변경 완료 (시간 기반)
            if elapsed >= self.lane_change_duration:
                self.in_changed_lane = True
                self._transition_to(DriveState.STABILIZE)

        elif self.state == DriveState.STABILIZE:
            # 안정화 완료 → NORMAL
            if elapsed >= self.stabilize_duration:
                self._transition_to(DriveState.NORMAL)

        elif self.state == DriveState.LANE_RETURN:
            # 복귀 완료 (시간 기반)
            if elapsed >= self.lane_change_duration:
                self.in_changed_lane = False
                self.obstacle_clear_time = None
                self._transition_to(DriveState.STABILIZE)

    # ──────────────────────────────────────────────────────────────
    # PD 제어
    # ──────────────────────────────────────────────────────────────

    def _compute_steering(self, target_offset: float, dt: float) -> float:
        """PD 제어로 조향각 계산.

        Args:
            target_offset: 목표 offset (-1.0 ~ 1.0, 왼쪽이 음수)
            dt: 시간 간격 (초)

        Returns:
            조향각 (라디안, 왼쪽이 양수)
        """
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

        # 1. HSV 차선 정보 타임아웃 체크 (LANE_CHANGE/LANE_RETURN 중에는 무시)
        if self.state not in (DriveState.LANE_CHANGE, DriveState.LANE_RETURN):
            if self.lane_offset is None or self.last_lane_time is None:
                self.get_logger().warn(
                    "STOP: no lane offset",
                    throttle_duration_sec=2.0)
                self.cmd_pub.publish(cmd)
                return

            elapsed = (now - self.last_lane_time).nanoseconds / 1e9
            if elapsed > 1.0:
                self.get_logger().warn(
                    f"STOP: lane timeout ({elapsed:.1f}s)",
                    throttle_duration_sec=2.0)
                self.cmd_pub.publish(cmd)
                return

        # 2. 신호등 체크
        if self.use_traffic_light and self.traffic_light_state == "red":
            self.get_logger().info(
                "STOP: red light",
                throttle_duration_sec=1.0)
            self.cmd_pub.publish(cmd)
            return

        # 3. 상태 업데이트
        self._update_state()

        # 4. 상태별 조향 + 속도 계산
        dt = 1.0 / 20.0

        if self.state == DriveState.NORMAL:
            if self.lane_offset is not None:
                steer = self._compute_steering(self.lane_offset, dt)
            else:
                steer = 0.0
            # 장애물 있지만 점선 없으면 감속
            if self.has_obstacle and not self.in_changed_lane:
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

        elif self.state == DriveState.LANE_RETURN:
            # 고정 조향으로 원래 차선 복귀
            steer = self.lane_change_steer * self.change_direction
            speed = self.slow_speed
            self.prev_error = 0.0  # PD 리셋

        else:
            steer = 0.0
            speed = 0.0

        # 5. 명령 발행
        cmd.speed = speed
        cmd.steering_angle = float(steer)
        self.cmd_pub.publish(cmd)

        # 6. 상태 발행
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        # 로그
        lane_str = f"{self.lane_offset:+.2f}" if self.lane_offset is not None else "N/A"
        self.get_logger().info(
            f"[{self.state.value}] lane={lane_str}, "
            f"steer={steer:+.3f}rad, speed={speed:.2f}m/s, "
            f"lines=L:{self.left_line_type}/R:{self.right_line_type}, "
            f"obs={self.has_obstacle}, changed={self.in_changed_lane}",
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
