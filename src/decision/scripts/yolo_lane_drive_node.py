#!/usr/bin/env python3
"""YOLO 세그멘테이션 기반 차선 주행 노드.

YOLO drivable_mask를 이용하여 주행 가능 영역의 중심선을 추출하고,
PID 제어로 조향각을 계산하여 모터를 제어합니다.

입력:
  - /perception/drivable_mask (Image): YOLO 세그멘테이션 주행 가능 영역 마스크
  - /perception/traffic_light_state (String): 신호등 상태 (red/green/unknown)
  - /ultrasonic/min_range (Float32): 초음파 최소 거리

출력:
  - /decision/cmd (AckermannDrive): 조향각 + 속도 명령
"""

from __future__ import annotations

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge


class YoloLaneDriveNode(Node):
    """YOLO drivable_mask 기반 차선 주행 노드."""

    def __init__(self) -> None:
        super().__init__('yolo_lane_drive_node')

        # ── 파라미터 선언 ──
        self.declare_parameter('cruise_speed_mps', 0.3)
        self.declare_parameter('max_steer_rad', 0.45)
        self.declare_parameter('min_speed_mps', 0.15)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('roi_top_ratio', 0.4)
        self.declare_parameter('roi_bottom_ratio', 0.9)
        self.declare_parameter('ultra_safe_distance_m', 0.2)
        self.declare_parameter('use_traffic_light', True)
        self.declare_parameter('stop_on_yellow', False)
        self.declare_parameter('mask_timeout_sec', 1.0)
        self.declare_parameter('control_hz', 20.0)

        # ── 파라미터 로드 ──
        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.max_steer = self.get_parameter('max_steer_rad').value
        self.min_speed = self.get_parameter('min_speed_mps').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.roi_top = self.get_parameter('roi_top_ratio').value
        self.roi_bottom = self.get_parameter('roi_bottom_ratio').value
        self.ultra_safe = self.get_parameter('ultra_safe_distance_m').value
        self.use_traffic_light = self.get_parameter('use_traffic_light').value
        self.stop_on_yellow = self.get_parameter('stop_on_yellow').value
        self.mask_timeout = self.get_parameter('mask_timeout_sec').value
        control_hz = self.get_parameter('control_hz').value

        # ── 내부 상태 ──
        self.bridge = CvBridge()
        self.latest_mask: np.ndarray | None = None
        self.last_mask_time: Time | None = None
        self.traffic_light_state = "unknown"
        self.ultra_range = 999.0

        # PID 제어
        self.prev_error = 0.0
        self.integral = 0.0

        # ── 퍼블리셔 ──
        self.cmd_pub = self.create_publisher(
            AckermannDrive, '/decision/cmd', 1)

        # ── 서브스크라이버 ──
        self.create_subscription(
            Image, '/perception/drivable_mask', self._mask_cb, 1)
        self.create_subscription(
            String, '/perception/traffic_light_state', self._traffic_light_cb, 1)
        self.create_subscription(
            Float32, '/ultrasonic/min_range', self._ultra_cb, 1)

        # ── 제어 타이머 ──
        period = 1.0 / max(control_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"[yolo_lane_drive] 초기화 완료: speed={self.cruise_speed}m/s, "
            f"kp={self.kp}, kd={self.kd}, roi=[{self.roi_top:.1%}, {self.roi_bottom:.1%}]"
        )

    # ──────────────────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────────────────

    def _mask_cb(self, msg: Image) -> None:
        """YOLO drivable_mask 수신."""
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.last_mask_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"마스크 변환 실패: {e}")

    def _traffic_light_cb(self, msg: String) -> None:
        """신호등 상태 수신."""
        self.traffic_light_state = msg.data

    def _ultra_cb(self, msg: Float32) -> None:
        """초음파 거리 수신."""
        self.ultra_range = msg.data

    # ──────────────────────────────────────────────────────────────
    # 차선 중심선 추출 (YOLO 마스크 기반)
    # ──────────────────────────────────────────────────────────────

    def _extract_lane_center(self, mask: np.ndarray) -> float | None:
        """YOLO drivable_mask에서 차선 중심선 x 좌표 추출.

        Args:
            mask: 주행 가능 영역 마스크 (255=주행 가능, 0=불가)

        Returns:
            중심선 x 좌표 (이미지 중심 기준 offset, -1.0~1.0) 또는 None
        """
        h, w = mask.shape[:2]

        # ROI 영역 설정 (하단 관심 영역)
        y1 = int(h * self.roi_top)
        y2 = int(h * self.roi_bottom)
        roi = mask[y1:y2, :]

        # 주행 가능 영역 찾기
        white_pixels = np.where(roi > 127)
        if len(white_pixels[0]) == 0:
            return None

        # 각 행(y)별로 주행 가능 영역의 중심 계산
        lane_centers = []
        for y in range(roi.shape[0]):
            row = roi[y, :]
            white_x = np.where(row > 127)[0]
            if len(white_x) > 0:
                center_x = int(np.mean(white_x))
                lane_centers.append(center_x)

        if not lane_centers:
            return None

        # 전체 중심선의 평균
        avg_center_x = int(np.mean(lane_centers))

        # 이미지 중심 기준 정규화 (-1.0 ~ 1.0)
        image_center = w / 2.0
        offset = (avg_center_x - image_center) / image_center

        return offset

    # ──────────────────────────────────────────────────────────────
    # PID 제어
    # ──────────────────────────────────────────────────────────────

    def _compute_steering(self, center_offset: float, dt: float) -> float:
        """PID 제어로 조향각 계산.

        Args:
            center_offset: 차선 중심 offset (-1.0 ~ 1.0, 왼쪽이 음수)
            dt: 시간 간격 (초)

        Returns:
            조향각 (라디안, 왼쪽이 양수)
        """
        # 오류: offset이 양수면 오른쪽으로 치우침 → 왼쪽으로 회전 필요
        error = -center_offset

        # PID 계산
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        steer = self.kp * error + self.ki * self.integral + self.kd * derivative

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

        # 1. YOLO 마스크 타임아웃 체크
        if self.latest_mask is None or self.last_mask_time is None:
            self.get_logger().warn(
                "STOP: no drivable mask",
                throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(cmd)
            return

        elapsed = (now - self.last_mask_time).nanoseconds / 1e9
        if elapsed > self.mask_timeout:
            self.get_logger().warn(
                f"STOP: mask timeout ({elapsed:.1f}s)",
                throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(cmd)
            return

        # 2. 초음파 장애물 체크
        if self.ultra_range < self.ultra_safe:
            self.get_logger().info(
                f"STOP: ultrasonic {self.ultra_range:.2f}m < {self.ultra_safe}m",
                throttle_duration_sec=1.0
            )
            self.cmd_pub.publish(cmd)
            return

        # 3. 신호등 체크
        if self.use_traffic_light:
            if self.traffic_light_state == "red":
                self.get_logger().info(
                    "STOP: red light",
                    throttle_duration_sec=1.0
                )
                self.cmd_pub.publish(cmd)
                return
            elif self.traffic_light_state == "yellow" and self.stop_on_yellow:
                self.get_logger().info(
                    "STOP: yellow light",
                    throttle_duration_sec=1.0
                )
                self.cmd_pub.publish(cmd)
                return

        # 4. 차선 중심선 추출
        center_offset = self._extract_lane_center(self.latest_mask)
        if center_offset is None:
            self.get_logger().warn(
                "STOP: no drivable area in mask",
                throttle_duration_sec=1.0
            )
            self.cmd_pub.publish(cmd)
            return

        # 5. PID 조향각 계산
        dt = 1.0 / 20.0  # 제어 주기
        steer = self._compute_steering(center_offset, dt)

        # 6. 속도 계산 (조향각에 따라 감속)
        abs_steer = abs(steer)
        if abs_steer < 0.1:
            speed = self.cruise_speed
        elif abs_steer < 0.3:
            speed = self.cruise_speed * 0.8
        else:
            speed = max(self.cruise_speed * 0.6, self.min_speed)

        # 7. 명령 발행
        cmd.speed = speed
        cmd.steering_angle = steer
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"DRIVE: offset={center_offset:+.2f}, steer={steer:+.3f}rad, speed={speed:.2f}m/s",
            throttle_duration_sec=0.5
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloLaneDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
