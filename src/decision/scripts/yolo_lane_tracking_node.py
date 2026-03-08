#!/usr/bin/env python3
"""YOLO 차선 추적 노드 (Predict+Hold + Polynomial Fitting).

YOLO segmentation으로 감지된 차선 마스크를 받아서:
  1. 차선 포인트 추출
  2. Polynomial fitting (2차 다항식)
  3. Predict+Hold (차선 누락 시 이전 결과 유지)
  4. Moving average (부드럽게)
  5. 조향각 계산 및 발행

입력:
  - /perception/lane_left_mask (Image): 좌차선 마스크
  - /perception/lane_right_mask (Image): 우차선 마스크

출력:
  - /lane/steering_angle (Float32): 조향각 (-1.0 ~ 1.0)
  - /yolo_lane/overlay (Image): 디버그 오버레이 (선택)
"""

from __future__ import annotations

from collections import deque
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class YoloLaneTrackingNode(Node):
    """YOLO 차선 추적 노드."""

    def __init__(self) -> None:
        super().__init__('yolo_lane_tracking_node')

        # ── 파라미터 선언 ──
        self.declare_parameter('max_lost_frames', 15)      # Predict+Hold 최대 유지 프레임
        self.declare_parameter('avg_window', 5)             # Moving average 윈도우 크기
        self.declare_parameter('kp', 2.5)                   # P 게인
        self.declare_parameter('kd', 1.5)                   # D 게인 (선택)
        self.declare_parameter('lane_width_ratio', 0.55)    # 단일 차선 시 예상 차선 폭
        self.declare_parameter('min_points', 10)            # 차선 최소 포인트 수
        self.declare_parameter('publish_overlay', True)     # 디버그 오버레이 발행
        self.declare_parameter('control_hz', 20.0)          # 제어 주기

        # ── 파라미터 로드 ──
        self.max_lost_frames = self.get_parameter('max_lost_frames').value
        self.avg_window = self.get_parameter('avg_window').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.lane_width_ratio = self.get_parameter('lane_width_ratio').value
        self.min_points = self.get_parameter('min_points').value
        self.publish_overlay = self.get_parameter('publish_overlay').value
        control_hz = self.get_parameter('control_hz').value

        # ── 내부 상태 ──
        self.bridge = CvBridge()
        self.latest_left_mask: Optional[np.ndarray] = None
        self.latest_right_mask: Optional[np.ndarray] = None

        # Predict+Hold
        self.lost_frames = 0
        self.last_valid_steering = 0.0

        # Moving average
        self.steering_history = deque(maxlen=self.avg_window)

        # D 게인용 이전 offset
        self.prev_offset = 0.0

        # ── 퍼블리셔 ──
        self.steer_pub = self.create_publisher(Float32, '/lane/steering_angle', 1)

        if self.publish_overlay:
            self.overlay_pub = self.create_publisher(Image, '/yolo_lane/overlay', 1)
        else:
            self.overlay_pub = None

        # ── 서브스크라이버 ──
        self.create_subscription(
            Image, '/perception/lane_left_mask', self._left_mask_cb, 1)
        self.create_subscription(
            Image, '/perception/lane_right_mask', self._right_mask_cb, 1)

        # ── 제어 타이머 ──
        period = 1.0 / max(control_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"[yolo_lane_tracking] 초기화 완료: "
            f"max_lost={self.max_lost_frames}, avg_window={self.avg_window}, "
            f"kp={self.kp}, kd={self.kd}, hz={control_hz}"
        )

    # ──────────────────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────────────────

    def _left_mask_cb(self, msg: Image) -> None:
        """좌차선 마스크 수신."""
        try:
            self.latest_left_mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f"좌차선 마스크 변환 실패: {e}")

    def _right_mask_cb(self, msg: Image) -> None:
        """우차선 마스크 수신."""
        try:
            self.latest_right_mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f"우차선 마스크 변환 실패: {e}")

    def _control_loop(self) -> None:
        """제어 루프 (타이머 콜백)."""
        left_mask = self.latest_left_mask
        right_mask = self.latest_right_mask

        if left_mask is None and right_mask is None:
            # 마스크가 하나도 없음 → 직진
            steering = 0.0
            self.steer_pub.publish(Float32(data=steering))
            return

        # 1. 차선 포인트 추출
        left_points = self._extract_lane_points(left_mask) if left_mask is not None else None
        right_points = self._extract_lane_points(right_mask) if right_mask is not None else None

        # 2. Polynomial fitting
        left_poly = self._fit_polynomial(left_points)
        right_poly = self._fit_polynomial(right_points)

        # 3. 차선 중심 계산
        h, w = left_mask.shape if left_mask is not None else right_mask.shape
        lane_center = self._compute_lane_center(left_poly, right_poly, h, w)

        # 4. Predict+Hold
        if lane_center is not None:
            # 차선 감지 성공
            self.lost_frames = 0
            img_center = w / 2.0
            offset = lane_center - img_center
            offset_norm = offset / img_center
        else:
            # 차선 감지 실패 → 이전 결과 유지
            self.lost_frames += 1
            if self.lost_frames < self.max_lost_frames:
                offset_norm = self.prev_offset  # Hold!
            else:
                offset_norm = 0.0  # 직진

        # 5. PD 제어
        p_term = self.kp * (-offset_norm)
        d_term = self.kd * (-(offset_norm - self.prev_offset))
        steering = p_term + d_term
        steering = np.clip(steering, -1.0, 1.0)

        # 6. Moving average
        self.steering_history.append(steering)
        smooth_steering = np.mean(self.steering_history)

        # 7. 상태 업데이트
        self.prev_offset = offset_norm
        self.last_valid_steering = smooth_steering

        # 8. 발행
        steer_msg = Float32()
        steer_msg.data = float(smooth_steering)
        self.steer_pub.publish(steer_msg)

        # 9. 디버그 오버레이
        if self.overlay_pub is not None:
            overlay = self._draw_overlay(
                left_mask, right_mask, left_poly, right_poly, lane_center
            )
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            self.overlay_pub.publish(overlay_msg)

    # ──────────────────────────────────────────────────────────────
    # 차선 처리
    # ──────────────────────────────────────────────────────────────

    def _extract_lane_points(self, mask: np.ndarray) -> Optional[np.ndarray]:
        """차선 마스크에서 포인트 추출.

        Args:
            mask: 차선 마스크 (uint8, 0 or 255)

        Returns:
            points: (N, 2) 형태의 좌표 배열 [[x1, y1], [x2, y2], ...] 또는 None
        """
        if mask is None or not np.any(mask):
            return None

        # Contour 추출
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 가장 큰 contour 선택
        largest = max(contours, key=cv2.contourArea)
        points = largest.reshape(-1, 2)

        if len(points) < self.min_points:
            return None

        return points

    def _fit_polynomial(self, points: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """Polynomial fitting (2차 다항식).

        Args:
            points: (N, 2) 형태의 좌표 배열 [[x1, y1], [x2, y2], ...]

        Returns:
            coeffs: 2차 다항식 계수 [a, b, c] (x = a*y^2 + b*y + c) 또는 None
        """
        if points is None or len(points) < self.min_points:
            return None

        x = points[:, 0]
        y = points[:, 1]

        try:
            # y를 독립변수로, x를 종속변수로 피팅 (차선은 y가 증가하면서 x가 변함)
            coeffs = np.polyfit(y, x, 2)  # x = a*y^2 + b*y + c
            return coeffs
        except Exception as e:
            self.get_logger().warn(f"Polynomial fitting 실패: {e}")
            return None

    def _compute_lane_center(
        self,
        left_poly: Optional[np.ndarray],
        right_poly: Optional[np.ndarray],
        img_height: int,
        img_width: int
    ) -> Optional[float]:
        """좌/우 차선 다항식에서 중심선 계산.

        Args:
            left_poly: 좌차선 다항식 계수
            right_poly: 우차선 다항식 계수
            img_height: 이미지 높이
            img_width: 이미지 폭

        Returns:
            center_x: 차선 중심 x 좌표 (화면 하단 70% 지점) 또는 None
        """
        if left_poly is None and right_poly is None:
            return None

        # 평가 지점: 화면 하단 70% 위치
        y_eval = img_height * 0.7

        if left_poly is not None and right_poly is not None:
            # 양쪽 차선 모두 있음
            left_x = np.polyval(left_poly, y_eval)
            right_x = np.polyval(right_poly, y_eval)
            center_x = (left_x + right_x) / 2.0

        elif left_poly is not None:
            # 좌차선만 있음 → 예상 차선 폭 사용
            left_x = np.polyval(left_poly, y_eval)
            lane_width = img_width * self.lane_width_ratio
            center_x = left_x + lane_width / 2.0

        else:
            # 우차선만 있음 → 예상 차선 폭 사용
            right_x = np.polyval(right_poly, y_eval)
            lane_width = img_width * self.lane_width_ratio
            center_x = right_x - lane_width / 2.0

        return center_x

    def _draw_overlay(
        self,
        left_mask: Optional[np.ndarray],
        right_mask: Optional[np.ndarray],
        left_poly: Optional[np.ndarray],
        right_poly: Optional[np.ndarray],
        lane_center: Optional[float]
    ) -> np.ndarray:
        """디버그 오버레이 생성."""
        # 마스크 중 하나라도 있으면 그걸 기준으로 생성
        if left_mask is not None:
            h, w = left_mask.shape
        elif right_mask is not None:
            h, w = right_mask.shape
        else:
            h, w = 480, 640  # 기본값

        overlay = np.zeros((h, w, 3), dtype=np.uint8)

        # 마스크 시각화
        if left_mask is not None:
            overlay[left_mask > 0] = [0, 255, 0]  # 녹색
        if right_mask is not None:
            overlay[right_mask > 0] = [255, 0, 0]  # 파란색

        # Polynomial 곡선 그리기
        y_range = np.arange(0, h, 10)

        if left_poly is not None:
            left_x = np.polyval(left_poly, y_range)
            for i in range(len(y_range) - 1):
                pt1 = (int(left_x[i]), int(y_range[i]))
                pt2 = (int(left_x[i + 1]), int(y_range[i + 1]))
                cv2.line(overlay, pt1, pt2, (0, 255, 255), 2)  # 노란색

        if right_poly is not None:
            right_x = np.polyval(right_poly, y_range)
            for i in range(len(y_range) - 1):
                pt1 = (int(right_x[i]), int(y_range[i]))
                pt2 = (int(right_x[i + 1]), int(y_range[i + 1]))
                cv2.line(overlay, pt1, pt2, (0, 255, 255), 2)  # 노란색

        # 차선 중심 표시
        if lane_center is not None:
            y_eval = int(h * 0.7)
            cv2.circle(overlay, (int(lane_center), y_eval), 10, (255, 255, 255), -1)
            cv2.line(overlay, (int(lane_center), 0), (int(lane_center), h), (255, 255, 255), 2)

        # 텍스트 정보
        steering_text = f"Steering: {self.last_valid_steering:.2f}"
        lost_text = f"Lost frames: {self.lost_frames}/{self.max_lost_frames}"
        cv2.putText(overlay, steering_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, lost_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return overlay


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloLaneTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
