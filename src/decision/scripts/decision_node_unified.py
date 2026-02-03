#!/usr/bin/env python3
"""Unified decision node: 2026 safety + ws-build-ai obstacle avoidance.

통합 의사결정 로직:
1. 라이다/초음파 → 즉시 정지 (위험 거리)
2. 카메라 장애물 → 조향 바이어스로 회피
3. 신호등 → 정지
4. 차선 추종 + 장애물 회피 바이어스 적용
"""

from __future__ import annotations

from typing import Optional
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool, Float32, String


class DecisionNodeUnified(Node):
    def __init__(self) -> None:
        super().__init__('decision_node_unified')

        # 파라미터 선언
        self.declare_parameter('cruise_speed_mps', 1.0)
        self.declare_parameter('max_steer_rad', 0.6)
        self.declare_parameter('soft_steer_rad', 0.3)
        self.declare_parameter('soft_steer_threshold', 0.35)
        self.declare_parameter('ultra_safe_distance_m', 0.2)
        self.declare_parameter('use_obstacle_avoidance', True)
        self.declare_parameter('obstacle_bias_weight', 0.3)
        self.declare_parameter('use_traffic_light', False)
        self.declare_parameter('stop_on_yellow', True)
        self.declare_parameter('lane_timeout_sec', 0.5)
        self.declare_parameter('allow_no_lane', False)
        self.declare_parameter('shutdown_stop_repeats', 5)
        self.declare_parameter('shutdown_stop_sleep_sec', 0.05)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('test_mode', False)

        # 파라미터 읽기
        self.cruise_speed = self.get_parameter('cruise_speed_mps').get_parameter_value().double_value
        self.max_steer_rad = self.get_parameter('max_steer_rad').get_parameter_value().double_value
        self.soft_steer_rad = self.get_parameter('soft_steer_rad').get_parameter_value().double_value
        self.soft_steer_threshold = self.get_parameter('soft_steer_threshold').get_parameter_value().double_value
        self.ultra_safe_m = self.get_parameter('ultra_safe_distance_m').get_parameter_value().double_value
        self.use_obstacle_avoidance = self.get_parameter('use_obstacle_avoidance').get_parameter_value().bool_value
        self.obstacle_bias_weight = self.get_parameter('obstacle_bias_weight').get_parameter_value().double_value
        self.use_traffic_light = self.get_parameter('use_traffic_light').get_parameter_value().bool_value
        self.stop_on_yellow = self.get_parameter('stop_on_yellow').get_parameter_value().bool_value
        self.lane_timeout = self.get_parameter('lane_timeout_sec').get_parameter_value().double_value
        self.allow_no_lane = self.get_parameter('allow_no_lane').get_parameter_value().bool_value
        self.shutdown_stop_repeats = self.get_parameter('shutdown_stop_repeats').get_parameter_value().integer_value
        self.shutdown_stop_sleep = self.get_parameter('shutdown_stop_sleep_sec').get_parameter_value().double_value
        control_hz = self.get_parameter('control_hz').get_parameter_value().double_value
        self.test_mode = self.get_parameter('test_mode').get_parameter_value().bool_value

        # 상태 변수
        self.lane_steer_norm = 0.0
        self.lane_stamp: Optional[Time] = None
        self.lidar_obstacle = False
        self.ultra_min = 0.0
        self.camera_obstacle_bias = 0.0
        self.traffic_state = "unknown"

        # Subscribers
        self.create_subscription(Float32, '/lane/steering_angle', self.lane_cb, 1)
        self.create_subscription(Bool, '/perception/obstacle_flag', self.lidar_obstacle_cb, 1)
        self.create_subscription(Float32, '/ultrasonic/min_range', self.ultra_cb, 1)
        self.create_subscription(Float32, '/perception/obstacle_bias', self.camera_obstacle_cb, 1)
        self.create_subscription(String, '/perception/traffic_light_state', self.traffic_cb, 1)

        # Publisher
        self.cmd_pub = self.create_publisher(AckermannDrive, '/decision/cmd', 1)

        # Timer
        period = 1.0 / max(control_hz, 1.0)
        self.create_timer(period, self.timer_cb)

        self.get_logger().info(f'[decision_unified] node started (test_mode={self.test_mode})')
        self.get_logger().info(
            f'[decision_unified] obstacle_avoidance={self.use_obstacle_avoidance} '
            f'bias_weight={self.obstacle_bias_weight:.2f}')
        self.get_logger().info(f'[decision_unified] allow_no_lane={self.allow_no_lane}')
        self.get_logger().info(f'[decision_unified] control_hz={control_hz:.1f}')

    # -- Callbacks --

    def lane_cb(self, msg: Float32) -> None:
        self.lane_steer_norm = float(msg.data)
        self.lane_stamp = self.get_clock().now()

    def lidar_obstacle_cb(self, msg: Bool) -> None:
        self.lidar_obstacle = bool(msg.data)

    def ultra_cb(self, msg: Float32) -> None:
        self.ultra_min = float(msg.data)

    def camera_obstacle_cb(self, msg: Float32) -> None:
        self.camera_obstacle_bias = float(msg.data)

    def traffic_cb(self, msg: String) -> None:
        self.traffic_state = msg.data.lower()

    # -- Control loop --

    def timer_cb(self) -> None:
        should_stop = False
        reason = ""

        # 우선순위 1: 라이다 장애물 (위험 거리)
        if self.lidar_obstacle:
            should_stop = True
            reason = "lidar_obstacle"

        # 우선순위 2: 초음파 전방 장애물 (위험 거리)
        if not should_stop and self.ultra_min > 0.0 and self.ultra_min < self.ultra_safe_m:
            should_stop = True
            reason = f"ultrasonic({self.ultra_min:.2f}m)"

        # 우선순위 3: 신호등 (선택적)
        if not should_stop and self.use_traffic_light:
            if self.traffic_state == "red":
                should_stop = True
                reason = "red_light"
            elif self.stop_on_yellow and self.traffic_state == "yellow":
                should_stop = True
                reason = "yellow_light"

        # 우선순위 4: 차선 유효성 확인
        if not should_stop and not self.allow_no_lane:
            if self.lane_stamp is None:
                should_stop = True
                reason = "no_lane_data"
            else:
                age = (self.get_clock().now() - self.lane_stamp).nanoseconds / 1e9
                if age > self.lane_timeout:
                    should_stop = True
                    reason = f"lane_timeout({age:.1f}s)"

        # 명령 생성
        cmd = AckermannDrive()

        if should_stop:
            cmd.speed = 0.0
            cmd.steering_angle = 0.0
            self.get_logger().info(
                f'[decision_unified] STOP: {reason}',
                throttle_duration_sec=1.0)
        else:
            # 주행: 차선 추종 + 장애물 회피
            steer = max(-1.0, min(1.0, self.lane_steer_norm))

            # 카메라 장애물 회피 바이어스 적용
            if self.use_obstacle_avoidance and abs(self.camera_obstacle_bias) > 0.01:
                steer += self.camera_obstacle_bias * self.obstacle_bias_weight
                steer = max(-1.0, min(1.0, steer))
                self.get_logger().info(
                    f'[decision_unified] AVOID: lane_steer={self.lane_steer_norm:.2f} '
                    f'bias={self.camera_obstacle_bias:.2f} → final={steer:.2f}',
                    throttle_duration_sec=2.0)

            cmd.speed = self.cruise_speed
            cmd.steering_angle = self._map_steer(steer)

        self.cmd_pub.publish(cmd)

    def _map_steer(self, steer_norm: float) -> float:
        """정규화된 조향값(-1~1)을 라디안으로 변환."""
        if self.soft_steer_threshold <= 0.0:
            return steer_norm * self.max_steer_rad

        if abs(steer_norm) <= self.soft_steer_threshold:
            scale = steer_norm / self.soft_steer_threshold
            return scale * self.soft_steer_rad

        return steer_norm * self.max_steer_rad


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNodeUnified()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령 전송
        node.get_logger().warn('[decision_unified] SHUTDOWN: Sending stop commands...')
        stop_cmd = AckermannDrive()
        stop_cmd.speed = 0.0
        stop_cmd.steering_angle = 0.0
        for i in range(max(1, node.shutdown_stop_repeats)):
            node.cmd_pub.publish(stop_cmd)
            node.get_logger().info(
                f'[decision_unified] Stop command {i+1}/{node.shutdown_stop_repeats} sent')
            time.sleep(node.shutdown_stop_sleep)
        node.get_logger().warn('[decision_unified] SHUTDOWN: Stop commands completed.')
        node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
