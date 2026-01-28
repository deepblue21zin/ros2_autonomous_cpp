#!/usr/bin/env python3
"""Decision node aligned with 2026 rule-based logic."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool, Float32, String


class DecisionNode2026(Node):
    def __init__(self) -> None:
        super().__init__('decision_node_2026')

        # Declare parameters
        self.declare_parameter('cruise_speed_mps', 1.0)
        self.declare_parameter('max_steer_rad', 0.6)
        self.declare_parameter('soft_steer_rad', 0.3)
        self.declare_parameter('soft_steer_threshold', 0.35)
        self.declare_parameter('ultra_safe_distance_m', 0.2)
        self.declare_parameter('use_traffic_light', False)
        self.declare_parameter('stop_on_yellow', True)
        self.declare_parameter('lane_timeout_sec', 0.5)

        # Get parameters
        self.cruise_speed = self.get_parameter('cruise_speed_mps').get_parameter_value().double_value
        self.max_steer_rad = self.get_parameter('max_steer_rad').get_parameter_value().double_value
        self.soft_steer_rad = self.get_parameter('soft_steer_rad').get_parameter_value().double_value
        self.soft_steer_threshold = self.get_parameter('soft_steer_threshold').get_parameter_value().double_value
        self.ultra_safe_m = self.get_parameter('ultra_safe_distance_m').get_parameter_value().double_value
        self.use_traffic_light = self.get_parameter('use_traffic_light').get_parameter_value().bool_value
        self.stop_on_yellow = self.get_parameter('stop_on_yellow').get_parameter_value().bool_value
        self.lane_timeout = self.get_parameter('lane_timeout_sec').get_parameter_value().double_value

        # State variables
        self.lane_steer_norm = 0.0
        self.lane_stamp: Optional[Time] = None
        self.obstacle = False
        self.ultra_min = 0.0
        self.traffic_state = "unknown"

        # Subscribers
        self.create_subscription(Float32, '/lane/steering_angle', self.lane_cb, 1)
        self.create_subscription(Bool, '/perception/obstacle_flag', self.obstacle_cb, 1)
        self.create_subscription(Float32, '/ultrasonic/min_range', self.ultra_cb, 1)
        self.create_subscription(String, '/perception/traffic_light_state', self.traffic_cb, 1)

        # Publisher
        self.cmd_pub = self.create_publisher(AckermannDrive, '/decision/cmd', 1)

        # Timer (20Hz = 50ms)
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info('[decision_2026] node started')

    def lane_cb(self, msg: Float32) -> None:
        self.lane_steer_norm = float(msg.data)
        self.lane_stamp = self.get_clock().now()

    def obstacle_cb(self, msg: Bool) -> None:
        self.obstacle = bool(msg.data)

    def ultra_cb(self, msg: Float32) -> None:
        self.ultra_min = float(msg.data)

    def traffic_cb(self, msg: String) -> None:
        self.traffic_state = msg.data.lower()

    def timer_cb(self) -> None:
        should_stop = False

        # 1) LiDAR obstacle
        if self.obstacle:
            should_stop = True

        # 2) Ultrasonic front check
        if self.ultra_min > 0.0 and self.ultra_min < self.ultra_safe_m:
            should_stop = True

        # 3) Traffic light (optional)
        if self.use_traffic_light:
            if self.traffic_state == "red":
                should_stop = True
            if self.stop_on_yellow and self.traffic_state == "yellow":
                should_stop = True

        # 4) Lane availability
        if self.lane_stamp is None:
            should_stop = True
        else:
            now = self.get_clock().now()
            age = (now - self.lane_stamp).nanoseconds / 1e9
            if age > self.lane_timeout:
                should_stop = True

        cmd = AckermannDrive()
        if should_stop:
            cmd.speed = 0.0
            cmd.steering_angle = 0.0
        else:
            steer = max(-1.0, min(1.0, self.lane_steer_norm))
            cmd.speed = self.cruise_speed
            cmd.steering_angle = self._map_steer(steer)

        self.cmd_pub.publish(cmd)

    def _map_steer(self, steer_norm: float) -> float:
        if self.soft_steer_threshold <= 0.0:
            return steer_norm * self.max_steer_rad
        if abs(steer_norm) <= self.soft_steer_threshold:
            scale = steer_norm / self.soft_steer_threshold
            return scale * self.soft_steer_rad
        return steer_norm * self.max_steer_rad


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode2026()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
