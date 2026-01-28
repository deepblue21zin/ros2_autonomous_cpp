#!/usr/bin/env python3
"""LiDAR obstacle detector based on angle + distance window."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


def normalize_angle(rad: float) -> float:
    while rad < 0.0:
        rad += 2.0 * math.pi
    while rad >= 2.0 * math.pi:
        rad -= 2.0 * math.pi
    return rad


def angle_in_window(angle: float, start: float, end: float) -> bool:
    if start <= end:
        return start <= angle <= end
    return angle >= start or angle <= end


class LidarObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_obstacle_node')

        # Declare parameters
        self.declare_parameter('angle_min_deg', 350.0)
        self.declare_parameter('angle_max_deg', 10.0)
        self.declare_parameter('distance_m', 0.5)
        self.declare_parameter('scan_topic', '/scan')

        # Get parameters
        self.angle_min_deg = self.get_parameter('angle_min_deg').get_parameter_value().double_value
        self.angle_max_deg = self.get_parameter('angle_max_deg').get_parameter_value().double_value
        self.distance_m = self.get_parameter('distance_m').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # Publishers
        self.flag_pub = self.create_publisher(Bool, '/perception/obstacle_flag', 1)
        self.dist_pub = self.create_publisher(Float32, '/perception/obstacle_distance', 1)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            1
        )

        self.get_logger().info(f'[lidar_obstacle] subscribe: {self.scan_topic}')

    def scan_cb(self, scan: LaserScan) -> None:
        start = math.radians(self.angle_min_deg)
        end = math.radians(self.angle_max_deg)
        start = normalize_angle(start)
        end = normalize_angle(end)

        min_dist: Optional[float] = None
        for idx, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            angle = scan.angle_min + idx * scan.angle_increment
            angle = normalize_angle(angle)
            if not angle_in_window(angle, start, end):
                continue
            if distance <= self.distance_m:
                if min_dist is None or distance < min_dist:
                    min_dist = distance

        has_obstacle = min_dist is not None
        flag_msg = Bool()
        flag_msg.data = has_obstacle
        self.flag_pub.publish(flag_msg)

        dist_msg = Float32()
        if min_dist is None:
            dist_msg.data = 0.0
        else:
            dist_msg.data = float(min_dist)
        self.dist_pub.publish(dist_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
