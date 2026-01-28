#!/usr/bin/env python3
"""Decision node: combines lane + obstacle + ultrasonic + traffic light."""

from __future__ import annotations

from typing import List

import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


class DecisionNode:
    def __init__(self) -> None:
        self.cruise_speed = float(rospy.get_param("~cruise_speed_mps", 1.0))
        self.max_steer_rad = float(rospy.get_param("~max_steer_rad", 0.6))
        self.ultra_safe_m = float(rospy.get_param("~ultra_safe_distance_m", 0.2))
        self.use_traffic_light = bool(rospy.get_param("~use_traffic_light", True))
        self.stop_on_yellow = bool(rospy.get_param("~stop_on_yellow", True))

        self.lane_steer_norm = 0.0
        self.obstacle = False
        self.ultra_ranges: List[float] = [0.0] * 6
        self.traffic_state = "unknown"

        rospy.Subscriber("/lane/steering_angle", Float32, self.lane_cb, queue_size=1)
        rospy.Subscriber("/perception/obstacle_flag", Bool, self.obstacle_cb, queue_size=1)
        rospy.Subscriber("/ultrasonic/ranges", Float32MultiArray, self.ultra_cb, queue_size=1)
        rospy.Subscriber("/perception/traffic_light_state", String, self.traffic_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher("/decision/cmd", AckermannDrive, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.timer_cb)

        rospy.loginfo("[decision] node started")

    def lane_cb(self, msg: Float32) -> None:
        self.lane_steer_norm = float(msg.data)

    def obstacle_cb(self, msg: Bool) -> None:
        self.obstacle = bool(msg.data)

    def ultra_cb(self, msg: Float32MultiArray) -> None:
        if msg.data:
            values = list(msg.data)
            if len(values) >= 6:
                self.ultra_ranges = values[:6]

    def traffic_cb(self, msg: String) -> None:
        self.traffic_state = msg.data.lower()

    def timer_cb(self, _event) -> None:
        should_stop = self.obstacle or self._ultra_blocked()
        if self.use_traffic_light:
            if self.traffic_state == "red":
                should_stop = True
            if self.stop_on_yellow and self.traffic_state == "yellow":
                should_stop = True

        cmd = AckermannDrive()
        if should_stop:
            cmd.speed = 0.0
            cmd.steering_angle = 0.0
        else:
            steer = max(-1.0, min(1.0, self.lane_steer_norm))
            cmd.speed = self.cruise_speed
            cmd.steering_angle = steer * self.max_steer_rad

        self.cmd_pub.publish(cmd)

    def _ultra_blocked(self) -> bool:
        # indices: F, FL, FR, R, RL, RR
        front = self.ultra_ranges[0]
        front_left = self.ultra_ranges[1]
        front_right = self.ultra_ranges[2]
        values = [front, front_left, front_right]
        for distance in values:
            if distance <= 0.0:
                continue
            if distance < self.ultra_safe_m:
                return True
        return False


def main() -> None:
    rospy.init_node("decision_node")
    DecisionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
