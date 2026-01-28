#!/usr/bin/env python3
"""Console monitor for ultrasonic topics."""

from __future__ import annotations

import rospy
from std_msgs.msg import Float32, Float32MultiArray


class UltrasonicMonitor:
    def __init__(self) -> None:
        self.ranges_topic = rospy.get_param("~ranges_topic", "/ultrasonic/ranges")
        self.min_topic = rospy.get_param("~min_topic", "/ultrasonic/min_range")

        rospy.Subscriber(self.ranges_topic, Float32MultiArray, self.ranges_cb, queue_size=1)
        rospy.Subscriber(self.min_topic, Float32, self.min_cb, queue_size=1)
        rospy.loginfo("[ultrasonic_monitor] ranges: %s min: %s", self.ranges_topic, self.min_topic)

    def ranges_cb(self, msg: Float32MultiArray) -> None:
        if msg.data:
            rospy.loginfo_throttle(1.0, "[ultrasonic_monitor] ranges=%s", [round(v, 2) for v in msg.data])

    def min_cb(self, msg: Float32) -> None:
        rospy.loginfo_throttle(1.0, "[ultrasonic_monitor] min_range=%.2f", msg.data)


def main() -> None:
    rospy.init_node("ultrasonic_monitor")
    UltrasonicMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
