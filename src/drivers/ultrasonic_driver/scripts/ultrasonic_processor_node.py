#!/usr/bin/env python3
"""Process ultrasonic ranges into obstacle/min-range outputs."""

from __future__ import annotations

from typing import List

import rospy
from std_msgs.msg import Bool, Float32, Float32MultiArray


class UltrasonicProcessorNode:
    def __init__(self) -> None:
        self.safe_distance = float(rospy.get_param("~safe_distance_m", 0.2))
        self.ranges_topic = rospy.get_param("~ranges_topic", "/ultrasonic/ranges")

        self.min_pub = rospy.Publisher("/ultrasonic/min_range", Float32, queue_size=1)
        self.obs_pub = rospy.Publisher("/ultrasonic/obstacle", Bool, queue_size=1)

        rospy.Subscriber(self.ranges_topic, Float32MultiArray, self.ranges_cb, queue_size=1)
        rospy.loginfo("[ultrasonic] subscribe: %s", self.ranges_topic)

    def ranges_cb(self, msg: Float32MultiArray) -> None:
        if not msg.data:
            return
        values: List[float] = list(msg.data)
        if len(values) < 3:
            return

        front_values = [v for v in values[:3] if v > 0.0]
        if not front_values:
            min_range = 0.0
        else:
            min_range = min(front_values)

        has_obstacle = min_range > 0.0 and min_range < self.safe_distance

        self.min_pub.publish(Float32(data=min_range))
        self.obs_pub.publish(Bool(data=has_obstacle))


def main() -> None:
    rospy.init_node("ultrasonic_processor_node")
    UltrasonicProcessorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
