#!/usr/bin/env python3
"""Console monitor for /scan to verify LiDAR streaming."""

from __future__ import annotations

import math
from typing import Optional

import rospy
from sensor_msgs.msg import LaserScan


class LidarMonitor:
    def __init__(self) -> None:
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.last_stamp: Optional[rospy.Time] = None
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=1)
        rospy.loginfo("[lidar_monitor] subscribe: %s", self.scan_topic)

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        valid = [d for d in msg.ranges if math.isfinite(d) and d > 0.0]
        min_dist = min(valid) if valid else 0.0
        rospy.loginfo_throttle(1.0, "[lidar_monitor] points=%d min=%.2f", len(msg.ranges), min_dist)


def main() -> None:
    rospy.init_node("lidar_monitor")
    LidarMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
