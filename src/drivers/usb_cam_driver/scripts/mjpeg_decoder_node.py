#!/usr/bin/env python3
"""Decode MJPEG byte-stream images to bgr8 Image messages.

This node is intended for usb_cam ``pixel_format=raw_mjpeg`` mode.
It subscribes to a raw camera topic and republishes decoded ``bgr8``.
"""

from __future__ import annotations

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


def _decode_uncompressed_image(msg: Image) -> np.ndarray | None:
    """Best-effort decode for non-JPEG Image encodings."""
    try:
        if msg.encoding == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ).copy()
        if msg.encoding == "rgb8":
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if msg.encoding == "mono8":
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width
            )
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if msg.encoding in ("yuyv", "yuy2", "yuv422_yuy2", "uyvy"):
            packed = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 2
            )
            if msg.encoding == "uyvy":
                return cv2.cvtColor(packed, cv2.COLOR_YUV2BGR_UYVY)
            return cv2.cvtColor(packed, cv2.COLOR_YUV2BGR_YUYV)
    except Exception:
        return None
    return None


class MjpegDecoderNode(Node):
    """Decode raw_mjpeg camera stream and publish bgr8 images."""

    def __init__(self) -> None:
        super().__init__("mjpeg_decoder_node")

        self.declare_parameter("input_topic", "/camera/front/image_raw_mjpeg")
        self.declare_parameter("output_topic", "/camera/front/image")
        self.declare_parameter("output_frame_id", "")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.output_frame_id = self.get_parameter("output_frame_id").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(Image, output_topic, qos)
        self.create_subscription(Image, input_topic, self._image_cb, qos)

        self.get_logger().info(
            f"[mjpeg_decoder] input={input_topic} -> output={output_topic}"
        )

    def _image_cb(self, msg: Image) -> None:
        frame: np.ndarray | None = None

        # raw_mjpeg path: JPEG byte stream in msg.data
        np_buf = np.frombuffer(msg.data, dtype=np.uint8)
        if np_buf.size > 0:
            frame = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)

        # Fallback for already-uncompressed input encodings
        if frame is None:
            frame = _decode_uncompressed_image(msg)

        if frame is None:
            self.get_logger().warn(
                f"decode failed (encoding={msg.encoding}, bytes={len(msg.data)})",
                throttle_duration_sec=1.0,
            )
            return

        out = Image()
        out.header = msg.header
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        out.height = frame.shape[0]
        out.width = frame.shape[1]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = frame.shape[1] * 3
        out.data = frame.tobytes()
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MjpegDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

