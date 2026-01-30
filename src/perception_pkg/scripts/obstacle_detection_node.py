#!/usr/bin/env python3
"""HSV/에지 기반 정적 장애물 감지 노드."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension


@dataclass
class Detection:
    """간단한 박스 표현."""

    x1: float
    y1: float
    x2: float
    y2: float
    score: float

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) * 0.5


class ObstacleDetectionNode:
    """HSV/에지 기반 ROI에서 장애물 후보를 찾아 퍼블리시."""

    def __init__(self) -> None:
        self.bridge = CvBridge()

        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image/compressed")
        self.use_compressed = bool(rospy.get_param("~use_compressed", True))
        self.roi_y_ratio = float(rospy.get_param("~roi_y_ratio", 0.55))
        self.band_ratio = float(rospy.get_param("~band_ratio", 0.6))
        self.center_ratio = float(
            rospy.get_param(
                "~obstacle_center_ratio",
                rospy.get_param("~center_ratio", 0.20),
            )
        )
        self.square_ratio = float(
            rospy.get_param(
                "~obstacle_square_ratio",
                rospy.get_param("~square_ratio", 0.0),
            )
        )
        self.min_area = float(rospy.get_param("~min_area", 600.0))
        self.min_height = float(rospy.get_param("~min_height", 20.0))
        self.saturation_thresh = int(rospy.get_param("~saturation_thresh", 60))
        self.value_thresh = int(rospy.get_param("~value_thresh", 40))
        self.blur_kernel = int(rospy.get_param("~blur_kernel", 5))
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1
        self.canny_low = int(rospy.get_param("~canny_low", 80))
        self.canny_high = int(rospy.get_param("~canny_high", 160))
        self.morph_kernel = int(rospy.get_param("~morph_kernel", 5))
        self.bias_gain = float(rospy.get_param("~bias_gain", 1.5))
        self.min_bias_weight = float(rospy.get_param("~min_bias_weight", 0.05))
        self.publish_overlay_enabled = bool(rospy.get_param("~publish_overlay", False))

        self.pub = rospy.Publisher("/perception/obstacles_2d", Float32MultiArray, queue_size=1)
        self.bias_pub = rospy.Publisher("/perception/obstacle_bias", Float32, queue_size=1)
        if self.publish_overlay_enabled:
            self.overlay_pub = rospy.Publisher(
                "/perception/obstacle_overlay", Image, queue_size=1
            )
        else:
            self.overlay_pub = None

        if self.use_compressed:
            rospy.Subscriber(
                self.camera_topic, CompressedImage, self.compressed_cb, queue_size=1
            )
        else:
            rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1)

        rospy.loginfo(
            "[obstacle_roi] subscribe: %s (compressed=%s)",
            self.camera_topic,
            self.use_compressed,
        )

    # ------------------------------------------------------------------
    # ROS 콜백

    def compressed_cb(self, msg: CompressedImage) -> None:
        array = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("[obstacle_roi] JPEG decode failed")
            return
        self.process_frame(frame, msg.header.stamp)

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover
            rospy.logwarn("[obstacle_roi] cv_bridge error: %s", exc)
            return
        self.process_frame(frame, msg.header.stamp)

    # ------------------------------------------------------------------

    def process_frame(self, frame: np.ndarray, stamp: rospy.Time) -> None:
        detections, overlay = self.detect_obstacles(frame)
        self.publish_detections(detections)
        bias = self.compute_bias(detections, frame.shape[1])
        self.bias_pub.publish(Float32(data=bias))

        if self.overlay_pub is not None and overlay is not None:
            try:
                msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                msg.header.stamp = stamp
                self.overlay_pub.publish(msg)
            except Exception as exc:
                rospy.logwarn("[obstacle_roi] overlay publish failed: %s", exc)

    def detect_obstacles(self, frame: np.ndarray) -> Tuple[List[Detection], Optional[np.ndarray]]:
        height, width = frame.shape[:2]
        use_square = self.square_ratio > 0.0

        if use_square:
            ratio = float(np.clip(self.square_ratio, 0.05, 0.9))
            side = int(min(height, width) * ratio)
            side = max(1, side)
            half_side = side // 2
            center_x = width // 2
            center_y = height // 2
            x_left = max(0, center_x - half_side)
            x_right = min(width, x_left + side)
            x_left = max(0, x_right - side)
            y_top = max(0, center_y - half_side)
            y_bottom = min(height, y_top + side)
            y_top = max(0, y_bottom - side)
            band = frame[y_top:y_bottom, x_left:x_right]
            roi_start = 0
        else:
            roi_start = int(height * np.clip(self.roi_y_ratio, 0.0, 0.95))
            roi = frame[roi_start:, :]
            if roi.size == 0:
                return [], None

            band_height = int(max(1, roi.shape[0] * np.clip(self.band_ratio, 0.05, 1.0)))
            band_start = max(0, roi.shape[0] - band_height)
            band_end = min(roi.shape[0], band_start + band_height)

            band_center = (band_start + band_end) // 2
            center_norm = np.clip(self.center_ratio, 0.05, 1.0)
            half_width = int(width * center_norm * 0.5)
            center_x = width // 2
            x_left = max(0, center_x - half_width)
            x_right = min(width, center_x + half_width)
            y_top = band_start
            y_bottom = band_end
            band = roi[y_top:y_bottom, x_left:x_right]
            y_top += roi_start
            y_bottom += roi_start

        hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
        sat_mask = cv2.threshold(hsv[:, :, 1], self.saturation_thresh, 255, cv2.THRESH_BINARY)[1]
        value_mask = cv2.threshold(hsv[:, :, 2], self.value_thresh, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.bitwise_and(sat_mask, value_mask)

        gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
        if self.blur_kernel > 1:
            gray = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)

        combined = cv2.bitwise_or(mask, edges)

        if self.morph_kernel > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT, (self.morph_kernel, self.morph_kernel)
            )
            combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections: List[Detection] = []

        overlay = None
        if self.overlay_pub is not None:
            overlay = frame.copy()
            cv2.rectangle(
                overlay,
                (x_left, y_top),
                (x_right - 1, y_bottom - 1),
                (0, 255, 255),
                2,
            )

        for contour in contours:
            if len(contour) < 3:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            area = float(w * h)
            if area < self.min_area or h < self.min_height:
                continue

            x1 = float(x + x_left)
            y1 = float(y + y_top)
            x2 = float(x + w + x_left)
            y2 = float(y + h + y_top)
            detections.append(Detection(x1=x1, y1=y1, x2=x2, y2=y2, score=area))

            if overlay is not None:
                cv2.rectangle(overlay, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 255), 2)

        return detections, overlay

    def publish_detections(self, detections: Iterable[Detection]) -> None:
        data: List[float] = []
        for det in detections:
            data.extend([det.x1, det.y1, det.x2, det.y2, det.score])

        msg = Float32MultiArray()
        if data:
            det_dim = MultiArrayDimension(
                label="detections[x1,y1,x2,y2,score]", size=len(data) // 5, stride=5
            )
            feature_dim = MultiArrayDimension(label="fields", size=5, stride=1)
            msg.layout.dim = [det_dim, feature_dim]
        msg.data = data
        self.pub.publish(msg)

    def compute_bias(self, detections: Iterable[Detection], width: int) -> float:
        centers: List[float] = []
        weights: List[float] = []
        for det in detections:
            if det.area <= 0.0:
                continue
            center_norm = det.center_x / float(max(width, 1))
            width_norm = det.width / float(max(width, 1))
            weight = max(width_norm, self.min_bias_weight)
            centers.append(center_norm)
            weights.append(weight)

        if not centers:
            return 0.0

        centers_np = np.asarray(centers, dtype=np.float32)
        weights_np = np.asarray(weights, dtype=np.float32)
        offsets = (0.5 - centers_np) * 2.0
        bias = float(np.average(offsets, weights=weights_np))
        bias *= self.bias_gain
        return float(np.clip(bias, -1.0, 1.0))


def main() -> None:
    rospy.init_node("obstacle_detection_node")
    node = ObstacleDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
