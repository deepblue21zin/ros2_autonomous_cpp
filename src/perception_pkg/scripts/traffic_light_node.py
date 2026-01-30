#!/usr/bin/env python3
"""신호등 인식 노드."""

from __future__ import annotations

from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from perception_pkg.perception.object_detection.detector import ObjectDetector


class TrafficLightNode:
    """YOLO 검출 + HSV 마스크 기반으로 신호등 상태를 판정."""

    LABEL_MAP = {
        "traffic_light_red": "red",
        "traffic_light_yellow": "yellow",
        "traffic_light_green": "green",
        "traffic_light_off": "off",
    }

    COLOR_KEYWORDS = {
        "red": "red",
        "yellow": "yellow",
        "amber": "yellow",
        "green": "green",
        "off": "off",
    }

    def __init__(self) -> None:
        self.bridge = CvBridge()

        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.use_compressed = rospy.get_param("~use_compressed", False)
        score_threshold = float(rospy.get_param("~score_threshold", 0.5))
        detector_model = rospy.get_param("~detector_model_path", "")
        detector_imgsz = int(rospy.get_param("~detector_imgsz", 640))
        detector_device_param = rospy.get_param("~detector_device", "")
        detector_device = detector_device_param or None
        detector_label_map_param = rospy.get_param(
            "~detector_label_mapping",
            {
                "traffic_light_red": "traffic_light_red",
                "traffic_light_green": "traffic_light_green",
                "traffic_light_yellow": "traffic_light_yellow",
                "red_light": "traffic_light_red",
                "green_light": "traffic_light_green",
                "yellow_light": "traffic_light_yellow",
            },
        )
        detector_label_map = {str(k): str(v) for k, v in detector_label_map_param.items()}

        self.unknown_timeout = rospy.Duration.from_sec(
            float(rospy.get_param("~unknown_timeout", 2.0))
        )

        self.pt_model_path = rospy.get_param("~pt_model_path", "")
        self.detector: Optional[ObjectDetector] = None
        ignore_keywords_param = rospy.get_param(
            "~ignored_label_keywords", ["pedestrian", "walk"]
        )
        self.ignored_label_keywords = tuple(
            str(keyword).lower() for keyword in ignore_keywords_param
        )

        try:
            if detector_model:
                self.detector = ObjectDetector(
                    score_threshold=score_threshold,
                    model_path=detector_model,
                    class_map=detector_label_map,
                    imgsz=detector_imgsz,
                    device=detector_device,
                )
                rospy.loginfo(
                    "[traffic_light] detector model loaded: %s (imgsz=%d, device=%s)",
                    detector_model,
                    detector_imgsz,
                    detector_device or "auto",
                )
            elif self.pt_model_path:
                pt_conf_threshold = float(rospy.get_param("~pt_conf_threshold", 0.4))
                pt_iou_threshold = float(rospy.get_param("~pt_iou_threshold", 0.45))
                label_map_param = rospy.get_param(
                    "~pt_label_map",
                    {
                        "Green Light": "traffic_light_green",
                        "Red Light": "traffic_light_red",
                    },
                )
                label_map = self._load_label_map(label_map_param)
                device = rospy.get_param("~pt_device", "")
                device_arg = device if device else None

                from perception_pkg.perception.object_detection.yolo_speed_sign_pt import (
                    YoloSpeedSignPTConfig,
                    YoloSpeedSignPTDetector,
                )

                config = YoloSpeedSignPTConfig(
                    model_path=self.pt_model_path,
                    conf_threshold=pt_conf_threshold,
                    iou_threshold=pt_iou_threshold,
                    label_prefix="traffic_light_",
                    label_map=label_map if label_map else None,
                    device=device_arg,
                )
                self.detector = YoloSpeedSignPTDetector(config)
                rospy.loginfo(
                    "[traffic_light] YOLO PT detector loaded (model=%s, device=%s)",
                    self.pt_model_path,
                    device_arg or "auto",
                )
            else:
                self.detector = ObjectDetector(score_threshold=score_threshold)
        except Exception as exc:
            rospy.logerr("[traffic_light] detector 초기화 실패: %s", exc)
            raise

        # HSV 기반 설정
        self.use_lane_split = bool(rospy.get_param("~use_lane_split", True))
        self.lane_roi_ratio = float(rospy.get_param("~lane_roi_ratio", 0.55))
        self.roi_margin = float(rospy.get_param("~roi_margin", 0.02))
        self.roi_x = float(rospy.get_param("~roi_x_start", 0.45))
        self.roi_y = float(rospy.get_param("~roi_y_start", 0.05))
        self.roi_w = float(rospy.get_param("~roi_width", 0.1))
        self.roi_h = float(rospy.get_param("~roi_height", 0.25))
        self.blur_kernel = int(rospy.get_param("~blur_kernel", 3))
        self.morph_kernel_size = int(rospy.get_param("~morph_kernel", 3))
        self.min_ratio = float(rospy.get_param("~ratio_threshold", 0.08))
        self.state_margin = float(rospy.get_param("~state_margin", 0.05))
        self.off_value_threshold = float(rospy.get_param("~off_value_threshold", 60.0))

        self.lower_red_1 = np.array(rospy.get_param("~lower_red_1", [0, 80, 80]), dtype=np.uint8)
        self.upper_red_1 = np.array(rospy.get_param("~upper_red_1", [10, 255, 255]), dtype=np.uint8)
        self.lower_red_2 = np.array(rospy.get_param("~lower_red_2", [170, 80, 80]), dtype=np.uint8)
        self.upper_red_2 = np.array(rospy.get_param("~upper_red_2", [180, 255, 255]), dtype=np.uint8)
        self.lower_yellow = np.array(rospy.get_param("~lower_yellow", [18, 120, 120]), dtype=np.uint8)
        self.upper_yellow = np.array(rospy.get_param("~upper_yellow", [38, 255, 255]), dtype=np.uint8)
        self.lower_green = np.array(rospy.get_param("~lower_green", [45, 80, 80]), dtype=np.uint8)
        self.upper_green = np.array(rospy.get_param("~upper_green", [90, 255, 255]), dtype=np.uint8)

        self.state_pub = rospy.Publisher("/perception/traffic_light_state", String, queue_size=1)
        self.publish_overlay_enabled = rospy.get_param("~publish_overlay", True)
        self.overlay_pub = (
            rospy.Publisher("/perception/traffic_light_overlay", Image, queue_size=1)
            if self.publish_overlay_enabled
            else None
        )
        self.yolo_cooldown = rospy.Duration.from_sec(
            float(rospy.get_param("~yolo_cooldown", 0.2))
        )
        self.yolo_on_change = bool(rospy.get_param("~yolo_on_change", True))
        self.last_yolo_stamp = rospy.Time(0)
        self.current_state = "unknown"
        self.last_update = rospy.Time(0)

        if self.use_compressed:
            self.sub = rospy.Subscriber(
                self.camera_topic, CompressedImage, self.compressed_cb, queue_size=1
            )
        else:
            self.sub = rospy.Subscriber(
                self.camera_topic, Image, self.image_cb, queue_size=1
            )
        rospy.loginfo(
            "[traffic_light] subscribe: %s (compressed=%s)",
            self.camera_topic,
            self.use_compressed,
        )

    def compressed_cb(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("[traffic_light] JPEG decode failed.")
            return
        self.handle_frame(frame, msg.header.stamp)

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:  # pragma: no cover
            rospy.logwarn("[traffic_light] cv_bridge error: %s", exc)
            return
        self.handle_frame(frame, msg.header.stamp)

    def handle_frame(self, frame: np.ndarray, stamp: rospy.Time) -> None:
        detections = None
        hsv_state = self.detect_state_with_hsv(frame)
        state = hsv_state

        now = rospy.Time.now()
        should_run_yolo = False
        if self.detector is not None:
            cooldown_passed = (
                self.yolo_cooldown.to_sec() <= 0.0 or now - self.last_yolo_stamp >= self.yolo_cooldown
            )
            if hsv_state is None and cooldown_passed:
                should_run_yolo = True
            elif (
                hsv_state is not None
                and self.yolo_on_change
                and hsv_state != self.current_state
                and cooldown_passed
            ):
                should_run_yolo = True

        if should_run_yolo and self.detector is not None:
            detections = self.detector.detect(frame)
            yolo_state = self.extract_state_from_detections(detections)
            self.last_yolo_stamp = now
            if yolo_state is not None:
                state = yolo_state
        elif state is None and self.detector is not None and self.yolo_cooldown.to_sec() <= 0.0:
            # fallback: if cooldown is zero we always try YOLO when HSV fails
            detections = self.detector.detect(frame)
            state = self.extract_state_from_detections(detections)
            self.last_yolo_stamp = now

        if state is None and hsv_state is not None:
            state = hsv_state

        if state is not None:
            self.current_state = state
            self.last_update = stamp if stamp != rospy.Time() else rospy.Time.now()
        else:
            if (
                self.unknown_timeout.to_sec() > 0
                and rospy.Time.now() - self.last_update > self.unknown_timeout
            ):
                self.current_state = "unknown"

        self.state_pub.publish(String(data=self.current_state))
        if self.overlay_pub is not None:
            self.publish_overlay(frame, detections)

    def extract_state_from_detections(self, detections) -> Optional[str]:
        best_score = -1.0
        best_state: Optional[str] = None

        for det in detections:
            label_lower = det.label.lower()
            if any(keyword in label_lower for keyword in self.ignored_label_keywords):
                continue

            state = self.LABEL_MAP.get(det.label)
            if state is None:
                for key, mapped in self.COLOR_KEYWORDS.items():
                    if key in label_lower:
                        state = mapped
                        break
            if state is None:
                continue
            if det.score > best_score:
                best_score = det.score
                best_state = state
        return best_state

    def detect_state_with_hsv(self, frame: np.ndarray) -> Optional[str]:
        roi = self.extract_roi(frame)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        if self.blur_kernel >= 3 and self.blur_kernel % 2 == 1:
            hsv = cv2.medianBlur(hsv, self.blur_kernel)

        kernel = None
        if self.morph_kernel_size >= 3:
            k = self.morph_kernel_size
            kernel = np.ones((k, k), np.uint8)

        ratios = {
            "red": self.compute_ratio(
                hsv,
                [
                    (self.lower_red_1, self.upper_red_1),
                    (self.lower_red_2, self.upper_red_2),
                ],
                kernel,
            ),
            "yellow": self.compute_ratio(hsv, [(self.lower_yellow, self.upper_yellow)], kernel),
            "green": self.compute_ratio(hsv, [(self.lower_green, self.upper_green)], kernel),
        }

        return self.decide_state(ratios, hsv)

    def extract_roi(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]

        roi_y = self.roi_y
        roi_h = self.roi_h

        if self.use_lane_split:
            usable_height = max(min(self.lane_roi_ratio, 1.0), 0.05)
            margin = np.clip(self.roi_margin, 0.0, usable_height - 0.02)
            roi_y = 0.0 + margin
            roi_h = max(usable_height - margin, 0.05)

        x = int(np.clip(self.roi_x * w, 0, w - 1))
        y = int(np.clip(roi_y * h, 0, h - 1))
        width = int(np.clip(self.roi_w * w, 1, w - x))
        height = int(np.clip(roi_h * h, 1, h - y))

        return frame[y : y + height, x : x + width]

    def compute_ratio(
        self,
        hsv: np.ndarray,
        ranges: Sequence[Tuple[np.ndarray, np.ndarray]],
        kernel: Optional[np.ndarray],
    ) -> float:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        if kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        ratio = float(cv2.countNonZero(mask)) / float(mask.size)
        return ratio

    def decide_state(self, ratios: Dict[str, float], hsv: np.ndarray) -> Optional[str]:
        sorted_states = sorted(ratios.items(), key=lambda item: item[1], reverse=True)
        best_state, best_ratio = sorted_states[0]
        second_ratio = sorted_states[1][1] if len(sorted_states) > 1 else 0.0

        if best_ratio >= self.min_ratio and (best_ratio - second_ratio) >= self.state_margin:
            return best_state

        brightness = float(np.mean(hsv[:, :, 2]))
        if brightness <= self.off_value_threshold:
            return "off"

        return None

    def publish_overlay(self, frame: np.ndarray, detections) -> None:
        if self.overlay_pub is None:
            return
        overlay = frame.copy()
        if detections:
            for det in detections:
                label_lower = det.label.lower()
                if any(keyword in label_lower for keyword in self.ignored_label_keywords):
                    continue

                state = self.LABEL_MAP.get(det.label)
                if state is None:
                    for key, mapped in self.COLOR_KEYWORDS.items():
                        if key in label_lower:
                            state = mapped
                            break
                if state not in ("red", "yellow", "green", "off"):
                    continue

                x1, y1, x2, y2 = det.bbox
                color = (0, 255, 0)
                if state == "red":
                    color = (0, 0, 255)
                elif state == "yellow":
                    color = (0, 255, 255)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
                text = f"{det.label} {det.score:.2f}"
                cv2.putText(
                    overlay,
                    text,
                    (x1, max(y1 - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    overlay,
                    text,
                    (x1, max(y1 - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        state_text = f"state: {self.current_state}"
        cv2.putText(
            overlay,
            state_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            state_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        try:
            msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            self.overlay_pub.publish(msg)
        except Exception as exc:
            rospy.logwarn("[traffic_light] overlay publish failed: %s", exc)

    def _load_label_map(self, param_dict) -> Dict[str, str]:
        mapping: Dict[str, str] = {}
        if isinstance(param_dict, dict):
            for key, value in param_dict.items():
                try:
                    mapping[str(key)] = str(value)
                except (TypeError, ValueError):
                    continue
        return mapping

    def spin(self) -> None:
        rospy.spin()


def main() -> None:
    rospy.init_node("traffic_light_node")
    TrafficLightNode().spin()


if __name__ == "__main__":
    main()
