#!/usr/bin/env python3
"""속도 표지판 인식 기반 제한속도 퍼블리셔."""

from __future__ import annotations

import re
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float32MultiArray

from perception_pkg.perception.object_detection.detector import ObjectDetector


class SpeedSignNode:
    """카메라 이미지를 입력으로 제한 속도를 퍼블리시."""

    SPEED_REGEX = re.compile(r"(\d+)")

    def __init__(self) -> None:
        self.bridge = CvBridge()

        # 파라미터
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.use_compressed = rospy.get_param("~use_compressed", False)
        self.default_speed = float(rospy.get_param("~default_speed_limit", 20.0))
        self.decay_timeout = rospy.Duration.from_sec(
            float(rospy.get_param("~decay_timeout", -1.0))
        )
        score_threshold = float(rospy.get_param("~score_threshold", 0.5))
        self.target_prefix = tuple(
            str(p)
            for p in rospy.get_param(
                "~label_prefixes",
                [
                    "speed limit ",
                    "speedlimit",
                    "speedlimit",
                ],
            )
        )
        self._target_prefix_normalized = tuple(
            prefix.lower().replace(" ", "").replace("_", "")
            for prefix in self.target_prefix
        )

        default_range_param = rospy.get_param(
            "~default_speed_range", [self.default_speed, self.default_speed]
        )
        self.default_range = self._parse_range_param(default_range_param, self.default_speed)
        self.speed_ranges = self._load_speed_ranges(
            rospy.get_param(
                "~speed_ranges",
                {
                    "speed_sign_30": [20.0, 30.0],
                    "speed_sign_40": [30.0, 40.0],
                    "speed_sign_50": [40.0, 50.0],
                    "30": [20.0, 30.0],
                    "40": [30.0, 40.0],
                    "50": [40.0, 50.0],
                    "KR_Sign_SL_30": [20.0, 30.0],
                    "KR_Sign_SL_40": [30.0, 40.0],
                    "KR_Sign_SL_50": [40.0, 50.0],
                },
            )
        )
        self.label_speed_map = self._load_label_speed_map(
            rospy.get_param(
                "~label_speed_map",
                {
                    "speed_sign_30": 21.0,
                    "speed_sign_40": 31.0,
                    "speed_sign_50": 41.0,
                    "30": 21.0,
                    "40": 31.0,
                    "50": 41.0,
                    "KR_Sign_SL_30": 21.0,
                    "KR_Sign_SL_40": 31.0,
                    "KR_Sign_SL_50": 41.0,
                    "speedlimit30": 21.0,
                    "speedlimit40": 31.0,
                    "speedlimit50": 41.0,
                    "speedlimit_30": 21.0,
                    "speedlimit_40": 31.0,
                    "speedlimit_50": 41.0,
                    "speedlimit 30": 21.0,
                    "speedlimit 40": 31.0,
                    "speedlimit 50": 41.0,
                    "speed limit 30": 21.0,
                    "speed limit 40": 31.0,
                    "speed limit 50": 41.0,
                    "speedlimit30kmh": 21.0,
                    "speedlimit40kmh": 31.0,
                    "speedlimit50kmh": 41.0,
                },
            )
        )

        detector_model = rospy.get_param("~detector_model_path", "")
        detector_imgsz = int(rospy.get_param("~detector_imgsz", 640))
        detector_device_param = rospy.get_param("~detector_device", "")
        detector_device = detector_device_param or None
        mapping_param = rospy.get_param(
            "~detector_label_mapping",
            {
                "speed_sign_30": "speed_sign_30",
                "speed_sign_40": "speed_sign_40",
                "speed_sign_50": "speed_sign_50",
                "speedlimit_30": "speed_sign_30",
                "speedlimit_40": "speed_sign_40",
                "speedlimit_50": "speed_sign_50",
                "KR_Sign_SL_30": "speed_sign_30",
                "KR_Sign_SL_40": "speed_sign_40",
                "KR_Sign_SL_50": "speed_sign_50",
            },
        )
        class_map = {str(k): str(v) for k, v in mapping_param.items()}

        self.pt_model_path = rospy.get_param("~pt_model_path", "")
        self.detector: Optional[ObjectDetector] = None

        try:
            if detector_model:
                self.detector = ObjectDetector(
                    score_threshold=score_threshold,
                    model_path=detector_model,
                    class_map=class_map,
                    imgsz=detector_imgsz,
                    device=detector_device,
                )
                rospy.loginfo(
                    "[speed_sign] detector model loaded: %s (imgsz=%d, device=%s)",
                    detector_model,
                    detector_imgsz,
                    detector_device or "auto",
                )
            elif self.pt_model_path:
                pt_conf_threshold = float(rospy.get_param("~pt_conf_threshold", 0.4))
                pt_iou_threshold = float(rospy.get_param("~pt_iou_threshold", 0.45))
                label_prefix = rospy.get_param(
                    "~pt_label_prefix",
                    self.target_prefix[0] if self.target_prefix else "speed_sign_",
                )
                class_names_param = rospy.get_param("~pt_class_names", [])
                class_names = (
                    tuple(str(name) for name in class_names_param) if class_names_param else None
                )
                label_map_param = rospy.get_param(
                    "~pt_label_map",
                    {
                        "Speed Limit 30": "speed_sign_30",
                        "Speed Limit 40": "speed_sign_40",
                        "Speed Limit 50": "speed_sign_50",
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
                    label_prefix=label_prefix,
                    class_names=class_names,
                    label_map=label_map if label_map else None,
                    device=device_arg,
                )
                self.detector = YoloSpeedSignPTDetector(config)
                rospy.loginfo(
                    "[speed_sign] YOLO PT detector loaded (model=%s, device=%s)",
                    self.pt_model_path,
                    device_arg or "auto",
                )
            else:
                self.detector = ObjectDetector(score_threshold=score_threshold)
        except Exception as exc:
            rospy.logerr("[speed_sign] detector 초기화 실패: %s", exc)
            raise

        self.limit_pub = rospy.Publisher("/perception/speed_limit", Float32, queue_size=1)
        self.range_pub = rospy.Publisher(
            "/perception/speed_limit_range", Float32MultiArray, queue_size=1
        )
        self.publish_overlay = rospy.get_param("~publish_overlay", True)
        self.overlay_pub = (
            rospy.Publisher("/perception/speed_sign_overlay", Image, queue_size=1)
            if self.publish_overlay
            else None
        )
        self.current_limit = self.default_speed
        self.current_range: Tuple[float, float] = self.default_range
        self.last_detection: rospy.Time = rospy.Time(0)
        self.current_label: Optional[str] = None
        self.last_valid_limit = self.default_speed
        self.last_valid_range: Tuple[float, float] = self.default_range

        if self.use_compressed:
            self.sub = rospy.Subscriber(
                self.camera_topic, CompressedImage, self.compressed_cb, queue_size=1
            )
        else:
            self.sub = rospy.Subscriber(
                self.camera_topic, Image, self.image_cb, queue_size=1
            )
        rospy.loginfo(
            "[speed_sign] subscribe: %s (compressed=%s)",
            self.camera_topic,
            self.use_compressed,
        )

    def compressed_cb(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("[speed_sign] JPEG decode failed.")
            return
        self.handle_frame(frame, msg.header.stamp)

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn("[speed_sign] cv_bridge error: %s", exc)
            return
        self.handle_frame(frame, msg.header.stamp)

    def handle_frame(self, frame: np.ndarray, stamp: rospy.Time) -> None:
        detections = self.detector.detect(frame)
        result = self.extract_speed_limit(detections)

        if result is not None:
            limit, label = result
            self.current_limit = limit
            self.current_label = label
            self.current_range = self._speed_range_for(label, limit)
            self.last_valid_limit = self.current_limit
            self.last_valid_range = self.current_range
            self.last_detection = stamp if stamp != rospy.Time() else rospy.Time.now()
        else:
            if (
                self.decay_timeout.to_sec() > 0
                and rospy.Time.now() - self.last_detection > self.decay_timeout
            ):
                self.current_limit = self.default_speed
                self.current_range = self.default_range
                self.current_label = None
            else:
                # 검출이 잠시 끊겨도 마지막 유효 표지판을 유지
                self.current_limit = self.last_valid_limit
                self.current_range = self.last_valid_range

        self.limit_pub.publish(Float32(data=float(self.current_limit)))
        self._publish_range()
        if self.overlay_pub is not None:
            self._publish_overlay(frame, detections)

    def extract_speed_limit(self, detections) -> Optional[Tuple[float, str]]:
        """라벨에서 속도 값을 추출."""
        best_score = -1.0
        best_limit: Optional[float] = None
        best_label: Optional[str] = None

        for det in detections:
            label_lower = det.label.lower()
            label_normalized = label_lower.replace(" ", "").replace("_", "")
            value = self._limit_for_label(det.label)
            if value is None:
                value = self._parse_speed(det.label)
            if value is None:
                continue
            if self.target_prefix and not any(
                prefix in label_normalized for prefix in self._target_prefix_normalized
            ):
                if (
                    not label_normalized.isdigit()
                    and det.label not in self.label_speed_map
                    and label_normalized not in self.label_speed_map
                ):
                    continue
            if det.score > best_score:
                best_score = det.score
                best_limit = value
                best_label = det.label
        if best_limit is None or best_label is None:
            return None
        return best_limit, best_label

    @classmethod
    def _parse_speed(cls, label: str) -> Optional[float]:
        match = cls.SPEED_REGEX.search(label)
        if not match:
            return None
        try:
            return float(match.group(1))
        except ValueError:  # pragma: no cover - 안전장치
            return None

    def _publish_range(self) -> None:
        msg = Float32MultiArray(data=[float(self.current_range[0]), float(self.current_range[1])])
        self.range_pub.publish(msg)

    def _publish_overlay(self, frame: np.ndarray, detections) -> None:
        if self.overlay_pub is None:
            return
        overlay = frame.copy()
        best_det = None
        best_score = -1.0
        best_limit = None

        for det in detections:
            value = self._limit_for_label(det.label)
            if value is None:
                value = self._parse_speed(det.label)
            if value is None:
                continue
            if det.score > best_score:
                best_det = det
                best_score = det.score
                best_limit = value

        if best_det is not None:
            x1, y1, x2, y2 = best_det.bbox
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 200, 255), 2)
            label_text = f"{best_det.label} ({best_det.score:.2f})"
            if best_limit is not None:
                label_text += f" -> {best_limit:.0f}"
            cv2.putText(
                overlay,
                label_text,
                (x1, max(y1 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                label_text,
                (x1, max(y1 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        try:
            msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            self.overlay_pub.publish(msg)
        except Exception as exc:
            rospy.logwarn("[speed_sign] overlay publish failed: %s", exc)

    def _parse_range_param(self, value, fallback: float) -> Tuple[float, float]:
        try:
            if isinstance(value, (list, tuple)) and len(value) == 2:
                return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            pass
        return (fallback, fallback)

    def _load_speed_ranges(self, param_dict) -> dict:
        ranges = {}
        if isinstance(param_dict, dict):
            for key, value in param_dict.items():
                ranges[str(key)] = self._parse_range_param(value, self.default_speed)
        return ranges

    def _load_label_map(self, param_dict) -> Dict[str, str]:
        mapping: Dict[str, str] = {}
        if isinstance(param_dict, dict):
            for key, value in param_dict.items():
                try:
                    mapping[str(key)] = str(value)
                except (TypeError, ValueError):
                    continue
        return mapping

    def _speed_range_for(self, label: str, limit: float) -> Tuple[float, float]:
        if label in self.speed_ranges:
            return self.speed_ranges[label]
        stripped_label = self._strip_prefix(label)
        if stripped_label in self.speed_ranges:
            return self.speed_ranges[stripped_label]
        numeric_key = str(int(round(limit)))
        if numeric_key in self.speed_ranges:
            return self.speed_ranges[numeric_key]
        return self.default_range

    def _strip_prefix(self, label: str) -> str:
        for prefix in self.target_prefix:
            if label.startswith(prefix):
                return label[len(prefix) :]
        normalized_label = label.lower().replace(" ", "").replace("_", "")
        for prefix in self._target_prefix_normalized:
            if normalized_label.startswith(prefix):
                return normalized_label[len(prefix) :]
        return label

    def _load_label_speed_map(self, param_dict) -> Dict[str, float]:
        mapping: Dict[str, float] = {}
        if isinstance(param_dict, dict):
            for key, value in param_dict.items():
                try:
                    mapping[str(key)] = float(value)
                except (TypeError, ValueError):
                    continue
        return mapping

    def _limit_for_label(self, label: str) -> Optional[float]:
        if label in self.label_speed_map:
            return self.label_speed_map[label]
        stripped = self._strip_prefix(label)
        if stripped in self.label_speed_map:
            return self.label_speed_map[stripped]
        normalized = label.lower().replace(" ", "").replace("_", "")
        if normalized in self.label_speed_map:
            return self.label_speed_map[normalized]
        stripped_norm = self._strip_prefix(label).lower().replace(" ", "").replace("_", "")
        if stripped_norm in self.label_speed_map:
            return self.label_speed_map[stripped_norm]
        return None

    def spin(self) -> None:
        rospy.spin()


def main() -> None:
    rospy.init_node("speed_sign_node")
    SpeedSignNode().spin()


if __name__ == "__main__":
    main()
