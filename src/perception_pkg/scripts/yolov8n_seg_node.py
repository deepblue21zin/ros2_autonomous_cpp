#!/usr/bin/env python3
"""YOLOv8n-seg ROS2 노드.

기능:
  1. 카메라 토픽 구독 → YOLOv8n-seg 추론
  2. 신호등 감지 + HSV 색상 분석 → /perception/traffic_light_state 발행
  3. 장애물 감지 → /perception/yolo_obstacles 발행
  4. 주행 가능 영역 마스크 → /perception/drivable_mask 발행
  5. 디버그 오버레이 → /yolo/overlay 발행 (rviz2 Image 패널)
"""

from __future__ import annotations

import os
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32MultiArray
from cv_bridge import CvBridge

from perception_pkg.perception.object_detection.detector_yolov8n import (
    Yolov8nSegDetector,
    Yolov8nSegConfig,
)


class Yolov8nSegNode(Node):
    """YOLOv8n-seg 통합 감지 ROS2 노드."""

    def __init__(self) -> None:
        super().__init__("yolov8n_seg_node")

        # ── 파라미터 선언 ──
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_topic", "/camera/front/image")
        self.declare_parameter("conf_threshold", 0.4)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("publish_drivable_mask", True)
        self.declare_parameter("publish_lane_masks", False)  # 차선 마스크 발행 (새로 추가)
        self.declare_parameter("detect_obstacles", True)
        self.declare_parameter("traffic_light_min_ratio", 0.05)
        self.declare_parameter("rate_hz", 10.0)

        # ── 파라미터 로드 ──
        model_path = self.get_parameter("model_path").value
        camera_topic = self.get_parameter("camera_topic").value
        conf_threshold = self.get_parameter("conf_threshold").value
        iou_threshold = self.get_parameter("iou_threshold").value
        imgsz = self.get_parameter("imgsz").value
        device = self.get_parameter("device").value or None
        self.publish_overlay = self.get_parameter("publish_overlay").value
        self.publish_drivable = self.get_parameter("publish_drivable_mask").value
        self.publish_lanes = self.get_parameter("publish_lane_masks").value
        detect_obstacles = self.get_parameter("detect_obstacles").value
        tl_min_ratio = self.get_parameter("traffic_light_min_ratio").value
        rate_hz = self.get_parameter("rate_hz").value

        # 모델 경로 자동 탐색 (lane_traffic_light_1st.pt 우선)
        if not model_path or not os.path.exists(model_path):
            search_paths = [
                "/root/ros2_ws/src/perception_pkg/models/yolo26n_main.pt",
                #"/root/ros2_ws/src/perception_pkg/models/best.pt",
               # os.path.join(os.path.dirname(__file__),
                #             "..", "models", "yolo26n_1st.pt"),
                #os.path.join(os.path.dirname(__file__),
                #             "..", "models", "best.pt"),
                #"/root/ros2_ws/src/perception_pkg/models/yolov8n-seg.pt",
                #os.path.join(os.path.dirname(__file__),
                #             "..", "models", "yolov8n-seg.pt"),
            ]
            for p in search_paths:
                if os.path.exists(p):
                    model_path = p
                    break

        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f"모델 파일 없음: {model_path}")
            raise RuntimeError(f"모델 파일 없음: {model_path}")

        # ── 감지기 초기화 ──
        config = Yolov8nSegConfig(
            model_path=model_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            imgsz=imgsz,
            device=device,
            traffic_light_min_ratio=tl_min_ratio,
            use_seg_mask=True,
            detect_obstacles=detect_obstacles,
        )
        self.detector = Yolov8nSegDetector(config)
        self.bridge = CvBridge()

        # ── 퍼블리셔 ──
        self.tl_state_pub = self.create_publisher(
            String, "/perception/traffic_light_state", 1)
        self.tl_flag_pub = self.create_publisher(
            Bool, "/perception/traffic_light_detected", 1)
        self.obstacle_pub = self.create_publisher(
            Float32MultiArray, "/perception/yolo_obstacles", 1)

        if self.publish_overlay:
            self.overlay_pub = self.create_publisher(
                Image, "/yolo/overlay", 1)
        else:
            self.overlay_pub = None

        if self.publish_drivable:
            self.drivable_pub = self.create_publisher(
                Image, "/perception/drivable_mask", 1)
        else:
            self.drivable_pub = None

        if self.publish_lanes:
            self.lane_left_pub = self.create_publisher(
                Image, "/perception/lane_left_mask", 1)
            self.lane_right_pub = self.create_publisher(
                Image, "/perception/lane_right_mask", 1)
        else:
            self.lane_left_pub = None
            self.lane_right_pub = None

        # ── 서브스크라이버 ──
        self.latest_frame: Optional[np.ndarray] = None
        self.create_subscription(
            Image, camera_topic, self._image_cb, 1)

        # ── 추론 타이머 (프레임 레이트 제한) ──
        period = 1.0 / max(rate_hz, 1.0)
        self.create_timer(period, self._inference_cb)

        self.get_logger().info(
            f"[yolov8n_seg] 초기화 완료: model={model_path}, "
            f"conf={conf_threshold}, rate={rate_hz}Hz, "
            f"camera={camera_topic}"
        )

    def _image_cb(self, msg: Image) -> None:
        """카메라 이미지 수신."""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

    def _inference_cb(self) -> None:
        """타이머 콜백: YOLO 추론 실행."""
        frame = self.latest_frame
        if frame is None:
            return

        # YOLO 추론 (1회만 실행, 결과 재사용)
        results = self.detector.predict(frame)

        # 1. 신호등 감지
        tl = self.detector.detect_traffic_light(frame, results)
        tl_msg = String()
        tl_flag = Bool()

        if tl is not None:
            tl_msg.data = tl.state
            tl_flag.data = True
            # TrafficLightState에는 color_ratio가 없으므로 로그에서 제외
            self.get_logger().info(
                f"신호등: {tl.state} (conf={tl.confidence:.2f})",
                throttle_duration_sec=1.0,
            )
        else:
            tl_msg.data = "unknown"
            tl_flag.data = False

        self.tl_state_pub.publish(tl_msg)
        self.tl_flag_pub.publish(tl_flag)

        # 2. 장애물 감지
        obstacles = self.detector.detect_obstacles(frame, results)
        if obstacles:
            obs_msg = Float32MultiArray()
            for obs in obstacles:
                x1, y1, x2, y2 = obs.bbox
                obs_msg.data.extend([
                    float(x1), float(y1), float(x2), float(y2),
                    obs.confidence
                ])
            self.obstacle_pub.publish(obs_msg)

        # 3. 주행 가능 영역 마스크
        if self.drivable_pub is not None:
            drivable = self.detector.get_drivable_mask(frame, results)
            drivable_msg = self.bridge.cv2_to_imgmsg(drivable, "mono8")
            self.drivable_pub.publish(drivable_msg)

        # 3.5. 차선 마스크 (좌/우 분리)
        if self.lane_left_pub is not None and self.lane_right_pub is not None:
            left_mask, right_mask = self._extract_lane_masks(frame, results)
            if left_mask is not None:
                left_msg = self.bridge.cv2_to_imgmsg(left_mask, "mono8")
                self.lane_left_pub.publish(left_msg)
            if right_mask is not None:
                right_msg = self.bridge.cv2_to_imgmsg(right_mask, "mono8")
                self.lane_right_pub.publish(right_msg)

        # 4. 디버그 오버레이
        if self.overlay_pub is not None:
            overlay = self.detector.draw_overlay(frame, results)

            # 신호등 상태 텍스트 표시
            if tl is not None:
                color_map = {
                    "red": (0, 0, 255),
                    "green": (0, 255, 0),
                    "yellow": (0, 255, 255),
                }
                text = f"SIGNAL: {tl.state.upper()}"
                text_color = color_map.get(tl.state, (255, 255, 255))
                cv2.putText(overlay, text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            text_color, 2, cv2.LINE_AA)

            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            self.overlay_pub.publish(overlay_msg)

    def _extract_lane_masks(self, frame: np.ndarray, results) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """YOLO segmentation 결과에서 좌/우 차선 마스크 추출.

        Args:
            frame: 원본 이미지
            results: YOLO 추론 결과

        Returns:
            (left_mask, right_mask): 좌/우 차선 마스크 (각각 uint8, 0 or 255)
        """
        h, w = frame.shape[:2]
        left_mask = np.zeros((h, w), dtype=np.uint8)
        right_mask = np.zeros((h, w), dtype=np.uint8)

        # YOLO 결과에서 segmentation 마스크가 있는지 확인
        if results is None or not hasattr(results[0], 'masks') or results[0].masks is None:
            return None, None

        result = results[0]  # 첫 번째 결과 (배치 크기 1)

        # 각 탐지된 객체에 대해
        for i, cls_id in enumerate(result.boxes.cls):
            cls_id = int(cls_id.item())

            # 클래스 이름 확인 (YOLO 모델의 클래스 이름에 따라 조정 필요)
            # 예: 'lane_left' = 0, 'lane_right' = 1 (모델에 따라 다름)
            class_name = result.names.get(cls_id, "")

            # Segmentation 마스크 추출
            mask = result.masks.data[i].cpu().numpy()

            # 원본 이미지 크기로 리사이즈
            if mask.shape != (h, w):
                mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

            # uint8로 변환 (0 or 255)
            mask_uint8 = (mask * 255).astype(np.uint8)

            # 클래스 이름에 따라 좌/우 분류
            if 'left' in class_name.lower() or cls_id == 0:  # 좌차선
                left_mask = np.maximum(left_mask, mask_uint8)
            elif 'right' in class_name.lower() or cls_id == 1:  # 우차선
                right_mask = np.maximum(right_mask, mask_uint8)

        # 마스크가 비어있으면 None 반환
        left_mask = left_mask if np.any(left_mask) else None
        right_mask = right_mask if np.any(right_mask) else None

        return left_mask, right_mask


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Yolov8nSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
