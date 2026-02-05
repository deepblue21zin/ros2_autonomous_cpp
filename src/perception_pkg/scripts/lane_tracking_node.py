#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차선 인지 및 스티어링 산출 노드 (ROS2). cv_bridge 미사용."""

from collections import deque
from typing import Deque, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage

from perception_pkg.perception.lane.preprocess import preprocess_image
from perception_pkg.perception.lane.detector import detect_lane_center


def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> OpenCV BGR (cv_bridge 없이 직접 변환)."""
    dtype = np.uint8
    channels = int(len(msg.data) / (msg.height * msg.width))
    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'bgr8':
        return img.copy()
    elif msg.encoding == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif msg.encoding in ('yuyv', 'uyvy', 'yuy2', 'yuv422_yuy2'):
        img_yuyv = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(img_yuyv, cv2.COLOR_YUV2BGR_YUYV)
    else:
        # 기타 인코딩은 그대로 시도
        return img.copy()


def cv2_to_imgmsg(frame: np.ndarray, header=None) -> Image:
    """OpenCV BGR -> sensor_msgs/Image (cv_bridge 없이 직접 변환)."""
    msg = Image()
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    if header is not None:
        msg.header = header
    return msg


class LaneTrackingNode(Node):
    """주요 라인트래킹 로직 클래스 (ROS2, cv_bridge 미사용)."""

    def __init__(self) -> None:
        super().__init__('lane_tracking_node')

        # 파라미터 선언
        self.declare_parameter('camera_topic', '/camera/front/image')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('kp', 0.7)
        self.declare_parameter('debug', True)
        self.declare_parameter('roi_y_ratio', 0.55)
        self.declare_parameter('canny_low', 25)
        self.declare_parameter('canny_high', 80)
        self.declare_parameter('gaussian_kernel', 5)
        self.declare_parameter('avg_window', 2)

        # 파라미터 로드
        self.camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.kp = self.get_parameter('kp').value
        self.debug = self.get_parameter('debug').value
        self.roi_y_ratio = self.get_parameter('roi_y_ratio').value
        self.canny_low = self.get_parameter('canny_low').value
        self.canny_high = self.get_parameter('canny_high').value
        self.gaussian_kernel = self.get_parameter('gaussian_kernel').value
        self.avg_window = self.get_parameter('avg_window').value

        # QoS: 이미지는 Best Effort, 큐 1
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 퍼블리셔
        self.offset_pub = self.create_publisher(Float32, '/lane/center_offset', 1)
        self.steer_pub = self.create_publisher(Float32, '/lane/steering_angle', 1)
        self.overlay_pub = self.create_publisher(Image, '/lane_overlay', 1)

        self.offset_buffer: Deque[float] = deque(maxlen=self.avg_window)

        # 이미지 타입에 맞는 콜백 등록
        if self.use_compressed:
            self.sub = self.create_subscription(
                CompressedImage, self.camera_topic,
                self.compressed_cb, image_qos)
        else:
            self.sub = self.create_subscription(
                Image, self.camera_topic,
                self.image_cb, image_qos)

        self.get_logger().info(
            f'[lane_tracking] subscribe: {self.camera_topic} '
            f'(compressed={self.use_compressed})')

    def update_params(self) -> None:
        """파라미터 값 갱신 (동적 튜닝)."""
        self.kp = self.get_parameter('kp').value
        self.roi_y_ratio = self.get_parameter('roi_y_ratio').value
        self.debug = self.get_parameter('debug').value

    def compressed_cb(self, msg: CompressedImage) -> None:
        """압축 이미지 콜백."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('JPEG 디코드 실패')
            return
        self.handle_frame(frame, msg.header)

    def image_cb(self, msg: Image) -> None:
        """RAW 이미지 콜백."""
        try:
            frame = imgmsg_to_cv2(msg)
        except Exception as exc:
            self.get_logger().warn(f'이미지 변환 실패: {exc}')
            return
        self.handle_frame(frame, msg.header)

    def handle_frame(self, frame: np.ndarray, header) -> None:
        """공통 프레임 처리 로직."""
        self.update_params()

        # 전처리: ROI 추출 및 엣지 생성
        edges, roi_color, roi_y = preprocess_image(
            frame, self.roi_y_ratio, self.canny_low, self.canny_high,
            self.gaussian_kernel)

        # 차선 검출: 허프 변환 기반 중심 추정
        lane_center, roi_overlay = detect_lane_center(edges, roi_color, self.debug)

        # 원본 영상 위에 ROI와 차선 합성
        overlay_frame = self.compose_overlay(frame, roi_overlay, roi_y, lane_center)

        # 스티어링 각도 계산
        steering, smooth_offset = self.compute_steering(lane_center, frame.shape[1])

        # 결과 퍼블리시
        self.publish_outputs(steering, smooth_offset, overlay_frame, header)

    def compose_overlay(self, frame: np.ndarray, roi_overlay: np.ndarray,
                        roi_y: int, lane_center: float) -> np.ndarray:
        """ROI 오버레이를 원본 이미지에 합성."""
        overlay = frame.copy()
        overlay[roi_y:, :] = cv2.addWeighted(
            overlay[roi_y:, :], 0.4, roi_overlay, 0.6, 0)

        width = frame.shape[1]
        cv2.line(overlay, (0, roi_y), (width - 1, roi_y), (255, 255, 0), 2)
        img_center = width // 2
        cv2.line(overlay, (img_center, roi_y), (img_center, frame.shape[0] - 1),
                 (0, 255, 255), 2)
        cv2.line(overlay, (int(lane_center), roi_y),
                 (int(lane_center), frame.shape[0] - 1), (0, 255, 0), 2)
        return overlay

    def compute_steering(self, lane_center: float, width: int) -> Tuple[float, float]:
        """차선 중심 오프셋 기반 비례 제어."""
        img_center = width / 2.0
        offset = lane_center - img_center
        offset_norm = float(np.clip(offset / img_center, -1.0, 1.0))
        self.offset_buffer.append(offset_norm)
        smooth_offset = float(np.mean(self.offset_buffer))

        steering = float(np.clip(self.kp * (-smooth_offset), -1.0, 1.0))
        return steering, smooth_offset

    def publish_outputs(self, steering: float, offset: float,
                        overlay_frame: np.ndarray, header) -> None:
        """계산 결과를 ROS2 토픽으로 송신."""
        offset_msg = Float32()
        offset_msg.data = offset
        self.offset_pub.publish(offset_msg)

        steer_msg = Float32()
        steer_msg.data = steering
        self.steer_pub.publish(steer_msg)

        if self.debug:
            try:
                overlay_msg = cv2_to_imgmsg(overlay_frame, header)
                self.overlay_pub.publish(overlay_msg)
            except Exception as exc:
                self.get_logger().warn(f'오버레이 퍼블리시 실패: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
