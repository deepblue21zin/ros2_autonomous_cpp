#!/usr/bin/env python3
"""
후방 카메라 주차 라인 감지 노드.
대회 규정: 수직 주차 완료 후 OUT 라인까지 주행
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

from perception_pkg.perception.parking.parking_line_detector import detect_parking_end_line


class ParkingLineNode(Node):
    """후방 카메라로 주차 종료 라인 감지."""

    def __init__(self):
        super().__init__('parking_line_node')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/rear/image')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('roi_y_start_ratio', 0.6)
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('min_line_length', 100)
        self.declare_parameter('debug', True)

        camera_topic = self.get_parameter('camera_topic').value
        use_compressed = self.get_parameter('use_compressed').value
        self.roi_y_start_ratio = self.get_parameter('roi_y_start_ratio').value
        self.canny_low = self.get_parameter('canny_low').value
        self.canny_high = self.get_parameter('canny_high').value
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.min_line_length = self.get_parameter('min_line_length').value
        self.debug = self.get_parameter('debug').value

        self.bridge = CvBridge()

        # Subscriber
        if use_compressed:
            from sensor_msgs.msg import CompressedImage
            self.image_sub = self.create_subscription(
                CompressedImage,
                camera_topic + '/compressed',
                self.compressed_image_callback,
                10
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                10
            )

        # Publishers
        self.line_detected_pub = self.create_publisher(
            Bool, '/parking/line_detected', 10
        )

        if self.debug:
            self.overlay_pub = self.create_publisher(
                Image, '/parking/overlay', 10
            )

        self.get_logger().info('Parking line detection node started')
        self.get_logger().info(f'  Camera topic: {camera_topic}')
        self.get_logger().info(f'  Use compressed: {use_compressed}')
        self.get_logger().info(f'  Debug mode: {self.debug}')

    def image_callback(self, msg: Image):
        """Raw 이미지 콜백."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        self.process_image(cv_image)

    def compressed_image_callback(self, msg):
        """Compressed 이미지 콜백."""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        self.process_image(cv_image)

    def process_image(self, cv_image):
        """이미지 처리 및 주차 라인 감지."""
        # 주차 라인 감지
        line_detected, overlay = detect_parking_end_line(
            cv_image,
            roi_y_start_ratio=self.roi_y_start_ratio,
            canny_low=self.canny_low,
            canny_high=self.canny_high,
            hough_threshold=self.hough_threshold,
            min_line_length=self.min_line_length,
            debug=self.debug
        )

        # 감지 결과 발행
        msg = Bool()
        msg.data = line_detected
        self.line_detected_pub.publish(msg)

        # 디버그 오버레이 발행
        if self.debug:
            try:
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                self.overlay_pub.publish(overlay_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish overlay: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingLineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
