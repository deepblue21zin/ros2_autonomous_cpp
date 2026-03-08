"""
YOLO Lane Tracking Test Launch File.

YOLO segmentation으로 차선을 인식하고 추적하는 시스템 테스트용.
HSV 기반 시스템과 별개로 작동합니다.

Architecture:
  [Camera] → [YOLOv8n-seg] → /perception/lane_left_mask
                             → /perception/lane_right_mask
                                      ↓
                      [YOLO Lane Tracking Node]
                       (Predict+Hold + Polynomial)
                                      ↓
                              /lane/steering_angle
                                      ↓
                              [Decision Node]
                                      ↓
                              [Arduino Bridge]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for YOLO lane tracking test."""

    # ── Launch arguments ──
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Camera topic name'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/root/ros2_ws/src/perception_pkg/models/yolo26n_main.pt',
        description='YOLO model path (must be trained with lane classes)'
    )

    max_lost_frames_arg = DeclareLaunchArgument(
        'max_lost_frames',
        default_value='15',
        description='Predict+Hold max frames'
    )

    avg_window_arg = DeclareLaunchArgument(
        'avg_window',
        default_value='5',
        description='Moving average window size'
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='2.5',
        description='P gain'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='1.5',
        description='D gain'
    )

    # ── 1. USB Camera (또는 rosbag) ──
    usb_cam_node = Node(
        package='usb_cam_driver',
        executable='usb_cam_node.py',
        name='usb_cam_front',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'config',
                'usb_cam.yaml'
            ])
        ],
        remappings=[
            ('/camera/image', LaunchConfiguration('camera_topic')),
        ]
    )

    # ── 2. YOLOv8n-seg Node (차선 마스크 발행) ──
    yolo_node = Node(
        package='perception_pkg',
        executable='yolov8n_seg_node.py',
        name='yolov8n_seg_node',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('camera_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'conf_threshold': 0.4,
                'iou_threshold': 0.45,
                'imgsz': 640,
                'publish_overlay': True,
                'publish_drivable_mask': False,      # Drivable mask 비활성화
                'publish_lane_masks': True,          # 차선 마스크 활성화!
                'detect_obstacles': False,
                'rate_hz': 10.0,
            }
        ]
    )

    # ── 3. YOLO Lane Tracking Node (차선 추적 + 연속성 보장) ──
    yolo_lane_tracking_node = Node(
        package='decision',
        executable='yolo_lane_tracking_node.py',
        name='yolo_lane_tracking',
        output='screen',
        parameters=[
            {
                'max_lost_frames': LaunchConfiguration('max_lost_frames'),
                'avg_window': LaunchConfiguration('avg_window'),
                'kp': LaunchConfiguration('kp'),
                'kd': LaunchConfiguration('kd'),
                'lane_width_ratio': 0.55,
                'min_points': 10,
                'publish_overlay': True,
                'control_hz': 20.0,
            }
        ]
    )

    # ── 4. Decision Node ──
    decision_node = Node(
        package='decision',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'cruise_speed_mps': 1.0,
            'max_steer_rad': 0.6,
            'use_traffic_light': False,
            'use_obstacle_avoidance': False,
            'test_mode': False,
            'lane_timeout_sec': 1.0,
        }],
        remappings=[
            ('/decision/cmd', '/arduino/cmd'),
        ]
    )

    # ── 5. Arduino Bridge ──
    arduino_node = Node(
        package='arduino_driver',
        executable='arduino_bridge_node.py',
        name='arduino_bridge',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('arduino_driver'),
                'config',
                'arduino.yaml'
            ])
        ]
    )

    # ── 6. Static TF ──
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    return LaunchDescription([
        # Launch arguments
        camera_topic_arg,
        model_path_arg,
        max_lost_frames_arg,
        avg_window_arg,
        kp_arg,
        kd_arg,

        # Nodes
        usb_cam_node,
        yolo_node,
        yolo_lane_tracking_node,
        decision_node,
        arduino_node,
        static_tf_camera,
    ])
