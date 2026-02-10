"""
HSV + YOLO 결합 주행 launch file (듀얼 카메라).

카메라 2대로 역할 분리:
  - Front Camera (아래쪽): HSV 차선 검출 (30Hz)
  - Upper Camera (위쪽): YOLO 장애물/신호등 검출 (10Hz)

노드:
  1. USB Camera Front: 차선 인식용 이미지 발행
  2. USB Camera Upper: 장애물/신호등 인식용 이미지 발행
  3. Lane Tracking (HSV): 차선 중심 검출
  4. YOLOv8n-seg: 주행 가능 영역 + 장애물 검출
  5. Hybrid Lane Drive: HSV + YOLO 결합 제어
  6. Arduino Bridge: 모터 제어
  7. RViz2: 시각화
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for hybrid drive mode."""

    # Launch arguments
    front_camera_topic_arg = DeclareLaunchArgument(
        'front_camera_topic',
        default_value='/camera/front/image',
        description='Front camera topic (lane tracking)'
    )

    upper_camera_topic_arg = DeclareLaunchArgument(
        'upper_camera_topic',
        default_value='/camera/upper/image',
        description='Upper camera topic (YOLO obstacle/traffic light)'
    )

    front_video_device_arg = DeclareLaunchArgument(
        'front_video_device',
        default_value='/dev/video4',
        description='Front camera device path'
    )

    upper_video_device_arg = DeclareLaunchArgument(
        'upper_video_device',
        default_value='/dev/video6',
        description='Upper camera device path'
    )

    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed',
        default_value='0.3',
        description='Cruise speed in m/s'
    )

    slow_speed_arg = DeclareLaunchArgument(
        'slow_speed',
        default_value='0.2',
        description='Slow speed in m/s (during obstacle avoidance)'
    )

    max_steer_arg = DeclareLaunchArgument(
        'max_steer',
        default_value='0.45',
        description='Maximum steering angle in radians'
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='2.5',
        description='Proportional gain for steering control'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.6',
        description='Derivative gain for steering control'
    )

    use_traffic_light_arg = DeclareLaunchArgument(
        'use_traffic_light',
        default_value='true',
        description='Enable traffic light detection'
    )

    # 1. Front USB Camera (차선 인식용 - 아래쪽 시선)
    front_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_front',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'config',
                'usb_cam.yaml'
            ]),
            {
                'video_device': LaunchConfiguration('front_video_device'),
                'camera_name': 'front_camera',
                'frame_id': 'camera_front',
            }
        ],
        remappings=[
            ('image_raw', LaunchConfiguration('front_camera_topic')),
        ]
    )

    # 2. Upper USB Camera (장애물/신호등 인식용 - 위쪽 시선)
    upper_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_upper',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'config',
                'usb_cam_upper.yaml'
            ]),
            {
                'video_device': LaunchConfiguration('upper_video_device'),
                'camera_name': 'upper_camera',
                'frame_id': 'camera_upper',
            }
        ],
        remappings=[
            ('image_raw', LaunchConfiguration('upper_camera_topic')),
        ]
    )

    # 3. HSV Lane Tracking Node → Front Camera
    lane_tracking_node = Node(
        package='perception_pkg',
        executable='lane_tracking_node.py',
        name='lane_tracking',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('front_camera_topic'),
                'kp': 0.7,
                'debug': True,
                'roi_y_ratio': 0.55,
                'canny_low': 25,
                'canny_high': 80,
                'gaussian_kernel': 5,
                'avg_window': 2,
            }
        ]
    )

    # 4. YOLOv8n-seg Node → Upper Camera
    yolo_node = Node(
        package='perception_pkg',
        executable='yolov8n_seg_node.py',
        name='yolov8n_seg_node',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('upper_camera_topic'),
                'model_path': '/root/ros2_ws/src/perception_pkg/models/yolo26n_main.pt',
                'conf_threshold': 0.4,
                'iou_threshold': 0.45,
                'imgsz': 640,
                'publish_overlay': True,
                'publish_drivable_mask': True,
                'detect_obstacles': True,
                'rate_hz': 10.0,
            }
        ]
    )

    # 5. Hybrid Lane Drive Node (HSV + YOLO)
    hybrid_drive_node = Node(
        package='decision',
        executable='hybrid_lane_drive_node.py',
        name='hybrid_lane_drive',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('decision'),
                'config',
                'hybrid_lane_drive.yaml'
            ]),
            {
                'cruise_speed_mps': LaunchConfiguration('cruise_speed'),
                'slow_speed_mps': LaunchConfiguration('slow_speed'),
                'max_steer_rad': LaunchConfiguration('max_steer'),
                'kp': LaunchConfiguration('kp'),
                'kd': LaunchConfiguration('kd'),
                'use_traffic_light': LaunchConfiguration('use_traffic_light'),
            }
        ],
        remappings=[
            ('/decision/cmd', '/arduino/cmd'),
        ]
    )

    # 6. Arduino Bridge
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

    # 7. Static TF
    static_tf_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    static_tf_upper = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_upper',
        arguments=['0.15', '0', '0.18', '0', '0', '0', 'base_link', 'camera_upper']
    )

    # 8. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bringup'),
            'config',
            'adas_default.rviz'
        ])]
    )

    return LaunchDescription([
        # Launch arguments
        front_camera_topic_arg,
        upper_camera_topic_arg,
        front_video_device_arg,
        upper_video_device_arg,
        cruise_speed_arg,
        slow_speed_arg,
        max_steer_arg,
        kp_arg,
        kd_arg,
        use_traffic_light_arg,
        # Nodes
        front_cam_node,
        upper_cam_node,
        lane_tracking_node,
        yolo_node,
        hybrid_drive_node,
        arduino_node,
        static_tf_front,
        static_tf_upper,
        rviz_node,
    ])
