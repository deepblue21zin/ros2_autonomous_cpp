"""
HSV + YOLO 결합 Rosbag 테스트 launch file.

rosbag 파일을 재생하여 hybrid lane drive (HSV+YOLO)를 테스트하는 모드.
카메라 대신 rosbag에서 /camera/front/image 토픽을 재생하고,
HSV 차선 검출 + YOLO 세그멘테이션 + Hybrid Drive로 모터 제어 테스트.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for hybrid rosbag test mode."""

    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ros2_ws/rosbag2_2026_01_30-03_20_53',
        description='Path to rosbag2 directory'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Camera topic name'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop rosbag playback'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Rosbag playback rate'
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

    # 1. Rosbag play (카메라 토픽만 재생)
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--loop',
            '--rate', LaunchConfiguration('rate'),
            '--topics', '/camera/front/image',
        ],
        output='screen'
    )

    # 2. HSV Lane Tracking Node
    lane_tracking_node = Node(
        package='perception_pkg',
        executable='lane_tracking_node.py',
        name='lane_tracking',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('camera_topic'),
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

    # 3. YOLOv8n-seg Node
    yolo_node = Node(
        package='perception_pkg',
        executable='yolov8n_seg_node.py',
        name='yolov8n_seg_node',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('camera_topic'),
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

    # 4. Hybrid Lane Drive Node (HSV + YOLO)
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
                'use_traffic_light': False,  # rosbag 테스트 시 신호등 비활성화
            }
        ],
        remappings=[
            ('/decision/cmd', '/arduino/cmd'),
        ]
    )

    # 5. Arduino Bridge
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

    # 6. Static TF: base_link -> camera_front
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    # 7. rviz2 시각화 (adas_default.rviz: YOLO overlay + HSV overlay 포함)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'adas_default.rviz'
    ])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        camera_topic_arg,
        loop_arg,
        rate_arg,
        cruise_speed_arg,
        slow_speed_arg,
        max_steer_arg,
        kp_arg,
        kd_arg,
        rosbag_play,
        lane_tracking_node,
        yolo_node,
        hybrid_drive_node,
        arduino_node,
        static_tf_camera,
        rviz2,
    ])
