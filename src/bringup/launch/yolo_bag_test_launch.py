"""
YOLO rosbag test launch file.
rosbag 파일을 재생하여 YOLO 세그멘테이션을 테스트하는 모드.
카메라 대신 rosbag에서 /camera/front/image 토픽을 재생하고,
    YOLOv8n-seg 노드 + rviz2로 시각화.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for YOLO rosbag test mode."""

    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ros2_ws/rosbag2_2026_02_08-04_45_24',
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

    yolo_imgsz_arg = DeclareLaunchArgument(
        'yolo_imgsz',
        default_value='512',
        description='YOLO input size (smaller = faster, lower detail)'
    )

    yolo_rate_hz_arg = DeclareLaunchArgument(
        'yolo_rate_hz',
        default_value='7.0',
        description='YOLO inference timer rate (Hz)'
    )

    yolo_publish_overlay_arg = DeclareLaunchArgument(
        'yolo_publish_overlay',
        default_value='true',
        description='Publish YOLO overlay image'
    )

    yolo_publish_drivable_mask_arg = DeclareLaunchArgument(
        'yolo_publish_drivable_mask',
        default_value='true',
        description='Publish drivable mask image'
    )

    # 1. Rosbag play (카메라 토픽만 재생)
    # loop 인자를 실제로 반영하기 위해 loop/non-loop 프로세스를 분리
    rosbag_play_loop = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--loop',
            '--rate', LaunchConfiguration('rate'),
            '--topics', '/camera/front/image',
        ],
        condition=IfCondition(LaunchConfiguration('loop')),
        output='screen'
    )

    rosbag_play_once = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--rate', LaunchConfiguration('rate'),
            '--topics', '/camera/front/image',
        ],
        condition=UnlessCondition(LaunchConfiguration('loop')),
        output='screen'
    )

    # 2. YOLOv8n-seg Node
    yolo_node = Node(
        package='perception_pkg',
        executable='yolov8n_seg_node.py',
        name='yolov8n_seg_node',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('camera_topic'),
                'model_path': '/root/ros2_ws/src/perception_pkg/models/yolo26n_line.pt',
                'conf_threshold': 0.4,
                'iou_threshold': 0.45,
                'imgsz': LaunchConfiguration('yolo_imgsz'),
                'publish_overlay': LaunchConfiguration('yolo_publish_overlay'),
                'publish_drivable_mask': LaunchConfiguration('yolo_publish_drivable_mask'),
                'detect_obstacles': True,
                'rate_hz': LaunchConfiguration('yolo_rate_hz'),
            }
        ]
    )

    # 3. Static TF: base_link -> camera_front
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    # 4. rviz2 시각화 (adas_default.rviz: YOLO overlay 포함)
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
        yolo_imgsz_arg,
        yolo_rate_hz_arg,
        yolo_publish_overlay_arg,
        yolo_publish_drivable_mask_arg,
        rosbag_play_loop,
        rosbag_play_once,
        yolo_node,
        static_tf_camera,
        rviz2,
    ])
