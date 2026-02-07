"""
Parking mode launch file.
Launches the rear-LiDAR based perpendicular parking system.
주차 전용: decision_node 미포함 (parking_node가 직접 /arduino/cmd 제어)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def launch_setup(context, *args, **kwargs):
    """Setup function for parking nodes."""

    nodes_to_launch = []

    # Arduino bridge node (항상 Python — C++ boost::asio 시리얼 문제)
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
            ]).perform(context)
        ]
    )
    nodes_to_launch.append(arduino_node)

    # Parking node (상태머신 + LiDAR 처리)
    parking_node = Node(
        package='decision',
        executable='parking_node',
        name='parking_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('decision'),
                'config',
                'parking_params.yaml'
            ]).perform(context)
        ],
        remappings=[
            ('/decision/cmd', '/arduino/cmd'),
        ]
    )
    nodes_to_launch.append(parking_node)

    # Parking line detection node (후방 카메라 → OUT 라인 감지)
    parking_line_node = Node(
        package='perception_pkg',
        executable='parking_line_node.py',
        name='parking_line_node',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('rear_camera_topic').perform(context),
                'use_compressed': False,
                'debug': True,
            }
        ]
    )
    nodes_to_launch.append(parking_line_node)

    # Static TF: base_link → laser (후방 장착)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['-0.15', '0', '0.05', '0', '0', '3.14159', 'base_link', 'laser']
        # 후방 15cm, 높이 5cm, 180도 회전 (LiDAR 전면이 차량 후방을 향함)
    )
    nodes_to_launch.append(static_tf_laser)

    # Static TF: base_link → camera_rear (후방 카메라)
    static_tf_camera_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_rear',
        arguments=['-0.15', '0', '0.08', '0', '0', '3.14159', 'base_link', 'camera_rear']
    )
    nodes_to_launch.append(static_tf_camera_rear)

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for parking mode."""

    rear_camera_topic_arg = DeclareLaunchArgument(
        'rear_camera_topic',
        default_value='/camera/rear/image',
        description='Rear camera topic for parking line detection'
    )

    rear_video_device_arg = DeclareLaunchArgument(
        'rear_video_device',
        default_value='/dev/video4',
        description='Rear camera video device'
    )

    # RPLiDAR driver
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_driver'),
                'launch',
                'rplidar_launch.py'
            ])
        ])
    )

    # Rear USB camera (단일 카메라)
    rear_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_rear',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('rear_video_device'),
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
        }],
        remappings=[
            ('image_raw', '/camera/rear/image'),
        ]
    )

    return LaunchDescription([
        rear_camera_topic_arg,
        rear_video_device_arg,
        rplidar_launch,
        rear_cam_node,
        OpaqueFunction(function=launch_setup),
    ])
