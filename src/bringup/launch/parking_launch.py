"""
Parking mode launch file.
주차 전용: decision_node 미포함 (parking_node가 직접 /arduino/cmd 제어)
전방 카메라 포함: RViz 모니터링용
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function for parking nodes."""

    nodes_to_launch = []

    # Arduino bridge node
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

    # Static TF: base_link -> laser (후방 장착 + 거꾸로 장착)
    # args: x y z yaw pitch roll parent child
    # yaw=π: LiDAR 전방 → 차량 후방 (전후 보정)
    # roll=π: LiDAR 뒤집힘 보정 (좌우 반전 보정)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['-0.15', '0', '0.05', '3.14159', '0', '3.14159', 'base_link', 'laser']
    )
    nodes_to_launch.append(static_tf_laser)

    # Static TF: base_link -> camera_front (전방 카메라)
    static_tf_camera_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )
    nodes_to_launch.append(static_tf_camera_front)

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for parking mode."""

    front_video_device_arg = DeclareLaunchArgument(
        'front_video_device',
        default_value='/dev/video4',
        description='Front camera video device'
    )

    # 전방 USB 카메라
    front_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'launch',
                'usb_cam_launch.py'
            ])
        ]),
        launch_arguments={
            'video_device': LaunchConfiguration('front_video_device'),
            'camera_topic': '/camera/front/image',
            'frame_id': 'camera_front',
        }.items()
    )

    # RPLiDAR driver (후방 장착)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_driver'),
                'launch',
                'rplidar_launch.py'
            ])
        ])
    )

    return LaunchDescription([
        front_video_device_arg,
        front_cam_launch,
        rplidar_launch,
        OpaqueFunction(function=launch_setup),
    ])
