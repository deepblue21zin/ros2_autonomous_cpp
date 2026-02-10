"""
USB Camera ROS2 launch file.
Launches the USB camera node with configuration.
Hardware settings (pixel_format, resolution, framerate 등)는 usb_cam.yaml에서 관리.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for USB camera driver."""

    # Launch arguments: 부모 launch에서 실제로 override하는 것만 선언
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video4',
        description='Video device path (default: front camera)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Topic name for camera images'
    )

    # Config file (pixel_format, resolution, framerate 등 하드웨어 설정의 source of truth)
    config_file = PathJoinSubstitution([
        FindPackageShare('usb_cam_driver'),
        'config',
        'usb_cam.yaml'
    ])

    # USB camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            config_file,
            {
                'video_device': LaunchConfiguration('video_device'),
                'camera_name': 'front_camera',
            }
        ],
        remappings=[
            ('image_raw', LaunchConfiguration('camera_topic')),
        ]
    )

    return LaunchDescription([
        video_device_arg,
        camera_topic_arg,
        usb_cam_node,
    ])
