"""
Dual USB Camera ROS2 launch file.
Launches both front and rear USB camera nodes.
Hardware settings는 각각 usb_cam.yaml, usb_cam_rear.yaml에서 관리.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for dual USB cameras."""

    pkg_share = FindPackageShare('usb_cam_driver')

    # Launch arguments: 부모 launch에서 실제로 override하는 것만 선언
    front_video_device_arg = DeclareLaunchArgument(
        'front_video_device',
        default_value='/dev/video6',
        description='Front camera video device path'
    )

    rear_video_device_arg = DeclareLaunchArgument(
        'rear_video_device',
        default_value='/dev/video4',
        description='Rear camera video device path'
    )

    # Config files (하드웨어 설정의 source of truth)
    front_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'usb_cam.yaml'
    ])

    rear_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'usb_cam_rear.yaml'
    ])

    # Front USB camera node
    front_usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_front',
        output='screen',
        parameters=[
            front_config_file,
            {
                'video_device': LaunchConfiguration('front_video_device'),
                'camera_name': 'front_camera',
            }
        ],
        remappings=[
            ('image_raw', '/camera/front/image_raw'),
            ('camera_info', '/camera/front/camera_info'),
        ]
    )

    front_mjpeg_decoder_node = Node(
        package='usb_cam_driver',
        executable='mjpeg_decoder_node.py',
        name='front_mjpeg_decoder',
        output='screen',
        parameters=[{
            'input_topic': '/camera/front/image_raw',
            'output_topic': '/camera/front/image',
            'output_frame_id': 'camera_front',
        }]
    )

    # Rear USB camera node
    rear_usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_rear',
        output='screen',
        parameters=[
            rear_config_file,
            {
                'video_device': LaunchConfiguration('rear_video_device'),
                'camera_name': 'rear_camera',
            }
        ],
        remappings=[
            ('image_raw', '/camera/rear/image'),
            ('camera_info', '/camera/rear/camera_info'),
        ]
    )

    return LaunchDescription([
        front_video_device_arg,
        rear_video_device_arg,
        front_usb_cam_node,
        front_mjpeg_decoder_node,
        rear_usb_cam_node,
    ])
