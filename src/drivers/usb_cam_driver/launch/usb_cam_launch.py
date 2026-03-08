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
from launch.conditions import IfCondition, UnlessCondition
import os
import glob


def _resolve_stable_device(default_device: str) -> str:
    """Resolve /dev/videoX to a stable /dev/v4l/by-* symlink when available."""
    if not os.path.exists(default_device):
        return default_device

    real_device = os.path.realpath(default_device)
    for base_dir in ('/dev/v4l/by-id', '/dev/v4l/by-path'):
        if not os.path.isdir(base_dir):
            continue
        for link_path in sorted(glob.glob(os.path.join(base_dir, '*'))):
            if os.path.islink(link_path) and os.path.realpath(link_path) == real_device:
                return link_path

    return default_device


def generate_launch_description():
    """Generate launch description for USB camera driver."""

    # Launch arguments: 부모 launch에서 실제로 override하는 것만 선언
    default_video_device = _resolve_stable_device('/dev/video4')

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value=default_video_device,
        description='Video device path (default: front camera)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Topic name for camera images'
    )

    enable_mjpeg_decoder_arg = DeclareLaunchArgument(
        'enable_mjpeg_decoder',
        default_value='true',
        description='Enable decode relay node (for compressed or packed raw streams)'
    )

    raw_camera_topic_arg = DeclareLaunchArgument(
        'raw_camera_topic',
        default_value='/camera/front/image_raw',
        description='Raw camera topic used before decode relay'
    )

    # Config file (pixel_format, resolution, framerate 등 하드웨어 설정의 source of truth)
    config_file = PathJoinSubstitution([
        FindPackageShare('usb_cam_driver'),
        'config',
        'usb_cam.yaml'
    ])

    # USB camera node (direct publish to output topic)
    usb_cam_node_direct = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('enable_mjpeg_decoder')),
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

    # USB camera node (publish raw stream to relay input topic)
    usb_cam_node_raw = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mjpeg_decoder')),
        parameters=[
            config_file,
            {
                'video_device': LaunchConfiguration('video_device'),
                'camera_name': 'front_camera',
            }
        ],
        remappings=[
            ('image_raw', LaunchConfiguration('raw_camera_topic')),
        ]
    )

    # Decode input Image bytes -> bgr8 Image
    mjpeg_decoder_node = Node(
        package='usb_cam_driver',
        executable='mjpeg_decoder_node.py',
        name='mjpeg_decoder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mjpeg_decoder')),
        parameters=[{
            'input_topic': LaunchConfiguration('raw_camera_topic'),
            'output_topic': LaunchConfiguration('camera_topic'),
        }]
    )

    return LaunchDescription([
        video_device_arg,
        camera_topic_arg,
        enable_mjpeg_decoder_arg,
        raw_camera_topic_arg,
        usb_cam_node_direct,
        usb_cam_node_raw,
        mjpeg_decoder_node,
    ])
