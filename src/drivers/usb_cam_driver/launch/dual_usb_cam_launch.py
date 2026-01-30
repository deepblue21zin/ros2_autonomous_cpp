"""
Dual USB Camera ROS2 launch file.
Launches both front and rear USB camera nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for dual USB cameras."""

    # Get package share directory
    pkg_share = FindPackageShare('usb_cam_driver')

    # Declare launch arguments
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

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width in pixels'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height in pixels'
    )

    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='yuyv',
        description='Pixel format (yuyv, mjpeg, etc.)'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate in Hz'
    )

    # Front camera config
    front_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'usb_cam.yaml'
    ])

    # Rear camera config
    rear_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'usb_cam_rear.yaml'
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
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'framerate': LaunchConfiguration('framerate'),
                'frame_id': 'camera_front',
                'camera_name': 'front_camera',
            }
        ],
        remappings=[
            ('image_raw', '/camera/front/image'),
            ('camera_info', '/camera/front/camera_info'),
        ]
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
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'framerate': LaunchConfiguration('framerate'),
                'frame_id': 'camera_rear',
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
        image_width_arg,
        image_height_arg,
        pixel_format_arg,
        framerate_arg,
        front_usb_cam_node,
        rear_usb_cam_node,
    ])
