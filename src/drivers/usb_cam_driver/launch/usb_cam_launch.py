"""
USB Camera ROS2 launch file.
Launches the USB camera node with configuration.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for USB camera driver."""

    # Declare launch arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path'
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
        default_value='30',
        description='Camera framerate in Hz'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_front',
        description='Frame ID for camera'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Topic name for camera images'
    )

    # Get package share directory
    pkg_share = FindPackageShare('usb_cam_driver')

    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'usb_cam.yaml'
    ])

    # USB camera node (using usb_cam package)
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            config_file,
            {
                'video_device': LaunchConfiguration('video_device'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'framerate': LaunchConfiguration('framerate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_name': 'front_camera',
            }
        ],
        remappings=[
            ('image_raw', LaunchConfiguration('camera_topic')),
        ]
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        pixel_format_arg,
        framerate_arg,
        frame_id_arg,
        camera_topic_arg,
        usb_cam_node,
    ])
