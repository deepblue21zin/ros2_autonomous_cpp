"""
RPLiDAR ROS2 launch file.
Launches the RPLiDAR node with configuration from YAML file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RPLiDAR driver."""

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLiDAR'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Serial baudrate for RPLiDAR'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for laser scan'
    )

    # Get package share directory
    pkg_share = FindPackageShare('rplidar_driver')

    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'rplidar.yaml'
    ])

    # RPLiDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidarNode',
        output='screen',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
            }
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        rplidar_node,
    ])
