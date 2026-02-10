"""
Camera bag test launch file.
rosbag 파일을 재생하여 차선 인식을 테스트하는 모드.
카메라 대신 rosbag에서 /camera/front/image 토픽을 재생하고,
C++ lane_tracking_node + rviz2로 시각화.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for camera bag test mode."""

    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ros2_ws/rosbag2_2026_01_30-03_08_19',
        description='Path to rosbag2 directory'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Camera topic name'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Use C++ lane tracking node (true=C++, false=Python)'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop rosbag playback'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Rosbag playback rate (0.5=half speed, 2.0=double speed)'
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

    # 2. Lane perception (차선 인식 - C++ 기본)
    lane_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('perception_pkg'),
                'launch',
                'lane_bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_topic': LaunchConfiguration('camera_topic'),
            'lane_marking_enabled': 'true',
            'traffic_light_enabled': 'false',
            'obstacle_enabled': 'false',
            'speed_sign_enabled': 'false',
            'use_cpp': LaunchConfiguration('use_cpp'),
        }.items()
    )

    # 3. Static TF: base_link -> camera_front
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    # 4. rviz2 시각화 (camera_test.rviz 설정 사용)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'camera_test.rviz'
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
        use_cpp_arg,
        loop_arg,
        rate_arg,
        rosbag_play,
        lane_perception_launch,
        static_tf_camera,
        rviz2,
    ])
