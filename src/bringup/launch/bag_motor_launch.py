"""
Bag motor test launch file.
rosbag 파일을 재생하여 차선 인식 + 모터 구동까지 테스트하는 모드.
카메라/라이다 대신 rosbag에서 토픽을 재생하고,
lane_tracking → decision → arduino_bridge로 실제 모터를 구동.
rviz2로 시각화.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for bag motor test mode."""

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
        description='Use C++ nodes (true=C++, false=Python)'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Rosbag playback rate (0.5=half speed, 2.0=double speed)'
    )

    # 1. Rosbag play (카메라 + 라이다 토픽만 재생, 제어 토픽 제외)
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--loop',
            '--rate', LaunchConfiguration('rate'),
            '--topics', '/camera/front/image', '/scan',
        ],
        output='screen'
    )

    # 2. Lane perception (차선 인식)
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
            'use_compressed': 'false',
            'lane_marking_enabled': 'false',
            'lane_debug': 'true',
            'traffic_light_enabled': 'false',
            'obstacle_enabled': 'false',
            'speed_sign_enabled': 'false',
            'use_cpp': LaunchConfiguration('use_cpp'),
        }.items()
    )

    # 3. LiDAR obstacle node (rosbag /scan 토픽 사용)
    lidar_obstacle_node = Node(
        package='decision',
        executable='lidar_obstacle_node',
        name='lidar_obstacle',
        output='screen'
    )

    # 4. Decision node (조향/속도 결정 → /arduino/cmd 발행)
    decision_node = Node(
        package='decision',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'stop_on_yellow': False,
            'test_mode': False,
            'lane_timeout_sec': 2.0,  # rosbag 처리 지연 허용 (기본 0.5 → 2.0)
        }],
        remappings=[
            ('/decision/cmd', '/arduino/cmd'),
        ]
    )

    # 5. Arduino bridge (모터 구동)
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
            ])
        ]
    )

    # 6. Static TFs
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )

    # 7. rviz2 시각화
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
        rate_arg,
        rosbag_play,
        lane_perception_launch,
        lidar_obstacle_node,
        decision_node,
        arduino_node,
        static_tf_camera,
        static_tf_laser,
        rviz2,
    ])
