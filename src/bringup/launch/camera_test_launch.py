"""
Camera test launch file.
카메라 + 차선 인식만 실행하여 rviz2로 확인하는 테스트 모드.
주행 관련 노드(arduino, ultrasonic, lidar, decision)는 실행하지 않음.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for camera test mode."""

    # Launch arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video4',
        description='Camera device path (test: /dev/video4, car: /dev/video6)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Camera topic name'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='false',
        description='Use C++ lane tracking node (false=Python)'
    )

    # 1. USB Camera
    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'launch',
                'usb_cam_launch.py'
            ])
        ]),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_topic': LaunchConfiguration('camera_topic'),
        }.items()
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

    # 4. rqt_image_view: Camera Raw (큰 화면)
    image_view_raw = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             '--ros-args', '-r', 'image:=/camera/front/image'],
        output='screen'
    )

    # 5. rqt_image_view: Lane Overlay (큰 화면)
    image_view_lane = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             '--ros-args', '-r', 'image:=/lane_overlay'],
        output='screen'
    )

    return LaunchDescription([
        video_device_arg,
        camera_topic_arg,
        use_cpp_arg,
        usb_cam_launch,
        lane_perception_launch,
        static_tf_camera,
        image_view_raw,
        image_view_lane,
    ])
