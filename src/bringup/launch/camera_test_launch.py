"""
Camera test launch file.
듀얼 카메라 + 차선 인식 + YOLO 실행하여 rviz2로 확인하는 테스트 모드.
- front_video_device (video4) → lane tracking → /lane_overlay
- upper_video_device (video6) → YOLO → /yolo/overlay
주행 관련 노드(arduino, ultrasonic, lidar, decision)는 실행하지 않음.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
import yaml
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
    """Generate launch description for camera test mode."""

    # Load use_rviz from lane_params.yaml
    perception_pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')
    lane_params_file = os.path.join(perception_pkg_share, 'config', 'lane_params.yaml')
    with open(lane_params_file, 'r') as f:
        lane_params = yaml.safe_load(f)
    use_rviz_default = str(lane_params['/**']['ros__parameters'].get('use_rviz', False)).lower()

    # Launch arguments
    front_default_device = _resolve_stable_device('/dev/video4')
    upper_default_device = _resolve_stable_device('/dev/video6')

    front_video_device_arg = DeclareLaunchArgument(
        'front_video_device',
        default_value=front_default_device,
        description='Front camera device (lane tracking)'
    )

    upper_video_device_arg = DeclareLaunchArgument(
        'upper_video_device',
        default_value=upper_default_device,
        description='Upper camera device (YOLO)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Front camera topic name'
    )

    front_raw_camera_topic_arg = DeclareLaunchArgument(
        'front_raw_camera_topic',
        default_value='/camera/front/image_raw',
        description='Front raw image topic before decode relay'
    )

    upper_camera_topic_arg = DeclareLaunchArgument(
        'upper_camera_topic',
        default_value='/camera/upper/image',
        description='Upper camera topic name'
    )

    upper_raw_camera_topic_arg = DeclareLaunchArgument(
        'upper_raw_camera_topic',
        default_value='/camera/upper/image_raw',
        description='Upper raw image topic before decode relay'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Use C++ lane tracking node (false=Python)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz_default,
        description='Launch rviz2 for visualization (from lane_params.yaml)'
    )

    # 1. Front USB Camera (video4 → lane tracking)
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
            'camera_topic': LaunchConfiguration('camera_topic'),
            'enable_mjpeg_decoder': 'true',
            'raw_camera_topic': LaunchConfiguration('front_raw_camera_topic'),
        }.items()
    )

    # 2. Upper USB Camera (video6 → YOLO, yuyv)
    upper_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_upper',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('upper_video_device'),
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'framerate': 10.0,
            'frame_id': 'camera_upper',
            'camera_name': 'upper_camera',
        }],
        remappings=[
            ('image_raw', LaunchConfiguration('upper_raw_camera_topic')),
        ]
    )

    upper_mjpeg_decoder_node = Node(
        package='usb_cam_driver',
        executable='mjpeg_decoder_node.py',
        name='upper_mjpeg_decoder',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('upper_raw_camera_topic'),
            'output_topic': LaunchConfiguration('upper_camera_topic'),
            'output_frame_id': 'camera_upper',
        }]
    )

    # 3. Lane perception (차선 인식 - front camera)
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
            'lane_marking_enabled': 'false',
            'traffic_light_enabled': 'false',
            'obstacle_enabled': 'false',
            'speed_sign_enabled': 'false',
            'use_cpp': LaunchConfiguration('use_cpp'),
        }.items()
    )

    # 4. YOLO node (upper camera)
    yolo_node = Node(
        package='perception_pkg',
        executable='yolov8n_seg_node.py',
        name='yolov8n_seg_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('upper_camera_topic'),
            'model_path': '/root/ros2_ws/src/perception_pkg/models/yolo26n_main.pt',
        }]
    )

    # 5. Static TFs
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    static_tf_upper = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_upper',
        arguments=['0.10', '0', '0.25', '0', '0', '0', 'base_link', 'camera_upper']
    )

    # 6. rviz2
    rviz_config = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'adas_default.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        front_video_device_arg,
        upper_video_device_arg,
        camera_topic_arg,
        front_raw_camera_topic_arg,
        upper_camera_topic_arg,
        upper_raw_camera_topic_arg,
        use_cpp_arg,
        use_rviz_arg,
        front_cam_launch,
        upper_cam_node,
        upper_mjpeg_decoder_node,
        lane_perception_launch,
        yolo_node,
        static_tf_camera,
        static_tf_upper,
        rviz_node,
    ])
