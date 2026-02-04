"""
YOLO-based lane driving launch file.
Uses YOLOv8n-seg drivable_mask for lane following without traditional lane detection.

Architecture:
  [Camera] → [YOLOv8n-seg] → /perception/drivable_mask
                             → /perception/traffic_light_state
                                      ↓
                              [YOLO Lane Drive Node]
                                      ↓
                              [Arduino Bridge] → Motor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for YOLO-based lane driving."""

    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Camera topic name'
    )

    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed',
        default_value='0.3',
        description='Cruise speed in m/s'
    )

    max_steer_arg = DeclareLaunchArgument(
        'max_steer',
        default_value='0.45',
        description='Maximum steering angle in radians'
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='1.2',
        description='PID proportional gain'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.3',
        description='PID derivative gain'
    )

    use_traffic_light_arg = DeclareLaunchArgument(
        'use_traffic_light',
        default_value='true',
        description='Enable traffic light detection'
    )

    # 1. USB Camera
    usb_cam_node = Node(
        package='usb_cam_driver',
        executable='usb_cam_node.py',
        name='usb_cam_front',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'config',
                'usb_cam.yaml'
            ])
        ],
        remappings=[
            ('/camera/image', LaunchConfiguration('camera_topic')),
        ]
    )

    # 2. YOLOv8n-seg Node
    yolo_node = Node(
        package='perception_pkg',
        executable='yolov8n_seg_node.py',
        name='yolov8n_seg_node',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('camera_topic'),
                'model_path': '',  # auto-detect best.pt
                'conf_threshold': 0.4,
                'iou_threshold': 0.45,
                'imgsz': 640,
                'publish_overlay': True,
                'publish_drivable_mask': True,
                'detect_obstacles': True,
                'rate_hz': 10.0,
            }
        ]
    )

    # 3. YOLO Lane Drive Node
    yolo_drive_node = Node(
        package='decision',
        executable='yolo_lane_drive_node.py',
        name='yolo_lane_drive',
        output='screen',
        parameters=[
            {
                'cruise_speed_mps': LaunchConfiguration('cruise_speed'),
                'max_steer_rad': LaunchConfiguration('max_steer'),
                'kp': LaunchConfiguration('kp'),
                'kd': LaunchConfiguration('kd'),
                'use_traffic_light': LaunchConfiguration('use_traffic_light'),
                'ultra_safe_distance_m': 0.2,
                'roi_top_ratio': 0.4,
                'roi_bottom_ratio': 0.9,
                'control_hz': 20.0,
            }
        ],
        remappings=[
            ('/decision/cmd', '/arduino/cmd'),
        ]
    )

    # 4. Arduino Bridge
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

    # 5. Ultrasonic Processor
    ultrasonic_node = Node(
        package='ultrasonic_driver',
        executable='ultrasonic_processor_node',
        name='ultrasonic_processor',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('ultrasonic_driver'),
                'config',
                'ultrasonic.yaml'
            ])
        ]
    )

    # 6. RPLiDAR (optional - for obstacle detection)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB_LIDAR',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }]
    )

    # 7. Static TF: base_link -> laser
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )

    # 8. Static TF: base_link -> camera_front
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
    )

    return LaunchDescription([
        # Launch arguments
        camera_topic_arg,
        cruise_speed_arg,
        max_steer_arg,
        kp_arg,
        kd_arg,
        use_traffic_light_arg,

        # Nodes
        usb_cam_node,
        yolo_node,
        yolo_drive_node,
        arduino_node,
        ultrasonic_node,
        rplidar_node,
        static_tf_laser,
        static_tf_camera,
    ])
