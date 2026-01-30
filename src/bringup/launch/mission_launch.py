"""
Mission mode launch file.
Launches the full autonomous system with all perception and control modules.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional node launches."""

    use_cpp = LaunchConfiguration('use_cpp').perform(context) == 'true'
    decision_mode = LaunchConfiguration('decision_mode').perform(context)

    nodes_to_launch = []

    # Arduino bridge node
    if use_cpp:
        arduino_node = Node(
            package='arduino_driver',
            executable='arduino_bridge_node',
            name='arduino_bridge',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('arduino_driver'),
                    'config',
                    'arduino.yaml'
                ]).perform(context)
            ]
        )
    else:
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
                ]).perform(context)
            ]
        )
    nodes_to_launch.append(arduino_node)

    # Ultrasonic processor node
    if use_cpp:
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
                ]).perform(context)
            ]
        )
    else:
        ultrasonic_node = Node(
            package='ultrasonic_driver',
            executable='ultrasonic_processor_node.py',
            name='ultrasonic_processor',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('ultrasonic_driver'),
                    'config',
                    'ultrasonic.yaml'
                ]).perform(context)
            ]
        )
    nodes_to_launch.append(ultrasonic_node)

    # LiDAR obstacle node
    if use_cpp:
        lidar_obstacle_node = Node(
            package='decision',
            executable='lidar_obstacle_node',
            name='lidar_obstacle',
            output='screen'
        )
    else:
        lidar_obstacle_node = Node(
            package='decision',
            executable='lidar_obstacle_node.py',
            name='lidar_obstacle',
            output='screen'
        )
    nodes_to_launch.append(lidar_obstacle_node)

    # Decision node based on mode
    if decision_mode == '2026':
        if use_cpp:
            decision_node = Node(
                package='decision',
                executable='decision_node',
                name='decision_node',
                output='screen',
                parameters=[
                    {
                        'use_traffic_light': True,
                        'stop_on_yellow': True,
                        'use_obstacle_avoidance': True,
                        'obstacle_bias_weight': 0.5,
                    }
                ],
                remappings=[
                    ('/decision/cmd', '/arduino/cmd'),
                ]
            )
        else:
            decision_node = Node(
                package='decision',
                executable='decision_node_unified.py',
                name='decision_node',
                output='screen',
                parameters=[
                    {
                        'use_traffic_light': True,
                        'stop_on_yellow': True,
                        'use_obstacle_avoidance': True,
                        'obstacle_bias_weight': 0.5,
                    }
                ],
                remappings=[
                    ('/decision/cmd', '/arduino/cmd'),
                ]
            )
    elif decision_mode == 'ai':
        decision_node = Node(
            package='decision',
            executable='decision_node_ai.py',
            name='decision_node',
            output='screen',
            parameters=[
                {
                    'use_traffic_light': True,
                    'stop_on_yellow': True,
                }
            ],
            remappings=[
                ('/decision/cmd', '/arduino/cmd'),
            ]
        )
    else:
        # Default to 2026 C++ node
        decision_node = Node(
            package='decision',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[
                {
                    'use_traffic_light': True,
                    'stop_on_yellow': True,
                    'use_obstacle_avoidance': True,
                    'obstacle_bias_weight': 0.5,
                }
            ],
            remappings=[
                ('/decision/cmd', '/arduino/cmd'),
            ]
        )

    nodes_to_launch.append(decision_node)

    # Static TF Publishers for sensor frames
    # base_link -> laser (LiDAR)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        # x y z yaw pitch roll parent_frame child_frame
        # LiDAR is 10cm above base_link
    )
    nodes_to_launch.append(static_tf_laser)

    # base_link -> camera_front (front camera)
    static_tf_camera_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_front',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_front']
        # Front camera: 15cm forward, 12cm up from base_link
    )
    nodes_to_launch.append(static_tf_camera_front)

    # base_link -> camera_rear (rear camera for parking)
    static_tf_camera_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_rear',
        arguments=['-0.15', '0', '0.08', '0', '0', '3.14159', 'base_link', 'camera_rear']
        # Rear camera: 15cm backward, 8cm up, rotated 180 degrees (pi radians)
    )
    nodes_to_launch.append(static_tf_camera_rear)

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for mission mode."""

    # Declare launch arguments
    decision_mode_arg = DeclareLaunchArgument(
        'decision_mode',
        default_value='2026',
        description='Decision mode: 2026 or ai'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/front/image',
        description='Camera topic name'
    )

    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='false',
        description='Use compressed image transport'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Use C++ nodes instead of Python'
    )

    # Include dual USB camera launch (front + rear)
    dual_usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'launch',
                'dual_usb_cam_launch.py'
            ])
        ]),
        launch_arguments={
            'front_video_device': '/dev/video6',
            'rear_video_device': '/dev/video4',
        }.items()
    )

    # Parking line detection node (rear camera)
    parking_line_node = Node(
        package='perception_pkg',
        executable='parking_line_node.py',
        name='parking_line_node',
        output='screen',
        parameters=[
            {
                'camera_topic': '/camera/rear/image',
                'use_compressed': False,
                'debug': True,
            }
        ]
    )

    # Include lane perception launch (full perception with all modules)
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
            'use_compressed': LaunchConfiguration('use_compressed'),
            'lane_marking_enabled': 'true',
            'allowed_stop_states': "red",
            'speed_sign_enabled': 'false',
            'traffic_light_enabled': 'true',
            'obstacle_enabled': 'true',
            'use_cpp': LaunchConfiguration('use_cpp'),
        }.items()
    )

    # Include RPLiDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_driver'),
                'launch',
                'rplidar_launch.py'
            ])
        ])
    )

    return LaunchDescription([
        decision_mode_arg,
        camera_topic_arg,
        use_compressed_arg,
        use_cpp_arg,
        dual_usb_cam_launch,  # 전방+후방 카메라
        parking_line_node,     # 후방 카메라 주차 라인 감지
        lane_perception_launch,
        rplidar_launch,  # RPLiDAR 드라이버
        OpaqueFunction(function=launch_setup),
    ])
