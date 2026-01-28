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

    # Include USB camera launch
    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usb_cam_driver'),
                'launch',
                'usb_cam_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_topic': LaunchConfiguration('camera_topic'),
        }.items()
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
            'allowed_stop_states': "['red']",
            'speed_sign_enabled': 'false',
            'traffic_light_enabled': 'true',
            'obstacle_enabled': 'true',
            'use_cpp': LaunchConfiguration('use_cpp'),
        }.items()
    )

    # NOTE: Ensure rplidar_ros is running separately if needed
    # You can uncomment the following to include RPLiDAR launch:
    #
    # rplidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('rplidar_driver'),
    #             'launch',
    #             'rplidar_launch.py'
    #         ])
    #     ])
    # )

    return LaunchDescription([
        decision_mode_arg,
        camera_topic_arg,
        use_compressed_arg,
        use_cpp_arg,
        usb_cam_launch,
        lane_perception_launch,
        # rplidar_launch,  # Uncomment if needed
        OpaqueFunction(function=launch_setup),
    ])
