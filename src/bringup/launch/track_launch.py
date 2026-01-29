"""
Track mode launch file.
Launches the full autonomous system for track driving mode.
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
from launch_ros.actions import Node, LoadComposableNodes, SetParameter
from launch.conditions import IfCondition, UnlessCondition


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional node launches."""

    use_cpp = LaunchConfiguration('use_cpp').perform(context) == 'true'
    decision_mode = LaunchConfiguration('decision_mode').perform(context)
    test_mode = LaunchConfiguration('test_mode').perform(context) == 'true'

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
                        'stop_on_yellow': False,
                        'test_mode': test_mode,
                    }
                ],
                remappings=[
                    ('/decision/cmd', '/arduino/cmd'),
                ]
            )
        else:
            decision_node = Node(
                package='decision',
                executable='decision_node_2026.py',
                name='decision_node',
                output='screen',
                parameters=[
                    {
                        'stop_on_yellow': False,
                        'test_mode': test_mode,
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
                    'stop_on_yellow': False,
                    'test_mode': test_mode,
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
                    'stop_on_yellow': False,
                    'test_mode': test_mode,
                }
            ],
            remappings=[
                ('/decision/cmd', '/arduino/cmd'),
            ]
        )

    nodes_to_launch.append(decision_node)

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for track mode."""

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

    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Test mode: bypass sensor checks for motor testing'
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

    # Include lane perception launch
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
            'traffic_light_enabled': 'false',
            'obstacle_enabled': 'false',
            'use_cpp': LaunchConfiguration('use_cpp'),
        }.items()
    )

    return LaunchDescription([
        decision_mode_arg,
        camera_topic_arg,
        use_compressed_arg,
        use_cpp_arg,
        test_mode_arg,
        usb_cam_launch,
        lane_perception_launch,
        OpaqueFunction(function=launch_setup),
    ])
