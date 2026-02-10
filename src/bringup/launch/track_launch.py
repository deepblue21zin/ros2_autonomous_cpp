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
import yaml
import os


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional node launches."""

    use_cpp = LaunchConfiguration('use_cpp').perform(context) == 'true'
    decision_mode = LaunchConfiguration('decision_mode').perform(context)
    test_mode = LaunchConfiguration('test_mode').perform(context) == 'true'

    nodes_to_launch = []

    # Arduino bridge node (항상 Python 사용 - C++ boost::asio 시리얼 문제)
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
    if decision_mode == 'cpp':
        # C++ decision node (기본값: 낮은 레이턴시)
        decision_config = PathJoinSubstitution([
            FindPackageShare('decision'), 'config', 'decision_params.yaml'
        ])
        decision_node = Node(
            package='decision',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[
                decision_config,
                {
                    'stop_on_yellow': False,
                    'test_mode': test_mode,
                }
            ],
            remappings=[
                ('/decision/cmd', '/arduino/cmd'),
            ]
        )
    elif decision_mode == 'unified':
        # Python fallback (C++ 노드 문제 시 사용)
        decision_node = Node(
            package='decision',
            executable='decision_node_unified.py',
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
        # Default to C++ node
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

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for track mode."""

    # Load use_rviz from lane_params.yaml
    perception_pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')
    lane_params_file = os.path.join(perception_pkg_share, 'config', 'lane_params.yaml')
    with open(lane_params_file, 'r') as f:
        lane_params = yaml.safe_load(f)
    use_rviz_default = str(lane_params['/**']['ros__parameters'].get('use_rviz', False)).lower()

    # Declare launch arguments
    decision_mode_arg = DeclareLaunchArgument(
        'decision_mode',
        default_value='cpp',
        description='Decision mode: cpp, unified, or ai'
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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz_default,
        description='Launch rviz2 for visualization (from lane_params.yaml)'
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
            'lane_marking_enabled': 'false',
            'allowed_stop_states': "red",
            'speed_sign_enabled': 'false',
            'traffic_light_enabled': 'false',
            'obstacle_enabled': 'false',
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

    # rviz2 visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'adas_default.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        decision_mode_arg,
        camera_topic_arg,
        use_compressed_arg,
        use_cpp_arg,
        test_mode_arg,
        use_rviz_arg,
        usb_cam_launch,
        lane_perception_launch,
        rplidar_launch,
        rviz_node,
        OpaqueFunction(function=launch_setup),
    ])
