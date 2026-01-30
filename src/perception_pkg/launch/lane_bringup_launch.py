"""
Lane perception bringup launch file.
Launches lane tracking, obstacle detection, and other perception nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional node launches."""

    # Get launch configurations
    use_cpp = LaunchConfiguration('use_cpp').perform(context) == 'true'
    lane_marking_enabled = LaunchConfiguration('lane_marking_enabled').perform(context) == 'true'
    speed_sign_enabled = LaunchConfiguration('speed_sign_enabled').perform(context) == 'true'
    traffic_light_enabled = LaunchConfiguration('traffic_light_enabled').perform(context) == 'true'
    obstacle_enabled = LaunchConfiguration('obstacle_enabled').perform(context) == 'true'

    # Get package share directory
    pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')

    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('perception_pkg'),
        'config',
        'lane_params.yaml'
    ])

    nodes_to_launch = []

    # Lane Tracking Node (always launched)
    if use_cpp:
        lane_tracking_node = Node(
            package='perception_pkg',
            executable='lane_tracking_node',
            name='lane_tracking_node',
            output='screen',
            parameters=[
                config_file.perform(context),
                {
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'kp': LaunchConfiguration('kp'),
                    'debug': LaunchConfiguration('lane_debug'),
                    'use_compressed': LaunchConfiguration('use_compressed'),
                }
            ]
        )
    else:
        lane_tracking_node = Node(
            package='perception_pkg',
            executable='lane_tracking_node.py',
            name='lane_tracking_node',
            output='screen',
            parameters=[
                config_file.perform(context),
                {
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'kp': LaunchConfiguration('kp'),
                    'debug': LaunchConfiguration('lane_debug'),
                    'use_compressed': LaunchConfiguration('use_compressed'),
                }
            ]
        )
    nodes_to_launch.append(lane_tracking_node)

    # Lane Marking Node (conditional)
    if lane_marking_enabled:
        if use_cpp:
            lane_marking_node = Node(
                package='perception_pkg',
                executable='lane_marking_node',
                name='lane_marking_node',
                output='screen',
                parameters=[
                    config_file.perform(context),
                    {
                        'camera_topic': LaunchConfiguration('camera_topic'),
                        'use_compressed': LaunchConfiguration('use_compressed'),
                        'allowed_stop_states': LaunchConfiguration('allowed_stop_states'),
                    }
                ]
            )
        else:
            lane_marking_node = Node(
                package='perception_pkg',
                executable='lane_marking_node.py',
                name='lane_marking_node',
                output='screen',
                parameters=[
                    config_file.perform(context),
                    {
                        'camera_topic': LaunchConfiguration('camera_topic'),
                        'use_compressed': LaunchConfiguration('use_compressed'),
                        'allowed_stop_states': LaunchConfiguration('allowed_stop_states'),
                    }
                ]
            )
        nodes_to_launch.append(lane_marking_node)

    # Speed Sign Node (Python only - YOLO based)
    if speed_sign_enabled:
        speed_sign_node = Node(
            package='perception_pkg',
            executable='speed_sign_node.py',
            name='speed_sign_node',
            output='screen',
            parameters=[
                {
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'use_compressed': LaunchConfiguration('use_compressed'),
                    'default_speed_limit': LaunchConfiguration('default_speed_limit'),
                    'detector_model_path': LaunchConfiguration('speed_sign_model'),
                    'publish_overlay': LaunchConfiguration('speed_sign_publish_overlay'),
                }
            ]
        )
        nodes_to_launch.append(speed_sign_node)

    # Traffic Light Node (Python only - YOLO based)
    if traffic_light_enabled:
        traffic_light_node = Node(
            package='perception_pkg',
            executable='traffic_light_node.py',
            name='traffic_light_node',
            output='screen',
            parameters=[
                {
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'use_compressed': LaunchConfiguration('use_compressed'),
                    'detector_model_path': LaunchConfiguration('traffic_light_model'),
                    'publish_overlay': LaunchConfiguration('traffic_light_publish_overlay'),
                }
            ]
        )
        nodes_to_launch.append(traffic_light_node)

    # Obstacle Detection Node (conditional)
    if obstacle_enabled:
        if use_cpp:
            obstacle_node = Node(
                package='perception_pkg',
                executable='obstacle_detection_node',
                name='obstacle_detection_node',
                output='screen',
                parameters=[
                    config_file.perform(context),
                    {
                        'camera_topic': LaunchConfiguration('camera_topic'),
                        'use_compressed': LaunchConfiguration('use_compressed'),
                        'roi_y_ratio': LaunchConfiguration('obstacle_roi_y_ratio'),
                        'band_ratio': LaunchConfiguration('obstacle_band_ratio'),
                        'obstacle_center_ratio': LaunchConfiguration('obstacle_center_ratio'),
                        'square_ratio': LaunchConfiguration('obstacle_square_ratio'),
                        'min_area': LaunchConfiguration('obstacle_min_area'),
                        'min_height': LaunchConfiguration('obstacle_min_height'),
                        'saturation_thresh': LaunchConfiguration('obstacle_saturation_thresh'),
                        'value_thresh': LaunchConfiguration('obstacle_value_thresh'),
                        'canny_low': LaunchConfiguration('obstacle_canny_low'),
                        'canny_high': LaunchConfiguration('obstacle_canny_high'),
                        'bias_gain': LaunchConfiguration('obstacle_bias_gain'),
                        'publish_overlay': LaunchConfiguration('obstacle_publish_overlay'),
                    }
                ]
            )
        else:
            obstacle_node = Node(
                package='perception_pkg',
                executable='obstacle_detection_node.py',
                name='obstacle_detection_node',
                output='screen',
                parameters=[
                    config_file.perform(context),
                    {
                        'camera_topic': LaunchConfiguration('camera_topic'),
                        'use_compressed': LaunchConfiguration('use_compressed'),
                        'roi_y_ratio': LaunchConfiguration('obstacle_roi_y_ratio'),
                        'band_ratio': LaunchConfiguration('obstacle_band_ratio'),
                        'obstacle_center_ratio': LaunchConfiguration('obstacle_center_ratio'),
                        'square_ratio': LaunchConfiguration('obstacle_square_ratio'),
                        'min_area': LaunchConfiguration('obstacle_min_area'),
                        'min_height': LaunchConfiguration('obstacle_min_height'),
                        'saturation_thresh': LaunchConfiguration('obstacle_saturation_thresh'),
                        'value_thresh': LaunchConfiguration('obstacle_value_thresh'),
                        'canny_low': LaunchConfiguration('obstacle_canny_low'),
                        'canny_high': LaunchConfiguration('obstacle_canny_high'),
                        'bias_gain': LaunchConfiguration('obstacle_bias_gain'),
                        'publish_overlay': LaunchConfiguration('obstacle_publish_overlay'),
                    }
                ]
            )
        nodes_to_launch.append(obstacle_node)

    return nodes_to_launch


def generate_launch_description():
    """Generate launch description for lane perception."""

    # Declare all launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_jpeg/compressed',
        description='Camera topic name'
    )

    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='Use compressed image transport'
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='0.7',  # 0.6→0.7 (대회 트랙 최적화)
        description='Proportional gain for steering control'
    )

    lane_debug_arg = DeclareLaunchArgument(
        'lane_debug',
        default_value='true',  # false→true (lane_overlay 발행)
        description='Enable debug output for lane tracking'
    )

    default_speed_limit_arg = DeclareLaunchArgument(
        'default_speed_limit',
        default_value='20.0',
        description='Default speed limit in km/h'
    )

    speed_sign_enabled_arg = DeclareLaunchArgument(
        'speed_sign_enabled',
        default_value='false',
        description='Enable speed sign detection'
    )

    speed_sign_model_arg = DeclareLaunchArgument(
        'speed_sign_model',
        default_value=PathJoinSubstitution([
            FindPackageShare('perception_pkg'),
            'models',
            'traffic_sign_detector.pt'
        ]),
        description='Path to speed sign YOLO model'
    )

    traffic_light_enabled_arg = DeclareLaunchArgument(
        'traffic_light_enabled',
        default_value='false',
        description='Enable traffic light detection'
    )

    traffic_light_model_arg = DeclareLaunchArgument(
        'traffic_light_model',
        default_value=PathJoinSubstitution([
            FindPackageShare('perception_pkg'),
            'models',
            'traffic_sign_detector.pt'
        ]),
        description='Path to traffic light YOLO model'
    )

    obstacle_enabled_arg = DeclareLaunchArgument(
        'obstacle_enabled',
        default_value='false',
        description='Enable obstacle detection'
    )

    obstacle_roi_y_ratio_arg = DeclareLaunchArgument(
        'obstacle_roi_y_ratio',
        default_value='0.55',
        description='ROI Y ratio for obstacle detection'
    )

    obstacle_band_ratio_arg = DeclareLaunchArgument(
        'obstacle_band_ratio',
        default_value='0.6',
        description='Band ratio for obstacle detection'
    )

    obstacle_center_ratio_arg = DeclareLaunchArgument(
        'obstacle_center_ratio',
        default_value='0.4',
        description='Center ratio for obstacle detection'
    )

    obstacle_square_ratio_arg = DeclareLaunchArgument(
        'obstacle_square_ratio',
        default_value='0.0',
        description='Square ratio for obstacle detection'
    )

    obstacle_min_area_arg = DeclareLaunchArgument(
        'obstacle_min_area',
        default_value='600',
        description='Minimum area for obstacle detection'
    )

    obstacle_min_height_arg = DeclareLaunchArgument(
        'obstacle_min_height',
        default_value='20',
        description='Minimum height for obstacle detection'
    )

    obstacle_saturation_thresh_arg = DeclareLaunchArgument(
        'obstacle_saturation_thresh',
        default_value='60',
        description='Saturation threshold for obstacle detection'
    )

    obstacle_value_thresh_arg = DeclareLaunchArgument(
        'obstacle_value_thresh',
        default_value='40',
        description='Value threshold for obstacle detection'
    )

    obstacle_canny_low_arg = DeclareLaunchArgument(
        'obstacle_canny_low',
        default_value='80',
        description='Canny low threshold for obstacle detection'
    )

    obstacle_canny_high_arg = DeclareLaunchArgument(
        'obstacle_canny_high',
        default_value='160',
        description='Canny high threshold for obstacle detection'
    )

    obstacle_bias_gain_arg = DeclareLaunchArgument(
        'obstacle_bias_gain',
        default_value='1.5',
        description='Bias gain for obstacle avoidance'
    )

    lane_marking_enabled_arg = DeclareLaunchArgument(
        'lane_marking_enabled',
        default_value='false',
        description='Enable lane marking detection'
    )

    allowed_stop_states_arg = DeclareLaunchArgument(
        'allowed_stop_states',
        default_value="['red','yellow']",
        description='Traffic light states that trigger stop'
    )

    speed_sign_publish_overlay_arg = DeclareLaunchArgument(
        'speed_sign_publish_overlay',
        default_value='false',
        description='Publish speed sign detection overlay'
    )

    traffic_light_publish_overlay_arg = DeclareLaunchArgument(
        'traffic_light_publish_overlay',
        default_value='false',
        description='Publish traffic light detection overlay'
    )

    obstacle_publish_overlay_arg = DeclareLaunchArgument(
        'obstacle_publish_overlay',
        default_value='false',
        description='Publish obstacle detection overlay'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Use C++ nodes instead of Python'
    )

    return LaunchDescription([
        camera_topic_arg,
        use_compressed_arg,
        kp_arg,
        lane_debug_arg,
        default_speed_limit_arg,
        speed_sign_enabled_arg,
        speed_sign_model_arg,
        traffic_light_enabled_arg,
        traffic_light_model_arg,
        obstacle_enabled_arg,
        obstacle_roi_y_ratio_arg,
        obstacle_band_ratio_arg,
        obstacle_center_ratio_arg,
        obstacle_square_ratio_arg,
        obstacle_min_area_arg,
        obstacle_min_height_arg,
        obstacle_saturation_thresh_arg,
        obstacle_value_thresh_arg,
        obstacle_canny_low_arg,
        obstacle_canny_high_arg,
        obstacle_bias_gain_arg,
        lane_marking_enabled_arg,
        allowed_stop_states_arg,
        speed_sign_publish_overlay_arg,
        traffic_light_publish_overlay_arg,
        obstacle_publish_overlay_arg,
        use_cpp_arg,
        OpaqueFunction(function=launch_setup),
    ])
