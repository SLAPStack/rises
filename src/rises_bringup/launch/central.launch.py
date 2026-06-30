"""
===============================================================================
CENTRAL BRIDGE LAUNCH FILE
===============================================================================

Launches the centralized ROS bridge for multi-AGV rosbag simulations:
  - centralized_translator : message_translator_node broadcasting map updates to
                             all AGV geofence nodes via service calls
  - Rosbag player          : replays recorded map data and orders

PARAMETERS → central_params.yaml  (rises_bringup/config/)
  All node parameters live in the YAML file.  Env vars are supported via
  $(env VAR_NAME default) syntax, resolved at launch time.

REMAPPINGS → this launch file only.
  The suppress_map_topics arg controls whether MQTT map topics are wired
  to real subscribers or to /unused/* – this is a remapping concern, so
  it stays in the launch file.

USAGE
  ros2 launch rises_bringup central.launch.py \
      agv_count:=3 \
      bag_file:=/path/to/bag \
      rosbag_delay:=5.0

  # Docker – use env vars to drive YAML substitutions
  AGV_COUNT=3 USE_SIM_TIME=true \
      ros2 launch rises_bringup central.launch.py \
          bag_file:=/bags/warehouse_data

===============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


# ---------------------------------------------------------------------------
# Helper: wrap a config-file launch arg so $(env …) in YAML is resolved.
# ---------------------------------------------------------------------------
def _pf(config_key: str) -> ParameterFile:
    return ParameterFile(LaunchConfiguration(config_key), allow_substs=True)


def generate_launch_description():
    bringup_dir = get_package_share_directory('rises_bringup')
    config_dir  = os.path.join(bringup_dir, 'config')

    # =========================================================================
    # LAUNCH ARGUMENTS
    # Only deployment / topology concerns – node parameters live in YAML.
    # =========================================================================
    args = [
        # -- Config file --------------------------------------------------------
        DeclareLaunchArgument(
            'central_params_file',
            default_value=os.path.join(config_dir, 'central_params.yaml'),
            description='Path to centralized_translator YAML parameter file. '
                        'Built-in choices: central_params.yaml'),

        # -- Time ---------------------------------------------------------------
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'),

        # -- Rosbag -------------------------------------------------------------
        DeclareLaunchArgument(
            'play_rosbag', default_value='true',
            description='Enable rosbag playback'),
        DeclareLaunchArgument(
            'bag_file', default_value='',
            description='Path to rosbag file/directory (required when play_rosbag=true)'),
        DeclareLaunchArgument(
            'rosbag_rate', default_value='1.0',
            description='Rosbag playback rate multiplier'),
        DeclareLaunchArgument(
            'rosbag_loop', default_value='false',
            description='Loop rosbag playback'),
        DeclareLaunchArgument(
            'rosbag_delay', default_value='5.0',
            description='Seconds to wait before starting rosbag'),
        DeclareLaunchArgument(
            'rosbag_remaps', default_value='',
            description='Comma-separated topic remaps for rosbag (/from:=/to,…)'),
        DeclareLaunchArgument(
            'storage', default_value='sqlite3',
            description='Rosbag storage backend (sqlite3 or mcap)'),

        # -- Topic suppression (remapping concern – must stay in launch file) --
        DeclareLaunchArgument(
            'suppress_map_topics', default_value='false',
            description='Remap MQTT map topics to /unused/* '
                        '(used when map is pre-initialized via JSON)'),

        # -- Visualisation ------------------------------------------------------
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Launch RViz2'),
        DeclareLaunchArgument(
            'rviz_namespace', default_value='agv_0',
            description='AGV namespace whose topics are wired into RViz'),
        DeclareLaunchArgument(
            'rviz_config_file', default_value='/workspace/resources/rises.rviz',
            description='Path to the RViz2 config file'),
    ]

    # =========================================================================
    # CENTRALIZED TRANSLATOR NODE
    # =========================================================================
    # Remappings change WHICH topics are subscribed to – suppress_map_topics
    # switches between live MQTT and /unused/*, so it belongs here.
    def _translator(context, *args, **kwargs):
        suppress = context.launch_configurations.get('suppress_map_topics', 'false') == 'true'

        # MQTT topic names – configurable via environment variables
        mqtt_obstacle  = os.environ.get('MQTT_OBSTACLE_TOPIC',  '/mqtt/discerning_safety_map')
        mqtt_contours  = os.environ.get('MQTT_CONTOURS_TOPIC',  '/mqtt/warehouse_contours')
        mqtt_order     = os.environ.get('MQTT_ORDER_TOPIC',     '/mqtt/order')
        mqtt_tf        = os.environ.get('MQTT_TF_TOPIC',        '/mqtt/agv/tf2')
        mqtt_scan      = os.environ.get('MQTT_SCAN_TOPIC',      '/mqtt/agv/scan')
        mqtt_validation = os.environ.get('MQTT_VALIDATION_TOPIC', '/mqtt/agv/validation')
        mqtt_area_locks = os.environ.get('MQTT_AREA_LOCKS_TOPIC', '/mqtt/area_locks')

        obstacle_topic = '/unused' + mqtt_obstacle if suppress else mqtt_obstacle
        contours_topic = '/unused' + mqtt_contours if suppress else mqtt_contours
        return [Node(
            package='message_translator',
            executable='message_translator_node',
            name='centralized_translator',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[_pf('central_params_file')],
            remappings=[
                # Map topics – conditionally suppressed
                ('obstacle_json',            obstacle_topic),
                ('warehouse_contours_json',  contours_topic),
                ('area_locks_json',          mqtt_area_locks),
                # Other MQTT inputs
                ('order',       mqtt_order),
                ('tf_mqtt',     mqtt_tf),
                ('scan',        mqtt_scan),
                ('validation',  mqtt_validation),
                # Outputs
                ('/tf',                    '/tf'),
                ('/tf_static',             '/tf_static'),
                ('initialization_ready',   '/initialization_ready'),
                ('warehouse_contours',     '/warehouse_contours'),
                ('map_updates',            '/warehouse/map_updates'),
                ('incoming_path',          '/incoming_path'),
            ],
        )]

    # =========================================================================
    # ROSBAG PLAYER
    # =========================================================================
    def _rosbag(context, *args, **kwargs):
        if context.launch_configurations.get('play_rosbag', 'true') != 'true':
            return []
        bag = context.launch_configurations.get('bag_file', '')
        if not bag:
            print('[WARN] play_rosbag=true but bag_file is empty – skipping')
            return []
        cfg = context.launch_configurations
        cmd = [
            'ros2', 'bag', 'play', bag,
            '-s', cfg.get('storage', 'sqlite3'),
            '-r', cfg.get('rosbag_rate', '1.0'),
            '--read-ahead-queue-size', '100',
            '--disable-keyboard-controls',
            '--wait-for-all-acked', '5000',
        ]
        if cfg.get('use_sim_time', 'false') == 'true':
            cmd.append('--clock')
        if cfg.get('rosbag_loop', 'false') == 'true':
            cmd.append('--loop')
        # ros2 bag play uses nargs='+' for --remap, so all remaps must be passed
        # as a single --remap flag with space-separated values. Using multiple
        # --remap flags causes each to override the previous.
        remaps = [r.strip() for r in cfg.get('rosbag_remaps', '').replace('\n', '').split(',')
                  if ':=' in r.strip()]
        if remaps:
            cmd += ['--remap'] + remaps
        return [ExecuteProcess(cmd=cmd, output='screen')]

    rosbag_delay = LaunchConfiguration('rosbag_delay')

    # =========================================================================
    # RVIZ2
    # =========================================================================
    rviz_namespace = LaunchConfiguration('rviz_namespace')
    use_sim_time   = LaunchConfiguration('use_sim_time')

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('warehouse_contours_viz', '/warehouse_contours_viz'),
            ('geofence_map_viz',       [rviz_namespace, '/geofence_map_viz']),
            ('safety_circle_viz',      [rviz_namespace, '/safety_circle_viz']),
            ('error_segments_viz',     [rviz_namespace, '/error_segments_viz']),
            ('matched_segments_viz',   [rviz_namespace, '/matched_segments_viz']),
            ('validated_path',         [rviz_namespace, '/validated_path']),
            ('incoming_path',          [rviz_namespace, '/incoming_path']),
            ('processed_scan',         [rviz_namespace, '/processed_scan']),
            ('scan',                   [rviz_namespace, '/scan_0']),
            ('lidar_segments',         [rviz_namespace, '/lidar_segments']),
            ('unmatched_obstacles',    [rviz_namespace, '/unmatched_obstacles']),
        ],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription(args + [
        OpaqueFunction(function=_translator),
        TimerAction(
            period=rosbag_delay,
            actions=[OpaqueFunction(function=_rosbag)],
        ),
        rviz_node,
    ])
