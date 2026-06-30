"""
===============================================================================
UNITY CENTRALIZED BRIDGE LAUNCH FILE
===============================================================================

Launches the Unity ↔ ROS 2 bridge infrastructure:
  - ROS-TCP-Endpoint      : Unity ↔ ROS 2 socket bridge (port 10000)
  - centralized_translator: message_translator_node broadcasting map updates /
                            TF / scan data to all AGV geofence nodes

PARAMETERS → central_params.yaml  (rises_bringup/config/)
  All translator parameters live in the YAML and support $(env VAR default)
  substitution (e.g. AGV_COUNT, TRANSLATOR_ENABLE_BUFFERING).

REMAPPINGS → this launch file only.
  suppress_map_topics controls whether incoming MQTT map topics are wired
  to real subscribers or to /unused/* (remapping concern → stays here).

USAGE
  ros2 launch rises_bringup unity.launch.py

  # Custom AGV count via env var (resolved by $(env AGV_COUNT 1) in YAML)
  AGV_COUNT=4 ros2 launch rises_bringup unity.launch.py

  # With rosbag replay alongside Unity
  ros2 launch rises_bringup unity.launch.py \\
      play_rosbag:=true \\
      bag_file:=/path/to/bag

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


def _pf(config_key: str) -> ParameterFile:
    return ParameterFile(LaunchConfiguration(config_key), allow_substs=True)


def generate_launch_description():
    bringup_dir = get_package_share_directory('rises_bringup')
    config_dir  = os.path.join(bringup_dir, 'config')

    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    args = [
        # -- Config file --------------------------------------------------------
        DeclareLaunchArgument(
            'central_params_file',
            default_value=os.path.join(config_dir, 'central_params.yaml'),
            description='Parameter file for centralized_translator. '
                        'Built-in choice: central_params.yaml'),

        # -- TCP Endpoint -------------------------------------------------------
        DeclareLaunchArgument(
            'tcp_ip', default_value='0.0.0.0',
            description='IP address for ROS-TCP-Endpoint '
                        '(0.0.0.0 = listen on all interfaces)'),
        DeclareLaunchArgument(
            'tcp_port', default_value='10000',
            description='Port for ROS-TCP-Endpoint'),

        # -- Rosbag (optional – alongside Unity) --------------------------------
        DeclareLaunchArgument(
            'play_rosbag', default_value='false',
            description='Enable rosbag playback'),
        DeclareLaunchArgument(
            'bag_file', default_value='',
            description='Path to rosbag file/directory'),
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

        # -- Topic suppression (remapping concern) ------------------------------
        DeclareLaunchArgument(
            'suppress_map_topics', default_value='false',
            description='Remap MQTT map topics to /unused/* '
                        '(when map is pre-initialized via JSON)'),

        # -- Visualisation ------------------------------------------------------
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Launch RViz2'),
        DeclareLaunchArgument(
            'rviz_namespace', default_value='agv_0',
            description='AGV namespace whose topics are wired into RViz'),
    ]

    # =========================================================================
    # ROS-TCP-ENDPOINT
    # =========================================================================
    tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP':       LaunchConfiguration('tcp_ip'),
            'ROS_TCP_PORT': LaunchConfiguration('tcp_port'),
        }],
    )

    # =========================================================================
    # CENTRALIZED TRANSLATOR
    # =========================================================================
    def _translator(context, *args_, **kwargs):
        suppress = context.launch_configurations.get('suppress_map_topics', 'false') == 'true'

        # MQTT topic names – configurable via environment variables
        mqtt_obstacle   = os.environ.get('MQTT_OBSTACLE_TOPIC',    '/mqtt/discerning_safety_map')
        mqtt_contours   = os.environ.get('MQTT_CONTOURS_TOPIC',    '/mqtt/warehouse_contours')
        mqtt_order      = os.environ.get('MQTT_ORDER_TOPIC',       '/mqtt/order')
        mqtt_tf         = os.environ.get('MQTT_TF_TOPIC',          '/mqtt/agv/tf2')
        mqtt_scan       = os.environ.get('MQTT_SCAN_TOPIC',        '/mqtt/agv/scan')
        mqtt_validation = os.environ.get('MQTT_VALIDATION_TOPIC',  '/mqtt/agv/validation')

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
                ('obstacle_json',           obstacle_topic),
                ('warehouse_contours_json', contours_topic),
                # Other inputs (via ROS-TCP-Endpoint)
                ('order',      mqtt_order),
                ('tf_mqtt',    mqtt_tf),
                ('scan',       mqtt_scan),
                ('validation', mqtt_validation),
                # Outputs
                ('/tf',                   '/tf'),
                ('/tf_static',            '/tf_static'),
                ('initialization_ready',  '/initialization_ready'),
                ('warehouse_contours',    '/warehouse_contours'),
                ('map_updates',           '/warehouse/map_updates'),
                ('incoming_path',         '/incoming_path'),
            ],
        )]

    # =========================================================================
    # ROSBAG PLAYER (optional, delayed)
    # =========================================================================
    def _rosbag(context, *args_, **kwargs):
        if context.launch_configurations.get('play_rosbag', 'false') != 'true':
            return []
        bag = context.launch_configurations.get('bag_file', '')
        if not bag:
            print('[WARN] play_rosbag=true but bag_file is empty – skipping')
            return []
        cfg = context.launch_configurations
        # Build the ros2 bag play command, then wrap in a bash -c with a small
        # pre-playback sleep.  The sleep ensures DDS discovery completes between
        # the rosbag publisher and the translator subscriber.  Without this, the
        # very first rosbag messages (contours + initial bulk obstacle load) can
        # be published before discovery finishes and are silently lost.
        parts = [
            'ros2', 'bag', 'play', bag,
            '-s', cfg.get('storage', 'sqlite3'),
            '-r', cfg.get('rosbag_rate', '1.0'),
            '--read-ahead-queue-size', '100',
            '--disable-keyboard-controls',
            '--wait-for-all-acked', '5000',
        ]
        if cfg.get('use_sim_time', 'false') == 'true':
            parts.append('--clock')
        if cfg.get('rosbag_loop', 'false') == 'true':
            parts.append('--loop')
        for remap in cfg.get('rosbag_remaps', '').replace('\n', '').split(','):
            remap = remap.strip()
            if ':=' in remap:
                parts += ['--remap', remap]
        pre_delay = cfg.get('rosbag_pre_delay', '2')
        bag_cmd = ' '.join(parts)
        cmd = ['bash', '-c', f'sleep {pre_delay} && {bag_cmd}']
        return [ExecuteProcess(cmd=cmd, output='screen')]

    rosbag_delay   = LaunchConfiguration('rosbag_delay')
    rviz_namespace = LaunchConfiguration('rviz_namespace')

    # =========================================================================
    # RVIZ2
    # =========================================================================
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', '/workspace/resources/rises.rviz'],
        output='screen',
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
        tcp_endpoint,
        OpaqueFunction(function=_translator),
        TimerAction(
            period=rosbag_delay,
            actions=[OpaqueFunction(function=_rosbag)],
        ),
        rviz_node,
    ])
