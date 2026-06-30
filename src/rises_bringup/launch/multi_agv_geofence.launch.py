"""
===============================================================================
MULTI-AGV GEOFENCE LAUNCH FILE
===============================================================================

Launches ALL AGV geofence stacks in a SINGLE composable container, sharing
one MultiThreadedExecutor process.  With N AGVs this collapses N separate
containers into one, saving significant memory and DDS overhead.

PARAMETERS → single scenario YAML (same file used by geofence.launch.py)
  All node parameters come from params_file.  Shared params (safety circle,
  detection tuning, …) apply identically to every AGV.
  Per-AGV values (robot_id, tf_prefix) are injected via a second parameter
  dict that overrides the YAML – ROS 2 merges parameter sources in order.

TOPOLOGY ARGS (launch-file concerns, not in YAML)
  agv_count      : Number of AGVs → namespaces generated as agv_0..agv_N-1
  params_file    : Scenario YAML (params_default / params_rosbag / params_unity)
  scan_topic_remap : Override scan topic (empty = use scan_0)
  publish_unity_tf / publish_slapstack_tf / target_frame : coordinate TFs
  Static TF args: same as geofence.launch.py

USAGE
  # 4 AGVs, Unity scenario
  ros2 launch rises_bringup multi_agv_geofence.launch.py \\
      agv_count:=4 \\
      params_file:=.../params_unity.yaml \\
      publish_unity_tf:=true

  # Driven from entrypoint.sh via SCENARIO=unity AGV_COUNT=4
===============================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def _pf() -> ParameterFile:
    """Wrap params_file so $(env …) in the YAML is resolved at launch time."""
    return ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)


def generate_launch_description():
    bringup_dir = get_package_share_directory('rises_bringup')
    config_dir  = os.path.join(bringup_dir, 'config')

    # =========================================================================
    # LAUNCH ARGUMENTS  (topology only – node params live in YAML)
    # =========================================================================
    args = [
        DeclareLaunchArgument(
            'agv_count', default_value='1',
            description='Number of AGVs (namespaces: agv_0 … agv_N-1)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'params_default.yaml'),
            description='Scenario YAML covering all nodes. '
                        'Choices: params_default.yaml, params_rosbag.yaml, '
                        'params_unity.yaml'),
        DeclareLaunchArgument(
            'scan_topic_remap', default_value='',
            description='Override the laserscan input topic. '
                        'Empty = use scan_0 (from YAML)'),

        # -- Static TF publishers (topology – which nodes to create) ----------
        DeclareLaunchArgument(
            'publish_unity_tf', default_value='false',
            description='Publish static TF: map → unity_map (+90° Z)'),
        DeclareLaunchArgument(
            'publish_slapstack_tf', default_value='false',
            description='Publish static TF: map → slapstack_map (−90° Z)'),
        DeclareLaunchArgument(
            'target_frame', default_value='map',
            description='Parent frame for coordinate-system TF publishers'),
        DeclareLaunchArgument(
            'publish_static_tf', default_value='false',
            description='Publish a generic static TF transform'),
        DeclareLaunchArgument('static_tf_parent_frame',  default_value='base_link'),
        DeclareLaunchArgument('static_tf_child_frame',   default_value='laser_link'),
        DeclareLaunchArgument('static_tf_use_namespace', default_value='true'),
        DeclareLaunchArgument('static_tf_x',     default_value='0.0'),
        DeclareLaunchArgument('static_tf_y',     default_value='0.0'),
        DeclareLaunchArgument('static_tf_z',     default_value='0.0'),
        DeclareLaunchArgument('static_tf_roll',  default_value='0.0'),
        DeclareLaunchArgument('static_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('static_tf_yaw',   default_value='0.0'),
        DeclareLaunchArgument(
            'publish_lidar_static_tf', default_value='false',
            description='Publish static lidar TF (base_link → laser_link)'),
        DeclareLaunchArgument('lidar_transform_x',     default_value='0.0'),
        DeclareLaunchArgument('lidar_transform_y',     default_value='0.0'),
        DeclareLaunchArgument('lidar_transform_z',     default_value='0.0'),
        DeclareLaunchArgument('lidar_transform_roll',  default_value='0.0'),
        DeclareLaunchArgument('lidar_transform_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_transform_yaw',   default_value='0.0'),
    ]

    # =========================================================================
    # REMAPPINGS  (topology – stays in launch file, not YAML)
    # =========================================================================
    GEOFENCE_REMAPS = [
        ('lidar_segments',         'lidar_segments'),
        ('map_boundary',           '/warehouse_contours'),
        ('unmatched_obstacles',    'unmatched_obstacles'),
        ('warehouse_contours_viz', 'warehouse_contours_viz'),
        ('geofence_map_viz',       'geofence_map_viz'),
        ('safety_circle_viz',      'safety_circle_viz'),
        ('error_segments_viz',     'error_segments_viz'),
        ('matched_segments_viz',   'matched_segments_viz'),
        ('validated_path',         'validated_path'),
        ('validate_path',          'validate_path'),
        ('set_area_state',         'set_area_state'),
        ('/tf',                    '/tf'),
        ('/tf_static',             '/tf_static'),
    ]
    LASERSCAN_BASE_REMAPS = [
        ('obstacles',      'lidar_segments'),
        ('world_scan',     'world_scan'),
        ('processed_scan', 'processed_scan'),
        ('/tf',            '/tf'),
        ('/tf_static',     '/tf_static'),
    ]
    FLEET_INTERFACE_REMAPS = [
        ('order',           'order'),
        ('incoming_path',   'incoming_path'),
        ('obstacle_report', 'obstacle_report'),
        ('obstacle_alert',  'obstacle_alert'),
        ('obstacle_state',  'obstacle_state'),
        ('/tf',             '/tf'),
        ('/tf_static',      '/tf_static'),
    ]

    # =========================================================================
    # CONTAINER + ALL AGV NODES
    # =========================================================================

    def _create_agv_nodes(context, *args_, **kwargs):
        agv_count  = int(context.launch_configurations.get('agv_count', '1'))
        scan_remap = context.launch_configurations.get('scan_topic_remap', '')
        scan_input = scan_remap if scan_remap else 'scan_0'

        # params_file is the shared base; per-AGV overrides are merged on top.
        shared_params = _pf()
        common_args   = ['--ros-args', '--log-level', 'info']
        laserscan_remaps = LASERSCAN_BASE_REMAPS + [('scan_0', scan_input)]

        descriptions = []
        for i in range(agv_count):
            ns       = f'agv_{i}'
            mqtt_id  = f'id{i}'
            per_agv  = {'robot_id': ns, 'tf_prefix': ns, 'mqtt_agv_name': mqtt_id}

            descriptions.append(ComposableNode(
                package='geofence',
                plugin='rises::GeofenceNode',
                name='geofence_node',
                namespace=ns,
                # Disable intra-process to preserve TRANSIENT_LOCAL QoS
                # (Humble bypasses durability for intra-process messages).
                extra_arguments=[{'use_intra_process_comms': False}],
                # shared_params (YAML) first; per_agv overrides robot_id/tf_prefix
                parameters=[shared_params, per_agv],
                remappings=GEOFENCE_REMAPS,
            ))
            descriptions.append(ComposableNode(
                package='laserscan_preprocessor',
                plugin='rises::LaserPreprocessorNode',
                name='laserscan_preprocessor_node',
                namespace=ns,
                extra_arguments=[{'use_intra_process_comms': False}],
                parameters=[shared_params, per_agv],
                remappings=laserscan_remaps,
            ))
            descriptions.append(ComposableNode(
                package='fleet_interface',
                plugin='rises::FleetInterfaceNode',
                name='fleet_interface_node',
                namespace=ns,
                extra_arguments=[{'use_intra_process_comms': False}],
                parameters=[shared_params, per_agv],
                remappings=FLEET_INTERFACE_REMAPS,
            ))

        return [ComposableNodeContainer(
            name='multi_agv_geofence_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=descriptions,
            output='screen',
            arguments=common_args,
        )]

    # =========================================================================
    # OPTIONAL TF PUBLISHERS
    # =========================================================================
    from launch_ros.actions import Node

    def _unity_tf(context, *args_, **kwargs):
        if context.launch_configurations.get('publish_unity_tf', 'false') != 'true':
            return []
        frame = context.launch_configurations.get('target_frame', 'map')
        return [Node(package='tf2_ros', executable='static_transform_publisher',
                     name='unity_map_tf',
                     arguments=['0','0','0','0','0','1.5708', frame, 'unity_map'],
                     output='screen')]

    def _slapstack_tf(context, *args_, **kwargs):
        if context.launch_configurations.get('publish_slapstack_tf', 'false') != 'true':
            return []
        frame = context.launch_configurations.get('target_frame', 'map')
        return [Node(package='tf2_ros', executable='static_transform_publisher',
                     name='slapstack_map_tf',
                     arguments=['0','0','0','0','0','-1.5708', frame, 'slapstack_map'],
                     output='screen')]

    def _static_tf(context, *args_, **kwargs):
        if context.launch_configurations.get('publish_static_tf', 'false') != 'true':
            return []
        cfg = context.launch_configurations
        use_ns = cfg.get('static_tf_use_namespace', 'true') == 'true'
        pfx    = cfg.get('tf_prefix', '') if use_ns else ''
        parent = f'{pfx}_{cfg.get("static_tf_parent_frame","base_link")}' if pfx else cfg.get('static_tf_parent_frame','base_link')
        child  = f'{pfx}_{cfg.get("static_tf_child_frame","laser_link")}' if pfx else cfg.get('static_tf_child_frame','laser_link')
        return [Node(package='tf2_ros', executable='static_transform_publisher',
                     name='static_tf_publisher',
                     arguments=[cfg.get('static_tf_x','0'), cfg.get('static_tf_y','0'),
                                 cfg.get('static_tf_z','0'), cfg.get('static_tf_roll','0'),
                                 cfg.get('static_tf_pitch','0'), cfg.get('static_tf_yaw','0'),
                                 parent, child],
                     output='screen')]

    def _lidar_tf(context, *args_, **kwargs):
        if context.launch_configurations.get('publish_lidar_static_tf', 'false') != 'true':
            return []
        cfg = context.launch_configurations
        return [Node(package='tf2_ros', executable='static_transform_publisher',
                     name='lidar_static_tf',
                     arguments=[cfg.get('lidar_transform_x','0'), cfg.get('lidar_transform_y','0'),
                                 cfg.get('lidar_transform_z','0'), cfg.get('lidar_transform_roll','0'),
                                 cfg.get('lidar_transform_pitch','0'), cfg.get('lidar_transform_yaw','0'),
                                 'base_link', 'laser_link'],
                     output='screen')]

    from launch.actions import OpaqueFunction

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription(args + [
        OpaqueFunction(function=_create_agv_nodes),
        OpaqueFunction(function=_unity_tf),
        OpaqueFunction(function=_slapstack_tf),
        OpaqueFunction(function=_static_tf),
        OpaqueFunction(function=_lidar_tf),
    ])
