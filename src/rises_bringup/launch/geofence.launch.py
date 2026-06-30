"""
===============================================================================
GEOFENCE SYSTEM LAUNCH FILE
===============================================================================

Launches the complete per-AGV geofence stack:
  - geofence_node               : Spatial safety, obstacle detection, path validation
  - laserscan_preprocessor_node : LiDAR segmentation and clustering
  - message_translator_node      : Optional per-AGV map-update translator
  - fleet_interface_node          : Per-AGV VDA5050 state + order conversion

PARAMETERS → single scenario YAML (in rises_bringup/config/)
  One file covers ALL nodes in this stack.  Choose the file that matches your
  deployment scenario:

    params_default.yaml   – real robot, no simulation
    params_rosbag.yaml    – rosbag replay  (use_sim_time=true, no transforms)
    params_unity.yaml     – Unity bridge   (coordinate transforms, sim time)

  All YAML files support $(env VAR_NAME default) substitution resolved at
  launch time via ParameterFile(allow_substs=True).

COMPOSABLE vs STANDALONE
  use_composable_nodes:=true  (default)
    → single MultiThreadedExecutor container; lower memory, faster IPC
  use_composable_nodes:=false
    → individual lifecycle nodes; easier log isolation / debugging

USAGE
  ros2 launch rises_bringup geofence.launch.py namespace:=agv_0

  # Rosbag scenario
  ros2 launch rises_bringup geofence.launch.py \\
      namespace:=agv_0 \\
      params_file:=$(ros2 pkg prefix rises_bringup)/share/rises_bringup/config/params_rosbag.yaml

  # Override single env-var-backed param without editing the file
  GEOFENCE_SAFETY_RADIUS=0.75 ros2 launch rises_bringup geofence.launch.py namespace:=agv_0

===============================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


# ---------------------------------------------------------------------------
# Helper: wrap the params_file launch arg so $(env …) inside the YAML is
# resolved by the launch substitution engine before the node sees the file.
# ---------------------------------------------------------------------------
def _pf() -> ParameterFile:
    return ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)


def generate_launch_description():
    bringup_dir = get_package_share_directory('rises_bringup')
    config_dir  = os.path.join(bringup_dir, 'config')

    # =========================================================================
    # LAUNCH ARGUMENTS
    # Topology / deployment concerns only.
    # Node parameters live in the scenario YAML.
    # =========================================================================
    args = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS namespace for all nodes in this stack'),

        # -- Single scenario config file --------------------------------------
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'params_default.yaml'),
            description='Scenario parameter file covering all nodes. '
                        'Built-in choices: params_default.yaml, '
                        'params_rosbag.yaml, params_unity.yaml'),

        # -- Node loading strategy --------------------------------------------
        DeclareLaunchArgument(
            'use_composable_nodes', default_value='true',
            description='true  → nodes share a MultiThreadedExecutor container\n'
                        'false → individual standalone lifecycle nodes'),

        # -- Per-AGV translator -----------------------------------------------
        DeclareLaunchArgument(
            'translator_enabled', default_value='false',
            description='Enable optional per-AGV translator node. '
                        'Default: false – use central.launch.py instead'),
        DeclareLaunchArgument(
            'mqtt_bridge_params_file',
            default_value='/ros2_ws/resources/mqtt_bridge_params.yaml',
            description='Path to mqtt_client YAML params (used when translator_enabled=true)'),

        # -- Fleet interface (per-AGV VDA5050 state + order) --------------------
        DeclareLaunchArgument(
            'fleet_interface_enabled', default_value='true',
            description='Enable per-AGV fleet interface node '
                        '(VDA5050 state publishing and order conversion)'),

        # -- Grid-map node ----------------------------------------------------
        DeclareLaunchArgument(
            'gridmap_enabled', default_value='false',
            description='Enable the grid-based occupancy GeofenceGridmapNode alongside '
                        'the spatial node. Disabled by default.'),
        DeclareLaunchArgument(
            'validation_enabled', default_value='false',
            description='Enable standalone validation node for detection latency measurement'),
        DeclareLaunchArgument(
            'launch_leg_filter', default_value='false',
            description='Enable the optional rises_leg_filter node (filters unmatched obstacles '
                        'for likely human legs from obstacle_report)'),
        DeclareLaunchArgument(
            'launch_safety', default_value='true',
            description='Launch the safety node (diagnostics health monitor + halt loop + '
                        'path validation). Started by default so it is always available.'),

        # -- Scan topic remapping ---------------------------------------------
        # Affects WHICH topic is subscribed to → belongs in launch, not YAML.
        DeclareLaunchArgument(
            'scan_topic_remap', default_value='',
            description='Override the laserscan input topic '
                        '(e.g. /my_scanner/scan). Empty = use scan_0'),

        # -- Optional visualisation -------------------------------------------
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Launch RViz2 (local testing only)'),
        DeclareLaunchArgument(
            'rviz_namespace', default_value='',
            description='Namespace whose per-robot topics are wired into RViz'),

        # -- Optional rosbag player -------------------------------------------
        DeclareLaunchArgument(
            'play_rosbag', default_value='false',
            description='Launch an embedded rosbag player alongside the stack'),
        DeclareLaunchArgument(
            'bag_file', default_value='',
            description='Path to rosbag directory / file'),
        DeclareLaunchArgument(
            'rosbag_rate', default_value='1.0',
            description='Rosbag playback rate multiplier'),
        DeclareLaunchArgument(
            'rosbag_loop', default_value='false',
            description='Loop rosbag playback'),
        DeclareLaunchArgument(
            'rosbag_delay', default_value='3.0',
            description='Seconds to wait before starting rosbag'),
        DeclareLaunchArgument(
            'rosbag_remaps', default_value='',
            description='Comma-separated topic remaps for rosbag (/from:=/to,…)'),
        DeclareLaunchArgument(
            'storage', default_value='sqlite3',
            description='Rosbag storage backend (sqlite3 or mcap)'),

        # -- Static TF publishers ---------------------------------------------
        # These affect which nodes are CREATED (topology), not parameters.
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
        DeclareLaunchArgument('lidar_frame_name',      default_value='lidar',
            description='Lidar TF frame name suffix (becomes {tf_prefix}_{name})'),

        DeclareLaunchArgument(
            'publish_unity_tf', default_value='false',
            description='Publish static TF: map → unity_map (+90° Z)'),
        DeclareLaunchArgument(
            'publish_slapstack_tf', default_value='false',
            description='Publish static TF: map → slapstack_map (−90° Z)'),
        DeclareLaunchArgument(
            'target_frame', default_value='map',
            description='Parent frame for coordinate-system TF publishers'),
    ]

    # =========================================================================
    # REMAPPINGS
    # Defined once; used in both composable and standalone code paths.
    # ROS 2 YAML parameter files do NOT support remappings.
    #
    # Override any target topic via environment variables (REMAP_<NODE>_<TOPIC>).
    # Example: REMAP_GEOFENCE_MAP_BOUNDARY=/my/contours changes where the
    #          geofence node subscribes for contour data.
    # =========================================================================
    def _env(var, default):
        return os.environ.get(var, default)

    GEOFENCE_REMAPS = [
        ('lidar_segments',         _env('REMAP_GEOFENCE_LIDAR_SEGMENTS',     'lidar_segments')),
        ('map_boundary',           _env('REMAP_GEOFENCE_MAP_BOUNDARY',       '/warehouse_contours')),
        ('unmatched_obstacles',    _env('REMAP_GEOFENCE_UNMATCHED_OBSTACLES', 'unmatched_obstacles')),
        ('warehouse_contours_viz', _env('REMAP_GEOFENCE_CONTOURS_VIZ',       'warehouse_contours_viz')),
        ('geofence_map_viz',       _env('REMAP_GEOFENCE_MAP_VIZ',            'geofence_map_viz')),
        ('safety_circle_viz',      _env('REMAP_GEOFENCE_SAFETY_CIRCLE_VIZ',  'safety_circle_viz')),
        ('error_segments_viz',     _env('REMAP_GEOFENCE_ERROR_SEGMENTS_VIZ', 'error_segments_viz')),
        ('matched_segments_viz',   _env('REMAP_GEOFENCE_MATCHED_SEGMENTS_VIZ', 'matched_segments_viz')),
        ('validated_path',         _env('REMAP_GEOFENCE_VALIDATED_PATH',     'validated_path')),
        ('validate_path',          _env('REMAP_GEOFENCE_VALIDATE_PATH',      'validate_path')),
        ('set_area_state',         _env('REMAP_GEOFENCE_SET_AREA_STATE',     'set_area_state')),
        ('/tf',                    '/tf'),
        ('/tf_static',             '/tf_static'),
    ]
    LASERSCAN_BASE_REMAPS = [
        ('obstacles',      _env('REMAP_LASERSCAN_OBSTACLES',      'lidar_segments')),
        ('world_scan',     _env('REMAP_LASERSCAN_WORLD_SCAN',     'world_scan')),
        ('processed_scan', _env('REMAP_LASERSCAN_PROCESSED_SCAN', 'processed_scan')),
        ('/tf',            '/tf'),
        ('/tf_static',     '/tf_static'),
    ]
    # Gridmap node uses the same topic layout as the spatial node.
    GRIDMAP_REMAPS = GEOFENCE_REMAPS

    TRANSLATOR_REMAPS = [
        ('obstacle_json',           _env('REMAP_TRANSLATOR_OBSTACLE_JSON',     '/mqtt/discerning_safety_map')),
        ('warehouse_contours_json', _env('REMAP_TRANSLATOR_CONTOURS_JSON',     '/mqtt/warehouse_contours')),
        ('area_locks_json',         _env('REMAP_TRANSLATOR_AREA_LOCKS_JSON',   '/mqtt/area_locks')),
        ('order',                   _env('REMAP_TRANSLATOR_ORDER',             'order')),
        ('/tf',                     '/tf'),
        ('/tf_static',              '/tf_static'),
        ('warehouse_contours',      _env('REMAP_TRANSLATOR_CONTOURS_PUB',      'warehouse_contours')),
        ('warehouse_contours_viz',  _env('REMAP_TRANSLATOR_CONTOURS_VIZ',      '/warehouse_contours_viz')),
        ('map_updates',             _env('REMAP_TRANSLATOR_MAP_UPDATES',       '/warehouse/map_updates')),
        ('incoming_path',           _env('REMAP_TRANSLATOR_INCOMING_PATH',     'incoming_path')),
    ]

    FLEET_INTERFACE_REMAPS = [
        ('order',           _env('REMAP_FLEET_ORDER',            'order')),
        ('incoming_path',   _env('REMAP_FLEET_INCOMING_PATH',    'incoming_path')),
        ('obstacle_report', _env('REMAP_FLEET_OBSTACLE_REPORT',  'obstacle_report')),
        ('obstacle_alert',  _env('REMAP_FLEET_OBSTACLE_ALERT',   'obstacle_alert')),
        ('obstacle_state',  _env('REMAP_FLEET_OBSTACLE_STATE',   'obstacle_state')),
        ('/tf',             '/tf'),
        ('/tf_static',      '/tf_static'),
    ]

    VALIDATION_REMAPS = [
        # Detections come from the geofence's obstacle_report (per-segment
        # positions), NOT the unmatched_obstacle topic (whole-scan centroid in
        # points-only mode). Same namespace as the geofence, so this is a no-op
        # by default but kept explicit for clarity / override.
        ('obstacle_report', _env('REMAP_VALIDATION_OBSTACLE_REPORT', 'obstacle_report')),
        ('obstacle_spawn',     '/validation/obstacle_spawn'),
    ]

    LEG_FILTER_REMAPS = [
        ('obstacle_report', _env('REMAP_LEG_FILTER_OBSTACLE_REPORT', 'obstacle_report')),
    ]

    # Safety node consumes the geofence's intruder output (published as
    # 'unmatched_obstacles'); predicted_occupancy / incoming_path / validate_path
    # resolve within the stack namespace, so only the obstacle input is remapped.
    SAFETY_REMAPS = [
        ('detected_obstacles', _env('REMAP_SAFETY_DETECTED_OBSTACLES', 'unmatched_obstacles')),
        ('/tf', '/tf'),
        ('/tf_static', '/tf_static'),
    ]

    # =========================================================================
    # MAIN NODE LAUNCHER
    # Returns EITHER a ComposableNodeContainer (all nodes inside it)
    #         OR individual standalone LifecycleNodes.
    # =========================================================================
    def _launch_nodes(context, *args_, **kwargs):
        cfg               = context.launch_configurations
        composable        = cfg.get('use_composable_nodes', 'true') == 'true'
        translator        = cfg.get('translator_enabled', 'false') == 'true'
        fleet_interface   = cfg.get('fleet_interface_enabled', 'true') == 'true'
        gridmap           = cfg.get('gridmap_enabled', 'false') == 'true'
        validation        = cfg.get('validation_enabled', 'false') == 'true'
        leg_filter        = cfg.get('launch_leg_filter', 'false') == 'true'
        safety_on         = cfg.get('launch_safety', 'true') == 'true'
        ns                = cfg.get('namespace', '')
        scan_remap        = cfg.get('scan_topic_remap', '')
        scan_input        = scan_remap if scan_remap else 'scan_0'
        ns_lc             = LaunchConfiguration('namespace')
        params            = _pf()
        common_args       = ['--ros-args', '--log-level', 'info']
        laserscan_remaps  = LASERSCAN_BASE_REMAPS + [('scan_0', scan_input)]

        if composable:
            # -----------------------------------------------------------------
            # COMPOSABLE PATH: one container, all nodes inside it.
            # Use intra-process=False to keep TRANSIENT_LOCAL QoS working
            # (Humble bypasses durability semantics for intra-process msgs).
            # -----------------------------------------------------------------
            descriptions = [
                ComposableNode(
                    package='laserscan_preprocessor',
                    plugin='rises::LaserPreprocessorNode',
                    name='laserscan_preprocessor_node',
                    namespace=ns_lc,
                    extra_arguments=[{'use_intra_process_comms': False}],
                    parameters=[params],
                    remappings=laserscan_remaps,
                ),
            ]
            # Spatial node OR gridmap node -- never both: they publish the same
            # topics (unmatched_obstacles, obstacle_alert, obstacle_report) and
            # would collide. gridmap_enabled selects the occupancy-grid variant.
            if gridmap:
                descriptions.append(ComposableNode(
                    package='geofence',
                    plugin='rises::GeofenceGridmapNode',
                    name='geofence_gridmap_node',
                    namespace=ns_lc,
                    extra_arguments=[{'use_intra_process_comms': False}],
                    parameters=[params],
                    remappings=GRIDMAP_REMAPS,
                ))
            else:
                descriptions.append(ComposableNode(
                    package='geofence',
                    plugin='rises::GeofenceNode',
                    name='geofence_node',
                    namespace=ns_lc,
                    extra_arguments=[{'use_intra_process_comms': False}],
                    parameters=[params],
                    remappings=GEOFENCE_REMAPS,
                ))
            if translator:
                descriptions.append(ComposableNode(
                    package='message_translator',
                    plugin='slapstack::MessageTranslatorNode',
                    name='message_translator_node',
                    namespace=ns_lc,
                    extra_arguments=[{'use_intra_process_comms': False}],
                    parameters=[params],
                    remappings=TRANSLATOR_REMAPS,
                ))
            if fleet_interface:
                descriptions.append(ComposableNode(
                    package='fleet_interface',
                    plugin='rises::FleetInterfaceNode',
                    name='fleet_interface_node',
                    namespace=ns_lc,
                    extra_arguments=[{'use_intra_process_comms': False}],
                    parameters=[params],
                    remappings=FLEET_INTERFACE_REMAPS,
                ))
            if validation:
                descriptions.append(ComposableNode(
                    package='geofence',
                    plugin='rises::ValidationNode',
                    name='validation_node',
                    namespace=ns_lc,
                    extra_arguments=[{'use_intra_process_comms': False}],
                    parameters=[params],
                    remappings=VALIDATION_REMAPS,
                ))
            if safety_on:
                descriptions.append(ComposableNode(
                    package='safety',
                    plugin='rises::Safety',
                    name='safety_node',
                    namespace=ns_lc,
                    extra_arguments=[{'use_intra_process_comms': False}],
                    parameters=[params],
                    remappings=SAFETY_REMAPS,
                ))
            result = [ComposableNodeContainer(
                name='geofence_container',
                namespace=ns_lc,
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=descriptions,
                output='screen',
                arguments=common_args,
            )]
            if translator:
                result.append(Node(
                    package='mqtt_client',
                    executable='mqtt_client',
                    name='mqtt_bridge',
                    namespace=ns_lc,
                    output='screen',
                    parameters=[cfg.get('mqtt_bridge_params_file',
                                        '/ros2_ws/resources/mqtt_bridge_params.yaml')],
                ))
            if leg_filter:
                result.append(Node(
                    package='rises_leg_filter',
                    executable='leg_filter_node',
                    name='rises_leg_filter',
                    namespace=ns_lc,
                    output='screen',
                    parameters=[params],
                    remappings=LEG_FILTER_REMAPS,
                ))
            return result

        else:
            # -----------------------------------------------------------------
            # STANDALONE PATH: separate lifecycle nodes – easier debugging.
            # -----------------------------------------------------------------
            nodes = [
                LifecycleNode(
                    package='laserscan_preprocessor',
                    executable='laserscan_preprocessor_node',
                    name='laserscan_preprocessor_node',
                    namespace=ns_lc,
                    output='screen',
                    arguments=common_args,
                    parameters=[params],
                    remappings=laserscan_remaps,
                ),
            ]
            # Spatial node OR gridmap node -- never both (same published topics).
            if gridmap:
                nodes.append(LifecycleNode(
                    package='geofence',
                    executable='geofence_gridmap_node',
                    name='geofence_gridmap_node',
                    namespace=ns_lc,
                    output='screen',
                    arguments=common_args,
                    parameters=[params],
                    remappings=GRIDMAP_REMAPS,
                ))
            else:
                nodes.append(LifecycleNode(
                    package='geofence',
                    executable='geofence_spatial_node',
                    name='geofence_node',
                    namespace=ns_lc,
                    output='screen',
                    arguments=common_args,
                    parameters=[params],
                    remappings=GEOFENCE_REMAPS,
                ))
            if translator:
                nodes.append(LifecycleNode(
                    package='message_translator',
                    executable='message_translator_node',
                    name='message_translator_node',
                    namespace=ns_lc,
                    output='screen',
                    arguments=common_args,
                    parameters=[params],
                    remappings=TRANSLATOR_REMAPS,
                ))
                nodes.append(Node(
                    package='mqtt_client',
                    executable='mqtt_client',
                    name='mqtt_bridge',
                    namespace=ns_lc,
                    output='screen',
                    parameters=[cfg.get('mqtt_bridge_params_file',
                                        '/ros2_ws/resources/mqtt_bridge_params.yaml')],
                ))
            if fleet_interface:
                nodes.append(Node(
                    package='fleet_interface',
                    executable='fleet_interface_node',
                    name='fleet_interface_node',
                    namespace=ns_lc,
                    output='screen',
                    arguments=common_args,
                    parameters=[params],
                    remappings=FLEET_INTERFACE_REMAPS,
                ))
            if validation:
                nodes.append(Node(
                    package='geofence',
                    executable='geofence_validation_node',
                    name='validation_node',
                    namespace=ns_lc,
                    output='screen',
                    parameters=[params],
                    remappings=VALIDATION_REMAPS,
                ))
            if leg_filter:
                nodes.append(Node(
                    package='rises_leg_filter',
                    executable='leg_filter_node',
                    name='rises_leg_filter',
                    namespace=ns_lc,
                    output='screen',
                    parameters=[params],
                    remappings=LEG_FILTER_REMAPS,
                ))
            if safety_on:
                nodes.append(Node(
                    package='safety',
                    executable='safety_node',
                    name='safety_node',
                    namespace=ns_lc,
                    output='screen',
                    parameters=[params],
                    remappings=SAFETY_REMAPS,
                ))
            return nodes

    # =========================================================================
    # SUPPORTING NODES (all optional, controlled by launch args)
    # =========================================================================

    def _static_tf(context, *args_, **kwargs):
        if context.launch_configurations.get('publish_static_tf', 'false') != 'true':
            return []
        cfg = context.launch_configurations
        ns  = cfg.get('namespace', '')
        pfx = (cfg.get('tf_prefix', '') or ns) if cfg.get('static_tf_use_namespace', 'true') == 'true' else ''
        parent = f'{pfx}_{cfg.get("static_tf_parent_frame", "base_link")}' if pfx else cfg.get('static_tf_parent_frame', 'base_link')
        child  = f'{pfx}_{cfg.get("static_tf_child_frame",  "laser_link")}' if pfx else cfg.get('static_tf_child_frame', 'laser_link')
        return [Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'static_tf_{ns}' if ns else 'static_tf_publisher', namespace=ns,
            arguments=[cfg.get('static_tf_x','0'), cfg.get('static_tf_y','0'),
                       cfg.get('static_tf_z','0'), cfg.get('static_tf_roll','0'),
                       cfg.get('static_tf_pitch','0'), cfg.get('static_tf_yaw','0'),
                       parent, child],
            output='screen',
        )]

    def _lidar_tf(context, *args_, **kwargs):
        if context.launch_configurations.get('publish_lidar_static_tf', 'false') != 'true':
            return []
        cfg = context.launch_configurations
        ns  = cfg.get('namespace', '')
        pfx = cfg.get('tf_prefix', '') or ns
        parent = f'{pfx}_base_link'  if pfx else 'base_link'
        lidar_frame = cfg.get('lidar_frame_name', 'lidar')
        child  = f'{pfx}_{lidar_frame}' if pfx else lidar_frame
        return [Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'lidar_tf_{ns}' if ns else 'lidar_static_tf', namespace=ns,
            arguments=[cfg.get('lidar_transform_x','0'), cfg.get('lidar_transform_y','0'),
                       cfg.get('lidar_transform_z','0'), cfg.get('lidar_transform_roll','0'),
                       cfg.get('lidar_transform_pitch','0'), cfg.get('lidar_transform_yaw','0'),
                       parent, child],
            output='screen',
        )]

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

    def _rosbag(context, *args_, **kwargs):
        if context.launch_configurations.get('play_rosbag', 'false') != 'true':
            return []
        bag = context.launch_configurations.get('bag_file', '')
        if not bag:
            print('[WARN] play_rosbag=true but bag_file is empty – skipping')
            return []
        cfg = context.launch_configurations
        cmd = ['ros2', 'bag', 'play', bag,
               '-s', cfg.get('storage', 'sqlite3'),
               '-r', cfg.get('rosbag_rate', '1.0'),
               '--read-ahead-queue-size', '100',
               '--disable-keyboard-controls']
        if cfg.get('use_sim_time', 'false') == 'true':
            cmd.append('--clock')
        if cfg.get('rosbag_loop', 'false') == 'true':
            cmd.append('--loop')
        for remap in cfg.get('rosbag_remaps', '').replace('\n', '').split(','):
            remap = remap.strip()
            if ':=' in remap:
                cmd += ['--remap', remap]
        return [ExecuteProcess(cmd=cmd, output='screen')]

    # RViz – wires per-robot topics from the specified namespace
    rviz_namespace = LaunchConfiguration('rviz_namespace')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(bringup_dir, '..', '..', '..', '..', 'resources', 'rises_local.rviz')],
        output='screen',
        parameters=[_pf()],
        remappings=[
            ('warehouse_contours_viz', '/warehouse_contours_viz'),
            ('geofence_map_viz',       [rviz_namespace, '/geofence_map_viz']),
            ('safety_circle_viz',      [rviz_namespace, '/safety_circle_viz']),
            ('error_segments_viz',     [rviz_namespace, '/error_segments_viz']),
            ('matched_segments_viz',   [rviz_namespace, '/matched_segments_viz']),
            ('validated_path',         [rviz_namespace, '/validated_path']),
            ('processed_scan',         [rviz_namespace, '/processed_scan']),
            ('lidar_segments',         [rviz_namespace, '/lidar_segments']),
            ('unmatched_obstacles',    [rviz_namespace, '/unmatched_obstacles']),
        ],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription(args + [
        # Main nodes (composable container OR standalone lifecycle nodes)
        OpaqueFunction(function=_launch_nodes),

        # Optional TF publishers
        OpaqueFunction(function=_static_tf),
        OpaqueFunction(function=_lidar_tf),
        OpaqueFunction(function=_unity_tf),
        OpaqueFunction(function=_slapstack_tf),

        # Delayed rosbag player
        OpaqueFunction(function=lambda ctx, *a, **kw: [
            TimerAction(
                period=float(ctx.launch_configurations.get('rosbag_delay', '3.0')),
                actions=[OpaqueFunction(function=_rosbag)]
            )
        ]),

        # Visualisation
        rviz_node,
    ])
