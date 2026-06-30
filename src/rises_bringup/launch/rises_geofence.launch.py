"""
===============================================================================
ARISE GEOFENCE LAUNCH FILE
===============================================================================

Launches the complete geofence stack WITH ARISE integration nodes:
  - All nodes from geofence.launch.py (geofence, preprocessor, fleet, translator)
  - rises_skill_bridge        : Wraps geofence services as ROS4HRI skill action servers
  - rises_mission_controller  : Maps intents to missions/tasks/skills
  - obstacle_heatmap_predictor: Trajectory prediction for HRI
  - fiware_bridge             : Translates ROS 2 msgs to JSON for DDS Enabler

The core geofence stack runs without these ARISE nodes.
This launch file adds them as an optional overlay.

USAGE
  ros2 launch rises_bringup rises_geofence.launch.py namespace:=agv_0

  # With rosbag
  ros2 launch rises_bringup rises_geofence.launch.py \\
      namespace:=agv_0 \\
      params_file:=$(ros2 pkg prefix rises_bringup)/share/rises_bringup/config/params_rosbag.yaml

  # Disable specific ARISE nodes
  ros2 launch rises_bringup rises_geofence.launch.py \\
      namespace:=agv_0 \\
      launch_fiware_bridge:=false \\
      launch_heatmap:=false

===============================================================================
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('rises_bringup')

    return LaunchDescription([
        # =====================================================================
        # ARISE-specific launch arguments
        # =====================================================================
        DeclareLaunchArgument(
            'launch_skill_bridge', default_value='true',
            description='Launch the ROS4HRI skill bridge node'),
        DeclareLaunchArgument(
            'launch_mission_controller', default_value='true',
            description='Launch the ARISE mission controller node'),
        DeclareLaunchArgument(
            'launch_heatmap', default_value='true',
            description='Launch the obstacle heatmap predictor node'),
        DeclareLaunchArgument(
            'launch_fiware_bridge', default_value='true',
            description='Launch the FIWARE bridge node (JSON topic translator)'),

        # Skill bridge params
        DeclareLaunchArgument(
            'geofence_node_name', default_value='geofence_node',
            description='Name of the geofence node for skill bridge service calls'),
        DeclareLaunchArgument(
            'service_timeout_sec', default_value='10',
            description='Timeout for skill bridge service calls'),

        # Mission controller params
        DeclareLaunchArgument(
            'action_timeout_sec', default_value='30',
            description='Timeout for mission controller action calls'),

        # Heatmap params
        DeclareLaunchArgument(
            'prediction_horizon_sec', default_value='30.0',
            description='How far ahead to predict obstacle positions'),
        DeclareLaunchArgument(
            'heatmap_publish_rate_hz', default_value='2.0',
            description='Heatmap occupancy grid publish rate'),
        DeclareLaunchArgument(
            'heatmap_grid_center_x', default_value='0.0',
            description='World X of the heatmap grid center (meters). '
                        'Set to the warehouse center X.'),
        DeclareLaunchArgument(
            'heatmap_grid_center_y', default_value='0.0',
            description='World Y of the heatmap grid center (meters). '
                        'Set to the warehouse center Y.'),
        DeclareLaunchArgument(
            'heatmap_grid_width', default_value='200.0',
            description='Heatmap grid width (meters). Default covers 200m.'),
        DeclareLaunchArgument(
            'heatmap_grid_height', default_value='200.0',
            description='Heatmap grid height (meters). Default covers 200m.'),
        DeclareLaunchArgument(
            'heatmap_min_observations', default_value='3',
            description='Minimum scan observations before a track appears on the heatmap'),
        DeclareLaunchArgument(
            'heatmap_gaussian_sigma', default_value='1.0',
            description='Gaussian spread (meters) stamped at each predicted position'),
        DeclareLaunchArgument(
            'heatmap_prediction_step_sec', default_value='0.5',
            description='Time step between prediction stamps (seconds). '
                        'Smaller = smoother trail; larger = discrete dots'),

        # FIWARE bridge params
        DeclareLaunchArgument(
            'fiware_report_throttle_hz', default_value='1.0',
            description='Max publish rate for FIWARE obstacle summary'),
        DeclareLaunchArgument(
            'fiware_geometry_republish_sec', default_value='60.0',
            description='Periodic warehouse geometry re-publish interval'),

        # =====================================================================
        # Pass through all geofence.launch.py arguments
        # =====================================================================
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            bringup_share, 'config', 'params_default.yaml'])),
        DeclareLaunchArgument('use_composable_nodes', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('play_rosbag', default_value='false'),
        DeclareLaunchArgument('bag_file', default_value=''),
        DeclareLaunchArgument('rosbag_rate', default_value='1.0'),

        # =====================================================================
        # Include the base geofence stack
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, 'launch', 'geofence.launch.py'])
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'params_file': LaunchConfiguration('params_file'),
                'use_composable_nodes': LaunchConfiguration('use_composable_nodes'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'play_rosbag': LaunchConfiguration('play_rosbag'),
                'bag_file': LaunchConfiguration('bag_file'),
                'rosbag_rate': LaunchConfiguration('rosbag_rate'),
            }.items(),
        ),

        # =====================================================================
        # ARISE Skill Bridge
        # =====================================================================
        GroupAction(
            condition=IfCondition(LaunchConfiguration('launch_skill_bridge')),
            actions=[
                Node(
                    package='rises_skill_bridge',
                    executable='skill_bridge_node',
                    name='skill_bridge_node',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'geofence_node_name': LaunchConfiguration('geofence_node_name'),
                        'service_timeout_sec': LaunchConfiguration('service_timeout_sec'),
                    }],
                    output='screen',
                ),
            ],
        ),

        # =====================================================================
        # ARISE Mission Controller
        # =====================================================================
        GroupAction(
            condition=IfCondition(LaunchConfiguration('launch_mission_controller')),
            actions=[
                Node(
                    package='rises_mission_controller',
                    executable='mission_controller_node',
                    name='rises_mission_controller',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'action_timeout_sec': LaunchConfiguration('action_timeout_sec'),
                    }],
                    output='screen',
                ),
            ],
        ),

        # =====================================================================
        # Obstacle Heatmap Predictor
        # =====================================================================
        GroupAction(
            condition=IfCondition(LaunchConfiguration('launch_heatmap')),
            actions=[
                Node(
                    package='obstacle_heatmap_predictor',
                    executable='heatmap_predictor_node',
                    name='obstacle_heatmap_predictor',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'use_sim_time': True,
                        'prediction_horizon_sec': LaunchConfiguration('prediction_horizon_sec'),
                        'prediction_step_sec': LaunchConfiguration('heatmap_prediction_step_sec'),
                        'publish_rate_hz': LaunchConfiguration('heatmap_publish_rate_hz'),
                        'grid_center_x': LaunchConfiguration('heatmap_grid_center_x'),
                        'grid_center_y': LaunchConfiguration('heatmap_grid_center_y'),
                        'grid_width': LaunchConfiguration('heatmap_grid_width'),
                        'grid_height': LaunchConfiguration('heatmap_grid_height'),
                        'min_observations': LaunchConfiguration('heatmap_min_observations'),
                        'gaussian_sigma': LaunchConfiguration('heatmap_gaussian_sigma'),
                    }],
                    output='screen',
                ),
            ],
        ),

        # =====================================================================
        # FIWARE Bridge (JSON topic translator for DDS Enabler)
        # =====================================================================
        GroupAction(
            condition=IfCondition(LaunchConfiguration('launch_fiware_bridge')),
            actions=[
                Node(
                    package='fiware_bridge',
                    executable='fiware_bridge_node',
                    name='fiware_bridge',
                    namespace=LaunchConfiguration('namespace'),
                    parameters=[{
                        'report_throttle_hz': LaunchConfiguration('fiware_report_throttle_hz'),
                        'geometry_republish_sec': LaunchConfiguration('fiware_geometry_republish_sec'),
                        'obstacles_json_file': EnvironmentVariable(
                            'GEOFENCE_OBSTACLES_JSON_FILE', default_value=''),
                    }],
                    remappings=[
                        # The central translator publishes contours on the global /warehouse_contours
                        # topic. The fiware_bridge runs in the AGV namespace so its relative
                        # 'warehouse_contours' subscription would resolve to /agv_0/warehouse_contours
                        # (no publisher). Pin it to the global topic so contours reach the bridge.
                        ('warehouse_contours', '/warehouse_contours'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
