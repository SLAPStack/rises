#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'publish_points_only',
            default_value='false',
            description='Whether to publish segments as individual points instead of fitted shapes'
        ),
        DeclareLaunchArgument(
            'segment_distance_threshold',
            default_value='0.2',
            description='Distance threshold for segmentation'
        ),
        DeclareLaunchArgument(
            'segment_angle_threshold_deg',
            default_value='30.0',
            description='Angle threshold for segmentation (degrees)'
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_link',
            description='Target frame for point cloud transformation'
        ),
        
        # Laser preprocessor node
        Node(
            package='laserscan_preprocessor',
            executable='laserscan_preprocessor_node',
            name='laser_preprocessor',
            parameters=[{
                'publish_points_only': LaunchConfiguration('publish_points_only'),
                'segment_distance_threshold': LaunchConfiguration('segment_distance_threshold'),
                'segment_angle_threshold_deg': LaunchConfiguration('segment_angle_threshold_deg'),
                'target_frame': LaunchConfiguration('target_frame'),
                'tf_prefix': '',
                'prefix_global_frame': False,
                'laser_frames': ['front_laser', 'rear_laser'],
                'laser_heights': [0.2, 0.2],
                
                # Advanced segmentation parameters
                'dbscan_eps': 0.15,
                'dbscan_min_points': 3,
                'region_grow_threshold': 0.1,
                'min_segment_size': 3,
                'outlier_removal_factor': 1.5,
                'use_adaptive_thresholding': True,
            }],
            remappings=[
                ('scan/front_laser', '/front_laser/scan'),
                ('scan/rear_laser', '/rear_laser/scan'),
            ],
            output='screen'
        )
    ])