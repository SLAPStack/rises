#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch file for testing the laser preprocessor with simulated data."""
    return LaunchDescription([
        # Test configuration arguments
        DeclareLaunchArgument(
            'test_points_only_mode',
            default_value='false',
            description='Test the points-only publishing mode'
        ),
        
        # Laser preprocessor node in test mode
        Node(
            package='laserscan_preprocessor',
            executable='laserscan_preprocessor_node',
            name='laser_preprocessor_test',
            parameters=[{
                'publish_points_only': LaunchConfiguration('test_points_only_mode'),
                'segment_distance_threshold': 0.3,
                'segment_angle_threshold_deg': 45.0,
                'target_frame': 'base_link',
                'tf_prefix': '',
                'prefix_global_frame': False,

                'laser_frames': ['test_laser'],
                'laser_heights': [0.0],
                
                # Test-friendly parameters
                'dbscan_eps': 0.2,
                'dbscan_min_points': 2,
                'region_grow_threshold': 0.15,
                'min_segment_size': 2,
                'outlier_removal_factor': 2.0,
                'use_adaptive_thresholding': False,
            }],
            remappings=[
                ('scan/test_laser', '/test_scan'),
            ],
            output='screen'
        ),
        
        # Static transform publisher for test laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='test_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'test_laser'],
            output='screen'
        ),
        
        # Test scan publisher (for manual testing)
        Node(
            package='laserscan_preprocessor',
            executable='test_scan_publisher',  # Would need to be created
            name='test_scan_publisher',
            parameters=[{
                'publish_rate': 10.0,
                'frame_id': 'test_laser',
                'num_points': 180,
                'max_range': 10.0,
                'obstacle_start_angle': -30,
                'obstacle_end_angle': 30,
                'obstacle_distance': 2.0,
            }],
            remappings=[
                ('scan', '/test_scan'),
            ],
            output='screen',
            condition='false'  # Disabled by default
        )
    ])