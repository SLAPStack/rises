from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS namespace (should match the geofence node namespace)'),
        DeclareLaunchArgument(
            'observation_window_sec', default_value='10.0',
            description='Sliding window for obstacle observations (seconds)'),
        DeclareLaunchArgument(
            'prediction_horizon_sec', default_value='30.0',
            description='How far ahead to predict (seconds)'),
        DeclareLaunchArgument(
            'prediction_step_sec', default_value='2.0',
            description='Time step between prediction samples (seconds)'),
        DeclareLaunchArgument(
            'grid_resolution', default_value='0.25',
            description='OccupancyGrid cell size (meters)'),
        DeclareLaunchArgument(
            'grid_width', default_value='30.0',
            description='OccupancyGrid width (meters)'),
        DeclareLaunchArgument(
            'grid_height', default_value='30.0',
            description='OccupancyGrid height (meters)'),
        DeclareLaunchArgument(
            'gaussian_sigma', default_value='1.0',
            description='Spatial spread of prediction Gaussian (meters)'),
        DeclareLaunchArgument(
            'frame_id', default_value='map',
            description='TF frame for the occupancy grid'),

        Node(
            package='obstacle_heatmap_predictor',
            executable='heatmap_predictor_node',
            name='obstacle_heatmap_predictor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'observation_window_sec': LaunchConfiguration('observation_window_sec'),
                'prediction_horizon_sec': LaunchConfiguration('prediction_horizon_sec'),
                'prediction_step_sec': LaunchConfiguration('prediction_step_sec'),
                'grid_resolution': LaunchConfiguration('grid_resolution'),
                'grid_width': LaunchConfiguration('grid_width'),
                'grid_height': LaunchConfiguration('grid_height'),
                'gaussian_sigma': LaunchConfiguration('gaussian_sigma'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
            output='screen',
        ),
    ])
