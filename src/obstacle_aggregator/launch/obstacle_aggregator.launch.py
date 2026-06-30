from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'agv_namespaces', default_value="['agv_0', 'agv_1']",
            description='List of AGV namespaces to aggregate obstacles from'),
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='5.0',
            description='Rate at which to publish combined obstacle report'),

        Node(
            package='obstacle_aggregator',
            executable='obstacle_aggregator_node',
            name='obstacle_aggregator',
            parameters=[{
                'agv_namespaces': LaunchConfiguration('agv_namespaces'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }],
            output='screen',
        ),
    ])
