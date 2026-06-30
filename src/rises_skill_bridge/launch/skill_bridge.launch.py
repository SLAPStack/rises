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
            'geofence_node_name', default_value='geofence_node',
            description='Name of the geofence node whose services to bridge'),
        DeclareLaunchArgument(
            'service_timeout_sec', default_value='10',
            description='Timeout in seconds for service calls to the geofence node'),

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
    ])
