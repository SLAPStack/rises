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
            'report_throttle_hz', default_value='1.0',
            description='Max publish rate for obstacle summary'),
        DeclareLaunchArgument(
            'odom_throttle_hz', default_value='2.0',
            description='Max publish rate for robot position'),
        DeclareLaunchArgument(
            'geometry_republish_sec', default_value='60.0',
            description='Periodic re-publish interval for warehouse geometry'),

        Node(
            package='fiware_bridge',
            executable='fiware_bridge_node',
            name='fiware_bridge',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'report_throttle_hz': LaunchConfiguration('report_throttle_hz'),
                'odom_throttle_hz': LaunchConfiguration('odom_throttle_hz'),
                'geometry_republish_sec': LaunchConfiguration('geometry_republish_sec'),
            }],
            output='screen',
        ),
    ])
