from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS namespace (should match the skill bridge namespace)'),
        DeclareLaunchArgument(
            'action_timeout_sec', default_value='30',
            description='Timeout in seconds for skill action calls'),

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
    ])
