"""Launch the task examples node alongside the skill bridge for testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="",
                              description="Node namespace (e.g., agv_0)"),
        DeclareLaunchArgument("skill_timeout_sec", default_value="30",
                              description="Timeout for each skill call (seconds)"),

        Node(
            package="rises_task_examples",
            executable="task_examples_node",
            name="task_examples",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                "skill_timeout_sec": LaunchConfiguration("skill_timeout_sec"),
            }],
            output="screen",
        ),
    ])
