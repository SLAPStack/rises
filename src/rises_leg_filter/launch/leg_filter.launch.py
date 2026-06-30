"""Launch file for the ARISE leg filter node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="",
                              description="Node namespace (e.g., agv_0)"),
        DeclareLaunchArgument("frame_id", default_value="map",
                              description="Reference frame for published TF"),
        DeclareLaunchArgument("min_width", default_value="0.15",
                              description="Minimum obstacle width (m)"),
        DeclareLaunchArgument("max_width", default_value="0.8",
                              description="Maximum obstacle width (m)"),
        DeclareLaunchArgument("min_velocity", default_value="0.05",
                              description="Minimum average velocity (m/s)"),
        DeclareLaunchArgument("base_confidence", default_value="0.3",
                              description="Confidence for LiDAR-only detections"),
        DeclareLaunchArgument("boosted_confidence", default_value="0.7",
                              description="Confidence when camera-confirmed"),
        DeclareLaunchArgument("camera_match_radius", default_value="1.5",
                              description="Max distance to match LiDAR to camera human (m)"),

        Node(
            package="rises_leg_filter",
            executable="leg_filter_node",
            name="rises_leg_filter",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                "frame_id": LaunchConfiguration("frame_id"),
                "min_width": LaunchConfiguration("min_width"),
                "max_width": LaunchConfiguration("max_width"),
                "min_velocity": LaunchConfiguration("min_velocity"),
                "base_confidence": LaunchConfiguration("base_confidence"),
                "boosted_confidence": LaunchConfiguration("boosted_confidence"),
                "camera_match_radius": LaunchConfiguration("camera_match_radius"),
            }],
            output="screen",
        ),
    ])
