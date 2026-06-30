from dataclasses import dataclass

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


@dataclass
class RisesConfig:
    use_composition: LaunchConfiguration
    bag_file: LaunchConfiguration
    record_bag: LaunchConfiguration
    record_topics: LaunchConfiguration
    storage_backend: LaunchConfiguration
    log_level: LaunchConfiguration
    namespace: LaunchConfiguration
    play_rosbag: LaunchConfiguration
    rosbag_delay: LaunchConfiguration
    rviz_config_file: LaunchConfiguration
    launch_rviz: LaunchConfiguration
    mqtt_params_path: LaunchConfiguration

    @classmethod
    def load_config(cls):
        return cls(
            use_composition=LaunchConfiguration('use_composition'),
            bag_file=LaunchConfiguration('bag_file'),
            record_bag=LaunchConfiguration('record_bag'),
            record_topics=LaunchConfiguration('record_topics'),
            storage_backend=LaunchConfiguration('storage_backend'),
            mqtt_params_path=LaunchConfiguration('mqtt_params_path'),
            log_level=LaunchConfiguration('log_level'),
            namespace=LaunchConfiguration('namespace',
                default=EnvironmentVariable('NS', default_value='')),
            play_rosbag=LaunchConfiguration('play_rosbag'),
            rosbag_delay=LaunchConfiguration('rosbag_delay'),
            launch_rviz=LaunchConfiguration('launch_rviz'),
            rviz_config_file=LaunchConfiguration('rviz_config_file'),
        )

    @classmethod
    def get_arg_declarations(cls):
        bridge_params_path = '/workspace/resources/mqtt_bridge_params.yaml'

        return [DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='true: use composable nodes; '
                        'false: launch standalone nodes'
            ),
            DeclareLaunchArgument(
                'bag_file', default_value='',
                description='Path to rosbag file to play'
            ),
            DeclareLaunchArgument(
                'storage_backend', default_value='sqlite3',
                description='Path to rosbag file to play'
            ),
            DeclareLaunchArgument(
                'record_bag', default_value='false',
                description='Whether to record a rosbag'
            ),
            DeclareLaunchArgument(
                'record_topics',
                default_value='/diagnostics_agg /obstacles',
                description='Topics to record in the bag (list of strings)'
            ),
            DeclareLaunchArgument(
                'mqtt_params_path', default_value=bridge_params_path,
                description='Path to MQTT bridge YAML file'
            ),
            DeclareLaunchArgument(
                'log_level', default_value='info',
                description='Default log level for all nodes'
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='ROS namespace for all nodes'
            ),
            DeclareLaunchArgument(
                'play_rosbag', default_value='true',
                description='Whether to play a rosbag'
            ),
            DeclareLaunchArgument(
                'rosbag_delay', default_value='5.0',
                description='Delay (in seconds) before rosbag playback starts'
            ),
            DeclareLaunchArgument(
                'launch_rviz', default_value='false',
                description='Whether to launch RViz2'
            ),
            DeclareLaunchArgument(
                'rviz_config_file', default_value='',
                description='Path to RViz2 config file'
            )]
