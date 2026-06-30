from typing import List, Union

from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, \
    EmitEvent, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from rises_bringup.rises_config import RisesConfig


def get_bag_playback_action(cfg: 'RisesConfig') -> TimerAction:
    rosbag_play_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            '-s', cfg.storage_backend,
            cfg.bag_file,
            '--read-ahead-queue-size', '100',
            '--remap',
            '/mqtt/agv/id0/order:=/agv_0/order',
            '/mqtt/agv/tf:=/tf_raw',
            '/mqtt/discerning_safety_map:=/discerning_safety_map',
            '/mqtt/warehouse_contours:=/warehouse_contours_mqtt',

            '/validation:=/validation_mqtt',
        ],
        output='screen',
        condition=IfCondition(cfg.play_rosbag)
    )
    delayed_rosbag_play = TimerAction(
        period=cfg.rosbag_delay,
        actions=[rosbag_play_process],
        condition=IfCondition(cfg.play_rosbag)
    )
    return delayed_rosbag_play


def get_rosbag_record_action(context: object) -> OpaqueFunction:
    def fn(context, *_, **__):
        # Evaluate LaunchConfigurations at runtime
        record_topics_value = LaunchConfiguration('record_topics').perform(
            context)
        log_level_value = LaunchConfiguration('log_level').perform(context)
        namespace_value = LaunchConfiguration('namespace').perform(context)
        record_bag_value = LaunchConfiguration('record_bag').perform(context)

        # Split the whitespace-separated topics string into a list
        topics_list = record_topics_value.split() if record_topics_value else []

        # Create the Node dynamically
        return [
            Node(
                package='rosbag2_transport',
                executable='record',
                name='rosbag_recorder',
                namespace=namespace_value,
                output='screen',
                arguments=[
                    '--ros-args',
                    '--log-level', log_level_value,
                ] + topics_list,
                condition=IfCondition(record_bag_value)
            )
        ]
    return OpaqueFunction(function=fn)


def get_rviz_action(cfg: 'RisesConfig') -> TimerAction:
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', cfg.rviz_config_file],
        condition=IfCondition(cfg.launch_rviz)
    )
    # To ensure simulated time is already being published before rviz launches
    return TimerAction(
        period='10.0',  # 10-second delay
        actions=[rviz_node],
        condition=IfCondition(cfg.launch_rviz)
    )


def get_node_transition_actions(
        lifecycle_nodes: List['LifecycleNode']) -> List['RegisterEventHandler']:
    actions = []
    for node in lifecycle_nodes:
        configure = RegisterEventHandler(
            OnProcessStart(
                target_action=node,
                on_start=[EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE))]))
        activate = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state='configuring',
                goal_state='inactive',
                entities=[EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE))]))
        actions += [configure, activate]
    return actions


def get_static_tf_node(namespace: Union['LaunchConfiguration', str]) -> Node:
    if isinstance(namespace, str):
        base_link = f'{namespace}_base_link'
        lidar_link = f'{namespace}_lidar'
    else:
        assert isinstance(namespace, LaunchConfiguration)
        base_link = PythonExpression(["'", namespace, "'", " + '_base_link'"])
        lidar_link = PythonExpression(["'", namespace, "'", " + '_lidar'"])
    # --- map -> base_link ---
    # Lidar Data is center around AGV. Hence identity transform.
    # In future, this would hold the transform to go from AGV to Lidar location.
    # But for now we assume Lidar it as AGV Center.
    agv_lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='agv0_to_laser_scan',
        arguments=['0', '0', '0', '1.5708', '0', '0', base_link,
                   lidar_link],
        output='screen'
    )
    return agv_lidar_transform


def get_tcp_node(cfg: 'RisesConfig') -> Node:
    tcp_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='tcp_node',
        namespace=cfg.namespace,
        output='screen',
        parameters=[{'ROS_IP': "0.0.0.0", 'ROS_TCP_PORT': 10000}],
    )
    return tcp_node
