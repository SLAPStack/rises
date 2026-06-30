"""
Configuration data classes for simulation orchestration
"""

from dataclasses import dataclass, field
from enum import Enum


class ContainerStatus(Enum):
    """Container lifecycle states"""
    CREATED = "created"
    RUNNING = "running"
    EXITED = "exited"
    FAILED = "failed"
    UNKNOWN = "unknown"


class DeployMode(Enum):
    """How containers are organized"""
    CENTRAL = "central"   # central.launch.py + rosbag (central_entrypoint.sh)
    UNITY = "unity"       # unity.launch.py + ROS-TCP-Endpoint (unity_entrypoint.sh)


@dataclass
class DockerConfig:
    """Docker-specific configuration"""
    network_mode: str = "host"
    restart_policy: str = "no"
    remove_on_exit: bool = False
    privileged: bool = False
    log_driver: str = "json-file"
    log_max_size: str = "10m"
    log_max_file: int = 3


@dataclass
class BridgeConfig:
    """Bridge container configuration (central or unity mode)

    Central mode: launches central.launch.py (translator + rosbag)
    Unity mode:   launches unity.launch.py (TCP endpoint + translator)
    """
    image: str = ""             # Auto-set from mode if empty
    dockerfile: str = "central.dockerfile"
    container_name: str = ""    # Auto-set from mode if empty

    # Translator settings
    enable_buffering: bool = True
    buffer_timeout: float = 1.0
    replay_rate: float = 33.0
    target_frame: str = "map"
    base_link_pub_rate: float = 20.0
    use_sim_time: bool = False
    launch_rviz: bool = False
    rviz_namespace: str = "agv_0"
    rviz_config_file: str = ""  # empty = use container default (rises.rviz)

    # Rosbag settings
    play_rosbag: bool = True
    bag_file: str = ""
    rosbag_rate: float = 1.0
    rosbag_loop: bool = False
    rosbag_delay: float = 5.0
    rosbag_remaps: str = ""
    storage: str = "sqlite3"

    # Geofence ready waiting
    wait_for_geofence_ready: bool = False
    geofence_ready_timeout: float = 30.0

    # Map update delivery: True = service call (synchronized), False = topic (fire-and-forget)
    use_service_for_map_updates: bool = True

    # Y-coordinate normalization for mixed-convention data sources
    normalize_obstacle_y: bool = False

    # Unity TCP endpoint (only used in unity mode)
    tcp_ip: str = "0.0.0.0"
    tcp_port: int = 10000

    # Topic suppression (remap MQTT map topics to /unused/*)
    suppress_map_topics: bool = False

    # MQTT topic names (override defaults in launch files)
    mqtt_obstacle_topic: str = ""
    mqtt_contours_topic: str = ""
    mqtt_order_topic: str = ""
    mqtt_tf_topic: str = ""
    mqtt_scan_topic: str = ""
    mqtt_validation_topic: str = ""

    # Environment overrides
    env_vars: dict[str, str] = field(default_factory=dict)


# Backwards compatibility alias
CentralConfig = BridgeConfig


@dataclass
class AGVConfig:
    """Individual AGV container configuration"""
    image: str = "rises:base"
    dockerfile: str = "central.dockerfile"
    container_prefix: str = "agv"

    # Scenario selection (selects params_*.yaml in entrypoint.sh)
    # Options: "default", "rosbag", "unity"
    scenario: str = "default"

    # ARISE mode: launch rises_geofence.launch.py (full ARISE stack) instead
    # of geofence.launch.py (core only). Enables skill bridge, heatmap predictor,
    # fiware bridge, and mission controller inside the AGV container.
    rises_mode: bool = False
    launch_skill_bridge: bool = True
    launch_mission_controller: bool = True
    launch_heatmap: bool = True
    launch_fiware_bridge: bool = True
    launch_leg_filter: bool = False

    # AGV deploy mode within a single container
    # Options: "per_agv" (one AGV per container), "multi" (all AGVs in one container)
    agv_deploy_mode: str = "per_agv"

    # Geofence settings
    enable_safety_circle: bool = True
    safety_circle_radius: float = 0.5
    visualizer_enabled: bool = False
    use_namespace_for_map_frame: bool = False
    target_frame: str = "map"
    base_link_frame: str = "base_link"

    # Robot filtering
    enable_robot_filtering: bool = False
    robot_footprint_type: str = "circle"
    robot_footprint_radius: float = 0.5
    robot_footprint_margin: float = 0.1

    # Laserscan settings
    dynamic_laser_detection: bool = True
    laser_frames: list[str] | None = None
    laser_frame_ids: list[str] | None = None
    laser_heights: list[float] | None = None
    segment_distance_threshold: float = 0.2
    segment_angle_threshold_deg: float = 30.0

    # Static TF publisher settings (generic)
    publish_static_tf: bool = False
    static_tf_parent_frame: str = "base_link"
    static_tf_child_frame: str = "lidar"
    static_tf_x: float = 0.0
    static_tf_y: float = 0.0
    static_tf_z: float = 0.0
    static_tf_roll: float = 0.0
    static_tf_pitch: float = 0.0
    static_tf_yaw: float = 0.0
    static_tf_use_namespace: bool = True

    # Lidar static TF publisher (base_link -> laser_link)
    publish_lidar_static_tf: bool = False
    lidar_transform_x: float = 0.0
    lidar_transform_y: float = 0.0
    lidar_transform_z: float = 0.0
    lidar_transform_roll: float = 0.0
    lidar_transform_pitch: float = 0.0
    lidar_transform_yaw: float = 0.0

    # Coordinate-system TF publishers
    publish_unity_tf: bool = False       # map -> unity_map (+90 deg Z)
    publish_slapstack_tf: bool = False   # map -> slapstack_map (-90 deg Z)

    # Translator settings (per-AGV, usually disabled)
    translator_enabled: bool = False
    use_service_for_map_updates: bool = True

    use_sim_time: bool = False

    # Logging
    log_level: str = "info"

    # Pre-initialization from JSON files
    obstacles_json_file: str = ""
    contours_json_file: str = ""
    publish_ready_signal: bool = False

    # Topics to remap to /unused/* when JSON pre-initialization is active
    json_suppress_topics: list[str] = field(default_factory=list)

    # Grid-map node
    gridmap_enabled: bool = False

    # Validation node (detection latency measurement)
    validation_enabled: bool = False
    validation_output_file: str = ""

    # Obstacle report publishing (VDA5050 bridge)
    # true = publish ObstacleReport on every scan (all obstacles)
    # false = only publish when unmatched obstacles are detected
    publish_report_always: bool = True

    # Environment overrides
    env_vars: dict[str, str] = field(default_factory=dict)


@dataclass
class FiwareConfig:
    """FIWARE stack configuration (Orion-LD, TimescaleDB, Grafana, etc.)

    The FIWARE stack is launched via docker compose from the fiware/ directory.
    A single shared stack serves all AGVs. The fiware_namespaces list controls
    which AGV namespaces are bridged to FIWARE (relevant for the FIWARE bridge
    node configuration, not for separate stack instances).
    """
    enabled: bool = False

    # Which AGV namespaces have their data bridged to FIWARE.
    # Empty list with enabled=True means all AGVs.
    fiware_namespaces: list[str] = field(default_factory=list)

    # Base directory containing docker-compose.yaml and config/
    compose_dir: str = "fiware"

    # Ports (single shared stack)
    grafana_port: int = 3000
    timescale_port: int = 5432
    mintaka_port: int = 8080

    # Credentials
    grafana_user: str = "admin"
    grafana_password: str = "admin"
    timescale_user: str = "postgres"
    timescale_password: str = "postgres"


@dataclass
class SimulationConfig:
    """Top-level simulation configuration"""
    # Deploy mode: "central" or "unity"
    mode: str = "central"

    agv_namespaces: list[str] = field(default_factory=lambda: ["agv_0"])
    agv_scan_topic_remaps: dict[str, str] = field(default_factory=dict)
    ros_domain_id: int = 0
    docker: DockerConfig = field(default_factory=DockerConfig)
    bridge: BridgeConfig = field(default_factory=BridgeConfig)
    agv: AGVConfig = field(default_factory=AGVConfig)
    fiware: FiwareConfig = field(default_factory=FiwareConfig)
    startup_delay: float = 2.0
    health_check_interval: float = 5.0
    max_startup_retries: int = 3

    @property
    def agv_count(self) -> int:
        return len(self.agv_namespaces)

    @property
    def deploy_mode(self) -> DeployMode:
        return DeployMode(self.mode)

    @property
    def bridge_image(self) -> str:
        """Resolve bridge container image name"""
        if self.bridge.image:
            return self.bridge.image
        return "rises:unity" if self.deploy_mode == DeployMode.UNITY else "rises:central"

    @property
    def bridge_target(self) -> str:
        """Docker build target for the bridge stage"""
        return "unity" if self.deploy_mode == DeployMode.UNITY else "central"

    @property
    def bridge_container_name(self) -> str:
        """Resolve bridge container name"""
        if self.bridge.container_name:
            return self.bridge.container_name
        return "unity_bridge" if self.deploy_mode == DeployMode.UNITY else "central_bridge"

    # Backwards compatibility: access bridge config as .central
    @property
    def central(self) -> BridgeConfig:
        return self.bridge
