"""
Docker container lifecycle management
"""

import subprocess
import time
import os
import json
import logging

from datetime import datetime

from .config import SimulationConfig, ContainerStatus, DeployMode


class ContainerManager:
    """Manages Docker container lifecycle"""

    def __init__(self, config: SimulationConfig, logger: logging.Logger):
        self.config = config
        self.logger = logger
        self.containers: dict[str, str] = {}  # name -> container_id
        self.startup_times: dict[str, datetime] = {}

    def build_image(self, dockerfile: str, tag: str, target: str | None = None, no_cache: bool = False) -> bool:
        """Build Docker image from Dockerfile with optional target stage and real-time progress"""
        target_info = f" (target: {target})" if target else ""
        cache_info = " [NO CACHE]" if no_cache else ""
        self.logger.info(f"Building image '{tag}' from {dockerfile}{target_info}{cache_info}...")

        cmd = [
            "docker", "build",
            "-f", dockerfile,
            "-t", tag,
        ]

        if target:
            cmd.extend(["--target", target])

        if no_cache:
            cmd.append("--no-cache")

        cmd.extend(["--progress=plain"])
        cmd.append(".")

        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            for line in process.stdout:
                print(line, end='', flush=True)

            return_code = process.wait()

            if return_code == 0:
                self.logger.info(f"Successfully built image: {tag}")
                return True
            else:
                self.logger.error(f"Failed to build image {tag} (exit code: {return_code})")
                return False

        except Exception as e:
            self.logger.error(f"Failed to build image {tag}: {str(e)}")
            return False

    def _build_bridge_env_vars(self) -> dict[str, str]:
        """Build environment variables common to both central and unity bridge containers"""
        cfg = self.config.bridge
        env_vars = {
            "ROS_DOMAIN_ID": str(self.config.ros_domain_id),
            "ROS_LOCALHOST_ONLY": "0",
            "AGV_COUNT": str(self.config.agv_count),
            "TRANSLATOR_ENABLE_BUFFERING": str(cfg.enable_buffering).lower(),
            "TRANSLATOR_BUFFER_TIMEOUT": str(cfg.buffer_timeout),
            "TRANSLATOR_REPLAY_RATE": str(cfg.replay_rate),
            "TRANSLATOR_TARGET_FRAME": cfg.target_frame,
            "TRANSLATOR_BASE_LINK_PUB_RATE": str(cfg.base_link_pub_rate),
            "USE_SIM_TIME": str(cfg.use_sim_time).lower(),
            "LAUNCH_RVIZ": str(cfg.launch_rviz).lower(),
            "RVIZ_NAMESPACE": cfg.rviz_namespace,
            **({"RVIZ_CONFIG_FILE": cfg.rviz_config_file} if cfg.rviz_config_file else {}),
            "SUPPRESS_MAP_TOPICS": str(cfg.suppress_map_topics).lower(),
            # MQTT topic names (empty = use launch file defaults)
            **({"MQTT_OBSTACLE_TOPIC": cfg.mqtt_obstacle_topic} if cfg.mqtt_obstacle_topic else {}),
            **({"MQTT_CONTOURS_TOPIC": cfg.mqtt_contours_topic} if cfg.mqtt_contours_topic else {}),
            **({"MQTT_ORDER_TOPIC": cfg.mqtt_order_topic} if cfg.mqtt_order_topic else {}),
            **({"MQTT_TF_TOPIC": cfg.mqtt_tf_topic} if cfg.mqtt_tf_topic else {}),
            **({"MQTT_SCAN_TOPIC": cfg.mqtt_scan_topic} if cfg.mqtt_scan_topic else {}),
            **({"MQTT_VALIDATION_TOPIC": cfg.mqtt_validation_topic} if cfg.mqtt_validation_topic else {}),
            # Rosbag settings
            "ROSBAG_FILE": cfg.bag_file,
            "ROSBAG_RATE": str(cfg.rosbag_rate),
            "ROSBAG_LOOP": str(cfg.rosbag_loop).lower(),
            "ROSBAG_DELAY": str(cfg.rosbag_delay),
            "STORAGE": cfg.storage,
            # Geofence ready waiting
            "TRANSLATOR_WAIT_FOR_GEOFENCE_READY": str(cfg.wait_for_geofence_ready).lower(),
            "TRANSLATOR_GEOFENCE_READY_TIMEOUT": str(cfg.geofence_ready_timeout),
            "TRANSLATOR_USE_SERVICE_FOR_MAP_UPDATES": str(cfg.use_service_for_map_updates).lower(),
            "TRANSLATOR_NORMALIZE_OBSTACLE_Y": str(cfg.normalize_obstacle_y).lower(),
        }

        # Build rosbag remaps with conditional map topic suppression
        rosbag_remaps = cfg.rosbag_remaps if cfg.rosbag_remaps else ""

        # If AGVs use JSON pre-initialization, remap specified bag topics to /unused/*
        # so they are never delivered to the translator or geofence.
        # Do NOT set SUPPRESS_MAP_TOPICS here — that would move the translator's
        # subscription to /unused/... too, making it reconnect to the same dead topic.
        # The translator keeps its real subscription; with no publisher on the real
        # topic it receives nothing, which is exactly what we want.
        if (self.config.agv.obstacles_json_file or self.config.agv.contours_json_file) and self.config.agv.json_suppress_topics:
            self.logger.info(f"JSON pre-initialization detected - suppressing {len(self.config.agv.json_suppress_topics)} bag topic(s)")
            suppress_remaps = []
            for topic in self.config.agv.json_suppress_topics:
                unused_topic = f"/unused{topic}" if not topic.startswith("/unused") else topic
                suppress_remaps.append(f"{topic}:={unused_topic}")
            map_topic_remaps = ",".join(suppress_remaps)
            rosbag_remaps = f"{rosbag_remaps},{map_topic_remaps}" if rosbag_remaps else map_topic_remaps

        if rosbag_remaps:
            env_vars["ROSBAG_REMAPS"] = rosbag_remaps

        # Apply custom env overrides
        env_vars.update(cfg.env_vars)
        return env_vars

    def _build_docker_run_cmd(self, container_name: str, env_vars: dict[str, str], image: str) -> list[str]:
        """Build the docker run command with common flags"""
        docker_cfg = self.config.docker
        cmd = ["docker", "run", "-d"]

        cmd.extend(["--name", container_name])
        cmd.extend(["--network", docker_cfg.network_mode])
        cmd.extend(["--ipc", "host"])
        cmd.append("--privileged")

        if docker_cfg.restart_policy:
            cmd.extend(["--restart", docker_cfg.restart_policy])

        if docker_cfg.remove_on_exit:
            cmd.append("--rm")

        cmd.extend(["--log-driver", docker_cfg.log_driver])
        cmd.extend(["--log-opt", f"max-size={docker_cfg.log_max_size}"])
        cmd.extend(["--log-opt", f"max-file={docker_cfg.log_max_file}"])

        for key, value in env_vars.items():
            cmd.extend(["-e", f"{key}={value}"])

        return cmd

    def _add_display_forwarding(self, cmd: list[str]) -> None:
        """Add X11/Wayland display forwarding for RViz"""
        try:
            subprocess.run(["xhost", "+local:docker"], check=False, capture_output=True)
        except FileNotFoundError:
            self.logger.warning("xhost command not found - RViz may not display")

        display = os.environ.get("DISPLAY", ":0")
        cmd.extend(["-e", f"DISPLAY={display}"])
        cmd.extend(["-e", "QT_X11_NO_MITSHM=1"])

        x11_socket = "/tmp/.X11-unix"
        if os.path.exists(x11_socket):
            cmd.extend(["-v", f"{x11_socket}:{x11_socket}:rw"])

        xauthority = os.environ.get("XAUTHORITY", os.path.expanduser("~/.Xauthority"))
        if os.path.exists(xauthority):
            cmd.extend(["-v", f"{xauthority}:/root/.Xauthority:ro,z"])
            cmd.extend(["-e", "XAUTHORITY=/root/.Xauthority"])

        wayland_display = os.environ.get("WAYLAND_DISPLAY")
        xdg_runtime_dir = os.environ.get("XDG_RUNTIME_DIR")
        if wayland_display and xdg_runtime_dir:
            cmd.extend(["-e", f"WAYLAND_DISPLAY={wayland_display}"])
            cmd.extend(["-e", f"XDG_RUNTIME_DIR={xdg_runtime_dir}"])
            wayland_socket = os.path.join(xdg_runtime_dir, wayland_display)
            if os.path.exists(wayland_socket):
                cmd.extend(["-v", f"{wayland_socket}:{wayland_socket}:rw"])
            if os.path.exists(xdg_runtime_dir):
                cmd.extend(["-v", f"{xdg_runtime_dir}:{xdg_runtime_dir}:rw"])

        if os.path.exists("/dev/dri"):
            cmd.extend(["--device", "/dev/dri"])

        self.logger.info(f"RViz enabled with DISPLAY={display}, Wayland={wayland_display or 'N/A'}")

    def _run_container(self, container_name: str, cmd: list[str]) -> bool:
        """Execute docker run and track the container"""
        # Remove any stopped container with the same name so docker run doesn't fail.
        subprocess.run(
            ["docker", "rm", "-f", container_name],
            capture_output=True,
        )
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            container_id = result.stdout.strip()
            self.containers[container_name] = container_id
            self.startup_times[container_name] = datetime.now()
            self.logger.info(f"Started container {container_name}: {container_id[:12]}")
            return True
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Failed to start container {container_name}: {e.stderr}")
            return False

    def start_bridge_container(self) -> bool:
        """Start the bridge container (central or unity, based on config.mode)"""
        cfg = self.config.bridge
        container_name = self.config.bridge_container_name
        image = self.config.bridge_image
        mode = self.config.deploy_mode

        self.logger.info(f"Starting {mode.value} bridge container: {container_name}")

        env_vars = self._build_bridge_env_vars()

        # Unity-specific: TCP endpoint settings
        if mode == DeployMode.UNITY:
            env_vars["ROS_TCP_IP"] = cfg.tcp_ip
            env_vars["ROS_TCP_PORT"] = str(cfg.tcp_port)

        cmd = self._build_docker_run_cmd(container_name, env_vars, image)

        # Volume mount for rosbag file
        if cfg.bag_file and os.path.exists(cfg.bag_file):
            bag_path = os.path.abspath(cfg.bag_file)
            cmd.extend(["-v", f"{bag_path}:{bag_path}:ro,z"])

        # Volume mount for custom RViz config (overrides baked-in image copy)
        if cfg.rviz_config_file and os.path.exists(cfg.rviz_config_file):
            rviz_path = os.path.abspath(cfg.rviz_config_file)
            cmd.extend(["-v", f"{rviz_path}:{rviz_path}:ro,z"])

        # Display forwarding for RViz
        if cfg.launch_rviz:
            self._add_display_forwarding(cmd)

        cmd.append(image)
        return self._run_container(container_name, cmd)

    # Backwards compatibility
    def start_central_container(self) -> bool:
        return self.start_bridge_container()

    def start_agv_container(self, namespace: str) -> bool:
        """Start an AGV container with explicit namespace (e.g., 'agv_0', 'agv_1')"""
        cfg = self.config.agv
        docker_cfg = self.config.docker

        container_name = namespace
        tf_prefix = namespace
        robot_id = namespace

        self.logger.info(f"Starting AGV container: {container_name} (namespace: {namespace})")

        env_vars = {
            "ROS_DOMAIN_ID": str(self.config.ros_domain_id),
            "ROS_LOCALHOST_ONLY": "0",
            "NAMESPACE": namespace,
            "TF_PREFIX": tf_prefix,
            "ROBOT_ID": robot_id,
            "USE_SIM_TIME": str(cfg.use_sim_time).lower(),
            # Scenario selection (entrypoint.sh uses this to pick params_*.yaml)
            "SCENARIO": cfg.scenario,
            # AGV deploy mode
            "AGV_DEPLOY_MODE": cfg.agv_deploy_mode,
            # Geofence / gridmap node params
            "GEOFENCE_ENABLE_SAFETY_CIRCLE": str(cfg.enable_safety_circle).lower(),
            "GEOFENCE_SAFETY_RADIUS": str(cfg.safety_circle_radius),
            "GEOFENCE_VISUALIZER_ENABLED": str(cfg.visualizer_enabled).lower(),
            "GEOFENCE_USE_NAMESPACE_FOR_MAP": str(cfg.use_namespace_for_map_frame).lower(),
            "GEOFENCE_TARGET_FRAME": cfg.target_frame,
            "GEOFENCE_BASE_LINK_FRAME": cfg.base_link_frame,
            "GEOFENCE_ENABLE_ROBOT_FILTERING": str(cfg.enable_robot_filtering).lower(),
            "GEOFENCE_ROBOT_FOOTPRINT_TYPE": cfg.robot_footprint_type,
            "GEOFENCE_ROBOT_FOOTPRINT_RADIUS": str(cfg.robot_footprint_radius),
            "GEOFENCE_ROBOT_FOOTPRINT_MARGIN": str(cfg.robot_footprint_margin),
            "GEOFENCE_USE_SERVICE_FOR_MAP_UPDATES": str(cfg.use_service_for_map_updates).lower(),
            # Pre-initialization from JSON files
            "GEOFENCE_OBSTACLES_JSON_FILE": cfg.obstacles_json_file,
            "GEOFENCE_CONTOURS_JSON_FILE": cfg.contours_json_file,
            "GEOFENCE_PUBLISH_READY_SIGNAL": str(cfg.publish_ready_signal).lower(),
            # Laserscan preprocessor node params
            "LASERSCAN_GLOBAL_FRAME": cfg.target_frame,
            "LASERSCAN_DISTANCE_THRESHOLD": str(cfg.segment_distance_threshold),
            "LASERSCAN_ANGLE_THRESHOLD_DEG": str(cfg.segment_angle_threshold_deg),
            # Translator (per-AGV, usually disabled)
            "TRANSLATOR_ENABLED": str(cfg.translator_enabled).lower(),
            # Grid-map node
            "GRIDMAP_ENABLED": str(cfg.gridmap_enabled).lower(),
            # Validation node
            "VALIDATION_ENABLED": str(cfg.validation_enabled).lower(),
            # Leg filter (optional, filters unmatched obstacles for likely human legs)
            "LAUNCH_LEG_FILTER": str(cfg.launch_leg_filter).lower(),
            "GEOFENCE_VALIDATION_OUTPUT_FILE": cfg.validation_output_file,
            # Obstacle report publishing
            "GEOFENCE_PUBLISH_REPORT_ALWAYS": str(cfg.publish_report_always).lower(),
            # Static TF publisher (generic)
            "PUBLISH_STATIC_TF": str(cfg.publish_static_tf).lower(),
            "STATIC_TF_PARENT_FRAME": cfg.static_tf_parent_frame,
            "STATIC_TF_CHILD_FRAME": cfg.static_tf_child_frame,
            "STATIC_TF_X": str(cfg.static_tf_x),
            "STATIC_TF_Y": str(cfg.static_tf_y),
            "STATIC_TF_Z": str(cfg.static_tf_z),
            "STATIC_TF_ROLL": str(cfg.static_tf_roll),
            "STATIC_TF_PITCH": str(cfg.static_tf_pitch),
            "STATIC_TF_YAW": str(cfg.static_tf_yaw),
            "STATIC_TF_USE_NAMESPACE": str(cfg.static_tf_use_namespace).lower(),
            # Lidar static TF publisher (base_link -> laser_link)
            "PUBLISH_LIDAR_STATIC_TF": str(cfg.publish_lidar_static_tf).lower(),
            "LIDAR_TRANSFORM_X": str(cfg.lidar_transform_x),
            "LIDAR_TRANSFORM_Y": str(cfg.lidar_transform_y),
            "LIDAR_TRANSFORM_Z": str(cfg.lidar_transform_z),
            "LIDAR_TRANSFORM_ROLL": str(cfg.lidar_transform_roll),
            "LIDAR_TRANSFORM_PITCH": str(cfg.lidar_transform_pitch),
            "LIDAR_TRANSFORM_YAW": str(cfg.lidar_transform_yaw),
            # Coordinate-system TF publishers
            "PUBLISH_UNITY_TF": str(cfg.publish_unity_tf).lower(),
            "PUBLISH_SLAPSTACK_TF": str(cfg.publish_slapstack_tf).lower(),
            "TARGET_FRAME": cfg.target_frame,
        }

        # ARISE mode: launch rises_geofence.launch.py with full ARISE stack
        if cfg.rises_mode:
            env_vars["ARISE_MODE"] = "true"
            env_vars["LAUNCH_SKILL_BRIDGE"] = str(cfg.launch_skill_bridge).lower()
            env_vars["LAUNCH_MISSION_CONTROLLER"] = str(cfg.launch_mission_controller).lower()
            env_vars["LAUNCH_HEATMAP"] = str(cfg.launch_heatmap).lower()
            env_vars["LAUNCH_FIWARE_BRIDGE"] = str(cfg.launch_fiware_bridge).lower()
            self.logger.info(
                f"AGV {namespace} ARISE mode: skill_bridge={cfg.launch_skill_bridge}, "
                f"heatmap={cfg.launch_heatmap}, fiware_bridge={cfg.launch_fiware_bridge}")

        # Set log level
        log_level = cfg.log_level.upper()
        if log_level in ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"]:
            env_vars["RCUTILS_LOGGING_SEVERITY_THRESHOLD"] = log_level
            env_vars["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{severity}] [{name}]: {message}"
            env_vars["RCUTILS_COLORIZED_OUTPUT"] = "1"
            env_vars["RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED"] = "1"
            self.logger.info(f"AGV {namespace} log level: {log_level}")
        else:
            self.logger.warning(f"Invalid log level '{cfg.log_level}', using default INFO")

        # Optional laser frame ID
        if cfg.laser_frame_ids:
            env_vars["LASERSCAN_FRAME_ID"] = cfg.laser_frame_ids[0]

        # AGV-specific scan topic remap
        if hasattr(self.config, 'agv_scan_topic_remaps') and namespace in self.config.agv_scan_topic_remaps:
            env_vars["SCAN_TOPIC_REMAP"] = self.config.agv_scan_topic_remaps[namespace]
            self.logger.info(f"AGV {namespace} scan topic remapped to: {self.config.agv_scan_topic_remaps[namespace]}")

        # Apply custom env overrides
        env_vars.update(cfg.env_vars)

        cmd = self._build_docker_run_cmd(container_name, env_vars, cfg.image)

        # Volume mounts for JSON files
        if cfg.obstacles_json_file and os.path.exists(cfg.obstacles_json_file):
            json_path = os.path.abspath(cfg.obstacles_json_file)
            cmd.extend(["-v", f"{json_path}:{json_path}:ro,z"])

        if cfg.contours_json_file and os.path.exists(cfg.contours_json_file):
            json_path = os.path.abspath(cfg.contours_json_file)
            cmd.extend(["-v", f"{json_path}:{json_path}:ro,z"])

        # Runtime-mount the Fast DDS XML profile (not baked into the image) so it can
        # be edited without rebuilding. Enables XTypes TypeObject propagation for the
        # FIWARE DDS-Enabler. If the file is absent, the container falls back to Fast
        # DDS defaults (no profile), which is fine for ROS-to-ROS pub/sub and services.
        fastdds_profile = os.path.abspath("fiware/config/fastdds_profiles.xml")
        if os.path.exists(fastdds_profile):
            cmd.extend(["-v", f"{fastdds_profile}:/config/fastdds_profiles.xml:ro,z"])
            cmd.extend(["-e", "FASTDDS_DEFAULT_PROFILES_FILE=/config/fastdds_profiles.xml"])

        cmd.append(cfg.image)
        return self._run_container(container_name, cmd)

    def get_container_status(self, container_name: str) -> ContainerStatus:
        """Get current status of a container"""
        try:
            result = subprocess.run(
                ["docker", "inspect", "-f", "{{.State.Status}}", container_name],
                check=True,
                capture_output=True,
                text=True
            )
            status_str = result.stdout.strip()

            if status_str == "running":
                return ContainerStatus.RUNNING
            elif status_str == "created":
                return ContainerStatus.CREATED
            elif status_str == "exited":
                return ContainerStatus.EXITED
            else:
                return ContainerStatus.UNKNOWN
        except subprocess.CalledProcessError:
            return ContainerStatus.FAILED

    def stop_container(self, container_name: str, timeout: int = 10) -> bool:
        """Stop a container gracefully"""
        self.logger.info(f"Stopping container: {container_name}")
        try:
            subprocess.run(
                ["docker", "stop", "-t", str(timeout), container_name],
                check=True,
                capture_output=True
            )
            return True
        except subprocess.CalledProcessError as e:
            if b"No such container" in e.stderr or b"is not running" in e.stderr:
                self.logger.debug(f"Container {container_name} already stopped or doesn't exist")
                return True
            self.logger.error(f"Failed to stop {container_name}: {e.stderr}")
            return False

    def remove_container(self, container_name: str, force: bool = False) -> bool:
        """Remove a container"""
        self.logger.info(f"Removing container: {container_name}")
        cmd = ["docker", "rm"]
        if force:
            cmd.append("-f")
        cmd.append(container_name)

        try:
            subprocess.run(cmd, check=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            if b"No such container" not in e.stderr:
                self.logger.error(f"Failed to remove {container_name}: {e.stderr}")
                return False
            self.logger.debug(f"Container {container_name} already removed or doesn't exist")

        if container_name in self.containers:
            del self.containers[container_name]
        return True

    def get_logs(self, container_name: str, lines: int = 50) -> str:
        """Get container logs"""
        try:
            result = subprocess.run(
                ["docker", "logs", "--tail", str(lines), container_name],
                check=True,
                capture_output=True,
                text=True
            )
            return result.stdout
        except subprocess.CalledProcessError as e:
            return f"Error getting logs: {e.stderr}"

    def start_fiware_stack(self) -> bool:
        """Start the shared FIWARE stack (Orion-LD, TimescaleDB, Grafana, etc.).

        Uses docker compose from the fiware/ directory. A single stack
        serves all AGV namespaces.
        """
        fiware_cfg = self.config.fiware
        compose_dir = os.path.abspath(fiware_cfg.compose_dir)
        project_name = "rises-fiware"

        namespaces_str = ", ".join(fiware_cfg.fiware_namespaces) or "all"
        self.logger.info(
            f"Starting FIWARE stack (bridged namespaces: {namespaces_str}): "
            f"Grafana=:{fiware_cfg.grafana_port}, "
            f"TimescaleDB=:{fiware_cfg.timescale_port}, "
            f"Mintaka=:{fiware_cfg.mintaka_port}"
        )

        env = os.environ.copy()
        env.update({
            "GRAFANA_PORT": str(fiware_cfg.grafana_port),
            "GRAFANA_USER": fiware_cfg.grafana_user,
            "GRAFANA_PASSWORD": fiware_cfg.grafana_password,
            "TIMESCALE_PORT": str(fiware_cfg.timescale_port),
            "TIMESCALE_USER": fiware_cfg.timescale_user,
            "TIMESCALE_PASSWORD": fiware_cfg.timescale_password,
            "MINTAKA_PORT": str(fiware_cfg.mintaka_port),
            "COMPOSE_PROJECT_NAME": project_name,
        })

        cmd = [
            "docker", "compose",
            "-f", os.path.join(compose_dir, "docker-compose.yaml"),
            "-p", project_name,
            "up", "-d", "--no-build",
            "mongodb", "timescaledb", "orion", "mintaka", "grafana",
        ]

        try:
            subprocess.run(cmd, env=env, capture_output=True, text=True, check=True)
            # Don't add to self.containers — FIWARE is a compose project, not a
            # single container. The health monitor uses docker inspect on container
            # names, which would fail for a project name.
            self.fiware_running = True
            self.logger.info(
                f"FIWARE stack started. "
                f"Grafana: http://localhost:{fiware_cfg.grafana_port}"
            )
            return True
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Failed to start FIWARE stack: {e.stderr}")
            return False

    def start_orion_bridge(self, namespace: str = "agv_0") -> bool:
        """Start orion_bridge.py inside the AGV container via docker exec.

        The bridge is a ROS 2 Python node that subscribes to geofence topics
        and pushes data to Orion-LD via HTTP. It runs inside the AGV container
        where the ROS workspace is already sourced.
        """
        container_name = namespace

        cmd = [
            "docker", "exec", "-d",
            "-e", "ORION_HOST=localhost",
            "-e", "ORION_PORT=1026",
            "-e", f"AGV_ID={namespace}",
            "-e", "BRIDGE_PORT=9091",
            container_name,
            "bash", "-c",
            "source /opt/ros/jazzy/setup.bash && "
            "source /workspace/install/setup.bash && "
            f"cd /workspace && python3 src/fiware_bridge/orion_bridge.py "
            f"--ros-args -r __ns:=/{namespace} -p use_sim_time:=true"
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.logger.info(
                f"Orion bridge started in container {container_name} "
                f"(AGV_ID={namespace}, BRIDGE_PORT=9091)"
            )
            return True
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Failed to start orion bridge in {container_name}: {e.stderr}")
            return False

    def stop_orion_bridge(self) -> None:
        """Orion bridge runs inside the AGV container — it stops when the container stops."""
        pass

    def stop_fiware_stack(self) -> bool:
        """Stop and remove the FIWARE stack."""
        fiware_cfg = self.config.fiware
        compose_dir = os.path.abspath(fiware_cfg.compose_dir)
        project_name = "rises-fiware"

        self.logger.info("Stopping FIWARE stack")
        cmd = [
            "docker", "compose",
            "-f", os.path.join(compose_dir, "docker-compose.yaml"),
            "-p", project_name,
            "down",
        ]

        try:
            subprocess.run(cmd, capture_output=True, text=True, check=True)
            self.fiware_running = False
            return True
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Failed to stop FIWARE stack: {e.stderr}")
            return False

    def cleanup_all(self, force: bool = True) -> None:
        """Stop and remove all managed containers"""
        self.logger.info("Cleaning up all containers...")

        # Stop orion bridge process
        self.stop_orion_bridge()

        # Stop FIWARE stack
        if getattr(self, 'fiware_running', False):
            self.stop_fiware_stack()

        for name in list(self.containers.keys()):
            self.remove_container(name, force=True)

        self.containers.clear()
        self.startup_times.clear()
