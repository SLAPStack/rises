#!/usr/bin/env python3
"""
Command-line interface for simulation orchestration
"""

import argparse
import os
import subprocess
import logging
import sys

from .orchestrator import SimulationOrchestrator
from .utils import load_config, parse_namespaces


def cleanup_containers(namespaces: list[str] | None = None, mode: str = "central") -> None:
    """Cleanup existing containers (force kill and remove)"""
    logger = logging.getLogger("Cleanup")
    logger.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter('%(levelname)s - %(message)s'))
    logger.addHandler(ch)

    logger.info("Cleaning up containers (force removal)...")

    # Remove bridge container (name depends on mode)
    bridge_name = "unity_bridge" if mode == "unity" else "central_bridge"
    for name in [bridge_name, "central_bridge", "unity_bridge"]:
        result = subprocess.run(["docker", "rm", "-f", name],
                      capture_output=True, check=False)
        if result.returncode == 0:
            logger.info(f"Removed {name}")

    # Remove AGV containers
    agv_names = namespaces if namespaces else [f"agv_{i}" for i in range(20)]
    for ns in agv_names:
        result = subprocess.run(["docker", "rm", "-f", ns],
                      capture_output=True, check=False)
        if result.returncode == 0:
            logger.info(f"Removed {ns}")

    # Also stop the shared FIWARE stack
    compose_file = os.path.join("fiware", "docker-compose.yaml")
    if os.path.exists(compose_file):
        result = subprocess.run(
            ["docker", "compose", "-f", compose_file, "-p", "rises-fiware", "down"],
            capture_output=True, check=False,
        )
        if result.returncode == 0:
            logger.info("Stopped FIWARE stack")

    logger.info("Cleanup complete")


def main() -> int:
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="Orchestrate multi-container ROS2 simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Central mode (rosbag replay)
  python3 -m orchestration.cli --config configs/geofensing_map_updates_rises.yaml

  # Unity mode (TCP bridge)
  python3 -m orchestration.cli --config configs/unity_sim.yaml --mode unity

  # Override AGV namespaces
  python3 -m orchestration.cli --config configs/unity_sim.yaml --agvs "agv_0,agv_1,agv_2"

  # Build images first
  python3 -m orchestration.cli --config configs/unity_sim.yaml --build

  # Cleanup all
  python3 -m orchestration.cli --cleanup
        """
    )

    parser.add_argument(
        '--config', '-c',
        type=str,
        help='Path to configuration YAML file'
    )

    parser.add_argument(
        '--mode',
        type=str, choices=['central', 'unity'],
        help='Deploy mode: central (rosbag + translator) or unity (TCP endpoint + translator). '
             'Overrides the mode set in config YAML.'
    )

    parser.add_argument(
        '--scenario',
        type=str, choices=['default', 'rosbag', 'unity'],
        help='AGV scenario: selects params_*.yaml in the container entrypoint. '
             'Overrides agv.scenario in config YAML.'
    )

    parser.add_argument(
        '--agvs',
        type=str,
        help='Comma-separated AGV namespaces (e.g., "agv_0,agv_1,agv_2")'
    )

    parser.add_argument(
        '--build',
        action='store_true',
        help='Build Docker images before starting'
    )

    parser.add_argument(
        '--no-cache',
        action='store_true',
        help='Build Docker images without using cache'
    )

    parser.add_argument(
        '--cleanup',
        action='store_true',
        help='Cleanup existing containers and exit'
    )

    parser.add_argument(
        '--ros-domain-id',
        type=int,
        help='Override ROS_DOMAIN_ID'
    )

    parser.add_argument(
        '--bag-file',
        type=str,
        help='Override rosbag file path'
    )

    parser.add_argument(
        '--rosbag-remaps',
        type=str,
        help='Topic remappings for rosbag (/from:=/to,…)'
    )

    parser.add_argument(
        '--launch-rviz',
        action='store_true',
        help='Launch RViz in the bridge container'
    )

    parser.add_argument(
        '--rviz-namespace',
        type=str,
        help='AGV namespace for RViz topics (e.g., "agv_0")'
    )

    # Unity TCP endpoint overrides
    parser.add_argument(
        '--tcp-ip',
        type=str,
        help='TCP endpoint IP for Unity mode (default: 0.0.0.0)'
    )

    parser.add_argument(
        '--tcp-port',
        type=int,
        help='TCP endpoint port for Unity mode (default: 10000)'
    )

    # Pre-initialization from JSON files
    parser.add_argument(
        '--obstacles-json',
        type=str,
        help='Path to obstacles JSON file for AGV pre-initialization'
    )

    parser.add_argument(
        '--contours-json',
        type=str,
        help='Path to contours JSON file for AGV pre-initialization'
    )

    parser.add_argument(
        '--fiware',
        type=str,
        nargs='?',
        const='__all__',
        default=None,
        help='Enable FIWARE stack. Optionally specify comma-separated AGV namespaces '
             '(e.g., "--fiware agv_0,agv_1"). Without a value, enables for all AGVs.'
    )

    parser.add_argument(
        '--no-fiware',
        action='store_true',
        default=False,
        help='Disable FIWARE stack even if enabled in config.'
    )

    parser.add_argument(
        '--wait-for-geofence-ready',
        action='store_true',
        help='Wait for all geofence nodes to signal ready before passthrough'
    )

    parser.add_argument(
        '--geofence-ready-timeout',
        type=float,
        help='Timeout in seconds for waiting for geofence ready signals'
    )

    args = parser.parse_args()

    # Cleanup mode
    if args.cleanup:
        namespaces = None
        if args.agvs:
            namespaces = parse_namespaces(args.agvs)
        cleanup_containers(namespaces, mode=args.mode or "central")
        return 0

    # Require config for normal operation
    if not args.config:
        parser.error("--config is required (or use --cleanup)")

    # Load configuration
    config = load_config(args.config)

    # Apply CLI overrides
    if args.mode:
        config.mode = args.mode

    if args.scenario:
        config.agv.scenario = args.scenario

    if args.agvs:
        config.agv_namespaces = parse_namespaces(args.agvs)

    if args.ros_domain_id is not None:
        config.ros_domain_id = args.ros_domain_id

    if args.bag_file:
        config.bridge.bag_file = args.bag_file

    if args.rosbag_remaps:
        config.bridge.rosbag_remaps = args.rosbag_remaps

    if args.launch_rviz:
        config.bridge.launch_rviz = True

    if args.rviz_namespace:
        config.bridge.rviz_namespace = args.rviz_namespace
        config.bridge.launch_rviz = True

    if args.tcp_ip:
        config.bridge.tcp_ip = args.tcp_ip

    if args.tcp_port is not None:
        config.bridge.tcp_port = args.tcp_port

    # Pre-initialization from JSON files
    if args.obstacles_json:
        config.agv.obstacles_json_file = args.obstacles_json
        config.agv.publish_ready_signal = True

    if args.contours_json:
        config.agv.contours_json_file = args.contours_json
        config.agv.publish_ready_signal = True

    if args.fiware is not None:
        config.fiware.enabled = True
        if args.fiware == '__all__':
            config.fiware.fiware_namespaces = list(config.agv_namespaces)
        else:
            config.fiware.fiware_namespaces = parse_namespaces(args.fiware)

    if args.no_fiware:
        config.fiware.enabled = False

    if args.wait_for_geofence_ready:
        config.bridge.wait_for_geofence_ready = True

    if args.geofence_ready_timeout is not None:
        config.bridge.geofence_ready_timeout = args.geofence_ready_timeout

    # Validate configuration
    if not config.agv_namespaces:
        logging.error("No AGV namespaces specified. Use --agvs or set in config.yaml")
        return 1

    # Warn if no bag_file but don't fail
    if not config.bridge.bag_file and config.deploy_mode.value == "central":
        logging.warning("No rosbag file specified. Rosbag playback will be disabled.")
        config.bridge.play_rosbag = False

    # Create orchestrator
    orchestrator = SimulationOrchestrator(config)

    # Build images if requested
    if args.build:
        if not orchestrator.build_images(no_cache=args.no_cache):
            orchestrator.logger.error("Image build failed")
            return 1

    # Run simulation
    return orchestrator.run()


if __name__ == "__main__":
    sys.exit(main())
