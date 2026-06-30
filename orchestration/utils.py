"""
Utility functions for orchestration
"""

import yaml

from .config import (
    SimulationConfig,
    DockerConfig,
    BridgeConfig,
    AGVConfig,
    FiwareConfig
)


def parse_namespaces(namespaces_str: str) -> list[str]:
    """Parse comma-separated AGV namespaces string into list"""
    if not namespaces_str:
        return []
    return [ns.strip() for ns in namespaces_str.split(',') if ns.strip()]


def load_config(config_path: str) -> SimulationConfig:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    # Parse nested structures
    docker_cfg = DockerConfig(**data.get('docker', {}))

    # Support both 'bridge' and legacy 'central' key
    bridge_data = data.get('bridge', data.get('central', {}))
    bridge_cfg = BridgeConfig(**bridge_data)

    agv_cfg = AGVConfig(**data.get('agv', {}))
    fiware_cfg = FiwareConfig(**data.get('fiware', {}))

    # Parse AGV namespaces (support both list and comma-separated string)
    namespaces_raw = data.get('agv_namespaces', ["agv_0"])
    if isinstance(namespaces_raw, str):
        agv_namespaces = parse_namespaces(namespaces_raw)
    elif isinstance(namespaces_raw, list):
        agv_namespaces = namespaces_raw
    else:
        agv_namespaces = ["agv_0"]

    return SimulationConfig(
        mode=data.get('mode', 'central'),
        agv_namespaces=agv_namespaces,
        ros_domain_id=data.get('ros_domain_id', 0),
        docker=docker_cfg,
        bridge=bridge_cfg,
        agv=agv_cfg,
        fiware=fiware_cfg,
        agv_scan_topic_remaps=data.get('agv_scan_topic_remaps', {}),
        startup_delay=data.get('startup_delay', 2.0),
        health_check_interval=data.get('health_check_interval', 5.0),
        max_startup_retries=data.get('max_startup_retries', 3)
    )
