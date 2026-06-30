"""
Simulation Orchestration Package

Multi-container ROS2 simulation orchestration with:
- Bridge container: central (translator + rosbag) or unity (TCP endpoint + translator)
- Per-AGV containers (geofence + laserscan preprocessor + optional translator)
"""

from .config import (
    ContainerStatus,
    DeployMode,
    DockerConfig,
    BridgeConfig,
    CentralConfig,
    AGVConfig,
    SimulationConfig
)
from .container_manager import ContainerManager
from .orchestrator import SimulationOrchestrator
from .utils import load_config

__version__ = "2.0.0"
__all__ = [
    "ContainerStatus",
    "DeployMode",
    "DockerConfig",
    "BridgeConfig",
    "CentralConfig",
    "AGVConfig",
    "SimulationConfig",
    "ContainerManager",
    "SimulationOrchestrator",
    "load_config"
]
