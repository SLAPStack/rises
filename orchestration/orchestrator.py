"""
Main simulation orchestration logic
"""

import signal
import sys
import time
import logging
from pathlib import Path
from datetime import datetime

from .config import SimulationConfig, ContainerStatus, DeployMode
from .container_manager import ContainerManager


class SimulationOrchestrator:
    """Main orchestration logic"""

    def __init__(self, config: SimulationConfig):
        self.config = config
        self.logger = self._setup_logging()
        self.container_manager = ContainerManager(config, self.logger)
        self.shutdown_requested = False

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _setup_logging(self) -> logging.Logger:
        """Configure logging"""
        logger = logging.getLogger("SimulationOrchestrator")
        logger.setLevel(logging.INFO)

        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        ch.setFormatter(formatter)
        logger.addHandler(ch)

        log_dir = Path("logs")
        log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fh = logging.FileHandler(log_dir / f"simulation_{timestamp}.log")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

        return logger

    def _signal_handler(self, signum: int, frame: object) -> None:
        """Handle shutdown signals"""
        self.logger.info(f"Received signal {signum}, initiating shutdown...")
        self.shutdown_requested = True

    def build_images(self, no_cache: bool = False) -> bool:
        """Build required Docker images"""
        cache_info = " (no cache)" if no_cache else ""
        mode = self.config.deploy_mode
        self.logger.info(f"Building Docker images for {mode.value} mode{cache_info}...")

        # Build AGV image: central.dockerfile, target=base -> rises:base
        if not self.container_manager.build_image(
            self.config.agv.dockerfile,
            self.config.agv.image,
            target="base",
            no_cache=no_cache
        ):
            return False

        # Build bridge image: central.dockerfile, target=central|unity -> rises:central|rises:unity
        if not self.container_manager.build_image(
            self.config.bridge.dockerfile,
            self.config.bridge_image,
            target=self.config.bridge_target,
            no_cache=no_cache
        ):
            return False

        return True

    def start_simulation(self) -> bool:
        """Start all containers"""
        mode = self.config.deploy_mode
        self.logger.info("=" * 70)
        self.logger.info(f"Starting simulation orchestration ({mode.value} mode)")
        self.logger.info(f"AGV Namespaces: {', '.join(self.config.agv_namespaces)}")
        self.logger.info(f"AGV Count: {self.config.agv_count}")
        self.logger.info(f"AGV Scenario: {self.config.agv.scenario}")
        self.logger.info(f"ROS Domain ID: {self.config.ros_domain_id}")
        if mode == DeployMode.UNITY:
            self.logger.info(f"TCP Endpoint: {self.config.bridge.tcp_ip}:{self.config.bridge.tcp_port}")
        self.logger.info("=" * 70)

        # Start AGV containers FIRST so geofencing nodes are ready before data arrives
        self.logger.info("Starting AGV containers first (geofencing nodes must be ready before data flow)")
        for namespace in self.config.agv_namespaces:
            if self.shutdown_requested:
                self.logger.warning("Shutdown requested during startup")
                return False

            if not self.container_manager.start_agv_container(namespace):
                self.logger.error(f"Failed to start AGV {namespace}")
                return False

            time.sleep(self.config.startup_delay)

        self.logger.info(f"All {self.config.agv_count} AGV container(s) started, now starting {mode.value} bridge container")

        # Start bridge container AFTER AGV containers are ready
        if not self.container_manager.start_bridge_container():
            self.logger.error(f"Failed to start {mode.value} bridge container")
            return False

        # Start shared FIWARE stack if enabled
        if self.config.fiware.enabled:
            if not self.container_manager.start_fiware_stack():
                self.logger.error("Failed to start FIWARE stack")
                return False

            # Give Orion-LD time to become healthy before bridge connects
            time.sleep(5)

            for namespace in self.config.agv_namespaces:
                if not self.container_manager.start_orion_bridge(namespace):
                    self.logger.error(f"Failed to start Orion bridge for {namespace}")
                    return False

        self.logger.info("All containers started successfully")
        return True

    def monitor_containers(self) -> None:
        """Monitor container health"""
        self.logger.info("Monitoring container health (Ctrl+C to stop)...")

        while not self.shutdown_requested:
            time.sleep(self.config.health_check_interval)

            failed_containers = []
            for name in self.container_manager.containers.keys():
                status = self.container_manager.get_container_status(name)
                if status in [ContainerStatus.EXITED, ContainerStatus.FAILED]:
                    failed_containers.append(name)
                    self.logger.error(f"Container {name} failed with status: {status.value}")

            if failed_containers:
                self.logger.error(f"Detected {len(failed_containers)} failed containers")
                self.logger.info("Initiating cleanup...")
                break

    def _export_kpis(self) -> None:
        """Export KPI CSV from TimescaleDB after bag playback ends."""
        fiware = self.config.fiware
        try:
            # Import here so the orchestrator works without psycopg2 when FIWARE is off.
            script_dir = Path(__file__).resolve().parent.parent / "scripts"
            sys.path.insert(0, str(script_dir))
            from export_kpis import export_kpis  # type: ignore

            self.logger.info("Exporting KPIs from TimescaleDB...")
            csv_path = export_kpis(
                host="localhost",
                port=fiware.timescale_port,
                db="orion",
                user=fiware.timescale_user,
                password=fiware.timescale_password,
            )
            self.logger.info(f"KPI export written to: {csv_path}")
        except Exception as exc:
            self.logger.warning(f"KPI export failed (non-fatal): {exc}")

    def shutdown(self) -> None:
        """Graceful shutdown"""
        self.logger.info("Shutting down simulation...")

        if self.config.fiware.enabled:
            self._export_kpis()

        self.container_manager.cleanup_all()
        self.logger.info("Shutdown complete")

    def run(self) -> int:
        """Main execution flow"""
        try:
            if not self.start_simulation():
                self.logger.error("Failed to start simulation")
                self.shutdown()
                return 1

            self.monitor_containers()
            self.shutdown()
            return 0

        except Exception as e:
            self.logger.exception(f"Unexpected error: {e}")
            self.shutdown()
            return 1
