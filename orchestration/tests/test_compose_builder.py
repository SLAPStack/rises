"""Tests for compose-stack composition in :mod:`orchestration.container_manager`.

The orchestrator does not build a compose dict in memory — it shells out
to ``docker compose -f fiware/docker-compose.yaml ...``. These tests
verify the shell invocation lists the expected services, propagates env
overrides, and rejects malformed config.
"""

from __future__ import annotations

import logging
from typing import Any
from unittest.mock import patch

import pytest

from orchestration.config import (
    AGVConfig,
    BridgeConfig,
    DockerConfig,
    FiwareConfig,
    SimulationConfig,
)
from orchestration.container_manager import ContainerManager


def _build_manager(**overrides: Any) -> ContainerManager:
    """Construct a :class:`ContainerManager` with default config."""
    cfg = SimulationConfig(
        mode=overrides.get("mode", "central"),
        agv_namespaces=["agv_0"],
        ros_domain_id=0,
        docker=DockerConfig(),
        bridge=BridgeConfig(),
        agv=AGVConfig(),
        fiware=overrides.get("fiware", FiwareConfig(enabled=True)),
    )
    logger = logging.getLogger("test_compose_builder")
    return ContainerManager(cfg, logger)


def test_compose_yaml_lists_required_services():
    """start_fiware_stack invokes ``docker compose up`` for the required services."""
    manager = _build_manager()
    captured: dict[str, Any] = {}

    def fake_run(cmd, *_, **__):
        captured["cmd"] = cmd
        class _R:
            returncode = 0
            stdout = ""
            stderr = ""
        return _R()

    with patch("subprocess.run", side_effect=fake_run):
        assert manager.start_fiware_stack() is True

    cmd = captured["cmd"]
    assert cmd[:2] == ["docker", "compose"]
    assert "up" in cmd
    # The orchestrator should always launch these services.
    for service in ("mongodb", "timescaledb", "orion", "mintaka", "grafana"):
        assert service in cmd, f"compose invocation missing service: {service}"


def test_env_overrides_propagate():
    """Custom Grafana / Timescale credentials reach the subprocess environment."""
    fiware = FiwareConfig(
        enabled=True,
        grafana_user="custom_admin",
        grafana_password="s3cret",
        timescale_password="pgpass",
        timescale_port=15432,
    )
    manager = _build_manager(fiware=fiware)
    captured: dict[str, Any] = {}

    def fake_run(cmd, env=None, **__):
        captured["env"] = env or {}
        class _R:
            returncode = 0
            stdout = ""
            stderr = ""
        return _R()

    with patch("subprocess.run", side_effect=fake_run):
        manager.start_fiware_stack()

    env = captured["env"]
    assert env["GRAFANA_USER"] == "custom_admin"
    assert env["GRAFANA_PASSWORD"] == "s3cret"
    assert env["TIMESCALE_PASSWORD"] == "pgpass"
    assert env["TIMESCALE_PORT"] == "15432"


def test_invalid_module_rejected():
    """An invalid deploy ``mode`` raises :class:`ValueError` when resolved."""
    cfg = SimulationConfig(mode="not_a_real_mode")
    with pytest.raises(ValueError):
        _ = cfg.deploy_mode


def test_module_dependency_resolution():
    """Enabling FIWARE keeps Orion-LD and its TimescaleDB peer in one invocation."""
    manager = _build_manager()
    captured: dict[str, Any] = {}

    def fake_run(cmd, *_, **__):
        captured["cmd"] = cmd
        class _R:
            returncode = 0
            stdout = ""
            stderr = ""
        return _R()

    with patch("subprocess.run", side_effect=fake_run):
        manager.start_fiware_stack()

    cmd = captured["cmd"]
    # Dependency rule: Orion-LD relies on Mongo + Timescale.
    assert "orion" in cmd
    assert "mongodb" in cmd
    assert "timescaledb" in cmd
