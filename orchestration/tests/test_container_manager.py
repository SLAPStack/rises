"""Tests for :class:`orchestration.container_manager.ContainerManager`.

Several tests assert security properties that the production code does
NOT yet honour (audit findings). Those are marked :func:`pytest.xfail`
so the suite turns green once the production code is hardened.
"""

from __future__ import annotations

import logging
import subprocess
from typing import Any
from unittest.mock import patch

import pytest

from orchestration.cli import cleanup_containers
from orchestration.config import (
    AGVConfig,
    BridgeConfig,
    DockerConfig,
    FiwareConfig,
    SimulationConfig,
)
from orchestration.container_manager import ContainerManager


def _build_manager(*, docker_cfg: DockerConfig | None = None) -> ContainerManager:
    """Return a manager with default config except optional Docker overrides."""
    cfg = SimulationConfig(
        agv_namespaces=["agv_0"],
        docker=docker_cfg or DockerConfig(),
        bridge=BridgeConfig(),
        agv=AGVConfig(),
        fiware=FiwareConfig(),
    )
    logger = logging.getLogger("test_container_manager")
    return ContainerManager(cfg, logger)


# ---------------------------------------------------------------------------
# Tests.
# ---------------------------------------------------------------------------
@pytest.mark.xfail(
    reason="Production code does not validate namespace against shell "
           "metachars. Tracked as a security audit finding.",
    strict=True,
)
def test_namespace_validation_rejects_shell_metachars():
    """A namespace containing shell metachars must be rejected before exec."""
    manager = _build_manager()
    with patch("subprocess.run") as fake_run:
        manager.start_agv_container("foo; rm -rf /")
        # Production code should raise or refuse to call subprocess at all.
        assert fake_run.call_count == 0, "namespace should be rejected"


def test_docker_rm_uses_argv_not_shell():
    """cleanup_containers calls docker rm with an argv list, never shell=True."""
    captured: list[dict[str, Any]] = []

    def fake_run(cmd, *_, **kwargs):
        captured.append({"cmd": cmd, "shell": kwargs.get("shell", False)})
        class _R:
            returncode = 0
            stdout = ""
            stderr = ""
        return _R()

    with patch("subprocess.run", side_effect=fake_run), \
         patch("os.path.exists", return_value=False):
        cleanup_containers(namespaces=["agv_0"], mode="central")

    assert captured, "cleanup_containers issued no subprocess calls"
    for entry in captured:
        assert entry["shell"] is False, "must never use shell=True"
        assert isinstance(entry["cmd"], list), "command must be argv list"
        # First two argv tokens are always the binary name and verb.
        if "docker" in entry["cmd"]:
            assert entry["cmd"][0] == "docker"


@pytest.mark.xfail(
    reason="container_manager has no secure_x11 toggle; xhost is always "
           "invoked when RViz is enabled.",
    strict=True,
)
def test_xhost_disabled_in_secure_mode():
    """secure_x11=True suppresses the xhost +local:docker grant."""
    manager = _build_manager()
    manager.config.bridge.launch_rviz = True
    setattr(manager.config.docker, "secure_x11", True)

    seen_cmds: list[list[str]] = []

    def fake_run(cmd, *_, **__):
        seen_cmds.append(cmd if isinstance(cmd, list) else [cmd])
        class _R:
            returncode = 0
            stdout = ""
            stderr = ""
        return _R()

    with patch("subprocess.run", side_effect=fake_run), \
         patch("os.path.exists", return_value=False):
        manager._add_display_forwarding(["docker", "run"])

    for cmd in seen_cmds:
        assert "xhost" not in cmd, "xhost must not be called in secure mode"


@pytest.mark.xfail(
    reason="container_manager._build_docker_run_cmd unconditionally appends "
           "--privileged. Tracked as a security audit finding.",
    strict=True,
)
def test_privileged_flag_only_when_config_says_so():
    """--privileged appears in argv only when DockerConfig.privileged is True."""
    manager = _build_manager(docker_cfg=DockerConfig(privileged=False))
    cmd = manager._build_docker_run_cmd(
        container_name="agv_0",
        env_vars={},
        image="rises:base",
    )
    assert "--privileged" not in cmd


def test_compose_up_invokes_docker_engine():
    """start_fiware_stack uses the docker compose plugin, not docker-compose v1."""
    manager = _build_manager()
    manager.config.fiware.enabled = True
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

    # Modern Docker uses ``docker compose`` (two-token) rather than
    # ``docker-compose`` (single binary). The spec requires the engine
    # choice be honoured. Today only docker is supported.
    cmd = captured["cmd"]
    assert cmd[0] == "docker"
    assert cmd[1] == "compose"
