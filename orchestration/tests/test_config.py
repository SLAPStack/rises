"""Tests for :mod:`orchestration.config` and :mod:`orchestration.utils`.

The production code does not yet warn on default credentials or
validate required YAML keys (these are tracked configuration gaps).
Those assertions are flagged :func:`pytest.xfail` so the suite tracks
the gap without blocking CI.
"""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest
import yaml

from orchestration.config import (
    BridgeConfig,
    DockerConfig,
    FiwareConfig,
    SimulationConfig,
)
from orchestration.utils import load_config


@pytest.mark.xfail(
    reason="load_config does not warn on default postgres/admin "
           "passwords yet. Tracked as a security audit finding.",
    strict=True,
)
def test_default_password_warning_emitted(tmp_path: Path, recwarn):
    """Loading a config with default credentials emits a deprecation warning."""
    cfg_path = tmp_path / "default_creds.yaml"
    cfg_path.write_text(textwrap.dedent("""
        mode: central
        agv_namespaces:
          - agv_0
        fiware:
          enabled: true
          grafana_password: admin
          timescale_password: postgres
    """).strip())

    load_config(str(cfg_path))

    messages = [str(w.message).lower() for w in recwarn.list]
    assert any("default" in msg and "password" in msg for msg in messages), \
        "expected a warning about default credentials"


def test_yaml_parse_errors_surface_clearly(tmp_path: Path):
    """Malformed YAML triggers a :class:`yaml.YAMLError`, not a silent failure."""
    cfg_path = tmp_path / "bad.yaml"
    cfg_path.write_text("mode: [unterminated")
    with pytest.raises(yaml.YAMLError):
        load_config(str(cfg_path))


@pytest.mark.xfail(
    reason="load_config silently falls back to defaults when required keys "
           "are missing instead of raising a typed error.",
    strict=True,
)
def test_missing_required_field_raises_with_context(tmp_path: Path):
    """A missing top-level ``mode`` key produces a descriptive error."""
    cfg_path = tmp_path / "missing.yaml"
    cfg_path.write_text("agv_namespaces:\n  - agv_0\n")
    with pytest.raises((KeyError, ValueError)) as excinfo:
        load_config(str(cfg_path))
    assert "mode" in str(excinfo.value).lower()


# ---------------------------------------------------------------------------
# Light, currently-passing sanity tests for the config dataclasses.
# ---------------------------------------------------------------------------
def test_simulation_config_defaults_are_sane():
    """Default :class:`SimulationConfig` has one AGV in central mode."""
    cfg = SimulationConfig()
    assert cfg.agv_count == 1
    assert cfg.deploy_mode.value == "central"
    assert cfg.bridge_container_name == "central_bridge"


def test_unity_mode_picks_unity_image():
    """Switching to ``unity`` mode resolves bridge_image to ``rises:unity``."""
    cfg = SimulationConfig(mode="unity")
    assert cfg.bridge_image == "rises:unity"
    assert cfg.bridge_container_name == "unity_bridge"


def test_default_docker_config_is_non_privileged():
    """The default :class:`DockerConfig` does not request privileged mode."""
    assert DockerConfig().privileged is False


def test_load_config_round_trip(tmp_path: Path):
    """A minimal YAML file produces the expected :class:`SimulationConfig`."""
    cfg_path = tmp_path / "ok.yaml"
    cfg_path.write_text(textwrap.dedent("""
        mode: central
        agv_namespaces:
          - agv_0
          - agv_1
        ros_domain_id: 7
    """).strip())

    cfg = load_config(str(cfg_path))
    assert cfg.mode == "central"
    assert cfg.agv_namespaces == ["agv_0", "agv_1"]
    assert cfg.ros_domain_id == 7
    assert isinstance(cfg.bridge, BridgeConfig)
    assert isinstance(cfg.fiware, FiwareConfig)
