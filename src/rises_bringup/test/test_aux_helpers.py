"""Tests for :mod:`rises_bringup.rises_aux` and :class:`rises_bringup.rises_config.RisesConfig`."""

from __future__ import annotations

from dataclasses import fields

import pytest

try:
    from launch.actions import OpaqueFunction  # type: ignore
    _LAUNCH_AVAILABLE = True
except ImportError:
    _LAUNCH_AVAILABLE = False

_skip_if_no_launch = pytest.mark.skipif(
    not _LAUNCH_AVAILABLE,
    reason="ROS 2 ``launch`` package unavailable — source install/setup.bash.",
)


@_skip_if_no_launch
def test_get_rosbag_record_action_returns_action():
    """rises_aux.get_rosbag_record_action returns an :class:`OpaqueFunction`."""
    from rises_bringup.rises_aux import get_rosbag_record_action  # type: ignore

    action = get_rosbag_record_action(context=None)
    assert isinstance(action, OpaqueFunction)


@_skip_if_no_launch
def test_rises_config_dataclass_fields_present():
    """:class:`RisesConfig` exposes all expected configuration fields."""
    from rises_bringup.rises_config import RisesConfig  # type: ignore

    expected = {
        "use_composition",
        "bag_file",
        "record_bag",
        "record_topics",
        "storage_backend",
        "log_level",
        "namespace",
        "play_rosbag",
        "rosbag_delay",
        "rviz_config_file",
        "launch_rviz",
        "mqtt_params_path",
    }
    actual = {f.name for f in fields(RisesConfig)}
    missing = expected - actual
    assert not missing, f"RisesConfig missing fields: {sorted(missing)}"


@_skip_if_no_launch
def test_rises_config_load_returns_instance():
    """:meth:`RisesConfig.load_config` instantiates the dataclass."""
    from rises_bringup.rises_config import RisesConfig  # type: ignore

    cfg = RisesConfig.load_config()
    assert isinstance(cfg, RisesConfig)
