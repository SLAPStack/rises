"""Smoke tests for :mod:`rises_bringup` launch files.

Each test imports a launch file as a Python module and invokes
``generate_launch_description()``. The launch files resolve the
``rises_bringup`` package share directory via :mod:`ament_index_python`;
when that lookup fails (e.g. the workspace has not been built into the
test environment), the affected test is skipped rather than failed.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from typing import Any

import pytest


_LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"


def _has_ament() -> bool:
    """Return True iff ``ament_index_python`` can locate ``rises_bringup``."""
    try:
        from ament_index_python.packages import (  # type: ignore
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return False
    try:
        get_package_share_directory("rises_bringup")
    except (PackageNotFoundError, Exception):  # noqa: BLE001
        return False
    return True


_AMENT_AVAILABLE = _has_ament()
_skip_if_no_ament = pytest.mark.skipif(
    not _AMENT_AVAILABLE,
    reason="rises_bringup package share dir not available — build the "
           "workspace and source install/setup.bash to enable this test.",
)


def _load_launch_module(filename: str) -> types.ModuleType:
    """Load ``rises_bringup/launch/<filename>`` as a private module."""
    path = _LAUNCH_DIR / filename
    module_name = f"_rises_launch_{path.stem}"
    if module_name in sys.modules:
        return sys.modules[module_name]
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod


def _declared_arg_names(launch_description: Any) -> set[str]:
    """Return the set of names declared by :class:`DeclareLaunchArgument` entries."""
    from launch.actions import DeclareLaunchArgument  # type: ignore

    return {
        entity.name for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def _default_for(launch_description: Any, arg_name: str) -> str | None:
    """Return the raw default for a declared launch argument, or None."""
    from launch.actions import DeclareLaunchArgument  # type: ignore

    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == arg_name:
            return entity.default_value[0].text if entity.default_value else None
    return None


# ---------------------------------------------------------------------------
# generate_launch_description() smoke tests.
# ---------------------------------------------------------------------------
@_skip_if_no_ament
def test_rises_geofence_launch_imports_without_error():
    """rises_geofence.launch.py imports and produces a LaunchDescription."""
    mod = _load_launch_module("rises_geofence.launch.py")
    ld = mod.generate_launch_description()
    assert ld is not None
    assert ld.entities, "launch description has no entities"


@_skip_if_no_ament
def test_central_launch_imports_without_error():
    """central.launch.py imports and produces a LaunchDescription."""
    mod = _load_launch_module("central.launch.py")
    ld = mod.generate_launch_description()
    assert ld is not None
    assert ld.entities


@_skip_if_no_ament
def test_geofence_launch_imports_without_error():
    """geofence.launch.py imports and produces a LaunchDescription."""
    mod = _load_launch_module("geofence.launch.py")
    ld = mod.generate_launch_description()
    assert ld is not None
    assert ld.entities


@_skip_if_no_ament
def test_multi_agv_geofence_launch_imports_without_error():
    """multi_agv_geofence.launch.py imports and produces a LaunchDescription."""
    mod = _load_launch_module("multi_agv_geofence.launch.py")
    ld = mod.generate_launch_description()
    assert ld is not None
    assert ld.entities


@_skip_if_no_ament
def test_unity_launch_imports_without_error():
    """unity.launch.py imports and produces a LaunchDescription."""
    mod = _load_launch_module("unity.launch.py")
    ld = mod.generate_launch_description()
    assert ld is not None
    assert ld.entities


# ---------------------------------------------------------------------------
# Argument-declaration tests.
# ---------------------------------------------------------------------------
@_skip_if_no_ament
def test_namespace_argument_declared():
    """Per-AGV launch files declare a ``namespace`` argument with a sensible default."""
    for filename in (
        "rises_geofence.launch.py",
        "geofence.launch.py",
    ):
        mod = _load_launch_module(filename)
        ld = mod.generate_launch_description()
        args = _declared_arg_names(ld)
        assert "namespace" in args, f"{filename} missing 'namespace' arg"
        default = _default_for(ld, "namespace")
        # Empty string or 'agv_0' are both acceptable defaults.
        assert default in ("", "agv_0", None), \
            f"{filename} namespace default '{default}' is unexpected"


@_skip_if_no_ament
def test_play_rosbag_default_false():
    """Default for ``play_rosbag`` is False on launches that declare it."""
    # geofence.launch.py defaults play_rosbag to 'false' (rosbag is opt-in).
    mod = _load_launch_module("geofence.launch.py")
    ld = mod.generate_launch_description()
    args = _declared_arg_names(ld)
    if "play_rosbag" in args:
        assert _default_for(ld, "play_rosbag") == "false"
