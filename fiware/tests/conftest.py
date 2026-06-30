"""Shared fixtures and ROS-import stubs for the fiware test suites.

The production modules under test import ROS 2 Python packages
(``rclpy``, ``tf2_ros``, ``rises_interfaces``) that are not available
when running plain ``pytest``. This conftest installs lightweight stub
modules into :data:`sys.modules` so the production modules import
cleanly. Tests exercise the network / file boundaries directly.

Production modules live alongside scripts (no ``__init__.py``); we load
them via :mod:`importlib.util` from absolute paths.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from typing import Iterator

import pytest


# ---------------------------------------------------------------------------
# Path setup.
# ---------------------------------------------------------------------------
_HERE = Path(__file__).resolve().parent
_FIWARE_ROOT = _HERE.parent
_BRIDGE_FILE = _FIWARE_ROOT / "bridge" / "orion_bridge.py"
_RECEIVER_FILE = _FIWARE_ROOT / "alert_service" / "alert_receiver.py"


# ---------------------------------------------------------------------------
# ROS 2 stubs — installed once at conftest import time.
# ---------------------------------------------------------------------------
def _install_stub(name: str, **attrs: object) -> types.ModuleType:
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


def _install_ros_stubs() -> None:
    """Install minimal stand-ins for the ROS 2 modules ``orion_bridge`` imports."""
    if "rclpy" in sys.modules and getattr(sys.modules["rclpy"], "_rises_stub", False):
        return

    rclpy = _install_stub("rclpy")
    rclpy._rises_stub = True  # type: ignore[attr-defined]

    class _Time:
        def __init__(self, *_, **__) -> None:
            pass

    rclpy.time = _install_stub("rclpy.time", Time=_Time)

    class _Logger:
        def info(self, *_): pass
        def warn(self, *_): pass
        def debug(self, *_): pass
        def error(self, *_): pass

    class _Pub:
        def publish(self, *_): return None

    class _Node:
        def __init__(self, *_, **__) -> None:
            self._logger = _Logger()
        def create_subscription(self, *_, **__): return None
        def create_publisher(self, *_, **__): return _Pub()
        def create_timer(self, *_, **__): return None
        def get_logger(self): return self._logger
        def destroy_node(self): return None

    rclpy.node = _install_stub("rclpy.node", Node=_Node)

    class _QoSProfile:
        def __init__(self, *_, **__): pass

    class _ReliabilityPolicy:
        RELIABLE = 1
        BEST_EFFORT = 2

    class _DurabilityPolicy:
        TRANSIENT_LOCAL = 1
        VOLATILE = 2

    rclpy.qos = _install_stub(
        "rclpy.qos",
        QoSProfile=_QoSProfile,
        ReliabilityPolicy=_ReliabilityPolicy,
        DurabilityPolicy=_DurabilityPolicy,
    )

    class _Buffer:
        def lookup_transform(self, *_, **__):
            raise RuntimeError("no transform")

    class _TransformListener:
        def __init__(self, *_, **__): pass

    _install_stub("tf2_ros", Buffer=_Buffer, TransformListener=_TransformListener)

    class _Empty: pass

    class _Bool:
        def __init__(self): self.data = False

    class _String:
        def __init__(self): self.data = ""

    _install_stub("std_msgs")
    _install_stub("std_msgs.msg", Empty=_Empty, Bool=_Bool, String=_String)

    class _DiagArr:
        def __init__(self): self.status = []

    _install_stub("diagnostic_msgs")
    _install_stub("diagnostic_msgs.msg", DiagnosticArray=_DiagArr)

    class _OccGrid:
        class info:
            resolution = 0.05
            width = 0
            height = 0
            class origin:
                class position:
                    x = 0.0
                    y = 0.0
        data: list = []

    _install_stub("nav_msgs")
    _install_stub("nav_msgs.msg", OccupancyGrid=_OccGrid)

    class _Contours:
        outer_contour_segments: list = []
        inner_contours: list = []
        class outer_contour_hull:
            points: list = []

    class _ObstacleReport:
        matched_obstacles: list = []
        unmatched_obstacles: list = []

    class _ObstacleUpdate:
        OP_INSERT = 0
        OP_DELETE = 1

    class _ObstacleUpdateArray:
        updates: list = []

    _install_stub("rises_interfaces")
    _install_stub(
        "rises_interfaces.msg",
        Contours=_Contours,
        ObstacleReport=_ObstacleReport,
        ObstacleUpdate=_ObstacleUpdate,
        ObstacleUpdateArray=_ObstacleUpdateArray,
    )


_install_ros_stubs()


# ---------------------------------------------------------------------------
# Module loaders.
# ---------------------------------------------------------------------------
def _load_from_path(module_name: str, path: Path) -> types.ModuleType:
    """Import a Python file by absolute path and cache it in :data:`sys.modules`."""
    if module_name in sys.modules:
        return sys.modules[module_name]
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load module {module_name} from {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="session")
def orion_bridge_module() -> types.ModuleType:
    """Load :mod:`orion_bridge` once per session."""
    return _load_from_path("orion_bridge", _BRIDGE_FILE)


@pytest.fixture(scope="session")
def alert_receiver_module() -> types.ModuleType:
    """Load :mod:`alert_receiver` once per session."""
    return _load_from_path("alert_receiver", _RECEIVER_FILE)


# ---------------------------------------------------------------------------
# Generic fixtures.
# ---------------------------------------------------------------------------
@pytest.fixture
def clean_env(monkeypatch) -> Iterator[pytest.MonkeyPatch]:
    """Strip bridge / receiver env vars so every test starts from a known state."""
    for key in (
        "ORION_HOST", "ORION_PORT", "AGV_ID", "BRIDGE_PORT", "BUFFER_MAP",
        "ORION_TOKEN", "ORION_TLS_VERIFY",
    ):
        monkeypatch.delenv(key, raising=False)
    yield monkeypatch


@pytest.fixture
def log_dir(tmp_path, monkeypatch, alert_receiver_module) -> Path:
    """Provide a tmp dir and redirect :data:`alert_receiver.LOG_DIR` to it."""
    monkeypatch.setattr(alert_receiver_module, "LOG_DIR", tmp_path)
    return tmp_path
