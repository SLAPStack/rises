"""
ROS 2 to NGSI-LD bridge — publishes ROS 2 topic data to Orion-LD via REST API.

Subscribes to geofence topics and pushes entity updates to the Orion-LD
Context Broker. Replaces the need for the standalone DDS Enabler.

Map/contour data is buffered locally. Grafana (or any client) triggers
``POST http://localhost:<BRIDGE_PORT>/sync`` to flush the buffered map and
switch to live streaming. All non-map data streams immediately.

This runs inside a ROS 2 environment (e.g., the Vulcanexus container or distrobox).

Usage:
  source /opt/ros/jazzy/setup.bash
  source your workspace's install/setup.bash
  python3 orion_bridge.py

Environment variables:
  ORION_HOST    — Orion-LD host (default: localhost)
  ORION_PORT    — Orion-LD port (default: 1026)
  AGV_ID        — AGV identifier for entity naming (default: agv_0)
  BRIDGE_PORT   — HTTP control port for map sync (default: 9091)
"""

from __future__ import annotations

import json
import logging
import math
import os
import random
import ssl
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros

from rises_interfaces.msg import Contours, ObstacleReport, ObstacleUpdateArray, ObstacleUpdate
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Empty, String

_log = logging.getLogger(__name__)

# Maximum bytes per TROE chunk attribute value.
# Orion-LD TROE silently drops compound JSONB values larger than ~2 KB.
# We use 1850 as a conservative safe limit (below the empirically observed drop point).
_TROE_CHUNK_LIMIT = 1850

# Reusable JSON template overhead: {"count":9999,"rectangles":[]}
_TROE_OVERHEAD = len('{"count":9999,"rectangles":[]}')

# Auth / TLS / retry configuration (env-driven; defaults safe for production).
_ORION_TOKEN_ENV = "ORION_TOKEN"
_ORION_TLS_VERIFY_ENV = "ORION_TLS_VERIFY"
_TLS_TRUE_VALUES = frozenset({"1", "true", "yes", "on"})
_TLS_FALSE_VALUES = frozenset({"0", "false", "no", "off"})

# Retry budget for transient upstream failures (503 / connection-reset).
_HTTP_MAX_RETRIES = 3
_HTTP_RETRY_BACKOFF_BASE_SEC = 0.2
_HTTP_RETRY_JITTER_SEC = 0.05
_HTTP_DEFAULT_TIMEOUT_SEC = 2.0

# urllib's URLError uses a "reason" attribute; treat these substrings as transient.
_TRANSIENT_URLERROR_REASONS = ("connection reset", "connection refused", "timed out")


def _tls_verify_enabled() -> bool:
    """Return True if TLS verification should be enabled (default), False otherwise.

    Driven by the ``ORION_TLS_VERIFY`` env var. Unset or unrecognised values fall
    back to True (fail-safe). An explicit opt-out warns once at call time so
    operators know certificate verification is off.
    """
    raw = os.environ.get(_ORION_TLS_VERIFY_ENV)
    if raw is None:
        return True
    normalised = raw.strip().lower()
    if normalised in _TLS_FALSE_VALUES:
        _log.warning(
            "TLS certificate verification disabled via %s=%s",
            _ORION_TLS_VERIFY_ENV, raw,
        )
        return False
    if normalised in _TLS_TRUE_VALUES:
        return True
    _log.warning(
        "Unrecognised %s=%r; defaulting to TLS verify=True",
        _ORION_TLS_VERIFY_ENV, raw,
    )
    return True


def _ssl_context_for_request(url: str) -> ssl.SSLContext | None:
    """Return an SSL context for HTTPS URLs, or None for plain HTTP.

    Plain ``http://`` URLs do not use TLS so the context is irrelevant.
    """
    if not url.lower().startswith("https://"):
        return None
    if _tls_verify_enabled():
        return ssl.create_default_context()
    return ssl._create_unverified_context()


def _make_request(method: str, url: str, body: bytes | None) -> urllib.request.Request:
    """Build a :class:`urllib.request.Request` with auth + content-type headers.

    Adds ``Authorization: Bearer <token>`` only when ``ORION_TOKEN`` is set and
    non-empty. Callers are responsible for passing pre-encoded JSON in ``body``.
    """
    headers: dict[str, str] = {"Content-Type": "application/json"}
    token = os.environ.get(_ORION_TOKEN_ENV, "").strip()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return urllib.request.Request(url, data=body, method=method, headers=headers)


def _is_transient_urlerror(err: urllib.error.URLError) -> bool:
    """Return True if a URLError looks like a retryable transient failure."""
    reason = getattr(err, "reason", None)
    if reason is None:
        return False
    text = str(reason).lower()
    return any(needle in text for needle in _TRANSIENT_URLERROR_REASONS)


def _urlopen_with_retry(req: urllib.request.Request, timeout: float = _HTTP_DEFAULT_TIMEOUT_SEC):
    """Open ``req`` with bounded exponential-backoff retries on transient errors.

    Retries on HTTP 503 and on URLError reasons that indicate a transport-level
    transient failure (connection reset / refused / timeout). All other errors
    propagate to the caller unchanged so existing 404/409 fallback logic in
    :class:`OrionBridgeNode` keeps working.
    """
    context = _ssl_context_for_request(req.full_url)
    last_exc: Exception | None = None
    for attempt in range(_HTTP_MAX_RETRIES + 1):
        try:
            if context is not None:
                return urllib.request.urlopen(req, timeout=timeout, context=context)
            return urllib.request.urlopen(req, timeout=timeout)
        except urllib.error.HTTPError as exc:
            if exc.code != 503 or attempt >= _HTTP_MAX_RETRIES:
                raise
            last_exc = exc
        except urllib.error.URLError as exc:
            if not _is_transient_urlerror(exc) or attempt >= _HTTP_MAX_RETRIES:
                raise
            last_exc = exc
        delay = _HTTP_RETRY_BACKOFF_BASE_SEC * (2 ** attempt)
        delay += random.uniform(0.0, _HTTP_RETRY_JITTER_SEC)
        _log.warning(
            "Orion request transient failure (attempt %d/%d): %s — retrying in %.2fs",
            attempt + 1, _HTTP_MAX_RETRIES, last_exc, delay,
        )
        time.sleep(delay)
    # Defensive: loop above always either returns or raises; the final retry
    # raises directly. This line is unreachable but satisfies type checkers.
    raise RuntimeError("retry loop exited without result")  # pragma: no cover


def _make_rect_chunks(rectangles: list) -> list[list]:
    """Split rectangles into sub-lists where each serialises to under the TROE size limit.

    Uses exact JSON sizing so chunks are as large as possible without exceeding
    _TROE_CHUNK_LIMIT bytes. Rect coordinates vary in width (3-digit vs 2-digit
    integers, 1-3 decimal places) so fixed counts are not reliable.
    """
    if not rectangles:
        return [[]]

    chunks: list[list] = []
    current: list = []
    # Accumulated byte count: overhead + brackets + per-rect data
    current_bytes = _TROE_OVERHEAD

    for rect in rectangles:
        rect_str = json.dumps(rect)
        # +1 for the comma before this element (not needed for first element)
        extra = len(rect_str) + (1 if current else 0)
        if current and current_bytes + extra > _TROE_CHUNK_LIMIT:
            chunks.append(current)
            current = [rect]
            current_bytes = _TROE_OVERHEAD + len(rect_str)
        else:
            current.append(rect)
            current_bytes += extra

    if current:
        chunks.append(current)
    return chunks


class OrionBridgeNode(Node):
    """Bridges ROS 2 geofence topics to Orion-LD NGSI-LD entities.

    Map geometry is buffered until a Grafana sync request arrives.
    All other attributes stream to Orion-LD immediately.
    """

    def __init__(self) -> None:
        super().__init__("orion_bridge")

        self.orion_host: str = os.environ.get("ORION_HOST", "localhost")
        self.orion_port: str = os.environ.get("ORION_PORT", "1026")
        self.agv_id: str = os.environ.get("AGV_ID", "agv_0")
        self.entity_id: str = f"urn:ngsi-ld:AGV:{self.agv_id}"
        self.base_url: str = f"http://{self.orion_host}:{self.orion_port}/ngsi-ld/v1"

        self.entity_created: bool = False
        self._last_report_time: float = 0.0
        self._report_interval: float = 1.0
        self._last_alert: bool | None = None

        # --- Map buffer state (thread-safe via GIL for simple assignments) ---
        self._buffered_geometry: dict | None = None
        # BUFFER_MAP=false (or unset) → stream geometry immediately (orchestrated mode)
        # BUFFER_MAP=true → buffer until POST /sync (manual test mode)
        self._geometry_streaming: bool = os.environ.get("BUFFER_MAP", "false").lower() != "true"
        self._geometry_lock: threading.Lock = threading.Lock()

        # Subscribe to geofence topics
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Bool, "obstacle_alert", self._alert_callback, 10)
        self.create_subscription(Bool, "geofence_ready", self._ready_callback, reliable_qos)
        self.create_subscription(
            ObstacleReport, "obstacle_report", self._report_callback, sensor_qos
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostics_callback, 10
        )
        # Robot position via TF: map → {agv_id}_base_link published by centralized_translator.
        # odometry/filtered has no publisher in this deployment (no EKF running).
        self._base_link_frame: str = f"{self.agv_id}_base_link"
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._last_position: tuple[float, float] | None = None
        self.create_timer(0.5, self._tf_position_callback)

        # Trigger FiwareBridgeNode to republish its full map state (pallet map + geometry).
        # Sent once after a short delay to ensure the bridge node is fully initialised.
        self._dds_trigger_pub = self.create_publisher(Empty, "fiware/dds_trigger", 1)
        self._trigger_sent: bool = False
        self.create_timer(3.0, self._send_fiware_trigger)
        # Warehouse contours and map updates are published globally (no AGV namespace).
        self.create_subscription(
            Contours, "/warehouse_contours", self._contours_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            OccupancyGrid, "predicted_occupancy",
            self._heatmap_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            ObstacleUpdateArray, "/warehouse/map_updates",
            self._map_updates_callback,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        # FiwareBridgeNode publishes the full pallet map as JSON.
        # Received after we send the dds_trigger at startup.
        self.create_subscription(
            String,
            "fiware/map_obstacles",
            self._fiware_map_obstacles_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        # Validation node result — relayed by FiwareBridgeNode as a JSON string.
        self.create_subscription(
            String,
            "fiware/validation_result",
            self._validation_result_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Accumulated pallet map from map_updates topic
        self._pallet_map: dict[int, tuple[float, float, float, float]] = {}
        self._pallet_map_dirty: bool = False
        self._last_pallet_push_time: float = 0.0
        self._pallet_push_interval: float = 2.0

        self._last_diag_time: float = 0.0
        self._diag_interval: float = 2.0
        self._diag_node_status: dict[str, dict] = {}
        self._last_odom_time: float = 0.0
        self._odom_interval: float = 0.5
        self._last_heatmap_time: float = 0.0
        self._heatmap_interval: float = 1.0

        # Start the HTTP control server in a daemon thread
        bridge_port = int(os.environ.get("BRIDGE_PORT", "9091"))
        self._start_http_server(bridge_port)

        self.get_logger().info(
            f"Orion bridge started. Entity: {self.entity_id}, "
            f"Orion: {self.orion_host}:{self.orion_port}, "
            f"HTTP control: :{bridge_port} "
            f"(map buffered until POST /sync)"
        )

        # Push any pre-existing Orion-LD data to TimescaleDB at startup.
        # Handles restarts where MongoDB has data but TimescaleDB is behind.
        self._initial_sync()

    # ------------------------------------------------------------------
    # HTTP control server
    # ------------------------------------------------------------------

    def _start_http_server(self, port: int) -> None:
        """Launch a lightweight HTTP server for map sync control."""
        node_ref = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                if self.path == "/sync":
                    node_ref._handle_sync_request()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    body = json.dumps({
                        "status": "ok",
                        "geometry_buffered": node_ref._buffered_geometry is not None,
                        "streaming": node_ref._geometry_streaming,
                    })
                    self.wfile.write(body.encode())
                else:
                    self.send_error(404)

            def do_GET(self) -> None:
                if self.path == "/status":
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    body = json.dumps({
                        "geometry_buffered": node_ref._buffered_geometry is not None,
                        "streaming": node_ref._geometry_streaming,
                        "entity_created": node_ref.entity_created,
                    })
                    self.wfile.write(body.encode())
                else:
                    self.send_error(404)

            def do_OPTIONS(self) -> None:
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def log_message(self, format: str, *args: object) -> None:
                pass  # suppress request logs

        server = HTTPServer(("0.0.0.0", port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

    def _handle_sync_request(self) -> None:
        """Flush buffered map geometry and enable live streaming."""
        with self._geometry_lock:
            if self._buffered_geometry is not None:
                self.upsert_attribute("warehouse_geometry", self._buffered_geometry)
                self.get_logger().info(
                    "Sync requested: flushed buffered warehouse geometry to Orion-LD"
                )
            else:
                self.get_logger().info(
                    "Sync requested: no geometry buffered yet"
                )
            self._geometry_streaming = True

    # ------------------------------------------------------------------
    # ROS callbacks — non-map data streams immediately
    # ------------------------------------------------------------------

    def _alert_callback(self, msg: Bool) -> None:
        if msg.data != self._last_alert:
            self._last_alert = msg.data
            self.upsert_attribute("obstacle_alert", msg.data)
            self.get_logger().info(f"obstacle_alert = {msg.data}")

    def _ready_callback(self, msg: Bool) -> None:
        self.upsert_attribute("geofence_ready", msg.data)
        self.get_logger().info(f"geofence_ready = {msg.data}")

    def _tf_position_callback(self) -> None:
        """Update robot_position from TF map→{agv_id}_base_link at 2 Hz.

        The TF map frame uses the Unity/ROS convention which is rotated
        relative to the warehouse coordinate frame used by pallets and contours.
        Transform: warehouse_x = -tf_y, warehouse_y = -tf_x.
        Heading transform: map_theta = atan2(-cos(tf_theta), -sin(tf_theta)).
        """
        try:
            t = self._tf_buffer.lookup_transform(
                "map", self._base_link_frame, rclpy.time.Time()
            )
            tf_x = t.transform.translation.x
            tf_y = t.transform.translation.y
            # Rotate TF frame into warehouse frame (same transform used by test_sync.py)
            x = round(-tf_y, 3)
            y = round(-tf_x, 3)
            q = t.transform.rotation
            tf_theta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            theta = round(math.atan2(-math.cos(tf_theta), -math.sin(tf_theta)), 4)
            if self._last_position != (x, y):
                self._last_position = (x, y)
                self.upsert_attribute("robot_position", {"x": x, "y": y, "theta": theta})
        except Exception:
            pass  # TF not yet available — silently skip

    def _send_fiware_trigger(self) -> None:
        """Send a one-shot dds_trigger to FiwareBridgeNode to republish pallet map."""
        if not self._trigger_sent:
            self._trigger_sent = True
            self._dds_trigger_pub.publish(Empty())
            self.get_logger().info(
                "Sent DDS trigger to FiwareBridgeNode for map data refresh"
            )

    def _fiware_map_obstacles_callback(self, msg: String) -> None:
        """Receive full pallet map JSON from FiwareBridgeNode and push to Orion-LD."""
        try:
            data = json.loads(msg.data)
            count = data.get("count", 0)
            if count == 0:
                # FiwareBridgeNode may have just restarted with no pallet data.
                # Don't overwrite a valid Orion-LD record with an empty one.
                return

            # FiwareBridgeNode sends object format {x_min, y_min, x_max, y_max}.
            # Convert to compact 4-element arrays and push in TROE-safe chunks.
            raw_rects = data.get("rectangles", [])
            compact = [
                [r["x_min"], r["y_min"], r["x_max"], r["y_max"]]
                for r in raw_rects
                if isinstance(r, dict)
            ]
            chunks = _make_rect_chunks(compact)
            for i, chunk_rects in enumerate(chunks):
                attr_name = f"map_obstacles_{i:02d}"
                self.upsert_attribute(attr_name, {"count": count, "rectangles": chunk_rects})

            self.get_logger().info(
                f"Received map_obstacles from FiwareBridgeNode: count={count}, "
                f"chunks={len(chunks)}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to parse fiware/map_obstacles JSON: {e}")

    def _validation_result_callback(self, msg: String) -> None:
        """Receive validation result JSON from FiwareBridgeNode and push to Orion-LD."""
        try:
            data = json.loads(msg.data)
            self.upsert_attribute("validation_result", data)
            stats = data.get("stats", {})
            self.get_logger().info(
                f"validation_result: spawned={stats.get('spawned')}, "
                f"detected={stats.get('detected')}, "
                f"avg_latency_ms={stats.get('avg_latency_ms')}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to parse fiware/validation_result JSON: {e}")

    def _initial_sync(self) -> None:
        """Push any pre-existing Orion-LD data to TimescaleDB at startup.

        Handles restarts where MongoDB has entity data but TimescaleDB is
        missing rows because no PATCH happened in this session.
        """
        url = f"{self.base_url}/entities/{self.entity_id}"
        try:
            req = _make_request("GET", url, body=None)
            with _urlopen_with_retry(req, timeout=5) as resp:
                entity = json.loads(resp.read())
        except Exception as e:
            self.get_logger().debug(f"Initial sync skipped (Orion not reachable): {e}")
            return

        self.entity_created = True

        for attr_name in [
            "warehouse_geometry", "robot_position",
            "obstacle_alert", "geofence_ready", "predicted_heatmap",
            "obstacle_report",
        ]:
            attr = entity.get(attr_name)
            if attr is None:
                continue
            value = attr.get("value")
            if value is None:
                continue
            # Skip empty placeholder values — not worth writing to TimescaleDB
            if attr_name == "warehouse_geometry" and isinstance(value, dict) and not value.get("wall_segments"):
                continue
            self.upsert_attribute(attr_name, value)
            self.get_logger().info(f"Initial sync: pushed {attr_name} from Orion-LD to TimescaleDB")

        # Re-chunk pallet data using the current TROE-safe sizing.
        # The entity in Orion-LD may have oversized chunks (from a previous code version)
        # that TROE silently drops. Collect all rectangles from all chunk attrs,
        # re-chunk with dynamic sizing, then PATCH both Orion-LD and TimescaleDB.
        all_rects: list = []
        total_count: int = 0
        for attr_name, attr in sorted(entity.items()):
            if not attr_name.startswith("map_obstacles_"):
                continue
            value = attr.get("value") if isinstance(attr, dict) else None
            if not value or not isinstance(value, dict):
                continue
            rects = value.get("rectangles", [])
            if not rects:
                continue
            if total_count == 0:
                total_count = value.get("count", len(rects))
            all_rects.extend(rects)

        if all_rects:
            chunks = _make_rect_chunks(all_rects)
            for i, chunk_rects in enumerate(chunks):
                attr_name = f"map_obstacles_{i:02d}"
                self.upsert_attribute(attr_name, {"count": total_count, "rectangles": chunk_rects})
            self.get_logger().info(
                f"Initial sync: re-chunked {total_count} pallets into "
                f"{len(chunks)} TROE-safe chunk(s)"
            )

    def _contours_callback(self, msg: Contours) -> None:
        """Buffer map geometry. Only push to Orion-LD if streaming is enabled."""
        walls = [
            {
                "x1": round(float(seg.start.x), 3),
                "y1": round(float(seg.start.y), 3),
                "x2": round(float(seg.end.x), 3),
                "y2": round(float(seg.end.y), 3),
            }
            for seg in msg.outer_contour_segments
        ]

        hull = [
            {"x": round(float(pt.x), 3), "y": round(float(pt.y), 3)}
            for pt in msg.outer_contour_hull.points
        ]

        inner_polygons = [
            [
                {"x": round(float(pt.x), 3), "y": round(float(pt.y), 3)}
                for pt in poly.points
            ]
            for poly in msg.inner_contours
        ]

        geometry = {
            "wall_segments": walls,
            "outer_hull": hull,
            "inner_polygons": inner_polygons,
        }

        with self._geometry_lock:
            self._buffered_geometry = geometry
            is_streaming = self._geometry_streaming

        self.get_logger().info(
            f"Warehouse geometry received: {len(walls)} walls, "
            f"{len(hull)} hull pts, {len(inner_polygons)} inner polygons "
            f"({'streamed' if is_streaming else 'buffered'})"
        )

        if is_streaming:
            self.upsert_attribute("warehouse_geometry", geometry)

    @staticmethod
    def _tf_to_warehouse(x: float, y: float) -> tuple[float, float]:
        """Convert TF map-frame coords to warehouse frame: wx = -tf_y, wy = -tf_x."""
        return (-y, -x)

    @classmethod
    def _build_segment(cls, obs) -> dict:
        """Build a warehouse-frame line-segment dict for one obstacle.

        Uses first/last vertices when available; falls back to a small
        horizontal stub centred on the obstacle position. All coords are
        rounded to mm precision to keep the compound JSON below Orion-LD's
        TROE 2 KB limit.
        """
        if len(obs.vertices) >= 2:
            x1, y1 = cls._tf_to_warehouse(obs.vertices[0].x, obs.vertices[0].y)
            x2, y2 = cls._tf_to_warehouse(obs.vertices[-1].x, obs.vertices[-1].y)
        else:
            cx, cy = cls._tf_to_warehouse(obs.position.x, obs.position.y)
            x1, y1 = cx - 0.1, cy
            x2, y2 = cx + 0.1, cy
        return {
            "x1": round(x1, 3), "y1": round(y1, 3),
            "x2": round(x2, 3), "y2": round(y2, 3),
        }

    def _report_callback(self, msg: ObstacleReport) -> None:
        now = time.monotonic()
        if now - self._last_report_time < self._report_interval:
            return
        self._last_report_time = now

        # Obstacles as line segments (start→end) for Grafana visualization.
        # Obstacle positions from the geofencing node are in the TF map frame;
        # apply the same transform as robot_position so they appear correctly
        # on the warehouse-frame map (warehouse_x = -tf_y, warehouse_y = -tf_x).
        # Cap segment counts so the compound JSON stays under the Orion-LD TROE
        # 2 KB hard limit.  With 12+12 segments × ~54 bytes each the payload is
        # ≈1500 bytes — safely within the limit.  Unmatched are more important for
        # anomaly detection so they get priority; matched fill the remainder.
        _MAX_UNMATCHED = 12
        _MAX_MATCHED = 12

        unmatched_segments = [
            self._build_segment(obs)
            for obs in msg.unmatched_obstacles[:_MAX_UNMATCHED]
        ]
        matched_segments = [
            self._build_segment(obs)
            for obs in msg.matched_obstacles[:_MAX_MATCHED]
        ]

        report = {
            "matched_count": len(msg.matched_obstacles),
            "unmatched_count": len(msg.unmatched_obstacles),
            "unmatched_ids": [int(o.id) for o in msg.unmatched_obstacles[:10]],
            "unmatched_segments": unmatched_segments,
            "matched_segments": matched_segments,
        }
        self.upsert_attribute("obstacle_report", report)

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        now = time.monotonic()
        if now - self._last_diag_time < self._diag_interval:
            return
        self._last_diag_time = now

        # Each node runs its own diagnostic_updater and publishes /diagnostics
        # independently, so a single message only ever carries one node's
        # status. Merge into the running cache (keyed by node name) instead
        # of replacing the whole attribute, or each push would clobber every
        # other node's last-known status.
        for status in msg.status:
            node_name = status.name.split(":")[0].strip()
            level = status.level  # 0=OK, 1=WARN, 2=ERROR, 3=STALE (byte in Humble)
            self._diag_node_status[node_name] = {
                "level": int.from_bytes(level, byteorder="little") if isinstance(level, bytes) else int(level),
                "message": status.message,
                "values": {kv.key: kv.value for kv in status.values[:10]},
            }

        if self._diag_node_status:
            self.upsert_attribute("diagnostics", self._diag_node_status)

    def _heatmap_callback(self, msg: OccupancyGrid) -> None:
        """Convert OccupancyGrid to compact hot-cell list for Grafana visualization."""
        now = time.monotonic()
        if now - self._last_heatmap_time < self._heatmap_interval:
            return
        self._last_heatmap_time = now

        resolution = float(msg.info.resolution)
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = int(msg.info.width)
        height = int(msg.info.height)

        # Collect cells above threshold as (x, y, value).
        # Cap at 200 to keep payload manageable for Orion-LD.
        heatmap_threshold = 10
        max_cells = 200
        hot_cells: list[dict] = []

        for row in range(height):
            for col in range(width):
                cell = msg.data[row * width + col]
                if cell >= heatmap_threshold and len(hot_cells) < max_cells:
                    world_x = origin_x + (col + 0.5) * resolution
                    world_y = origin_y + (row + 0.5) * resolution
                    hot_cells.append({
                        "x": round(world_x, 2),
                        "y": round(world_y, 2),
                        "v": int(cell),
                    })

        heatmap_data = {
            "nonzero_cells": sum(1 for c in msg.data if c > 0),
            "max_value": int(max(msg.data)) if msg.data else 0,
            "resolution": resolution,
            "origin_x": round(origin_x, 2),
            "origin_y": round(origin_y, 2),
            "grid_width": width,
            "grid_height": height,
            "hot_cells": hot_cells,
        }
        self.upsert_attribute("predicted_heatmap", heatmap_data)

    def _map_updates_callback(self, msg: ObstacleUpdateArray) -> None:
        """Accumulate pallet INSERT/DELETE operations and push as map_obstacles."""
        for update in msg.updates:
            obstacle_id = int(update.obstacle.id)
            if update.operation == ObstacleUpdate.OP_DELETE:
                self._pallet_map.pop(obstacle_id, None)
            else:  # OP_INSERT
                obs = update.obstacle
                half_w = float(obs.width) * 0.5
                half_h = float(obs.height) * 0.5
                cx = float(obs.position.x)
                cy = float(obs.position.y)
                self._pallet_map[obstacle_id] = (
                    round(cx - half_w, 3), round(cy - half_h, 3),
                    round(cx + half_w, 3), round(cy + half_h, 3),
                )
        self._pallet_map_dirty = True

        # Throttle pushing to Orion-LD
        now = time.monotonic()
        if now - self._last_pallet_push_time < self._pallet_push_interval:
            return
        self._last_pallet_push_time = now
        self._push_pallet_map()

    def _push_pallet_map(self) -> None:
        """Push current pallet map state to Orion-LD as TROE-safe chunked attributes."""
        if not self._pallet_map_dirty:
            return
        if not self._pallet_map:
            # Accumulator is empty — we missed the initial VOLATILE burst.
            # Don't overwrite a valid record that FiwareBridgeNode already pushed.
            self._pallet_map_dirty = False
            return
        self._pallet_map_dirty = False

        # Store as compact 4-element arrays [x_min, y_min, x_max, y_max].
        # TROE silently drops compound values > ~2 KB, so split across chunk
        # attributes map_obstacles_00, map_obstacles_01, … Each chunk fits TROE.
        rectangles = [
            [aabb[0], aabb[1], aabb[2], aabb[3]]
            for aabb in self._pallet_map.values()
        ]
        total_count = len(rectangles)
        chunks = _make_rect_chunks(rectangles)
        for i, chunk_rects in enumerate(chunks):
            attr_name = f"map_obstacles_{i:02d}"
            self.upsert_attribute(attr_name, {"count": total_count, "rectangles": chunk_rects})

        self.get_logger().info(
            f"Pushed {total_count} map obstacles to Orion-LD in {len(chunks)} chunk(s)"
        )

    # ------------------------------------------------------------------
    # Orion-LD helpers
    # ------------------------------------------------------------------

    def upsert_attribute(self, attr_name: str, value: object) -> None:
        """Create or update an attribute on the AGV entity in Orion-LD."""
        if not self.entity_created:
            self.ensure_entity()

        payload = {attr_name: {"type": "Property", "value": value}}
        url = f"{self.base_url}/entities/{self.entity_id}/attrs"

        try:
            data = json.dumps(payload).encode("utf-8")
            req = _make_request("PATCH", url, body=data)
            with _urlopen_with_retry(req, timeout=2):
                pass
        except urllib.error.HTTPError as e:
            if e.code in (204, 207):
                pass  # partial update or no content
            elif e.code == 404:
                # Attribute doesn't exist yet — append via POST
                try:
                    req = _make_request("POST", url, body=data)
                    with _urlopen_with_retry(req, timeout=2):
                        self.get_logger().info(f"Created attribute {attr_name}")
                except Exception as post_err:
                    self.get_logger().warn(
                        f"Orion POST failed for {attr_name}: {post_err}"
                    )
            else:
                self.get_logger().warn(
                    f"Orion PATCH failed ({e.code}): {e.read().decode()[:200]}"
                )
        except Exception as e:
            self.get_logger().debug(f"Orion update failed: {e}")

    def ensure_entity(self) -> None:
        """Create the AGV entity in Orion-LD if it doesn't exist."""
        payload = {
            "id": self.entity_id,
            "type": "AGV",
            "obstacle_alert": {"type": "Property", "value": False},
            "geofence_ready": {"type": "Property", "value": False},
            "robot_position": {
                "type": "Property",
                "value": {"x": 0.0, "y": 0.0},
            },
            "warehouse_geometry": {
                "type": "Property",
                "value": {"wall_segments": [], "outer_hull": [], "inner_polygons": []},
            },
            "diagnostics": {
                "type": "Property",
                "value": {},
            },
            "obstacle_report": {
                "type": "Property",
                "value": {
                    "matched_count": 0,
                    "unmatched_count": 0,
                    "unmatched_ids": [],
                },
            },
            "predicted_heatmap": {
                "type": "Property",
                "value": {
                    "nonzero_cells": 0,
                    "max_value": 0,
                    "hot_cells": [],
                },
            },
            # map_obstacles is not pre-declared here — chunk attributes
            # (map_obstacles_00, map_obstacles_01, …) are created on first push.
            "validation_result": {
                "type": "Property",
                "value": {},
            },
        }

        try:
            data = json.dumps(payload).encode("utf-8")
            req = _make_request("POST", f"{self.base_url}/entities", body=data)
            with _urlopen_with_retry(req, timeout=5):
                self.get_logger().info(f"Created entity {self.entity_id}")
        except urllib.error.HTTPError as e:
            if e.code == 409:
                self.get_logger().info(f"Entity {self.entity_id} already exists")
            else:
                self.get_logger().warn(
                    f"Entity creation failed ({e.code}): {e.read().decode()[:200]}"
                )
                return
        except Exception as e:
            self.get_logger().warn(f"Cannot reach Orion-LD: {e}")
            return

        self.entity_created = True


def main() -> None:
    rclpy.init()
    node = OrionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
