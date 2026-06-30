#!/usr/bin/env python3
"""
Replays real ROS bag data through the Orion-LD bridge pipeline without ROS.

Reads warehouse contours, pallet map updates, odometry, and TF transforms
directly from bag files (sqlite3). Supports both MQTT-based bags
(discerning_safety_humble) and processed-topic bags (arise_warehouse_recording_final).

Map geometry is buffered until Grafana requests it via POST /sync.
Pallet rectangles are written directly to TimescaleDB (Orion-LD TROE
truncates large compound values).

Usage:
    python3 test_sync.py [bag_dir]

    bag_dir defaults to ~/rosbags/arise_warehouse_recording_final
"""

from __future__ import annotations

import json
import logging
import os
import sqlite3
import struct
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("test_sync")

ORION = "http://localhost:1026/ngsi-ld/v1"
ENTITY = "urn:ngsi-ld:AGV:agv_0"
BRIDGE_PORT = int(os.environ.get("BRIDGE_PORT", "9091"))

# Pallet map stored separately — too large for Orion-LD TROE compound
TIMESCALE_DIRECT = os.environ.get("TIMESCALE_DIRECT", "true").lower() == "true"


class SyncState:
    """Encapsulates mutable state shared between the HTTP server and the main loop."""

    def __init__(self) -> None:
        self.buffered_geometry: dict | None = None
        self.buffered_pallets: dict | None = None
        self.geometry_streaming: bool = False
        self.lock: threading.Lock = threading.Lock()


# Single instance — functions reference this instead of module-level globals.
state = SyncState()


# ---------------------------------------------------------------------------
# Bag reading helpers
# ---------------------------------------------------------------------------

def bag_topics(db_path: str) -> dict[str, tuple[int, str]]:
    """Return {topic_name: (topic_id, type)} for all topics in the bag."""
    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("SELECT id, name, type FROM topics")
    result = {name: (tid, ttype) for tid, name, ttype in cur.fetchall()}
    db.close()
    return result


def read_bag_string_messages(
    db_path: str, topic_name: str
) -> list[tuple[int, str]]:
    """Read std_msgs/String messages from a bag. Returns [(timestamp_ns, text)]."""
    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        db.close()
        return []
    tid = row[0]

    messages: list[tuple[int, str]] = []
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,),
    )
    for ts, raw in cur.fetchall():
        str_len = struct.unpack_from("<I", raw, 4)[0]
        text = raw[8 : 8 + str_len - 1].decode("utf-8")
        messages.append((ts, text))

    db.close()
    return messages


def read_bag_tf_positions(
    db_path: str, topic_name: str, child_prefix: str
) -> list[tuple[int, float, float]]:
    """Read TF transforms from a bag. Returns [(timestamp_ns, x, y)]."""
    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        db.close()
        return []
    tid = row[0]

    positions: list[tuple[int, float, float]] = []
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,),
    )
    for ts, raw in cur.fetchall():
        offset = 4  # CDR header
        offset += 4  # n_transforms
        offset += 8  # stamp sec + nsec
        slen = struct.unpack_from("<I", raw, offset)[0]
        offset += 4 + slen
        offset = (offset + 3) & ~3
        slen = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        child = raw[offset : offset + slen - 1].decode("utf-8")
        offset += slen
        offset = (offset + 3) & ~3
        if not child.startswith(child_prefix):
            continue
        tx, ty = struct.unpack_from("<dd", raw, offset)
        positions.append((ts, tx, ty))

    db.close()
    return positions


def read_bag_odometry(db_path: str) -> list[tuple[int, float, float]]:
    """Read nav_msgs/Odometry from /odometry/filtered. Returns [(ts_ns, x, y)]."""
    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("SELECT id FROM topics WHERE name='/odometry/filtered'")
    row = cur.fetchone()
    if row is None:
        db.close()
        return []
    tid = row[0]

    positions: list[tuple[int, float, float]] = []
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,),
    )
    for ts, raw in cur.fetchall():
        # Odometry CDR: 4 header + Header(stamp 8 + frame_id string) + child_frame_id string
        # + Pose (position xyz + orientation xyzw + covariance 36*8)
        offset = 4  # CDR
        offset += 8  # stamp
        slen = struct.unpack_from("<I", raw, offset)[0]
        offset += 4 + slen
        offset = (offset + 3) & ~3  # align after frame_id
        slen = struct.unpack_from("<I", raw, offset)[0]
        offset += 4 + slen
        offset = (offset + 3) & ~3  # align after child_frame_id
        # PoseWithCovariance: Pose (position xyz 3*8 + orientation xyzw 4*8)
        x, y = struct.unpack_from("<dd", raw, offset)
        positions.append((ts, x, y))

    db.close()
    return positions


def read_bag_bool(db_path: str, topic_name: str) -> list[tuple[int, bool]]:
    """Read std_msgs/Bool messages. Returns [(ts_ns, value)]."""
    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        db.close()
        return []
    tid = row[0]

    messages: list[tuple[int, bool]] = []
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,),
    )
    for ts, raw in cur.fetchall():
        # Bool CDR: 4 header + 1 byte bool
        val = raw[4] != 0
        messages.append((ts, val))

    db.close()
    return messages


# ---------------------------------------------------------------------------
# Convert bag contours format to bridge format
# ---------------------------------------------------------------------------

def convert_contours_json(raw_json: str) -> dict:
    """Convert /mqtt/warehouse_contours JSON to warehouse_geometry format."""
    data = json.loads(raw_json)

    wall_segments = []
    hull_set: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()
    for seg in data.get("outer_contour", []):
        p1, p2 = seg
        wall_segments.append({
            "x1": round(p1[0], 3), "y1": round(p1[1], 3),
            "x2": round(p2[0], 3), "y2": round(p2[1], 3),
        })
        for p in [p1, p2]:
            key = (round(p[0], 3), round(p[1], 3))
            if key not in seen:
                seen.add(key)
                hull_set.append(key)

    outer_hull = [{"x": pt[0], "y": pt[1]} for pt in hull_set]
    if outer_hull and outer_hull[0] != outer_hull[-1]:
        outer_hull.append(outer_hull[0])

    inner_polygons: list[list[dict[str, float]]] = []
    for contour_segs in data.get("inner_contours", []):
        ring = [{"x": round(seg[0][0], 3), "y": round(seg[0][1], 3)} for seg in contour_segs]
        if contour_segs:
            ring.append({"x": round(contour_segs[-1][1][0], 3),
                         "y": round(contour_segs[-1][1][1], 3)})
        inner_polygons.append(ring)

    return {"wall_segments": wall_segments, "outer_hull": outer_hull,
            "inner_polygons": inner_polygons}


def build_pallet_state(
    map_msgs: list[tuple[int, str]],
) -> list[tuple[int, dict[int, tuple[float, float, float, float]]]]:
    """Build cumulative pallet snapshots from discerning_safety_map INSERT/DELETE ops."""
    pallets: dict[int, tuple[float, float, float, float]] = {}
    snapshots: list[tuple[int, dict[int, tuple[float, float, float, float]]]] = []
    for ts, msg_text in map_msgs:
        data = json.loads(msg_text)
        for pallet in data.get("pallets", []):
            pid = pallet["id"]
            op = pallet.get("operation", "INSERT")
            if op == "DELETE" and pid in pallets:
                del pallets[pid]
            elif op == "INSERT" and pallet.get("aabb"):
                aabb = pallet["aabb"]
                if aabb and isinstance(aabb[0][0], list):
                    aabb = aabb[0]
                pallets[pid] = (aabb[0][0], aabb[0][1], aabb[1][0], aabb[1][1])
        snapshots.append((ts, dict(pallets)))
    return snapshots


# ---------------------------------------------------------------------------
# Orion-LD helpers
# ---------------------------------------------------------------------------

def orion_patch(attrs: dict) -> bool:
    url = f"{ORION}/entities/{ENTITY}/attrs"
    data = json.dumps(attrs).encode()
    try:
        req = urllib.request.Request(
            url, data=data, method="PATCH",
            headers={"Content-Type": "application/json"},
        )
        urllib.request.urlopen(req, timeout=3)
        return True
    except urllib.error.HTTPError as e:
        if e.code in (204, 207):
            return True
        if e.code == 404:
            try:
                req = urllib.request.Request(
                    url, data=data, method="POST",
                    headers={"Content-Type": "application/json"},
                )
                urllib.request.urlopen(req, timeout=3)
                return True
            except Exception as post_err:
                log.warning("orion_patch POST fallback failed: %s", post_err)
        return False
    except Exception as err:
        log.debug("orion_patch failed: %s", err)
        return False


def ensure_entity() -> None:
    payload = {
        "id": ENTITY, "type": "AGV",
        "obstacle_alert": {"type": "Property", "value": False},
        "geofence_ready": {"type": "Property", "value": False},
        "robot_position": {"type": "Property", "value": {"x": 0.0, "y": 0.0}},
        "warehouse_geometry": {"type": "Property", "value": {
            "wall_segments": [], "outer_hull": [], "inner_polygons": []}},
        "diagnostics": {"type": "Property", "value": {}},
        "obstacle_report": {"type": "Property", "value": {
            "matched_count": 0, "unmatched_count": 0, "unmatched_ids": [],
            "unmatched_positions": [], "matched_positions": [],
            "matched_rectangles": []}},
    }
    try:
        data = json.dumps(payload).encode()
        req = urllib.request.Request(
            f"{ORION}/entities", data=data, method="POST",
            headers={"Content-Type": "application/json"},
        )
        urllib.request.urlopen(req, timeout=5)
        log.info("[entity] Created")
    except urllib.error.HTTPError as e:
        if e.code == 409:
            log.info("[entity] Already exists")
        else:
            log.info(f"[entity] Error: {e.code}")


def write_pallets_to_timescale(pallets: dict[int, tuple[float, float, float, float]]) -> None:
    """Write pallet rectangles directly to TimescaleDB as a pallet_map attribute.

    Orion-LD TROE truncates large compound values, so we bypass it.
    """
    import psycopg2  # noqa: delayed import — only needed for direct DB writes
    rects = [
        {"x_min": round(a[0], 3), "y_min": round(a[1], 3),
         "x_max": round(a[2], 3), "y_max": round(a[3], 3)}
        for a in pallets.values()
    ]
    compound = json.dumps({"rectangles": rects, "count": len(rects)})

    ts_host = os.environ.get("TIMESCALE_HOST", "localhost")
    ts_port = os.environ.get("TIMESCALE_PORT", "5432")
    ts_user = os.environ.get("TIMESCALE_USER", "postgres")
    ts_pass = os.environ.get("TIMESCALE_PASSWORD", "postgres")

    conn = psycopg2.connect(
        host=ts_host, port=ts_port, dbname="orion",
        user=ts_user, password=ts_pass,
    )
    cur = conn.cursor()
    cur.execute(
        """INSERT INTO attributes
           (instanceid, id, entityid, valuetype, compound, ts, datasetid)
           VALUES (%s, %s, %s, 'Compound', %s::jsonb, now(), '@none')""",
        (
            f"urn:ngsi-ld:attribute:instance:{ENTITY}:pallet_map",
            "https://uri.etsi.org/ngsi-ld/default-context/pallet_map",
            ENTITY,
            compound,
        ),
    )
    conn.commit()
    cur.close()
    conn.close()
    log.info(f"[pallets] Wrote {len(rects)} rectangles directly to TimescaleDB")


# ---------------------------------------------------------------------------
# HTTP sync server
# ---------------------------------------------------------------------------

def handle_sync() -> dict:

    with state.lock:
        flushed_geo = False
        flushed_pallets = False

        if state.buffered_geometry is not None:
            orion_patch({"warehouse_geometry": {
                "type": "Property", "value": state.buffered_geometry}})
            flushed_geo = True
            log.info("[sync] Flushed buffered geometry to Orion-LD")

        if state.buffered_pallets is not None:
            try:
                write_pallets_to_timescale(state.buffered_pallets)
                flushed_pallets = True
            except Exception as e:
                log.info(f"[sync] Pallet write failed: {e}")
                # Fallback: push to Orion-LD (may truncate)
                rects = [
                    {"x_min": round(a[0], 3), "y_min": round(a[1], 3),
                     "x_max": round(a[2], 3), "y_max": round(a[3], 3)}
                    for a in list(state.buffered_pallets.values())[:50]
                ]
                orion_patch({"obstacle_report": {"type": "Property", "value": {
                    "matched_count": len(state.buffered_pallets),
                    "unmatched_count": 0, "unmatched_ids": [],
                    "unmatched_positions": [], "matched_positions": [],
                    "matched_rectangles": rects,
                }}})
                flushed_pallets = True

        if not flushed_geo and not flushed_pallets:
            log.info("[sync] Nothing buffered yet")

        state.geometry_streaming = True
        log.info("[sync] Streaming enabled")

    return {"status": "ok", "geometry": flushed_geo, "pallets": flushed_pallets,
            "streaming": True}


def start_http_server() -> None:
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self) -> None:
            if self.path == "/sync":
                result = handle_sync()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps(result).encode())
            else:
                self.send_error(404)

        def do_GET(self) -> None:
            if self.path == "/status":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                body = json.dumps({
                    "geometry_buffered": state.buffered_geometry is not None,
                    "pallets_buffered": state.buffered_pallets is not None,
                    "streaming": state.geometry_streaming,
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

        def log_message(self, fmt: str, *args: object) -> None:
            pass

    server = HTTPServer(("0.0.0.0", BRIDGE_PORT), Handler)
    log.info(f"[http] Listening on :{BRIDGE_PORT}")
    server.serve_forever()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def find_bag_db3(bag_dir: Path) -> str:
    db3_files = list(bag_dir.glob("*.db3"))
    if not db3_files:
        log.info(f"No .db3 files found in {bag_dir}")
        sys.exit(1)
    return str(db3_files[0])


def main() -> None:
    # State accessed via module-level state instance

    # Resolve bag directory
    if len(sys.argv) > 1:
        bag_dir = Path(sys.argv[1]).expanduser()
    else:
        bag_dir = Path.home() / "rosbags" / "arise_warehouse_recording_final"

    if not bag_dir.exists():
        log.info(f"Bag directory not found: {bag_dir}")
        sys.exit(1)

    db3_path = find_bag_db3(bag_dir)
    log.info(f"[bag] Reading from {db3_path}")

    topics = bag_topics(db3_path)
    log.info(f"[bag] Topics: {', '.join(sorted(topics.keys()))}")

    ensure_entity()

    # --- Load warehouse contours ---
    contour_msgs = read_bag_string_messages(db3_path, "/mqtt/warehouse_contours")
    if contour_msgs:
        _, contour_json = contour_msgs[-1]
        state.buffered_geometry = convert_contours_json(contour_json)
        n_walls = len(state.buffered_geometry["wall_segments"])
        n_inner = len(state.buffered_geometry["inner_polygons"])
        log.info(f"[map] Buffered geometry: {n_walls} wall segments, {n_inner} inner polygons")
    else:
        log.info("[map] No /mqtt/warehouse_contours messages")

    # --- Load pallet map updates ---
    map_msgs = read_bag_string_messages(db3_path, "/mqtt/discerning_safety_map")
    pallet_snapshots = build_pallet_state(map_msgs)
    if pallet_snapshots:
        _, final_pallets = pallet_snapshots[-1]
        state.buffered_pallets = final_pallets
        log.info(f"[map] Buffered {len(final_pallets)} pallets from "
              f"{len(pallet_snapshots)} map updates")
    else:
        log.info("[map] No /mqtt/discerning_safety_map messages")

    # --- Load robot positions ---
    # Prefer /odometry/filtered (direct), fall back to /tf or /mqtt/agv/tf
    positions: list[tuple[int, float, float]] = []
    needs_tf_transform = False

    if "/odometry/filtered" in topics:
        positions = read_bag_odometry(db3_path)
        log.info(f"[odom] Loaded {len(positions)} odometry positions")
    elif "/tf" in topics:
        agv_id = os.environ.get("AGV_ID", "agv_0")
        prefix = f"{agv_id}_base_link"
        raw = read_bag_tf_positions(db3_path, "/tf", prefix)
        # TF from Unity frame: map_x = -tf_y, map_y = -tf_x
        positions = [(ts, -y, -x) for ts, x, y in raw]
        needs_tf_transform = True
        log.info(f"[tf]  Loaded {len(positions)} positions (transformed)")
    elif "/mqtt/agv/tf" in topics:
        agv_id = os.environ.get("AGV_ID", "agv_0")
        prefix = f"{agv_id}_base_link"
        raw = read_bag_tf_positions(db3_path, "/mqtt/agv/tf", prefix)
        positions = [(ts, -y, -x) for ts, x, y in raw]
        needs_tf_transform = True
        log.info(f"[tf]  Loaded {len(positions)} positions (transformed)")

    if positions:
        xs = [p[1] for p in positions[:100]]
        ys = [p[2] for p in positions[:100]]
        log.info(f"[pos] Range: X [{min(xs):.1f}, {max(xs):.1f}] "
              f"Y [{min(ys):.1f}, {max(ys):.1f}]"
              f"{' (TF transformed)' if needs_tf_transform else ''}")

    # --- Load obstacle alerts ---
    alerts: list[tuple[int, bool]] = []
    if "/obstacle_alert" in topics:
        alerts = read_bag_bool(db3_path, "/obstacle_alert")
        log.info(f"[alert] Loaded {len(alerts)} obstacle_alert messages")

    log.info()
    log.info("=== Ready to replay ===")
    log.info(f"  Grafana:  http://localhost:3000  (admin/admin)")
    log.info(f"  Sync:     curl -X POST http://localhost:{BRIDGE_PORT}/sync")
    log.info()

    # Start HTTP server
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    # --- Merge all events and replay ---
    all_events: list[tuple[int, str, object]] = []
    for ts, x, y in positions:
        all_events.append((ts, "position", (x, y)))
    for ts, val in alerts:
        all_events.append((ts, "alert", val))
    # Pallet updates at their timestamps
    for ts, snap in pallet_snapshots:
        all_events.append((ts, "pallets", snap))
    all_events.sort(key=lambda e: e[0])

    if not all_events:
        log.info("[replay] No events to replay. Waiting for sync requests...")
        try:
            threading.Event().wait()
        except KeyboardInterrupt:
            return

    t0_bag = all_events[0][0]
    t0_wall = time.monotonic()
    odom_throttle = 0.5
    last_odom_push = 0.0
    last_alert: bool | None = None
    step = 0

    log.info("[replay] Starting playback...")
    try:
        for ts, event_type, payload in all_events:
            elapsed_bag = (ts - t0_bag) / 1e9
            elapsed_wall = time.monotonic() - t0_wall
            wait = elapsed_bag - elapsed_wall
            if 0 < wait < 5.0:
                time.sleep(wait)
            elif wait >= 5.0:
                t0_bag = ts
                t0_wall = time.monotonic()

            if event_type == "position":
                x, y = payload
                now = time.monotonic()
                if now - last_odom_push < odom_throttle:
                    continue
                last_odom_push = now
                orion_patch({"robot_position": {
                    "type": "Property",
                    "value": {"x": round(x, 3), "y": round(y, 3)},
                }})
                streaming_tag = " [streaming]" if state.geometry_streaming else ""
                log.info(f"[{step:4d}] pos=({x:8.3f}, {y:8.3f}){streaming_tag}")
                step += 1

            elif event_type == "alert":
                val = payload
                if val != last_alert:
                    last_alert = val
                    orion_patch({"obstacle_alert": {
                        "type": "Property", "value": val}})
                    log.info(f"[{step:4d}] alert={val}")

            elif event_type == "pallets":
                pallet_dict = payload
                orion_patch({"geofence_ready": {
                    "type": "Property", "value": True}})
                # Update buffered pallets for sync
                with state.lock:
                    state.buffered_pallets = pallet_dict
                    if state.geometry_streaming:
                        try:
                            write_pallets_to_timescale(pallet_dict)
                        except Exception:
                            pass  # non-critical

    except KeyboardInterrupt:
        pass

    log.info("\n[replay] Playback complete. Press Ctrl+C to exit.")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        log.info("Stopped.")


if __name__ == "__main__":
    main()
