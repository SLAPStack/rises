# Interfaces

This module exposes ROS 2 / Vulcanexus interfaces, a FIWARE/NGSI-LD mapping, an
optional DDS-Enabler configuration, and a (partial) ROS4HRI alignment. All
interfaces below are extracted from the source; representative file references
are given.

## 1. ROS 2 / Vulcanexus

### Nodes

| Node (name) | Class / plugin | Lifecycle | Role |
|---|---|---|---|
| `geofence_node` | `rises::GeofenceNode` | LifecycleNode | Spatial (nanoflann KD-tree) detection backend — **default** |
| `geofence_gridmap_node` | `rises::GeofenceGridmapNode` | LifecycleNode | Occupancy-grid detection backend — alternative (mutually exclusive) |
| `validation_node` | `rises::ValidationNode` | Node | KPI/validation (spawn↔detection matching) — optional |

All three are also registered as composable components and run in a
`component_container_mt` (MultiThreadedExecutor) by default.

### Subscriptions (geofence node)

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `lidar_segments` | `rises_interfaces/msg/ObstacleArray` | SensorData (best-effort, depth 10), reentrant cb group | Segmented LiDAR input from the preprocessor |
| `/warehouse/map_updates` | `rises_interfaces/msg/ObstacleUpdateArray` | reliable, depth 100, volatile | Bulk map insert/delete |
| `map_boundary` | `rises_interfaces/msg/Contours` | reliable, depth 10, transient-local | Warehouse outer boundary + inner contours |
| `<robot_pose_prefix>/<id>/pose` | `geometry_msgs/msg/PoseStamped` | sensor QoS | Per-robot footprint filtering (only if `enable_robot_filtering=true`) |

### Publications (geofence node)

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `obstacle_report` | `rises_interfaces/msg/ObstacleReport` | reliable, depth 10 | Per-segment matched/unmatched report — the primary detection contract |
| `obstacle_alert` | `std_msgs/msg/Bool` | reliable, depth 10 | Safety alert (true when an unmatched obstacle is in the safety zone) |
| `unmatched_obstacles` | `rises_interfaces/msg/ObstacleArray` | reliable, depth 10 | Unmatched segments republished for RViz / leg-filter |
| `geofence_ready` | `std_msgs/msg/Bool` | reliable, depth 1, transient-local | Readiness signal after map pre-load (if `publish_ready_signal=true`) |
| RViz markers | `visualization_msgs` | — | Map, safety circle, matched/error segments (throttled) |

### Validation node

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| sub | `obstacle_report` | `rises_interfaces/msg/ObstacleReport` | Detections to score |
| sub | `obstacle_spawn` | `std_msgs/msg/String` (JSON) | Ground-truth obstacle spawns |
| pub | `validation_result` | `std_msgs/msg/String` (JSON) | Cumulative KPI stats (detected, spawned, latency, static-miss, …) |

### Services (geofence node)

| Service | Type | Purpose |
|---|---|---|
| `validate_path` | `rises_interfaces/srv/ValidatePath` | Is a `nav_msgs/Path` blocked by an obstacle/boundary? |
| `set_area_state` | `rises_interfaces/srv/SetAreaState` | Lock/unlock a rectangular area |
| `update_map` | `rises_interfaces/srv/UpdateMap` | Bulk obstacle insert/delete |
| `update_warehouse_layout` | `rises_interfaces/srv/UpdateWarehouseLayout` | Load boundary contours |
| `set_safety_circle_radius` | `rises_interfaces/srv/SetSafetyCircleRadius` | Tune the safety radius at runtime |
| `get_area_state` / `get_safety_radius` / `get_map_info` / `get_warehouse_contours` | `rises_interfaces/srv/Get*` | Read-only queries (spatial node) |

### Key parameters (defaults)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `enable_safety_circle` / `safety_circle_radius` | bool / double | true / 0.5 m | Radial detection zone (set per scenario, e.g. 5.0 m in the smoke) |
| `correspondence_tolerance` | double | 0.10 m | Vertex→map match threshold (validated optimum — see KPI note) |
| `process_points_only` | bool | true | Per-vertex matching mode |
| `min_segment_points` / `outlier_filter_distance` | int / double | 3 / 0.10 m | Segmentation noise control |
| `enable_robot_filtering` / `robot_ids` | bool / string[] | false / ["self"] | Multi-robot self/other footprint filtering |
| `auto_activate` | bool | false | Auto lifecycle transition to ACTIVE |
| `grid_resolution` / `obstacle_inflation_radius` (gridmap) | double / double | 0.05 m / 0.0 m | Occupancy-grid cell size / inflation |
| `match_tolerance` / `match_window` / `persistent_min_reports` (validation) | double / double / int | 2.5 m / 3.0 s / 120 | Spawn↔detection association + static-miss persistence |

### Launch

`rises_bringup/launch/geofence.launch.py` — composable or standalone; `gridmap_enabled`
selects the backend (spatial XOR gridmap); `validation_enabled` adds the validation node;
topics are remappable via `REMAP_*` env vars. Scenario parameters come from
`rises_bringup/config/params_{default,rosbag,unity}.yaml`.

### Messages (summary)

- **`ObstacleReport`** — `Header`, `Obstacle[] matched_obstacles`, `Obstacle[] unmatched_obstacles` (each unmatched carries a persistent tracker `id`).
- **`Obstacle`** — `type` (LINE/POINT/RECTANGLE/POLYGON/CIRCLE…), `id`, `position`, `vertices[]`, `width`/`height`/`radius`/`orientation`.
- **`ObstacleArray`** — `Header`, `Obstacle[]`.
- **`Contours`** — outer boundary segments + hull + inner polygons.
- **`ObstacleUpdateArray`** — `ObstacleUpdate[]` (OP_INSERT / OP_DELETE).

## 2. FIWARE / NGSI-LD

The bridge (`fiware/bridge/orion_bridge.py`) upserts geofence state to an Orion-LD
broker. Entity: **`urn:ngsi-ld:AGV:<agv_id>`** (type `AGV`). Endpoint
`http://<ORION_HOST>:1026/ngsi-ld/v1` (`PATCH …/entities/{id}/attrs`, `POST` fallback).
NGSI-LD core `@context` (no custom context). Persisted to **TimescaleDB TROE**
(`attributes` table: `boolean` / `compound` columns), which the KPI exporter reads.
Compound payloads are chunked/capped to stay under the TROE ~2 KB limit.

| Attribute | NGSI-LD kind | From topic | Meaning |
|---|---|---|---|
| `obstacle_report` | Property (compound) | `obstacle_report` | `{matched_count, unmatched_count, unmatched_ids[], matched_segments[], unmatched_segments[]}` (segments capped at 12 each) |
| `obstacle_alert` | Property (boolean) | `obstacle_alert` | Safety alert state |
| `geofence_ready` | Property (boolean) | `geofence_ready` | Node readiness (uptime KPI) |
| `robot_position` | Property (compound) | TF `map→<id>_base_link` | `{x, y, theta}` (warehouse frame), ~2 Hz |
| `validation_result` | Property (compound) | `validation_result` | `{stats:{spawned, detected, avg_latency_ms, static_miss_segments, …}}` — KPI source |
| `warehouse_geometry` | Property (compound) | `map_boundary` | Walls + outer hull + inner polygons |
| `map_obstacles_NN` | Property (compound) | map updates | Pallet AABBs, TROE-safe chunks |
| `predicted_heatmap`, `diagnostics` | Property (compound) | heatmap / `/diagnostics` | Prediction + node health |

A minimal payload example and the full `@context` strategy are in
`fiware/bridge/orion_bridge.py`.

## 3. DDS ↔ NGSI-LD (DDS-Enabler)

| Item | Path / status |
|---|---|
| DDS-Enabler config | `fiware/config/dds-enabler.json` — maps `rt/<ns>/fiware/*` DDS topics → AGV NGSI-LD attributes (domain 0, UDP) |
| Fast-DDS XTypes profile | `fiware/config/fastdds_profiles.xml` (shared type discovery) |
| Enabler container | `fiware/dds-enabler/Dockerfile` (eProsima FIWARE DDS-Enabler), in `fiware/docker-compose.yaml` |

**Honest status:** the DDS-Enabler is configured and available for direct
DDS→NGSI-LD bridging, but the **default production write path** is the
`OrionBridgeNode` (a native ROS 2 subscriber → HTTP `PATCH` to Orion-LD). The
DDS-Enabler is the alternative ingestion path. State this explicitly in the report
rather than claiming DDS-Enabler is the sole path.

## 4. ROS4HRI / ROS4RI alignment

Provided by the **optional** `rises_leg_filter` node, which consumes the geofence's
`unmatched_obstacles` and detects human leg-pairs.

| HRI concept | Module representation | Alignment | Evidence |
|---|---|---|---|
| Human presence | leg-pair detections → body IDs on `/humans/bodies/lidar_tracked` | **Partial** — uses the `/humans/bodies/` namespace but as a `std_msgs/String` ID list (not yet `hri_msgs/IdsList`) | `rises_leg_filter/src/leg_filter_node.cpp` |
| Body pose | per-body TF frames `body_lidar_<id>` | Aligned (TF convention) | same |
| Confidence | LiDAR-only 0.3 / camera-confirmed 0.7 | Two-tier; camera fusion disabled by default | same |

**Honest status:** alignment is partial — the namespace and TF convention follow
ROS4HRI, but the message types are placeholders pending migration to `hri_msgs`,
and camera fusion is gated off. The geofence itself is decoupled (works without the
leg-filter). Recommend documenting this as "alignment + planned contribution" in the
report, with the `hri_msgs` migration as future work.
