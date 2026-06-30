# Installation & Hello World

The module runs **without any robot hardware**: a recorded rosbag drives the
pipeline and a dockerised FIWARE stack stores the results. Docker is the supported
path.

## Prerequisites

| Dependency | Hello-world | Full demo | Evidence |
|---|---|---|---|
| OS | Ubuntu 24.04 (host w/ Docker) | same | — |
| ROS 2 / Vulcanexus | **Jazzy** (in the image) | same | `rises.dockerfile` (`eprosima/vulcanexus:jazzy-base`) |
| Docker + Compose | required | required | `rises.dockerfile`, `fiware/docker-compose.yaml` |
| Build deps (in image) | Eigen3, nanoflann, liburcu, Boost, nlohmann-json, (xsimd if `USE_SIMD=ON`) | same | `geofence/CMakeLists.txt`, `geofence/package.xml` |
| FIWARE stack | Orion-LD, TimescaleDB, Mintaka, Grafana (+ optional DDS-Enabler) | same | `fiware/docker-compose.yaml` |
| Hardware | **none** (rosbag) | none (rosbag) | `resources/smoke_bag_andros_300s/` |
| Python (host) | 3 (for `orchestration.cli`, `export_kpis.py`) | same | `orchestration/`, `scripts/` |

## Build the image

```bash
cd <repo-root>
docker build -f rises.dockerfile --target base -t rises:base .
```

This builds the ROS 2 workspace (`rises_interfaces` first, then `geofence` and the
rest) into `/workspace/install` inside the image. Release build, LTO on, SIMD off by
default.

> Note: a fast iteration path exists for source-only changes — an overlay build
> `FROM rises:base` that re-runs only `colcon build --packages-select <pkg>` (no apt),
> useful when the upstream apt mirrors are unavailable.

## Hello World (proves the install)

The "hello world" is a short pipeline run that produces a KPI CSV — evidence the
geofence, bridge, and FIWARE stack are wired correctly.

```bash
# 1. reset + start the FIWARE stack
docker compose -f fiware/docker-compose.yaml -p rises-fiware down -v
docker compose -f fiware/docker-compose.yaml -p rises-fiware up -d \
    mongodb timescaledb orion mintaka grafana
# wait until Orion-LD answers:
until [ "$(curl -s -o /dev/null -w '%{http_code}' \
    'http://localhost:1026/ngsi-ld/v1/entities?type=AGV')" = 200 ]; do sleep 2; done

# 2. run the 300 s recorded scenario through the geofence + validation + bridge
python3 -m orchestration.cli -c orchestration/configs/andros_moving_obstacles_smoke.yaml
#    (replays resources/smoke_bag_andros_300s; geofence + validation + FIWARE bridge run in the AGV container)

# 3. on shutdown the KPIs are written to:
ls -t logs/kpis_*.csv | head -1
```

### Expected output

A `logs/kpis_*.csv` with ~11 KPIs and an empty error column, e.g.:

```
Static Structure Match Rate, 1.0000, ratio, >0.98, ...
Detection Ratio (spawned vs detected), 0.9659, ratio, >0.95, ...
Detection Latency (avg), 0.0352, seconds, <2, ...
```

Live state can also be inspected while running:
```bash
docker exec rises-timescaledb psql -U postgres -d orion -t -A \
  -c "SELECT id, ts FROM attributes WHERE id LIKE '%obstacle_report' ORDER BY ts DESC LIMIT 3;"
```

### FIWARE service ports

| Service | Port | Use |
|---|---|---|
| Orion-LD | 1026 | NGSI-LD broker |
| TimescaleDB | 5432 | TROE history (KPI source) |
| Mintaka | 8080 | NGSI-LD temporal query API |
| Grafana | 3000 | dashboards (`admin`/`admin`) |

## Adapting to your own robot

Point the geofence at your scan source and map:
- Provide a planar `sensor_msgs/LaserScan` (segmented to `lidar_segments` via the
  preprocessor) or publish `rises_interfaces/msg/ObstacleArray` on `lidar_segments`.
- Load your map via `update_map` / `update_warehouse_layout` services or JSON files
  (`obstacles_json_file`, `contours_json_file`).
- Remap topics with `REMAP_*` env vars; set `safety_circle_radius` for your platform.
