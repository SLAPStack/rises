# ARISE Geofence — Spatial Safety & Obstacle Detection Module

> **D4 reusable HRI module (ARISE Open Call 1, experiment RISES / ID 13).**
> Overview for the shareable-module repository. A couple of organisation-side items
> remain to confirm before final submission — the stable release tag and the
> maintainer GitHub/Discord handle (marked `TODO` below).

## What this module is

A ROS 2 / Vulcanexus **safety geofence** for mobile robots / AGVs in shared
industrial spaces. It consumes 2-D LiDAR detections, matches them against a known
warehouse map, and flags everything that is **not** part of the map as a potential
intruder — raising a safety alert and publishing a per-segment obstacle report.
It is the spatial-awareness layer that lets an AGV operate safely around people and
unexpected objects, and it exposes its state to the ARISE middleware (FIWARE/NGSI-LD)
for fleet-level monitoring and KPI validation.

| | |
|---|---|
| **Inputs** | 2-D LiDAR segments (`ObstacleArray`), warehouse map (obstacles + boundary), robot pose (multi-robot) |
| **Outputs** | per-segment `obstacle_report` (matched/unmatched), boolean `obstacle_alert`, `unmatched_obstacles`, `geofence_ready` |
| **HRI capability** | detect-and-flag novel/dynamic obstacles (incl. people) inside a configurable safety zone; feeds a ROS4HRI leg-filter for human-body detection |
| **Off-the-shelf** | two interchangeable detection backends (spatial KD-tree / occupancy grid), path-validation + map services, a FIWARE/NGSI-LD bridge, and a KPI/validation pipeline |
| **Maturity** | TRL ~5; validated on a recorded warehouse scenario without real hardware |

## Connection with ARISE

- **ROS 2 / Vulcanexus (Jazzy):** lifecycle nodes, composable containers, Fast DDS.
- **FIWARE / NGSI-LD:** a bridge upserts geofence state to an Orion-LD broker
  (`urn:ngsi-ld:AGV:<id>`), persisted to TimescaleDB (TROE) for KPI export.
- **DDS ↔ NGSI-LD:** a Fast-DDS **DDS-Enabler** config is provided
  (`fiware/config/dds-enabler.json`) for direct DDS-to-NGSI-LD bridging; the
  default production write path is the ROS→Orion bridge node (see
  [02_interfaces.md](02_interfaces.md)).
- **ROS4HRI:** an optional `rises_leg_filter` consumes the geofence's
  `unmatched_obstacles` and publishes human-body detections into the
  `/humans/bodies/` namespace (partial alignment — see interfaces doc).

## Robot missions & tasks

| Mission | Contribution |
|---|---|
| Human-aware navigation / intralogistics | flags people & unexpected objects in the safety zone so the planner/fleet can slow or stop |
| Operator assistance / shared workspace safety | boolean safety alert + per-obstacle report for HMI / fleet supervisor |
| Multi-robot coordination | per-AGV obstacle reports, self/other-robot footprint filtering |

## Target platforms

| Category | Status |
|---|---|
| Mobile robot / AMR / AGV (2-D safety LiDAR) | **Tested** (recorded "andros" warehouse AGV scenario) |
| Any ROS 2 Jazzy robot publishing a planar `LaserScan` / segmented `ObstacleArray` | Expected compatible |
| Manipulator / humanoid | Not targeted |
| Simulation / recorded data | **Supported** — full hardware-free run from a rosbag (see below) |

## Quick start (hardware-free)

A complete run needs **no robot** — a recorded bag drives the pipeline and a
dockerised FIWARE stack stores the results. See
[03_installation_and_hello_world.md](03_installation_and_hello_world.md)
for prerequisites and the hello-world, and
[04_basic_demo_how_to_use.md](04_basic_demo_how_to_use.md) for the demo.

```bash
# 1. build the image (ROS 2 Vulcanexus Jazzy)
docker build -f rises.dockerfile --target base -t rises:base .
# 2. bring up FIWARE (Orion-LD + TimescaleDB + Mintaka + Grafana)
docker compose -f fiware/docker-compose.yaml -p rises-fiware up -d
# 3. replay the 300 s warehouse bag through the geofence + validation + bridge
python3 -m orchestration.cli -c orchestration/configs/andros_moving_obstacles_smoke.yaml
# 4. KPIs are exported to logs/kpis_*.csv
```

## Validation evidence

Measured on the 300 s recorded scenario (88 spawned moving obstacles):

| KPI | Result | Target |
|---|---|---|
| Static Structure Match Rate (known map structure correctly matched) | **1.000** | > 0.98 |
| Detection Ratio (spawned intruders detected) | **0.966** | > 0.95 |
| Detection Latency (avg) | **35.2 ms** (mean per-scan, 40 Hz operating point) | < 2 s |
| Miss Rate | 3.4 % | < 10 % |

See [05_role_in_demonstrator.md](05_role_in_demonstrator.md) for the KPI
methodology.

## Repository map

```
<module>/
  README.md                     # this file
  LICENSE                       # Apache-2.0 (geofence package)
  docs/
    01_arise_context.md
    02_interfaces.md            # ROS 2 / NGSI-LD / DDS / ROS4HRI
    03_installation_and_hello_world.md
    04_basic_demo_how_to_use.md
    05_role_in_demonstrator.md
    06_limitations.md
    architecture.md             # architecture + sequence diagrams
  geofence/                     # the module source (spatial + gridmap + validation nodes)
  rises_interfaces/             # custom messages
  fiware/                       # NGSI-LD bridge + DDS-Enabler config + docker-compose
  orchestration/                # hardware-free run harness + scenario configs
  scripts/export_kpis.py        # validation KPI export
```

## Limitations & boundaries

See [06_limitations.md](06_limitations.md). In short: 2-D only; validated
on one recorded warehouse scenario; ROS4HRI leg-filter alignment is partial; the
DDS-Enabler path is provided but the ROS→Orion bridge is the default.

## License / contact

- **License:** Apache-2.0 — see the top-level [`LICENSE`](../LICENSE) and [`NOTICE`](../NOTICE).
- **Maintainer:** SLAPStack GmbH — niklas.dietz@slapstack.de · GitHub/Discord handle `TODO`
- **Repository:** <https://github.com/SLAPStack/rises> · **Release tag:** `TODO: stable v1.0.0`
- **Demonstrator video:** <https://drive.google.com/file/d/1QFXvJPv2RzLDe1EQnjTharl72Os4S0RL/view> (see also [`media/video_link.md`](../media/video_link.md))
