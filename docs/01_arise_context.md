# ARISE context

## Why this module exists

ARISE FSTP experiment **RISES** built an AGV human-robot-interaction stack for
shared industrial spaces. Its safety-critical core is the **geofence**: the
component that decides, in real time, whether what the LiDAR sees is *known
warehouse structure* (safe) or a *novel/dynamic obstacle* such as a person or a
misplaced object (act on it). This module packages that capability as a reusable,
documented, hardware-free-runnable unit.

## Continuity across milestones

| Milestone | Result | Reflected in this module |
|---|---|---|
| Stage 1 — IMP | Need: safe AGV operation around people in a warehouse | Defined the geofence scope (spatial safety + intruder detection) |
| Stage 2 — PoC | Prototype obstacle detection vs. a known map | Became the spatial-matching backend |
| Stage 3 — TRL6-7 demonstrator (D3, **approved**) | Full AGV stack: Vulcanexus + Fast DDS + DDS-Enabler + NGSI-LD, validation framework | The geofence is **extracted** here as the reusable module, with its validation/KPI pipeline |
| Stage 4 — D4 (this) | Open, documented, reproducible module | The geofence nodes + interfaces + FIWARE bridge + hardware-free demo |

## Role of ARISE middleware concepts

- **ROS 2 / Vulcanexus (Jazzy):** the module is a set of lifecycle/composable ROS 2
  nodes built on Vulcanexus; Fast DDS carries the data, with tuned QoS per topic
  (best-effort sensor data, transient-local map/boundary, reliable reports/alerts).
- **FIWARE / NGSI-LD:** geofence state is published as NGSI-LD properties on an
  `AGV` entity in Orion-LD and persisted to TimescaleDB (TROE) — the basis for
  fleet monitoring and the validation KPIs.
- **DDS ↔ NGSI-LD:** a Fast-DDS DDS-Enabler configuration is included for direct
  DDS-to-NGSI-LD bridging (alongside the default ROS→Orion bridge).
- **ROS4HRI:** the optional leg-filter maps geofence detections into the
  `/humans/bodies/` namespace (partial alignment; see interfaces).

A third party should be able to understand *what the module solves*, *how to run it
without the warehouse*, and *how it plugs into ARISE* in under 10 minutes via this
`docs/` set.
