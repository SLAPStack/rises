# Role in the TRL6-7 demonstrator & validation

## Role in the demonstrator

In the RISES demonstrator the geofence is the **spatial-safety decision layer**
between perception and fleet control:

```
LiDAR scan ─▶ laserscan_preprocessor ─▶ GEOFENCE ─┬─▶ obstacle_alert ─▶ fleet / operator (slow / stop)
                                                   ├─▶ obstacle_report ─▶ validation (KPIs) + FIWARE bridge ─▶ Orion-LD / TimescaleDB / Grafana
                                                   └─▶ unmatched_obstacles ─▶ rises_leg_filter ─▶ /humans/bodies/* (ROS4HRI)
```

It is the component that turns raw LiDAR into a safety decision ("is there
something here that shouldn't be?") and exposes that decision both to the robot's
fleet/operator loop (the `obstacle_alert` Bool) and to the ARISE middleware (the
NGSI-LD `obstacle_report`/`obstacle_alert` attributes for monitoring). See
[architecture.md](architecture.md) for the component and sequence diagrams.

## What was extracted as reusable

The demonstrator was a full warehouse integration; this module extracts the
geofence capability in isolation, with:
- two interchangeable detection backends (spatial KD-tree / occupancy grid),
- a hardware-free run path (rosbag + dockerised FIWARE),
- the validation/KPI pipeline as built-in evidence,
- the ROS 2 + NGSI-LD interfaces documented for third-party reuse.

What remains demonstrator-specific (out of this module's open scope): the physical
AGV drivers, the Unity/MQTT scenario source, and the full fleet controller — these
are represented here by the recorded bag and the orchestration harness.

## Evidence of validation

Measured on the 300 s recorded scenario (88 spawned moving obstacles), spatial
backend:

| KPI | Result | Target | Meaning |
|---|---|---|---|
| Static Structure Match Rate | **1.000** | > 0.98 | Of genuine static structure, fraction matched (intruders excluded via ground-truth + persistence) |
| Detection Ratio | **0.966** | > 0.95 | Spawned intruders detected (geofence physically saw all 88) |
| Detection Latency (avg) | **35.2 ms** | < 2 s | Mean per-scan detection processing latency (40 Hz operating point) |
| Miss Rate | 3.4 % | < 10 % | Spawned intruders not detected |
| System Uptime | ~100 % | > 99 | Geofence-ready fraction of the window |

Gridmap backend reproduces these within tolerance (Static 0.977, Detection 0.943).
KPI definitions live in `scripts/export_kpis.py`.

**Video:** <https://drive.google.com/file/d/1QFXvJPv2RzLDe1EQnjTharl72Os4S0RL/view>.
