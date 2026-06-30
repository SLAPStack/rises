# Basic demo & how to use

The hello-world proves the install; this demo shows the geofence *doing its job* —
detecting moving intruders against a known warehouse map and reporting them to the
ARISE middleware — and how to read the result.

## Scenario

`orchestration/configs/andros_moving_obstacles_smoke.yaml` replays a 300 s recording
of an AGV in the "andros" warehouse. The map (walls + ~2000 pallets) arrives from the
bag; **88 moving obstacles** are spawned at the safety boundary and move through the
scene. The geofence must match the static structure and flag the moving obstacles.

## Run

```bash
docker compose -f fiware/docker-compose.yaml -p rises-fiware down -v
docker compose -f fiware/docker-compose.yaml -p rises-fiware up -d
python3 -m orchestration.cli -c orchestration/configs/andros_moving_obstacles_smoke.yaml
```

To run the **occupancy-grid backend** instead of the spatial one:
```bash
python3 -m orchestration.cli -c orchestration/configs/andros_gridmap_verify.yaml
```

## What to observe

1. **Live, in TimescaleDB** (while running): `obstacle_report` rows accumulate;
   `obstacle_alert` flips true when a moving obstacle is in the safety circle.
2. **Grafana** (`http://localhost:3000`): the AGV entity's obstacle/alert state.
3. **KPIs, on shutdown** (`logs/kpis_*.csv`): the headline evidence.

### Expected KPI result (spatial backend)

| KPI | Value |
|---|---|
| Static Structure Match Rate | 1.000 (all known map structure matched) |
| Detection Ratio | 0.966 (85/88 spawned intruders detected) |
| Detection Latency (avg) | 35.2 ms (mean per-scan, 40 Hz operating point) |
| Miss Rate | 3.4 % |

Interpretation: the geofence matches **100 %** of the genuine static structure and
detects **~97 %** of intruders (the geofence physically saw all 88; the residual is
the validator's conservative association margin). The aggregate "LiDAR Map Match
Rate (all returns)" ≈ 0.63 is *intentionally* below 1 — it includes the moving
intruders, which correctly do **not** match the map.

## Sweeping / tuning

Detection parameters are env-overridable (wired in `params_rosbag.yaml`), e.g.
`GEOFENCE_CORRESPONDENCE_TOLERANCE`, `GEOFENCE_MIN_SEGMENT_POINTS`,
`GEOFENCE_OUTLIER_FILTER_DISTANCE`, `GEOFENCE_SAFETY_RADIUS`. An empirical sweep
showed the shipped defaults are already at a flat optimum for this scenario
(lowering tolerances does not improve detection and can raise false stops) — so they
are the recommended starting point.
