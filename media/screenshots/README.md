# Screenshots — capture guide (visual evidence for the basic demo)

These PNGs are the "visual evidence" required by D4 §3.2.8 / §3.3.9. They must be
captured from a real run — do not use mock-ups. Run the basic demo first:

```bash
docker compose -f fiware/docker-compose.yaml -p rises-fiware down -v
docker compose -f fiware/docker-compose.yaml -p rises-fiware up -d
python3 -m orchestration.cli -c orchestration/configs/andros_moving_obstacles_smoke.yaml
```

Capture and commit the following into this folder:

| File | What to capture | Where |
|---|---|---|
| `grafana_obstacle_alert.png` | The AGV dashboard while a moving intruder is in the safety circle (obstacle_alert = true) and the obstacle_report panel | Grafana, http://localhost:3000 (admin/admin) |
| `grafana_kpi_overview.png` | The KPI / validation_result panel over the 300 s window | Grafana |
| `rviz_matched_unmatched.png` | RViz showing the map, safety circle, matched (green) vs unmatched (red) segments | RViz on the geofence markers |
| `kpis_csv.png` (optional) | The exported `logs/kpis_*.csv` opened, showing Static Structure Match Rate 1.000 / Detection Ratio 0.966 | terminal / spreadsheet |

Note: the KPI numbers must come from the CURRENT corrected export
(`scripts/export_kpis.py`, per-population metrics). Do not use the stale
`logs/kpis_20260601_*.csv` files — those predate the KPI methodology fix and show
the old aggregate values (Detection ~0.48), which contradict the deliverable.
