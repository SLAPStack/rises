#!/usr/bin/env python3
"""
KPI CSV exporter for ARISE geofencing system.

Connects to TimescaleDB, runs each KPI query, and writes a timestamped CSV
to logs/kpis_YYYYMMDD_HHMMSS.csv.

Usage:
    python3 scripts/export_kpis.py [--host HOST] [--port PORT] [--db DB]
                                   [--user USER] [--password PASSWORD]
                                   [--window-hours N] [--output PATH]
"""

import argparse
import csv
import sys
from datetime import datetime, timezone
from pathlib import Path

try:
    import psycopg2
    import psycopg2.extras
except ImportError:
    print(
        "ERROR: psycopg2 not installed. Run: pip install psycopg2-binary",
        file=sys.stderr,
    )
    sys.exit(1)


# ---------------------------------------------------------------------------
# KPI definitions
# Each entry is:
#   name        – human-readable KPI name
#   target      – target value string (or "" if N/A)
#   target_note – description of what the target means
#   sql         – query that returns exactly ONE numeric column called "value"
#                 Uses %(window)s for the time window interval string.
# ---------------------------------------------------------------------------
KPI_QUERIES = [
    {
        "name": "False Stop Rate",
        "unit": "%",
        "target": "<10",
        "target_note": "Less than 10% of alerts are transient (false stops)",
        "sql": """
            WITH events AS (
                SELECT ts, boolean as alert,
                       LAG(boolean)  OVER (ORDER BY ts) as prev,
                       LEAD(boolean) OVER (ORDER BY ts) as next
                FROM attributes
                WHERE id LIKE '%%obstacle_alert'
                  AND ts > now() - interval %(window)s
            ),
            starts AS (
                SELECT * FROM events
                WHERE alert = true AND (prev IS NULL OR prev = false)
            ),
            transient AS (
                SELECT * FROM starts WHERE next = false
            )
            SELECT
                CASE WHEN (SELECT COUNT(*) FROM starts) > 0
                     THEN 100.0 * (SELECT COUNT(*) FROM transient)
                              / (SELECT COUNT(*) FROM starts)
                     ELSE NULL
                END as value
        """,
    },
    {
        "name": "System Uptime",
        "unit": "%",
        "target": ">99",
        "target_note": "Geofence node ready fraction of monitored window",
        "sql": """
            WITH events AS (
                SELECT ts, boolean as ready,
                       LEAD(ts) OVER (ORDER BY ts) as next_ts
                FROM attributes
                WHERE id LIKE '%%geofence_ready'
                  AND ts > now() - interval %(window)s
            ),
            durations AS (
                SELECT ready,
                       EXTRACT(EPOCH FROM
                           (COALESCE(next_ts, now()) - ts)) as dur
                FROM events
            )
            SELECT
                CASE WHEN SUM(dur) > 0
                     THEN 100.0 * SUM(CASE WHEN ready THEN dur ELSE 0 END)
                              / SUM(dur)
                     ELSE NULL
                END as value
            FROM durations
        """,
    },
    {
        "name": "Static Structure Match Rate",
        "unit": "ratio",
        "target": ">0.98",
        "target_note": (
            "Of LiDAR returns from genuine static structure, the fraction "
            "matched = matched / (matched + static-miss). A static miss is a "
            "persistent unmatched segment (tracker id recurs) that is never near "
            "a ground-truth spawn -- i.e. real structure the map lacks. Moving "
            "intruders (transient ids) and stationary intruders (near their "
            "spawn) are excluded, isolating known-structure match quality. "
            "From the validation node."
        ),
        "sql": """
            SELECT
                CASE WHEN ((compound->'stats'->>'matched_segments_total')::float
                         + (compound->'stats'->>'static_miss_segments')::float) > 0
                     THEN (compound->'stats'->>'matched_segments_total')::float
                          / ((compound->'stats'->>'matched_segments_total')::float
                           + (compound->'stats'->>'static_miss_segments')::float)
                     ELSE NULL
                END as value
            FROM attributes
            WHERE id LIKE '%%validation_result'
              AND compound IS NOT NULL
            ORDER BY ts DESC
            LIMIT 1
        """,
    },
    {
        "name": "LiDAR Map Match Rate (all returns)",
        "unit": "ratio",
        "target": "",
        "target_note": (
            "Fraction of ALL in-zone LiDAR segments matched to the map, pooled "
            "over the window. Mixes populations: it is intentionally < 1 because "
            "moving intruders correctly do not match. For known-obstacle match "
            "quality see Static Structure Match Rate; for intruder detection see "
            "Detection Ratio."
        ),
        "sql": """
            SELECT
                SUM((compound->>'matched_count')::float)
                / NULLIF(SUM((compound->>'matched_count')::float
                           + (compound->>'unmatched_count')::float), 0)
                as value
            FROM attributes
            WHERE id LIKE '%%obstacle_report'
              AND ts > now() - interval %(window)s
              AND compound IS NOT NULL
        """,
    },
    {
        "name": "Detection Ratio (spawned vs detected)",
        "unit": "ratio",
        "target": ">0.95",
        "target_note": "Fraction of spawned obstacles detected by geofence (from validation node)",
        "sql": """
            SELECT
                CASE WHEN (compound->'stats'->>'spawned')::float > 0
                     THEN (compound->'stats'->>'detected')::float
                          / (compound->'stats'->>'spawned')::float
                     ELSE NULL
                END as value
            FROM attributes
            WHERE id LIKE '%%validation_result'
              AND compound IS NOT NULL
            ORDER BY ts DESC
            LIMIT 1
        """,
    },
    {
        "name": "Miss Rate (spawned, not detected)",
        "unit": "%",
        "target": "<10",
        "target_note": "Percentage of spawned obstacles the geofence failed to detect (from validation node)",
        "sql": """
            SELECT
                CASE WHEN (compound->'stats'->>'spawned')::float > 0
                     THEN 100.0
                          * ((compound->'stats'->>'spawned')::float
                             - (compound->'stats'->>'detected')::float)
                          / (compound->'stats'->>'spawned')::float
                     ELSE NULL
                END as value
            FROM attributes
            WHERE id LIKE '%%validation_result'
              AND compound IS NOT NULL
            ORDER BY ts DESC
            LIMIT 1
        """,
    },
    {
        "name": "Detection Latency (avg)",
        "unit": "seconds",
        "target": "<2",
        "target_note": "Average time from an obstacle entering the safety circle to its first geofence detection (signed; negative = detected before entry; matched on the obstacle_report per-segment position within a short window; from validation node)",
        "sql": """
            SELECT (compound->'stats'->>'avg_latency_ms')::float / 1000.0 as value
            FROM attributes
            WHERE id LIKE '%%validation_result'
              AND compound IS NOT NULL
            ORDER BY ts DESC
            LIMIT 1
        """,
    },
    {
        "name": "Total Alerts",
        "unit": "count",
        "target": "",
        "target_note": "Number of distinct obstacle alert events in the window",
        "sql": """
            WITH events AS (
                SELECT ts, boolean as alert,
                       LAG(boolean) OVER (ORDER BY ts) as prev
                FROM attributes
                WHERE id LIKE '%%obstacle_alert'
                  AND ts > now() - interval %(window)s
            )
            SELECT COUNT(*) as value
            FROM events
            WHERE alert = true AND (prev IS NULL OR prev = false)
        """,
    },
    {
        "name": "Mean Time Between Alerts (MTBA)",
        "unit": "seconds",
        "target": "",
        "target_note": "Average time between successive alert-start events",
        "sql": """
            WITH alert_starts AS (
                SELECT ts, LAG(ts) OVER (ORDER BY ts) as prev_ts
                FROM attributes
                WHERE id LIKE '%%obstacle_alert'
                  AND boolean = true
                  AND ts > now() - interval %(window)s
            ),
            intervals AS (
                SELECT EXTRACT(EPOCH FROM (ts - prev_ts)) as gap_sec
                FROM alert_starts
                WHERE prev_ts IS NOT NULL
            )
            SELECT AVG(gap_sec) as value FROM intervals
        """,
    },
    {
        "name": "Latest Matched Obstacle Count",
        "unit": "count",
        "target": "",
        "target_note": "Number of matched LiDAR segments in the most recent report",
        "sql": """
            SELECT (compound->>'matched_count')::float as value
            FROM attributes
            WHERE id LIKE '%%obstacle_report'
              AND compound IS NOT NULL
            ORDER BY ts DESC
            LIMIT 1
        """,
    },
    {
        "name": "Latest Unmatched Obstacle Count",
        "unit": "count",
        "target": "",
        "target_note": "Number of unmatched LiDAR segments in the most recent report",
        "sql": """
            SELECT (compound->>'unmatched_count')::float as value
            FROM attributes
            WHERE id LIKE '%%obstacle_report'
              AND compound IS NOT NULL
            ORDER BY ts DESC
            LIMIT 1
        """,
    },
    {
        "name": "Average Alert Latency",
        "unit": "ms",
        "target": "<5",
        "target_note": (
            "Rolling-average scan-in -> alert-out processing time (geofence's own "
            "contribution), aggregated over the diagnostic_updater's 1 Hz window "
            "and reset every tick. Requires orion_bridge.py's diagnostics "
            "callback to merge per-node status into the shared attribute "
            "(rather than replace it) so geofence_node's reading survives "
            "alongside other nodes publishing on the same /diagnostics topic."
        ),
        "sql": """
            SELECT COALESCE(
                (compound->'geofence_node'->'values'->>'avg_alert_latency_ms')::float,
                (compound->'geofence_gridmap_node'->'values'->>'avg_alert_latency_ms')::float
            ) as value
            FROM attributes
            WHERE id LIKE '%%diagnostics' AND compound IS NOT NULL
            ORDER BY ts DESC
            LIMIT 1
        """,
    },
]


def run_kpi_query(cur, kpi: dict, window: str) -> tuple[float | None, str]:
    """Execute one KPI query. Returns (value_or_None, error_or_empty)."""
    try:
        cur.execute(kpi["sql"], {"window": window})
        row = cur.fetchone()
        if row is None or row[0] is None:
            return None, "no data"
        return float(row[0]), ""
    except Exception as exc:
        return None, str(exc)


def export_kpis(
    host: str = "localhost",
    port: int = 5432,
    db: str = "orion",
    user: str = "postgres",
    password: str = "postgres",
    window_hours: int = 24,
    output_path: str | None = None,
) -> str:
    """
    Query TimescaleDB and write KPI CSV.

    Returns the path of the written CSV file.
    """
    window = f"{window_hours} hours"
    now = datetime.now(tz=timezone.utc)
    timestamp_str = now.strftime("%Y%m%d_%H%M%S")

    if output_path is None:
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)
        output_path = str(logs_dir / f"kpis_{timestamp_str}.csv")

    dsn = f"host={host} port={port} dbname={db} user={user} password={password}"
    conn = psycopg2.connect(dsn)
    conn.autocommit = True

    rows = []
    try:
        with conn.cursor() as cur:
            for kpi in KPI_QUERIES:
                value, error = run_kpi_query(cur, kpi, window)
                rows.append(
                    {
                        "kpi_name": kpi["name"],
                        "value": f"{value:.4f}" if value is not None else "",
                        "unit": kpi["unit"],
                        "target": kpi["target"],
                        "target_note": kpi["target_note"],
                        "window_hours": window_hours,
                        "error": error,
                        "exported_at": now.isoformat(),
                    }
                )
    finally:
        conn.close()

    fieldnames = [
        "kpi_name",
        "value",
        "unit",
        "target",
        "target_note",
        "window_hours",
        "error",
        "exported_at",
    ]
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5432)
    parser.add_argument("--db", default="orion")
    parser.add_argument("--user", default="postgres")
    parser.add_argument("--password", default="postgres")
    parser.add_argument(
        "--window-hours",
        type=int,
        default=24,
        help="Time window for historical KPIs (default: 24)",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output CSV path (default: logs/kpis_TIMESTAMP.csv)",
    )
    args = parser.parse_args()

    try:
        path = export_kpis(
            host=args.host,
            port=args.port,
            db=args.db,
            user=args.user,
            password=args.password,
            window_hours=args.window_hours,
            output_path=args.output,
        )
        print(f"KPI export written to: {path}")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
