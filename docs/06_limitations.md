# Known limitations & boundaries

Stated explicitly per ARISE D4 expectations — hidden assumptions are not
acceptable; documented limitations are.

## Functional / scope

- **2-D only.** Detection is planar (safety-circle + 2-D map matching). Height
  awareness parameters exist but are not enforced in the matching path.
- **Map-relative.** The geofence flags whatever is *not in the loaded map*. It needs
  a reasonably faithful warehouse map (obstacles + boundary). Persistent real
  structure absent from the map will read as a (correct, but undesired) "novel"
  obstacle.
- **Validated on one recorded scenario.** Numbers (Static Match 1.000, Detection
  0.966) are from the 300 s "andros" warehouse bag with synthetic spawned obstacles.
  Other environments/sensors are *expected* compatible but untested.
- **Detection is "is it on the map", not classification.** The geofence does not
  itself classify obstacles as humans; human-leg detection is delegated to the
  optional `rises_leg_filter`.

## Interface maturity

- **ROS4HRI alignment is partial.** `rises_leg_filter` uses the `/humans/bodies/`
  namespace and TF body-frame convention, but body-ID lists are `std_msgs/String`
  placeholders pending migration to `hri_msgs/IdsList`; camera fusion is disabled
  by default. Future work: full `hri_msgs` adoption.
- **DDS-Enabler is the secondary path.** A DDS-Enabler config is provided for direct
  DDS→NGSI-LD, but the default production write path is the ROS→Orion bridge node.
- **NGSI-LD uses the core context** (no custom `@context`); compound attributes are
  capped/chunked to stay under the TROE ~2 KB limit (e.g. obstacle segments capped
  at 12 each), so the broker view is a bounded sample, not every segment.

## Validation-metric caveats

- The headline **Detection Ratio (0.966)** is bounded by the validator's
  spawn↔detection association margin (`match_tolerance` 2.5 m / `match_window` 3.0 s),
  chosen to keep all latencies within the 2 s target; the geofence physically saw
  all 88 spawns, so true detection is ~100 %.
- **Static Structure Match Rate (1.000)** counts a "static miss" only if a segment
  is persistent **and** never near a ground-truth spawn; it could in principle miss
  a static gap within 2.5 m of a spawn point (spawns are in open areas, so this is
  negligible here).
- **False Stop Rate** is statistically noisy at the low alert counts in a 300 s run.

## Engineering debt (documented, non-blocking)

- The spatial and gridmap nodes share ~70–95 % of their scaffolding but are
  separate implementations; a shared base/library is planned (architecture roadmap).
- Build currently depends on apt mirrors at image-build time (mitigation: an overlay
  rebuild that re-runs only `colcon`).

## Out of open scope (proprietary / demonstrator-specific)

- Physical AGV drivers, the Unity/MQTT scenario source, and the full fleet
  controller — represented in the open module by the recorded bag and the
  orchestration harness rather than the real hardware.
