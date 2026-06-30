# Error Segment Tracking Across Consecutive Scans

## Overview

The geofence node classifies every scan point as *matched* (corresponds to a
known map obstacle) or *unmatched* (unknown obstacle). Both categories are then
grouped and simplified into LINE segments. The **ErrorSegmentTracker** assigns a
persistent integer ID to each *unmatched* segment so that downstream consumers
can correlate the same physical obstacle across consecutive lidar scans.

## Pipeline

```
LiDAR scan
    |
    v
LaserScan Preprocessor  (clusters raw points into ObstacleArray,
                          forwards angle_increment from LaserScan)
    |
    v
GeofenceNode / BatchPointChecker  (classifies each point as matched/unmatched)
    |
    v
ObstacleReportBuilder
    |-- emitSegments()        groups points, fits LINE segments (Douglas-Peucker)
    |-- ErrorSegmentTracker   assigns persistent IDs to unmatched segments
    |
    v
ObstacleReport  (published on ROS topic, optionally in robot local frame)
```

## Segmentation (ObstacleReportBuilder)

Both matched and unmatched vertices arrive in scan order. Segmentation is a
two-phase process that outputs exclusively LINE-type obstacles.

### Phase 1: Distance-Based Grouping

Points are split into groups whenever the Euclidean distance between consecutive
points exceeds a gap threshold derived from the lidar's angular resolution:

```
gap_threshold = max(segment_min_gap, segment_gap_multiplier * range * angle_increment)
```

`angle_increment` is read from the `ObstacleArray` message (set by the
laserscan preprocessor from `LaserScan::angle_increment`). If the message field
is zero (older publisher), the `lidar_angle_increment` ROS parameter is used as
fallback.

The threshold tracks the actual point spacing at each range. A "gap" is defined
as a distance between consecutive points that is `segment_gap_multiplier` times
larger than expected for a continuous surface. The `segment_min_gap` floor
prevents false splits from close-range noise.

For a 360 deg lidar with 0.1 deg resolution (default multiplier 8):

| Range | Point spacing | Gap threshold | Separation needed |
|---|---|---|---|
| 1 m | 1.7 mm | 0.05 m (clamped) | 5 cm |
| 5 m | 8.7 mm | 0.07 m | 7 cm |
| 10 m | 17.5 mm | 0.14 m | 14 cm |
| 20 m | 35 mm | 0.28 m | 28 cm |

Groups with fewer than `min_segment_points` vertices are discarded as noise.

### Phase 2: Douglas-Peucker Line Simplification

Each surviving group is converted into the minimal set of LINE segments that
approximate the shape within `line_fit_tolerance`. The algorithm is a
non-recursive (stack-based) implementation of Douglas-Peucker:

```
function simplify(points, first, last, tolerance):
    if last - first <= 1:
        emit LINE(points[first], points[last])
        return

    find k in (first+1 .. last-1) with max perpendicular distance
         from the line points[first] -> points[last]

    if max_distance <= tolerance:
        emit LINE(points[first], points[last])     # all points fit one line
    else:
        simplify(points, first, k, tolerance)       # left half
        simplify(points, k, last, tolerance)         # right half
```

This produces exactly the right number of lines for any shape:

| Shape | Points | Lines emitted |
|---|---|---|
| Straight wall | 20 | 1 |
| L-shaped corner | 20 (10+10) | 2 |
| U-shaped alcove | 30 (10+10+10) | 3 |
| Gentle curve | 15 | 3-5 (depends on curvature) |
| Single point | 1 | 1 (degenerate: start == end) |

The `line_fit_tolerance` default of 0.05 m (5 cm) means intermediate points
must deviate less than 5 cm from the first-to-last line to be collapsed into a
single line segment. This is tight enough to preserve wall corners but loose
enough to absorb scan noise.

**Degenerate cases:**
- **1 point**: emitted as a LINE with identical start and end coordinates.
- **2 points**: emitted as a single LINE directly.
- **Coincident endpoints** (first and last point are the same): the algorithm
  picks the farthest intermediate point and splits there.

### Output Per Segment

Every segment becomes an `Obstacle` message with:
- `type` = LINE (always)
- `position` = midpoint of the line's two endpoints
- `vertices[0]` = start point, `vertices[1]` = end point

## Persistent ID Assignment (ErrorSegmentTracker)

### Data Structure

A flat `std::vector<Entry>` sorted by `(cell_x, cell_y)` grid coordinates.
Two buffers are swapped each cycle (double-buffer pattern):

- `previous_`: entries from the last completed scan cycle (read-only during matching)
- `current_`: entries being built for the current cycle (write-only)

### Lifecycle

```
beginScanCycle()
    clear current_ buffer, reserve space, clear claimed ID set

for each unmatched segment:
    getOrAssignId(centroid_x, centroid_y, aabb)
        -> returns persistent ID (reused or newly minted)

endScanCycle()
    sort current_ by grid cell
    swap current_ <-> previous_
    (entries not seen this cycle are implicitly evicted)
```

### Matching Algorithm

Given a new segment with centroid `(cx, cy)` and bounding box `aabb`:

1. **Grid search region**: Compute the grid cells covered by both the centroid
   neighbourhood (+-1 cell) and the AABB extent (+-1 cell). The search region
   is the union of both, ensuring large segments that span multiple cells are
   matched even when the centroid shifts.

2. **Candidate lookup**: Binary search into `previous_` (sorted by cell_x,
   cell_y) to find entries within the search columns and row range.

3. **Filtering**:
   - Skip entries whose ID was already claimed this cycle (prevents two
     current segments from getting the same ID).
   - Skip entries where the current centroid has drifted beyond `max_drift`
     from the entry's **origin** (first-detection position). This prevents a
     slowly moving obstacle from retaining the same ID indefinitely.

4. **Scoring** (two complementary metrics, best-of):

   a. **Centroid proximity**: `score = 1 - (dist^2 / max_dist^2)` where
      `max_dist = cell_size * 1.5`. Range: 0 to 1. Rewards segments whose
      centroids haven't moved much.

   b. **AABB overlap (IoMin)**: `overlap_area / min(area_prev, area_curr)`.
      Mapped to range 0.5 to 1.0. Handles the case where a segment shifts by
      one scan point on each end — the centroid barely moves but the AABB
      largely overlaps. IoMin (intersection over minimum) is used instead of
      IoU because a segment gaining or losing an endpoint makes the union grow
      while the overlap stays the same, which would unfairly penalize IoU.

   The final score is `max(centroid_score, aabb_score)`. The candidate with
   the highest score wins.

5. **ID assignment**: If a match is found, the previous ID is reused. If no
   match (new obstacle), a fresh sequential ID is minted (`next_id_++`).

### Origin Anchoring

Each entry stores two positions:
- `cx, cy, aabb` — the **latest** observation (used for frame-to-frame matching)
- `origin_cx, origin_cy, origin_aabb` — the **first detection** (used for drift check)

When a segment matches an existing entry, the origin is carried forward
unchanged. When a new segment appears, its current position becomes the origin.

The drift check compares the **current** observation against the **origin**:
```
drift = sqrt((cx - origin_cx)^2 + (cy - origin_cy)^2)
if drift > max_drift:  reject match, assign new ID
```

This means a stationary obstacle that fluctuates slightly (due to scan noise)
keeps its ID, but an obstacle that gradually moves across the floor gets a new
ID once it has moved more than `max_drift` meters from where it first appeared.

### Shifted Segment Handling

A common scenario: scan N produces a line segment from points A-B-C-D-E.
Scan N+1 produces a line from points B-C-D-E-F (lost point A, gained point F).

- The centroid shifts slightly (from mean(A..E) to mean(B..F)).
- The AABB largely overlaps (only the endpoints changed).

The AABB overlap score handles this correctly because IoMin measures how much
of the smaller box is covered by the intersection. Even though the segment
shifted, the overlap ratio stays high, so the same ID is assigned.

### Eviction

Entries that are not seen (not matched by any current segment) during a scan
cycle are implicitly evicted when `previous_` is replaced by `current_` at
`endScanCycle()`. There is no explicit timeout or age counter — if an obstacle
disappears from the scan for even one cycle, its ID is freed.

## Configuration

| ROS Parameter | Default | Description |
|---|---|---|
| `enable_error_segment_tracking` | `true` | Enable/disable persistent ID assignment |
| `error_segment_tracker_cell_size` | `0.3` | Grid cell size in meters for spatial lookup |
| `error_segment_tracker_max_drift` | `1.0` | Max drift from origin before new ID (meters, 0 = unlimited) |
| `min_segment_points` | `3` | Minimum points per segment (noise filter) |
| `segment_min_gap` | `0.05` | Minimum gap to split regardless of range (meters) |
| `segment_gap_multiplier` | `8.0` | How many point spacings constitute a gap |
| `lidar_angle_increment` | `0.001745` | Fallback angular resolution (radians, 0.1 deg). Overridden by `ObstacleArray.angle_increment` when available. |
| `line_fit_tolerance` | `0.05` | Douglas-Peucker tolerance: max perpendicular deviation before splitting into sub-lines (meters) |
| `report_error_segments_as_points` | `false` | Emit raw point clouds instead of segmented lines |
| `publish_report_in_local_frame` | `false` | Transform report to robot base_link frame before publishing |

All parameters are overridable via environment variables (see `params_default.yaml`).

## Output

The `ObstacleReport` message contains:
- `matched_obstacles[]` — LINE segments corresponding to known map obstacles (sequential IDs)
- `unmatched_obstacles[]` — LINE segments for unknown obstacles (persistent tracked IDs)
- `header` — timestamp and frame_id (map or base_link depending on config)

Each obstacle entry has `id`, `type` (LINE), `position` (midpoint of the line),
and `vertices` (exactly 2 points: start and end of the line segment).
