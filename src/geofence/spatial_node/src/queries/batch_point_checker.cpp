#include "geofence/spatial/queries/batch_point_checker.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include <cmath>
#include <numeric>
#include <rclcpp/rclcpp.hpp>

#ifdef USE_SIMD
#include "geofence/spatial/common/simd_types.hpp"
#endif

namespace rises::geofence::query {

// Hard cap on per-call vertex flattening. POINT obstacles can carry thousands
// of vertices each; without an upper bound a single adversarial /
// mis-sized message can drive the thread_local SoA buffers into gigabytes.
// See audit finding #4.
constexpr std::size_t kMaxTotalVerts = 500'000;

namespace {

struct ContourEdge {
  float x1, y1, x2, y2;
};

// Build a flat edge list from all warehouse boundary contours.
std::vector<ContourEdge>
extractContourEdges(const GeofenceMap &map) {
  std::vector<ContourEdge> edges;
  const rises::shape::MapBoundaryContours *contours = map.getMapContours();
  if (!contours)
    return edges;

  const auto addRingEdges = [&](const std::vector<Point2D> &verts) {
    const std::size_t n = verts.size();
    if (n < 2)
      return;
    for (std::size_t i = 0; i + 1 < n; ++i) {
      edges.push_back({static_cast<float>(verts[i].x),
                       static_cast<float>(verts[i].y),
                       static_cast<float>(verts[i + 1].x),
                       static_cast<float>(verts[i + 1].y)});
    }
    // Closing edge (ring wraps back to first vertex)
    edges.push_back(
        {static_cast<float>(verts[n - 1].x), static_cast<float>(verts[n - 1].y),
         static_cast<float>(verts[0].x), static_cast<float>(verts[0].y)});
  };

  const auto &outer_verts = contours->getOuterContour().getVertices();
  edges.reserve(outer_verts.size() + 20);
  addRingEdges(outer_verts);

  for (const rises::shape::PolygonContour &inner :
       contours->getInnerContours()) {
    addRingEdges(inner.getVertices());
  }

  // Outer segments are open line segments (no closing edge)
  for (const rises::shape::LineSegment2D &seg : contours->getOuterSegments()) {
    edges.push_back(
        {static_cast<float>(seg.start.x), static_cast<float>(seg.start.y),
         static_cast<float>(seg.end.x), static_cast<float>(seg.end.y)});
  }
  return edges;
}

// ============================================================================
// Obstacle-major SIMD matching helpers (SoA x[]/y[] input, OR-accumulate)
//
// Each function takes:
//   x_coords/y_coords  - in-zone vertices (SoA float arrays, length count)
//   matched_inout      - uint8_t match flags; function ORs new matches in
//   count              - number of in-zone vertices
// Returns true if at least one NEW match was produced (used for ID tracking).
// ============================================================================

// AABB containment with per-axis tolerance already folded into min/max bounds.
inline bool checkRectContainment_OR(const float *__restrict__ x_coords,
                                    const float *__restrict__ y_coords,
                                    const float min_x, const float min_y,
                                    const float max_x, const float max_y,
                                    uint8_t *__restrict__ matched_inout,
                                    const std::size_t count) {
  bool any_new = false;

#ifdef USE_SIMD
  using simd_batch = rises::geofence::simd::float_batch;
  constexpr std::size_t simd_size = simd_batch::size;
  const std::size_t simd_end = count - (count % simd_size);

  const simd_batch xmin_b(min_x), ymin_b(min_y);
  const simd_batch xmax_b(max_x), ymax_b(max_y);

  for (std::size_t i = 0; i < simd_end; i += simd_size) {
    const simd_batch x = xsimd::load_unaligned(&x_coords[i]);
    const simd_batch y = xsimd::load_unaligned(&y_coords[i]);
    const auto in_rect =
        (x >= xmin_b) & (x <= xmax_b) & (y >= ymin_b) & (y <= ymax_b);

    alignas(rises::geofence::simd::simd_alignment) bool lane[simd_size];
    in_rect.store_aligned(lane);

    for (std::size_t j = 0; j < simd_size; ++j) {
      if (lane[j] && !matched_inout[i + j]) {
        matched_inout[i + j] = 1u;
        any_new = true;
      }
    }
  }

  for (std::size_t i = simd_end; i < count; ++i) {
    if (!matched_inout[i] && x_coords[i] >= min_x && x_coords[i] <= max_x &&
        y_coords[i] >= min_y && y_coords[i] <= max_y) {
      matched_inout[i] = 1u;
      any_new = true;
    }
  }
#else
  for (std::size_t i = 0; i < count; ++i) {
    if (!matched_inout[i] && x_coords[i] >= min_x && x_coords[i] <= max_x &&
        y_coords[i] >= min_y && y_coords[i] <= max_y) {
      matched_inout[i] = 1u;
      any_new = true;
    }
  }
#endif
  return any_new;
}

// Parametric closest-point distance² to a line segment.  Pre-pass: caller
// computes dx_e/dy_e and inv_len_sq once and passes them in.
inline bool checkLineProximity_OR(
    const float *__restrict__ x_coords, const float *__restrict__ y_coords,
    const float ax, const float ay, const float dx_e, const float dy_e,
    const float inv_len_sq, const float tolerance_sq,
    uint8_t *__restrict__ matched_inout, const std::size_t count) {
  bool any_new = false;

#ifdef USE_SIMD
  using simd_batch = rises::geofence::simd::float_batch;
  constexpr std::size_t simd_size = simd_batch::size;
  const std::size_t simd_end = count - (count % simd_size);

  const simd_batch ax_b(ax), ay_b(ay);
  const simd_batch dxe_b(dx_e), dye_b(dy_e);
  const simd_batch inv_b(inv_len_sq);
  const simd_batch zero_b(0.0f), one_b(1.0f);
  const simd_batch tol_sq_b(tolerance_sq);

  for (std::size_t i = 0; i < simd_end; i += simd_size) {
    const simd_batch px = xsimd::load_unaligned(&x_coords[i]);
    const simd_batch py = xsimd::load_unaligned(&y_coords[i]);

    const simd_batch t_raw =
        (xsimd::fma(px - ax_b, dxe_b, (py - ay_b) * dye_b)) * inv_b;
    const simd_batch t = xsimd::min(xsimd::max(t_raw, zero_b), one_b);

    const simd_batch cx = xsimd::fma(t, dxe_b, ax_b);
    const simd_batch cy = xsimd::fma(t, dye_b, ay_b);
    const simd_batch ddx = px - cx;
    const simd_batch ddy = py - cy;
    const simd_batch dist_sq = xsimd::fma(ddx, ddx, ddy * ddy);

    const auto near = dist_sq <= tol_sq_b;

    alignas(rises::geofence::simd::simd_alignment) bool lane[simd_size];
    near.store_aligned(lane);

    for (std::size_t j = 0; j < simd_size; ++j) {
      if (lane[j] && !matched_inout[i + j]) {
        matched_inout[i + j] = 1u;
        any_new = true;
      }
    }
  }

  for (std::size_t i = simd_end; i < count; ++i) {
    if (!matched_inout[i]) {
      const float t = std::clamp(
          ((x_coords[i] - ax) * dx_e + (y_coords[i] - ay) * dy_e) * inv_len_sq,
          0.0f, 1.0f);
      const float cx = ax + t * dx_e;
      const float cy = ay + t * dy_e;
      const float ddx = x_coords[i] - cx;
      const float ddy = y_coords[i] - cy;
      if (ddx * ddx + ddy * ddy <= tolerance_sq) {
        matched_inout[i] = 1u;
        any_new = true;
      }
    }
  }
#else
  for (std::size_t i = 0; i < count; ++i) {
    if (!matched_inout[i]) {
      const float t = std::clamp(
          ((x_coords[i] - ax) * dx_e + (y_coords[i] - ay) * dy_e) * inv_len_sq,
          0.0f, 1.0f);
      const float cx = ax + t * dx_e;
      const float cy = ay + t * dy_e;
      const float ddx = x_coords[i] - cx;
      const float ddy = y_coords[i] - cy;
      if (ddx * ddx + ddy * ddy <= tolerance_sq) {
        matched_inout[i] = 1u;
        any_new = true;
      }
    }
  }
#endif
  return any_new;
}

// Squared-distance proximity to a single point obstacle.
inline bool checkPointProximity_OR(const float *__restrict__ x_coords,
                                   const float *__restrict__ y_coords,
                                   const float px, const float py,
                                   const float tolerance_sq,
                                   uint8_t *__restrict__ matched_inout,
                                   const std::size_t count) {
  bool any_new = false;

#ifdef USE_SIMD
  using simd_batch = rises::geofence::simd::float_batch;
  constexpr std::size_t simd_size = simd_batch::size;
  const std::size_t simd_end = count - (count % simd_size);

  const simd_batch px_b(px), py_b(py);
  const simd_batch tol_sq_b(tolerance_sq);

  for (std::size_t i = 0; i < simd_end; i += simd_size) {
    const simd_batch x = xsimd::load_unaligned(&x_coords[i]);
    const simd_batch y = xsimd::load_unaligned(&y_coords[i]);
    const simd_batch dx = x - px_b;
    const simd_batch dy = y - py_b;
    const simd_batch dist_sq = xsimd::fma(dx, dx, dy * dy);

    const auto in_range = dist_sq <= tol_sq_b;

    alignas(rises::geofence::simd::simd_alignment) bool lane[simd_size];
    in_range.store_aligned(lane);

    for (std::size_t j = 0; j < simd_size; ++j) {
      if (lane[j] && !matched_inout[i + j]) {
        matched_inout[i + j] = 1u;
        any_new = true;
      }
    }
  }

  for (std::size_t i = simd_end; i < count; ++i) {
    if (!matched_inout[i]) {
      const float dx = x_coords[i] - px;
      const float dy = y_coords[i] - py;
      if (dx * dx + dy * dy <= tolerance_sq) {
        matched_inout[i] = 1u;
        any_new = true;
      }
    }
  }
#else
  for (std::size_t i = 0; i < count; ++i) {
    if (!matched_inout[i]) {
      const float dx = x_coords[i] - px;
      const float dy = y_coords[i] - py;
      if (dx * dx + dy * dy <= tolerance_sq) {
        matched_inout[i] = 1u;
        any_new = true;
      }
    }
  }
#endif
  return any_new;
}

// Iterates all contour edges, calling checkLineProximity_OR for each.
// Warehouse walls have no IDs so no return value is needed.
inline void checkContourEdges_OR(const float *__restrict__ x_coords,
                                 const float *__restrict__ y_coords,
                                 const std::vector<ContourEdge> &edges,
                                 const float tolerance_sq,
                                 uint8_t *__restrict__ matched_inout,
                                 const std::size_t count) {
  for (const ContourEdge &edge : edges) {
    const float dx_e = edge.x2 - edge.x1;
    const float dy_e = edge.y2 - edge.y1;
    const float len_sq = dx_e * dx_e + dy_e * dy_e;
    if (len_sq < 1e-12f)
      continue;
    checkLineProximity_OR(x_coords, y_coords, edge.x1, edge.y1, dx_e, dy_e,
                          1.0f / len_sq, tolerance_sq, matched_inout, count);
  }
}

// Winding number point-in-polygon test.
// Faster than boost::geometry::within for simple 2D polygons — no template
// overhead, no multi-geometry dispatch. Handles concave polygons correctly.
// Returns true if point (px, py) is inside the polygon ring.
inline bool windingNumberContains(const float px, const float py,
                                  const rises::geofence::Polygon &polygon) {
  const auto &ring = boost::geometry::exterior_ring(polygon);
  const std::size_t n = ring.size();
  if (n < 3)
    return false;

  int winding = 0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const float yi = static_cast<float>(ring[i].y());
    const float yj = static_cast<float>(ring[j].y());

    if (yj <= py) {
      if (yi > py) {
        // Upward crossing — check if point is left of edge
        const float xi = static_cast<float>(ring[i].x());
        const float xj = static_cast<float>(ring[j].x());
        const float cross = (xj - px) * (yi - py) - (xi - px) * (yj - py);
        if (cross > 0.0f)
          ++winding;
      }
    } else {
      if (yi <= py) {
        // Downward crossing — check if point is right of edge
        const float xi = static_cast<float>(ring[i].x());
        const float xj = static_cast<float>(ring[j].x());
        const float cross = (xj - px) * (yi - py) - (xi - px) * (yj - py);
        if (cross < 0.0f)
          --winding;
      }
    }
  }
  return winding != 0;
}

// AABB pre-filter + winding number for polygon containment.
// First checks bounding box (cheap), then winding number only for vertices
// inside AABB. Tracks which map obstacle ID each vertex matched against via
// matched_id_out.
inline bool checkPolygonContainment_OR(
    const float *__restrict__ x_coords, const float *__restrict__ y_coords,
    const rises::geofence::Polygon &polygon, const float bbox_min_x,
    const float bbox_min_y, const float bbox_max_x, const float bbox_max_y,
    uint8_t *__restrict__ matched_inout, int64_t *__restrict__ matched_id_out,
    const int64_t map_obstacle_id, const std::size_t count) {
  bool any_new = false;

#ifdef USE_SIMD
  // SIMD AABB pre-filter — reject most vertices cheaply in batches
  using simd_batch = rises::geofence::simd::float_batch;
  constexpr std::size_t simd_size = simd_batch::size;
  const std::size_t simd_end = count - (count % simd_size);

  const simd_batch xmin_b(bbox_min_x), ymin_b(bbox_min_y);
  const simd_batch xmax_b(bbox_max_x), ymax_b(bbox_max_y);

  for (std::size_t i = 0; i < simd_end; i += simd_size) {
    const simd_batch x = xsimd::load_unaligned(&x_coords[i]);
    const simd_batch y = xsimd::load_unaligned(&y_coords[i]);
    const auto in_aabb =
        (x >= xmin_b) & (x <= xmax_b) & (y >= ymin_b) & (y <= ymax_b);

    alignas(rises::geofence::simd::simd_alignment) bool lane[simd_size];
    in_aabb.store_aligned(lane);

    for (std::size_t j = 0; j < simd_size; ++j) {
      if (lane[j] && !matched_inout[i + j]) {
        if (windingNumberContains(x_coords[i + j], y_coords[i + j], polygon)) {
          matched_inout[i + j] = 1u;
          matched_id_out[i + j] = map_obstacle_id;
          any_new = true;
        }
      }
    }
  }

  for (std::size_t i = simd_end; i < count; ++i) {
    if (!matched_inout[i] && x_coords[i] >= bbox_min_x &&
        x_coords[i] <= bbox_max_x && y_coords[i] >= bbox_min_y &&
        y_coords[i] <= bbox_max_y) {
      if (windingNumberContains(x_coords[i], y_coords[i], polygon)) {
        matched_inout[i] = 1u;
        matched_id_out[i] = map_obstacle_id;
        any_new = true;
      }
    }
  }
#else
  for (std::size_t i = 0; i < count; ++i) {
    if (!matched_inout[i] && x_coords[i] >= bbox_min_x &&
        x_coords[i] <= bbox_max_x && y_coords[i] >= bbox_min_y &&
        y_coords[i] <= bbox_max_y) {
      if (windingNumberContains(x_coords[i], y_coords[i], polygon)) {
        matched_inout[i] = 1u;
        matched_id_out[i] = map_obstacle_id;
        any_new = true;
      }
    }
  }
#endif
  return any_new;
}

} // anonymous namespace

// ============================================================================
// checkObstacles — RobotSafetyProfile variant (primary hot path)
//
// Optimizations over previous version:
//   - thread_local pre-allocated SoA buffers (eliminates ~10 allocations/frame)
//   - AABB pre-filter before polygon winding number test
//   - Winding number replaces boost::geometry::within (no template overhead)
//   - Per-vertex map obstacle ID tracking for downstream segmentation
//   - Warehouse boundary check merged into zone filter (single pass)
// ============================================================================
BatchPointChecker::ObstacleResult BatchPointChecker::checkObstacles(
    const GeofenceMap &map,
    const rises_interfaces::msg::ObstacleArray &obstacles,
    const Point2D &robot_position, const float robot_heading_rad,
    const RobotSafetyProfile &profile, const float tolerance,
    const bool include_dynamic_obstacles) {
  ObstacleResult obs_result;

  if (obstacles.obstacles.empty()) {
    return obs_result;
  }

  const std::size_t num_obstacles = obstacles.obstacles.size();
  obs_result.matched_vertices_per_obstacle.resize(num_obstacles);
  obs_result.unmatched_vertices_per_obstacle.resize(num_obstacles);
  obs_result.matched_vertex_map_ids.resize(num_obstacles);

  // --- Phase 1: Flatten all POINT-type scan vertices into pre-allocated SoA
  // --- thread_local buffers: reused across frames, only clear + resize needed.
  thread_local std::vector<float> flat_x, flat_y;
  thread_local std::vector<std::size_t> flat_obs_idx, flat_vert_idx;
  thread_local std::vector<uint8_t> flat_in_zone;
  thread_local std::vector<float> in_x, in_y;
  thread_local std::vector<std::size_t> in_obs_idx, in_vert_idx;
  thread_local std::vector<uint8_t> matched_mask;
  thread_local std::vector<int64_t> matched_map_id;

  // Count total vertices to size buffers.
  std::size_t total_verts = 0;
  for (const rises_interfaces::msg::Obstacle &obs : obstacles.obstacles) {
    if (obs.type == rises_interfaces::msg::Obstacle::POINT) {
      total_verts += obs.vertices.size();
    }
  }
  if (total_verts == 0)
    return obs_result;

  if (total_verts > kMaxTotalVerts) {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("batch_point_checker"),
                         throttle_clock, 5000,
                         "Vertex flatten cap hit: %zu > %zu — clamping",
                         total_verts, kMaxTotalVerts);
    total_verts = kMaxTotalVerts;
  }

  // Resize (not allocate) thread_local buffers — no allocation after first
  // frame.
  flat_x.resize(total_verts);
  flat_y.resize(total_verts);
  flat_obs_idx.resize(total_verts);
  flat_vert_idx.resize(total_verts);

  std::size_t write = 0;
  for (std::size_t oi = 0; oi < num_obstacles && write < total_verts; ++oi) {
    const rises_interfaces::msg::Obstacle &obs = obstacles.obstacles[oi];
    if (obs.type != rises_interfaces::msg::Obstacle::POINT)
      continue;
    for (std::size_t vi = 0; vi < obs.vertices.size() && write < total_verts;
         ++vi) {
      flat_x[write] = static_cast<float>(obs.vertices[vi].x);
      flat_y[write] = static_cast<float>(obs.vertices[vi].y);
      flat_obs_idx[write] = oi;
      flat_vert_idx[write] = vi;
      ++write;
    }
  }
  total_verts = write;

  // Zone filter on entire flat array — single SIMD pass.
  flat_in_zone.resize(total_verts);
  profile.isInDetectionZoneBatch(flat_x.data(), flat_y.data(),
                                 flat_in_zone.data(), total_verts,
                                 robot_position, robot_heading_rad);

  // Merge warehouse boundary check into zone filter.
  // Vertices outside warehouse bounds are excluded from matching (single pass).
  const rises::shape::MapBoundaryContours *contours = map.getMapContours();
  if (contours) {
    for (std::size_t i = 0; i < total_verts; ++i) {
      if (flat_in_zone[i]) {
        const Point2D pt{static_cast<double>(flat_x[i]),
                         static_cast<double>(flat_y[i])};
        if (!contours->isPointInside(pt)) {
          flat_in_zone[i] = 0u;
        }
      }
    }
  }

  // Compact: collect only in-zone vertices into contiguous arrays.
  in_x.clear();
  in_y.clear();
  in_obs_idx.clear();
  in_vert_idx.clear();

  // Reserve to avoid reallocation (capacity stays from previous frames).
  if (in_x.capacity() < total_verts) {
    in_x.reserve(total_verts);
    in_y.reserve(total_verts);
    in_obs_idx.reserve(total_verts);
    in_vert_idx.reserve(total_verts);
  }

  for (std::size_t i = 0; i < total_verts; ++i) {
    if (flat_in_zone[i]) {
      in_x.push_back(flat_x[i]);
      in_y.push_back(flat_y[i]);
      in_obs_idx.push_back(flat_obs_idx[i]);
      in_vert_idx.push_back(flat_vert_idx[i]);
    }
  }

  const std::size_t n_in = in_x.size();
  if (n_in == 0)
    return obs_result;

  // Match mask + per-vertex map ID tracking (pre-allocated).
  matched_mask.assign(n_in, 0u);
  matched_map_id.assign(n_in, -1);

  RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("batch_point_checker"),
                        *rclcpp::Clock::make_shared(RCL_STEADY_TIME), 5000,
                        "[ZONE FILTER] %zu/%zu total vertices in detection "
                        "zone across all scan obstacles",
                        n_in, total_verts);

  // Thread-local contour edge cache. Invalidated when the map's contour pointer
  // changes.
  thread_local const rises::shape::MapBoundaryContours *t_last_contours =
      nullptr;
  thread_local std::vector<ContourEdge> t_contour_edges;
  const rises::shape::MapBoundaryContours *current_contours =
      map.getMapContours();
  if (current_contours != t_last_contours) {
    t_last_contours = current_contours;
    t_contour_edges = extractContourEdges(map);
  }

  const BoundingBox query_box =
      profile.getSearchBoundingBox(robot_position, robot_heading_rad);
  const float tolerance_sq = tolerance * tolerance;
  const ObstacleLayer relevant_layers =
      include_dynamic_obstacles
          ? (ObstacleLayer::FIXED | ObstacleLayer::STATIC |
             ObstacleLayer::DYNAMIC)
          : (ObstacleLayer::FIXED | ObstacleLayer::STATIC);

  // --- Phase 2: Spatial query — match inline in visitor ---
  // Each map obstacle is SIMD-checked against all in-zone vertices.
  // matched_map_id[i] records which map obstacle ID matched vertex i.
  map.forEachObstacleInRegion(query_box, [&](const GeometryEntry &entry) {
    if (!hasLayer(relevant_layers, entry.layer))
      return;

    const int64_t map_id = entry.id;

    std::visit(
        [&](const auto &geom) {
          using GeomType = std::decay_t<decltype(geom)>;

          if constexpr (std::is_same_v<GeomType, rises::geofence::Rectangle>) {
            const float min_x =
                static_cast<float>(entry.bbox.min.x) - tolerance;
            const float min_y =
                static_cast<float>(entry.bbox.min.y) - tolerance;
            const float max_x =
                static_cast<float>(entry.bbox.max.x) + tolerance;
            const float max_y =
                static_cast<float>(entry.bbox.max.y) + tolerance;

            bool any_new = false;
        // Inline rect check with ID tracking (can't reuse
        // checkRectContainment_OR because we need to write map_id per vertex).
#ifdef USE_SIMD
            using simd_batch = rises::geofence::simd::float_batch;
            constexpr std::size_t simd_size = simd_batch::size;
            const std::size_t simd_end = n_in - (n_in % simd_size);

            const simd_batch xmin_b(min_x), ymin_b(min_y);
            const simd_batch xmax_b(max_x), ymax_b(max_y);

            for (std::size_t i = 0; i < simd_end; i += simd_size) {
              const simd_batch x = xsimd::load_unaligned(&in_x[i]);
              const simd_batch y = xsimd::load_unaligned(&in_y[i]);
              const auto in_rect =
                  (x >= xmin_b) & (x <= xmax_b) & (y >= ymin_b) & (y <= ymax_b);

              alignas(
                  rises::geofence::simd::simd_alignment) bool lane[simd_size];
              in_rect.store_aligned(lane);

              for (std::size_t j = 0; j < simd_size; ++j) {
                if (lane[j] && !matched_mask[i + j]) {
                  matched_mask[i + j] = 1u;
                  matched_map_id[i + j] = map_id;
                  any_new = true;
                }
              }
            }
            for (std::size_t i = simd_end; i < n_in; ++i) {
              if (!matched_mask[i] && in_x[i] >= min_x && in_x[i] <= max_x &&
                  in_y[i] >= min_y && in_y[i] <= max_y) {
                matched_mask[i] = 1u;
                matched_map_id[i] = map_id;
                any_new = true;
              }
            }
#else
                    for (std::size_t i = 0; i < n_in; ++i) {
                        if (!matched_mask[i] &&
                            in_x[i] >= min_x && in_x[i] <= max_x &&
                            in_y[i] >= min_y && in_y[i] <= max_y) {
                            matched_mask[i] = 1u;
                            matched_map_id[i] = map_id;
                            any_new = true;
                        }
                    }
#endif
            if (any_new)
              obs_result.matched_map_obstacle_ids.insert(map_id);

          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Line>) {
            const float ax = geom.first.x();
            const float ay = geom.first.y();
            const float dx_e = geom.second.x() - ax;
            const float dy_e = geom.second.y() - ay;
            const float len_sq = dx_e * dx_e + dy_e * dy_e;
            if (len_sq < 1e-12f)
              return;
            const float inv_len_sq = 1.0f / len_sq;

            bool any_new = false;
#ifdef USE_SIMD
            using simd_batch = rises::geofence::simd::float_batch;
            constexpr std::size_t simd_size = simd_batch::size;
            const std::size_t simd_end = n_in - (n_in % simd_size);

            const simd_batch ax_b(ax), ay_b(ay);
            const simd_batch dxe_b(dx_e), dye_b(dy_e);
            const simd_batch inv_b(inv_len_sq);
            const simd_batch zero_b(0.0f), one_b(1.0f);
            const simd_batch tol_sq_b(tolerance_sq);

            for (std::size_t i = 0; i < simd_end; i += simd_size) {
              const simd_batch px = xsimd::load_unaligned(&in_x[i]);
              const simd_batch py = xsimd::load_unaligned(&in_y[i]);

              const simd_batch t_raw =
                  (xsimd::fma(px - ax_b, dxe_b, (py - ay_b) * dye_b)) * inv_b;
              const simd_batch t = xsimd::min(xsimd::max(t_raw, zero_b), one_b);

              const simd_batch cx = xsimd::fma(t, dxe_b, ax_b);
              const simd_batch cy = xsimd::fma(t, dye_b, ay_b);
              const simd_batch ddx = px - cx;
              const simd_batch ddy = py - cy;
              const simd_batch dist_sq = xsimd::fma(ddx, ddx, ddy * ddy);

              const auto near = dist_sq <= tol_sq_b;

              alignas(
                  rises::geofence::simd::simd_alignment) bool lane[simd_size];
              near.store_aligned(lane);

              for (std::size_t j = 0; j < simd_size; ++j) {
                if (lane[j] && !matched_mask[i + j]) {
                  matched_mask[i + j] = 1u;
                  matched_map_id[i + j] = map_id;
                  any_new = true;
                }
              }
            }
            for (std::size_t i = simd_end; i < n_in; ++i) {
              if (!matched_mask[i]) {
                const float t =
                    std::clamp(((in_x[i] - ax) * dx_e + (in_y[i] - ay) * dy_e) *
                                   inv_len_sq,
                               0.0f, 1.0f);
                const float cx = ax + t * dx_e;
                const float cy = ay + t * dy_e;
                const float ddx = in_x[i] - cx;
                const float ddy = in_y[i] - cy;
                if (ddx * ddx + ddy * ddy <= tolerance_sq) {
                  matched_mask[i] = 1u;
                  matched_map_id[i] = map_id;
                  any_new = true;
                }
              }
            }
#else
                    for (std::size_t i = 0; i < n_in; ++i) {
                        if (!matched_mask[i]) {
                            const float t = std::clamp(
                                ((in_x[i] - ax) * dx_e + (in_y[i] - ay) * dy_e) * inv_len_sq,
                                0.0f, 1.0f);
                            const float cx = ax + t * dx_e;
                            const float cy = ay + t * dy_e;
                            const float ddx = in_x[i] - cx;
                            const float ddy = in_y[i] - cy;
                            if (ddx * ddx + ddy * ddy <= tolerance_sq) {
                                matched_mask[i] = 1u;
                                matched_map_id[i] = map_id;
                                any_new = true;
                            }
                        }
                    }
#endif
            if (any_new)
              obs_result.matched_map_obstacle_ids.insert(map_id);

          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Point>) {
            const float ox = geom.x();
            const float oy = geom.y();
            bool any_new = false;
#ifdef USE_SIMD
            using simd_batch = rises::geofence::simd::float_batch;
            constexpr std::size_t simd_size = simd_batch::size;
            const std::size_t simd_end = n_in - (n_in % simd_size);

            const simd_batch px_b(ox), py_b(oy);
            const simd_batch tol_sq_b(tolerance_sq);

            for (std::size_t i = 0; i < simd_end; i += simd_size) {
              const simd_batch x = xsimd::load_unaligned(&in_x[i]);
              const simd_batch y = xsimd::load_unaligned(&in_y[i]);
              const simd_batch dx = x - px_b;
              const simd_batch dy = y - py_b;
              const simd_batch dist_sq = xsimd::fma(dx, dx, dy * dy);

              const auto in_range = dist_sq <= tol_sq_b;

              alignas(
                  rises::geofence::simd::simd_alignment) bool lane[simd_size];
              in_range.store_aligned(lane);

              for (std::size_t j = 0; j < simd_size; ++j) {
                if (lane[j] && !matched_mask[i + j]) {
                  matched_mask[i + j] = 1u;
                  matched_map_id[i + j] = map_id;
                  any_new = true;
                }
              }
            }
            for (std::size_t i = simd_end; i < n_in; ++i) {
              if (!matched_mask[i]) {
                const float dx = in_x[i] - ox;
                const float dy = in_y[i] - oy;
                if (dx * dx + dy * dy <= tolerance_sq) {
                  matched_mask[i] = 1u;
                  matched_map_id[i] = map_id;
                  any_new = true;
                }
              }
            }
#else
                    for (std::size_t i = 0; i < n_in; ++i) {
                        if (!matched_mask[i]) {
                            const float dx = in_x[i] - ox;
                            const float dy = in_y[i] - oy;
                            if (dx * dx + dy * dy <= tolerance_sq) {
                                matched_mask[i] = 1u;
                                matched_map_id[i] = map_id;
                                any_new = true;
                            }
                        }
                    }
#endif
            if (any_new)
              obs_result.matched_map_obstacle_ids.insert(map_id);

          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Polygon>) {
            // AABB pre-filter + winding number (replaces
            // boost::geometry::within)
            const bool any_new = checkPolygonContainment_OR(
                in_x.data(), in_y.data(), geom,
                static_cast<float>(entry.bbox.min.x),
                static_cast<float>(entry.bbox.min.y),
                static_cast<float>(entry.bbox.max.x),
                static_cast<float>(entry.bbox.max.y), matched_mask.data(),
                matched_map_id.data(), map_id, n_in);
            if (any_new)
              obs_result.matched_map_obstacle_ids.insert(map_id);
          }
        },
        entry.geometry);
  });

  // Warehouse contour edges (no map obstacle IDs — walls are anonymous).
  if (!t_contour_edges.empty()) {
    checkContourEdges_OR(in_x.data(), in_y.data(), t_contour_edges,
                         tolerance_sq, matched_mask.data(), n_in);
  }

  // --- Phase 3: Re-attribute matched_mask[] back to per-scan-obstacle results
  // ---
  for (std::size_t i = 0; i < n_in; ++i) {
    const std::size_t oi = in_obs_idx[i];
    const std::size_t vi = in_vert_idx[i];
    if (matched_mask[i]) {
      obs_result.matched_vertices_per_obstacle[oi].push_back(vi);
      obs_result.matched_vertex_map_ids[oi].push_back(matched_map_id[i]);
    } else {
      obs_result.unmatched_vertices_per_obstacle[oi].push_back(vi);
    }
  }

  // Classify each scan obstacle: matched only if ALL in-zone vertices matched.
  for (std::size_t oi = 0; oi < num_obstacles; ++oi) {
    if (obstacles.obstacles[oi].type != rises_interfaces::msg::Obstacle::POINT)
      continue;
    const bool has_unmatched =
        !obs_result.unmatched_vertices_per_obstacle[oi].empty();
    const bool has_in_zone =
        has_unmatched || !obs_result.matched_vertices_per_obstacle[oi].empty();
    if (!has_in_zone)
      continue;

    if (has_unmatched) {
      obs_result.unmatched_obstacle_indices.push_back(oi);
    } else {
      obs_result.matched_obstacle_indices.push_back(oi);
    }
  }

  return obs_result;
}

// ============================================================================
// checkObstaclesInBbox — tight bounding box variant (no safety circle)
// All vertices are used directly — no zone filter needed.
// ============================================================================
BatchPointChecker::ObstacleResult BatchPointChecker::checkObstaclesInBbox(
    const GeofenceMap &map,
    const rises_interfaces::msg::ObstacleArray &obstacles,
    const float tolerance, const bool include_dynamic_obstacles) {
  ObstacleResult obs_result;

  if (obstacles.obstacles.empty()) {
    return obs_result;
  }

  obs_result.matched_vertices_per_obstacle.resize(obstacles.obstacles.size());
  obs_result.unmatched_vertices_per_obstacle.resize(obstacles.obstacles.size());

  // Compute bounding box and count total vertices in one pass.
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  std::size_t total_verts = 0;

  for (const rises_interfaces::msg::Obstacle &obs : obstacles.obstacles) {
    if (obs.type != rises_interfaces::msg::Obstacle::POINT)
      continue;
    for (const geometry_msgs::msg::Point &v : obs.vertices) {
      const float vx = static_cast<float>(v.x);
      const float vy = static_cast<float>(v.y);
      min_x = std::min(min_x, vx);
      min_y = std::min(min_y, vy);
      max_x = std::max(max_x, vx);
      max_y = std::max(max_y, vy);
      ++total_verts;
    }
  }
  if (total_verts == 0)
    return obs_result;

  const BoundingBox query_box(min_x - tolerance, min_y - tolerance,
                              max_x + tolerance, max_y + tolerance);

  // Build flat SoA — no zone filter, all vertices included.
  // Use prefix-sum obs_start[] so we can avoid allocating flat_obs_idx[].
  std::vector<float> flat_x(total_verts);
  std::vector<float> flat_y(total_verts);
  std::vector<std::size_t> obs_start(obstacles.obstacles.size() + 1, 0);

  std::size_t write = 0;
  for (std::size_t oi = 0; oi < obstacles.obstacles.size(); ++oi) {
    obs_start[oi] = write;
    const rises_interfaces::msg::Obstacle &obs = obstacles.obstacles[oi];
    if (obs.type != rises_interfaces::msg::Obstacle::POINT)
      continue;
    for (const geometry_msgs::msg::Point &v : obs.vertices) {
      flat_x[write] = static_cast<float>(v.x);
      flat_y[write] = static_cast<float>(v.y);
      ++write;
    }
  }
  obs_start[obstacles.obstacles.size()] = write;

  std::vector<uint8_t> matched_mask(total_verts, 0u);

  const float tolerance_sq = tolerance * tolerance;
  const ObstacleLayer relevant_layers =
      include_dynamic_obstacles
          ? (ObstacleLayer::FIXED | ObstacleLayer::STATIC |
             ObstacleLayer::DYNAMIC)
          : (ObstacleLayer::FIXED | ObstacleLayer::STATIC);

  thread_local const rises::shape::MapBoundaryContours *t_last_contours =
      nullptr;
  thread_local std::vector<ContourEdge> t_contour_edges;
  const rises::shape::MapBoundaryContours *current_contours =
      map.getMapContours();
  if (current_contours != t_last_contours) {
    t_last_contours = current_contours;
    t_contour_edges = extractContourEdges(map);
  }

  map.forEachObstacleInRegion(query_box, [&](const GeometryEntry &entry) {
    if (!hasLayer(relevant_layers, entry.layer))
      return;

    std::visit(
        [&](const auto &geom) {
          using GeomType = std::decay_t<decltype(geom)>;

          if constexpr (std::is_same_v<GeomType, rises::geofence::Rectangle>) {
            const bool any_new = checkRectContainment_OR(
                flat_x.data(), flat_y.data(),
                static_cast<float>(entry.bbox.min.x) - tolerance,
                static_cast<float>(entry.bbox.min.y) - tolerance,
                static_cast<float>(entry.bbox.max.x) + tolerance,
                static_cast<float>(entry.bbox.max.y) + tolerance,
                matched_mask.data(), total_verts);
            if (any_new)
              obs_result.matched_map_obstacle_ids.insert(entry.id);

          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Line>) {
            const float dx_e = geom.second.x() - geom.first.x();
            const float dy_e = geom.second.y() - geom.first.y();
            const float len_sq = dx_e * dx_e + dy_e * dy_e;
            if (len_sq >= 1e-12f) {
              const bool any_new = checkLineProximity_OR(
                  flat_x.data(), flat_y.data(), geom.first.x(), geom.first.y(),
                  dx_e, dy_e, 1.0f / len_sq, tolerance_sq, matched_mask.data(),
                  total_verts);
              if (any_new)
                obs_result.matched_map_obstacle_ids.insert(entry.id);
            }

          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Point>) {
            const bool any_new = checkPointProximity_OR(
                flat_x.data(), flat_y.data(), geom.x(), geom.y(), tolerance_sq,
                matched_mask.data(), total_verts);
            if (any_new)
              obs_result.matched_map_obstacle_ids.insert(entry.id);

          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Polygon>) {
            bool any_new = false;
            for (std::size_t i = 0; i < total_verts; ++i) {
              if (!matched_mask[i]) {
                const rises::geofence::Point pt(flat_x[i], flat_y[i]);
                if (boost::geometry::within(pt, geom)) {
                  matched_mask[i] = 1u;
                  any_new = true;
                }
              }
            }
            if (any_new)
              obs_result.matched_map_obstacle_ids.insert(entry.id);
          }
        },
        entry.geometry);
  });

  if (!t_contour_edges.empty()) {
    checkContourEdges_OR(flat_x.data(), flat_y.data(), t_contour_edges,
                         tolerance_sq, matched_mask.data(), total_verts);
  }

  // Re-attribute results using prefix-sum obs_start[].
  for (std::size_t oi = 0; oi < obstacles.obstacles.size(); ++oi) {
    if (obstacles.obstacles[oi].type != rises_interfaces::msg::Obstacle::POINT)
      continue;
    const std::size_t start = obs_start[oi];
    const std::size_t end = obs_start[oi + 1];
    if (start == end)
      continue; // No vertices

    bool any_unmatched = false;
    for (std::size_t i = start; i < end; ++i) {
      if (matched_mask[i]) {
        obs_result.matched_vertices_per_obstacle[oi].push_back(i - start);
      } else {
        obs_result.unmatched_vertices_per_obstacle[oi].push_back(i - start);
        any_unmatched = true;
      }
    }

    if (any_unmatched) {
      obs_result.unmatched_obstacle_indices.push_back(oi);
    } else {
      obs_result.matched_obstacle_indices.push_back(oi);
    }
  }

  return obs_result;
}

// ============================================================================
// checkBatch — generic points-vs-map helper (not the laser-scan hot path)
// ============================================================================
BatchPointChecker::Result BatchPointChecker::checkBatch(
    const GeofenceMap &map, const std::vector<Point2D> &points,
    const Point2D &search_center, const float search_radius) {
  if (points.empty())
    return Result{};

  const BoundingBox query_box(
      search_center.x - search_radius, search_center.y - search_radius,
      search_center.x + search_radius, search_center.y + search_radius);

  std::vector<BoundingBox> rectangles;
  std::vector<rises::geofence::Polygon> polygons;
  std::vector<rises::geofence::Line> lines;
  std::vector<rises::geofence::Point> obstacle_points;

  rectangles.reserve(15);
  polygons.reserve(5);
  lines.reserve(5);
  obstacle_points.reserve(5);

  constexpr ObstacleLayer relevant_layers =
      ObstacleLayer::FIXED | ObstacleLayer::STATIC;

  map.forEachObstacleInRegion(query_box, [&](const GeometryEntry &entry) {
    if (!hasLayer(relevant_layers, entry.layer))
      return;

    std::visit(
        [&](const auto &geom) {
          using GeomType = std::decay_t<decltype(geom)>;
          if constexpr (std::is_same_v<GeomType, rises::geofence::Rectangle>) {
            const ::Rectangle &bbox = entry.bbox;
            rectangles.emplace_back(
                static_cast<float>(bbox.min.x), static_cast<float>(bbox.min.y),
                static_cast<float>(bbox.max.x), static_cast<float>(bbox.max.y));
          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Polygon>) {
            polygons.push_back(geom);
          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Line>) {
            lines.push_back(geom);
          } else if constexpr (std::is_same_v<GeomType,
                                              rises::geofence::Point>) {
            obstacle_points.push_back(geom);
          }
        },
        entry.geometry);
  });

  return checkPointsVsObstacles(points, rectangles, polygons, lines,
                                obstacle_points);
}

BatchPointChecker::Result BatchPointChecker::checkPointsVsObstacles(
    const std::vector<Point2D> &points,
    const std::vector<BoundingBox> &rectangles,
    const std::vector<rises::geofence::Polygon> &polygons,
    const std::vector<rises::geofence::Line> &lines,
    const std::vector<rises::geofence::Point> &obstacle_points) {
  Result result;
  result.matched_indices.reserve(points.size() / 2);
  result.unmatched_indices.reserve(points.size() / 2);

  if (rectangles.empty() && polygons.empty() && lines.empty() &&
      obstacle_points.empty()) {
    result.unmatched_indices.resize(points.size());
    std::iota(result.unmatched_indices.begin(), result.unmatched_indices.end(),
              0);
    return result;
  }

  std::vector<uint8_t> matched(points.size(), false);

  for (const BoundingBox &rect : rectangles) {
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!matched[i] && points[i].x >= rect.min_x &&
          points[i].x <= rect.max_x && points[i].y >= rect.min_y &&
          points[i].y <= rect.max_y) {
        matched[i] = true;
      }
    }
  }

  for (const rises::geofence::Polygon &polygon : polygons) {
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!matched[i]) {
        const rises::geofence::Point pt(points[i].x, points[i].y);
        if (boost::geometry::within(pt, polygon)) {
          matched[i] = true;
        }
      }
    }
  }

  constexpr float line_tolerance_sq = 0.05f * 0.05f;
  for (const rises::geofence::Line &line : lines) {
    const float dx_e = line.second.x() - line.first.x();
    const float dy_e = line.second.y() - line.first.y();
    const float len_sq = dx_e * dx_e + dy_e * dy_e;
    if (len_sq < 1e-12f)
      continue;
    const float inv = 1.0f / len_sq;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!matched[i]) {
        const float t = std::clamp(
            static_cast<float>(((points[i].x - line.first.x()) * dx_e +
                                (points[i].y - line.first.y()) * dy_e) *
                               inv),
            0.0f, 1.0f);
        const float cx = line.first.x() + t * dx_e;
        const float cy = line.first.y() + t * dy_e;
        const float ddx = points[i].x - cx;
        const float ddy = points[i].y - cy;
        if (ddx * ddx + ddy * ddy <= line_tolerance_sq) {
          matched[i] = true;
        }
      }
    }
  }

  constexpr float point_tolerance_sq = 0.05f * 0.05f;
  for (const rises::geofence::Point &obs_pt : obstacle_points) {
    const float ox = obs_pt.x();
    const float oy = obs_pt.y();
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!matched[i]) {
        const float dx = points[i].x - ox;
        const float dy = points[i].y - oy;
        if (dx * dx + dy * dy <= point_tolerance_sq) {
          matched[i] = true;
        }
      }
    }
  }

  for (std::size_t i = 0; i < matched.size(); ++i) {
    if (matched[i]) {
      result.matched_indices.push_back(i);
    } else {
      result.unmatched_indices.push_back(i);
    }
  }

  return result;
}

} // namespace rises::geofence::query
