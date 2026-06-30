#pragma once

#include "rises_interfaces/msg/obstacle_array.hpp"
#include "geofence/spatial/common/bounding_box.hpp"
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/policies/robot_safety_profile.hpp"
#include "geofence/spatial/queries/obstacle_match_result.hpp"
#include "spatial_index_selection.hpp"
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace rises::geofence::query {

/**
 * @brief Batch point checker for safety zone queries
 *
 * Performs a spatial query to find map obstacles in a region, then checks each
 * detected scan point for containment using geometry-type-specific tests.
 *
 * Processing pipeline (per call):
 *   1. Spatial query  -> typed SoA buffers (rects, polygons, lines, points)
 *   2. Detection-zone filter (SIMD) -> compact in-zone vertex arrays
 *   3. Obstacle-major SIMD matching:
 *      a. Rectangles  - SIMD AABB test over all in-zone vertices per rectangle
 *      b. Lines       - SIMD parametric closest-point per line segment
 *      c. Point obst. - SIMD distance² per obstacle point
 *      d. Polygons    - scalar boost::geometry::within (unavoidable)
 *      e. Contour edges - SIMD closest-point per wall edge
 *   4. Populate flat-vector result (no heap allocations per vertex)
 *
 * Warehouse contour edges are cached per-thread via thread_local and
 * invalidated automatically when the map's contour pointer changes.
 */
class BatchPointChecker {
public:
  struct Result {
    std::vector<std::size_t> matched_indices;
    std::vector<std::size_t> unmatched_indices;
  };

  // The match-result data lives in a spatial-index-free header so report
  // building can be reused by nodes that don't link the spatial index. Aliased
  // here so existing BatchPointChecker::ObstacleResult references keep working.
  using ObstacleResult = rises::geofence::query::ObstacleMatchResult;

  /**
   * @brief Check obstacles using robot safety profile (recommended)
   *
   * Handles two common patterns efficiently:
   * 1. Multiple obstacles, each with one vertex
   * 2. One obstacle with multiple vertices (current scan format)
   *
   * Supports safety donut filtering: outer zone defines detection area,
   * inner zone defines exclusion area (e.g., fork pickup zone).
   *
   * @param map Geofence map
   * @param obstacles Obstacle array (only POINT type obstacles processed)
   * @param robot_position Robot center position
   * @param robot_heading_rad Robot heading in radians (for oriented zones)
   * @param profile Robot safety profile defining detection zones
   * @param tolerance Matching tolerance for all geometry types (meters)
   * @param include_dynamic_obstacles Whether to include dynamic layer in
   * matching
   * @return Obstacle-level match results
   */
  [[nodiscard]] static ObstacleResult
  checkObstacles(const GeofenceMap &map,
                 const rises_interfaces::msg::ObstacleArray &obstacles,
                 const Point2D &robot_position, const float robot_heading_rad,
                 const RobotSafetyProfile &profile,
                 const float tolerance = 0.10f,
                 const bool include_dynamic_obstacles = false);

  /**
   * @brief Check obstacles against map using bounding box query (more efficient
   * when no radius limit)
   *
   * Calculates bounding box from all obstacle vertices, queries only obstacles
   * in that region, then checks points. More efficient than radius-based query
   * when checking all detected points regardless of distance from robot.
   *
   * @param map Geofence map
   * @param obstacles Obstacle array (only POINT type obstacles processed)
   * @param tolerance Matching tolerance for all geometry types (meters)
   * @return Obstacle-level match results
   */
  [[nodiscard]] static ObstacleResult
  checkObstaclesInBbox(const GeofenceMap &map,
                       const rises_interfaces::msg::ObstacleArray &obstacles,
                       const float tolerance = 0.10f,
                       const bool include_dynamic_obstacles = false);

  /**
   * @brief Check points against map obstacles
   *
   * Performs spatial query to find obstacles in the region, then checks
   * each point for geometric containment/proximity.
   *
   * Typical workload: 500 points × 10-20 obstacles = 5,000-10,000 checks
   * (~10-50μs)
   *
   * @param map Geofence map
   * @param points Points to check (typically from safety circle)
   * @param search_center Center point for spatial query (typically robot
   * position)
   * @param search_radius Search radius (typically safety circle radius)
   * @return Match results
   */
  [[nodiscard]] Result checkBatch(const GeofenceMap &map,
                                  const std::vector<Point2D> &points,
                                  const Point2D &search_center,
                                  const float search_radius);

private:
  // Helper: Check points vs obstacles using specialized functions for each
  // geometry type
  static Result checkPointsVsObstacles(
      const std::vector<Point2D> &points,
      const std::vector<BoundingBox> &rectangles,
      const std::vector<rises::geofence::Polygon> &polygons,
      const std::vector<rises::geofence::Line> &lines,
      const std::vector<rises::geofence::Point> &obstacle_points);
};

} // namespace rises::geofence::query
