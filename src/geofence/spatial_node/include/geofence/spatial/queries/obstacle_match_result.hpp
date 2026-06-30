#pragma once

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace rises::geofence::query {

/**
 * @brief Per-obstacle matched/unmatched classification of scan vertices.
 *
 * Plain data produced by a matcher (the spatial BatchPointChecker, or the
 * gridmap node's per-vertex grid lookup) and consumed by ObstacleReportBuilder
 * to emit a per-segment ObstacleReport. Deliberately free of any spatial-index
 * or map dependency so report building can be reused by nodes that do not link
 * the spatial index (e.g. the occupancy-grid node).
 */
struct ObstacleMatchResult {
  // Obstacle-level classification (indices into the queried ObstacleArray).
  std::vector<std::size_t>
      matched_obstacle_indices; // All in-zone vertices matched
  std::vector<std::size_t>
      unmatched_obstacle_indices; // At least one in-zone vertex unmatched

  // Per-obstacle vertex tracking, indexed by obstacle index.
  // Inner vector holds matched / unmatched vertex indices for that obstacle.
  // Pre-allocated to obstacles.size(); empty inner vector = no in-zone
  // vertices.
  std::vector<std::vector<std::size_t>> matched_vertices_per_obstacle;
  std::vector<std::vector<std::size_t>> unmatched_vertices_per_obstacle;

  // Map obstacle IDs that had at least one scan point matched (for
  // visualizer).
  std::unordered_set<int64_t> matched_map_obstacle_ids;

  // Per-vertex map obstacle ID attribution.
  // For each matched in-zone vertex: which map obstacle ID it matched
  // against. Indexed same as matched_vertices_per_obstacle[oi][i]. Enables
  // grouping matched scan points by their corresponding map obstacle for
  // downstream segmentation (e.g., extracting visible line segments per
  // obstacle).
  std::vector<std::vector<int64_t>> matched_vertex_map_ids;
};

} // namespace rises::geofence::query
