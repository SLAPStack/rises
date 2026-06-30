#include "geofence/spatial/node/map_update_handler.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include <algorithm>

namespace rises {

// Hard cap on inbound ObstacleUpdateArray size. Picked well above any realistic
// warehouse map (~thousands of obstacles) while small enough that a single
// adversarial / mis-sized message cannot drive the spatial index into hundreds
// of MB of work. See audit finding #4.
constexpr std::size_t kMaxUpdatesPerMessage = 10'000;

MapUpdateHandler::Stats MapUpdateHandler::applyUpdates(
    const rises_interfaces::msg::ObstacleUpdateArray &updates,
    rises::geofence::GeofenceMap &map,
    const std::shared_ptr<GeofenceVisualizer> &visualizer,
    rclcpp::Logger logger, const std::string &source_tag) {
  Stats stats;

  if (updates.updates.size() > kMaxUpdatesPerMessage) {
    RCLCPP_ERROR(logger,
                 "[%s] ObstacleUpdateArray oversize: %zu updates exceeds cap "
                 "%zu — rejecting message",
                 source_tag.c_str(), updates.updates.size(),
                 kMaxUpdatesPerMessage);
    return stats;
  }
  constexpr std::size_t MAX_SAMPLE_LOG = 3;
  constexpr std::size_t MAX_OUTSIDE_LOG = 5;
  std::size_t insert_log_count = 0;
  std::size_t remove_log_count = 0;
  std::size_t outside_log_count = 0;

  const rises::shape::MapBoundaryContours *contours = map.getMapContours();

  // Log contour bounds for diagnostics
  if (contours) {
    const std::vector<Point2D> &outer =
        contours->getOuterContour().getVertices();
    if (!outer.empty()) {
      double cx_min = outer[0].x, cx_max = outer[0].x;
      double cy_min = outer[0].y, cy_max = outer[0].y;
      for (const Point2D &v : outer) {
        cx_min = std::min(cx_min, v.x);
        cx_max = std::max(cx_max, v.x);
        cy_min = std::min(cy_min, v.y);
        cy_max = std::max(cy_max, v.y);
      }
      RCLCPP_INFO(
          logger,
          "[%s] Contour bounds: x=[%.1f, %.1f] y=[%.1f, %.1f] (%zu vertices)",
          source_tag.c_str(), cx_min, cx_max, cy_min, cy_max, outer.size());
    }
  }

  RCLCPP_DEBUG(logger,
               "[MAP_DEBUG] %s START: %zu updates, map_size=%zu, contours=%s",
               source_tag.c_str(), updates.updates.size(),
               map.getAllObstacleIds().size(), contours ? "set" : "NOT SET");

  for (const rises_interfaces::msg::ObstacleUpdate &update : updates.updates) {
    if (update.operation == rises_interfaces::msg::ObstacleUpdate::OP_INSERT) {
      rises_interfaces::msg::Obstacle obs = update.obstacle;

      const float bx = static_cast<float>(obs.position.x);
      const float by = static_cast<float>(obs.position.y);

      rises::geofence::CoordinateTransform::transformObstacle(obs);

      const bool inside =
          contours
              ? contours->isPointInside(Point2D{obs.position.x, obs.position.y})
              : true;
      if (!inside)
        stats.outside_contours++;

      if (insert_log_count < MAX_SAMPLE_LOG) {
        RCLCPP_DEBUG(logger,
                    "[MAP_DEBUG] [%s INSERT #%zu] BEFORE(%.3f,%.3f) "
                    "AFTER(%.3f,%.3f) w=%.3f h=%.3f inside=%s id=%ld",
                    source_tag.c_str(), insert_log_count, bx, by,
                    obs.position.x, obs.position.y, obs.width, obs.height,
                    inside ? "YES" : "NO", obs.id);
        insert_log_count++;
      }
      if (!inside && outside_log_count < MAX_OUTSIDE_LOG) {
        RCLCPP_WARN(logger,
                    "[MAP_DEBUG] [%s OUTSIDE #%zu] BEFORE(%.3f,%.3f) "
                    "AFTER(%.3f,%.3f) w=%.3f h=%.3f id=%ld",
                    source_tag.c_str(), outside_log_count, bx, by,
                    obs.position.x, obs.position.y, obs.width, obs.height,
                    obs.id);
        outside_log_count++;
      }

      const rises::geofence::Geometry shape =
          rises::geofence::fromObstacleMsg(obs);
      map.insertObstacle(obs.id, shape);
      stats.added++;
      if (visualizer) {
        visualizer->addObstacle(obs);
      }
    } else if (update.operation ==
               rises_interfaces::msg::ObstacleUpdate::OP_DELETE) {
      const bool existed = map.getObstacle(update.obstacle.id) != nullptr;
      if (!existed)
        stats.remove_not_found++;

      if (remove_log_count < MAX_SAMPLE_LOG) {
        RCLCPP_DEBUG(logger,
                    "[MAP_DEBUG] [%s REMOVE #%zu] id=%ld existed_in_map=%s",
                    source_tag.c_str(), remove_log_count, update.obstacle.id,
                    existed ? "YES" : "NO");
        remove_log_count++;
      }

      map.removeObstacle(update.obstacle.id);
      stats.removed++;
      if (visualizer) {
        visualizer->removeObstacle(update.obstacle.id);
      }
    }
  }

  RCLCPP_DEBUG(logger,
              "[MAP_DEBUG] [%s COMPLETE] +%d inserted, -%d removed, %d "
              "outside_contours, %d remove_not_found, map_size=%zu",
              source_tag.c_str(), stats.added, stats.removed,
              stats.outside_contours, stats.remove_not_found,
              map.getAllObstacleIds().size());

  return stats;
}

rises::shape::MapBoundaryContours MapUpdateHandler::parseContours(
    const rises_interfaces::msg::Contours &contours_msg) {
  std::vector<Point2D> outer_vertices;
  outer_vertices.reserve(contours_msg.outer_contour_hull.points.size());
  for (const geometry_msgs::msg::Point32 &pt :
       contours_msg.outer_contour_hull.points)
    outer_vertices.push_back(
        {static_cast<double>(pt.x), static_cast<double>(pt.y)});

  std::vector<shape::LineSegment2D> outer_segments;
  outer_segments.reserve(contours_msg.outer_contour_segments.size());
  for (const rises_interfaces::msg::LineSegment &seg_msg :
       contours_msg.outer_contour_segments) {
    const Point2D start{static_cast<double>(seg_msg.start.x),
                        static_cast<double>(seg_msg.start.y)};
    const Point2D end{static_cast<double>(seg_msg.end.x),
                      static_cast<double>(seg_msg.end.y)};
    outer_segments.emplace_back(start, end);
  }

  std::vector<shape::PolygonContour> inner_polygons;
  inner_polygons.reserve(contours_msg.inner_contours.size());
  for (const geometry_msgs::msg::Polygon &poly : contours_msg.inner_contours) {
    std::vector<Point2D> hole_vertices;
    hole_vertices.reserve(poly.points.size());
    for (const geometry_msgs::msg::Point32 &pt : poly.points)
      hole_vertices.push_back(
          {static_cast<double>(pt.x), static_cast<double>(pt.y)});
    inner_polygons.emplace_back(hole_vertices);
  }

  const shape::PolygonContour outer_poly(outer_vertices);
  return shape::MapBoundaryContours(outer_poly, inner_polygons, outer_segments);
}

} // namespace rises
