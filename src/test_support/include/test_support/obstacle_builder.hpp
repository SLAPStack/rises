// Centralised builder for rises_interfaces::msg::Obstacle used across tests.
// Replaces three near-identical ObstacleBuilder helpers previously duplicated
// in geofence/test/, message_translator/test/, and
// laserscan_preprocessor/test/. Consolidated here so any schema change to
// Obstacle is updated in one place.

#pragma once

#include <cstdint>
#include <vector>

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_array.hpp>
#include <rises_interfaces/msg/obstacle_update.hpp>
#include <rises_interfaces/msg/obstacle_update_array.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace test_support {

inline geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

struct ObstacleBuilder {
  static rises_interfaces::msg::Obstacle point(std::uint64_t id, double x,
                                               double y) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    // Production POINT obstacles set BOTH .position and .vertices[0] to the same
    // coordinates: intersectsCircle reads .position, fromObstacleMsg reads
    // .vertices[0]. Setting only vertices left .position at the (0,0,0) default.
    obs.position = makePoint(x, y);
    obs.vertices.push_back(makePoint(x, y));
    return obs;
  }

  static rises_interfaces::msg::Obstacle line(std::uint64_t id, double x1,
                                              double y1, double x2, double y2) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    obs.vertices = {makePoint(x1, y1), makePoint(x2, y2)};
    return obs;
  }

  static rises_interfaces::msg::Obstacle rectangle(std::uint64_t id,
                                                   double min_x, double min_y,
                                                   double max_x, double max_y) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 0.5 * (min_x + max_x);
    obs.position.y = 0.5 * (min_y + max_y);
    obs.width = static_cast<float>(max_x - min_x);
    obs.height = static_cast<float>(max_y - min_y);
    obs.vertices = {makePoint(min_x, min_y), makePoint(max_x, min_y),
                    makePoint(max_x, max_y), makePoint(min_x, max_y)};
    return obs;
  }

  static rises_interfaces::msg::Obstacle
  polygon(std::uint64_t id,
          const std::vector<geometry_msgs::msg::Point> &vertices) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    obs.vertices = vertices;
    return obs;
  }
};

} // namespace test_support
