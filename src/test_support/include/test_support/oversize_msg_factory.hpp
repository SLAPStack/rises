// Factory helpers producing deliberately oversize ROS messages and JSON
// strings used to drive the resource-exhaustion audit regression tests
// (audit finding #4). Sizes are passed by the caller so each test can
// pick a value just under and just over its expected cap.

#pragma once

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>

#include <rises_interfaces/msg/contours.hpp>
#include <rises_interfaces/msg/line_segment.hpp>
#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_update.hpp>
#include <rises_interfaces/msg/obstacle_update_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "test_support/obstacle_builder.hpp"

namespace test_support::oversize {

// Build an ObstacleUpdateArray containing `count` synthetic point obstacles.
// IDs are dense from 0 to count-1; coordinates fan out to avoid spatial-index
// degeneracy at the origin.
inline rises_interfaces::msg::ObstacleUpdateArray
makeObstacleUpdateArray(std::size_t count) {
  rises_interfaces::msg::ObstacleUpdateArray msg;
  msg.updates.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    rises_interfaces::msg::ObstacleUpdate update;
    update.operation = rises_interfaces::msg::ObstacleUpdate::OP_INSERT;
    update.obstacle = ObstacleBuilder::point(static_cast<std::uint64_t>(i),
                                             0.01 * static_cast<double>(i),
                                             0.01 * static_cast<double>(i));
    msg.updates.push_back(std::move(update));
  }
  return msg;
}

// Contours with a single outer hull of `outer_hull_size` points and
// `inner_count` inner polygons each with 4 corners. Inner polygons share the
// same simple square shape so the test focuses on count, not geometry.
inline rises_interfaces::msg::Contours
makeContoursWithVertices(std::size_t outer_hull_size, std::size_t inner_count) {
  rises_interfaces::msg::Contours msg;
  msg.outer_contour_hull.points.reserve(outer_hull_size);
  for (std::size_t i = 0; i < outer_hull_size; ++i) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(0.01 * static_cast<double>(i));
    p.y = static_cast<float>(0.01 * static_cast<double>(i));
    p.z = 0.0f;
    msg.outer_contour_hull.points.push_back(p);
  }
  msg.inner_contours.reserve(inner_count);
  for (std::size_t i = 0; i < inner_count; ++i) {
    geometry_msgs::msg::Polygon poly;
    // resize() avoids the C++11 explicit-default-ctor warning that an
    // initializer list of {} entries triggers under rosidl-generated types.
    poly.points.resize(4);
    msg.inner_contours.push_back(std::move(poly));
  }
  return msg;
}

// VDA5050 order JSON with `node_count` synthetic nodes. Minimal valid
// structure per the VDA5050 v2.x schema so the converter accepts the shape
// and rejects only on size. Each node has a unique nodeId and position.
inline std::string makeVda5050OrderJson(std::size_t node_count) {
  std::ostringstream out;
  out << R"({"headerId":1,"timestamp":"2026-05-10T00:00:00Z",)"
      << R"("version":"2.0.0","manufacturer":"test","serialNumber":"sn0",)"
      << R"("orderId":"o0","orderUpdateId":0,"nodes":[)";
  for (std::size_t i = 0; i < node_count; ++i) {
    if (i > 0) {
      out << ',';
    }
    out << R"({"nodeId":"n)" << i << R"(","sequenceId":)" << i
        << R"(,"released":true,"nodePosition":{"x":)"
        << 0.01 * static_cast<double>(i) << R"(,"y":)"
        << 0.01 * static_cast<double>(i) << R"(,"mapId":"m"},"actions":[]})";
  }
  out << R"(],"edges":[]})";
  return out.str();
}

// AABB JSON array of `obstacle_count` synthetic boxes consumed by
// AabbConverter::parseObstacleUpdates. Each entry carries a unique id and a
// minimal [min,max] pair.
inline std::string makeAabbJson(std::size_t obstacle_count) {
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < obstacle_count; ++i) {
    if (i > 0) {
      out << ',';
    }
    const double base = 0.01 * static_cast<double>(i);
    out << R"({"id":)" << i << R"(,"aabb":[[)" << base << ',' << base
        << R"(],[)" << (base + 0.5) << ',' << (base + 0.5) << "]]}";
  }
  out << ']';
  return out.str();
}

} // namespace test_support::oversize
