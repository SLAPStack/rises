#include "geofence/spatial/visualization/geofence_visualizer.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>
#include <thread>

rises::GeofenceVisualizer::GeofenceVisualizer(
    rclcpp_lifecycle::LifecycleNode *node, std::string tf_prefix,
    std::string target_frame, std::string base_link_frame,
    bool enable_safety_circle, float safety_circle_radius)
    : node_(node), tf_prefix_(tf_prefix), target_frame_(target_frame),
      base_link_frame_(base_link_frame),
      enable_safety_circle_(enable_safety_circle),
      safety_circle_radius_(safety_circle_radius) {
  rclcpp::QoS persistent_qos(10);
  persistent_qos.transient_local();
  persistent_qos.reliable();

  this->map_pub_ =
      this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "geofence_map_viz", persistent_qos);
  this->safety_circle_pub_ =
      this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "safety_circle_viz", persistent_qos);
  this->area_pub_ =
      this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "area_viz", persistent_qos);
  this->contour_pub_ =
      this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "warehouse_contours_viz", persistent_qos);

  rclcpp::QoS ephemeral_qos(10);
  ephemeral_qos.transient_local(); // Changed from default VOLATILE to
                                   // TRANSIENT_LOCAL for RViz compatibility
  ephemeral_qos.reliable();

  this->matched_segments_pub_ =
      this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "matched_segments_viz", ephemeral_qos);
  this->error_segments_pub_ =
      this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "error_segments_viz", ephemeral_qos);
  this->path_pub_ =
      this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "path_validation_viz", ephemeral_qos);
}

visualization_msgs::msg::Marker
rises::GeofenceVisualizer::createBaseMarker(const std::string &ns,
                                            const std::string tf_prefix,
                                            const std::string target_frame) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = tf_prefix + target_frame;
  marker.header.stamp = this->node_->now();
  marker.ns = ns;
  marker.id = this->next_id_++;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

int32_t
rises::GeofenceVisualizer::getOrAssignMarkerId(const int64_t obstacle_id) {
  auto it = this->obstacle_to_marker_id_.find(obstacle_id);
  if (it != this->obstacle_to_marker_id_.end()) {
    return it->second;
  }
  const int32_t marker_id = this->next_marker_id_++;
  this->obstacle_to_marker_id_[obstacle_id] = marker_id;
  return marker_id;
}

int32_t
rises::GeofenceVisualizer::getMarkerId(const int64_t obstacle_id) const {
  auto it = this->obstacle_to_marker_id_.find(obstacle_id);
  return (it != this->obstacle_to_marker_id_.end()) ? it->second : -1;
}

void rises::GeofenceVisualizer::addObstacle(
    const rises_interfaces::msg::Obstacle &obstacle) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  visualization_msgs::msg::Marker marker =
      this->createBaseMarker("geofence_obstacles", "", this->target_frame_);

  // RViz marker ID must be 32-bit — use stable mapping to avoid collisions
  marker.id = this->getOrAssignMarkerId(static_cast<int64_t>(obstacle.id));

  // Determine color based on match state: gray->green when matched, gray again
  // if not matched in next scan
  const bool is_currently_matched =
      this->currently_matched_obstacles_.find(obstacle.id) !=
      this->currently_matched_obstacles_.end();
  const bool was_previously_matched =
      this->previously_matched_obstacles_.find(obstacle.id) !=
      this->previously_matched_obstacles_.end();

  float color_r = 0.5f; // gray
  float color_g = 0.5f; // gray
  float color_b = 0.5f; // gray

  if (is_currently_matched) {
    // Currently matched: green
    color_r = 0.0f;
    color_g = 1.0f;
    color_b = 0.0f;
  } else if (was_previously_matched) {
    // Was matched before but not now: return to gray (already set above)
    // No change needed
  }
  // else: never matched or first time seen: gray (already set above)

  switch (obstacle.type) {
  case rises_interfaces::msg::Obstacle::CIRCLE:
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position.x = obstacle.position.x;
    marker.pose.position.y = obstacle.position.y;
    marker.pose.position.z = obstacle.position.z;
    marker.scale.x = marker.scale.y = marker.scale.z = obstacle.radius * 2.0f;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = 0.8f;
    break;

  case rises_interfaces::msg::Obstacle::RECTANGLE:
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = obstacle.position.x;
    marker.pose.position.y = obstacle.position.y;
    marker.pose.position.z = obstacle.position.z;
    marker.scale.x = obstacle.width;
    marker.scale.y = obstacle.height;
    // Use minor_axis for 3D height if provided, otherwise use default
    marker.scale.z = (obstacle.minor_axis > 0.01f) ? obstacle.minor_axis : 0.5f;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = 0.8f;
    break;

  case rises_interfaces::msg::Obstacle::POLYGON:
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.1f;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = 0.8f;
    for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
      marker.points.push_back(pt);
    }
    if (obstacle.vertices.size() > 2)
      marker.points.push_back(obstacle.vertices.front());
    // close polygon
    break;

  default:
    RCLCPP_WARN(this->node_->get_logger(),
                "Unknown obstacle type %d for ID %ld", obstacle.type,
                obstacle.id);
    return;
  }

  // Store the marker in the map keyed by full 64-bit ID
  this->obstacle_markers_[obstacle.id] = marker;
  this->obstacles_dirty_ = true;

  RCLCPP_DEBUG(
      this->node_->get_logger(),
      "Added obstacle marker ID=%ld, type=%d, matched=%s, total now=%zu",
      obstacle.id, obstacle.type, is_currently_matched ? "YES" : "NO",
      this->obstacle_markers_.size());
}

void rises::GeofenceVisualizer::beginNewScanCycle() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  // Move current matches to previous, clear current
  this->previously_matched_obstacles_ = this->currently_matched_obstacles_;
  this->currently_matched_obstacles_.clear();
}

void rises::GeofenceVisualizer::markObstacleMatched(const int64_t id) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->currently_matched_obstacles_.insert(id);
}

void rises::GeofenceVisualizer::refreshMapColors() {
  std::lock_guard<std::mutex> lock(this->mutex_);

  // Update colors for all obstacle markers based on current match state
  for (std::pair<const int64_t, visualization_msgs::msg::Marker>
           &id_marker_pair : this->obstacle_markers_) {
    const int64_t obstacle_id = id_marker_pair.first;
    visualization_msgs::msg::Marker &marker = id_marker_pair.second;

    // Skip DELETE markers
    if (marker.action == visualization_msgs::msg::Marker::DELETE) {
      continue;
    }

    // Determine color based on match state
    const bool is_currently_matched =
        this->currently_matched_obstacles_.find(obstacle_id) !=
        this->currently_matched_obstacles_.end();

    if (is_currently_matched) {
      // Currently matched: green
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else {
      // Not matched (or was matched before but not now): gray
      marker.color.r = 0.5f;
      marker.color.g = 0.5f;
      marker.color.b = 0.5f;
    }

    // Keep alpha unchanged
    // marker.color.a remains at 0.8f
  }

  // Mark dirty to trigger republishing
  this->obstacles_dirty_ = true;
}

void rises::GeofenceVisualizer::removeObstacle(const int64_t id) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  const int32_t marker_id = this->getMarkerId(id);
  if (marker_id < 0) {
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "removeObstacle: no marker mapping for ID=%ld, skipping", id);
    return;
  }

  // Convert existing marker into a DELETE marker
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.id = marker_id;
  delete_marker.ns = "geofence_obstacles";
  delete_marker.action = visualization_msgs::msg::Marker::DELETE;
  delete_marker.header.stamp = this->node_->now();
  delete_marker.header.frame_id = this->target_frame_;

  this->obstacles_dirty_ = true;
  this->obstacle_markers_[id] = delete_marker;
  RCLCPP_DEBUG(this->node_->get_logger(),
               "Marked obstacle ID=%ld (marker_id=%d) for deletion", id,
               marker_id);
}

void rises::GeofenceVisualizer::updateAreaColor(int64_t id, float r, float g,
                                                float b) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  for (visualization_msgs::msg::Marker &m : this->area_markers_) {
    if (m.id == id) {
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      this->areas_dirty_ = true;
      break;
    }
  }
}

void rises::GeofenceVisualizer::addMapBoundary(
    const rises::shape::MapBoundaryContours &contours, const std::string &ns) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->contour_markers_.clear();

  // Default wall height for 3D visualization (can be made configurable)
  constexpr float wall_height = 2.0f;

  const std::vector<Point2D> &outer = contours.getOuterContour().getVertices();
  if (!outer.empty()) {
    // Create bottom outline
    visualization_msgs::msg::Marker outer_marker_bottom =
        this->createBaseMarker(ns + "_bottom", "", this->target_frame_);
    outer_marker_bottom.type = visualization_msgs::msg::Marker::LINE_STRIP;
    outer_marker_bottom.scale.x = 0.05f;
    outer_marker_bottom.color.r = 0.0f;
    outer_marker_bottom.color.g = 1.0f;
    outer_marker_bottom.color.b = 0.0f;
    outer_marker_bottom.color.a = 0.8f;

    // Create top outline for 3D visualization
    visualization_msgs::msg::Marker outer_marker_top = outer_marker_bottom;
    outer_marker_top.ns = ns + "_top";
    outer_marker_top.id = this->next_id_++;

    // Create vertical lines for 3D walls
    visualization_msgs::msg::Marker outer_marker_walls =
        this->createBaseMarker(ns + "_walls", "", this->target_frame_);
    outer_marker_walls.type = visualization_msgs::msg::Marker::LINE_LIST;
    outer_marker_walls.scale.x = 0.03f;
    outer_marker_walls.color.r = 0.0f;
    outer_marker_walls.color.g = 1.0f;
    outer_marker_walls.color.b = 0.0f;
    outer_marker_walls.color.a = 0.6f;

    // Add each vertex to bottom and top outlines
    for (const Point2D &v : outer) {
      geometry_msgs::msg::Point p_bottom;
      p_bottom.x = v.x;
      p_bottom.y = v.y;
      p_bottom.z = 0.0;
      outer_marker_bottom.points.push_back(p_bottom);

      geometry_msgs::msg::Point p_top;
      p_top.x = v.x;
      p_top.y = v.y;
      p_top.z = wall_height;
      outer_marker_top.points.push_back(p_top);

      // Add vertical line from bottom to top
      outer_marker_walls.points.push_back(p_bottom);
      outer_marker_walls.points.push_back(p_top);
    }

    // Close the loops
    if (outer.size() > 2) {
      outer_marker_bottom.points.push_back(outer_marker_bottom.points.front());
      outer_marker_top.points.push_back(outer_marker_top.points.front());
    }

    this->contour_markers_.push_back(outer_marker_bottom);
    this->contour_markers_.push_back(outer_marker_top);
    this->contour_markers_.push_back(outer_marker_walls);
  }

  const std::vector<rises::shape::PolygonContour> &inner_contours =
      contours.getInnerContours();
  for (const rises::shape::PolygonContour &inner : inner_contours) {
    const std::vector<Point2D> &vertices = inner.getVertices();
    if (vertices.empty())
      continue;

    // Create bottom outline for inner contour
    visualization_msgs::msg::Marker inner_marker_bottom =
        this->createBaseMarker(ns + "_inner_bottom", "", this->target_frame_);
    inner_marker_bottom.type = visualization_msgs::msg::Marker::LINE_STRIP;
    inner_marker_bottom.scale.x = 0.05f;
    inner_marker_bottom.color.r = 0.0f;
    inner_marker_bottom.color.g = 1.0f;
    inner_marker_bottom.color.b = 0.0f;
    inner_marker_bottom.color.a = 0.8f;

    // Create top outline for inner contour
    visualization_msgs::msg::Marker inner_marker_top = inner_marker_bottom;
    inner_marker_top.ns = ns + "_inner_top";
    inner_marker_top.id = this->next_id_++;

    // Create vertical walls for inner contour
    visualization_msgs::msg::Marker inner_marker_walls =
        this->createBaseMarker(ns + "_inner_walls", "", this->target_frame_);
    inner_marker_walls.type = visualization_msgs::msg::Marker::LINE_LIST;
    inner_marker_walls.scale.x = 0.03f;
    inner_marker_walls.color.r = 0.0f;
    inner_marker_walls.color.g = 1.0f;
    inner_marker_walls.color.b = 0.0f;
    inner_marker_walls.color.a = 0.6f;

    // Add each vertex to bottom and top outlines
    for (const Point2D &v : vertices) {
      geometry_msgs::msg::Point p_bottom;
      p_bottom.x = v.x;
      p_bottom.y = v.y;
      p_bottom.z = 0.0;
      inner_marker_bottom.points.push_back(p_bottom);

      geometry_msgs::msg::Point p_top;
      p_top.x = v.x;
      p_top.y = v.y;
      p_top.z = wall_height;
      inner_marker_top.points.push_back(p_top);

      // Add vertical line from bottom to top
      inner_marker_walls.points.push_back(p_bottom);
      inner_marker_walls.points.push_back(p_top);
    }

    // Close the loops
    if (vertices.size() > 2) {
      inner_marker_bottom.points.push_back(inner_marker_bottom.points.front());
      inner_marker_top.points.push_back(inner_marker_top.points.front());
    }

    this->contour_markers_.push_back(inner_marker_bottom);
    this->contour_markers_.push_back(inner_marker_top);
    this->contour_markers_.push_back(inner_marker_walls);
  }

  this->contours_dirty_ = true;
}

void rises::GeofenceVisualizer::addSafetyCircle(float radius,
                                                const std::string &ns) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  const float height = 0.1f;

  visualization_msgs::msg::Marker marker =
      this->createBaseMarker(ns, this->tf_prefix_, this->target_frame_);
  marker.header.frame_id =
      this->tf_prefix_.empty()
          ? this->base_link_frame_
          : this->tf_prefix_ + "_" + this->base_link_frame_;
  marker.header.stamp = rclcpp::Time{0, 0, RCL_ROS_TIME};
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = height / 2.0f; // place cylinder so base is at z=0
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = radius * 2.0f; // diameter in x
  marker.scale.y = radius * 2.0f; // diameter in y
  marker.scale.z = height;        // height along z-axis

  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;

  this->safety_circle_markers_.push_back(marker);
  this->safety_circles_dirty_ = true;

  RCLCPP_INFO(this->node_->get_logger(),
              "Added safety cylinder with radius=%.2f, height=%.2f attached to "
              "frame '%s'",
              radius, height, marker.header.frame_id.c_str());
}

void rises::GeofenceVisualizer::clearSafetyCircle() {
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (this->safety_circle_markers_.empty()) {
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "No safety circle markers to clear");
    return;
  }

  // Publish DELETE markers for existing safety circles
  std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
      std::make_unique<visualization_msgs::msg::MarkerArray>();

  for (visualization_msgs::msg::Marker &marker : this->safety_circle_markers_) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    array->markers.push_back(marker);
  }

  if (this->safety_circle_pub_) {
    this->safety_circle_pub_->publish(std::move(array));
  }

  // Clear the vector
  this->safety_circle_markers_.clear();
  this->safety_circles_dirty_ = true;

  RCLCPP_DEBUG(this->node_->get_logger(), "Cleared safety circle markers");
}

void rises::GeofenceVisualizer::addArea(int64_t, float x, float y, float width,
                                        float height, const std::string &ns) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  visualization_msgs::msg::Marker marker =
      this->createBaseMarker(ns, "", this->target_frame_);
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.scale.x = width;
  marker.scale.y = height;
  marker.scale.z = 0.2f;
  marker.color.r = 0.5f;
  marker.color.g = 0.5f;
  marker.color.b = 0.5f;
  marker.color.a = 0.3f;

  this->area_markers_.push_back(marker);
  this->areas_dirty_ = true;
}

void rises::GeofenceVisualizer::addMatchedSegment(
    const rises_interfaces::msg::Obstacle &obstacle) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  visualization_msgs::msg::Marker marker =
      this->createBaseMarker("matched_segment_ns", "", this->target_frame_);
  marker.id = this->getOrAssignMarkerId(static_cast<int64_t>(obstacle.id));

  // Matched = blue
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.8f;

  // Choose marker type based on obstacle type
  if (obstacle.type == rises_interfaces::msg::Obstacle::POINT) {
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.scale.x = 0.05f;
    marker.scale.y = 0.05f;
    for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
      marker.points.push_back(pt);
    }
  } else if (obstacle.type == rises_interfaces::msg::Obstacle::POLYGON) {
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.05f;
    for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
      marker.points.push_back(pt);
    }
    if (obstacle.vertices.size() > 2) {
      marker.points.push_back(obstacle.vertices.front());
    }
  } else if (obstacle.type == rises_interfaces::msg::Obstacle::LINE) {
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.05f;
    for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
      marker.points.push_back(pt);
    }
  } else {
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.05f;
    geometry_msgs::msg::Point p1 = obstacle.position;
    geometry_msgs::msg::Point p2 =
        obstacle.vertices.empty() ? p1 : obstacle.vertices.front();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  // Auto-expire after 500ms so stale markers don't persist in RViz
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  matched_segment_markers_[obstacle.id] = marker; // overwrite existing
}

void rises::GeofenceVisualizer::addErrorSegment(
    const rises_interfaces::msg::Obstacle &obstacle) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  RCLCPP_DEBUG(this->node_->get_logger(),
               "[VIZ] addErrorSegment: id=%ld, type=%d, vertices=%zu",
               obstacle.id, obstacle.type, obstacle.vertices.size());

  visualization_msgs::msg::Marker marker =
      this->createBaseMarker("error_segment_ns", "", this->target_frame_);
  marker.id = this->getOrAssignMarkerId(static_cast<int64_t>(obstacle.id));

  // Error = red
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  if (this->error_segments_as_lines_) {
    // Line mode: LINE_STRIP for 2+ vertices (open, no closing loop), SPHERE for
    // single point
    if (obstacle.vertices.size() == 1) {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.scale.x = 0.08f;
      marker.scale.y = 0.08f;
      marker.scale.z = 0.08f;
      marker.pose.position = obstacle.vertices.front();
    } else if (obstacle.vertices.size() >= 2) {
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.scale.x = 0.05f;
      for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
        marker.points.push_back(pt);
      }
    }
  } else {
    // Default: respect obstacle type from the report pipeline
    if (obstacle.type == rises_interfaces::msg::Obstacle::POINT) {
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.scale.x = 0.05f;
      marker.scale.y = 0.05f;
      for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
        marker.points.push_back(pt);
      }
    } else if (obstacle.type == rises_interfaces::msg::Obstacle::POLYGON) {
      // Closed polygon — connect last vertex back to the first
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.scale.x = 0.05f;
      for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
        marker.points.push_back(pt);
      }
      if (obstacle.vertices.size() > 2) {
        marker.points.push_back(obstacle.vertices.front());
      }
    } else if (obstacle.type == rises_interfaces::msg::Obstacle::LINE) {
      // Open line segment — no closing loop
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.scale.x = 0.05f;
      for (const geometry_msgs::msg::Point &pt : obstacle.vertices) {
        marker.points.push_back(pt);
      }
    } else {
      // Fallback: draw from position to first vertex
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.scale.x = 0.05f;
      const geometry_msgs::msg::Point p1 = obstacle.position;
      const geometry_msgs::msg::Point p2 =
          obstacle.vertices.empty() ? p1 : obstacle.vertices.front();
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  }

  // Auto-expire after 500ms so stale markers don't persist in RViz
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  this->error_segment_markers_[obstacle.id] = marker;
}

void rises::GeofenceVisualizer::processCorrespondenceResult(
    const rises_interfaces::msg::Obstacle &obstacle, bool matched) {
  if (matched) {
    this->addMatchedSegment(obstacle);
  } else {
    this->addErrorSegment(obstacle);
  }
}

void rises::GeofenceVisualizer::publishMap() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (!this->obstacles_dirty_ || !this->map_pub_)
    return;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
      std::make_unique<visualization_msgs::msg::MarkerArray>();

  // Publish all markers
  for (std::pair<const int64_t, visualization_msgs::msg::Marker>
           &id_marker_pair : this->obstacle_markers_) {
    array->markers.push_back(id_marker_pair.second);
  }

  this->map_pub_->publish(std::move(array));
  this->obstacles_dirty_ = false;

  // Remove markers that were DELETEd and clean up ID mapping
  for (std::unordered_map<int64_t, visualization_msgs::msg::Marker>::iterator
           it = this->obstacle_markers_.begin();
       it != this->obstacle_markers_.end();) {
    if (it->second.action == visualization_msgs::msg::Marker::DELETE) {
      this->obstacle_to_marker_id_.erase(it->first);
      it = obstacle_markers_.erase(it);
    } else {
      ++it;
    }
  }
}

void rises::GeofenceVisualizer::publishSafetyCircle() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (!this->safety_circles_dirty_ || !this->safety_circle_pub_)
    return;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
      std::make_unique<visualization_msgs::msg::MarkerArray>();
  array->markers = this->safety_circle_markers_;
  this->safety_circle_pub_->publish(std::move(array));
  // Keep dirty flag - safety circle persists
}

void rises::GeofenceVisualizer::publishAreas() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (!this->areas_dirty_ || !this->area_pub_)
    return;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
      std::make_unique<visualization_msgs::msg::MarkerArray>();
  array->markers = this->area_markers_;
  this->area_pub_->publish(std::move(array));
  this->areas_dirty_ = false;
}

void rises::GeofenceVisualizer::publishContours() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (!this->contours_dirty_ || !this->contour_pub_)
    return;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
      std::make_unique<visualization_msgs::msg::MarkerArray>();
  array->markers = this->contour_markers_;
  this->contour_pub_->publish(std::move(array));
  this->contours_dirty_ = false;
}

void rises::GeofenceVisualizer::publishSegments() {
  std::lock_guard<std::mutex> lock(this->mutex_);

  RCLCPP_DEBUG(
      this->node_->get_logger(),
      "[VIZ] publishSegments called: matched=%zu, error=%zu, activated=%s",
      this->matched_segment_markers_.size(),
      this->error_segment_markers_.size(), this->is_activated_ ? "YES" : "NO");

  // Publish matched segments (markers have lifetime, auto-expire in RViz)
  if (this->matched_segments_pub_ && !this->matched_segment_markers_.empty()) {
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "[VIZ] Publishing %zu matched segments",
                 this->matched_segment_markers_.size());
    std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
        std::make_unique<visualization_msgs::msg::MarkerArray>();

    for (auto &[id, marker] : this->matched_segment_markers_) {
      array->markers.push_back(marker);
    }

    this->matched_segments_pub_->publish(std::move(array));
    this->matched_segment_markers_.clear();
  }

  // Publish error segments (markers have lifetime, auto-expire in RViz)
  if (this->error_segments_pub_ && !this->error_segment_markers_.empty()) {
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "[VIZ] Publishing %zu error segments",
                 this->error_segment_markers_.size());
    std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
        std::make_unique<visualization_msgs::msg::MarkerArray>();

    for (auto &[id, marker] : this->error_segment_markers_) {
      array->markers.push_back(marker);
    }

    this->error_segments_pub_->publish(std::move(array));
    this->error_segment_markers_.clear();
  }
}

void rises::GeofenceVisualizer::publishPath() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  if (!this->path_dirty_ || !this->path_pub_)
    return;

  std::unique_ptr<visualization_msgs::msg::MarkerArray> array =
      std::make_unique<visualization_msgs::msg::MarkerArray>();
  array->markers = this->path_markers_;
  this->path_pub_->publish(std::move(array));
  this->path_dirty_ = false;
  // Clear after publishing - paths are ephemeral
  this->path_markers_.clear();
}

void rises::GeofenceVisualizer::publishAll() {
  this->publishMap();
  this->publishSafetyCircle();
  this->publishAreas();
  this->publishContours();
  this->publishSegments();
  this->publishPath();
}

void rises::GeofenceVisualizer::clear() {
  std::lock_guard<std::mutex> lock(this->mutex_);

  // Clear all obstacle markers (now a map keyed by 64-bit ID)
  this->obstacle_markers_.clear();
  this->obstacles_dirty_ = false;

  // Clear other marker vectors
  this->safety_circle_markers_.clear();
  this->safety_circles_dirty_ = false;

  this->area_markers_.clear();
  this->areas_dirty_ = false;

  this->matched_segment_markers_.clear();
  this->error_segment_markers_.clear();

  this->path_markers_.clear();
  this->path_dirty_ = false;

  // Reset next_id_ for markers that rely on it (non-obstacle markers)
  this->next_id_ = 0;

  // Reset 64→32 marker ID mapping
  this->obstacle_to_marker_id_.clear();
  this->next_marker_id_ = 1;
}

void rises::GeofenceVisualizer::clearOldObstacleUpdates() {
  // Clear all obstacle markers (now a map keyed by 64-bit ID)
  this->obstacle_markers_.clear();
  this->obstacles_dirty_ = false;
}

void rises::GeofenceVisualizer::activate() {
  // Activate publishers and check flags (with lock)
  bool should_publish_obstacles = false;
  bool should_publish_contours = false;
  bool should_add_safety_circle = false;

  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->map_pub_)
      this->map_pub_->on_activate();
    if (this->safety_circle_pub_)
      this->safety_circle_pub_->on_activate();
    if (this->area_pub_)
      this->area_pub_->on_activate();
    if (this->contour_pub_)
      this->contour_pub_->on_activate();
    if (this->matched_segments_pub_)
      this->matched_segments_pub_->on_activate();
    if (this->error_segments_pub_)
      this->error_segments_pub_->on_activate();
    if (this->path_pub_)
      this->path_pub_->on_activate();

    this->is_activated_ = true;

    // Check what needs to be published
    should_publish_obstacles = this->obstacles_dirty_;
    should_publish_contours = this->contours_dirty_;
    should_add_safety_circle = this->enable_safety_circle_;
  }
  // Lock released here - now safe to call methods that acquire the lock

  // Setup safety circle visualization if enabled
  if (should_add_safety_circle) {
    this->addSafetyCircle(this->safety_circle_radius_, "safety_circle");
    this->publishSafetyCircle();
  }

  // Publish any existing data that was loaded during configuration
  // Small delay to ensure RViz has time to subscribe (transient_local should
  // latch, but timing helps)
  if (should_publish_obstacles || should_publish_contours) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (should_publish_obstacles) {
    RCLCPP_INFO(this->node_->get_logger(),
                "Publishing %zu pre-loaded obstacle markers",
                this->obstacle_markers_.size());
    this->publishMap();
  }
  if (should_publish_contours) {
    RCLCPP_INFO(this->node_->get_logger(),
                "Publishing pre-loaded contour markers");
    this->publishContours();
  }
}

void rises::GeofenceVisualizer::deactivate() {
  std::lock_guard<std::mutex> lock(this->mutex_);

  this->is_activated_ = false;

  if (this->map_pub_)
    this->map_pub_->on_deactivate();
  if (this->safety_circle_pub_)
    this->safety_circle_pub_->on_deactivate();
  if (this->area_pub_)
    this->area_pub_->on_deactivate();
  if (this->contour_pub_)
    this->contour_pub_->on_deactivate();
  if (this->matched_segments_pub_)
    this->matched_segments_pub_->on_deactivate();
  if (this->error_segments_pub_)
    this->error_segments_pub_->on_deactivate();
  if (this->path_pub_)
    this->path_pub_->on_deactivate();
}

bool rises::GeofenceVisualizer::isActivated() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->is_activated_;
}

void rises::GeofenceVisualizer::cleanup() {
  this->clear();

  if (this->map_pub_)
    this->map_pub_.reset();
  if (this->safety_circle_pub_)
    this->safety_circle_pub_.reset();
  if (this->area_pub_)
    this->area_pub_.reset();
  if (this->contour_pub_)
    this->contour_pub_.reset();
  if (this->matched_segments_pub_)
    this->matched_segments_pub_.reset();
  if (this->error_segments_pub_)
    this->error_segments_pub_.reset();
  if (this->path_pub_)
    this->path_pub_.reset();
}

// ============================================================================
// GeofenceObserver Interface Implementation
// ============================================================================

void rises::GeofenceVisualizer::onMapChanged() {
  // Map structure changed - refresh all static visualizations
  RCLCPP_DEBUG(this->node_->get_logger(),
               "Map changed - refreshing visualization");

  // Publish updated map visualization
  this->publishMap();
}

void rises::GeofenceVisualizer::onDynamicObstacleUpdate(
    const rises_interfaces::msg::Obstacle &obstacle) {
  // Dynamic obstacle detected - add to visualization
  RCLCPP_DEBUG(this->node_->get_logger(), "Dynamic obstacle updated: %ld",
               obstacle.id);

  this->addObstacle(obstacle);
  this->publishMap();
}

void rises::GeofenceVisualizer::onPathValidationUpdate(
    const nav_msgs::msg::Path &path, bool is_safe) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  RCLCPP_DEBUG(this->node_->get_logger(),
               "Path validation result: %s (path with %zu poses)",
               is_safe ? "SAFE" : "UNSAFE", path.poses.size());

  // Clear previous path visualization
  this->path_markers_.clear();

  if (path.poses.empty()) {
    return;
  }

  // Create line strip marker for the path
  visualization_msgs::msg::Marker path_marker =
      this->createBaseMarker("validated_path", "", this->target_frame_);
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.scale.x = 0.08f; // Line width

  // Color code: Green for safe, Red for unsafe
  if (is_safe) {
    path_marker.color.r = 0.0f;
    path_marker.color.g = 1.0f;
    path_marker.color.b = 0.0f;
  } else {
    path_marker.color.r = 1.0f;
    path_marker.color.g = 0.0f;
    path_marker.color.b = 0.0f;
  }
  path_marker.color.a = 0.8f;

  // Add all path points
  for (const geometry_msgs::msg::PoseStamped &pose_stamped : path.poses) {
    geometry_msgs::msg::Point p;
    p.x = pose_stamped.pose.position.x;
    p.y = pose_stamped.pose.position.y;
    p.z = 0.05; // Slightly above ground for visibility
    path_marker.points.push_back(p);
  }

  this->path_markers_.push_back(path_marker);
  this->path_dirty_ = true;

  // Publish path visualization
  this->publishPath();
}

void rises::GeofenceVisualizer::onObstacleCorrespondence(
    const rises_interfaces::msg::Obstacle &obstacle, bool matched) {
  // Add to appropriate segment visualization
  this->processCorrespondenceResult(obstacle, matched);
}

void rises::GeofenceVisualizer::onMapBoundaryUpdate(
    const rises::shape::MapBoundaryContours &contours) {
  this->addMapBoundary(contours, "map_boundary");
  this->publishContours();
}
