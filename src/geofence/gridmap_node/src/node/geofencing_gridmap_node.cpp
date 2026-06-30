// =============================================================================
// GEOFENCE GRIDMAP NODE - backend implementation on LifecycleGeofenceNodeBase
//
// The base (LifecycleGeofenceNodeBase) owns the shared lifecycle scaffolding:
// TF2, the auto-activate timer, subscriptions/publishers/services, the per-scan
// callback skeleton (robot pose -> matchScan -> single alert publish -> report
// build/publish -> throttled viz), JSON-load orchestration, and the coordinate
// transform apply in on_activate.
//
// This file provides the gridmap-specific backend:
//   matchScan          - safety-circle pre-filter + O(1) grid lookup per
//                        obstacle (checkCorrespondence) for the fast alert, plus
//                        per-vertex classification (buildMatchResult) for the
//                        per-segment ObstacleReport.
//   applyMapUpdates    - batch RCU snapshot update (single copy-on-write).
//   loadObstaclesFromJson / service handlers - gridmap-specific.
// =============================================================================

#include "geofence/gridmap/node/geofencing_gridmap_node.hpp"
#include "geofence/gridmap/policies/gridmap_policies.hpp"
#include "geofence/spatial/node/obstacle_report_builder.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include "geofence/spatial/utils/json_loader.hpp"
#include "geofence/utils/compiler_hints.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace rises {

// =============================================================================
// Constructor
// =============================================================================

GeofenceGridmapNode::GeofenceGridmapNode(const rclcpp::NodeOptions &options)
    : LifecycleGeofenceNodeBase("geofence_gridmap_node", options) {
  RCLCPP_DEBUG(this->get_logger(), "[CONSTRUCTOR] GeofenceGridmapNode created");

  // Load all parameters (shared + grid) before initCommon, which reads
  // commonConfig().
  this->cfg_ = GridmapConfig::fromNode(*this);

  const std::size_t grid_cells = static_cast<std::size_t>(
      std::ceil(this->cfg_.grid_width_meters / this->cfg_.grid_resolution) *
      std::ceil(this->cfg_.grid_height_meters / this->cfg_.grid_resolution));
  const std::size_t memory_mb = (grid_cells * sizeof(bool)) / (1024UL * 1024UL);

  RCLCPP_INFO(this->get_logger(),
              "Gridmap configuration: %.0fm x %.0fm @ %.2fcm resolution = %zu "
              "cells (~%zu MB)",
              this->cfg_.grid_width_meters, this->cfg_.grid_height_meters,
              this->cfg_.grid_resolution * 100.0, grid_cells, memory_mb);

  // Build gridmap.
  geofence::GridMap::Config grid_cfg;
  grid_cfg.resolution = this->cfg_.grid_resolution;
  grid_cfg.width_meters = this->cfg_.grid_width_meters;
  grid_cfg.height_meters = this->cfg_.grid_height_meters;
  grid_cfg.origin_x = this->cfg_.grid_origin_x;
  grid_cfg.origin_y = this->cfg_.grid_origin_y;
  this->gridmap_ = std::make_shared<geofence::GridMap>(grid_cfg);

  RCLCPP_INFO(this->get_logger(),
              "Parameters: safety_circle=%s (radius=%.2fm), visualizer=%s, "
              "frames: map='%s' base_link='%s'",
              this->cfg_.enable_safety_circle ? "enabled" : "disabled",
              this->cfg_.safety_circle_outer_radius,
              this->cfg_.visualizer_enabled ? "enabled" : "disabled",
              this->cfg_.target_frame.c_str(),
              this->cfg_.base_link_frame.c_str());

  // Build the RobotTrackingChecker + auto-activate timer from the shared config.
  this->initCommon();

  RCLCPP_DEBUG(this->get_logger(), "[CONSTRUCTOR] Gridmap initialised");
}

// =============================================================================
// on_configure extension: build the shared ObstacleReportBuilder
// =============================================================================
//
// The base declares report_builder_ but leaves construction to each backend
// (the spatial builder is configured from spatial-only segmentation params the
// gridmap config does not have). Without this, the base's report path is
// silently skipped and the gridmap node publishes NO obstacle_report -- a KPI
// pipeline blackout. The base has already built map_frame_id_/robot_frame_id_
// before calling this hook. Uses only the shared framing fields; segmentation
// fields keep their defaults since the gridmap's per-vertex classification
// comes from buildMatchResult, not lidar re-segmentation.
void GeofenceGridmapNode::onConfigureExtra() {
  ObstacleReportBuilder::Config report_cfg;
  report_cfg.map_frame_id = this->map_frame_id_;
  report_cfg.robot_frame_id = this->robot_frame_id_;
  report_cfg.publish_report_in_local_frame =
      this->cfg_.publish_report_in_local_frame;
  report_cfg.report_output_frame = this->cfg_.report_output_frame;
  this->report_builder_ = std::make_unique<ObstacleReportBuilder>(report_cfg);

  // Diagnostic updater -- publishes to /diagnostics at 1 Hz.
  this->diagnostic_updater_ =
      std::make_unique<diagnostic_updater::Updater>(this, 1.0);
  this->diagnostic_updater_->setHardwareID("geofence_gridmap_node");
  this->diagnostic_updater_->add(
      "geofence_status",
      std::bind(&GeofenceGridmapNode::produceDiagnostics, this,
                std::placeholders::_1));

  // Latency recorder (optional).
  const std::string latency_file =
      this->get_parameter("latency_output_file").as_string();
  if (!latency_file.empty()) {
    this->latency_recorder_ =
        rises::geofence::LatencyRecorder::create(latency_file);
    if (this->latency_recorder_) {
      RCLCPP_INFO(this->get_logger(), "Latency recording enabled: %s",
                  latency_file.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "[onConfigureExtra] Failed to open latency output file: %s",
                   latency_file.c_str());
    }
  }
}

void GeofenceGridmapNode::onCleanupExtra() {
  this->diagnostic_updater_.reset();
  this->latency_recorder_.reset();
}

void GeofenceGridmapNode::produceDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  const int64_t last_ns = this->last_scan_ns_.load(std::memory_order_relaxed);
  const int64_t now_ns = this->now().nanoseconds();
  const double age_sec =
      (last_ns > 0) ? static_cast<double>(now_ns - last_ns) / 1e9 : -1.0;
  const int64_t scans = this->scan_count_.load(std::memory_order_relaxed);
  const int64_t alerts = this->alert_count_.load(std::memory_order_relaxed);

  if (age_sec < 0.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "No scans received yet");
  } else if (age_sec > 5.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Scan data stale (last scan " + std::to_string(age_sec) +
                     "s ago)");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Geofence operational");
  }

  stat.add("last_scan_age_sec", age_sec);
  stat.add("total_scans_processed", scans);
  stat.add("total_obstacle_alerts", alerts);
  stat.add("safety_radius_m", this->cfg_.safety_circle_outer_radius);

  const AlertLatencyStats alert_latency_stats =
      this->getAndResetAlertLatencyStats();
  stat.add("avg_alert_latency_ms", alert_latency_stats.avg_processing_ms);
  stat.add("avg_scan_to_alert_latency_ms",
           alert_latency_stats.avg_scan_to_alert_ms);
}

// =============================================================================
// Per-scan match (occupancy-grid backend)
// =============================================================================

GeofenceGridmapNode::ScanMatchResult GeofenceGridmapNode::matchScan(
    const rises_interfaces::msg::ObstacleArray::ConstSharedPtr &msg,
    const Point2D &robot_pos, bool /*have_pos*/) {
  const rclcpp::Time callback_start = this->now();
  this->last_scan_ns_.store(callback_start.nanoseconds(),
                            std::memory_order_relaxed);
  this->scan_count_.fetch_add(1, std::memory_order_relaxed);

  std::size_t matched_count = 0;
  std::size_t unmatched_count = 0;

  for (const rises_interfaces::msg::Obstacle &obstacle : msg->obstacles) {
    // Multi-robot filter.
    if (this->cfg_.enable_robot_filtering && this->robot_tracking_checker_) {
      if (!this->robot_tracking_checker_->getMatchingRobotId(obstacle)
               .empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Filtered obstacle %lu as known robot",
                     obstacle.id);
        continue;
      }
    }

    const bool matched = this->checkCorrespondence(obstacle, robot_pos);
    if (matched) {
      ++matched_count;
      if (this->visualizer_)
        this->visualizer_->addMatchedSegment(obstacle);
    } else {
      ++unmatched_count;
      if (this->visualizer_)
        this->visualizer_->addErrorSegment(obstacle);
    }
  }

  ScanMatchResult scan_result;
  scan_result.has_unmatched = (unmatched_count > 0);
  if (scan_result.has_unmatched) {
    this->alert_count_.fetch_add(1, std::memory_order_relaxed);
  }

  // Per-vertex classification for the per-segment ObstacleReport (real
  // positions): the single detection representation consumed by validation /
  // FIWARE / fleet. The per-vertex pass subsumes the cheap per-obstacle
  // checkCorrespondence above, which is kept only for the fast per-scan alert.
  scan_result.report_result = this->buildMatchResult(msg, robot_pos);

  // Diagnostics (every 5 s).
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[MATCH DIAG] robot=(%.3f, %.3f) matched=%zu unmatched=%zu "
      "safety_r=%.2fm",
      robot_pos.x, robot_pos.y, matched_count, unmatched_count,
      this->cfg_.enable_safety_circle
          ? static_cast<double>(this->cfg_.safety_circle_outer_radius)
          : 0.0);

  // Performance logging.
  {
    const rclcpp::Time callback_end = this->now();
    const double processing_ms =
        (callback_end - callback_start).seconds() * 1000.0;
    const rclcpp::Time msg_time(msg->header.stamp);
    const double total_latency_ms =
        (callback_end - msg_time).seconds() * 1000.0;
    RCLCPP_DEBUG(this->get_logger().get_child("perf"),
                 "[PERF] obstacle_array processing: %.2f ms (callback), %.2f "
                 "ms (total from scan)",
                 processing_ms, total_latency_ms);
  }

  if (this->latency_recorder_) {
    const rclcpp::Time msg_time(msg->header.stamp);
    const int64_t scan_ns = msg_time.nanoseconds();
    const int64_t start_ns = callback_start.nanoseconds();
    const int64_t end_ns = this->now().nanoseconds();
    const int32_t matched = static_cast<int32_t>(
        scan_result.report_result.matched_obstacle_indices.size());
    const int32_t unmatched = static_cast<int32_t>(
        scan_result.report_result.unmatched_obstacle_indices.size());
    this->latency_recorder_->record(scan_ns, start_ns, end_ns, matched,
                                    unmatched);
  }

  return scan_result;
}

// =============================================================================
// Map updates
// =============================================================================

void GeofenceGridmapNode::applyMapUpdates(
    const rises_interfaces::msg::ObstacleUpdateArray &updates,
    const char *source_tag) {
  // Batch all inserts/removes into a SINGLE copy-on-write snapshot. This avoids
  // N individual RCU copies when processing a large map update.
  this->gridmap_->updateSnapshot([&](geofence::GridMapData &data) {
    for (const rises_interfaces::msg::ObstacleUpdate &update :
         updates.updates) {
      if (update.operation ==
          rises_interfaces::msg::ObstacleUpdate::OP_INSERT) {
        rises_interfaces::msg::Obstacle obs = update.obstacle;
        rises::geofence::CoordinateTransform::transformObstacle(obs);
        // STATIC layer: dynamic map updates arriving over topic/service.
        geofence::ObstacleInsertionPolicy::insert(
            data, obs.id, obs, this->cfg_.obstacle_inflation_radius,
            geofence::ObstacleLayer::STATIC);
        if (this->visualizer_)
          this->visualizer_->addObstacle(obs);
      } else if (update.operation ==
                 rises_interfaces::msg::ObstacleUpdate::OP_DELETE) {
        geofence::ObstacleRemovalPolicy::remove(data, update.obstacle.id);
        if (this->visualizer_)
          this->visualizer_->removeObstacle(update.obstacle.id);
      }
    }
  });

  RCLCPP_DEBUG(this->get_logger(),
               "[MAP_DEBUG] applyMapUpdates: processed %zu updates via %s "
               "(single RCU snapshot)",
               updates.updates.size(), source_tag);
}

void GeofenceGridmapNode::resetBackend() {
  if (this->gridmap_)
    this->gridmap_->clear();
}

// =============================================================================
// Service handlers
// =============================================================================

void GeofenceGridmapNode::validatePathService(
    const std::shared_ptr<rises_interfaces::srv::ValidatePath::Request> request,
    const std::shared_ptr<rises_interfaces::srv::ValidatePath::Response>
        response) {
  if (!this->gridmap_) {
    response->blocked = true;
    return;
  }

  response->blocked = false;
  for (std::size_t i = 1; i < request->path.poses.size(); ++i) {
    const auto &p1 = request->path.poses[i - 1].pose.position;
    const auto &p2 = request->path.poses[i].pose.position;
    if (this->gridmap_->isPathBlocked(p1.x, p1.y, p2.x, p2.y,
                                      geofence::ObstacleLayer::FIXED |
                                          geofence::ObstacleLayer::STATIC)) {
      response->blocked = true;
      break;
    }
  }
}

void GeofenceGridmapNode::areaStateService(
    const std::shared_ptr<rises_interfaces::srv::SetAreaState::Request>
    /*request*/,
    const std::shared_ptr<rises_interfaces::srv::SetAreaState::Response>
        response) {
  // The gridmap has no concept of named lockable areas.
  response->success = false;
  response->message =
      "Area locking is not supported by the gridmap implementation";
}

void GeofenceGridmapNode::updateMapService(
    const std::shared_ptr<rises_interfaces::srv::UpdateMap::Request> request,
    const std::shared_ptr<rises_interfaces::srv::UpdateMap::Response>
        response) {
  const auto start_time = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(),
              "[SERVICE START] updateMapService: %zu updates",
              request->updates.updates.size());

  if (!this->gridmap_) {
    response->success = false;
    response->message = "Gridmap not initialised";
    response->obstacles_added = 0;
    response->obstacles_removed = 0;
    return;
  }

  int32_t added = 0, removed = 0;

  // Batch all service inserts/removes into a SINGLE RCU snapshot.
  this->gridmap_->updateSnapshot([&](geofence::GridMapData &data) {
    for (const rises_interfaces::msg::ObstacleUpdate &update :
         request->updates.updates) {
      if (update.operation ==
          rises_interfaces::msg::ObstacleUpdate::OP_INSERT) {
        rises_interfaces::msg::Obstacle obs = update.obstacle;
        rises::geofence::CoordinateTransform::transformObstacle(obs);
        // STATIC layer: service-provided map updates.
        geofence::ObstacleInsertionPolicy::insert(
            data, obs.id, obs, this->cfg_.obstacle_inflation_radius,
            geofence::ObstacleLayer::STATIC);
        ++added;
        if (this->visualizer_)
          this->visualizer_->addObstacle(obs);
      } else if (update.operation ==
                 rises_interfaces::msg::ObstacleUpdate::OP_DELETE) {
        geofence::ObstacleRemovalPolicy::remove(data, update.obstacle.id);
        ++removed;
        if (this->visualizer_)
          this->visualizer_->removeObstacle(update.obstacle.id);
      }
    }
  });

  if (this->visualizer_)
    this->visualizer_->publishMap();

  const int64_t duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_time)
          .count();

  response->success = true;
  response->message = "Map updated successfully";
  response->obstacles_added = added;
  response->obstacles_removed = removed;

  RCLCPP_INFO(
      this->get_logger(),
      "[MAP_DEBUG] [SERVICE COMPLETE] +%d obstacles, -%d obstacles in %ld ms",
      added, removed, duration_ms);
}

void GeofenceGridmapNode::updateWarehouseLayoutService(
    const std::shared_ptr<rises_interfaces::srv::UpdateWarehouseLayout::Request>
        request,
    const std::shared_ptr<
        rises_interfaces::srv::UpdateWarehouseLayout::Response>
        response) {
  if (request->contours.outer_contour_segments.empty()) {
    response->success = false;
    response->message = "No segments provided";
    response->segments_count = 0;
    response->inner_polygons_count = 0;
    return;
  }

  // Gridmap does not apply vector contours (see setContours comment in base).
  response->success = true;
  response->message =
      "Warehouse layout received (contours not rasterised into gridmap)";
  response->segments_count =
      static_cast<int32_t>(request->contours.outer_contour_segments.size());
  response->inner_polygons_count =
      static_cast<int32_t>(request->contours.inner_contours.size());

  RCLCPP_WARN(this->get_logger(),
              "updateWarehouseLayoutService: received %d segments, %d inner "
              "contours (not applied)",
              response->segments_count, response->inner_polygons_count);
}

// =============================================================================
// Correspondence checks
// =============================================================================

bool GeofenceGridmapNode::checkCorrespondence(
    const rises_interfaces::msg::Obstacle &obs,
    const Point2D &robot_pos) const {
  // ---- 1. Safety-circle pre-filter ----------------------------------------
  // Reject obstacles that don't intersect the robot's safety circle so distant
  // obstacles are never queried against the grid (cheap early-out).
  if (this->cfg_.enable_safety_circle) {
    bool in_circle = false;
    const double r = this->cfg_.safety_circle_outer_radius;
    const double r2 = r * r;
    const double rx = robot_pos.x;
    const double ry = robot_pos.y;

    switch (obs.type) {
    case rises_interfaces::msg::Obstacle::POINT: {
      const double dx = obs.position.x - rx, dy = obs.position.y - ry;
      in_circle = (dx * dx + dy * dy <= r2);
      break;
    }
    case rises_interfaces::msg::Obstacle::CIRCLE: {
      const double dx = obs.position.x - rx, dy = obs.position.y - ry;
      const double d = std::sqrt(dx * dx + dy * dy);
      in_circle = (d <= r + obs.radius);
      break;
    }
    case rises_interfaces::msg::Obstacle::LINE: {
      if (obs.vertices.size() >= 2) {
        const geometry_msgs::msg::Point &p1 = obs.vertices[0];
        const geometry_msgs::msg::Point &p2 = obs.vertices[1];
        auto pt_in = [&](const double px, const double py) {
          const double dx = px - rx, dy = py - ry;
          return (dx * dx + dy * dy <= r2);
        };
        if (pt_in(p1.x, p1.y) || pt_in(p2.x, p2.y)) {
          in_circle = true;
        } else {
          const double ldx = p2.x - p1.x, ldy = p2.y - p1.y;
          const double llen2 = ldx * ldx + ldy * ldy;
          if (llen2 > 1e-8) {
            const double t = std::max(
                0.0,
                std::min(1.0, ((rx - p1.x) * ldx + (ry - p1.y) * ldy) / llen2));
            const double cx = p1.x + t * ldx - rx;
            const double cy = p1.y + t * ldy - ry;
            in_circle = (cx * cx + cy * cy <= r2);
          }
        }
      }
      break;
    }
    case rises_interfaces::msg::Obstacle::POLYGON:
    case rises_interfaces::msg::Obstacle::RECTANGLE: {
      for (const geometry_msgs::msg::Point &v : obs.vertices) {
        const double dx = v.x - rx, dy = v.y - ry;
        if (dx * dx + dy * dy <= r2) {
          in_circle = true;
          break;
        }
      }
      if (!in_circle && obs.vertices.size() >= 3) {
        // Point-in-polygon test: odd crossing count means robot centre is
        // inside.
        int cross = 0;
        for (std::size_t i = 0; i < obs.vertices.size(); ++i) {
          const geometry_msgs::msg::Point &v1 = obs.vertices[i];
          const geometry_msgs::msg::Point &v2 =
              obs.vertices[(i + 1) % obs.vertices.size()];
          if (((v1.y <= ry && ry < v2.y) || (v2.y <= ry && ry < v1.y)) &&
              (rx < (v2.x - v1.x) * (ry - v1.y) / (v2.y - v1.y) + v1.x))
            ++cross;
        }
        in_circle = (cross % 2 == 1);
      }
      break;
    }
    default: {
      const double dx = obs.position.x - rx, dy = obs.position.y - ry;
      in_circle = (dx * dx + dy * dy <= r2);
      break;
    }
    }
    if (!in_circle)
      return false; // Outside safety circle - not a concern
  }

  // ---- 2. Grid lookup (O(1) per sample point) ------------------------------
  switch (obs.type) {
  case rises_interfaces::msg::Obstacle::POINT:
    return this->gridmap_->isOccupied(obs.position.x, obs.position.y,
                                      geofence::ObstacleLayer::FIXED |
                                          geofence::ObstacleLayer::STATIC);

  case rises_interfaces::msg::Obstacle::CIRCLE: {
    constexpr int kSamples = 16;
    for (int i = 0; i < kSamples; ++i) {
      const double a = 2.0 * M_PI * i / kSamples;
      if (this->gridmap_->isOccupied(obs.position.x + obs.radius * std::cos(a),
                                     obs.position.y + obs.radius * std::sin(a),
                                     geofence::ObstacleLayer::FIXED |
                                         geofence::ObstacleLayer::STATIC))
        return true;
    }
    break;
  }
  case rises_interfaces::msg::Obstacle::LINE: {
    if (obs.vertices.size() >= 2) {
      const geometry_msgs::msg::Point &p1 = obs.vertices[0];
      const geometry_msgs::msg::Point &p2 = obs.vertices[1];
      constexpr int kSamples = 10;
      for (int i = 0; i <= kSamples; ++i) {
        const double t = static_cast<double>(i) / kSamples;
        if (this->gridmap_->isOccupied(p1.x + t * (p2.x - p1.x),
                                       p1.y + t * (p2.y - p1.y),
                                       geofence::ObstacleLayer::FIXED |
                                           geofence::ObstacleLayer::STATIC))
          return true;
      }
    }
    break;
  }
  case rises_interfaces::msg::Obstacle::POLYGON:
  case rises_interfaces::msg::Obstacle::RECTANGLE: {
    for (const geometry_msgs::msg::Point &v : obs.vertices)
      if (this->gridmap_->isOccupied(v.x, v.y,
                                     geofence::ObstacleLayer::FIXED |
                                         geofence::ObstacleLayer::STATIC))
        return true;
    // Sample edges to catch thin obstacles that have no vertex on a rasterised
    // cell.
    constexpr int kEdgeSamples = 5;
    for (std::size_t i = 0; i < obs.vertices.size(); ++i) {
      const geometry_msgs::msg::Point &v1 = obs.vertices[i];
      const geometry_msgs::msg::Point &v2 =
          obs.vertices[(i + 1) % obs.vertices.size()];
      for (int j = 1; j < kEdgeSamples; ++j) {
        const double t = static_cast<double>(j) / kEdgeSamples;
        if (this->gridmap_->isOccupied(v1.x + t * (v2.x - v1.x),
                                       v1.y + t * (v2.y - v1.y),
                                       geofence::ObstacleLayer::FIXED |
                                           geofence::ObstacleLayer::STATIC))
          return true;
      }
    }
    break;
  }
  default:
    break;
  }
  return false;
}

rises::geofence::query::ObstacleMatchResult
GeofenceGridmapNode::buildMatchResult(
    const rises_interfaces::msg::ObstacleArray::ConstSharedPtr &msg,
    const Point2D &robot_pos) const {
  rises::geofence::query::ObstacleMatchResult result;
  const std::size_t obstacle_count = msg->obstacles.size();
  result.matched_vertices_per_obstacle.resize(obstacle_count);
  result.unmatched_vertices_per_obstacle.resize(obstacle_count);

  const bool gate = this->cfg_.enable_safety_circle;
  const double r2 = static_cast<double>(this->cfg_.safety_circle_outer_radius) *
                    static_cast<double>(this->cfg_.safety_circle_outer_radius);

  for (std::size_t oi = 0; oi < obstacle_count; ++oi) {
    const rises_interfaces::msg::Obstacle &obs = msg->obstacles[oi];
    // Multi-robot filter: a known robot is neither a matched map obstacle nor
    // an unmatched intruder, so leave both vertex lists empty.
    if (this->cfg_.enable_robot_filtering && this->robot_tracking_checker_ &&
        !this->robot_tracking_checker_->getMatchingRobotId(obs).empty()) {
      continue;
    }
    bool any_matched = false;
    bool any_unmatched = false;
    for (std::size_t vi = 0; vi < obs.vertices.size(); ++vi) {
      const geometry_msgs::msg::Point &v = obs.vertices[vi];
      if (gate) {
        const double dx = v.x - robot_pos.x;
        const double dy = v.y - robot_pos.y;
        if (dx * dx + dy * dy > r2)
          continue; // outside safety circle: ignored, not counted as unmatched
      }
      if (this->gridmap_->isOccupied(v.x, v.y,
                                     geofence::ObstacleLayer::FIXED |
                                         geofence::ObstacleLayer::STATIC)) {
        result.matched_vertices_per_obstacle[oi].push_back(vi);
        any_matched = true;
      } else {
        result.unmatched_vertices_per_obstacle[oi].push_back(vi);
        any_unmatched = true;
      }
    }
    if (any_unmatched)
      result.unmatched_obstacle_indices.push_back(oi);
    else if (any_matched)
      result.matched_obstacle_indices.push_back(oi);
  }
  return result;
}

// =============================================================================
// JSON pre-initialisation
// =============================================================================

bool GeofenceGridmapNode::loadObstaclesFromJson(const std::string &filepath) {
  if (!this->gridmap_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Cannot load JSON: gridmap not initialised");
    return false;
  }
  try {
    // JsonLoader::loadObstacles() is header-only (json_loader_impl.hpp) and
    // returns rises_interfaces Obstacle messages - no spatial-index types.
    const std::vector<rises_interfaces::msg::Obstacle> obstacles =
        rises::geofence::utils::JsonLoader::loadObstacles(
            filepath, /*apply_transform=*/true);

    // Insert the whole batch in a single RCU snapshot (FIXED layer: loaded from
    // JSON).
    this->gridmap_->updateSnapshot([&](geofence::GridMapData &data) {
      for (const rises_interfaces::msg::Obstacle &obs : obstacles) {
        geofence::ObstacleInsertionPolicy::insert(
            data, obs.id, obs, this->cfg_.obstacle_inflation_radius,
            geofence::ObstacleLayer::FIXED);
        if (this->visualizer_)
          this->visualizer_->addObstacle(obs);
      }
    });

    RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacles from JSON file: %s",
                obstacles.size(), filepath.c_str());
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading obstacles JSON: %s",
                 e.what());
    return false;
  }
}

bool GeofenceGridmapNode::loadContoursFromJson(const std::string &filepath) {
  try {
    const rises::shape::MapBoundaryContours contours =
        rises::geofence::utils::JsonLoader::loadContours(filepath, true);

    if (this->visualizer_) {
      this->visualizer_->addMapBoundary(contours);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded contours from JSON file: %s",
                filepath.c_str());
    RCLCPP_INFO(this->get_logger(),
                "  Outer hull vertices: %zu, Inner contours: %zu",
                contours.getOuterContour().getVertices().size(),
                contours.getInnerContours().size());
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading contours JSON: %s",
                 e.what());
    return false;
  }
}

} // namespace rises

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rises::GeofenceGridmapNode)
