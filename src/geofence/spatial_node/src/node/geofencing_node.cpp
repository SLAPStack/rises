#include "geofence/spatial/node/geofencing_node.hpp"
#include "geofence/spatial/common/scoped_timer.hpp"
#include "geofence/spatial/map/batch_operations.hpp"
#include "geofence/spatial/policies/collision_checker_selector.hpp"
#include "geofence/common/policies/coordinate_transform.hpp"
#include "geofence/spatial/policies/correspondence_matcher_selector.hpp"
#include "geofence/spatial/queries/boundary_checker.hpp"
#include "geofence/common/queries/robot_tracking_checker.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include "geofence/spatial/utils/json_loader.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sstream>
#include <stdexcept>

namespace rises {

GeofenceNode::GeofenceNode(const rclcpp::NodeOptions &options)
    : LifecycleGeofenceNodeBase("geofence_node", options) {
  RCLCPP_DEBUG(this->get_logger(), "[CONSTRUCTOR] GeofenceNode created");

  // Load all parameters via GeofenceConfig (before initCommon, which reads
  // commonConfig()).
  this->cfg_ = GeofenceConfig::fromNode(*this);

  RCLCPP_INFO(this->get_logger(),
              "Parameters: safety_circle=%s (outer_radius=%.2fm), "
              "visualizer=%s, mode=%s, frames: map='%s', base_link='%s'",
              this->cfg_.enable_safety_circle ? "enabled" : "disabled",
              this->cfg_.safety_circle_outer_radius,
              this->cfg_.visualizer_enabled ? "enabled" : "disabled",
              this->cfg_.process_points_only ? "points_only" : "shape_matching",
              this->cfg_.target_frame.c_str(),
              this->cfg_.base_link_frame.c_str());

  const std::function<std::shared_ptr<rises::geofence::SpatialIndex>()> factory =
      []() -> std::shared_ptr<rises::geofence::SpatialIndex> {
    return std::make_shared<rises::geofence::SpatialIndex>();
  };
  this->geofence_map_ =
      std::make_unique<rises::geofence::GeofenceMap>(factory);

  // Initialize safety profile
  this->safety_profile_.setOuterZone(rises::geofence::makeCircle(
      0.0f, 0.0f, this->cfg_.safety_circle_outer_radius));
  RCLCPP_INFO(
      this->get_logger(),
      "Safety profile: single circle, radius=%.2fm (inner zone: disabled)",
      this->cfg_.safety_circle_outer_radius);

  // Initialize static policy classes
  rises::geofence::PathSafetyChecker::initialize(
      rises::geofence::PathSafetyChecker::Config{});
  rises::geofence::CorrespondenceMatcher::Config matcher_config;
  rises::geofence::CorrespondenceMatcher::initialize(matcher_config);
  rises::geofence::ObstacleMatchChecker::Config match_config;
  rises::geofence::ObstacleMatchChecker::initialize(match_config);
  rises::geofence::CollisionChecker::Config collision_config;
  rises::geofence::CollisionChecker::initialize(collision_config);

  if (!this->cfg_.obstacles_json_file.empty()) {
    RCLCPP_INFO(this->get_logger(), "Will load obstacles from JSON file: %s",
                this->cfg_.obstacles_json_file.c_str());
  }
  if (!this->cfg_.contours_json_file.empty()) {
    RCLCPP_INFO(this->get_logger(), "Will load contours from JSON file: %s",
                this->cfg_.contours_json_file.c_str());
  }

  // Build the RobotTrackingChecker + auto-activate timer from the shared config.
  this->initCommon();

  RCLCPP_DEBUG(this->get_logger(),
               "[CONSTRUCTOR] GeofenceMap pointer initialized at %p",
               static_cast<void *>(this->geofence_map_.get()));
}

// ---------------- on_configure extras (spatial-only) ----------------

void GeofenceNode::onConfigureExtra() {
  // ObstacleReportBuilder uses spatial-specific segmentation parameters.
  ObstacleReportBuilder::Config report_cfg;
  report_cfg.segment_min_gap = this->cfg_.segment_min_gap;
  report_cfg.segment_gap_multiplier = this->cfg_.segment_gap_multiplier;
  report_cfg.lidar_angle_increment = this->cfg_.lidar_angle_increment;
  report_cfg.line_fit_tolerance = this->cfg_.line_fit_tolerance;
  report_cfg.min_segment_points =
      static_cast<std::size_t>(std::max(1, this->cfg_.min_segment_points));
  report_cfg.report_error_segments_as_points =
      this->cfg_.report_error_segments_as_points;
  report_cfg.enable_error_segment_tracking =
      this->cfg_.enable_error_segment_tracking;
  report_cfg.error_segment_tracker_cell_size =
      this->cfg_.error_segment_tracker_cell_size;
  report_cfg.error_segment_tracker_max_drift =
      this->cfg_.error_segment_tracker_max_drift;
  report_cfg.outlier_filter_distance = this->cfg_.outlier_filter_distance;
  report_cfg.publish_report_in_local_frame =
      this->cfg_.publish_report_in_local_frame;
  report_cfg.map_frame_id = this->map_frame_id_;
  report_cfg.robot_frame_id = this->robot_frame_id_;
  report_cfg.report_output_frame =
      (!this->cfg_.report_output_frame.empty() && !this->cfg_.tf_prefix.empty())
          ? this->cfg_.tf_prefix + "_" + this->cfg_.report_output_frame
          : this->cfg_.report_output_frame;
  this->report_builder_ = std::make_unique<ObstacleReportBuilder>(report_cfg);

  if (this->visualizer_) {
    this->visualizer_->setErrorSegmentLineMode(
        !this->cfg_.report_error_segments_as_points);
  }

  // Read-only query services (spatial-only).
  this->get_area_state_srv_ =
      this->create_service<rises_interfaces::srv::GetAreaState>(
          "get_area_state",
          [this](
              const std::shared_ptr<rmw_request_id_t>,
              const std::shared_ptr<rises_interfaces::srv::GetAreaState::Request>
                  request,
              const std::shared_ptr<
                  rises_interfaces::srv::GetAreaState::Response>
                  response) { this->getAreaStateService(request, response); });

  this->get_safety_radius_srv_ =
      this->create_service<rises_interfaces::srv::GetSafetyRadius>(
          "get_safety_radius",
          [this](const std::shared_ptr<rmw_request_id_t>,
                 const std::shared_ptr<
                     rises_interfaces::srv::GetSafetyRadius::Request>,
                 const std::shared_ptr<
                     rises_interfaces::srv::GetSafetyRadius::Response>
                     response) {
            this->getSafetyRadiusService(nullptr, response);
          });

  this->get_map_info_srv_ =
      this->create_service<rises_interfaces::srv::GetMapInfo>(
          "get_map_info",
          [this](
              const std::shared_ptr<rmw_request_id_t>,
              const std::shared_ptr<rises_interfaces::srv::GetMapInfo::Request>,
              const std::shared_ptr<rises_interfaces::srv::GetMapInfo::Response>
                  response) { this->getMapInfoService(nullptr, response); });

  this->get_warehouse_contours_srv_ =
      this->create_service<rises_interfaces::srv::GetWarehouseContours>(
          "get_warehouse_contours",
          [this](const std::shared_ptr<rmw_request_id_t>,
                 const std::shared_ptr<
                     rises_interfaces::srv::GetWarehouseContours::Request>,
                 const std::shared_ptr<
                     rises_interfaces::srv::GetWarehouseContours::Response>
                     response) {
            this->getWarehouseContoursService(nullptr, response);
          });

  // Diagnostic updater -- publishes to /diagnostics at 1 Hz.
  this->diagnostic_updater_ =
      std::make_unique<diagnostic_updater::Updater>(this, 1.0);
  this->diagnostic_updater_->setHardwareID("geofence_node");
  this->diagnostic_updater_->add("geofence_status",
                                 std::bind(&GeofenceNode::produceDiagnostics,
                                           this, std::placeholders::_1));

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

void GeofenceNode::onCleanupExtra() {
  // Reset the spatial-only handles created in onConfigureExtra so a
  // configure -> cleanup -> configure cycle does not double-register the
  // services or re-initialise the diagnostic updater / latency recorder.
  this->get_area_state_srv_.reset();
  this->get_safety_radius_srv_.reset();
  this->get_map_info_srv_.reset();
  this->get_warehouse_contours_srv_.reset();
  this->diagnostic_updater_.reset();
  this->latency_recorder_.reset();
}

void GeofenceNode::produceDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  const int64_t last_ns = this->last_scan_ns_.load(std::memory_order_relaxed);
  const int64_t now_ns = this->now().nanoseconds();
  const double age_sec =
      (last_ns > 0) ? static_cast<double>(now_ns - last_ns) / 1e9 : -1.0;
  const int64_t scans = this->scan_count_.load(std::memory_order_relaxed);
  const int64_t alerts = this->alert_count_.load(std::memory_order_relaxed);

  const std::size_t map_obstacles =
      this->geofence_map_ ? this->geofence_map_->getAllObstacleIds().size() : 0;
  const bool contours_loaded =
      this->geofence_map_ && this->geofence_map_->getMapContours() != nullptr;

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
  stat.add("map_obstacle_count", static_cast<int64_t>(map_obstacles));
  stat.add("contours_loaded", contours_loaded);
  stat.add("safety_radius_m", this->cfg_.safety_circle_outer_radius);

  const AlertLatencyStats alert_latency_stats =
      this->getAndResetAlertLatencyStats();
  stat.add("avg_alert_latency_ms", alert_latency_stats.avg_processing_ms);
  stat.add("avg_scan_to_alert_latency_ms",
           alert_latency_stats.avg_scan_to_alert_ms);
}

// ---------------- Per-scan match (spatial backend) ----------------

GeofenceNode::ScanMatchResult GeofenceNode::matchScan(
    const rises_interfaces::msg::ObstacleArray::ConstSharedPtr &msg,
    const Point2D &robot_pos, bool have_robot_pos) {
  const auto wall_start = std::chrono::steady_clock::now();
  const rclcpp::Time callback_start = this->now();
  this->last_scan_ns_.store(callback_start.nanoseconds(),
                            std::memory_order_relaxed);
  this->scan_count_.fetch_add(1, std::memory_order_relaxed);

  const float robot_heading = 0.0f;
  ScanMatchResult scan_result;

  if (this->cfg_.process_points_only) {
    rises::geofence::query::BatchPointChecker::ObstacleResult result;

    {
      rises::geofence::ScopedTimer timer("batch_point_check",
                                         this->get_logger());

      if (this->cfg_.enable_safety_circle) {
        RCLCPP_DEBUG_ONCE(
            this->get_logger(),
            "Batch point checker (profile): robot=(%.2f, %.2f, %.2frad), "
            "max_radius=%.2fm, tolerance=%.2fm, map=%zu obstacles",
            robot_pos.x, robot_pos.y, robot_heading,
            this->safety_profile_.getMaxSearchRadius(),
            this->cfg_.correspondence_tolerance,
            this->geofence_map_->getAllObstacleIds().size());

        result = rises::geofence::query::BatchPointChecker::checkObstacles(
            *this->geofence_map_, *msg, robot_pos, robot_heading,
            this->safety_profile_, this->cfg_.correspondence_tolerance,
            this->cfg_.enable_robot_filtering);
      } else {
        RCLCPP_DEBUG_ONCE(
            this->get_logger(),
            "Batch point checker (bbox): tolerance=%.2fm, map=%zu obstacles",
            this->cfg_.correspondence_tolerance,
            this->geofence_map_->getAllObstacleIds().size());

        result =
            rises::geofence::query::BatchPointChecker::checkObstaclesInBbox(
                *this->geofence_map_, *msg, this->cfg_.correspondence_tolerance,
                this->cfg_.enable_robot_filtering);
      }
    }

    // Periodic diagnostics (every 5s)
    {
      const int64_t now_ns = callback_start.nanoseconds();
      if (now_ns - this->last_diag_ns_.load(std::memory_order_relaxed) >=
          5'000'000'000LL) {
        this->last_diag_ns_.store(now_ns, std::memory_order_relaxed);

        const std::size_t map_size =
            this->geofence_map_->getAllObstacleIds().size();
        RCLCPP_DEBUG(
            this->get_logger(),
            "[MATCH DIAG] map_obstacles=%zu, robot=(%.3f, %.3f), "
            "heading=%.2frad, "
            "matched=%zu, unmatched=%zu, safety_radius=%.2fm, tolerance=%.3fm",
            map_size, robot_pos.x, robot_pos.y, robot_heading,
            result.matched_obstacle_indices.size(),
            result.unmatched_obstacle_indices.size(),
            this->cfg_.enable_safety_circle
                ? this->safety_profile_.getMaxSearchRadius()
                : 0.0f,
            this->cfg_.correspondence_tolerance);

        if (!msg->obstacles.empty() && !msg->obstacles[0].vertices.empty()) {
          const geometry_msgs::msg::Point &v0 = msg->obstacles[0].vertices[0];
          const geometry_msgs::msg::Point &vmid =
              msg->obstacles[0].vertices[msg->obstacles[0].vertices.size() / 2];
          RCLCPP_DEBUG(this->get_logger(),
                       "[MATCH DIAG] Sample scan points: p0=(%.3f, %.3f), "
                       "pmid=(%.3f, %.3f)",
                       v0.x, v0.y, vmid.x, vmid.y);
        }

        if (map_size > 0) {
          float map_min_x = std::numeric_limits<float>::max();
          float map_min_y = std::numeric_limits<float>::max();
          float map_max_x = std::numeric_limits<float>::lowest();
          float map_max_y = std::numeric_limits<float>::lowest();
          this->geofence_map_->forEachObstacle(
              [&](const geofence::GeometryEntry &entry) {
                map_min_x =
                    std::min(map_min_x, static_cast<float>(entry.bbox.min.x));
                map_min_y =
                    std::min(map_min_y, static_cast<float>(entry.bbox.min.y));
                map_max_x =
                    std::max(map_max_x, static_cast<float>(entry.bbox.max.x));
                map_max_y =
                    std::max(map_max_y, static_cast<float>(entry.bbox.max.y));
              });
          RCLCPP_DEBUG(this->get_logger(),
                       "[MATCH DIAG] Map coordinate range: X=[%.3f, %.3f], "
                       "Y=[%.3f, %.3f] "
                       "(robot at %.3f, %.3f) | dx_to_map_center=%.1f, "
                       "dy_to_map_center=%.1f",
                       map_min_x, map_max_x, map_min_y, map_max_y, robot_pos.x,
                       robot_pos.y,
                       ((map_min_x + map_max_x) / 2.0f) - robot_pos.x,
                       ((map_min_y + map_max_y) / 2.0f) - robot_pos.y);
        }
      }
    }

    // Warehouse boundary filtering is merged into BatchPointChecker's zone
    // filter (Phase 1 of checkObstacles). No second pass needed here.

    scan_result.has_unmatched = !result.unmatched_obstacle_indices.empty();
    if (scan_result.has_unmatched) {
      this->alert_count_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Detected %zu unmatched obstacles",
                           result.unmatched_obstacle_indices.size());
    }

    if (this->visualizer_) {
      for (const int64_t map_obstacle_id : result.matched_map_obstacle_ids) {
        this->visualizer_->markObstacleMatched(map_obstacle_id);
      }
    }

    scan_result.report_result = std::move(result);
  } else {
    // Shape matching classifies per-obstacle and publishes unmatched obstacles
    // directly via alertUnmatchedObstacle(); it does not populate report_result,
    // so it must not feed the per-segment ObstacleReportBuilder (matches
    // pre-unification behaviour -- otherwise an empty report is emitted each
    // scan).
    scan_result.publish_report = false;

    RCLCPP_DEBUG_ONCE(
        this->get_logger(),
        "Shape matching checker: processing full obstacle shapes");

    const bool check_robots =
        this->cfg_.enable_robot_filtering && this->robot_tracking_checker_;
    const float safety_radius_sq =
        this->cfg_.enable_safety_circle
            ? this->cfg_.safety_circle_outer_radius *
                  this->cfg_.safety_circle_outer_radius
            : 0.0f;

    for (const rises_interfaces::msg::Obstacle &obstacle : msg->obstacles) {
      if (this->cfg_.enable_safety_circle && have_robot_pos) {
        const double dx = obstacle.position.x - robot_pos.x;
        const double dy = obstacle.position.y - robot_pos.y;
        if (dx * dx + dy * dy > static_cast<double>(safety_radius_sq)) {
          continue;
        }
      }

      if (check_robots) {
        const std::string matching_robot =
            this->robot_tracking_checker_->getMatchingRobotId(obstacle);
        if (!matching_robot.empty()) {
          RCLCPP_DEBUG(this->get_logger(),
                       "Filtered obstacle %lu as robot '%s'", obstacle.id,
                       matching_robot.c_str());
          continue;
        }
      }

      const rises::geofence::ObstacleMatchChecker::MatchResult match_result =
          rises::geofence::ObstacleMatchChecker::matchDetectedObstacle(
              *this->geofence_map_, obstacle);

      if (match_result.type ==
          rises::geofence::ObstacleMatchChecker::MatchType::UNKNOWN) {
        // Skip unmatched obstacles outside warehouse
        if (this->cfg_.ignore_outside_warehouse) {
          const Point2D obs_pt{obstacle.position.x, obstacle.position.y};
          if (!rises::geofence::BoundaryChecker::isPointInsideMapBounds(
                  *this->geofence_map_, obs_pt)) {
            continue;
          }
        }
        scan_result.has_unmatched = true;
        this->alertUnmatchedObstacle(obstacle, msg->header);
        if (this->visualizer_) {
          this->visualizer_->addErrorSegment(obstacle);
        }
      } else {
        if (this->visualizer_) {
          if (match_result.type ==
              rises::geofence::ObstacleMatchChecker::MatchType::OBSTACLE) {
            this->visualizer_->markObstacleMatched(match_result.obstacle_id);
          }
          this->visualizer_->addMatchedSegment(obstacle);
        }
      }
    }

    if (scan_result.has_unmatched) {
      this->alert_count_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  // Performance + latency logging (wall clock for accurate processing time;
  // this->now() uses sim time which may be quantized to /clock publish rate).
  {
    const auto wall_end = std::chrono::steady_clock::now();
    const double processing_ms =
        std::chrono::duration<double, std::milli>(wall_end - wall_start)
            .count();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[PERF] obstacle callback: %.2f ms (wall clock)",
                         processing_ms);
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

// ---------------- Map updates ----------------

void GeofenceNode::applyMapUpdates(
    const rises_interfaces::msg::ObstacleUpdateArray &updates,
    const char *source_tag) {
  RCLCPP_DEBUG(this->get_logger(),
               "[MAP_DEBUG] [%s START] applyMapUpdates with %zu updates, "
               "map_size=%zu",
               source_tag, updates.updates.size(),
               this->geofence_map_->getAllObstacleIds().size());

  MapUpdateHandler::applyUpdates(updates, *this->geofence_map_,
                                 this->visualizer_, this->get_logger(),
                                 source_tag);
}

void GeofenceNode::setContours(
    const rises_interfaces::msg::Contours &msg) {
  if (!this->geofence_map_)
    return;

  // Log first few input points BEFORE transform
  if (!msg.outer_contour_hull.points.empty()) {
    const std::size_t n =
        std::min<std::size_t>(3, msg.outer_contour_hull.points.size());
    for (std::size_t i = 0; i < n; ++i) {
      const geometry_msgs::msg::Point32 &pt = msg.outer_contour_hull.points[i];
      RCLCPP_INFO(this->get_logger(),
                  "[CONTOUR DEBUG] Input hull[%zu]: (%.3f, %.3f)", i, pt.x,
                  pt.y);
    }
  }

  rises_interfaces::msg::Contours contours_transformed = msg;
  rises::geofence::CoordinateTransform::transformContours(contours_transformed);

  // Log first few points AFTER transform
  if (!contours_transformed.outer_contour_hull.points.empty()) {
    const std::size_t n = std::min<std::size_t>(
        3, contours_transformed.outer_contour_hull.points.size());
    for (std::size_t i = 0; i < n; ++i) {
      const geometry_msgs::msg::Point32 &pt =
          contours_transformed.outer_contour_hull.points[i];
      RCLCPP_INFO(this->get_logger(),
                  "[CONTOUR DEBUG] Transformed hull[%zu]: (%.3f, %.3f)", i,
                  pt.x, pt.y);
    }
  }

  const rises::shape::MapBoundaryContours map_contours =
      MapUpdateHandler::parseContours(contours_transformed);

  this->geofence_map_->setMapContours(map_contours);

  RCLCPP_INFO(this->get_logger(),
              "Map boundary updated with %zu outer vertices, %zu segments, and "
              "%zu inner contours",
              contours_transformed.outer_contour_hull.points.size(),
              contours_transformed.outer_contour_segments.size(),
              contours_transformed.inner_contours.size());

  if (this->visualizer_) {
    this->visualizer_->addMapBoundary(map_contours);
    this->visualizer_->publishContours();
  }
}

void GeofenceNode::onReportBuilt(
    const rises_interfaces::msg::ObstacleReport &report) {
  // Shape mode adds raw per-obstacle segments inside matchScan; only the
  // points-only path needs the report's per-segment geometry pushed to viz.
  if (!this->cfg_.process_points_only || !this->visualizer_) {
    return;
  }

  for (const rises_interfaces::msg::Obstacle &seg : report.matched_obstacles) {
    this->visualizer_->addMatchedSegment(seg);
  }
  for (const rises_interfaces::msg::Obstacle &seg :
       report.unmatched_obstacles) {
    this->visualizer_->addErrorSegment(seg);
  }
}

std::size_t GeofenceNode::backendObstacleCount() const {
  return this->geofence_map_ ? this->geofence_map_->getAllObstacleIds().size()
                             : 0;
}

void GeofenceNode::resetBackend() { this->geofence_map_.reset(); }

void GeofenceNode::setSafetyCircleRadius(float radius) {
  this->cfg_.safety_circle_outer_radius = radius;

  this->safety_profile_.setOuterZone(
      rises::geofence::makeCircle(0.0f, 0.0f, radius));
  RCLCPP_INFO(this->get_logger(), "Updated safety profile: radius=%.2fm",
              radius);

  if (this->visualizer_) {
    this->visualizer_->clearSafetyCircle();
    this->visualizer_->addSafetyCircle(radius, "safety_circle");
    this->visualizer_->publishSafetyCircle();
  }
}

// ---------------- Services ----------------

void GeofenceNode::validatePathService(
    const std::shared_ptr<rises_interfaces::srv::ValidatePath::Request> request,
    const std::shared_ptr<rises_interfaces::srv::ValidatePath::Response>
        response) {
  try {
    const bool is_safe = rises::geofence::PathSafetyChecker::isPathSafe(
        *this->geofence_map_, request->path);
    response->blocked = !is_safe;
    if (!is_safe) {
      response->reason = "Path blocked by geofence (obstacle, locked area, or "
                         "boundary violation)";
      RCLCPP_WARN(this->get_logger(),
                  "Path validation REJECTED: %zu waypoints — %s",
                  request->path.poses.size(), response->reason.c_str());
    }
  } catch (const std::exception &e) {
    response->blocked = true;
    response->reason = std::string("Path validation error: ") + e.what();
    RCLCPP_ERROR(this->get_logger(),
                 "[validatePathService] Path validation exception: %s",
                 e.what());
  }
}

void GeofenceNode::areaStateService(
    const std::shared_ptr<rises_interfaces::srv::SetAreaState::Request> request,
    const std::shared_ptr<rises_interfaces::srv::SetAreaState::Response>
        response) {
  if (!this->geofence_map_) {
    response->success = false;
    response->message = "Geofence map not initialized";
    return;
  }

  if (request->lock) {
    // Register the area geometry first so the spatial index knows about it,
    // then lock it to prevent traversal.
    const ::Rectangle area{
        {static_cast<float>(request->x0), static_cast<float>(request->y0)},
        {static_cast<float>(request->x1), static_cast<float>(request->y1)}};
    this->geofence_map_->registerArea(request->area_id, area);
    this->geofence_map_->lockArea(request->area_id);
    RCLCPP_INFO(this->get_logger(),
                "Area locked: id=%ld aabb=[(%.3f,%.3f),(%.3f,%.3f)]",
                request->area_id, request->x0, request->y0, request->x1,
                request->y1);
  } else {
    this->geofence_map_->unlockArea(request->area_id);
    RCLCPP_INFO(this->get_logger(), "Area unlocked: id=%ld", request->area_id);
  }

  response->success = true;
  response->message = request->lock ? "Area locked" : "Area unlocked";
}

void GeofenceNode::updateMapService(
    const std::shared_ptr<rises_interfaces::srv::UpdateMap::Request> request,
    const std::shared_ptr<rises_interfaces::srv::UpdateMap::Response>
        response) {
  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(),
              "[SERVICE START] updateMapService called with %zu updates",
              request->updates.updates.size());

  if (!this->geofence_map_) {
    RCLCPP_ERROR(this->get_logger(),
                 "[SERVICE ERROR] Geofence map not initialized!");
    response->success = false;
    response->message = "Geofence map not initialized";
    response->obstacles_added = 0;
    response->obstacles_removed = 0;
    return;
  }

  const MapUpdateHandler::Stats stats = MapUpdateHandler::applyUpdates(
      request->updates, *this->geofence_map_, this->visualizer_,
      this->get_logger(), "SERVICE");

  if (this->visualizer_) {
    this->visualizer_->publishMap();
  }

  response->success = true;
  response->message = "Map updated successfully";
  response->obstacles_added = stats.added;
  response->obstacles_removed = stats.removed;

  const std::chrono::time_point<std::chrono::steady_clock> end_time =
      std::chrono::steady_clock::now();
  const int64_t duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                            start_time)
          .count();

  RCLCPP_INFO(this->get_logger(), "[SERVICE COMPLETE] map_size=%zu, %ld ms",
              this->geofence_map_->getAllObstacleIds().size(), duration_ms);
}

void GeofenceNode::updateWarehouseLayoutService(
    const std::shared_ptr<rises_interfaces::srv::UpdateWarehouseLayout::Request>
        request,
    const std::shared_ptr<
        rises_interfaces::srv::UpdateWarehouseLayout::Response>
        response) {
  rises_interfaces::msg::Contours contours = request->contours;
  rises::geofence::CoordinateTransform::transformContours(contours);

  if (!this->geofence_map_) {
    response->success = false;
    response->message = "Geofence map not initialized";
    response->segments_count = 0;
    response->inner_polygons_count = 0;
    return;
  }

  if (contours.outer_contour_segments.empty()) {
    response->success = false;
    response->message = "No segments provided in warehouse contours";
    response->segments_count = 0;
    response->inner_polygons_count = 0;
    return;
  }

  const rises::shape::MapBoundaryContours map_contours =
      MapUpdateHandler::parseContours(contours);

  this->geofence_map_->setMapContours(map_contours);

  if (this->visualizer_) {
    this->visualizer_->addMapBoundary(map_contours);
    this->visualizer_->publishContours();
  }

  response->success = true;
  response->message = "Warehouse layout applied to map";
  response->segments_count =
      static_cast<int32_t>(contours.outer_contour_segments.size());
  response->inner_polygons_count =
      static_cast<int32_t>(contours.inner_contours.size());

  RCLCPP_INFO(this->get_logger(),
              "updateWarehouseLayoutService: Applied %d segments, %d inner "
              "contours to geofence map",
              response->segments_count, response->inner_polygons_count);
}

// ---------------- Query Services (spatial-only) ----------------

void GeofenceNode::getAreaStateService(
    const std::shared_ptr<rises_interfaces::srv::GetAreaState::Request> request,
    const std::shared_ptr<rises_interfaces::srv::GetAreaState::Response>
        response) {
  if (!this->geofence_map_) {
    RCLCPP_WARN(
        this->get_logger(),
        "[GeofenceNode::getAreaStateService] Geofence map not initialized");
    response->locked = false;
    response->found = false;
    response->message = "Geofence map not initialized";
    return;
  }

  const std::vector<int64_t> ids = this->geofence_map_->getAllObstacleIds();
  const bool exists =
      std::find(ids.begin(), ids.end(), request->area_id) != ids.end();

  if (!exists) {
    response->locked = false;
    response->found = false;
    response->message =
        "Area ID " + std::to_string(request->area_id) + " not found";
    return;
  }

  response->locked = this->geofence_map_->isAreaLocked(request->area_id);
  response->found = true;
  response->message = response->locked ? "Area is locked" : "Area is unlocked";
}

void GeofenceNode::getSafetyRadiusService(
    const std::shared_ptr<rises_interfaces::srv::GetSafetyRadius::Request>,
    const std::shared_ptr<rises_interfaces::srv::GetSafetyRadius::Response>
        response) {
  response->radius = this->cfg_.safety_circle_outer_radius;
}

void GeofenceNode::getMapInfoService(
    const std::shared_ptr<rises_interfaces::srv::GetMapInfo::Request>,
    const std::shared_ptr<rises_interfaces::srv::GetMapInfo::Response>
        response) {
  if (!this->geofence_map_) {
    RCLCPP_WARN(
        this->get_logger(),
        "[GeofenceNode::getMapInfoService] Geofence map not initialized");
    response->obstacle_count = 0;
    response->contours_loaded = false;
    response->contour_segment_count = 0;
    response->inner_polygon_count = 0;
    return;
  }

  response->obstacle_count =
      static_cast<int32_t>(this->geofence_map_->getAllObstacleIds().size());

  const rises::shape::MapBoundaryContours *contours =
      this->geofence_map_->getMapContours();
  response->contours_loaded = (contours != nullptr);
  if (contours) {
    response->contour_segment_count =
        static_cast<int32_t>(contours->getOuterSegments().size());
    response->inner_polygon_count =
        static_cast<int32_t>(contours->getInnerContours().size());
  } else {
    response->contour_segment_count = 0;
    response->inner_polygon_count = 0;
  }
}

void GeofenceNode::getWarehouseContoursService(
    const std::shared_ptr<rises_interfaces::srv::GetWarehouseContours::Request>,
    const std::shared_ptr<rises_interfaces::srv::GetWarehouseContours::Response>
        response) {
  if (!this->geofence_map_) {
    RCLCPP_WARN(this->get_logger(),
                "[GeofenceNode::getWarehouseContoursService] Geofence map not "
                "initialized");
    response->contours = rises_interfaces::msg::Contours{};
    return;
  }

  const rises::shape::MapBoundaryContours *contours =
      this->geofence_map_->getMapContours();
  if (!contours) {
    RCLCPP_INFO(
        this->get_logger(),
        "[GeofenceNode::getWarehouseContoursService] No contours loaded");
    response->contours = rises_interfaces::msg::Contours{};
    return;
  }

  rises_interfaces::msg::Contours &contours_msg = response->contours;

  // Convert outer segments (walls)
  for (const rises::shape::LineSegment2D &seg : contours->getOuterSegments()) {
    geometry_msgs::msg::Point32 start_pt, end_pt;
    start_pt.x = static_cast<float>(seg.start.x);
    start_pt.y = static_cast<float>(seg.start.y);
    start_pt.z = 0.0f;
    end_pt.x = static_cast<float>(seg.end.x);
    end_pt.y = static_cast<float>(seg.end.y);
    end_pt.z = 0.0f;

    rises_interfaces::msg::LineSegment line_seg;
    line_seg.start = start_pt;
    line_seg.end = end_pt;
    contours_msg.outer_contour_segments.push_back(line_seg);
  }

  // Convert outer hull (for visualization/containment)
  for (const Point2D &pt : contours->getOuterContour().getVertices()) {
    geometry_msgs::msg::Point32 ros_pt;
    ros_pt.x = static_cast<float>(pt.x);
    ros_pt.y = static_cast<float>(pt.y);
    ros_pt.z = 0.0f;
    contours_msg.outer_contour_hull.points.push_back(ros_pt);
  }

  // Convert inner contours (holes)
  for (const rises::shape::PolygonContour &inner :
       contours->getInnerContours()) {
    geometry_msgs::msg::Polygon polygon;
    for (const Point2D &pt : inner.getVertices()) {
      geometry_msgs::msg::Point32 ros_pt;
      ros_pt.x = static_cast<float>(pt.x);
      ros_pt.y = static_cast<float>(pt.y);
      ros_pt.z = 0.0f;
      polygon.points.push_back(ros_pt);
    }
    contours_msg.inner_contours.push_back(polygon);
  }

  RCLCPP_INFO(this->get_logger(),
              "[GeofenceNode::getWarehouseContoursService] Returned %zu "
              "segments, %zu hull points, %zu inner polygons",
              contours_msg.outer_contour_segments.size(),
              contours_msg.outer_contour_hull.points.size(),
              contours_msg.inner_contours.size());
}

// ---------------- Alerts ----------------

void GeofenceNode::alertUnmatchedObstacle(
    const rises_interfaces::msg::Obstacle &obstacle,
    const std_msgs::msg::Header &detection_header) {
  if (!this->unmatched_obstacle_pub_ ||
      !this->unmatched_obstacle_pub_->is_activated())
    return;

  const rclcpp::Time scan_time(detection_header.stamp);
  const rclcpp::Time current_time = this->now();
  const double latency_ms = (current_time - scan_time).seconds() * 1000.0;

  std::unique_ptr<rises_interfaces::msg::ObstacleArray> msg =
      std::make_unique<rises_interfaces::msg::ObstacleArray>();
  msg->header = detection_header;
  msg->obstacles.push_back(obstacle);

  RCLCPP_WARN(this->get_logger(), "DANGER: Unmatched obstacle %lu detected",
              obstacle.id);
  RCLCPP_DEBUG(this->get_logger().get_child("perf"),
               "[PERF] laserscan→unmatched_obstacle: %.2f ms (obstacle %lu)",
               latency_ms, obstacle.id);

  this->unmatched_obstacle_pub_->publish(std::move(msg));
}

// ---------------- Pre-initialization from JSON files ----------------

bool GeofenceNode::loadObstaclesFromJson(const std::string &filepath) {
  if (!this->geofence_map_) {
    RCLCPP_ERROR(this->get_logger(),
                 "[loadObstaclesFromJson] Cannot load obstacles: geofence_map_ "
                 "not initialized");
    return false;
  }

  try {
    const std::vector<rises_interfaces::msg::Obstacle> obstacles =
        rises::geofence::utils::JsonLoader::loadObstacles(filepath, true);

    std::vector<rises::geofence::BatchOperations::ObstaclePair> batch;
    batch.reserve(obstacles.size());

    for (const rises_interfaces::msg::Obstacle &obs_msg : obstacles) {
      rises::geofence::Geometry shape =
          rises::geofence::fromObstacleMsg(obs_msg);
      batch.emplace_back(obs_msg.id, std::move(shape));
      if (this->visualizer_) {
        this->visualizer_->addObstacle(obs_msg);
      }
    }

    const std::size_t inserted = rises::geofence::BatchOperations::insertBatch(
        *this->geofence_map_, std::move(batch));

    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu/%zu obstacles from JSON file: %s", inserted,
                obstacles.size(), filepath.c_str());

    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "[loadObstaclesFromJson] Error loading obstacles JSON: %s",
                 e.what());
    return false;
  }
}

bool GeofenceNode::loadContoursFromJson(const std::string &filepath) {
  if (!this->geofence_map_) {
    RCLCPP_ERROR(this->get_logger(), "[loadContoursFromJson] Cannot load "
                                     "contours: geofence_map_ not initialized");
    return false;
  }

  try {
    const shape::MapBoundaryContours contours =
        rises::geofence::utils::JsonLoader::loadContours(filepath, true);

    this->geofence_map_->setMapContours(contours);

    if (this->visualizer_) {
      this->visualizer_->addMapBoundary(contours);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded contours from JSON file: %s",
                filepath.c_str());
    RCLCPP_INFO(
        this->get_logger(),
        "  Outer hull vertices: %zu, Outer segments: %zu, Inner contours: %zu",
        contours.getOuterContour().getVertices().size(),
        contours.getOuterSegments().size(), contours.getInnerContours().size());
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "[loadContoursFromJson] Error loading contours JSON: %s",
                 e.what());
    return false;
  }
}

} // namespace rises

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rises::GeofenceNode)
