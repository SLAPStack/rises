#include "geofence/common/node/lifecycle_geofence_node_base.hpp"

#include <algorithm>
#include <chrono>
#include <lifecycle_msgs/msg/state.hpp>
#include <utility>

namespace rises {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

LifecycleGeofenceNodeBase::LifecycleGeofenceNodeBase(
    const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode(node_name, options) {
  // Config-dependent setup (RobotTrackingChecker, auto-activate timer) CANNOT
  // run here: commonConfig() is a pure virtual that is not yet valid during
  // base construction. The derived ctor loads its config then calls
  // initCommon(). Only the backend-agnostic TF2 setup belongs here.
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
      this->get_clock(), tf2::durationFromSec(10.0));
  this->tf_buffer_->setUsingDedicatedThread(true);
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *this->tf_buffer_, this, false,
      rclcpp::QoS(100).reliable().durability_volatile(),
      rclcpp::QoS(100).reliable().transient_local());
}

void LifecycleGeofenceNodeBase::initCommon() {
  const GeofenceCommonConfig &cfg = this->commonConfig();

  if (cfg.enable_robot_filtering) {
    rises::geofence::RobotTrackingChecker::Config robot_config;
    robot_config.max_pose_age_ms =
        static_cast<uint64_t>(cfg.robot_pose_max_age_ms);
    robot_config.footprint_expansion_margin = cfg.default_robot_footprint_margin;

    this->robot_tracking_checker_ =
        std::make_unique<rises::geofence::RobotTrackingChecker>(robot_config);

    for (const std::string &robot_id : cfg.robot_ids) {
      const rises::geofence::policies::RobotFootprint footprint =
          this->createRobotFootprint(robot_id);
      this->robot_tracking_checker_->registerRobot(robot_id, footprint);
      RCLCPP_INFO(this->get_logger(), "Registered robot '%s' with %s footprint",
                  robot_id.c_str(),
                  getFootprintTypeString(footprint.type).c_str());
    }
    RCLCPP_INFO(this->get_logger(),
                "Multi-robot filtering enabled for %zu robots",
                cfg.robot_ids.size());
  }

  if (cfg.auto_activate) {
    this->auto_transition_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { this->auto_transition(); });
    RCLCPP_INFO(this->get_logger(),
                "Auto-activation enabled - node will automatically configure "
                "and activate");
  }
}

void LifecycleGeofenceNodeBase::auto_transition() {
  const uint8_t current_state = this->get_current_state().id();
  if (current_state == this->last_transition_state_) {
    return;
  }
  this->last_transition_state_ = current_state;

  switch (current_state) {
  case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
    RCLCPP_INFO(this->get_logger(), "Auto-configuring lifecycle node");
    this->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    break;
  case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
    RCLCPP_INFO(this->get_logger(), "Auto-activating lifecycle node");
    this->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    break;
  case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
    RCLCPP_INFO(this->get_logger(),
                "Lifecycle node active, disabling auto-transition");
    this->auto_transition_timer_->cancel();
    break;
  default:
    break;
  }
}

// ---------------------------------------------------------------------------
// Lifecycle skeletons
// ---------------------------------------------------------------------------

CallbackReturn
LifecycleGeofenceNodeBase::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "Configuring geofence node...");

  GeofenceCommonConfig &cfg = this->commonConfig();

  // Fail fast on invalid safety config rather than running with wrong values.
  const std::vector<std::string> config_errors = cfg.validate();
  if (!config_errors.empty()) {
    for (const std::string &err : config_errors) {
      RCLCPP_ERROR(this->get_logger(), "[CONFIG] invalid parameter: %s",
                   err.c_str());
    }
    return CallbackReturn::FAILURE;
  }

  this->map_frame_id_ =
      cfg.use_namespace_for_map_frame && !cfg.tf_prefix.empty()
          ? cfg.tf_prefix + "_" + cfg.target_frame
          : cfg.target_frame;
  this->robot_frame_id_ = !cfg.tf_prefix.empty()
                              ? cfg.tf_prefix + "_" + cfg.base_link_frame
                              : cfg.base_link_frame;

  RCLCPP_INFO(this->get_logger(), "Using TF frames: map='%s', robot='%s'",
              this->map_frame_id_.c_str(), this->robot_frame_id_.c_str());

  // BEST_EFFORT for incoming sensor data (lidar_segments from preprocessor).
  const rclcpp::QoS qos_sensor_input = rclcpp::SensorDataQoS().keep_last(10);
  // RELIABLE for outgoing obstacle data (fiware_bridge, heatmap, fleet).
  const rclcpp::QoS qos_reliable_output = rclcpp::QoS(10).reliable();
  const rclcpp::QoS map_updates_qos =
      rclcpp::QoS(100).reliable().durability_volatile();
  const rclcpp::QoS qos_contours = rclcpp::QoS(10).reliable().transient_local();

  this->reentrant_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  this->map_updates_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleUpdateArray>(
          "/warehouse/map_updates", map_updates_qos,
          [this](const rises_interfaces::msg::ObstacleUpdateArray::ConstSharedPtr
                     msg) { this->updatesCallback(msg); });
  RCLCPP_INFO(
      this->get_logger(),
      "Map updates: topic subscription enabled (/warehouse/map_updates)");

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = this->reentrant_callback_group_;
  this->obstacles_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleArray>(
          "lidar_segments", qos_sensor_input,
          [this](const rises_interfaces::msg::ObstacleArray::ConstSharedPtr
                     msg) { this->obstaclesCallback(msg); },
          sub_options);

  this->map_boundary_sub_ =
      this->create_subscription<rises_interfaces::msg::Contours>(
          "map_boundary", qos_contours,
          [this](const rises_interfaces::msg::Contours::ConstSharedPtr msg) {
            this->mapBoundaryCallback(msg);
          });

  if (cfg.enable_robot_filtering) {
    const std::string &topic_prefix = cfg.robot_pose_topic_prefix;
    for (const std::string &robot_id : cfg.robot_ids) {
      const std::string robot_topic = topic_prefix + robot_id + "/pose";
      this->robot_pose_subs_[robot_id] =
          this->create_subscription<geometry_msgs::msg::PoseStamped>(
              robot_topic, qos_sensor_input,
              [this, robot_id](
                  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                this->robotPoseCallback(robot_id, msg);
              });
      RCLCPP_INFO(this->get_logger(),
                  "Subscribed to robot '%s' poses on topic: %s",
                  robot_id.c_str(), robot_topic.c_str());
    }
  }

  this->unmatched_obstacle_pub_ =
      this->create_publisher<rises_interfaces::msg::ObstacleArray>(
          "unmatched_obstacles", qos_reliable_output);
  this->obstacle_report_pub_ =
      this->create_publisher<rises_interfaces::msg::ObstacleReport>(
          "obstacle_report", qos_reliable_output);
  this->obstacle_alert_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "obstacle_alert", rclcpp::QoS(10).reliable());

  if (cfg.publish_ready_signal) {
    rclcpp::QoS qos_latched(1);
    qos_latched.transient_local().reliable();
    this->ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "geofence_ready", qos_latched);
    RCLCPP_INFO(this->get_logger(),
                "Ready signal publisher enabled on 'geofence_ready' topic");
  }

  this->validate_path_srv_ =
      this->create_service<rises_interfaces::srv::ValidatePath>(
          "validate_path",
          [this](
              const std::shared_ptr<rmw_request_id_t>,
              const std::shared_ptr<rises_interfaces::srv::ValidatePath::Request>
                  request,
              const std::shared_ptr<
                  rises_interfaces::srv::ValidatePath::Response>
                  response) { this->validatePathService(request, response); });

  this->area_state_srv_ =
      this->create_service<rises_interfaces::srv::SetAreaState>(
          "set_area_state",
          [this](
              const std::shared_ptr<rmw_request_id_t>,
              const std::shared_ptr<rises_interfaces::srv::SetAreaState::Request>
                  request,
              const std::shared_ptr<
                  rises_interfaces::srv::SetAreaState::Response>
                  response) { this->areaStateService(request, response); });

  this->update_map_srv_ =
      this->create_service<rises_interfaces::srv::UpdateMap>(
          "update_map",
          [this](
              const std::shared_ptr<rmw_request_id_t>,
              const std::shared_ptr<rises_interfaces::srv::UpdateMap::Request>
                  request,
              const std::shared_ptr<rises_interfaces::srv::UpdateMap::Response>
                  response) { this->updateMapService(request, response); });

  this->update_warehouse_layout_srv_ =
      this->create_service<rises_interfaces::srv::UpdateWarehouseLayout>(
          "update_warehouse_layout",
          [this](const std::shared_ptr<rmw_request_id_t>,
                 const std::shared_ptr<
                     rises_interfaces::srv::UpdateWarehouseLayout::Request>
                     request,
                 const std::shared_ptr<
                     rises_interfaces::srv::UpdateWarehouseLayout::Response>
                     response) {
            this->updateWarehouseLayoutService(request, response);
          });

  this->set_safety_circle_radius_srv_ =
      this->create_service<rises_interfaces::srv::SetSafetyCircleRadius>(
          "set_safety_circle_radius",
          [this](const std::shared_ptr<rmw_request_id_t>,
                 const std::shared_ptr<
                     rises_interfaces::srv::SetSafetyCircleRadius::Request>
                     request,
                 const std::shared_ptr<
                     rises_interfaces::srv::SetSafetyCircleRadius::Response>
                     response) {
            this->setSafetyCircleRadiusService(request, response);
          });

  if (cfg.visualizer_enabled) {
    this->visualizer_ = std::make_shared<GeofenceVisualizer>(
        this, cfg.tf_prefix, cfg.target_frame, cfg.base_link_frame,
        cfg.enable_safety_circle, cfg.safety_circle_outer_radius);
    RCLCPP_INFO(this->get_logger(),
                "Visualizer initialized (will be activated later)");
  }

  this->onConfigureExtra();

  RCLCPP_INFO(this->get_logger(), "Geofence node configured successfully.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifecycleGeofenceNodeBase::on_activate(const rclcpp_lifecycle::State &) {
  GeofenceCommonConfig &cfg = this->commonConfig();

  // Coordinate transforms are validated + applied here so a mis-configured
  // transform matrix fails the lifecycle transition instead of crashing
  // construction.
  try {
    cfg.applyCoordinateTransform(this->get_logger());
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "[on_activate] Invalid coordinate transform configuration: %s",
                 e.what());
    // Recoverable user-config error: FAILURE returns the node to INACTIVE
    // (the activating transition's originating state) so an operator can fix
    // the config and retry activation without a full reconfigure cycle. ERROR
    // would route through on_error() and drop the node to UNCONFIGURED.
    return CallbackReturn::FAILURE;
  }

  if (!cfg.obstacles_json_file.empty()) {
    if (this->loadObstaclesFromJson(cfg.obstacles_json_file)) {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully loaded obstacles from JSON file");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "[on_activate] Failed to load obstacles from JSON file: %s",
                   cfg.obstacles_json_file.c_str());
      // Recoverable config error -> FAILURE leaves the node INACTIVE so the
      // operator can fix the file and retry activation (see note above).
      return CallbackReturn::FAILURE;
    }
  }

  if (!cfg.contours_json_file.empty()) {
    if (this->loadContoursFromJson(cfg.contours_json_file)) {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully loaded contours from JSON file");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "[on_activate] Failed to load contours from JSON file: %s",
                   cfg.contours_json_file.c_str());
      // Recoverable config error -> FAILURE leaves the node INACTIVE so the
      // operator can fix the file and retry activation (see note above).
      return CallbackReturn::FAILURE;
    }
  }

  if (this->visualizer_) {
    this->visualizer_->activate();
    RCLCPP_INFO(this->get_logger(), "Visualizer activated");
  }

  if (this->unmatched_obstacle_pub_)
    this->unmatched_obstacle_pub_->on_activate();
  if (this->obstacle_report_pub_)
    this->obstacle_report_pub_->on_activate();
  if (this->obstacle_alert_pub_)
    this->obstacle_alert_pub_->on_activate();
  if (this->ready_pub_)
    this->ready_pub_->on_activate();

  this->onActivateExtra();

  this->publishReadySignal();

  if (this->visualizer_ && cfg.enable_safety_circle) {
    this->safety_circle_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200), [this]() {
          if (this->visualizer_) {
            this->visualizer_->publishSafetyCircle();
          }
        });
    RCLCPP_INFO(this->get_logger(),
                "Started safety circle update timer (5 Hz)");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifecycleGeofenceNodeBase::on_deactivate(const rclcpp_lifecycle::State &) {
  if (this->safety_circle_timer_) {
    this->safety_circle_timer_->cancel();
    this->safety_circle_timer_.reset();
  }

  if (this->visualizer_)
    this->visualizer_->deactivate();
  if (this->unmatched_obstacle_pub_)
    this->unmatched_obstacle_pub_->on_deactivate();
  if (this->obstacle_report_pub_)
    this->obstacle_report_pub_->on_deactivate();
  if (this->obstacle_alert_pub_)
    this->obstacle_alert_pub_->on_deactivate();
  if (this->ready_pub_)
    this->ready_pub_->on_deactivate();

  this->onDeactivateExtra();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifecycleGeofenceNodeBase::on_cleanup(const rclcpp_lifecycle::State &) {
  this->map_updates_sub_.reset();
  this->obstacles_sub_.reset();
  this->map_boundary_sub_.reset();
  this->robot_pose_subs_.clear();

  this->unmatched_obstacle_pub_.reset();
  this->obstacle_report_pub_.reset();
  this->obstacle_alert_pub_.reset();
  this->ready_pub_.reset();

  this->validate_path_srv_.reset();
  this->area_state_srv_.reset();
  this->update_map_srv_.reset();
  this->update_warehouse_layout_srv_.reset();
  this->set_safety_circle_radius_srv_.reset();

  this->visualizer_.reset();
  this->report_builder_.reset();

  this->onCleanupExtra();
  this->resetBackend();

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Shared callbacks
// ---------------------------------------------------------------------------

void LifecycleGeofenceNodeBase::updatesCallback(
    const rises_interfaces::msg::ObstacleUpdateArray::ConstSharedPtr &msg) {
  if (!this->backendReady() || msg->updates.empty())
    return;

  this->applyMapUpdates(*msg, "TOPIC");

  if (this->visualizer_) {
    this->visualizer_->publishMap();
  }
}

void LifecycleGeofenceNodeBase::obstaclesCallback(
    const rises_interfaces::msg::ObstacleArray::ConstSharedPtr &msg) {
  if (!this->backendReady())
    return;

  // Start of per-scan processing -- used to measure the safety-reaction
  // (scan -> obstacle_alert) latency below.
  const std::chrono::steady_clock::time_point scan_processing_start =
      std::chrono::steady_clock::now();

  if (this->visualizer_) {
    this->visualizer_->beginNewScanCycle();
  }

  const GeofenceCommonConfig &cfg = this->commonConfig();

  Point2D robot_pos;
  const rclcpp::Time msg_time(msg->header.stamp);
  const bool have_robot_pos = this->getRobotPosition(robot_pos, msg_time);

  const bool robot_pos_required =
      cfg.enable_safety_circle || cfg.enable_robot_filtering;
  if (robot_pos_required && !have_robot_pos) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Cannot get robot position - cannot process obstacles");
    return;
  }

  const ScanMatchResult result =
      this->matchScan(msg, robot_pos, have_robot_pos);

  // Publish the alert state ONCE per scan: true when an unmatched obstacle is
  // present, false to CLEAR. Publishing the cleared state is essential --
  // otherwise the alert latches on forever after the first intruder. Published
  // BEFORE the report so the safety reaction is as fast as possible.
  if (this->obstacle_alert_pub_ && this->obstacle_alert_pub_->is_activated()) {
    std::unique_ptr<std_msgs::msg::Bool> alert =
        std::make_unique<std_msgs::msg::Bool>();
    alert->data = result.has_unmatched;
    this->obstacle_alert_pub_->publish(std::move(alert));
  }

  // Safety-reaction latency: the obstacle_alert is the fast path (published
  // before the per-segment report), so this is the per-scan latency that
  // matters for reaction time. processing = pure compute this callback
  // (scan in -> alert out); scan_to_alert = end-to-end including transport,
  // measured against the scan acquisition stamp.
  const double alert_processing_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - scan_processing_start)
          .count();
  const double scan_to_alert_ms = (this->now() - msg_time).seconds() * 1000.0;

  // Accumulate into the rolling-average KPI on every scan (not throttled --
  // the THROTTLE log below stays throttled, but the accumulation must see
  // every scan or the average would silently undercount).
  {
    const std::lock_guard<std::mutex> lock(this->alert_latency_mutex_);
    this->alert_latency_accumulator_.processing_ms_sum += alert_processing_ms;
    this->alert_latency_accumulator_.scan_to_alert_ms_sum += scan_to_alert_ms;
    ++this->alert_latency_accumulator_.count;
  }

  // Operational heartbeat (5 s) of the safety-reaction latency. `processing` is
  // the geofence's own contribution (scan in -> alert out, sub-ms in practice);
  // `scan->alert` is end-to-end and is dominated by the sensor frame period +
  // transport, NOT the node. Throttled so it is a health signal, not noise.
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "[alert latency] processing=%.3f ms  scan->alert=%.3f ms "
                       "(unmatched=%s)",
                       alert_processing_ms, scan_to_alert_ms,
                       result.has_unmatched ? "true" : "false");

  // Build + publish the per-segment obstacle_report (real positions): the
  // single detection representation consumed by validation / FIWARE / fleet,
  // and republished on the unmatched_obstacle topic. Gated on publish_report so
  // the spatial shape-matching path (which classifies per-obstacle and does not
  // populate report_result) does not emit an empty report every scan, matching
  // pre-unification behaviour. beginScanCycle/endScanCycle are paired here.
  if (this->report_builder_ && result.publish_report) {
    this->report_builder_->beginScanCycle();
    this->report_builder_->buildReport(msg, result.report_result,
                                        this->tf_buffer_, this->get_logger());
    this->report_builder_->publishReport(this->obstacle_report_pub_,
                                          cfg.publish_report_always,
                                          this->tf_buffer_, this->get_logger());

    // Single detection representation: the topic carries the SAME per-segment
    // unmatched obstacles (real positions, with width), not a whole-scan
    // centroid -- so RViz / rises_leg_filter get real locations.
    if (this->unmatched_obstacle_pub_ &&
        this->unmatched_obstacle_pub_->is_activated()) {
      const rises_interfaces::msg::ObstacleReport &report =
          this->report_builder_->lastReport();
      if (!report.unmatched_obstacles.empty()) {
        auto unmatched_msg =
            std::make_unique<rises_interfaces::msg::ObstacleArray>();
        unmatched_msg->header = report.header;
        unmatched_msg->obstacles = report.unmatched_obstacles;
        this->unmatched_obstacle_pub_->publish(std::move(unmatched_msg));
      }
    }

    // Evict error segments not seen in this scan cycle.
    this->report_builder_->endScanCycle();
  }

  // Throttled visualization: segments at 5 Hz, map markers at 1 Hz.
  if (this->visualizer_) {
    const std::chrono::steady_clock::time_point wall_now =
        std::chrono::steady_clock::now();

    if (wall_now - this->last_seg_viz_time_ >= SEG_VIZ_INTERVAL) {
      this->last_seg_viz_time_ = wall_now;
      // Populate report-derived segments (spatial points-only) before publish.
      if (this->report_builder_) {
        this->onReportBuilt(this->report_builder_->lastReport());
      }
      this->visualizer_->publishSegments();
    }

    if (wall_now - this->last_map_viz_time_ >= MAP_VIZ_INTERVAL) {
      this->last_map_viz_time_ = wall_now;
      this->visualizer_->refreshMapColors();
      this->visualizer_->publishMap();
    }
  }
}

void LifecycleGeofenceNodeBase::mapBoundaryCallback(
    const rises_interfaces::msg::Contours::ConstSharedPtr &msg) {
  if (!this->backendReady())
    return;
  this->setContours(*msg);
}

void LifecycleGeofenceNodeBase::robotPoseCallback(
    const std::string &robot_id,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  if (!this->commonConfig().enable_robot_filtering ||
      !this->robot_tracking_checker_)
    return;

  this->robot_tracking_checker_->updateRobotPose(robot_id, *msg);

  RCLCPP_DEBUG(this->get_logger(), "Updated robot '%s' pose: x=%.2f, y=%.2f",
               robot_id.c_str(), msg->pose.position.x, msg->pose.position.y);
}

// ---------------------------------------------------------------------------
// Optional-hook defaults
// ---------------------------------------------------------------------------

void LifecycleGeofenceNodeBase::onReportBuilt(
    const rises_interfaces::msg::ObstacleReport & /*report*/) {}

void LifecycleGeofenceNodeBase::setContours(
    const rises_interfaces::msg::Contours &contours) {
  RCLCPP_INFO_ONCE(
      this->get_logger(),
      "Received map boundary contours (%zu outer segments, %zu inner polygons) "
      "- this backend does not store contour boundaries; ignoring.",
      contours.outer_contour_segments.size(), contours.inner_contours.size());
}

void LifecycleGeofenceNodeBase::setSafetyCircleRadius(float radius) {
  this->commonConfig().safety_circle_outer_radius = radius;

  if (this->visualizer_) {
    this->visualizer_->clearSafetyCircle();
    this->visualizer_->addSafetyCircle(radius, "safety_circle");
    this->visualizer_->publishSafetyCircle();
  }
}

void LifecycleGeofenceNodeBase::setSafetyCircleRadiusService(
    const std::shared_ptr<rises_interfaces::srv::SetSafetyCircleRadius::Request>
        request,
    const std::shared_ptr<
        rises_interfaces::srv::SetSafetyCircleRadius::Response>
        response) {
  if (request->radius <= 0.0f) {
    response->success = false;
    response->message = "Invalid radius: must be > 0";
    RCLCPP_WARN(this->get_logger(),
                "SetSafetyCircleRadius rejected: radius=%.2f (must be > 0)",
                request->radius);
    return;
  }

  this->setSafetyCircleRadius(request->radius);

  response->success = true;
  response->message =
      "Safety circle radius updated to " + std::to_string(request->radius) +
      (this->visualizer_ ? " meters" : " meters (visualizer not enabled)");
  RCLCPP_INFO(this->get_logger(), "Safety circle radius updated to %.2f meters",
              request->radius);
}

// ---------------------------------------------------------------------------
// Shared utilities
// ---------------------------------------------------------------------------

bool LifecycleGeofenceNodeBase::getRobotPosition(
    Point2D &position, const rclcpp::Time &timestamp) const {
  // Use canTransform first to avoid expensive exception throwing.
  // Exception-as-control-flow costs ~10-100us per throw -- at 20 Hz that's
  // significant.
  if (this->tf_buffer_->canTransform(this->map_frame_id_, this->robot_frame_id_,
                                     timestamp, tf2::durationFromSec(0.0))) {
    const geometry_msgs::msg::TransformStamped transform =
        this->tf_buffer_->lookupTransform(this->map_frame_id_,
                                          this->robot_frame_id_, timestamp);
    position.x = transform.transform.translation.x;
    position.y = transform.transform.translation.y;
    return true;
  }

  // Fallback: try latest available transform.
  if (this->tf_buffer_->canTransform(this->map_frame_id_, this->robot_frame_id_,
                                     tf2::TimePointZero,
                                     tf2::durationFromSec(0.0))) {
    const geometry_msgs::msg::TransformStamped transform =
        this->tf_buffer_->lookupTransform(this->map_frame_id_,
                                          this->robot_frame_id_,
                                          rclcpp::Time(0));
    position.x = transform.transform.translation.x;
    position.y = transform.transform.translation.y;
    return true;
  }

  return false;
}

rises::geofence::policies::RobotFootprint
LifecycleGeofenceNodeBase::createRobotFootprint(
    const std::string &robot_id) const {
  const GeofenceCommonConfig &cfg = this->commonConfig();
  const std::string robot_prefix = "robot_" + robot_id + "_";

  const std::string footprint_type_param = robot_prefix + "footprint_type";
  const std::string footprint_type =
      this->has_parameter(footprint_type_param)
          ? this->get_parameter(footprint_type_param).as_string()
          : cfg.default_robot_footprint_type;

  const std::string margin_param = robot_prefix + "footprint_margin";
  const double margin = this->has_parameter(margin_param)
                            ? this->get_parameter(margin_param).as_double()
                            : cfg.default_robot_footprint_margin;

  if (footprint_type == "circle") {
    const std::string radius_param = robot_prefix + "footprint_radius";
    const double radius = this->has_parameter(radius_param)
                              ? this->get_parameter(radius_param).as_double()
                              : cfg.default_robot_footprint_radius;
    return rises::geofence::policies::RobotFootprint::createCircle(radius,
                                                                   margin);
  }
  if (footprint_type == "rectangle") {
    const std::string width_param = robot_prefix + "footprint_width";
    const std::string height_param = robot_prefix + "footprint_height";
    const double width = this->has_parameter(width_param)
                             ? this->get_parameter(width_param).as_double()
                             : cfg.default_robot_footprint_width;
    const double height = this->has_parameter(height_param)
                              ? this->get_parameter(height_param).as_double()
                              : cfg.default_robot_footprint_height;
    return rises::geofence::policies::RobotFootprint::createRectangle(
        width, height, margin);
  }
  if (footprint_type == "polygon") {
    const std::string polygon_param = robot_prefix + "footprint_polygon";
    const std::vector<double> polygon_flat =
        this->has_parameter(polygon_param)
            ? this->get_parameter(polygon_param).as_double_array()
            : cfg.default_robot_footprint_polygon;

    if (polygon_flat.size() % 2 != 0 || polygon_flat.size() < 6) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid polygon footprint for robot '%s': must have at "
                   "least 3 points (6 values). Falling back to circle.",
                   robot_id.c_str());
      return rises::geofence::policies::RobotFootprint::createCircle(
          cfg.default_robot_footprint_radius, margin);
    }
    std::vector<rises::geofence::policies::RobotFootprint::Point2D> vertices;
    for (std::size_t i = 0; i < polygon_flat.size(); i += 2) {
      vertices.push_back({polygon_flat[i], polygon_flat[i + 1]});
    }
    return rises::geofence::policies::RobotFootprint::createPolygon(vertices,
                                                                    margin);
  }

  RCLCPP_ERROR(
      this->get_logger(),
      "Unknown footprint type '%s' for robot '%s'. Falling back to circle.",
      footprint_type.c_str(), robot_id.c_str());
  return rises::geofence::policies::RobotFootprint::createCircle(
      cfg.default_robot_footprint_radius, margin);
}

std::string LifecycleGeofenceNodeBase::getFootprintTypeString(
    rises::geofence::policies::RobotFootprint::Type type) {
  switch (type) {
  case rises::geofence::policies::RobotFootprint::Type::CIRCLE:
    return "circle";
  case rises::geofence::policies::RobotFootprint::Type::RECTANGLE:
    return "rectangle";
  case rises::geofence::policies::RobotFootprint::Type::POLYGON:
    return "polygon";
  }
  return "unknown";
}

void LifecycleGeofenceNodeBase::publishReadySignal() {
  const GeofenceCommonConfig &cfg = this->commonConfig();
  if (!cfg.publish_ready_signal || !this->ready_pub_) {
    return;
  }

  if (this->visualizer_) {
    this->visualizer_->publishMap();
    this->visualizer_->publishContours();
    this->visualizer_->publishSafetyCircle();
    RCLCPP_INFO(this->get_logger(), "Published pre-loaded visualizations "
                                    "(obstacles + contours + safety circle)");
  }

  std::unique_ptr<std_msgs::msg::Bool> msg =
      std::make_unique<std_msgs::msg::Bool>();
  msg->data = true;
  this->ready_pub_->publish(std::move(msg));

  RCLCPP_INFO(this->get_logger(),
              "[MAP_DEBUG] [READY] Published geofence_ready signal - map is "
              "initialized with %zu obstacles",
              this->backendObstacleCount());
}

LifecycleGeofenceNodeBase::AlertLatencyStats
LifecycleGeofenceNodeBase::getAndResetAlertLatencyStats() {
  const std::lock_guard<std::mutex> lock(this->alert_latency_mutex_);

  AlertLatencyStats stats;
  if (this->alert_latency_accumulator_.count > 0) {
    const double count =
        static_cast<double>(this->alert_latency_accumulator_.count);
    stats.avg_processing_ms =
        this->alert_latency_accumulator_.processing_ms_sum / count;
    stats.avg_scan_to_alert_ms =
        this->alert_latency_accumulator_.scan_to_alert_ms_sum / count;
  }

  this->alert_latency_accumulator_ = AlertLatencyAccumulator{};
  return stats;
}

} // namespace rises
