#include "safety/safety.hpp"

rises::Safety::Safety(const rclcpp::NodeOptions &options)
    : rclcpp::Node("safety_node", options) {
  this->diagnostics_sub_ =
      this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/diagnostics", 10,
          std::bind(&Safety::diagnosticsCallback, this, std::placeholders::_1));

  this->detected_obstacles_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleArray>(
          "detected_obstacles", 10,
          std::bind(&Safety::detectedObstaclesCallback, this,
                    std::placeholders::_1));

  this->path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "incoming_path", 10,
      std::bind(&Safety::pathCallback, this, std::placeholders::_1));

  this->heatmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "predicted_occupancy", rclcpp::QoS(1).reliable(),
      std::bind(&Safety::heatmapCallback, this, std::placeholders::_1));

  this->alert_pub_ =
      this->create_publisher<rises_interfaces::msg::ObstacleArray>("alert", 10);
  this->validated_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("validated_path", 10);
  this->response_pub_ =
      this->create_publisher<std_msgs::msg::String>("response", 10);
  this->halt_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "halt", rclcpp::QoS(1).reliable());

  this->declare_parameter("validate_path_service_name", "validate_path");
  const std::string validate_path_service_name =
      this->get_parameter("validate_path_service_name").as_string();
  this->validate_path_client_ =
      this->create_client<rises_interfaces::srv::ValidatePath>(
          validate_path_service_name);

  this->declare_parameter("node_timeout_sec", 10.0);
  this->node_timeout_sec_ = this->get_parameter("node_timeout_sec").as_double();

  this->declare_parameter("heatmap_overlap_threshold", 50);
  this->heatmap_overlap_threshold_ =
      this->get_parameter("heatmap_overlap_threshold").as_int();

  // Periodic health check — verify monitored nodes are still reporting
  this->health_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&Safety::healthCheckCallback, this));

  RCLCPP_INFO(
      this->get_logger(),
      "Safety node initialized (node_timeout=%.0fs, heatmap_threshold=%d)",
      this->node_timeout_sec_, this->heatmap_overlap_threshold_);
}

void rises::Safety::diagnosticsCallback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
  const rclcpp::Time now = this->now();

  for (const diagnostic_msgs::msg::DiagnosticStatus &status : msg->status) {
    const std::string node_name = status.name.substr(0, status.name.find(':'));

    NodeHealth &health = this->monitored_nodes_[node_name];
    health.level = status.level;
    health.last_seen = now;

    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      RCLCPP_ERROR(this->get_logger(), "Node '%s' reports ERROR: %s",
                   node_name.c_str(), status.message.c_str());
      this->haltSystem("Node '" + node_name +
                       "' in ERROR state: " + status.message);
    } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      RCLCPP_WARN(this->get_logger(), "Node '%s' reports WARN: %s",
                  node_name.c_str(), status.message.c_str());
    }
  }
}

void rises::Safety::healthCheckCallback() {
  const rclcpp::Time now = this->now();
  bool all_healthy = true;

  for (const auto &[name, health] : this->monitored_nodes_) {
    const double age_sec = (now - health.last_seen).seconds();
    if (age_sec > this->node_timeout_sec_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Node '%s' has not reported diagnostics for %.1f seconds "
                   "(timeout: %.0fs)",
                   name.c_str(), age_sec, this->node_timeout_sec_);
      this->haltSystem("Node '" + name + "' unresponsive (no diagnostics for " +
                       std::to_string(static_cast<int>(age_sec)) + "s)");
      all_healthy = false;
    }
  }

  if (all_healthy && this->system_halted_.load()) {
    this->resumeSystem("All monitored nodes reporting healthy");
  }
}

void rises::Safety::detectedObstaclesCallback(
    const rises_interfaces::msg::ObstacleArray::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received %zu detected obstacles",
               msg->obstacles.size());

  this->alert_pub_->publish(*msg);

  std::unique_ptr<std_msgs::msg::String> string_response_msg =
      std::make_unique<std_msgs::msg::String>();
  string_response_msg->data = "Alerted " +
                              std::to_string(msg->obstacles.size()) +
                              " detected obstacles.";
  this->response_pub_->publish(std::move(string_response_msg));
}

void rises::Safety::heatmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Concurrent writer/reader on latest_heatmap_: heatmapCallback writes,
  // pathCallback reads, both via a MultiThreadedExecutor. std::atomic_store
  // synchronizes the shared_ptr control block and pointer slot. C++17 only;
  // see latest_heatmap_ declaration for the C++20 migration note.
  std::atomic_store(&this->latest_heatmap_, msg);
}

void rises::Safety::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received path with %zu poses",
               msg->poses.size());

  // Check if system is halted due to node failure
  if (this->system_halted_.load()) {
    RCLCPP_WARN(this->get_logger(), "System halted — rejecting path");
    std::unique_ptr<std_msgs::msg::String> resp =
        std::make_unique<std_msgs::msg::String>();
    resp->data = "Path rejected: system halted due to safety condition";
    this->response_pub_->publish(std::move(resp));
    return;
  }

  // Check predictive heatmap overlap — slow down or reject if predicted
  // obstacle presence. Snapshot latest_heatmap_ once here (atomic_load) so the
  // predicate body operates on a stable shared_ptr even if heatmapCallback
  // swaps the member concurrently. The snapshot keeps the grid alive for the
  // duration of the scan.
  const nav_msgs::msg::OccupancyGrid::SharedPtr heatmap_snapshot =
      std::atomic_load(&this->latest_heatmap_);
  if (this->isPathOverlappingPrediction(*msg, heatmap_snapshot)) {
    RCLCPP_WARN(this->get_logger(),
                "Path overlaps with predicted obstacle positions — consider "
                "slowing down or rerouting");
    std::unique_ptr<std_msgs::msg::String> resp =
        std::make_unique<std_msgs::msg::String>();
    resp->data = "Path warning: overlaps with predicted obstacle trajectory";
    this->response_pub_->publish(std::move(resp));
    // Note: we still validate the path but warn the fleet manager.
    // In production, this could trigger a speed reduction command.
  }

  if (!this->validate_path_client_->wait_for_service(std::chrono::seconds(1))) {
    // ValidatePath unavailable means the geofence node is dead or partitioned
    // off the DDS graph. Continuing to forward paths to the fleet manager
    // without validation is unsafe; silently dropping is worse because the
    // fleet manager never learns the path was rejected. Halt the system AND
    // publish an explicit rejection on the response topic so callers get a
    // terminal answer instead of timing out.
    RCLCPP_ERROR(this->get_logger(), "ValidatePath service unavailable -- "
                                     "halting system and rejecting path");
    std::unique_ptr<std_msgs::msg::String> resp =
        std::make_unique<std_msgs::msg::String>();
    resp->data = "Path rejected: ValidatePath service unavailable";
    this->response_pub_->publish(std::move(resp));
    this->haltSystem("ValidatePath service unavailable");
    return;
  }

  std::shared_ptr<rises_interfaces::srv::ValidatePath::Request> request =
      std::make_shared<rises_interfaces::srv::ValidatePath::Request>();
  request->path = *msg;

  std::shared_ptr<Safety> self =
      std::dynamic_pointer_cast<Safety>(this->shared_from_this());

  this->validate_path_client_->async_send_request(
      request,
      [self,
       msg](rclcpp::Client<rises_interfaces::srv::ValidatePath>::SharedFuture
                future) {
        const std::shared_ptr<rises_interfaces::srv::ValidatePath::Response>
            response = future.get();

        std::unique_ptr<std_msgs::msg::String> string_response_msg =
            std::make_unique<std_msgs::msg::String>();

        if (response->blocked) {
          RCLCPP_WARN(self->get_logger(), "Path is blocked by geofence!");
          string_response_msg->data = "Path is blocked by geofence!";
        } else {
          RCLCPP_INFO(self->get_logger(), "Path is allowed by geofence!");
          self->validated_path_pub_->publish(*msg);
          string_response_msg->data = "Path is allowed by geofence!";
        }

        self->response_pub_->publish(std::move(string_response_msg));
      });
}

bool rises::Safety::isPathOverlappingPrediction(
    const nav_msgs::msg::Path &path,
    const nav_msgs::msg::OccupancyGrid::SharedPtr &heatmap) const {
  // heatmap is a caller-owned snapshot from std::atomic_load on
  // latest_heatmap_. Do NOT read latest_heatmap_ here; doing so would
  // re-introduce the unsynchronized access this fix exists to remove.
  if (!heatmap || path.poses.empty()) {
    return false;
  }

  const nav_msgs::msg::OccupancyGrid &grid = *heatmap;
  const double res = static_cast<double>(grid.info.resolution);
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);

  for (const geometry_msgs::msg::PoseStamped &pose : path.poses) {
    const int col = static_cast<int>((pose.pose.position.x - origin_x) / res);
    const int row = static_cast<int>((pose.pose.position.y - origin_y) / res);

    if (col < 0 || col >= width || row < 0 || row >= height) {
      continue;
    }

    const int idx = row * width + col;
    if (grid.data[static_cast<std::size_t>(idx)] >=
        this->heatmap_overlap_threshold_) {
      RCLCPP_WARN(this->get_logger(),
                  "Path waypoint (%.2f, %.2f) overlaps predicted obstacle at "
                  "cell (%d, %d) = %d%%",
                  pose.pose.position.x, pose.pose.position.y, col, row,
                  static_cast<int>(grid.data[static_cast<std::size_t>(idx)]));
      return true;
    }
  }

  return false;
}

void rises::Safety::haltSystem(const std::string &reason) {
  if (!this->system_halted_.exchange(true)) {
    RCLCPP_ERROR(this->get_logger(), "SYSTEM HALT: %s", reason.c_str());
    std::unique_ptr<std_msgs::msg::Bool> halt_msg =
        std::make_unique<std_msgs::msg::Bool>();
    halt_msg->data = true;
    this->halt_pub_->publish(std::move(halt_msg));

    std::unique_ptr<std_msgs::msg::String> resp =
        std::make_unique<std_msgs::msg::String>();
    resp->data = "HALT: " + reason;
    this->response_pub_->publish(std::move(resp));
  }
}

void rises::Safety::resumeSystem(const std::string &reason) {
  if (this->system_halted_.exchange(false)) {
    RCLCPP_INFO(this->get_logger(), "SYSTEM RESUME: %s", reason.c_str());
    std::unique_ptr<std_msgs::msg::Bool> halt_msg =
        std::make_unique<std_msgs::msg::Bool>();
    halt_msg->data = false;
    this->halt_pub_->publish(std::move(halt_msg));

    std::unique_ptr<std_msgs::msg::String> resp =
        std::make_unique<std_msgs::msg::String>();
    resp->data = "RESUME: " + reason;
    this->response_pub_->publish(std::move(resp));
  }
}

// ---------------- Node registration ----------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rises::Safety)
