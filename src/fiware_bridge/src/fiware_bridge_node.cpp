/// @file fiware_bridge_node.cpp
/// @brief FIWARE bridge node — translates ROS 2 messages to JSON strings for
/// DDS Enabler.

#include "fiware_bridge/fiware_bridge_node.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string_view>
#include <utility>
#include <vector>

namespace {

// -----------------------------------------------------------------------------
// Diagnostic JSON injection hardening (audit finding #15).
//
// /diagnostics is a free-publish topic: any node on the ROS graph can publish
// KeyValue entries that get forwarded into the fiware/system_health JSON. The
// pre-fix code did `node_info[kv.key] = kv.value` unconditionally, allowing:
//   - reserved-key shadowing (level / message / name / hardware_id);
//   - unbounded key / value payloads (1 MiB+ strings forwarded to Orion-LD);
//   - raw control bytes / NUL inside JSON keys.
//
// We adopt a denylist (reserved structural keys) + length caps + control-byte
// rejection. Allowlist was considered but the diagnostic schema is open: every
// ROS node can declare its own keys, so an allowlist would silently drop most
// legitimate diagnostics.
// -----------------------------------------------------------------------------
constexpr std::array<std::string_view, 4> kReservedDiagKeys = {
    "level",
    "message",
    "name",
    "hardware_id",
};

constexpr std::size_t kMaxDiagKeyLen = 64;
constexpr std::size_t kMaxDiagValueLen = 1024;

bool isReservedDiagKey(std::string_view key) {
  for (std::string_view reserved : kReservedDiagKeys) {
    if (key == reserved) {
      return true;
    }
  }
  return false;
}

bool containsControlByte(std::string_view s) {
  for (char c : s) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (uc < 0x20u || uc == 0x7Fu) {
      return true;
    }
  }
  return false;
}

bool isDiagKeyAcceptable(std::string_view key, std::string_view value) {
  if (key.empty()) {
    return false;
  }
  if (key.size() > kMaxDiagKeyLen || value.size() > kMaxDiagValueLen) {
    return false;
  }
  if (isReservedDiagKey(key)) {
    return false;
  }
  if (containsControlByte(key)) {
    return false;
  }
  return true;
}

} // namespace

namespace rises {

FiwareBridgeNode::FiwareBridgeNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("fiware_bridge", options) {
  // Parameters
  this->declare_parameter("report_throttle_hz", 1.0);
  this->declare_parameter("odom_throttle_hz", 2.0);
  this->declare_parameter("diag_throttle_hz", 0.5);
  this->declare_parameter("heatmap_throttle_hz", 1.0);
  this->declare_parameter("max_unmatched_positions", 30);
  this->declare_parameter("max_matched_positions", 50);
  this->declare_parameter("obstacles_json_file", std::string{});

  const double report_hz =
      this->get_parameter("report_throttle_hz").as_double();
  const double odom_hz = this->get_parameter("odom_throttle_hz").as_double();
  const double diag_hz = this->get_parameter("diag_throttle_hz").as_double();
  const double heatmap_hz =
      this->get_parameter("heatmap_throttle_hz").as_double();
  this->max_unmatched_positions_ =
      this->get_parameter("max_unmatched_positions").as_int();
  this->max_matched_positions_ =
      this->get_parameter("max_matched_positions").as_int();

  this->report_interval_ = std::chrono::duration<double>(1.0 / report_hz);
  this->odom_interval_ = std::chrono::duration<double>(1.0 / odom_hz);
  this->diag_interval_ = std::chrono::duration<double>(1.0 / diag_hz);
  this->heatmap_interval_ = std::chrono::duration<double>(1.0 / heatmap_hz);

  // Subscriptions
  const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
  const rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliable();
  const rclcpp::QoS transient_qos = rclcpp::QoS(1).reliable().transient_local();

  this->report_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleReport>(
          "obstacle_report", sensor_qos,
          std::bind(&FiwareBridgeNode::reportCallback, this,
                    std::placeholders::_1));

  this->alert_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "obstacle_alert", reliable_qos,
      std::bind(&FiwareBridgeNode::alertCallback, this, std::placeholders::_1));

  this->ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "geofence_ready", transient_qos,
      std::bind(&FiwareBridgeNode::readyCallback, this, std::placeholders::_1));

  this->contours_sub_ =
      this->create_subscription<rises_interfaces::msg::Contours>(
          "warehouse_contours", transient_qos,
          std::bind(&FiwareBridgeNode::contoursCallback, this,
                    std::placeholders::_1));

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", reliable_qos,
      std::bind(&FiwareBridgeNode::odomCallback, this, std::placeholders::_1));

  this->diag_sub_ =
      this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/diagnostics", reliable_qos,
          std::bind(&FiwareBridgeNode::diagCallback, this,
                    std::placeholders::_1));

  this->heatmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "predicted_occupancy", rclcpp::QoS(1).reliable(),
      std::bind(&FiwareBridgeNode::heatmapCallback, this,
                std::placeholders::_1));

  this->map_updates_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleUpdateArray>(
          "/warehouse/map_updates", rclcpp::QoS(100).reliable(),
          std::bind(&FiwareBridgeNode::mapUpdatesCallback, this,
                    std::placeholders::_1));

  this->republish_geometry_sub_ =
      this->create_subscription<std_msgs::msg::Empty>(
          "fiware_bridge/republish_geometry", rclcpp::QoS(1).reliable(),
          std::bind(&FiwareBridgeNode::republishGeometryCallback, this,
                    std::placeholders::_1));

  this->dds_trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "fiware/dds_trigger", rclcpp::QoS(1).reliable(),
      std::bind(&FiwareBridgeNode::ddsTriggerCallback, this,
                std::placeholders::_1));

  this->validation_result_sub_ =
      this->create_subscription<std_msgs::msg::String>(
          "validation_result", rclcpp::QoS(10).reliable(),
          std::bind(&FiwareBridgeNode::validationResultCallback, this,
                    std::placeholders::_1));

  // Service client for requesting current contours from geofence
  this->get_contours_client_ =
      this->create_client<rises_interfaces::srv::GetWarehouseContours>(
          "geofence_node/get_warehouse_contours");

  // Publishers (fiware/* topics)
  const rclcpp::QoS fiware_qos = rclcpp::QoS(1).reliable();
  const rclcpp::QoS fiware_transient_qos =
      rclcpp::QoS(1).reliable().transient_local();

  this->obstacle_summary_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/obstacle_summary", fiware_qos);
  this->obstacle_alert_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/obstacle_alert", fiware_qos);
  this->geofence_ready_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/geofence_ready", fiware_qos);
  this->warehouse_geometry_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/warehouse_geometry", fiware_transient_qos);
  this->robot_position_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/robot_position", fiware_qos);
  this->system_health_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/system_health", fiware_qos);
  this->heatmap_summary_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/heatmap_summary", fiware_qos);
  this->map_obstacles_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/map_obstacles", fiware_transient_qos);
  this->obstacle_segments_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/obstacle_segments", fiware_qos);
  this->validation_result_pub_ = this->create_publisher<std_msgs::msg::String>(
      "fiware/validation_result", fiware_qos);

  // JSON pre-initialisation: load pallets from file if provided, same as
  // geofence_node
  const std::string obstacles_json =
      this->get_parameter("obstacles_json_file").as_string();
  if (!obstacles_json.empty()) {
    this->loadPalletsFromJson(obstacles_json);
  }

  RCLCPP_INFO(this->get_logger(),
              "FIWARE bridge initialized. Throttle: report=%.0fHz, "
              "odom=%.0fHz, diag=%.1fHz, "
              "heatmap=%.0fHz (TRANSIENT_LOCAL QoS for geometry, trigger topic "
              "~/republish_geometry)",
              report_hz, odom_hz, diag_hz, heatmap_hz);
}

void FiwareBridgeNode::reportCallback(
    const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg) {
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  if (now - this->last_report_time_ < this->report_interval_)
    return;
  this->last_report_time_ = now;

  nlohmann::json j;
  j["matched_count"] = static_cast<int>(msg->matched_obstacles.size());
  j["unmatched_count"] = static_cast<int>(msg->unmatched_obstacles.size());

  // Build line segments from obstacle vertices for Grafana visualization
  auto buildSegments =
      [](const std::vector<rises_interfaces::msg::Obstacle> &obstacles,
         int limit) -> nlohmann::json {
    nlohmann::json segs = nlohmann::json::array();
    const int n = std::min(static_cast<int>(obstacles.size()), limit);
    for (int i = 0; i < n; ++i) {
      const rises_interfaces::msg::Obstacle &obs =
          obstacles[static_cast<std::size_t>(i)];
      if (obs.vertices.size() >= 2) {
        segs.push_back(
            {{"x1", std::round(obs.vertices.front().x * 1000.0) / 1000.0},
             {"y1", std::round(obs.vertices.front().y * 1000.0) / 1000.0},
             {"x2", std::round(obs.vertices.back().x * 1000.0) / 1000.0},
             {"y2", std::round(obs.vertices.back().y * 1000.0) / 1000.0}});
      } else {
        const double cx = std::round(obs.position.x * 1000.0) / 1000.0;
        const double cy = std::round(obs.position.y * 1000.0) / 1000.0;
        segs.push_back(
            {{"x1", cx - 0.1}, {"y1", cy}, {"x2", cx + 0.1}, {"y2", cy}});
      }
    }
    return segs;
  };

  j["matched_segments"] =
      buildSegments(msg->matched_obstacles, this->max_matched_positions_);
  j["unmatched_segments"] =
      buildSegments(msg->unmatched_obstacles, this->max_unmatched_positions_);

  nlohmann::json unmatched_ids = nlohmann::json::array();
  const int unmatched_limit =
      std::min(static_cast<int>(msg->unmatched_obstacles.size()),
               this->max_unmatched_positions_);
  for (int i = 0; i < unmatched_limit; ++i) {
    unmatched_ids.push_back(static_cast<int64_t>(
        msg->unmatched_obstacles[static_cast<std::size_t>(i)].id));
  }
  j["unmatched_ids"] = unmatched_ids;

  this->publishJson(this->obstacle_summary_pub_, j.dump());

  // Also publish as a separate topic for the DDS Enabler
  nlohmann::json seg_json;
  seg_json["matched_segments"] = j["matched_segments"];
  seg_json["unmatched_segments"] = j["unmatched_segments"];
  this->publishJson(this->obstacle_segments_pub_, seg_json.dump());
}

void FiwareBridgeNode::alertCallback(
    const std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (this->alert_initialized_ && msg->data == this->last_alert_value_)
    return;
  this->alert_initialized_ = true;
  this->last_alert_value_ = msg->data;

  nlohmann::json j;
  j["active"] = msg->data;
  this->publishJson(this->obstacle_alert_pub_, j.dump());

  RCLCPP_INFO(this->get_logger(), "obstacle_alert = %s",
              msg->data ? "true" : "false");
}

void FiwareBridgeNode::readyCallback(
    const std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (this->ready_initialized_ && msg->data == this->last_ready_value_)
    return;
  this->ready_initialized_ = true;
  this->last_ready_value_ = msg->data;

  nlohmann::json j;
  j["ready"] = msg->data;
  this->publishJson(this->geofence_ready_pub_, j.dump());

  RCLCPP_INFO(this->get_logger(), "geofence_ready = %s",
              msg->data ? "true" : "false");
}

void FiwareBridgeNode::contoursCallback(
    const rises_interfaces::msg::Contours::ConstSharedPtr &msg) {
  nlohmann::json j;

  nlohmann::json walls = nlohmann::json::array();
  for (const rises_interfaces::msg::LineSegment &seg :
       msg->outer_contour_segments) {
    walls.push_back(
        {{"x1", std::round(static_cast<double>(seg.start.x) * 1000.0) / 1000.0},
         {"y1", std::round(static_cast<double>(seg.start.y) * 1000.0) / 1000.0},
         {"x2", std::round(static_cast<double>(seg.end.x) * 1000.0) / 1000.0},
         {"y2", std::round(static_cast<double>(seg.end.y) * 1000.0) / 1000.0}});
  }
  j["wall_segments"] = walls;

  nlohmann::json hull = nlohmann::json::array();
  for (const geometry_msgs::msg::Point32 &pt : msg->outer_contour_hull.points) {
    hull.push_back(
        {{"x", std::round(static_cast<double>(pt.x) * 1000.0) / 1000.0},
         {"y", std::round(static_cast<double>(pt.y) * 1000.0) / 1000.0}});
  }
  j["outer_hull"] = hull;

  nlohmann::json inner = nlohmann::json::array();
  for (const geometry_msgs::msg::Polygon &poly : msg->inner_contours) {
    nlohmann::json ring = nlohmann::json::array();
    for (const geometry_msgs::msg::Point32 &pt : poly.points) {
      ring.push_back(
          {{"x", std::round(static_cast<double>(pt.x) * 1000.0) / 1000.0},
           {"y", std::round(static_cast<double>(pt.y) * 1000.0) / 1000.0}});
    }
    inner.push_back(ring);
  }
  j["inner_polygons"] = inner;

  this->publishJson(this->warehouse_geometry_pub_, j.dump());

  RCLCPP_INFO(this->get_logger(),
              "Warehouse geometry: %zu wall segments, %zu hull points, %zu "
              "inner polygons",
              msg->outer_contour_segments.size(),
              msg->outer_contour_hull.points.size(),
              msg->inner_contours.size());
}

void FiwareBridgeNode::odomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  if (now - this->last_odom_time_ < this->odom_interval_)
    return;
  this->last_odom_time_ = now;

  nlohmann::json j;
  j["x"] = std::round(msg->pose.pose.position.x * 1000.0) / 1000.0;
  j["y"] = std::round(msg->pose.pose.position.y * 1000.0) / 1000.0;

  this->publishJson(this->robot_position_pub_, j.dump());
}

void FiwareBridgeNode::diagCallback(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &msg) {
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  if (now - this->last_diag_time_ < this->diag_interval_)
    return;
  this->last_diag_time_ = now;

  nlohmann::json j;
  for (const diagnostic_msgs::msg::DiagnosticStatus &status : msg->status) {
    const std::string node_name = status.name.substr(0, status.name.find(':'));
    nlohmann::json node_info;
    node_info["level"] = static_cast<int>(status.level);
    node_info["message"] = status.message;

    for (const diagnostic_msgs::msg::KeyValue &kv : status.values) {
      if (!isDiagKeyAcceptable(kv.key, kv.value)) {
        // Throttled DEBUG, not WARN: an attacker publishing to /diagnostics
        // could otherwise spam logs by intentionally sending rejected keys.
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Dropped diagnostic kv (reserved/oversized/"
                              "control-byte): keylen=%zu valuelen=%zu",
                              kv.key.size(), kv.value.size());
        continue;
      }
      node_info[kv.key] = kv.value;
    }
    j[node_name] = node_info;
  }

  if (!j.empty()) {
    this->publishJson(this->system_health_pub_, j.dump());
  }
}

void FiwareBridgeNode::heatmapCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg) {
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  if (now - this->last_heatmap_time_ < this->heatmap_interval_)
    return;
  this->last_heatmap_time_ = now;

  const double resolution = static_cast<double>(msg->info.resolution);
  const double origin_x = msg->info.origin.position.x;
  const double origin_y = msg->info.origin.position.y;
  // width/height are uint32 on the wire. Promote to size_t for index
  // arithmetic so width * height cannot overflow a signed int and so the
  // row/col loop indices stay in the same unsigned domain as data.size().
  const std::size_t width = static_cast<std::size_t>(msg->info.width);
  const std::size_t height = static_cast<std::size_t>(msg->info.height);
  const std::size_t expected_cells = width * height;
  if (width == 0 || height == 0 || expected_cells != msg->data.size()) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting OccupancyGrid: width=%zu height=%zu data.size=%zu "
                "(expected %zu)",
                width, height, msg->data.size(), expected_cells);
    return;
  }

  int nonzero = 0;
  int8_t max_val = 0;

  // Collect cells above threshold as (x, y, value) for Grafana visualization.
  // Cap at 200 cells to keep the payload size manageable for Orion-LD.
  constexpr int heatmap_cell_threshold = 10;
  constexpr std::size_t max_hot_cells = 200;
  nlohmann::json hot_cells = nlohmann::json::array();

  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t col = 0; col < width; ++col) {
      const int8_t cell = msg->data[row * width + col];
      if (cell > 0) {
        ++nonzero;
        max_val = std::max(max_val, cell);
      }
      if (cell >= heatmap_cell_threshold && hot_cells.size() < max_hot_cells) {
        const double world_x =
            origin_x + (static_cast<double>(col) + 0.5) * resolution;
        const double world_y =
            origin_y + (static_cast<double>(row) + 0.5) * resolution;
        hot_cells.push_back({{"x", std::round(world_x * 100.0) / 100.0},
                             {"y", std::round(world_y * 100.0) / 100.0},
                             {"v", static_cast<int>(cell)}});
      }
    }
  }

  nlohmann::json j;
  j["nonzero_cells"] = nonzero;
  j["max_value"] = static_cast<int>(max_val);
  j["grid_width"] = width;
  j["grid_height"] = height;
  j["resolution"] = msg->info.resolution;
  j["origin_x"] = origin_x;
  j["origin_y"] = origin_y;
  j["hot_cells"] = hot_cells;

  this->publishJson(this->heatmap_summary_pub_, j.dump());
}

void FiwareBridgeNode::mapUpdatesCallback(
    const rises_interfaces::msg::ObstacleUpdateArray::ConstSharedPtr &msg) {
  {
    std::lock_guard<std::mutex> guard(this->pallet_mutex_);
    for (const rises_interfaces::msg::ObstacleUpdate &update : msg->updates) {
      const int64_t obstacle_id = static_cast<int64_t>(update.obstacle.id);

      if (update.operation ==
          rises_interfaces::msg::ObstacleUpdate::OP_DELETE) {
        this->pallet_map_.erase(obstacle_id);
      } else {
        const float half_w = update.obstacle.width * 0.5f;
        const float half_h = update.obstacle.height * 0.5f;
        const float cx = static_cast<float>(update.obstacle.position.x);
        const float cy = static_cast<float>(update.obstacle.position.y);
        this->pallet_map_[obstacle_id] = {cx - half_w, cy - half_h, cx + half_w,
                                          cy + half_h};
      }
    }
    this->pallet_map_dirty_ = true;
  }

  // Throttle pushing — max every 2 seconds. last_pallet_push_time_ is only
  // written here, so no lock needed.
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  if (now - this->last_pallet_push_time_ < std::chrono::seconds(2))
    return;
  this->last_pallet_push_time_ = now;
  this->pushPalletMap();
}

void FiwareBridgeNode::loadPalletsFromJson(const std::string &path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    RCLCPP_WARN(this->get_logger(), "obstacles_json_file not found: %s",
                path.c_str());
    return;
  }

  nlohmann::json doc;
  try {
    f >> doc;
  } catch (const nlohmann::json::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse obstacles JSON: %s",
                 e.what());
    return;
  }

  if (!doc.contains("pallets") || !doc["pallets"].is_array()) {
    RCLCPP_ERROR(this->get_logger(), "obstacles JSON missing 'pallets' array");
    return;
  }

  std::size_t loaded = 0;
  {
    std::lock_guard<std::mutex> guard(this->pallet_mutex_);
    for (const auto &entry : doc["pallets"]) {
      if (!entry.contains("id") || !entry.contains("aabb"))
        continue;
      const auto &aabb = entry["aabb"];
      if (!aabb.is_array() || aabb.size() < 2)
        continue;
      const int64_t id = entry["id"].get<int64_t>();
      this->pallet_map_[id] = {
          static_cast<float>(aabb[0][0].get<double>()),
          static_cast<float>(aabb[0][1].get<double>()),
          static_cast<float>(aabb[1][0].get<double>()),
          static_cast<float>(aabb[1][1].get<double>()),
      };
      ++loaded;
    }
    this->pallet_map_dirty_ = true;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu pallets from %s", loaded,
              path.c_str());
  this->pushPalletMap();
}

void FiwareBridgeNode::pushPalletMap() {
  // Snapshot under the lock then release before publishing. publishJson can
  // block waiting on the DDS Enabler service; holding pallet_mutex_ across
  // that wait would serialise all pallet-map traffic behind a single I/O.
  std::size_t snapshot_size = 0;
  nlohmann::json rects = nlohmann::json::array();
  {
    std::lock_guard<std::mutex> guard(this->pallet_mutex_);
    if (!this->pallet_map_dirty_)
      return;
    this->pallet_map_dirty_ = false;

    snapshot_size = this->pallet_map_.size();
    for (const auto &[id, aabb] : this->pallet_map_) {
      rects.push_back({{"x_min", std::round(aabb.x_min * 1000.0) / 1000.0},
                       {"y_min", std::round(aabb.y_min * 1000.0) / 1000.0},
                       {"x_max", std::round(aabb.x_max * 1000.0) / 1000.0},
                       {"y_max", std::round(aabb.y_max * 1000.0) / 1000.0}});
    }
  }

  nlohmann::json j;
  j["count"] = static_cast<int>(snapshot_size);
  j["rectangles"] = std::move(rects);

  this->publishJson(this->map_obstacles_pub_, j.dump());

  RCLCPP_INFO(this->get_logger(), "Published all %zu map obstacles",
              snapshot_size);
}

void FiwareBridgeNode::republishGeometryCallback(
    const std_msgs::msg::Empty::ConstSharedPtr &msg) {
  (void)msg; // unused

  // Check if service is available
  if (!this->get_contours_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(),
                "Geofence get_warehouse_contours service not available");
    return;
  }

  // Call the service to get current contours
  auto request =
      std::make_shared<rises_interfaces::srv::GetWarehouseContours::Request>();
  auto future = this->get_contours_client_->async_send_request(request);

  // Wait for response with timeout
  auto status = future.wait_for(std::chrono::seconds(2));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timeout waiting for geofence contours service response");
    return;
  }

  auto response = future.get();

  // Convert the received contours to JSON and publish
  const rises_interfaces::msg::Contours &contours = response->contours;

  nlohmann::json j;

  nlohmann::json walls = nlohmann::json::array();
  for (const rises_interfaces::msg::LineSegment &seg :
       contours.outer_contour_segments) {
    walls.push_back(
        {{"x1", std::round(static_cast<double>(seg.start.x) * 1000.0) / 1000.0},
         {"y1", std::round(static_cast<double>(seg.start.y) * 1000.0) / 1000.0},
         {"x2", std::round(static_cast<double>(seg.end.x) * 1000.0) / 1000.0},
         {"y2", std::round(static_cast<double>(seg.end.y) * 1000.0) / 1000.0}});
  }
  j["wall_segments"] = walls;

  nlohmann::json hull = nlohmann::json::array();
  for (const geometry_msgs::msg::Point32 &pt :
       contours.outer_contour_hull.points) {
    hull.push_back(
        {{"x", std::round(static_cast<double>(pt.x) * 1000.0) / 1000.0},
         {"y", std::round(static_cast<double>(pt.y) * 1000.0) / 1000.0}});
  }
  j["outer_hull"] = hull;

  nlohmann::json inner = nlohmann::json::array();
  for (const geometry_msgs::msg::Polygon &poly : contours.inner_contours) {
    nlohmann::json ring = nlohmann::json::array();
    for (const geometry_msgs::msg::Point32 &pt : poly.points) {
      ring.push_back(
          {{"x", std::round(static_cast<double>(pt.x) * 1000.0) / 1000.0},
           {"y", std::round(static_cast<double>(pt.y) * 1000.0) / 1000.0}});
    }
    inner.push_back(ring);
  }
  j["inner_polygons"] = inner;

  this->publishJson(this->warehouse_geometry_pub_, j.dump());

  RCLCPP_INFO(this->get_logger(),
              "Requested and republished warehouse geometry via service call");
}

void FiwareBridgeNode::ddsTriggerCallback(
    const std_msgs::msg::Empty::ConstSharedPtr &msg) {
  (void)msg; // unused

  RCLCPP_INFO(this->get_logger(), "Received DDS trigger for map data refresh");

  // Republish pallet map immediately (force-dirty so pushPalletMap doesn't skip
  // it). Brief lock to flip the dirty flag; pushPalletMap takes its own lock.
  {
    std::lock_guard<std::mutex> guard(this->pallet_mutex_);
    this->pallet_map_dirty_ = true;
  }
  this->last_pallet_push_time_ =
      std::chrono::steady_clock::time_point{}; // reset throttle
  this->pushPalletMap();

  // Check if service is available
  if (!this->get_contours_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(),
                "Geofence get_warehouse_contours service not available");
    return;
  }

  // Call the service to get current contours
  auto request =
      std::make_shared<rises_interfaces::srv::GetWarehouseContours::Request>();
  auto future = this->get_contours_client_->async_send_request(request);

  // Wait for response with timeout
  auto status = future.wait_for(std::chrono::seconds(2));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timeout waiting for geofence contours service response");
    return;
  }

  auto response = future.get();

  // Convert the received contours to JSON and publish
  const rises_interfaces::msg::Contours &contours = response->contours;

  nlohmann::json j;

  nlohmann::json walls = nlohmann::json::array();
  for (const rises_interfaces::msg::LineSegment &seg :
       contours.outer_contour_segments) {
    walls.push_back(
        {{"x1", std::round(static_cast<double>(seg.start.x) * 1000.0) / 1000.0},
         {"y1", std::round(static_cast<double>(seg.start.y) * 1000.0) / 1000.0},
         {"x2", std::round(static_cast<double>(seg.end.x) * 1000.0) / 1000.0},
         {"y2", std::round(static_cast<double>(seg.end.y) * 1000.0) / 1000.0}});
  }
  j["wall_segments"] = walls;

  nlohmann::json hull = nlohmann::json::array();
  for (const geometry_msgs::msg::Point32 &pt :
       contours.outer_contour_hull.points) {
    hull.push_back(
        {{"x", std::round(static_cast<double>(pt.x) * 1000.0) / 1000.0},
         {"y", std::round(static_cast<double>(pt.y) * 1000.0) / 1000.0}});
  }
  j["outer_hull"] = hull;

  nlohmann::json inner = nlohmann::json::array();
  for (const geometry_msgs::msg::Polygon &poly : contours.inner_contours) {
    nlohmann::json ring = nlohmann::json::array();
    for (const geometry_msgs::msg::Point32 &pt : poly.points) {
      ring.push_back(
          {{"x", std::round(static_cast<double>(pt.x) * 1000.0) / 1000.0},
           {"y", std::round(static_cast<double>(pt.y) * 1000.0) / 1000.0}});
    }
    inner.push_back(ring);
  }
  j["inner_polygons"] = inner;

  this->publishJson(this->warehouse_geometry_pub_, j.dump());

  RCLCPP_INFO(this->get_logger(),
              "Refreshed warehouse geometry via DDS trigger");
}

void FiwareBridgeNode::validationResultCallback(
    const std_msgs::msg::String::ConstSharedPtr &msg) {
  // The validation node already publishes well-formed JSON — relay it directly.
  this->publishJson(this->validation_result_pub_, msg->data);
}

void FiwareBridgeNode::publishJson(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub,
    const std::string &json) {
  std_msgs::msg::String msg;
  msg.data = json;
  pub->publish(msg);
}

} // namespace rises

RCLCPP_COMPONENTS_REGISTER_NODE(rises::FiwareBridgeNode)
