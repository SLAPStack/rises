/// @file fiware_bridge_node.hpp
/// @brief Translates complex ROS 2 messages into FIWARE-friendly JSON strings.
///
/// Subscribes to geofence topics (ObstacleReport, Contours, diagnostics, etc.)
/// and publishes simplified std_msgs/String (JSON) on fiware/* topics.
/// The DDS Enabler forwards these simple topics to Orion-LD.
///
/// No FIWARE, NGSI-LD, or HTTP dependencies — purely a ROS 2 topic translator.

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rises_interfaces/msg/contours.hpp"
#include "rises_interfaces/msg/obstacle_report.hpp"
#include "rises_interfaces/msg/obstacle_update.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/srv/get_warehouse_contours.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rises {

class FiwareBridgeNode : public rclcpp::Node {
public:
  explicit FiwareBridgeNode(const rclcpp::NodeOptions &options);

private:
  // --- Subscriptions ---
  rclcpp::Subscription<rises_interfaces::msg::ObstacleReport>::SharedPtr
      report_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alert_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<rises_interfaces::msg::Contours>::SharedPtr
      contours_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diag_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_sub_;
  rclcpp::Subscription<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr
      map_updates_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr republish_geometry_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr dds_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr validation_result_sub_;

  // --- Publishers (fiware/* topics, std_msgs/String with JSON) ---
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_summary_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_alert_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr geofence_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warehouse_geometry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heatmap_summary_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_obstacles_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_segments_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr validation_result_pub_;

  // --- Throttle state (wall clock) ---
  std::chrono::steady_clock::time_point last_report_time_{};
  std::chrono::steady_clock::time_point last_odom_time_{};
  std::chrono::steady_clock::time_point last_diag_time_{};
  std::chrono::steady_clock::time_point last_heatmap_time_{};

  std::chrono::duration<double> report_interval_;
  std::chrono::duration<double> odom_interval_;
  std::chrono::duration<double> diag_interval_;
  std::chrono::duration<double> heatmap_interval_;

  // --- On-change state ---
  bool last_alert_value_ = false;
  bool last_ready_value_ = false;
  bool alert_initialized_ = false;
  bool ready_initialized_ = false;

  // --- Service client for requesting contours ---
  rclcpp::Client<rises_interfaces::srv::GetWarehouseContours>::SharedPtr
      get_contours_client_;

  // --- Config ---
  int max_unmatched_positions_;
  int max_matched_positions_;

  // --- Accumulated pallet map from map_updates ---
  // Concurrency: pallet_map_ + pallet_map_dirty_ are mutated by callbacks
  // running on the MultiThreadedExecutor (mapUpdatesCallback,
  // ddsTriggerCallback, republishGeometryCallback) and read by pushPalletMap,
  // which can execute on any of the executor threads. The dirty flag is
  // logically tied to the map's contents (publish-if-dirty); a single mutex
  // covers both. last_pallet_push_ _time_ is only read/written inside the same
  // callbacks, so it lives under the same mutex for consistency. CRITICAL:
  // never hold pallet_mutex_ across a publish() / wait_for_service() call.
  // Lock, snapshot, unlock, then publish.
  struct PalletAABB {
    float x_min, y_min, x_max, y_max;
  };
  mutable std::mutex pallet_mutex_;
  std::unordered_map<int64_t, PalletAABB> pallet_map_;
  bool pallet_map_dirty_ = false;
  std::chrono::steady_clock::time_point last_pallet_push_time_{};

  // --- Startup helpers ---
  void loadPalletsFromJson(const std::string &path);

  // --- Callbacks ---
  void reportCallback(
      const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg);
  void alertCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
  void readyCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
  void
  contoursCallback(const rises_interfaces::msg::Contours::ConstSharedPtr &msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
  void diagCallback(
      const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &msg);
  void heatmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg);
  void mapUpdatesCallback(
      const rises_interfaces::msg::ObstacleUpdateArray::ConstSharedPtr &msg);
  void pushPalletMap();
  void
  republishGeometryCallback(const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void ddsTriggerCallback(const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void
  validationResultCallback(const std_msgs::msg::String::ConstSharedPtr &msg);

  /// @brief Publishes a JSON string on the given publisher.
  void
  publishJson(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub,
              const std::string &json);
};

} // namespace rises
