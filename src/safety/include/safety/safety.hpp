#pragma once

// Project headers
#include "rises_interfaces/msg/obstacle_array.hpp"
#include "rises_interfaces/msg/obstacle_update.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/srv/validate_path.hpp"

// Third-party (ROS 2)
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// Standard library
#include <atomic>
#include <unordered_map>

namespace rises {

class Safety : public rclcpp::Node {
public:
  explicit Safety(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Subscribers
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_sub_;
  rclcpp::Subscription<rises_interfaces::msg::ObstacleArray>::SharedPtr
      detected_obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_sub_;

  // Publishers
  rclcpp::Publisher<rises_interfaces::msg::ObstacleArray>::SharedPtr alert_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr validated_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr halt_pub_;

  // Service client
  rclcpp::Client<rises_interfaces::srv::ValidatePath>::SharedPtr
      validate_path_client_;

  // Diagnostics monitoring
  struct NodeHealth {
    uint8_t level = 0; // 0=OK, 1=WARN, 2=ERROR
    rclcpp::Time last_seen;
  };
  std::unordered_map<std::string, NodeHealth> monitored_nodes_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;
  double node_timeout_sec_ = 10.0;
  std::atomic<bool> system_halted_{false};

  // Heatmap-based path overlap.
  //
  // Concurrency contract: latest_heatmap_ is written by heatmapCallback
  // and read by pathCallback/isPathOverlappingPrediction, which run on
  // different executor threads under a MultiThreadedExecutor. The
  // shared_ptr's control-block refcount and the pointer slot itself are
  // therefore shared mutable state. We protect them with the
  // std::atomic_store / std::atomic_load free-function overloads from
  // <memory>. The lock-free shared_ptr atomic free functions are
  // deprecated in C++20 (replaced by std::atomic<std::shared_ptr<T>>),
  // but this package requires C++17, so the free-function form is the
  // correct choice here. Reading code MUST
  // take a local snapshot via std::atomic_load BEFORE touching the grid
  // — never dereference the member directly.
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_heatmap_;
  int heatmap_overlap_threshold_ = 50; // 0-100 probability threshold

  // Callbacks
  void diagnosticsCallback(
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void detectedObstaclesCallback(
      const rises_interfaces::msg::ObstacleArray::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void heatmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /// @brief Periodically checks if monitored nodes are still reporting healthy
  /// diagnostics.
  void healthCheckCallback();

  /// @brief Checks if the given path overlaps with high-probability cells in
  /// the supplied heatmap snapshot.
  /// @param path     Path to test against the heatmap.
  /// @param heatmap  Local snapshot of the latest heatmap, obtained via
  ///                 std::atomic_load(&latest_heatmap_) by the caller. May be
  ///                 nullptr; the predicate returns false in that case. Taking
  ///                 the snapshot at the call site (and not inside this
  ///                 predicate) guarantees we hold a single, stable
  ///                 shared_ptr for the entire scan even if heatmapCallback
  ///                 swaps latest_heatmap_ concurrently.
  /// @return true if any path waypoint overlaps with predicted obstacle
  /// presence.
  [[nodiscard]] bool isPathOverlappingPrediction(
      const nav_msgs::msg::Path &path,
      const nav_msgs::msg::OccupancyGrid::SharedPtr &heatmap) const;

  /// @brief Publishes a halt command and logs the reason.
  void haltSystem(const std::string &reason);

  /// @brief Resumes the system after a halt condition clears.
  void resumeSystem(const std::string &reason);
};
} // namespace rises
