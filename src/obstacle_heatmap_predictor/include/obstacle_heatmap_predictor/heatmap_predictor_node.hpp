/// @file heatmap_predictor_node.hpp
/// @brief Predicts future obstacle positions and publishes an occupancy grid
/// heatmap.
///
/// Subscribes to obstacle_report from the geofence node, tracks unmatched
/// obstacle trajectories using their persistent segment IDs, estimates linear
/// velocity, and projects positions forward in time. The result is an
/// OccupancyGrid where each cell represents the probability (0-100) of an
/// obstacle being present at that location within the prediction horizon.
///
/// This addresses the ARISE ROS4HRI requirement for predictive human-robot
/// interaction: "develop a module that extends current ROS4HRI perception to
/// predict potential human positions in the next 30-60 seconds (e.g.,
/// generating heatmaps of probable human presence), which could be valuable for
/// path planning or predictive geofencing."

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "rises_interfaces/msg/obstacle_report.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace rises {

/// @brief A single timestamped position observation of an obstacle.
struct Observation {
  double x;
  double y;
  double timestamp_sec;
};

/// @brief Tracked obstacle with observation history and estimated velocity.
struct TrackedObstacle {
  std::deque<Observation> history;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double last_seen_sec = 0.0;
};

/// @brief Predicts obstacle positions and publishes a heatmap occupancy grid.
class HeatmapPredictorNode : public rclcpp::Node {
public:
  explicit HeatmapPredictorNode(const rclcpp::NodeOptions &options);

private:
  // Subscriptions
  rclcpp::Subscription<rises_interfaces::msg::ObstacleReport>::SharedPtr
      obstacle_report_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_pub_;

  // Timer for periodic heatmap publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Tracked obstacles keyed by persistent segment ID.
  //
  // Concurrency contract: tracked_obstacles_ is mutated by
  // obstacleReportCallback (subscription thread) and read by publishHeatmap
  // (timer thread); under a MultiThreadedExecutor these can fire on different
  // executor threads. The map is non-trivial state (rehash + erase invalidate
  // iterators) so a mutex is the correct tool here -- an atomic_shared_ptr
  // swap would force a full deep copy of the map on every report, which is
  // strictly more expensive than the brief critical section. publishHeatmap
  // snapshots the map under the lock and then releases the lock before doing
  // any ROS publish, so incoming reports are not serialized with the I/O.
  mutable std::mutex tracks_mutex_;
  std::unordered_map<uint64_t, TrackedObstacle> tracked_obstacles_;

  // Configuration (cached from parameters)
  double observation_window_sec_; // How long to keep observations (seconds)
  double prediction_horizon_sec_; // How far ahead to predict (seconds)
  double prediction_step_sec_;    // Time step for prediction samples
  double eviction_timeout_sec_;   // Remove tracks not seen for this long
  double grid_resolution_;        // Meters per cell
  double grid_width_;             // Grid width in meters
  double grid_height_;            // Grid height in meters
  double grid_origin_x_;          // Grid origin X (meters, bottom-left)
  double grid_origin_y_;          // Grid origin Y (meters, bottom-left)
  double gaussian_sigma_;  // Spatial spread of each prediction point (meters)
  double publish_rate_hz_; // Heatmap publish rate
  int min_observations_;   // Minimum observations before predicting
  std::string frame_id_;   // Frame for the occupancy grid

  // Grid dimensions in cells
  int grid_cols_ = 0;
  int grid_rows_ = 0;

  /// @brief Callback for obstacle report messages.
  void obstacleReportCallback(
      const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg);

  /// @brief Updates tracked obstacle with a new observation.
  void updateTrack(uint64_t id, double x, double y, double timestamp_sec);

  /// @brief Estimates linear velocity from the observation history using
  /// least-squares fit.
  void estimateVelocity(TrackedObstacle &track) const;

  /// @brief Evicts stale tracks not seen within the timeout.
  void evictStaleTracks(double current_time_sec);

  /// @brief Builds and publishes the prediction heatmap.
  void publishHeatmap();

  /// @brief Stamps a Gaussian blob onto the probability grid at (x, y) with
  /// given weight.
  /// @param grid Flat probability grid (grid_rows_ * grid_cols_).
  /// @param x World X coordinate.
  /// @param y World Y coordinate.
  /// @param weight Probability weight (0.0 - 1.0).
  void stampGaussian(std::vector<float> &grid, double x, double y,
                     float weight) const;
};

} // namespace rises
