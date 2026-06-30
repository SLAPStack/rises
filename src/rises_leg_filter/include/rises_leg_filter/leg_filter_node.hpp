/// @file leg_filter_node.hpp
/// @brief Filters unmatched geofence obstacles for likely human leg-pairs.
///
/// Subscribes to obstacle_report from the geofence node and reads its
/// unmatched_obstacles[] array, applying width, velocity, and persistence
/// heuristics to identify obstacles that are likely human legs. Publishes
/// candidate detections to the ROS4HRI /humans/bodies/ namespace with low
/// confidence, providing 360-degree LiDAR-based human detection that the
/// ROS4HRI person manager can fuse with camera-based detections.
///
/// Optionally subscribes to /humans/bodies/tracked to boost confidence
/// when a LiDAR candidate overlaps with a camera-confirmed human.
///
/// This node is fully optional. The geofence operates identically without it.

#pragma once

#include "rises_interfaces/msg/obstacle_report.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rises {

/// @brief Per-obstacle tracking state for the leg filter.
struct LegTrack {
  double last_x = 0.0;
  double last_y = 0.0;
  double last_seen_sec = 0.0;
  double first_seen_sec = 0.0;

  /// Rolling displacement accumulator for velocity estimation.
  double displacement_sum = 0.0;
  double time_span = 0.0;
  int observation_count = 0;

  /// Previous position for displacement calculation.
  double prev_x = 0.0;
  double prev_y = 0.0;
  bool has_prev = false;

  /// Whether this track is currently classified as a likely human.
  bool is_candidate = false;

  /// Boosted by /humans/ confirmation.
  bool confirmed_by_camera = false;
};

/// @brief Filters unmatched obstacles for likely human legs and publishes
///        to the ROS4HRI /humans/bodies/ namespace.
class LegFilterNode : public rclcpp::Node {
public:
  explicit LegFilterNode(const rclcpp::NodeOptions &options);

private:
  // --- Subscriptions ---
  rclcpp::Subscription<rises_interfaces::msg::ObstacleReport>::SharedPtr
      obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr humans_tracked_sub_;

  // --- Publishers ---
  /// Publishes a comma-separated list of body IDs on /humans/bodies/tracked.
  /// When hri_msgs becomes available, change type to hri_msgs::msg::IdsList.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bodies_tracked_pub_;

  // --- TF broadcaster for body frames ---
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // --- Periodic publish timer ---
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // --- Track state ---
  //
  // Concurrency contract: tracks_ and confirmed_humans_ are both mutated
  // from subscription callbacks (obstacleCallback writes tracks_;
  // humansTrackedCallback writes confirmed_humans_) and read together by
  // publishCandidates on the timer thread. publishCandidates also writes
  // back into LegTrack fields (confirmed_by_camera, is_candidate). Under a
  // MultiThreadedExecutor any of those can fire on different threads.
  //
  // We use a single mutex covering both maps because passesLegFilter and
  // matchesCameraHuman walk them together; splitting the lock would force
  // careful lock ordering with no measurable upside. publishCandidates
  // snapshots the data it needs under the lock, then releases the lock
  // before doing any ROS publish or TF broadcast, so I/O does not serialize
  // with incoming scans.
  mutable std::mutex state_mutex_;
  std::unordered_map<uint64_t, LegTrack> tracks_;

  // --- Camera-confirmed human positions (from /humans/) ---
  struct ConfirmedHuman {
    double x = 0.0;
    double y = 0.0;
    double timestamp_sec = 0.0;
  };
  std::unordered_map<std::string, ConfirmedHuman> confirmed_humans_;

  // --- Configuration (cached from parameters) ---
  double min_width_; ///< Minimum obstacle width to consider (meters).
  double max_width_; ///< Maximum obstacle width to consider (meters).
  double
      min_velocity_; ///< Minimum average velocity to classify as moving (m/s).
  double
      stationary_timeout_; ///< Reject if stationary longer than this (seconds).
  double eviction_timeout_; ///< Remove tracks not seen for this long (seconds).
  int min_observations_;    ///< Minimum observations before classifying.
  double publish_rate_hz_;  ///< How often to publish tracked bodies.
  double base_confidence_;  ///< Confidence for LiDAR-only detections (0-1).
  double boosted_confidence_;  ///< Confidence when confirmed by camera (0-1).
  double camera_match_radius_; ///< Max distance to match LiDAR track to camera
                               ///< human (m).
  double camera_stale_sec_; ///< Discard camera data older than this (seconds).
  /// Gate for camera-fusion path. Off by default until the TF lookup on the
  /// body_<id> frame replaces the (0,0) placeholder in humansTrackedCallback.
  /// While off, /humans/bodies/tracked is consumed only to keep a heartbeat
  /// of confirmed IDs; legs are never marked confirmed_by_camera. Audit #12.
  bool enable_camera_fusion_;
  std::string frame_id_; ///< Reference frame for published TF.

  // --- Callbacks ---
  void obstacleCallback(
      const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg);

  void humansTrackedCallback(const std_msgs::msg::String::ConstSharedPtr &msg);

  /// @brief Periodic: evaluate all tracks, publish candidates to /humans/.
  void publishCandidates();

  /// @brief Evicts stale tracks not seen within eviction_timeout_.
  void evictStaleTracks(double current_sec);

  /// @brief Checks if a track passes the leg-pair heuristic.
  [[nodiscard]] bool passesLegFilter(const LegTrack &track,
                                     double current_sec) const;

  /// @brief Checks if a LiDAR track is near a camera-confirmed human.
  [[nodiscard]] bool matchesCameraHuman(double x, double y,
                                        double current_sec) const;
};

} // namespace rises
