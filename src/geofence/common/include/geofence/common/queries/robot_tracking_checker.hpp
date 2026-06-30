/**
 * @file robot_tracking_checker.hpp
 * @brief Multi-robot tracking checker for filtering robot footprints from
 * detected obstacles
 *
 * This checker maintains a registry of multiple robots and their footprints,
 * allowing filtering of detected obstacles that correspond to known robot
 * positions.
 *
 * @author geofence team
 * @date 2025
 */

#pragma once

#include "rises_interfaces/msg/obstacle.hpp"
#include "geofence/common/policies/robot_tracking.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace rises {
namespace geofence {

/**
 * @brief Multi-robot tracking checker that filters obstacles based on known
 * robot positions
 *
 * This class maintains a registry of multiple robots with their footprints and
 * poses, and provides functionality to check if detected obstacles correspond
 * to known robots.
 */
class RobotTrackingChecker {
public:
  struct Config {
    /// Maximum age (in milliseconds) for robot poses to be considered valid
    uint64_t max_pose_age_ms;

    /// Safety margin to expand robot footprints for conservative matching
    double footprint_expansion_margin;

    /// Enable debug logging for robot tracking
    bool enable_debug_logging;

    // Default constructor with default values
    Config()
        : max_pose_age_ms(2000), footprint_expansion_margin(0.1),
          enable_debug_logging(false) {}
  };

  /**
   * @brief Construct robot tracking checker with configuration
   * @param config Configuration parameters
   */
  explicit RobotTrackingChecker(const Config &config = Config{});

  /**
   * @brief Register a robot with its footprint
   * @param robot_id Unique identifier for the robot
   * @param footprint Robot's geometric footprint
   */
  void
  registerRobot(const std::string &robot_id,
                const rises::geofence::policies::RobotFootprint &footprint);

  /**
   * @brief Update pose for a registered robot
   * @param robot_id Robot identifier
   * @param pose Updated pose with timestamp
   */
  void updateRobotPose(const std::string &robot_id,
                       const rises::geofence::policies::RobotPose &pose);

  /**
   * @brief Update robot pose from ROS message
   * @param robot_id Robot identifier
   * @param pose_msg ROS pose message
   */
  void updateRobotPose(const std::string &robot_id,
                       const geometry_msgs::msg::PoseStamped &pose_msg);

  /**
   * @brief Check if detected obstacle corresponds to any known robot
   * @param obstacle Detected obstacle to check
   * @param max_age_ms Maximum age for robot poses to consider (optional
   * override)
   * @return true if obstacle matches a known robot footprint
   */
  bool isObstacleFromRobot(const rises_interfaces::msg::Obstacle &obstacle,
                           uint64_t max_age_ms = 0) const;

  /**
   * @brief Get robot ID that matches the obstacle (if any)
   * @param obstacle Detected obstacle to check
   * @param max_age_ms Maximum age for robot poses to consider
   * @return Robot ID if matched, empty string if no match
   */
  std::string
  getMatchingRobotId(const rises_interfaces::msg::Obstacle &obstacle,
                     uint64_t max_age_ms = 0) const;

  /**
   * @brief Remove robot from tracking registry
   * @param robot_id Robot to remove
   */
  void unregisterRobot(const std::string &robot_id);

  /**
   * @brief Get list of currently registered robot IDs
   * @return Vector of robot IDs
   */
  std::vector<std::string> getRegisteredRobots() const;

  /**
   * @brief Clear all robot registrations and poses
   */
  void clear();

  /**
   * @brief Get configuration
   * @return Current configuration
   */
  const Config &getConfig() const { return config_; }

private:
  Config config_;

  /// Guards robot_footprints_ and robot_poses_ against concurrent access
  /// between robotPoseCallback (writer) and obstaclesCallback (reader) under
  /// a MultiThreadedExecutor. unordered_map insertion can rehash and
  /// invalidate iterators / bucket pointers held by a concurrent reader.
  mutable std::mutex mutex_;

  /// Registry of robot footprints indexed by robot ID
  std::unordered_map<std::string, rises::geofence::policies::RobotFootprint>
      robot_footprints_;

  /// Latest poses for each robot indexed by robot ID
  std::unordered_map<std::string, rises::geofence::policies::RobotPose>
      robot_poses_;

  /**
   * @brief Get current timestamp in nanoseconds
   * @return Current time as nanoseconds since epoch
   */
  uint64_t getCurrentTimestamp() const;

  /**
   * @brief Check if robot pose is recent enough to be valid
   * @param pose Robot pose with timestamp
   * @param max_age_ms Maximum allowed age in milliseconds
   * @return true if pose is within age limit
   */
  bool isPoseValid(const rises::geofence::policies::RobotPose &pose,
                   uint64_t max_age_ms) const;
};

} // namespace geofence
} // namespace rises