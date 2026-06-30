#include "geofence/common/queries/robot_tracking_checker.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace rises {
namespace geofence {

RobotTrackingChecker::RobotTrackingChecker(const Config &config)
    : config_(config) {}

void RobotTrackingChecker::registerRobot(
    const std::string &robot_id,
    const rises::geofence::policies::RobotFootprint &footprint) {
  const std::lock_guard<std::mutex> guard(this->mutex_);
  this->robot_footprints_[robot_id] = footprint;
}

void RobotTrackingChecker::updateRobotPose(
    const std::string &robot_id,
    const rises::geofence::policies::RobotPose &pose) {
  const std::lock_guard<std::mutex> guard(this->mutex_);
  if (this->robot_footprints_.find(robot_id) != this->robot_footprints_.end()) {
    this->robot_poses_[robot_id] = pose;
  }
}

void RobotTrackingChecker::updateRobotPose(
    const std::string &robot_id,
    const geometry_msgs::msg::PoseStamped &pose_msg) {
  rises::geofence::policies::RobotPose pose;
  pose.x = pose_msg.pose.position.x;
  pose.y = pose_msg.pose.position.y;

  // Extract yaw from quaternion
  double qw = pose_msg.pose.orientation.w;
  double qz = pose_msg.pose.orientation.z;
  pose.theta = 2.0 * std::atan2(qz, qw);

  pose.timestamp_ns =
      static_cast<uint64_t>(pose_msg.header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(pose_msg.header.stamp.nanosec);

  this->updateRobotPose(robot_id, pose);
}

bool RobotTrackingChecker::isObstacleFromRobot(
    const rises_interfaces::msg::Obstacle &obstacle,
    uint64_t max_age_ms) const {
  return !this->getMatchingRobotId(obstacle, max_age_ms).empty();
}

std::string RobotTrackingChecker::getMatchingRobotId(
    const rises_interfaces::msg::Obstacle &obstacle,
    uint64_t max_age_ms) const {
  const std::lock_guard<std::mutex> guard(this->mutex_);
  if (max_age_ms == 0) {
    max_age_ms = this->config_.max_pose_age_ms;
  }

  for (const std::pair<const std::string,
                       rises::geofence::policies::RobotFootprint>
           &robot_footprint_pair : this->robot_footprints_) {
    const std::string &robot_id = robot_footprint_pair.first;
    const rises::geofence::policies::RobotFootprint &footprint =
        robot_footprint_pair.second;

    const std::unordered_map<
        std::string, rises::geofence::policies::RobotPose>::const_iterator
        pose_it = this->robot_poses_.find(robot_id);
    if (pose_it == this->robot_poses_.end()) {
      continue;
    }

    const rises::geofence::policies::RobotPose &pose = pose_it->second;

    if (!this->isPoseValid(pose, max_age_ms)) {
      continue;
    }

    if (footprint.containsObstacle(obstacle, pose,
                                   this->config_.footprint_expansion_margin)) {
      return robot_id;
    }
  }

  return ""; // No matching robot found
}

void RobotTrackingChecker::unregisterRobot(const std::string &robot_id) {
  const std::lock_guard<std::mutex> guard(this->mutex_);
  this->robot_footprints_.erase(robot_id);
  this->robot_poses_.erase(robot_id);
}

std::vector<std::string> RobotTrackingChecker::getRegisteredRobots() const {
  const std::lock_guard<std::mutex> guard(this->mutex_);
  std::vector<std::string> robot_ids;
  robot_ids.reserve(this->robot_footprints_.size());

  for (const std::pair<const std::string,
                       rises::geofence::policies::RobotFootprint>
           &robot_footprint_pair : this->robot_footprints_) {
    robot_ids.push_back(robot_footprint_pair.first);
  }

  return robot_ids;
}

void RobotTrackingChecker::clear() {
  const std::lock_guard<std::mutex> guard(this->mutex_);
  this->robot_footprints_.clear();
  this->robot_poses_.clear();
}

uint64_t RobotTrackingChecker::getCurrentTimestamp() const {
  const std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  const std::chrono::nanoseconds duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

bool RobotTrackingChecker::isPoseValid(
    const rises::geofence::policies::RobotPose &pose,
    uint64_t max_age_ms) const {
  const uint64_t current_time_ns = this->getCurrentTimestamp();
  const uint64_t max_age_ns = max_age_ms * 1000000ULL;

  return (current_time_ns - pose.timestamp_ns) <= max_age_ns;
}

} // namespace geofence
} // namespace rises