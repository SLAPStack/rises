// =============================================================================
// Gap-closure unit tests for RobotTrackingChecker (single-threaded only).
//
// Source under test:
//   geofence/common/src/queries/robot_tracking_checker.cpp
//   geofence/common/include/geofence/common/queries/robot_tracking_checker.hpp
//
// Audit flagged concurrent access to the pose map as unsafe; concurrency
// coverage is owned by Phase 3 -- these tests are deliberately single-threaded
// and cover the correctness of update, lookup, timeout eviction, multi-robot
// disambiguation, frame-id handling, and the empty-state contract.
// =============================================================================

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "test_support/obstacle_builder.hpp"

#include "geofence/common/policies/robot_tracking.hpp"
#include "geofence/common/queries/robot_tracking_checker.hpp"

namespace {

using rises::geofence::RobotTrackingChecker;
using rises::geofence::policies::RobotFootprint;
using rises::geofence::policies::RobotPose;

constexpr double kRobotRadius = 0.5;
constexpr double kFootprintMargin = 0.1;
constexpr uint64_t kPoseAgeMs = 1000;
constexpr uint64_t kNsPerMs = 1'000'000ULL;
constexpr uint64_t kNsPerSec = 1'000'000'000ULL;

// Build a pose anchored at the current monotonic wall time. Tests that need
// stale poses subtract from this value rather than calling getCurrentTimestamp.
uint64_t nowNs() {
  const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

RobotPose poseAt(double x, double y, uint64_t timestamp_ns) {
  RobotPose pose;
  pose.x = x;
  pose.y = y;
  pose.theta = 0.0;
  pose.timestamp_ns = timestamp_ns;
  return pose;
}

geometry_msgs::msg::PoseStamped poseStamped(double x, double y, double t_sec,
                                            const std::string &frame_id) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp.sec = static_cast<int32_t>(t_sec);
  msg.header.stamp.nanosec = 0U;
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.orientation.w = 1.0;
  return msg;
}

} // namespace

TEST(RobotTracking, PoseUpdatedOnPublish) {
  RobotTrackingChecker::Config cfg;
  cfg.max_pose_age_ms = kPoseAgeMs;
  RobotTrackingChecker checker(cfg);

  const std::string robot_id = "robot_0";
  checker.registerRobot(
      robot_id, RobotFootprint::createCircle(kRobotRadius, kFootprintMargin));

  // Publish a pose at "now" -- the obstacle below sits at the robot centre
  // and so must match the registered footprint.
  checker.updateRobotPose(robot_id, poseAt(/*x=*/5.0, /*y=*/5.0, nowNs()));

  const auto obstacle =
      test_support::ObstacleBuilder::point(/*id=*/1U, /*x=*/5.0, /*y=*/5.0);
  EXPECT_TRUE(checker.isObstacleFromRobot(obstacle));
  EXPECT_EQ(checker.getMatchingRobotId(obstacle), robot_id);
}

TEST(RobotTracking, PoseTimeoutEvicts) {
  RobotTrackingChecker::Config cfg;
  cfg.max_pose_age_ms = kPoseAgeMs;
  RobotTrackingChecker checker(cfg);

  const std::string robot_id = "robot_0";
  checker.registerRobot(
      robot_id, RobotFootprint::createCircle(kRobotRadius, kFootprintMargin));

  // Publish a pose stamped well past the timeout window in the past.
  const uint64_t stale_ns = nowNs() - (kPoseAgeMs * 2 * kNsPerMs);
  checker.updateRobotPose(robot_id, poseAt(/*x=*/5.0, /*y=*/5.0, stale_ns));

  const auto obstacle =
      test_support::ObstacleBuilder::point(/*id=*/1U, /*x=*/5.0, /*y=*/5.0);
  // Pose is older than max_pose_age_ms -> matcher rejects it as expired.
  EXPECT_FALSE(checker.isObstacleFromRobot(obstacle));
  EXPECT_TRUE(checker.getMatchingRobotId(obstacle).empty());
}

TEST(RobotTracking, MultiRobotDisambiguation) {
  RobotTrackingChecker::Config cfg;
  cfg.max_pose_age_ms = kPoseAgeMs;
  RobotTrackingChecker checker(cfg);

  checker.registerRobot(
      "robot_a", RobotFootprint::createCircle(kRobotRadius, kFootprintMargin));
  checker.registerRobot(
      "robot_b", RobotFootprint::createCircle(kRobotRadius, kFootprintMargin));

  const uint64_t now = nowNs();
  checker.updateRobotPose("robot_a", poseAt(/*x=*/0.0, /*y=*/0.0, now));
  checker.updateRobotPose("robot_b", poseAt(/*x=*/100.0, /*y=*/100.0, now));

  const auto near_a =
      test_support::ObstacleBuilder::point(/*id=*/1U, /*x=*/0.1, /*y=*/0.1);
  const auto near_b =
      test_support::ObstacleBuilder::point(/*id=*/2U, /*x=*/100.1,
                                           /*y=*/100.1);

  EXPECT_EQ(checker.getMatchingRobotId(near_a), std::string("robot_a"));
  EXPECT_EQ(checker.getMatchingRobotId(near_b), std::string("robot_b"));
}

TEST(RobotTracking, FrameIdMismatchHandled) {
  // The ROS-message overload of updateRobotPose does not consult the frame
  // id -- the production code unconditionally stores the pose. This test
  // pins that contract: a pose in any frame_id is accepted as-is. If/when a
  // frame-id check is added (audit follow-up), the second EXPECT_TRUE below
  // should flip to EXPECT_FALSE and the comment updated.
  RobotTrackingChecker::Config cfg;
  cfg.max_pose_age_ms = kPoseAgeMs * 1000U; // Generous timeout for the test.
  RobotTrackingChecker checker(cfg);

  const std::string robot_id = "robot_0";
  checker.registerRobot(
      robot_id, RobotFootprint::createCircle(kRobotRadius, kFootprintMargin));

  // Use a future-ish stamp so the pose is considered "recent" relative to
  // wall-clock now() in isPoseValid().
  const double t_sec = static_cast<double>(nowNs() / kNsPerSec);
  checker.updateRobotPose(
      robot_id, poseStamped(/*x=*/5.0, /*y=*/5.0, t_sec, "wrong_frame"));

  const auto obstacle =
      test_support::ObstacleBuilder::point(/*id=*/1U, /*x=*/5.0, /*y=*/5.0);
  EXPECT_TRUE(checker.isObstacleFromRobot(obstacle));
}

TEST(RobotTracking, EmptyOnNoUpdates) {
  RobotTrackingChecker::Config cfg;
  cfg.max_pose_age_ms = kPoseAgeMs;
  RobotTrackingChecker checker(cfg);

  // Register without publishing any pose. Lookups must return the sentinel.
  checker.registerRobot(
      "robot_0", RobotFootprint::createCircle(kRobotRadius, kFootprintMargin));

  const auto obstacle =
      test_support::ObstacleBuilder::point(/*id=*/1U, /*x=*/5.0, /*y=*/5.0);
  EXPECT_FALSE(checker.isObstacleFromRobot(obstacle));
  EXPECT_TRUE(checker.getMatchingRobotId(obstacle).empty());

  const std::vector<std::string> registered = checker.getRegisteredRobots();
  ASSERT_EQ(registered.size(), 1U);
  EXPECT_EQ(registered.front(), std::string("robot_0"));
}
