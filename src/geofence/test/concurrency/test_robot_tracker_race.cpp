// =============================================================================
// Phase 3 concurrency test: RobotTrackingChecker robot_poses_ / footprints_
// =============================================================================
//
// Audit finding covered:
//   geofence/spatial_node/include/geofence/spatial/queries/
//     robot_tracking_checker.hpp lines 126-128.
//     - robot_footprints_ : std::unordered_map<std::string, RobotFootprint>
//     - robot_poses_      : std::unordered_map<std::string, RobotPose>
//   Both maps are written via registerRobot/updateRobotPose (typically driven
//   by /robot_pose callbacks in the spatial node) and read via
//   isObstacleFromRobot / getMatchingRobotId (driven by /obstacles
//   callbacks). Neither map is protected by a mutex.
//
// Race exercised:
//   Writer thread spams updateRobotPose for kRobotCount distinct robots in a
//   hot loop. Reader thread queries isObstacleFromRobot for synthetic
//   obstacles. unordered_map insertion can trigger a rehash which invalidates
//   iterators currently held by a concurrent find()/begin()/end() in the
//   reader path. TSan reports the data race on the hash-table internals.
//
// TSan invocation:
//   colcon build --packages-select geofence \
//     --cmake-args -DBUILD_TESTING=ON -DARIS_ENABLE_TSAN=ON
//   colcon test --packages-select geofence \
//     --ctest-args -R test_robot_tracker_race
//
// Expected TSan output: data race on the internal nodes of the unordered_map
// (typically reported as "Write of size N at 0x... by thread T1" plus "Read
// of size N ... by thread T2") between updateRobotPose() and
// getMatchingRobotId().
//
// Standards: function cap 100, nesting <= 3, named constants.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rises_interfaces/msg/obstacle.hpp"
#include "geofence/common/policies/robot_tracking.hpp"
#include "geofence/common/queries/robot_tracking_checker.hpp"

namespace {

constexpr int kRobotCount = 8;
constexpr auto kRunWindow = std::chrono::milliseconds(500);
constexpr double kFootprintRadius = 0.3;
constexpr double kPoseStep = 0.05;
constexpr int kRehashRobotPoolSize = 64;

std::string robotId(int i) { return "robot_" + std::to_string(i); }

rises::geofence::policies::RobotFootprint defaultFootprint() {
  return rises::geofence::policies::RobotFootprint::createCircle(
      kFootprintRadius);
}

rises::geofence::policies::RobotPose makePose(double x, double y) {
  rises::geofence::policies::RobotPose pose;
  pose.x = x;
  pose.y = y;
  pose.theta = 0.0;
  // Always-fresh timestamp so isPoseValid passes.
  const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
  pose.timestamp_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
  return pose;
}

rises_interfaces::msg::Obstacle makeProbeObstacle(double x, double y) {
  rises_interfaces::msg::Obstacle obs;
  obs.id = 1;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.position.x = x;
  obs.position.y = y;
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  obs.vertices.push_back(p);
  return obs;
}

} // namespace

// -----------------------------------------------------------------------------
// ConcurrentPoseUpdateAndQueryIsRaceFree
// -----------------------------------------------------------------------------
TEST(RobotTrackerRace, ConcurrentPoseUpdateAndQueryIsRaceFree) {
  rises::geofence::RobotTrackingChecker checker;
  for (int i = 0; i < kRobotCount; ++i) {
    checker.registerRobot(robotId(i), defaultFootprint());
  }

  std::atomic<bool> stop{false};
  std::atomic<bool> producer_failed{false};

  std::thread writer([&]() {
    try {
      double x = 0.0;
      while (!stop.load(std::memory_order_acquire)) {
        for (int i = 0; i < kRobotCount; ++i) {
          checker.updateRobotPose(robotId(i), makePose(x + i, x + i));
        }
        x += kPoseStep;
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::thread reader([&]() {
    try {
      double y = 0.0;
      while (!stop.load(std::memory_order_acquire)) {
        const rises_interfaces::msg::Obstacle probe = makeProbeObstacle(y, y);
        (void)checker.isObstacleFromRobot(probe);
        y += kPoseStep;
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(kRunWindow);
  stop.store(true, std::memory_order_release);

  writer.join();
  reader.join();

  EXPECT_FALSE(producer_failed.load())
      << "concurrent updateRobotPose + isObstacleFromRobot threw — likely a "
         "torn unordered_map node access";
}

// -----------------------------------------------------------------------------
// IteratorInvalidationGuard
// -----------------------------------------------------------------------------
//
// Drives insertion AND removal of robots in the writer so the bucket array
// rehashes. The reader iterates over robot_footprints_ (indirectly, inside
// getMatchingRobotId) at the same time. TSan should flag the rehash as a
// data race with the reader; on develop the test still completes without
// crashing because each iteration is short enough that the bucket pointers
// typically remain valid. Without TSan the assertion is "no crash, no
// exception".
TEST(RobotTrackerRace, IteratorInvalidationGuard) {
  rises::geofence::RobotTrackingChecker checker;
  // Pre-populate so removal in the writer is meaningful.
  for (int i = 0; i < kRehashRobotPoolSize; ++i) {
    checker.registerRobot(robotId(i), defaultFootprint());
    checker.updateRobotPose(robotId(i), makePose(i, i));
  }

  std::atomic<bool> stop{false};
  std::atomic<bool> producer_failed{false};

  std::thread mutator([&]() {
    int cursor = 0;
    try {
      while (!stop.load(std::memory_order_acquire)) {
        const std::string id = robotId(cursor % kRehashRobotPoolSize);
        if ((cursor & 1) == 0) {
          checker.unregisterRobot(id);
        } else {
          checker.registerRobot(id, defaultFootprint());
          checker.updateRobotPose(id, makePose(cursor, cursor));
        }
        ++cursor;
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::thread reader([&]() {
    try {
      double y = 0.0;
      while (!stop.load(std::memory_order_acquire)) {
        const rises_interfaces::msg::Obstacle probe = makeProbeObstacle(y, y);
        (void)checker.getMatchingRobotId(probe);
        y += kPoseStep;
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(kRunWindow);
  stop.store(true, std::memory_order_release);

  mutator.join();
  reader.join();

  EXPECT_FALSE(producer_failed.load())
      << "rehash-driven mutation racing with getMatchingRobotId threw";
}
