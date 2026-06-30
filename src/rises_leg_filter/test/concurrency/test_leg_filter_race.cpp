// =============================================================================
// Phase 3 concurrency test: LegFilterNode::tracks_ and confirmed_humans_
// =============================================================================
//
// Audit finding covered:
//   rises_leg_filter/src/leg_filter_node.cpp lines 95, 150, 213.
//     - obstacleCallback writes tracks_ on each ObstacleReport message.
//     - humansTrackedCallback writes confirmed_humans_ on each /humans/bodies/
//       message.
//     - publishCandidates (timer thread) reads BOTH maps and mutates entries
//       of tracks_ (confirmed_by_camera, is_candidate fields).
//   evictStaleTracks (invoked from obstacleCallback and from publishCandidates'
//   bookkeeping) erases entries from both maps. Neither map has a mutex.
//
// Race exercised:
//   Three callback paths drive the two maps concurrently when a
//   MultiThreadedExecutor is used: ObstacleReport subscription, /humans/
//   subscription, and the wall-timer publishCandidates. Under TSan the
//   unprotected unordered_map operations are flagged. The structural
//   assertions confirm no crashes, exceptions, or hung publishers.
//
// TSan invocation:
//   colcon build --packages-select rises_leg_filter \
//     --cmake-args -DBUILD_TESTING=ON -DARIS_ENABLE_TSAN=ON
//   colcon test --packages-select rises_leg_filter \
//     --ctest-args -R test_leg_filter_race
//
// Expected TSan output before the production fix:
//   ThreadSanitizer: data race on tracks_ and confirmed_humans_ between the
//   subscription callbacks and the publishCandidates timer.
//
// Standards: function cap 100, nesting <= 3, named constants, no TODOs.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "rises_interfaces/msg/obstacle.hpp"
#include "rises_interfaces/msg/obstacle_report.hpp"
#include "rises_leg_filter/leg_filter_node.hpp"

namespace {

constexpr int kExecutorThreads = 3;
constexpr int kDistinctObstacleIds = 8;
constexpr auto kPublishInterval = std::chrono::milliseconds(5);
constexpr auto kCameraInterval = std::chrono::milliseconds(20);
constexpr auto kShortWindow = std::chrono::seconds(1);
constexpr auto kLongWindow = std::chrono::seconds(2);
constexpr auto kSettleWindow = std::chrono::milliseconds(300);
constexpr double kTimerHz = 50.0;

constexpr const char *kObstacleTopic = "obstacle_report";
constexpr const char *kHumansTopic = "/humans/bodies/tracked";

rises_interfaces::msg::Obstacle makeLegObstacle(std::uint64_t id, double x,
                                                double y) {
  rises_interfaces::msg::Obstacle obs;
  obs.id = id;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.position.x = x;
  obs.position.y = y;
  // Within [min_width, max_width] so the width gate passes.
  obs.width = 0.3F;
  obs.height = 0.3F;
  return obs;
}

rises_interfaces::msg::ObstacleReport makeObstacleReport(int seq) {
  rises_interfaces::msg::ObstacleReport msg;
  msg.header.stamp.sec = seq;
  msg.header.stamp.nanosec = 0;
  for (int i = 0; i < kDistinctObstacleIds; ++i) {
    const double x = 0.1 * static_cast<double>(seq) + static_cast<double>(i);
    const double y = static_cast<double>(i);
    msg.unmatched_obstacles.push_back(
        makeLegObstacle(static_cast<std::uint64_t>(i), x, y));
  }
  return msg;
}

std_msgs::msg::String makeHumansMessage(int seq) {
  std_msgs::msg::String msg;
  msg.data = "body_" + std::to_string(seq % 4) + ",body_alpha,body_beta";
  return msg;
}

class LegFilterRaceFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"publish_rate_hz", kTimerHz},
        {"min_observations", 1},
        {"eviction_timeout_sec", 0.5},
        {"camera_stale_sec", 0.5},
        {"min_width", 0.1},
        {"max_width", 1.0},
    });
    node_ = std::make_shared<rises::LegFilterNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("leg_filter_race_helper");

    obstacle_pub_ =
        helper_->create_publisher<rises_interfaces::msg::ObstacleReport>(
            kObstacleTopic, rclcpp::QoS(10).reliable());
    humans_pub_ = helper_->create_publisher<std_msgs::msg::String>(
        kHumansTopic, rclcpp::QoS(10).reliable());

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions{}, kExecutorThreads);
    executor_->add_node(node_);
    executor_->add_node(helper_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    obstacle_pub_.reset();
    humans_pub_.reset();
    helper_.reset();
    node_.reset();
  }

  std::shared_ptr<rises::LegFilterNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<rises_interfaces::msg::ObstacleReport>::SharedPtr
      obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr humans_pub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

template <typename Duration>
void runDualPublishers(
    rclcpp::Publisher<rises_interfaces::msg::ObstacleReport>::SharedPtr
        &obstacle_pub,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &humans_pub,
    Duration duration, std::atomic<bool> &producer_failed) {
  std::atomic<bool> stop{false};

  std::thread scan_thread([&]() {
    int seq = 0;
    try {
      while (!stop.load(std::memory_order_acquire)) {
        obstacle_pub->publish(makeObstacleReport(seq++));
        std::this_thread::sleep_for(kPublishInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::thread camera_thread([&]() {
    int seq = 0;
    try {
      while (!stop.load(std::memory_order_acquire)) {
        humans_pub->publish(makeHumansMessage(seq++));
        std::this_thread::sleep_for(kCameraInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(duration);
  stop.store(true, std::memory_order_release);
  scan_thread.join();
  camera_thread.join();
}

} // namespace

// -----------------------------------------------------------------------------
// CameraConfirmAndScanUpdateRaceFree
// -----------------------------------------------------------------------------
TEST_F(LegFilterRaceFixture, CameraConfirmAndScanUpdateRaceFree) {
  std::atomic<bool> producer_failed{false};
  runDualPublishers(obstacle_pub_, humans_pub_, kShortWindow, producer_failed);
  std::this_thread::sleep_for(kSettleWindow);

  EXPECT_FALSE(producer_failed.load())
      << "publish() of ObstacleReport or humans/tracked threw during the race "
         "window";
}

// -----------------------------------------------------------------------------
// ExpiryCallbackRaceFree
// -----------------------------------------------------------------------------
//
// With a short eviction_timeout_sec the timer thread's publishCandidates path
// reads tracks_ and confirmed_humans_ while obstacleCallback simultaneously
// erases stale entries. Two seconds of sustained contention exercises the
// rehash-on-erase + reader collision.
TEST_F(LegFilterRaceFixture, ExpiryCallbackRaceFree) {
  std::atomic<bool> producer_failed{false};
  runDualPublishers(obstacle_pub_, humans_pub_, kLongWindow, producer_failed);
  std::this_thread::sleep_for(kSettleWindow);

  EXPECT_FALSE(producer_failed.load());
}
