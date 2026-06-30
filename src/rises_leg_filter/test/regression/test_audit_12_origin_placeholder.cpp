// Regression test for audit finding #12 (origin-placeholder camera fusion).
//
// Bug location: rises_leg_filter/src/leg_filter_node.cpp:150-156
// (humansTrackedCallback
//   inserts ConfirmedHuman with x=0, y=0 as a TF-lookup placeholder), exploited
//   by matchesCameraHuman() at the same source file's later loop which then
//   produces false-positive "confirmed_by_camera" for any leg candidate that
//   happens to be near the world origin.
//
// Expected post-fix behaviour: a leg candidate that has no real camera match
// must NOT be marked confirmed_by_camera. The fix is allowed to either
//   (a) skip the placeholder insertion altogether until TF integration lands,
//   or
//   (b) gate the camera-fusion path behind a feature flag that is default-off,
// per audit recommendation. Either path makes the tests below GREEN.
//
// Detection mechanism: we cannot read confirmed_by_camera directly from the
// private LegTrack, so we observe the externally-visible side effect — when a
// leg candidate passes the filter only because of camera confirmation (the
// stationary path), the node publishes its body id on
// /humans/bodies/lidar_tracked. A leg that is stationary, near origin, and has
// no real camera match must not appear on that topic.
//
// Status on develop:
//   - LegAtOriginIsNotSpuriouslyConfirmed: RED on develop (this is the bug).
//   - LegFarFromOriginUnaffected: positive control, GREEN on develop and after
//     the fix.
//   - LegMatchedByRealCameraIsConfirmed: RED on develop and STAYS red until
//     the separate TF-integration PR replaces the (0,0) placeholder with a
//     real TF lookup.
//   - EmptyCameraTracksMeansNoConfirmation: positive control, GREEN on
//     develop and after the fix.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "rises_leg_filter/leg_filter_node.hpp"

namespace {

using rises_interfaces::msg::Obstacle;
using rises_interfaces::msg::ObstacleArray;

constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr int kMaxSpinIterations = 250;
constexpr int kBurstObservationCount = 8; // > default min_observations (5).
constexpr double kPublishRateHz = 50.0;
constexpr double kStationaryTimeoutSec = 0.5; // shrink so test stays fast.
constexpr double kEvictionTimeoutSec = 2.0;
constexpr double kBurstDt = 0.05; // 50 ms between observations.

class LegFilterFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void buildNode() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"min_width", 0.15},
        {"max_width", 0.8},
        {"min_velocity", 0.05},
        {"stationary_timeout_sec", kStationaryTimeoutSec},
        {"eviction_timeout_sec", kEvictionTimeoutSec},
        {"min_observations", 3},
        {"publish_rate_hz", kPublishRateHz},
        {"frame_id", std::string("map")},
        {"base_confidence", 0.3},
        {"boosted_confidence", 0.7},
        {"camera_match_radius", 1.5},
        {"camera_stale_sec", 2.0},
    });

    node_ = std::make_shared<rises::LegFilterNode>(options);

    publisher_node_ = std::make_shared<rclcpp::Node>("test_leg_filter_pub");
    obstacle_pub_ = publisher_node_->create_publisher<ObstacleArray>(
        "unmatched_obstacles", rclcpp::SensorDataQoS().keep_last(10));
    humans_pub_ = publisher_node_->create_publisher<std_msgs::msg::String>(
        "/humans/bodies/tracked", rclcpp::QoS(10).reliable());

    spy_node_ = std::make_shared<rclcpp::Node>("test_leg_filter_spy");
    last_published_ids_.clear();
    spy_sub_ = spy_node_->create_subscription<std_msgs::msg::String>(
        "/humans/bodies/lidar_tracked", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          last_published_ids_.push_back(msg->data);
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(publisher_node_);
    executor_->add_node(spy_node_);
  }

  void TearDown() override {
    executor_.reset();
    spy_sub_.reset();
    obstacle_pub_.reset();
    humans_pub_.reset();
    spy_node_.reset();
    publisher_node_.reset();
    node_.reset();
    last_published_ids_.clear();
  }

  bool spinUntil(const std::function<bool()> &predicate,
                 int max_iterations = kMaxSpinIterations) {
    for (int i = 0; i < max_iterations; ++i) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    return false;
  }

  void spinFor(std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      std::this_thread::sleep_for(kSpinSlice);
    }
  }

  /// Publishes @p kBurstObservationCount stationary observations of a leg
  /// candidate at (@p x, @p y) with width=0.3 (inside leg-width gate).
  /// All observations share the same obstacle id so the track accumulates.
  void publishStationaryLeg(std::uint64_t id, double x, double y) {
    rclcpp::Time base = node_->now();
    for (int i = 0; i < kBurstObservationCount; ++i) {
      ObstacleArray array;
      const double seconds_offset = i * kBurstDt;
      array.header.stamp =
          base + rclcpp::Duration::from_seconds(seconds_offset);
      array.header.frame_id = "map";
      Obstacle obs;
      obs.id = id;
      obs.type = Obstacle::POINT;
      obs.width = 0.3f;
      obs.position.x = x;
      obs.position.y = y;
      array.obstacles.push_back(obs);
      obstacle_pub_->publish(array);
      spinFor(std::chrono::milliseconds(10));
    }
  }

  bool seenIdContaining(const std::string &token) {
    for (const std::string &payload : last_published_ids_) {
      if (payload.find(token) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<rises::LegFilterNode> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::Node> spy_node_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr humans_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spy_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::string> last_published_ids_;
};

} // namespace

// A leg candidate near the world origin, while a /humans/bodies/tracked id is
// active (which today inserts a (0,0)-placeholder ConfirmedHuman entry), must
// NOT be spuriously confirmed. Post-fix we expect either (a) no placeholder is
// inserted, or (b) the camera-fusion path is gated default-off, so the leg
// stays unpublished. Pre-fix the (0,0) placeholder lies within the leg's
// camera_match_radius, so confirmed_by_camera flips true and the stationary
// track is published — that is the RED failure mode this test detects.
TEST_F(LegFilterFixture, LegAtOriginIsNotSpuriouslyConfirmed) {
  buildNode();

  // Trigger the placeholder insertion via a /humans/bodies/tracked message.
  std_msgs::msg::String tracked;
  tracked.data = "ghost_id";
  humans_pub_->publish(tracked);
  spinFor(std::chrono::milliseconds(50));

  publishStationaryLeg(/*id=*/1, /*x=*/0.1, /*y=*/0.1);
  // Let stationary_timeout elapse and the periodic publisher run.
  spinFor(std::chrono::milliseconds(800));

  EXPECT_FALSE(seenIdContaining("lidar_1"))
      << "Leg near origin was spuriously confirmed by camera placeholder.";
}

// Same expectation for a leg far from the origin: no camera => not confirmed.
TEST_F(LegFilterFixture, LegFarFromOriginUnaffected) {
  buildNode();

  publishStationaryLeg(/*id=*/2, /*x=*/50.0, /*y=*/50.0);
  spinFor(std::chrono::milliseconds(800));

  EXPECT_FALSE(seenIdContaining("lidar_2"))
      << "Stationary leg far from origin must not be confirmed without "
         "a real camera match.";
}

// A leg with a real camera match SHOULD be confirmed. As long as TF lookup
// remains unimplemented in production this test is expected to stay RED — it
// becomes GREEN once the TF integration replacing the (0,0) placeholder lands.
TEST_F(LegFilterFixture, LegMatchedByRealCameraIsConfirmed) {
  buildNode();

  // Publish a tracked-human id so the node knows there is a camera body.
  std_msgs::msg::String ids;
  ids.data = "alice";
  humans_pub_->publish(ids);
  spinFor(std::chrono::milliseconds(50));

  publishStationaryLeg(/*id=*/3, /*x=*/10.0, /*y=*/10.0);
  spinFor(std::chrono::milliseconds(800));

  EXPECT_TRUE(seenIdContaining("lidar_3"))
      << "Leg with a real camera match should be confirmed and published "
         "(test stays RED until TF lookup replaces (0,0) placeholder).";
}

// Without any camera messages, no legs should ever be marked confirmed —
// even when geometrically stationary near the world origin.
TEST_F(LegFilterFixture, EmptyCameraTracksMeansNoConfirmation) {
  buildNode();

  publishStationaryLeg(/*id=*/4, /*x=*/0.0, /*y=*/0.0);
  publishStationaryLeg(/*id=*/5, /*x=*/0.5, /*y=*/0.5);
  spinFor(std::chrono::milliseconds(800));

  EXPECT_FALSE(seenIdContaining("lidar_4"));
  EXPECT_FALSE(seenIdContaining("lidar_5"));
}
