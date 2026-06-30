// Unit tests for rises::Vda5050StateBridge.
//
// Package: fleet_interface
// Strategy:
//   - The bridge is a non-Node helper that hangs publishers, subscribers and a
//     timer off a host rclcpp::Node and reads pose from a shared
//     tf2_ros::Buffer. Tests stand up the bridge against a real test Node,
//     drive inputs via publishers on the same executor, and observe outputs by
//     subscribing to the two String topics it publishes ("obstacle_state" and
//     "/mqtt/agv/<name>/obstacle_state").
//   - Time is driven via wall-clock spin slices bounded by a deadline (the
//     bridge's timer is a wall timer, so /clock injection cannot drive it).
//     All tests cap spin time with kSpinDeadline so they remain deterministic
//     and bounded; assertions are on outputs, not on wall-clock duration.
//   - JSON output is parsed with nlohmann::json (same dependency the bridge
//     already pulls in) to avoid brittle substring matching.
//
// GTEST_SKIP rationale:
//   - BatteryStateForwarded: the production Vda5050StateBridge has no battery
//     subscription and emits no "batteryState" field. The test documents the
//     contract requested by the spec and skips with an explicit message until
//     the seam is implemented.
//   - ErrorStateForwarded: the production bridge has no diagnostics
//     subscription and emits no "errors[]" array. Same SKIP rationale as
//     BatteryStateForwarded.
//   - EmptyStateAtStartup: the bridge does NOT publish on construction. It
//     republishes the last cached report from its periodic timer, and does
//     nothing if no report has ever arrived. The "sensible defaults at
//     startup" expectation is therefore not satisfiable without a new
//     production-side seam (e.g. an explicit "publish empty state on init"
//     mode). This test documents that and skips.

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <rises_interfaces/msg/obstacle_report.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>

#include "fleet_interface/vda5050_state_bridge.hpp"
#include "test_support/obstacle_builder.hpp"

namespace {

using rises_interfaces::msg::Obstacle;
using rises_interfaces::msg::ObstacleReport;

constexpr const char *kGlobalFrame = "map";
constexpr const char *kBaseFrame = "base_link";
constexpr const char *kMqttAgvName = "id0";
constexpr double kStateRateHz = 20.0;
constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr std::chrono::milliseconds kSpinDeadline{1000};
constexpr double kPosTolerance = 1e-6;

class StateBridgeFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void buildBridge() {
    host_node_ = std::make_shared<rclcpp::Node>("test_state_bridge_host");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(host_node_->get_clock());

    rises::Vda5050StateBridge::Config config;
    config.position_source = "tf";
    config.state_publish_rate = kStateRateHz;
    config.global_frame = kGlobalFrame;
    config.target_frame = kBaseFrame;
    config.tf_prefix = "";
    config.mqtt_agv_name = kMqttAgvName;

    bridge_ = std::make_unique<rises::Vda5050StateBridge>(host_node_.get(),
                                                          config, tf_buffer_);

    pub_node_ = std::make_shared<rclcpp::Node>("test_state_bridge_pub");
    report_pub_ = pub_node_->create_publisher<ObstacleReport>(
        "/test_state_bridge_host/obstacle_report", 10);

    spy_node_ = std::make_shared<rclcpp::Node>("test_state_bridge_spy");
    captured_state_.clear();
    state_sub_ = spy_node_->create_subscription<std_msgs::msg::String>(
        "/test_state_bridge_host/obstacle_state", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          captured_state_.push_back(msg->data);
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(host_node_);
    executor_->add_node(pub_node_);
    executor_->add_node(spy_node_);
  }

  void TearDown() override {
    executor_.reset();
    state_sub_.reset();
    report_pub_.reset();
    bridge_.reset();
    tf_buffer_.reset();
    spy_node_.reset();
    pub_node_.reset();
    host_node_.reset();
    captured_state_.clear();
  }

  void seedTransform(double x, double y) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = host_node_->now();
    tf.header.frame_id = kGlobalFrame;
    tf.child_frame_id = kBaseFrame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(tf, "test", /*is_static=*/false);
  }

  bool spinUntil(const std::function<bool()> &predicate) {
    const auto deadline = std::chrono::steady_clock::now() + kSpinDeadline;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    return false;
  }

  void publishReportWithUnmatched(double obstacle_x, double obstacle_y) {
    ObstacleReport report;
    report.header.frame_id = kGlobalFrame;
    report.header.stamp = host_node_->now();
    report.unmatched_obstacles.push_back(
        test_support::ObstacleBuilder::point(1, obstacle_x, obstacle_y));
    report_pub_->publish(report);
  }

  std::optional<nlohmann::json> lastStateJson() const {
    if (captured_state_.empty()) {
      return std::nullopt;
    }
    try {
      return nlohmann::json::parse(captured_state_.back());
    } catch (const nlohmann::json::parse_error &) {
      return std::nullopt;
    }
  }

  std::shared_ptr<rclcpp::Node> host_node_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  std::shared_ptr<rclcpp::Node> spy_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<rises::Vda5050StateBridge> bridge_;
  rclcpp::Publisher<ObstacleReport>::SharedPtr report_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::string> captured_state_;
};

constexpr double kRobotX = 3.25;
constexpr double kRobotY = -1.5;
constexpr double kObstacleX = 7.0;
constexpr double kObstacleY = 8.0;

} // namespace

// A TF update from map -> base_link, followed by an obstacle report, must
// produce a published state JSON whose agvPosition matches the seeded TF
// translation. This is the "publishes on TF update" contract: the bridge's
// publish path consults TF on every report, so a TF update is observable only
// once the report drives an emit.
TEST_F(StateBridgeFixture, StateBridgePublishesOnTfUpdate) {
  buildBridge();
  seedTransform(kRobotX, kRobotY);
  publishReportWithUnmatched(kObstacleX, kObstacleY);

  ASSERT_TRUE(spinUntil([this]() { return !captured_state_.empty(); }))
      << "Bridge did not publish state JSON after report + TF update.";

  const auto state = lastStateJson();
  ASSERT_TRUE(state.has_value()) << "Last state was not parseable JSON.";
  ASSERT_TRUE(state->contains("agvPosition")) << state->dump();
  EXPECT_NEAR((*state)["agvPosition"]["x"].get<double>(), kRobotX,
              kPosTolerance);
  EXPECT_NEAR((*state)["agvPosition"]["y"].get<double>(), kRobotY,
              kPosTolerance);
}

// With no TF available the bridge must still republish the last known report
// on its periodic timer without throwing and without populating agvPosition.
// This pins the "missing TF holds last state" contract.
TEST_F(StateBridgeFixture, MissingTfHoldsLastState) {
  buildBridge();
  // Deliberately do NOT seed a transform.
  publishReportWithUnmatched(kObstacleX, kObstacleY);

  ASSERT_TRUE(spinUntil([this]() { return captured_state_.size() >= 2u; }))
      << "Bridge did not republish on its periodic timer without TF.";

  // No exception escapes the bridge.
  EXPECT_NO_THROW({
    const auto state = lastStateJson();
    ASSERT_TRUE(state.has_value());
    // No agvPosition because TF is unavailable.
    EXPECT_FALSE(state->contains("agvPosition")) << state->dump();
    // safetyState is still present — last state is preserved.
    ASSERT_TRUE(state->contains("safetyState"));
    EXPECT_TRUE((*state)["safetyState"]["fieldViolation"].get<bool>());
  });
}

// Pins expected batteryState forwarding; skipped until the bridge subscribes
// to a battery topic.
TEST_F(StateBridgeFixture, BatteryStateForwarded) {
  GTEST_SKIP_(
      "Vda5050StateBridge does not subscribe to a battery topic and does not "
      "emit batteryState. Re-enable once a sensor_msgs/BatteryState "
      "subscription "
      "is added and the published JSON exposes a 'batteryState' object.");
}

// Pins expected error-array forwarding; skipped until the bridge subscribes to
// diagnostics.
TEST_F(StateBridgeFixture, ErrorStateForwarded) {
  GTEST_SKIP_(
      "Vda5050StateBridge does not subscribe to "
      "diagnostic_msgs/DiagnosticArray "
      "and does not emit an 'errors' array. Re-enable once the diagnostics "
      "forwarding seam is implemented.");
}

// Pins expected default state at startup; skipped until the bridge has an
// explicit publish-on-init path.
TEST_F(StateBridgeFixture, EmptyStateAtStartup) {
  GTEST_SKIP_(
      "Vda5050StateBridge does not publish until an obstacle_report or alert "
      "is received. Re-enable once an explicit startup-publish seam exposes a "
      "default state with a valid header and no NaN fields.");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
