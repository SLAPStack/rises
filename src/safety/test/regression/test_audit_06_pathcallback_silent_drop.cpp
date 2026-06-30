// Regression tests for audit finding 06: Safety::pathCallback handling of an
// unavailable ValidatePath service.
//
// These tests verify that when the ValidatePath service is missing,
// pathCallback raises a halt (true on /halt, "HALT: ..." on /response) and
// publishes an explicit rejection on /response; that an available service is
// forwarded and its validated path republished; and that an "unsafe" result
// from the service still raises a halt.
//
// Standards binding:
//   - Real Safety class instantiated as rclcpp::Node subclass; no mocks of
//     rclcpp.
//   - Deterministic. No sleep_for over 100ms; we spin_some with bounded
//     timeouts.
//   - Function cap 100 lines, nesting <= 3, named constants for magic
//     values.

#include <chrono>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "rises_interfaces/srv/validate_path.hpp"
#include "safety/safety.hpp"

namespace {

constexpr auto kSpinPollStep = std::chrono::milliseconds(5);
constexpr int kSpinIterations = 20; // total bounded wait = 100 ms
constexpr const char *kPathTopic = "incoming_path";
constexpr const char *kHaltTopic = "halt";
constexpr const char *kResponseTopic = "response";
constexpr const char *kValidatePathService = "validate_path";
constexpr const char *kRejectionSubstring = "rejected";
constexpr const char *kHaltSubstring = "HALT";

// Spin the executor in small slices so callbacks fire without blocking the
// test for long. Total wait bounded by kSpinIterations * kSpinPollStep.
void spinBriefly(rclcpp::Executor &executor) {
  for (int i = 0; i < kSpinIterations; ++i) {
    executor.spin_some();
    rclcpp::sleep_for(kSpinPollStep);
  }
}

class SafetyPathHarness : public ::testing::Test {
protected:
  // Default-constructed executor_ member needs a valid rclcpp context. Member
  // initialization runs BEFORE SetUp(), so the init must happen at suite
  // scope.
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"validate_path_service_name", std::string(kValidatePathService)},
        {"node_timeout_sec", 60.0},
    });
    safety_node_ = std::make_shared<rises::Safety>(options);
    helper_node_ = std::make_shared<rclcpp::Node>("safety_path_test_helper");

    halt_received_ = false;
    halt_value_ = false;
    response_messages_.clear();

    halt_sub_ = helper_node_->create_subscription<std_msgs::msg::Bool>(
        kHaltTopic, rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          halt_received_ = true;
          halt_value_ = msg->data;
        });
    response_sub_ = helper_node_->create_subscription<std_msgs::msg::String>(
        kResponseTopic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          response_messages_.push_back(msg->data);
        });
    path_pub_ =
        helper_node_->create_publisher<nav_msgs::msg::Path>(kPathTopic, 10);

    executor_.add_node(safety_node_);
    executor_.add_node(helper_node_);
  }

  void TearDown() override {
    executor_.remove_node(helper_node_);
    executor_.remove_node(safety_node_);
    helper_node_.reset();
    safety_node_.reset();
  }

  nav_msgs::msg::Path makeTrivialPath() const {
    nav_msgs::msg::Path path;
    path.header.stamp = helper_node_->now();
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 0.0;
    path.poses.push_back(pose);
    return path;
  }

  bool responseContains(const std::string &needle) const {
    for (const std::string &line : response_messages_) {
      if (line.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<rises::Safety> safety_node_;
  std::shared_ptr<rclcpp::Node> helper_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr halt_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr response_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  bool halt_received_ = false;
  bool halt_value_ = false;
  std::vector<std::string> response_messages_;
};

// Stub ValidatePath server used by the "service available" tests. Replies
// synchronously with a configurable blocked flag.
class ValidatePathStub {
public:
  ValidatePathStub(rclcpp::Node::SharedPtr node, bool reply_blocked)
      : node_(std::move(node)), reply_blocked_(reply_blocked) {
    using rises_interfaces::srv::ValidatePath;
    server_ = node_->create_service<ValidatePath>(
        kValidatePathService,
        [this](const std::shared_ptr<ValidatePath::Request> /*req*/,
               std::shared_ptr<ValidatePath::Response> resp) {
          resp->blocked = reply_blocked_;
          resp->reason = reply_blocked_ ? "stub: blocked" : "stub: clear";
          ++call_count_;
        });
  }

  int callCount() const { return call_count_; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<rises_interfaces::srv::ValidatePath>::SharedPtr server_;
  bool reply_blocked_;
  int call_count_ = 0;
};

} // namespace

TEST_F(SafetyPathHarness, MissingServiceTriggersHaltSystem) {
  // No ValidatePath server is registered. pathCallback must escalate.
  nav_msgs::msg::Path path = makeTrivialPath();
  path_pub_->publish(path);

  spinBriefly(executor_);

  EXPECT_TRUE(halt_received_)
      << "Safety must publish on /halt when ValidatePath service is missing";
  EXPECT_TRUE(halt_value_) << "Halt message payload must be true";
}

TEST_F(SafetyPathHarness, MissingServicePublishesRejection) {
  nav_msgs::msg::Path path = makeTrivialPath();
  path_pub_->publish(path);

  spinBriefly(executor_);

  EXPECT_TRUE(responseContains(kRejectionSubstring))
      << "Safety must publish an explicit rejection on /response when "
         "ValidatePath service is missing";
}

TEST_F(SafetyPathHarness, BothActionsAreAtomicNoPartialState) {
  // Atomicity contract: after pathCallback returns, observers see BOTH a
  // halt event AND a rejection response. Never one without the other.
  nav_msgs::msg::Path path = makeTrivialPath();
  path_pub_->publish(path);

  spinBriefly(executor_);

  const bool saw_halt = halt_received_ && halt_value_;
  const bool saw_rejection =
      responseContains(kRejectionSubstring) || responseContains(kHaltSubstring);

  EXPECT_TRUE(saw_halt && saw_rejection)
      << "Expected both halt AND rejection. halt_received=" << halt_received_
      << " halt_value=" << halt_value_
      << " response_count=" << response_messages_.size();
}

TEST_F(SafetyPathHarness, AvailableServiceForwardsRequest) {
  // Control case: when ValidatePath is reachable and replies "not blocked",
  // Safety must NOT halt. Path is forwarded to /validated_path.
  ValidatePathStub stub(helper_node_, /*reply_blocked=*/false);

  nav_msgs::msg::Path received_validated;
  bool validated_received = false;
  auto validated_sub = helper_node_->create_subscription<nav_msgs::msg::Path>(
      "validated_path", 10, [&](const nav_msgs::msg::Path::SharedPtr msg) {
        received_validated = *msg;
        validated_received = true;
      });

  nav_msgs::msg::Path path = makeTrivialPath();
  path_pub_->publish(path);

  spinBriefly(executor_);

  EXPECT_FALSE(halt_received_ && halt_value_)
      << "Safety must NOT halt when service is reachable and path is clear";
  EXPECT_GE(stub.callCount(), 1) << "ValidatePath server must be called";
  EXPECT_TRUE(validated_received) << "Validated path must be republished";
}

TEST_F(SafetyPathHarness, AvailableServiceWithUnsafeResultStillHalts) {
  // When the server is reachable but replies "blocked", current behaviour is
  // to publish a rejection string but NOT halt. After the fix, an unsafe
  // path verdict should also escalate to haltSystem.
  ValidatePathStub stub(helper_node_, /*reply_blocked=*/true);

  nav_msgs::msg::Path path = makeTrivialPath();
  path_pub_->publish(path);

  spinBriefly(executor_);

  EXPECT_GE(stub.callCount(), 1) << "ValidatePath server must be called";
  EXPECT_TRUE(halt_received_ && halt_value_)
      << "Blocked verdict must trigger haltSystem after fix";
  EXPECT_TRUE(responseContains("blocked") ||
              responseContains(kRejectionSubstring))
      << "Blocked verdict must publish a rejection";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
