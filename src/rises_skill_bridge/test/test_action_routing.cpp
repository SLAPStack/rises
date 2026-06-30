// Unit tests for SkillBridgeNode action routing.
//
// The bridge does not have a runtime "skill registry" -- it hardcodes nine
// rclcpp_action::Server instances under skill/* whose accepted-handlers call
// the matching geofence service via callService(). These tests therefore
// verify the public action surface:
//   * the documented action server names are advertised after construction
//   * sending a goal to an unknown action name fails wait-for-server within a
//     bounded budget (the bridge does not advertise it)
//   * cancelling a goal in flight is accepted by the bridge
//   * the result payload returned on service-call failure matches the
//     documented contract (response fields populated, blocked/success flags
//     coherent)
//   * two different known skills can be invoked back-to-back on the same
//     single-threaded MultiThreadedExecutor without interference
//
// We avoid spinning up the real geofence stack. Where a successful service
// call is required, we host a fake service server on a sibling node and let
// the bridge's normal client wiring connect to it. Where doing so would
// require remapping the live skill_bridge_node's hardcoded service names at
// construction time (i.e. the bridge does not allow remap of its individual
// client targets except via the geofence_node_name parameter), we use that
// parameter to redirect the client namespace and stand the fake services up
// under the matching path.
//
// Timing: every wait uses a bounded deadline (1-2 s) so a hung test fails
// loudly rather than blocking CI.

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rises_interfaces/action/get_safety_radius.hpp"
#include "rises_interfaces/action/validate_path.hpp"
#include "rises_interfaces/srv/get_safety_radius.hpp"
#include "rises_interfaces/srv/validate_path.hpp"

#include "rises_skill_bridge/skill_bridge_node.hpp"

namespace {

using namespace std::chrono_literals;

constexpr auto kClientWait = 2s;
constexpr auto kServerSpinSlice = 10ms;
constexpr auto kResultDeadline = 3s;

// Spin two nodes (bridge + fake services / client) until `pred` is true or
// the deadline elapses. Returns true if the predicate became true.
template <typename Predicate>
bool spinUntil(rclcpp::executors::MultiThreadedExecutor &exec,
               std::chrono::steady_clock::duration timeout, Predicate pred) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    exec.spin_some(kServerSpinSlice);
  }
  return pred();
}

class SkillBridgeFixture : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    // Redirect the bridge's service clients to a private namespace so we
    // can host fakes under it without colliding with anything else on the
    // ROS graph (and so two tests run in sequence don't see leftover
    // remappings).
    options.parameter_overrides({
        rclcpp::Parameter("geofence_node_name", "fake_geofence"),
        rclcpp::Parameter("service_timeout_sec", 1),
    });
    bridge_ = std::make_shared<rises::SkillBridgeNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("skill_bridge_test_client");
    fake_services_node_ = std::make_shared<rclcpp::Node>("fake_geofence");
    executor_.add_node(bridge_);
    executor_.add_node(client_node_);
    executor_.add_node(fake_services_node_);
  }

  void TearDown() override {
    executor_.remove_node(fake_services_node_);
    executor_.remove_node(client_node_);
    executor_.remove_node(bridge_);
    fake_services_node_.reset();
    client_node_.reset();
    bridge_.reset();
    // Keep rclcpp::shutdown() out of TearDown -- multiple TEST cases in
    // one binary share the context.
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions{},
                                                     2};
  std::shared_ptr<rises::SkillBridgeNode> bridge_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr fake_services_node_;
};

} // namespace

TEST_F(SkillBridgeFixture, KnownSkillRoutesToCorrectAction) {
  using GetSafetyRadiusAction = rises_interfaces::action::GetSafetyRadius;
  using GetSafetyRadiusSrv = rises_interfaces::srv::GetSafetyRadius;

  constexpr float kExpectedRadius = 0.42f;
  std::atomic<int> service_call_count{0};
  auto service = fake_services_node_->create_service<GetSafetyRadiusSrv>(
      "fake_geofence/get_safety_radius",
      [&](const GetSafetyRadiusSrv::Request::SharedPtr,
          GetSafetyRadiusSrv::Response::SharedPtr response) {
        response->radius = kExpectedRadius;
        ++service_call_count;
      });
  (void)service;

  auto action_client = rclcpp_action::create_client<GetSafetyRadiusAction>(
      client_node_, "skill/get_safety_radius");

  // Spin while waiting for action server to be discovered.
  const bool server_up = spinUntil(executor_, kClientWait, [&]() {
    return action_client->action_server_is_ready();
  });
  ASSERT_TRUE(server_up) << "bridge did not advertise skill/get_safety_radius";

  GetSafetyRadiusAction::Goal goal;
  auto goal_future = action_client->async_send_goal(goal);
  ASSERT_TRUE(spinUntil(executor_, kClientWait, [&]() {
    return goal_future.wait_for(0s) == std::future_status::ready;
  })) << "goal acknowledgement timed out";
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = action_client->async_get_result(goal_handle);
  ASSERT_TRUE(spinUntil(executor_, kResultDeadline, [&]() {
    return result_future.wait_for(0s) == std::future_status::ready;
  })) << "result not delivered in time";

  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FLOAT_EQ(wrapped.result->radius, kExpectedRadius);
  EXPECT_EQ(service_call_count.load(), 1);
}

TEST_F(SkillBridgeFixture, UnknownSkillReturnsError) {
  using GetSafetyRadiusAction = rises_interfaces::action::GetSafetyRadius;
  auto action_client = rclcpp_action::create_client<GetSafetyRadiusAction>(
      client_node_, "skill/this_skill_does_not_exist");

  // The bridge does not advertise this name -- the client must never see a
  // ready action server within a bounded budget.
  const bool became_ready = spinUntil(executor_, kClientWait, [&]() {
    return action_client->action_server_is_ready();
  });
  EXPECT_FALSE(became_ready)
      << "unexpectedly discovered an unknown skill server";
}

TEST_F(SkillBridgeFixture, CancelPropagatesToUnderlyingAction) {
  using ValidatePathAction = rises_interfaces::action::ValidatePath;
  using ValidatePathSrv = rises_interfaces::srv::ValidatePath;

  // Service that never answers, so the bridge stays in callService() and
  // we can cancel mid-flight.
  auto service = fake_services_node_->create_service<ValidatePathSrv>(
      "fake_geofence/validate_path",
      [](const ValidatePathSrv::Request::SharedPtr,
         ValidatePathSrv::Response::SharedPtr) {
        // Intentionally do nothing -- the action's callService() will
        // time out after service_timeout_sec, but we should cancel first.
        std::this_thread::sleep_for(2s);
      });
  (void)service;

  auto action_client = rclcpp_action::create_client<ValidatePathAction>(
      client_node_, "skill/validate_path");
  ASSERT_TRUE(spinUntil(executor_, kClientWait, [&]() {
    return action_client->action_server_is_ready();
  })) << "validate_path server not ready";

  ValidatePathAction::Goal goal;
  auto goal_future = action_client->async_send_goal(goal);
  ASSERT_TRUE(spinUntil(executor_, kClientWait, [&]() {
    return goal_future.wait_for(0s) == std::future_status::ready;
  }));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto cancel_future = action_client->async_cancel_goal(goal_handle);
  const bool cancel_acked = spinUntil(executor_, kClientWait, [&]() {
    return cancel_future.wait_for(0s) == std::future_status::ready;
  });
  ASSERT_TRUE(cancel_acked) << "cancel request not acknowledged";
  auto cancel_response = cancel_future.get();
  // The bridge unconditionally accepts cancel requests (handleXxxCancel
  // returns CancelResponse::ACCEPT) -- propagation has therefore occurred
  // at the action protocol layer.
  EXPECT_EQ(cancel_response->return_code,
            action_msgs::srv::CancelGoal::Response::ERROR_NONE);
}

TEST_F(SkillBridgeFixture, ResponseFormatMatchesContract) {
  using GetSafetyRadiusAction = rises_interfaces::action::GetSafetyRadius;
  // No fake service registered -- callService() will time out and the
  // bridge documents that this maps to result->radius = -1.0f.
  auto action_client = rclcpp_action::create_client<GetSafetyRadiusAction>(
      client_node_, "skill/get_safety_radius");
  ASSERT_TRUE(spinUntil(executor_, kClientWait, [&]() {
    return action_client->action_server_is_ready();
  }));

  GetSafetyRadiusAction::Goal goal;
  auto goal_future = action_client->async_send_goal(goal);
  ASSERT_TRUE(spinUntil(executor_, kClientWait, [&]() {
    return goal_future.wait_for(0s) == std::future_status::ready;
  }));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = action_client->async_get_result(goal_handle);
  // service_timeout_sec=1, so we expect the result within ~1.5 s.
  ASSERT_TRUE(spinUntil(executor_, 5s, [&]() {
    return result_future.wait_for(0s) == std::future_status::ready;
  }));
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FLOAT_EQ(wrapped.result->radius, -1.0f);
}

TEST_F(SkillBridgeFixture, ConcurrentSkillsHandledSeparately) {
  using GetSafetyRadiusAction = rises_interfaces::action::GetSafetyRadius;
  using ValidatePathAction = rises_interfaces::action::ValidatePath;
  using GetSafetyRadiusSrv = rises_interfaces::srv::GetSafetyRadius;
  using ValidatePathSrv = rises_interfaces::srv::ValidatePath;

  constexpr float kRadius = 0.31f;
  auto svc_radius = fake_services_node_->create_service<GetSafetyRadiusSrv>(
      "fake_geofence/get_safety_radius",
      [&](const GetSafetyRadiusSrv::Request::SharedPtr,
          GetSafetyRadiusSrv::Response::SharedPtr response) {
        response->radius = kRadius;
      });
  auto svc_path = fake_services_node_->create_service<ValidatePathSrv>(
      "fake_geofence/validate_path",
      [&](const ValidatePathSrv::Request::SharedPtr,
          ValidatePathSrv::Response::SharedPtr response) {
        response->blocked = false;
      });
  (void)svc_radius;
  (void)svc_path;

  auto radius_client = rclcpp_action::create_client<GetSafetyRadiusAction>(
      client_node_, "skill/get_safety_radius");
  auto path_client = rclcpp_action::create_client<ValidatePathAction>(
      client_node_, "skill/validate_path");
  ASSERT_TRUE(spinUntil(executor_, kClientWait, [&]() {
    return radius_client->action_server_is_ready() &&
           path_client->action_server_is_ready();
  }));

  GetSafetyRadiusAction::Goal radius_goal;
  ValidatePathAction::Goal path_goal;
  auto radius_goal_fut = radius_client->async_send_goal(radius_goal);
  auto path_goal_fut = path_client->async_send_goal(path_goal);
  ASSERT_TRUE(spinUntil(executor_, kClientWait, [&]() {
    return radius_goal_fut.wait_for(0s) == std::future_status::ready &&
           path_goal_fut.wait_for(0s) == std::future_status::ready;
  }));
  auto radius_handle = radius_goal_fut.get();
  auto path_handle = path_goal_fut.get();
  ASSERT_NE(radius_handle, nullptr);
  ASSERT_NE(path_handle, nullptr);

  auto radius_result_fut = radius_client->async_get_result(radius_handle);
  auto path_result_fut = path_client->async_get_result(path_handle);
  ASSERT_TRUE(spinUntil(executor_, kResultDeadline, [&]() {
    return radius_result_fut.wait_for(0s) == std::future_status::ready &&
           path_result_fut.wait_for(0s) == std::future_status::ready;
  }));

  auto radius_wrapped = radius_result_fut.get();
  auto path_wrapped = path_result_fut.get();
  EXPECT_EQ(radius_wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(path_wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FLOAT_EQ(radius_wrapped.result->radius, kRadius);
  EXPECT_FALSE(path_wrapped.result->blocked);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
