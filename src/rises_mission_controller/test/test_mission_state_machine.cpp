// Package under test: rises_mission_controller
//
// The mission controller subscribes to /intents
// (hri_actions_msgs::msg::Intent), decomposes each intent into a list of skill
// action calls, and reports progress on /mission_status
// (rises_interfaces::msg::MissionStatus).
//
// Test strategy
// -------------
// 1. Spin up a real MissionControllerNode in a MultiThreadedExecutor on a
//    background thread (the node spawns its own mission thread + uses a
//    separate action callback group, see main.cpp).
// 2. Stand up fake rclcpp_action::Server instances on the /skill/* topics
//    the node calls, with per-test behaviour (succeed / reject / hang / fail).
// 3. Subscribe to /mission_status from a sidecar node so the test can observe
//    every state transition (STATUS_ACTIVE / STATUS_SUCCEEDED / STATUS_FAILED /
//    STATUS_CANCELLED) without touching production internals.
// 4. Publish Intent messages on /intents to drive the state machine.
//
// Why no direct member assertions?
// --------------------------------
// MissionControllerNode exposes no public getters for mission_active_,
// active_mission_id_, or current task index. Observable state is only the
// /mission_status topic, which is sufficient for state-machine assertions
// because the production code publishes STATUS_* at every transition.
//
// Production seams that would simplify tests (Phase 2 follow-up; do NOT add
// them as part of writing these tests):
//   - `bool missionActive() const noexcept;`
//   - `std::string activeMissionId() const;` (must take mission_mutex_)
//   - `void setMissionIdFactory(std::function<std::string()>);` to inject
//     deterministic mission ids for assertions.
//
// Determinism
// -----------
// No std::this_thread::sleep_for over 100ms. Long waits use a polling loop
// over rclcpp::spin_some with a hard deadline (kAssertTimeout). The fake
// skill servers complete or reject synchronously inside their accepted_cb,
// so the mission thread finishes within a few milliseconds in the happy
// path.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hri_actions_msgs/msg/intent.hpp>

#include <rises_interfaces/action/get_area_state.hpp>
#include <rises_interfaces/action/get_map_info.hpp>
#include <rises_interfaces/action/get_safety_radius.hpp>
#include <rises_interfaces/action/set_area_state.hpp>
#include <rises_interfaces/msg/mission_status.hpp>

#include "rises_mission_controller/mission_controller_node.hpp"

namespace {

using rises_interfaces::msg::MissionStatus;
using hri_actions_msgs::msg::Intent;

// --------------------------------------------------------------------------
// Tuning constants. Kept small so the suite is fast on CI; the production
// code is single-process so transitions land within a few ms in practice.
// --------------------------------------------------------------------------
constexpr std::chrono::milliseconds kAssertTimeout{2000};
constexpr std::chrono::milliseconds kSpinSlice{5};
constexpr std::chrono::milliseconds kQuietWindow{200};
constexpr int kActionTimeoutSec = 2;
constexpr int64_t kAreaId = 42;

// --------------------------------------------------------------------------
// Configurable fake skill server. Captures every received goal so the test
// can assert "no goal sent" in rejection cases, and lets the test pick the
// outcome per server (succeed / reject_goal / return_failure).
// --------------------------------------------------------------------------
enum class Outcome { kSucceed, kRejectGoal, kReturnFailure };

template <typename ActionT> class FakeSkillServer {
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalSP = std::shared_ptr<GoalHandle>;

  FakeSkillServer(rclcpp::Node::SharedPtr node, const std::string &name)
      : node_{std::move(node)}, name_{name}, outcome_{Outcome::kSucceed},
        hold_{false} {
    server_ = rclcpp_action::create_server<ActionT>(
        node_, name_,
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const typename ActionT::Goal>) {
          ++received_goals_;
          if (outcome_.load() == Outcome::kRejectGoal) {
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const GoalSP &) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const GoalSP &gh) {
          // Optionally hold until the test releases us. Used by the cancel
          // test to keep task 1 in flight while STOP_ACTIVITY arrives.
          if (hold_.load()) {
            std::unique_lock<std::mutex> lk{release_mu_};
            release_cv_.wait(lk, [this]() { return !hold_.load(); });
          }
          auto result = std::make_shared<typename ActionT::Result>();
          this->fillResult(*result);
          if (outcome_.load() == Outcome::kReturnFailure) {
            gh->abort(result);
          } else {
            gh->succeed(result);
          }
        });
  }

  void setOutcome(Outcome o) noexcept { outcome_.store(o); }
  void setHold(bool h) {
    hold_.store(h);
    if (!h) {
      std::lock_guard<std::mutex> lk{release_mu_};
      release_cv_.notify_all();
    }
  }
  int receivedGoals() const noexcept { return received_goals_.load(); }

private:
  // Populates a "success" result. For get_area_state we set found=true and
  // locked=false so the buildAreaControlMission first task passes.
  void fillResult(typename ActionT::Result &r) const {
    if constexpr (std::is_same_v<ActionT,
                                 rises_interfaces::action::GetAreaState>) {
      r.found = true;
      r.locked = false;
      r.message = "ok";
    } else if constexpr (std::is_same_v<
                             ActionT, rises_interfaces::action::SetAreaState>) {
      r.success = (outcome_.load() != Outcome::kReturnFailure);
      r.message = "ok";
    } else if constexpr (std::is_same_v<ActionT,
                                        rises_interfaces::action::GetMapInfo>) {
      r.obstacle_count = 0;
      r.contours_loaded = true;
      r.contour_segment_count = 0;
      r.inner_polygon_count = 0;
    } else if constexpr (std::is_same_v<
                             ActionT,
                             rises_interfaces::action::GetSafetyRadius>) {
      r.radius = 1.0f;
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::atomic<Outcome> outcome_;
  std::atomic<int> received_goals_{0};
  std::atomic<bool> hold_;
  std::mutex release_mu_;
  std::condition_variable release_cv_;
};

// --------------------------------------------------------------------------
// Status recorder: subscribes to /mission_status and stores every message
// in arrival order. Thread-safe for read from the test thread.
// --------------------------------------------------------------------------
class StatusRecorder {
public:
  explicit StatusRecorder(rclcpp::Node::SharedPtr node) {
    sub_ = node->create_subscription<MissionStatus>(
        "/mission_status", rclcpp::QoS(10),
        [this](MissionStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk{mu_};
          history_.push_back(*msg);
        });
  }

  std::vector<MissionStatus> history() const {
    std::lock_guard<std::mutex> lk{mu_};
    return history_;
  }

  std::optional<MissionStatus> firstWithStatus(uint8_t s) const {
    std::lock_guard<std::mutex> lk{mu_};
    for (const auto &m : history_) {
      if (m.status == s) {
        return m;
      }
    }
    return std::nullopt;
  }

  bool sawStatus(uint8_t s) const { return firstWithStatus(s).has_value(); }

private:
  mutable std::mutex mu_;
  std::vector<MissionStatus> history_;
  rclcpp::Subscription<MissionStatus>::SharedPtr sub_;
};

// --------------------------------------------------------------------------
// Test fixture. Owns the system-under-test, a sidecar node for fakes/observer,
// and a MultiThreadedExecutor on a background thread.
// --------------------------------------------------------------------------
class MissionStateMachineTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(
        {rclcpp::Parameter{"action_timeout_sec", kActionTimeoutSec}});
    sut_ = std::make_shared<rises::MissionControllerNode>(opts);

    helper_ = std::make_shared<rclcpp::Node>("mission_controller_test_helper");

    recorder_ = std::make_unique<StatusRecorder>(helper_);
    intent_pub_ =
        helper_->create_publisher<Intent>("/intents", rclcpp::QoS(10));

    get_area_state_ = std::make_unique<
        FakeSkillServer<rises_interfaces::action::GetAreaState>>(
        helper_, "skill/get_area_state");
    set_area_state_ = std::make_unique<
        FakeSkillServer<rises_interfaces::action::SetAreaState>>(
        helper_, "skill/set_area_state");
    get_map_info_ =
        std::make_unique<FakeSkillServer<rises_interfaces::action::GetMapInfo>>(
            helper_, "skill/get_map_info");
    get_safety_radius_ = std::make_unique<
        FakeSkillServer<rises_interfaces::action::GetSafetyRadius>>(
        helper_, "skill/get_safety_radius");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(sut_);
    executor_->add_node(helper_);

    spin_thread_ = std::thread{[this]() { executor_->spin(); }};
  }

  void TearDown() override {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    get_area_state_.reset();
    set_area_state_.reset();
    get_map_info_.reset();
    get_safety_radius_.reset();
    recorder_.reset();
    intent_pub_.reset();
    helper_.reset();
    sut_.reset();
  }

  static Intent makeLockAreaIntent(int64_t area_id) {
    Intent msg;
    msg.intent = Intent::START_ACTIVITY;
    msg.data = std::string{"{\"goal\":\"lock_area\",\"area_id\":"} +
               std::to_string(area_id) + "}";
    msg.source = "test";
    msg.modality = "voice";
    msg.confidence = 1.0F;
    msg.priority = 0U;
    return msg;
  }

  static Intent makeStopIntent() {
    Intent msg;
    msg.intent = Intent::STOP_ACTIVITY;
    msg.source = "test";
    msg.modality = "voice";
    msg.confidence = 1.0F;
    msg.priority = 0U;
    return msg;
  }

  static Intent makeUpdateMapIntent() {
    Intent msg;
    msg.intent = Intent::START_ACTIVITY;
    msg.data = "{\"goal\":\"update_map\"}";
    msg.source = "test";
    msg.modality = "voice";
    msg.confidence = 1.0F;
    msg.priority = 0U;
    return msg;
  }

  // Polls a predicate up to kAssertTimeout. Spins are driven by the
  // executor on the background thread, so this is purely a wait.
  template <typename Pred> bool waitFor(Pred &&p) const {
    const auto deadline = std::chrono::steady_clock::now() + kAssertTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (p()) {
        return true;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    return p();
  }

  std::shared_ptr<rises::MissionControllerNode> sut_;
  std::shared_ptr<rclcpp::Node> helper_;
  std::unique_ptr<StatusRecorder> recorder_;
  rclcpp::Publisher<Intent>::SharedPtr intent_pub_;

  std::unique_ptr<FakeSkillServer<rises_interfaces::action::GetAreaState>>
      get_area_state_;
  std::unique_ptr<FakeSkillServer<rises_interfaces::action::SetAreaState>>
      set_area_state_;
  std::unique_ptr<FakeSkillServer<rises_interfaces::action::GetMapInfo>>
      get_map_info_;
  std::unique_ptr<FakeSkillServer<rises_interfaces::action::GetSafetyRadius>>
      get_safety_radius_;

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

} // namespace

TEST_F(MissionStateMachineTest, IdleToActiveOnGoal) {
  intent_pub_->publish(makeLockAreaIntent(kAreaId));

  const bool saw_active = waitFor(
      [&]() { return recorder_->sawStatus(MissionStatus::STATUS_ACTIVE); });
  EXPECT_TRUE(saw_active)
      << "Expected at least one STATUS_ACTIVE on /mission_status after a "
         "START_ACTIVITY(lock_area) intent";

  const auto first_active =
      recorder_->firstWithStatus(MissionStatus::STATUS_ACTIVE);
  ASSERT_TRUE(first_active.has_value());
  EXPECT_EQ(first_active->mission_type, "AREA_ACCESS_CONTROL");
  EXPECT_FALSE(first_active->mission_id.empty());
}

TEST_F(MissionStateMachineTest, ActiveToSucceededOnComplete) {
  intent_pub_->publish(makeLockAreaIntent(kAreaId));

  const bool succeeded = waitFor(
      [&]() { return recorder_->sawStatus(MissionStatus::STATUS_SUCCEEDED); });
  ASSERT_TRUE(succeeded);

  EXPECT_TRUE(recorder_->sawStatus(MissionStatus::STATUS_ACTIVE));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_FAILED));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_CANCELLED));
}

TEST_F(MissionStateMachineTest, ActiveToFailedOnReject) {
  // First task in lock_area is get_area_state -- have that server reject.
  get_area_state_->setOutcome(Outcome::kRejectGoal);

  intent_pub_->publish(makeLockAreaIntent(kAreaId));

  const bool failed = waitFor(
      [&]() { return recorder_->sawStatus(MissionStatus::STATUS_FAILED); });
  ASSERT_TRUE(failed);

  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_SUCCEEDED));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_CANCELLED));
}

TEST_F(MissionStateMachineTest, CancelDuringActiveTransitionsToCancelled) {
  // Use the update_map mission (two tasks: get_map_info -> get_safety_radius).
  // Hold the first task's server so the worker is parked inside task 1's
  // execute() with the goal accepted but not yet finalised. STOP_ACTIVITY
  // arrives, flipping mission_active_ to false. Release task 1 -- it
  // succeeds and returns; the loop's top-of-iteration check for task 2
  // sees mission_active_==false and emits STATUS_CANCELLED.
  get_map_info_->setHold(true);

  intent_pub_->publish(makeUpdateMapIntent());

  const bool active = waitFor([&]() {
    return get_map_info_->receivedGoals() > 0 &&
           recorder_->sawStatus(MissionStatus::STATUS_ACTIVE);
  });
  ASSERT_TRUE(active);

  intent_pub_->publish(makeStopIntent());

  // Give the intent callback time to process STOP_ACTIVITY before releasing.
  std::this_thread::sleep_for(kSpinSlice * 4);
  get_map_info_->setHold(false);

  const bool cancelled = waitFor(
      [&]() { return recorder_->sawStatus(MissionStatus::STATUS_CANCELLED); });
  EXPECT_TRUE(cancelled)
      << "Expected STATUS_CANCELLED after STOP_ACTIVITY mid-mission";
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_SUCCEEDED));
}

TEST_F(MissionStateMachineTest, ConcurrentGoalsRejectedWhileActive) {
  // Hold the first task's server so the mission stays active. While it is
  // parked, send a second goal -- the intent callback must see
  // mission_active_==true and refuse to spawn a second mission thread.
  get_map_info_->setHold(true);

  intent_pub_->publish(makeUpdateMapIntent());

  // Wait for the first mission to publish STATUS_ACTIVE (start banner).
  const bool first_active = waitFor([&]() {
    return get_map_info_->receivedGoals() > 0 &&
           recorder_->sawStatus(MissionStatus::STATUS_ACTIVE);
  });
  ASSERT_TRUE(first_active);

  const auto first = recorder_->firstWithStatus(MissionStatus::STATUS_ACTIVE);
  ASSERT_TRUE(first.has_value());
  const std::string first_id = first->mission_id;

  // Publish a second goal while the first is still executing.
  intent_pub_->publish(makeUpdateMapIntent());

  // Give the system a moment; if a second mission were accepted we'd see
  // additional STATUS_ACTIVE messages with a different mission_id.
  std::this_thread::sleep_for(kQuietWindow);

  const auto history = recorder_->history();
  for (const auto &m : history) {
    if (m.status == MissionStatus::STATUS_ACTIVE) {
      EXPECT_EQ(m.mission_id, first_id)
          << "Second mission was accepted while first was active";
    }
  }

  // Drain: release the held server so the mission completes before teardown.
  // The SUT has no destructor that joins mission_thread_; leaving it
  // joinable at destruction would std::terminate().
  get_map_info_->setHold(false);
  const bool done = waitFor([&]() {
    return recorder_->sawStatus(MissionStatus::STATUS_SUCCEEDED) ||
           recorder_->sawStatus(MissionStatus::STATUS_FAILED) ||
           recorder_->sawStatus(MissionStatus::STATUS_CANCELLED);
  });
  EXPECT_TRUE(done);
}

TEST_F(MissionStateMachineTest, EmptyIntentRejected) {
  Intent empty; // All fields default-constructed (empty strings, zeros).
  intent_pub_->publish(empty);

  // The node should ignore an unrecognised intent. No status should be
  // published. We wait a short quiet window to confirm absence.
  std::this_thread::sleep_for(kQuietWindow);

  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_ACTIVE));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_SUCCEEDED));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_FAILED));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_CANCELLED));
}

TEST_F(MissionStateMachineTest, MalformedIntentRejected) {
  // START_ACTIVITY with garbage payload -- not lock_area / unlock_area /
  // update_map. The controller hits the "not handled by geofence
  // controller" branch and returns without publishing status.
  Intent msg;
  msg.intent = Intent::START_ACTIVITY;
  msg.data = "{not-json: definitely-not-a-known-activity";
  msg.source = "test";
  msg.modality = "voice";
  msg.confidence = 1.0F;
  msg.priority = 0U;

  intent_pub_->publish(msg);

  std::this_thread::sleep_for(kQuietWindow);

  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_ACTIVE));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_SUCCEEDED));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_FAILED));
  EXPECT_FALSE(recorder_->sawStatus(MissionStatus::STATUS_CANCELLED));
}
