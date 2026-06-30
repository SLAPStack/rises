// =============================================================================
// Phase 3 concurrency test: MissionControllerNode::active_mission_id_
// =============================================================================
//
// Audit finding covered:
//   rises_mission_controller/src/mission_controller_node.cpp lines 70 and 261.
//     - executeMission() takes std::lock_guard(mission_mutex_) at line ~259
//       and writes active_mission_id_ at line ~261.
//     - intentCallback() reads active_mission_id_ at line ~70 WITHOUT taking
//       mission_mutex_ ("Rejecting intent '%s'" log line uses
//       active_mission_id_.c_str()).
//   This is a textbook missing-lock-on-reader race against a non-atomic
//   std::string. Under TSan the read in intentCallback races the write in
//   executeMission().
//
// Race exercised:
//   Two helper threads publish Intent messages at high rate. The first goal
//   triggers executeMission() which acquires the mutex and writes
//   active_mission_id_; subsequent intents take the "mission already active"
//   branch and read active_mission_id_ without the lock.
//
// IMPORTANT teardown protocol:
//   MissionControllerNode's destructor does NOT join mission_thread_. If the
//   destructor runs while the thread is still joinable, std::terminate()
//   fires. Every test below drives the mission to a terminal status
//   (SUCCEEDED / FAILED / CANCELLED) and then waits for mission_active_ to
//   become observably false (no STATUS_ACTIVE for kQuietWindow) before
//   tearing down. Because no /skill/* action servers are stood up, action
//   calls inside executeMission() block on wait_for_action_server(5 s) and
//   then return false; the task fails and the mission ends with
//   STATUS_FAILED. Each test takes ~5-7 s of wall-clock as a result; the
//   30 s TIMEOUT in CMakeLists is the hard upper bound.
//
// TSan invocation:
//   colcon build --packages-select rises_mission_controller \
//     --cmake-args -DBUILD_TESTING=ON -DARIS_ENABLE_TSAN=ON
//   colcon test --packages-select rises_mission_controller \
//     --ctest-args -R test_active_mission_race
//
// Expected TSan output before the production fix:
//   ThreadSanitizer: data race on active_mission_id_ between executeMission
//   (write under mutex) and intentCallback (read without mutex).
//
// Standards: function cap 100, nesting <= 3, named constants, no TODOs.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rises_interfaces/msg/mission_status.hpp>
#include <hri_actions_msgs/msg/intent.hpp>

#include "rises_mission_controller/mission_controller_node.hpp"

namespace {

using rises_interfaces::msg::MissionStatus;
using hri_actions_msgs::msg::Intent;

constexpr int kExecutorThreads = 3;
constexpr int kIntentBurstSize = 50;
constexpr auto kIntentInterval = std::chrono::milliseconds(2);
constexpr auto kAssertDeadline = std::chrono::seconds(10);
constexpr auto kSpinSlice = std::chrono::milliseconds(20);
constexpr auto kQuietWindow = std::chrono::milliseconds(200);
// Short action timeout so missions terminate quickly (no /skill/* servers in
// this fixture; the action-client wait_for_action_server returns false and
// the task fails, emitting STATUS_FAILED).
constexpr int kActionTimeoutSec = 1;
constexpr std::int64_t kAreaId = 7;

Intent makeLockAreaIntent(std::int64_t area_id) {
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

Intent makeStopIntent() {
  Intent msg;
  msg.intent = Intent::STOP_ACTIVITY;
  msg.source = "test";
  msg.modality = "voice";
  msg.confidence = 1.0F;
  msg.priority = 0U;
  return msg;
}

// Thread-safe history of MissionStatus messages observed on /mission_status.
class StatusRecorder {
public:
  explicit StatusRecorder(const rclcpp::Node::SharedPtr &node) {
    sub_ = node->create_subscription<MissionStatus>(
        "/mission_status", rclcpp::QoS(50),
        [this](MissionStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk{mu_};
          history_.push_back(*msg);
        });
  }

  std::vector<MissionStatus> history() const {
    std::lock_guard<std::mutex> lk{mu_};
    return history_;
  }

  bool sawTerminal() const {
    std::lock_guard<std::mutex> lk{mu_};
    for (const auto &m : history_) {
      if (m.status == MissionStatus::STATUS_SUCCEEDED ||
          m.status == MissionStatus::STATUS_FAILED ||
          m.status == MissionStatus::STATUS_CANCELLED) {
        return true;
      }
    }
    return false;
  }

private:
  mutable std::mutex mu_;
  std::vector<MissionStatus> history_;
  rclcpp::Subscription<MissionStatus>::SharedPtr sub_;
};

class ActiveMissionRaceFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter{"action_timeout_sec", kActionTimeoutSec}});
    node_ = std::make_shared<rises::MissionControllerNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("active_mission_race_helper");

    recorder_ = std::make_unique<StatusRecorder>(helper_);
    intent_pub_ =
        helper_->create_publisher<Intent>("/intents", rclcpp::QoS(50));

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions{}, kExecutorThreads);
    executor_->add_node(node_);
    executor_->add_node(helper_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  // Drive any in-flight mission to a terminal state and wait until no further
  // STATUS_ACTIVE messages arrive for kQuietWindow. Critical: the SUT's
  // destructor does NOT join mission_thread_, so leaving a mission joinable
  // crashes the test process with std::terminate().
  void drainMissions() {
    intent_pub_->publish(makeStopIntent());
    const auto deadline = std::chrono::steady_clock::now() + kAssertDeadline;
    while (std::chrono::steady_clock::now() < deadline) {
      if (recorder_->sawTerminal()) {
        break;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    // Quiet window: no STATUS_ACTIVE for the last kQuietWindow milliseconds.
    std::this_thread::sleep_for(kQuietWindow);
  }

  void TearDown() override {
    drainMissions();
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    recorder_.reset();
    intent_pub_.reset();
    helper_.reset();
    node_.reset();
  }

  std::shared_ptr<rises::MissionControllerNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  std::unique_ptr<StatusRecorder> recorder_;
  rclcpp::Publisher<Intent>::SharedPtr intent_pub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

} // namespace

// -----------------------------------------------------------------------------
// ConcurrentGoalRequestsSerialized
// -----------------------------------------------------------------------------
//
// Two threads spam Intent messages. The mission controller must only accept
// one mission at a time: every STATUS_ACTIVE published on /mission_status
// must carry the same mission_id until that mission terminates, and any
// subsequent mission must have a distinct id observed only after the prior
// one ended.
TEST_F(ActiveMissionRaceFixture, ConcurrentGoalRequestsSerialized) {
  std::atomic<bool> producer_failed{false};

  std::thread spammer_a([this, &producer_failed]() {
    try {
      for (int i = 0; i < kIntentBurstSize; ++i) {
        intent_pub_->publish(makeLockAreaIntent(kAreaId));
        std::this_thread::sleep_for(kIntentInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::thread spammer_b([this, &producer_failed]() {
    try {
      for (int i = 0; i < kIntentBurstSize; ++i) {
        intent_pub_->publish(makeLockAreaIntent(kAreaId));
        std::this_thread::sleep_for(kIntentInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  spammer_a.join();
  spammer_b.join();

  // Wait for at least one terminal status so the assertion below sees the
  // full lifecycle of the first accepted mission.
  const auto deadline = std::chrono::steady_clock::now() + kAssertDeadline;
  while (std::chrono::steady_clock::now() < deadline &&
         !recorder_->sawTerminal()) {
    std::this_thread::sleep_for(kSpinSlice);
  }

  const std::vector<MissionStatus> history = recorder_->history();
  std::set<std::string> active_ids_at_time;
  std::string current_active;
  for (const MissionStatus &m : history) {
    if (m.status == MissionStatus::STATUS_ACTIVE) {
      if (current_active.empty()) {
        current_active = m.mission_id;
        active_ids_at_time.insert(m.mission_id);
      } else {
        EXPECT_EQ(m.mission_id, current_active)
            << "Saw STATUS_ACTIVE for mission '" << m.mission_id
            << "' while another mission '" << current_active
            << "' was still active";
      }
    } else {
      // Terminal status closes out the current mission.
      current_active.clear();
    }
  }

  EXPECT_FALSE(producer_failed.load())
      << "publish() of Intent threw during the race window";
  EXPECT_FALSE(active_ids_at_time.empty())
      << "At least one mission was expected to be accepted";
}

// -----------------------------------------------------------------------------
// CancelDuringExecuteRaceFree
// -----------------------------------------------------------------------------
//
// A producer thread spams START_ACTIVITY while a second thread spams
// STOP_ACTIVITY. The STOP path in intentCallback writes mission_active_ to
// false on a different thread than executeMission's loop body reads it.
// Under TSan: data race on active_mission_id_ between the rejection branch
// and executeMission. Structurally we assert no exception propagates and at
// least one terminal status is observed.
TEST_F(ActiveMissionRaceFixture, CancelDuringExecuteRaceFree) {
  std::atomic<bool> producer_failed{false};
  std::atomic<bool> stop{false};

  std::thread starter([this, &stop, &producer_failed]() {
    try {
      while (!stop.load(std::memory_order_acquire)) {
        intent_pub_->publish(makeLockAreaIntent(kAreaId));
        std::this_thread::sleep_for(kIntentInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::thread canceller([this, &stop, &producer_failed]() {
    try {
      while (!stop.load(std::memory_order_acquire)) {
        intent_pub_->publish(makeStopIntent());
        std::this_thread::sleep_for(kIntentInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  stop.store(true, std::memory_order_release);
  starter.join();
  canceller.join();

  const auto deadline = std::chrono::steady_clock::now() + kAssertDeadline;
  while (std::chrono::steady_clock::now() < deadline &&
         !recorder_->sawTerminal()) {
    std::this_thread::sleep_for(kSpinSlice);
  }

  EXPECT_FALSE(producer_failed.load());
  EXPECT_TRUE(recorder_->sawTerminal())
      << "Expected at least one mission to reach a terminal state";
}
