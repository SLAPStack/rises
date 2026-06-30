// Package under test: rises_mission_controller
//
// Focus: rises::MissionControllerNode::active_mission_id_ sequencing.
//
// active_mission_id_ is a std::string member written under mission_mutex_ in
// executeMission() and read without the lock in intentCallback() (line ~70
// of mission_controller_node.cpp). The data race is a Phase 3 concern; Phase
// 2 (these tests) only verify single-threaded correctness:
//
//   * Assigned to the current mission id when a mission is accepted.
//   * Cleared on terminal transitions (SUCCEEDED / FAILED / CANCELLED).
//   * Remains constant for the duration of an active mission.
//
// All five test cases require *read access* to active_mission_id_, but
// MissionControllerNode exposes no public getter and no way to inject a
// deterministic mission id (generateMissionId() returns "mission_<ms>" from
// the wall clock). Two production seams are required to make these tests
// run; both are deliberately deferred so this file does NOT modify
// production code:
//
//   Seam A (read):   `const std::string& activeMissionId() const;`
//                    Must take std::lock_guard<std::mutex>(mission_mutex_) to
//                    avoid the very race Phase 3 will exercise. Returns by
//                    value -- not by reference -- once the mutex is taken.
//
//   Seam B (inject): `void setMissionIdFactory(std::function<std::string()>);`
//                    or constructor parameter, so a test can pin the id to a
//                    known string like "mission_abc". Without this we cannot
//                    write the AssignedOnAccept "equals mission_abc"
//                    assertion the task specifies.
//
// Until those seams exist, each test below uses GTEST_SKIP_(...) with a
// descriptive reason so CI surfaces the missing instrumentation rather than
// silently passing a no-op. The mission_id field of the published
// MissionStatus message provides a partial substitute (observable through
// /mission_status), and tests for that observable are already covered in
// test_mission_state_machine.cpp -- we deliberately do not duplicate them
// here.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "rises_mission_controller/mission_controller_node.hpp"

namespace {

class ActiveMissionIdTest : public ::testing::Test {
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
};

constexpr const char *kSeamReason =
    "Requires production seams `MissionControllerNode::activeMissionId()` "
    "(read under mission_mutex_) and a mission-id injection point (factory "
    "or ctor parameter). See file header for details.";

} // namespace

TEST_F(ActiveMissionIdTest, AssignedOnAccept) {
  // Construction is cheap; do it here so the skip path still exercises the
  // ctor and surfaces any link-time regressions.
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rises::MissionControllerNode>(options);
  ASSERT_NE(node, nullptr);

  GTEST_SKIP() << kSeamReason
               << " Cannot assert active_mission_id_ == \"mission_abc\" "
                  "without an id injection seam and a public getter.";
}

TEST_F(ActiveMissionIdTest, ClearedOnTerminal) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rises::MissionControllerNode>(options);
  ASSERT_NE(node, nullptr);

  GTEST_SKIP() << kSeamReason
               << " Cannot observe active_mission_id_ becoming empty after "
                  "STATUS_SUCCEEDED without a public getter.";
}

TEST_F(ActiveMissionIdTest, ClearedOnFailure) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rises::MissionControllerNode>(options);
  ASSERT_NE(node, nullptr);

  GTEST_SKIP() << kSeamReason
               << " Cannot observe active_mission_id_ becoming empty after "
                  "STATUS_FAILED without a public getter.";
}

TEST_F(ActiveMissionIdTest, ClearedOnCancel) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rises::MissionControllerNode>(options);
  ASSERT_NE(node, nullptr);

  GTEST_SKIP() << kSeamReason
               << " Cannot observe active_mission_id_ becoming empty after "
                  "STATUS_CANCELLED without a public getter.";
}

TEST_F(ActiveMissionIdTest, NotMutatedDuringExecution) {
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rises::MissionControllerNode>(options);
  ASSERT_NE(node, nullptr);

  GTEST_SKIP()
      << kSeamReason
      << " Cannot poll active_mission_id_ across consecutive task "
         "boundaries without a public getter. The mission_id field on "
         "the published MissionStatus messages is a partial substitute "
         "and is asserted via the state-machine suite instead.";
}
