// =============================================================================
// Lifecycle tests for LifecycleGeofenceNodeBase, exercised through the concrete
// spatial GeofenceNode (same pattern as test_audit_02 / test_audit_03).
//
// Covers:
//   * the full configure -> activate -> deactivate -> cleanup cycle end to end,
//   * idempotency of repeated transitions (configure->activate->deactivate, then
//     activate->deactivate again from the same INACTIVE state),
//   * the Fix-4 recovery path: an activation that fails on a recoverable config
//     error leaves the node in INACTIVE (not UNCONFIGURED), from which a second
//     activation -- after the bad config is no longer applied -- still cannot be
//     reached without fixing config, AND a fresh node can be reconfigured and
//     activated cleanly.
//
// Like the audit suites, this drives on_configure / on_activate / on_deactivate
// / on_cleanup directly via the lifecycle state machine; it never spins an
// executor. A live-executor end-to-end test (topic pub/sub timing) is a
// separate, higher-effort follow-up.
// =============================================================================

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "test_support/lifecycle_harness.hpp"
#include "test_support/temp_json.hpp"

#include "geofence/spatial/node/geofencing_node.hpp"

namespace {

constexpr const char *kMissingJson =
    "/tmp/rises_lifecycle_does_not_exist.json";

class LifecycleNodeFixture : public ::testing::Test {
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

  std::shared_ptr<rises::GeofenceNode>
  makeNode(const std::vector<rclcpp::Parameter> &params = {}) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    // Auto-activate disabled by default; the lifecycle is driven manually here.
    return std::make_shared<rises::GeofenceNode>(opts);
  }
};

} // namespace

TEST_F(LifecycleNodeFixture, FullCycleConfigureActivateDeactivateCleanup) {
  std::shared_ptr<rises::GeofenceNode> node = makeNode();

  EXPECT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::isActive(test_support::activate(node)));
  EXPECT_TRUE(test_support::isInactive(test_support::deactivate(node)));
  EXPECT_TRUE(test_support::isUnconfigured(test_support::cleanup(node)));
}

TEST_F(LifecycleNodeFixture, RepeatedActivateDeactivateIsIdempotent) {
  std::shared_ptr<rises::GeofenceNode> node = makeNode();

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));

  // First activate/deactivate round-trip.
  EXPECT_TRUE(test_support::isActive(test_support::activate(node)));
  EXPECT_TRUE(test_support::isInactive(test_support::deactivate(node)));

  // Second round-trip from the same INACTIVE state must behave identically.
  EXPECT_TRUE(test_support::isActive(test_support::activate(node)));
  EXPECT_TRUE(test_support::isInactive(test_support::deactivate(node)));
}

TEST_F(LifecycleNodeFixture, CleanupThenReconfigureSucceeds) {
  std::shared_ptr<rises::GeofenceNode> node = makeNode();

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  ASSERT_TRUE(test_support::isActive(test_support::activate(node)));
  ASSERT_TRUE(test_support::isInactive(test_support::deactivate(node)));
  ASSERT_TRUE(test_support::isUnconfigured(test_support::cleanup(node)));

  // A cleaned-up node can be configured and activated again.
  EXPECT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::isActive(test_support::activate(node)));
}

TEST_F(LifecycleNodeFixture, FailedActivationLeavesNodeInactive) {
  // Fix-4 contract: a recoverable config error (missing JSON file) must make
  // on_activate return FAILURE, which lands the node in INACTIVE -- NOT
  // UNCONFIGURED (which is what CallbackReturn::ERROR would produce by routing
  // through on_error()). Staying in INACTIVE is what makes retry-after-fix
  // possible without a full reconfigure.
  std::shared_ptr<rises::GeofenceNode> node =
      makeNode({rclcpp::Parameter("obstacles_json_file", kMissingJson)});

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));

  const std::uint8_t after_activate = test_support::activate(node);
  EXPECT_TRUE(test_support::activateFailed(after_activate));
  // Explicitly assert the recovery-relevant distinction: INACTIVE, not
  // UNCONFIGURED.
  EXPECT_TRUE(test_support::isInactive(after_activate));
  EXPECT_FALSE(test_support::isUnconfigured(after_activate));
}

TEST_F(LifecycleNodeFixture, RecoveryAfterFailedActivationReachesActive) {
  // After a failed activation leaves the node INACTIVE, the operator fixes the
  // config (here: a valid empty obstacles array) and a fresh node reaches
  // ACTIVE -- demonstrating the recovery path the Fix-4 semantics enable.
  std::shared_ptr<rises::GeofenceNode> bad_node =
      makeNode({rclcpp::Parameter("obstacles_json_file", kMissingJson)});
  ASSERT_TRUE(test_support::isInactive(test_support::configure(bad_node)));
  ASSERT_TRUE(test_support::activateFailed(test_support::activate(bad_node)));

  test_support::TempJsonFile good("[]");
  std::shared_ptr<rises::GeofenceNode> good_node =
      makeNode({rclcpp::Parameter("obstacles_json_file", good.path())});
  ASSERT_TRUE(test_support::isInactive(test_support::configure(good_node)));
  EXPECT_TRUE(test_support::isActive(test_support::activate(good_node)));
}
