// =============================================================================
// Audit Finding #2 -- on_activate swallows JSON load failures
// =============================================================================
//
// Location: geofence/spatial_node/src/node/geofencing_node.cpp:273-289
//           (gridmap mirror at geofencing_gridmap_node.cpp:437-452)
//
// When the `obstacles_json_file` or `contours_json_file` parameter is non-empty
// but the file cannot be loaded (missing, malformed, etc.), the node merely
// logs RCLCPP_ERROR and continues. on_activate still returns SUCCESS, so the
// lifecycle reaches ACTIVE with an empty map and downstream consumers operate
// blind.
//
// Expected fix: when a configured JSON file fails to load, return
// `CallbackReturn::ERROR` (or FAILURE) from on_activate so the lifecycle stays
// in INACTIVE. test_support::activateFailed() codifies that expectation.
//
// STATUS: RED on develop. The "missing/malformed -> activateFailed" cases
// below are expected to FAIL until the fix lands. The "valid empty array"
// control case should be GREEN today (it tests that a successful load still
// reaches ACTIVE -- this guards against an over-eager fix).
//
// Implementation note: this suite drives the real GeofenceNode through its
// lifecycle. Construction allocates many publishers/subscribers/services; we
// do not spin the executor -- only on_configure / on_activate are exercised.
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

constexpr const char *kMissingPath = "/tmp/rises_audit02_does_not_exist.json";

class JsonLoadAbortFixture : public ::testing::Test {
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
  makeNode(const std::vector<rclcpp::Parameter> &params) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    // Auto-activate disabled by default; we drive the lifecycle manually.
    return std::make_shared<rises::GeofenceNode>(opts);
  }
};

} // namespace

TEST_F(JsonLoadAbortFixture, MissingObstaclesJsonFailsActivation) {
  auto node =
      makeNode({rclcpp::Parameter("obstacles_json_file", kMissingPath)});

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::activateFailed(test_support::activate(node)));
}

TEST_F(JsonLoadAbortFixture, MalformedObstaclesJsonFailsActivation) {
  test_support::TempJsonFile bad("not valid json");
  auto node = makeNode({rclcpp::Parameter("obstacles_json_file", bad.path())});

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::activateFailed(test_support::activate(node)));
}

TEST_F(JsonLoadAbortFixture, MissingContoursJsonFailsActivation) {
  auto node = makeNode({rclcpp::Parameter("contours_json_file", kMissingPath)});

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::activateFailed(test_support::activate(node)));
}

TEST_F(JsonLoadAbortFixture, ValidEmptyArrayActivatesSuccessfully) {
  // Control case: a syntactically valid empty array is a successful load and
  // must not block activation. This protects against an over-eager fix that
  // would reject any "no obstacles loaded" outcome.
  test_support::TempJsonFile good("[]");
  auto node = makeNode({rclcpp::Parameter("obstacles_json_file", good.path())});

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::isActive(test_support::activate(node)));
}
