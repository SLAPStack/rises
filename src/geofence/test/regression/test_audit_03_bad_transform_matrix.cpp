// =============================================================================
// Audit Finding #3 -- Silent failure on invalid obstacle_transform_matrix
// =============================================================================
//
// Location: geofence/spatial_node/src/node/geofence_config.cpp:20,31
//
// `getMatrixParam` returns an empty vector when std::stod throws while parsing
// a string-typed matrix parameter. The caller `GeofenceConfig::fromNode`
// stores that empty vector without checking. Downstream
// `applyCoordinateTransform()` logs ERROR and silently skips installing the
// transform (geofence_config.cpp:190-196), so the node activates as if no
// transform were configured -- even though the user explicitly enabled it.
//
// Expected fix: when `transform_obstacles_enabled=true`, an invalid /
// wrong-size matrix must fail loudly. The cleanest path is for
// applyCoordinateTransform (or on_configure / on_activate) to return
// CallbackReturn::ERROR / FAILURE rather than RCLCPP_ERROR + continue.
//
// STATUS: RED on develop. The three "invalid -> activateFailed" cases below
// are expected to FAIL until the fix lands. The valid-identity case is the
// control and should be GREEN today.
// =============================================================================

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "test_support/lifecycle_harness.hpp"

#include "geofence/spatial/node/geofencing_node.hpp"

namespace {

class BadTransformFixture : public ::testing::Test {
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
    return std::make_shared<rises::GeofenceNode>(opts);
  }
};

// Identity matrix expressed as a 16-element row-major double array.
const std::vector<double> kValidIdentity = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                            0.0, 0.0, 0.0, 1.0};

// 15 elements -- off-by-one, must be rejected.
const std::vector<double> kFifteenElements = {
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};

} // namespace

TEST_F(BadTransformFixture, EnabledFlagWithMalformedMatrixFailsActivation) {
  auto node = makeNode({
      rclcpp::Parameter("transform_obstacles_enabled", true),
      // String type triggers the std::stod parsing path that swallows errors.
      rclcpp::Parameter("obstacle_transform_matrix",
                        std::string("not numeric")),
  });

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::activateFailed(test_support::activate(node)));
}

TEST_F(BadTransformFixture, EnabledFlagWithEmptyArrayFailsActivation) {
  auto node = makeNode({
      rclcpp::Parameter("transform_obstacles_enabled", true),
      rclcpp::Parameter("obstacle_transform_matrix", std::string("")),
  });

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::activateFailed(test_support::activate(node)));
}

TEST_F(BadTransformFixture, EnabledFlagWithFifteenElementsFailsActivation) {
  auto node = makeNode({
      rclcpp::Parameter("transform_obstacles_enabled", true),
      rclcpp::Parameter("obstacle_transform_matrix", kFifteenElements),
  });

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::activateFailed(test_support::activate(node)));
}

TEST_F(BadTransformFixture, ValidIdentityMatrixActivatesNormally) {
  // Control case: a valid 16-element identity must activate cleanly.
  auto node = makeNode({
      rclcpp::Parameter("transform_obstacles_enabled", true),
      rclcpp::Parameter("obstacle_transform_matrix", kValidIdentity),
  });

  ASSERT_TRUE(test_support::isInactive(test_support::configure(node)));
  EXPECT_TRUE(test_support::isActive(test_support::activate(node)));
}
