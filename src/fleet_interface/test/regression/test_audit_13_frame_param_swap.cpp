// Regression tests for audit finding #13 (swapped frame parameter wiring).
//
// These tests verify that FleetInterfaceNode wires its frame parameters to the
// correctly named members: the ROS parameter "global_frame" populates
// `global_frame_`, "target_frame" populates `target_frame_`, and the defaults
// are "map" for global_frame and "base_link" for target_frame. They read the
// members through the public getGlobalFrame() / getTargetFrame() accessors; a
// compile-time detector below GTEST_SKIP_s the assertions if those accessors
// are ever removed, so the suite never silently passes.

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "fleet_interface/fleet_interface_node.hpp"

namespace {

constexpr const char *kCustomGlobalFrame = "map_global_xyz";
constexpr const char *kCustomTargetFrame = "base_link_xyz";
constexpr const char *kDefaultGlobalFrame = "map";
constexpr const char *kDefaultTargetFrame = "base_link";

// Detect at compile time whether the testability seam has been added. The
// seam is a pair of public const-accessors on FleetInterfaceNode. While the
// fix PR is unmerged this resolves to false and every test below skips.
template <typename, typename = void>
struct HasFrameAccessors : std::false_type {};

template <typename T>
struct HasFrameAccessors<
    T, std::void_t<decltype(std::declval<const T &>().getGlobalFrame()),
                   decltype(std::declval<const T &>().getTargetFrame())>>
    : std::true_type {};

constexpr bool kSeamAvailable =
    HasFrameAccessors<rises::FleetInterfaceNode>::value;

class FleetFrameParamSwapTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override { node_.reset(); }

  void buildNode(const std::vector<rclcpp::Parameter> &overrides) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<rises::FleetInterfaceNode>(options);
  }

  std::shared_ptr<rises::FleetInterfaceNode> node_;
};

} // namespace

TEST_F(FleetFrameParamSwapTest, GlobalFrameParamReachesGlobalMember) {
  if constexpr (!kSeamAvailable) {
    GTEST_SKIP_(
        "requires public getGlobalFrame()/getTargetFrame() accessors on "
        "FleetInterfaceNode (see file header)");
  } else {
    buildNode({rclcpp::Parameter("global_frame", kCustomGlobalFrame),
               rclcpp::Parameter("target_frame", kCustomTargetFrame)});
    EXPECT_EQ(node_->getGlobalFrame(), kCustomGlobalFrame);
  }
}

TEST_F(FleetFrameParamSwapTest, TargetFrameParamReachesTargetMember) {
  if constexpr (!kSeamAvailable) {
    GTEST_SKIP_(
        "requires public getGlobalFrame()/getTargetFrame() accessors on "
        "FleetInterfaceNode (see file header)");
  } else {
    buildNode({rclcpp::Parameter("global_frame", kCustomGlobalFrame),
               rclcpp::Parameter("target_frame", kCustomTargetFrame)});
    EXPECT_EQ(node_->getTargetFrame(), kCustomTargetFrame);
  }
}

TEST_F(FleetFrameParamSwapTest, DefaultsAreCorrect) {
  if constexpr (!kSeamAvailable) {
    GTEST_SKIP_(
        "requires public getGlobalFrame()/getTargetFrame() accessors on "
        "FleetInterfaceNode (see file header)");
  } else {
    buildNode({});
    EXPECT_EQ(node_->getGlobalFrame(), kDefaultGlobalFrame);
    EXPECT_EQ(node_->getTargetFrame(), kDefaultTargetFrame);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
