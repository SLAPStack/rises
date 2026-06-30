// Regression tests for audit finding #4 (oversize VDA5050 order DoS).
//
// Bug location:
//   fleet_interface/src/vda5050_converter.cpp:137
//     path.poses.reserve(nodes.size());
//   message_translator/src/vda5050_converter.cpp:99 — identical duplicate.
//
// The `nodes` array length is attacker-controlled JSON input; an order with
// millions of node entries forces an unbounded reserve and exhausts heap
// before any per-node validation runs.
//
// Expected behaviour after the fix:
//   - A constexpr cap (suggested name MAX_VDA5050_NODES = 10000) is enforced
//     in both fleet_interface and message_translator copies of the converter.
//   - Orders with `nodes.size() <= MAX_VDA5050_NODES` are accepted and
//     produce a Path with one pose per parsable node.
//   - Orders with `nodes.size() >  MAX_VDA5050_NODES` are rejected — the
//     converter returns an empty Path and logs an error on the
//     "vda5050_converter" logger.
//
// Red-on-develop status:
//   AtCapAccepted              — likely GREEN today (accepts everything).
//   BelowCapAccepted           — GREEN today.
//   EmptyNodesArrayProducesEmptyPath — GREEN today.
//   JustOverCapRejected        — RED until the fix lands (current converter
//                                 happily reserves 10001 poses and returns a
//                                 non-empty path).
//   BothPackagesShareIdenticalCap — RED until the fix lands.
//
// Cross-package note: message_translator ships a second copy of
// `rises::Vda5050Converter` with the same symbol name. The two definitions
// cannot coexist in one binary without an ODR violation, so this test only
// links fleet_interface's copy and exercises it twice with the same payload
// to assert cap stability. The matching duplicate in message_translator is
// covered indirectly via that package's own oversize regression test until
// the converters are deduplicated into a shared library.

#include <chrono>
#include <cstddef>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "fleet_interface/vda5050_converter.hpp"
#include "test_support/oversize_msg_factory.hpp"

namespace {

// The fix PR must define this same constant in production. Any divergence
// between this value and the production cap fails the tests by design — it
// is the contract that pins the cap in place.
constexpr std::size_t kMaxVda5050Nodes = 10000;
constexpr const char *kFrameId = "map";

// Wall-clock deadline: an unbounded reserve on N+1 nodes (where N is in the
// millions for true DoS, but our test stays at the cap edge) must still
// complete promptly once the cap is enforced.
constexpr std::chrono::milliseconds kRejectDeadline{500};

rclcpp::Time zeroStamp() { return rclcpp::Time(0, 0, RCL_ROS_TIME); }

} // namespace

TEST(OversizeVda5050, BelowCapAccepted) {
  const std::size_t node_count = 1000;
  const std::string order =
      test_support::oversize::makeVda5050OrderJson(node_count);

  const nav_msgs::msg::Path path =
      rises::Vda5050Converter::orderToPath(order, kFrameId, zeroStamp());

  EXPECT_EQ(path.poses.size(), node_count);
  EXPECT_EQ(path.header.frame_id, kFrameId);
}

TEST(OversizeVda5050, AtCapAccepted) {
  const std::string order =
      test_support::oversize::makeVda5050OrderJson(kMaxVda5050Nodes);

  const nav_msgs::msg::Path path =
      rises::Vda5050Converter::orderToPath(order, kFrameId, zeroStamp());

  EXPECT_EQ(path.poses.size(), kMaxVda5050Nodes);
}

TEST(OversizeVda5050, JustOverCapRejected) {
  const std::string order =
      test_support::oversize::makeVda5050OrderJson(kMaxVda5050Nodes + 1);

  const auto start = std::chrono::steady_clock::now();
  const nav_msgs::msg::Path path =
      rises::Vda5050Converter::orderToPath(order, kFrameId, zeroStamp());
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_TRUE(path.poses.empty())
      << "Over-cap order should be rejected with an empty Path; got "
      << path.poses.size() << " poses.";
  EXPECT_LT(elapsed, kRejectDeadline)
      << "Over-cap rejection should be cheap (no per-node parse, no reserve).";
}

TEST(OversizeVda5050, EmptyNodesArrayProducesEmptyPath) {
  const std::string order = test_support::oversize::makeVda5050OrderJson(0);

  const nav_msgs::msg::Path path =
      rises::Vda5050Converter::orderToPath(order, kFrameId, zeroStamp());

  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, kFrameId);
}

TEST(OversizeVda5050, BothPackagesShareIdenticalCap) {
  // Documents that fleet_interface and message_translator carry duplicate
  // copies of Vda5050Converter; both must enforce the same cap. ODR
  // prevents linking both copies into one binary, so we exercise
  // fleet_interface's copy twice and rely on the message_translator
  // package's own regression test to pin its duplicate. The fix PR should
  // either share the cap via a header constant or deduplicate into a single
  // converter library.
  const std::string over_cap_order =
      test_support::oversize::makeVda5050OrderJson(kMaxVda5050Nodes + 1);

  const nav_msgs::msg::Path first = rises::Vda5050Converter::orderToPath(
      over_cap_order, kFrameId, zeroStamp());
  const nav_msgs::msg::Path second = rises::Vda5050Converter::orderToPath(
      over_cap_order, kFrameId, zeroStamp());

  EXPECT_TRUE(first.poses.empty());
  EXPECT_TRUE(second.poses.empty());
  EXPECT_EQ(first.poses.size(), second.poses.size());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
