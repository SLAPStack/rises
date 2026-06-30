// Unit tests for rises::ObstacleAggregatorNode combine logic.
//
// Package: obstacle_aggregator
// Strategy:
//   - Stand up the real ObstacleAggregatorNode under a SingleThreadedExecutor
//     with a high-rate publish timer (kTimerRateHz) so combined output arrives
//     promptly without depending on wall-clock duration.
//   - Drive inputs via per-namespace publishers on /<ns>/obstacle_report and
//     observe outputs via a spy subscriber on /fleet/obstacle_report. This
//     mirrors the deployment topology exactly.
//   - All waits are bounded by a deterministic spin-budget (kMaxSpinIterations
//     * kSpinSlice); tests assert on outputs, not on elapsed wall time.
//
// GTEST_SKIP rationale:
//   - StaleReportEvictedAfterTtl: the production aggregator has NO TTL
//     concept. publishCombined() unconditionally republishes the cached
//     `latest_reports_` map; stale entries are never evicted. Skipping until
//     a TTL seam is added.
//   - OutputCadenceMatchesTimer: the wall-timer cannot be driven from the
//     test thread deterministically — its period is decoupled from any
//     /clock and there is no injected clock seam on ObstacleAggregatorNode.
//     Skipping until the timer becomes test-driven (e.g. a public
//     triggerPublish() method or sim-time wall_timer).

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_report.hpp>
#include <rclcpp/rclcpp.hpp>

#include "obstacle_aggregator/obstacle_aggregator_node.hpp"
#include "test_support/obstacle_builder.hpp"

namespace {

using rises_interfaces::msg::Obstacle;
using rises_interfaces::msg::ObstacleReport;

constexpr double kTimerRateHz = 50.0;
constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr int kMaxSpinIterations = 200;

class AggregatorCombineFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void buildAggregator(const std::vector<std::string> &namespaces) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"agv_namespaces", namespaces}, {"publish_rate_hz", kTimerRateHz}});
    node_ = std::make_shared<rises::ObstacleAggregatorNode>(options);

    for (const std::string &ns : namespaces) {
      const std::string topic = "/" + ns + "/obstacle_report";
      auto pub_node = std::make_shared<rclcpp::Node>("test_combine_pub_" + ns);
      auto pub = pub_node->create_publisher<ObstacleReport>(
          topic, rclcpp::SensorDataQoS().keep_last(10));
      pub_nodes_.push_back(pub_node);
      publishers_[ns] = pub;
    }

    spy_ = std::make_shared<rclcpp::Node>("test_combine_spy");
    last_combined_.reset();
    combined_count_ = 0;
    spy_sub_ = spy_->create_subscription<ObstacleReport>(
        "/fleet/obstacle_report", rclcpp::SensorDataQoS().keep_last(10),
        [this](ObstacleReport::SharedPtr msg) {
          last_combined_ = msg;
          ++combined_count_;
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    for (auto &pub_node : pub_nodes_) {
      executor_->add_node(pub_node);
    }
    executor_->add_node(spy_);
  }

  void TearDown() override {
    executor_.reset();
    spy_sub_.reset();
    spy_.reset();
    publishers_.clear();
    pub_nodes_.clear();
    node_.reset();
    last_combined_.reset();
    combined_count_ = 0;
  }

  void publishReport(const std::string &ns,
                     const std::vector<Obstacle> &unmatched,
                     const std::vector<Obstacle> &matched = {}) {
    ObstacleReport report;
    report.header.frame_id = "map";
    report.header.stamp = node_->now();
    report.unmatched_obstacles = unmatched;
    report.matched_obstacles = matched;
    publishers_.at(ns)->publish(report);
  }

  bool spinUntil(const std::function<bool()> &predicate) {
    for (int i = 0; i < kMaxSpinIterations; ++i) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    return false;
  }

  bool waitForCombinedWithUnmatched(std::size_t expected_count) {
    return spinUntil([&]() {
      return last_combined_ != nullptr &&
             last_combined_->unmatched_obstacles.size() >= expected_count;
    });
  }

  std::shared_ptr<rises::ObstacleAggregatorNode> node_;
  std::vector<std::shared_ptr<rclcpp::Node>> pub_nodes_;
  std::unordered_map<std::string, rclcpp::Publisher<ObstacleReport>::SharedPtr>
      publishers_;
  std::shared_ptr<rclcpp::Node> spy_;
  rclcpp::Subscription<ObstacleReport>::SharedPtr spy_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  ObstacleReport::SharedPtr last_combined_;
  int combined_count_ = 0;
};

std::unordered_set<std::uint64_t>
collectUnmatchedIds(const ObstacleReport &report) {
  std::unordered_set<std::uint64_t> ids;
  for (const Obstacle &obs : report.unmatched_obstacles) {
    ids.insert(obs.id);
  }
  return ids;
}

} // namespace

// A single AGV reporting one unmatched obstacle must appear in the combined
// fleet output. Position metadata is preserved.
TEST_F(AggregatorCombineFixture, SingleAgvReportPassesThrough) {
  buildAggregator({"agv_0"});

  constexpr double kX = 1.5;
  constexpr double kY = 2.5;
  publishReport("agv_0", {test_support::ObstacleBuilder::point(7, kX, kY)});

  ASSERT_TRUE(waitForCombinedWithUnmatched(1u));
  ASSERT_EQ(last_combined_->unmatched_obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(
      last_combined_->unmatched_obstacles.front().vertices.front().x, kX);
  EXPECT_DOUBLE_EQ(
      last_combined_->unmatched_obstacles.front().vertices.front().y, kY);
}

// Two AGVs with disjoint obstacle IDs must produce the union in the combined
// output. This test uses different IDs so it is independent of the audit-11
// ID-offset behaviour and remains valid both pre- and post-fix.
TEST_F(AggregatorCombineFixture, TwoAgvUnmatchedConcatenated) {
  buildAggregator({"agv_0", "agv_1"});

  publishReport("agv_0", {test_support::ObstacleBuilder::point(10, 1.0, 1.0)});
  publishReport("agv_1", {test_support::ObstacleBuilder::point(20, 2.0, 2.0)});

  ASSERT_TRUE(waitForCombinedWithUnmatched(2u));
  const auto ids = collectUnmatchedIds(*last_combined_);
  EXPECT_NE(ids.find(10), ids.end());
  EXPECT_NE(ids.find(20), ids.end());
}

// Two AGVs report obstacles at similar positions but with disjoint IDs. The
// aggregator does no spatial fusion (it appends every obstacle verbatim), so
// both entries must be preserved.
TEST_F(AggregatorCombineFixture, MatchedObstaclesPreservedPerAgv) {
  buildAggregator({"agv_0", "agv_1"});

  // Use the matched_obstacles channel: the aggregator forwards both
  // matched and unmatched arrays independently.
  publishReport(
      /*ns=*/"agv_0", /*unmatched=*/{},
      /*matched=*/{test_support::ObstacleBuilder::point(100, 5.0, 5.0)});
  publishReport(
      /*ns=*/"agv_1", /*unmatched=*/{},
      /*matched=*/{test_support::ObstacleBuilder::point(200, 5.05, 5.0)});

  ASSERT_TRUE(spinUntil([&]() {
    return last_combined_ != nullptr &&
           last_combined_->matched_obstacles.size() >= 2u;
  })) << "Aggregator did not preserve matched obstacles from both AGVs.";

  std::unordered_set<std::uint64_t> matched_ids;
  for (const Obstacle &obs : last_combined_->matched_obstacles) {
    matched_ids.insert(obs.id);
  }
  EXPECT_NE(matched_ids.find(100), matched_ids.end());
  EXPECT_NE(matched_ids.find(200), matched_ids.end());
}

// An empty report from agv_0 must not suppress agv_1's obstacles. The cached
// map keeps one entry per namespace, so the union must still include agv_1.
TEST_F(AggregatorCombineFixture, EmptyReportFromOneAgvDoesNotDropOthers) {
  buildAggregator({"agv_0", "agv_1"});

  publishReport("agv_0", /*unmatched=*/{});
  publishReport("agv_1", {test_support::ObstacleBuilder::point(42, 3.0, 3.0)});

  ASSERT_TRUE(waitForCombinedWithUnmatched(1u));
  const auto ids = collectUnmatchedIds(*last_combined_);
  EXPECT_NE(ids.find(42), ids.end())
      << "agv_1's obstacle was dropped after agv_0 sent an empty report.";
}

// No TTL implemented in production — see file header.
TEST_F(AggregatorCombineFixture, StaleReportEvictedAfterTtl) {
  GTEST_SKIP_(
      "ObstacleAggregatorNode has no TTL/eviction logic; "
      "latest_reports_ entries are kept indefinitely. Re-enable once a TTL "
      "parameter and per-entry timestamp-based eviction land.");
}

// No injectable clock / timer seam — see file header.
TEST_F(AggregatorCombineFixture, OutputCadenceMatchesTimer) {
  GTEST_SKIP_(
      "ObstacleAggregatorNode uses a wall_timer with no test-driving seam; "
      "cadence cannot be asserted deterministically. Re-enable once a public "
      "triggerPublish() method or sim-time timer is exposed.");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
