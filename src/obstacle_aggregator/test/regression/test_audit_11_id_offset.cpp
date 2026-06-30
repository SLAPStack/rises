// Regression test for audit finding #11 (ID-offset collision).
//
// Bug location: obstacle_aggregator/src/obstacle_aggregator_node.cpp:68-69
// (lines that comment "Offset IDs to avoid collisions: agv_0 gets IDs
//  0-999999, agv_1 gets 1000000-1999999, etc." but copy obstacle IDs verbatim
//  in publishCombined()).
//
// Expected post-fix behaviour: each subscribed AGV namespace gets a
// deterministic, non-overlapping ID-offset range, so two AGVs reporting
// obstacles with the same internal ID end up as distinct entries in the
// combined /fleet/obstacle_report. The fix is expected to live in
// publishCombined() (and may introduce a helper such as offsetFor(ns)).
//
// Status on develop:
//   - TwoAgvsWithIdOneAreDistinguished: RED on develop.
//   - ThreeAgvsWithOverlappingIds: RED on develop.
//   - IdOffsetIsDeterministicPerNamespace: GREEN on develop (verbatim copy is
//     trivially deterministic) and GREEN post-fix; positive control.
//   - SingleAgvIdsPassUnmodifiedOrConsistentlyOffset: GREEN on develop and
//     post-fix; documents the contract that single-AGV ids must remain unique.
//   - OffsetDoesNotOverlapAcrossNamespaces: RED on develop (the verbatim
//     copy guarantees overlap).

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_report.hpp>
#include <rclcpp/rclcpp.hpp>
#include <test_support/obstacle_builder.hpp>

#include "obstacle_aggregator/obstacle_aggregator_node.hpp"

namespace {

using rises_interfaces::msg::Obstacle;
using rises_interfaces::msg::ObstacleReport;

constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr int kMaxSpinIterations = 200;
constexpr double kTimerRateHz = 50.0;

class AggregatorFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuiteIfNeeded() {}

  void buildAggregator(const std::vector<std::string> &namespaces) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"agv_namespaces", namespaces}, {"publish_rate_hz", kTimerRateHz}});
    node_ = std::make_shared<rises::ObstacleAggregatorNode>(options);

    // Publisher per namespace simulating each AGV's /obstacle_report.
    for (const std::string &ns : namespaces) {
      const std::string topic = "/" + ns + "/obstacle_report";
      auto pub = std::make_shared<rclcpp::Node>("test_pub_" + ns);
      auto p = pub->create_publisher<ObstacleReport>(
          topic, rclcpp::SensorDataQoS().keep_last(10));
      pub_nodes_.push_back(pub);
      publishers_[ns] = p;
    }

    // Spy subscriber on /fleet/obstacle_report.
    spy_ = std::make_shared<rclcpp::Node>("test_fleet_spy");
    last_combined_.reset();
    spy_sub_ = spy_->create_subscription<ObstacleReport>(
        "/fleet/obstacle_report", rclcpp::SensorDataQoS().keep_last(10),
        [this](const ObstacleReport::SharedPtr msg) { last_combined_ = msg; });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    for (auto &pub_node : pub_nodes_) {
      executor_->add_node(pub_node);
    }
    executor_->add_node(spy_);
  }

  void TearDown() override {
    if (executor_) {
      executor_.reset();
    }
    publishers_.clear();
    pub_nodes_.clear();
    spy_sub_.reset();
    spy_.reset();
    node_.reset();
    last_combined_.reset();
  }

  void publishReport(const std::string &ns,
                     const std::vector<Obstacle> &unmatched) {
    ObstacleReport report;
    report.header.frame_id = "map";
    report.unmatched_obstacles = unmatched;
    publishers_.at(ns)->publish(report);
  }

  /// Spin until @p predicate returns true or @p kMaxSpinIterations expires.
  /// Returns true on success.
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

  bool waitForCombinedWithAtLeast(std::size_t expected_count) {
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
};

std::unordered_set<std::uint64_t> collectIds(const ObstacleReport &report) {
  std::unordered_set<std::uint64_t> ids;
  for (const Obstacle &obs : report.unmatched_obstacles) {
    ids.insert(obs.id);
  }
  return ids;
}

} // namespace

// Two AGVs each publish an obstacle with id=1. Post-fix the combined report
// must contain two entries with distinct IDs (the offset disambiguates).
TEST_F(AggregatorFixture, TwoAgvsWithIdOneAreDistinguished) {
  buildAggregator({"agv_0", "agv_1"});

  publishReport("agv_0", {test_support::ObstacleBuilder::point(1, 1.0, 1.0)});
  publishReport("agv_1", {test_support::ObstacleBuilder::point(1, 2.0, 2.0)});

  ASSERT_TRUE(waitForCombinedWithAtLeast(2))
      << "Combined report did not contain both AGVs' obstacles in time";

  const std::unordered_set<std::uint64_t> ids = collectIds(*last_combined_);
  EXPECT_EQ(ids.size(), 2u)
      << "Two AGV obstacles with the same internal id collided "
         "(missing ID offset).";
}

// Three AGVs each publish ids 1..5 -> 15 distinct entries post-fix.
TEST_F(AggregatorFixture, ThreeAgvsWithOverlappingIds) {
  buildAggregator({"agv_0", "agv_1", "agv_2"});

  constexpr int kPerAgv = 5;
  auto makeBatch = [](double base_x) {
    std::vector<Obstacle> obstacles;
    for (int i = 1; i <= kPerAgv; ++i) {
      obstacles.push_back(test_support::ObstacleBuilder::point(
          static_cast<std::uint64_t>(i), base_x + i, 0.0));
    }
    return obstacles;
  };

  publishReport("agv_0", makeBatch(0.0));
  publishReport("agv_1", makeBatch(10.0));
  publishReport("agv_2", makeBatch(20.0));

  constexpr std::size_t kExpected = 3u * kPerAgv;
  ASSERT_TRUE(waitForCombinedWithAtLeast(kExpected))
      << "Combined report did not aggregate 15 entries in time";

  const std::unordered_set<std::uint64_t> ids = collectIds(*last_combined_);
  EXPECT_EQ(ids.size(), kExpected)
      << "Overlapping ids across three AGVs collapsed; ID offset is missing.";
}

// Reissuing the same AGV report twice must produce the same offset IDs
// (offset is a pure function of namespace, not call count).
TEST_F(AggregatorFixture, IdOffsetIsDeterministicPerNamespace) {
  buildAggregator({"agv_0", "agv_1"});

  publishReport("agv_0", {test_support::ObstacleBuilder::point(42, 0.0, 0.0)});
  publishReport("agv_1", {test_support::ObstacleBuilder::point(99, 0.0, 0.0)});
  ASSERT_TRUE(waitForCombinedWithAtLeast(2));

  const std::unordered_set<std::uint64_t> first_ids =
      collectIds(*last_combined_);

  // Reset spy and republish identical payloads.
  last_combined_.reset();
  publishReport("agv_0", {test_support::ObstacleBuilder::point(42, 0.0, 0.0)});
  publishReport("agv_1", {test_support::ObstacleBuilder::point(99, 0.0, 0.0)});
  ASSERT_TRUE(waitForCombinedWithAtLeast(2));

  const std::unordered_set<std::uint64_t> second_ids =
      collectIds(*last_combined_);
  EXPECT_EQ(first_ids, second_ids)
      << "Offset assignment is not deterministic per namespace.";
}

// Single-AGV scenario: IDs are either pass-through OR uniformly offset.
// Either is acceptable; the contract is that they remain unique within the
// single namespace.
TEST_F(AggregatorFixture, SingleAgvIdsPassUnmodifiedOrConsistentlyOffset) {
  buildAggregator({"agv_0"});

  publishReport("agv_0", {
                             test_support::ObstacleBuilder::point(1, 0.0, 0.0),
                             test_support::ObstacleBuilder::point(2, 1.0, 0.0),
                             test_support::ObstacleBuilder::point(3, 2.0, 0.0),
                         });

  ASSERT_TRUE(waitForCombinedWithAtLeast(3));
  const std::unordered_set<std::uint64_t> ids = collectIds(*last_combined_);
  EXPECT_EQ(ids.size(), 3u) << "Single-AGV obstacles lost distinctness.";
}

// Offset ranges per namespace must not intersect.
TEST_F(AggregatorFixture, OffsetDoesNotOverlapAcrossNamespaces) {
  buildAggregator({"agv_0", "agv_1"});

  // Probe a range of internal IDs from each AGV and assert no overlap.
  std::vector<Obstacle> agv0;
  std::vector<Obstacle> agv1;
  constexpr int kProbeCount = 8;
  for (int i = 0; i < kProbeCount; ++i) {
    agv0.push_back(test_support::ObstacleBuilder::point(
        static_cast<std::uint64_t>(i), 0.0, 0.0));
    agv1.push_back(test_support::ObstacleBuilder::point(
        static_cast<std::uint64_t>(i), 1.0, 0.0));
  }
  publishReport("agv_0", agv0);
  publishReport("agv_1", agv1);

  ASSERT_TRUE(waitForCombinedWithAtLeast(2u * kProbeCount));
  const std::unordered_set<std::uint64_t> ids = collectIds(*last_combined_);
  EXPECT_EQ(ids.size(), 2u * static_cast<std::size_t>(kProbeCount))
      << "agv_0 and agv_1 offset ranges intersect.";
}
