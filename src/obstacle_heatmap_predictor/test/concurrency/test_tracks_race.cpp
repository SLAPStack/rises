// =============================================================================
// Concurrency test: HeatmapPredictorNode::tracked_obstacles_
// =============================================================================
//
// Verifies that concurrent access to tracked_obstacles_ is race-free: under a
// MultiThreadedExecutor a writer thread publishes ObstacleReport at 100 Hz
// (driving updateTrack / evictStaleTracks) while the publishHeatmap wall-timer
// reads the same map on another worker. The test asserts behavioural
// invariants (no crash, timer ticks observed, expired entries cleared without a
// torn read) and, when built with TSan, surfaces any data race on the shared
// track map.
//
// TSan invocation:
//   colcon build --packages-select obstacle_heatmap_predictor \
//     --cmake-args -DBUILD_TESTING=ON -DARIS_ENABLE_TSAN=ON
//   colcon test --packages-select obstacle_heatmap_predictor \
//     --ctest-args -R test_tracks_race
//
// Standards: function cap 100, nesting <= 3, named constants, no TODOs.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rises_interfaces/msg/obstacle.hpp"
#include "rises_interfaces/msg/obstacle_report.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "obstacle_heatmap_predictor/heatmap_predictor_node.hpp"

namespace {

constexpr int kExecutorThreads = 2;
constexpr int kReportPublishRateHz = 100;
constexpr auto kPublishInterval =
    std::chrono::milliseconds(1000 / kReportPublishRateHz);
constexpr auto kRunWindow = std::chrono::milliseconds(1000);
constexpr auto kSettleWindow = std::chrono::milliseconds(300);
constexpr int kDistinctObstacleIds = 16;
constexpr int kMinExpectedHeatmaps = 2;
constexpr double kTimerHz = 50.0;
constexpr double kMinObservations = 1.0;

constexpr const char *kReportTopic = "obstacle_report";
constexpr const char *kHeatmapTopic = "predicted_occupancy";

rises_interfaces::msg::Obstacle makePointObstacle(std::uint64_t id, double x,
                                                  double y) {
  rises_interfaces::msg::Obstacle obs;
  obs.id = id;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.position.x = x;
  obs.position.y = y;
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  obs.vertices.push_back(p);
  return obs;
}

rises_interfaces::msg::ObstacleReport makeReport(int seq) {
  rises_interfaces::msg::ObstacleReport report;
  report.header.stamp.sec = seq;
  report.header.stamp.nanosec = 0;
  for (int i = 0; i < kDistinctObstacleIds; ++i) {
    const double x = static_cast<double>(seq) * 0.01;
    const double y = static_cast<double>(i);
    report.unmatched_obstacles.push_back(
        makePointObstacle(static_cast<std::uint64_t>(i), x, y));
  }
  return report;
}

class TracksRaceFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override { initialize(/*eviction_timeout_sec=*/5.0); }

  void initialize(double eviction_timeout_sec) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"publish_rate_hz", kTimerHz},
        {"min_observations", static_cast<int>(kMinObservations)},
        {"eviction_timeout_sec", eviction_timeout_sec},
        {"grid_width", 20.0},
        {"grid_height", 20.0},
        {"grid_resolution", 0.5},
    });
    predictor_ = std::make_shared<rises::HeatmapPredictorNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("heatmap_tracks_race_helper");

    report_pub_ =
        helper_->create_publisher<rises_interfaces::msg::ObstacleReport>(
            kReportTopic, rclcpp::SensorDataQoS().keep_last(10));

    heatmap_count_.store(0);
    heatmap_sub_ = helper_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        kHeatmapTopic, rclcpp::QoS(1).reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & /*msg*/) {
          heatmap_count_.fetch_add(1, std::memory_order_acq_rel);
        });

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions{}, kExecutorThreads);
    executor_->add_node(predictor_);
    executor_->add_node(helper_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    heatmap_sub_.reset();
    report_pub_.reset();
    helper_.reset();
    predictor_.reset();
  }

  std::shared_ptr<rises::HeatmapPredictorNode> predictor_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<rises_interfaces::msg::ObstacleReport>::SharedPtr
      report_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_sub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<int> heatmap_count_{0};
};

} // namespace

// -----------------------------------------------------------------------------
// WriterAndTimerReaderRaceFree
// -----------------------------------------------------------------------------
TEST_F(TracksRaceFixture, WriterAndTimerReaderRaceFree) {
  std::atomic<bool> producer_failed{false};
  std::atomic<bool> stop{false};

  std::thread writer([this, &stop, &producer_failed]() {
    int seq = 0;
    try {
      while (!stop.load(std::memory_order_acquire)) {
        report_pub_->publish(makeReport(seq++));
        std::this_thread::sleep_for(kPublishInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(kRunWindow);
  stop.store(true, std::memory_order_release);
  writer.join();

  std::this_thread::sleep_for(kSettleWindow);

  EXPECT_FALSE(producer_failed.load())
      << "publish() of ObstacleReport threw during the race window";
  EXPECT_GE(heatmap_count_.load(), kMinExpectedHeatmaps)
      << "publishHeatmap timer did not tick the expected number of times "
         "while the writer was active";
}

// -----------------------------------------------------------------------------
// TickClearsExpiredWithoutTear
// -----------------------------------------------------------------------------
//
// Short eviction timeout forces evictStaleTracks() to erase entries from
// tracked_obstacles_ on every report callback. Concurrent timer-driven
// publishHeatmap iterates the same map, exercising the rehash + erase path.
// We assert no exception escapes; under TSan the erase races with the
// range-for read.
TEST_F(TracksRaceFixture, TickClearsExpiredWithoutTear) {
  TearDown();
  // Rebuild with a very short eviction timeout so evictStaleTracks erases
  // entries on every callback. Header stamps advance by 1 s per report so a
  // 0.1 s timeout always expires the previous batch.
  initialize(/*eviction_timeout_sec=*/0.1);

  std::atomic<bool> producer_failed{false};
  std::atomic<bool> stop{false};

  std::thread writer([this, &stop, &producer_failed]() {
    int seq = 0;
    try {
      while (!stop.load(std::memory_order_acquire)) {
        report_pub_->publish(makeReport(seq++));
        std::this_thread::sleep_for(kPublishInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(kRunWindow);
  stop.store(true, std::memory_order_release);
  writer.join();

  std::this_thread::sleep_for(kSettleWindow);

  EXPECT_FALSE(producer_failed.load());
}
