/// @file test_track_decay.cpp
/// @brief Single-threaded algorithm correctness tests for HeatmapPredictorNode
///        track lifecycle: ingest, velocity estimation, linear extrapolation,
///        eviction, and confidence decay along the prediction horizon.
///
/// These tests exercise the public ROS surface only (subscribed topic
/// `obstacle_report` -> published topic `predicted_occupancy`). The internal
/// `tracked_obstacles_` map is private and has no accessor, so behaviour is
/// verified by inspecting the published OccupancyGrid.
///
/// Determinism notes:
///   * Track ingest timestamps come from `msg->header.stamp` and are fully
///     test-controlled. No wall-clock sleeps are used for ingest timing.
///   * The publish path is driven by a `create_wall_timer`. Tests set
///     `publish_rate_hz` high (100 Hz) and `spin_until` a message arrives,
///     bounded by a deadline (<= 500 ms). No real-time decay logic depends on
///     wall time.
///
/// Phase 2 scope: single-threaded algorithm correctness. The Phase 3 work on
/// `tracked_obstacles_` data races between callback and timer is out of scope.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_report.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "obstacle_heatmap_predictor/heatmap_predictor_node.hpp"

namespace {

constexpr double kGridResolution = 0.1; // metres per cell
constexpr double kGridWidth = 10.0;     // metres
constexpr double kGridHeight = 10.0;    // metres
constexpr double kGridCenter = 5.0;     // origin at (0, 0)
constexpr double kPublishRateHz = 100.0;
constexpr int kPublishWaitMs = 500;
constexpr int kSpinSliceMs = 5;

/// @brief Builds an obstacle with the position field populated (the predictor
/// reads `obstacle.position.x/y`, not vertices).
rises_interfaces::msg::Obstacle makePositionObstacle(std::uint64_t id, double x,
                                                     double y) {
  rises_interfaces::msg::Obstacle obs;
  obs.id = id;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.position.x = x;
  obs.position.y = y;
  return obs;
}

/// @brief Builds an ObstacleReport with a single unmatched obstacle at the
/// supplied header timestamp (seconds since the epoch).
rises_interfaces::msg::ObstacleReport
makeReport(double timestamp_sec,
           const std::vector<rises_interfaces::msg::Obstacle> &obstacles) {
  rises_interfaces::msg::ObstacleReport msg;
  const auto secs = static_cast<std::int32_t>(timestamp_sec);
  const auto nsecs = static_cast<std::uint32_t>(
      (timestamp_sec - static_cast<double>(secs)) * 1.0e9);
  msg.header.stamp.sec = secs;
  msg.header.stamp.nanosec = nsecs;
  msg.header.frame_id = "map";
  msg.unmatched_obstacles = obstacles;
  return msg;
}

class TrackDecayTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"observation_window_sec", 10.0},
        {"prediction_horizon_sec", 4.0},
        {"prediction_step_sec", 1.0},
        {"eviction_timeout_sec", 5.0},
        {"min_observations", 1},
        {"grid_resolution", kGridResolution},
        {"grid_width", kGridWidth},
        {"grid_height", kGridHeight},
        {"grid_center_x", kGridCenter},
        {"grid_center_y", kGridCenter},
        {"frame_id", std::string("map")},
        {"gaussian_sigma", kGridResolution}, // tight blob (~1 cell radius)
        {"publish_rate_hz", kPublishRateHz},
    });

    node_ = std::make_shared<rises::HeatmapPredictorNode>(options);
    helper_node_ = std::make_shared<rclcpp::Node>("track_decay_helper");

    publisher_ =
        helper_node_->create_publisher<rises_interfaces::msg::ObstacleReport>(
            "obstacle_report", rclcpp::SensorDataQoS().keep_last(10));

    latest_grid_.reset();
    subscriber_ =
        helper_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "predicted_occupancy", rclcpp::QoS(1).reliable(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
              latest_grid_ = *msg;
            });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(helper_node_);
  }

  void TearDown() override {
    executor_->cancel();
    executor_.reset();
    subscriber_.reset();
    publisher_.reset();
    helper_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void publishReport(const rises_interfaces::msg::ObstacleReport &msg) {
    publisher_->publish(msg);
  }

  /// @brief Spins until a fresh OccupancyGrid arrives or the deadline expires.
  /// Returns true when a message was captured.
  bool spinUntilGrid() {
    latest_grid_.reset();
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kPublishWaitMs);
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (latest_grid_.has_value()) {
        return true;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(kSpinSliceMs));
    }
    return false;
  }

  static std::size_t worldToCellIndex(double x, double y, std::uint32_t cols) {
    const int col = static_cast<int>(std::round(x / kGridResolution));
    const int row = static_cast<int>(std::round(y / kGridResolution));
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) +
           static_cast<std::size_t>(col);
  }

  std::shared_ptr<rises::HeatmapPredictorNode> node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Publisher<rises_interfaces::msg::ObstacleReport>::SharedPtr
      publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_grid_;
};

TEST_F(TrackDecayTest, TrackAddedOnObstacle) {
  // Publish one obstacle. With min_observations=1 the predictor must stamp
  // it onto the grid, so at least one cell becomes non-zero.
  auto report = makeReport(1.0, {makePositionObstacle(42, 5.0, 5.0)});
  publishReport(report);

  ASSERT_TRUE(spinUntilGrid());
  const auto &grid = latest_grid_.value();
  ASSERT_FALSE(grid.data.empty());

  bool any_nonzero = false;
  for (std::int8_t cell : grid.data) {
    if (cell > 0) {
      any_nonzero = true;
      break;
    }
  }
  EXPECT_TRUE(any_nonzero)
      << "Expected at least one populated cell after one obstacle report";
}

TEST_F(TrackDecayTest, TrackDecaysOverTime) {
  GTEST_SKIP() << "Production code does not decay per-track confidence over "
                  "wall-time; the confidence variable only attenuates the "
                  "weight of future prediction steps within a single publish "
                  "call (see heatmap_predictor_node.cpp:209). The "
                  "PredictedPositionMatchesLinearExtrapolation test covers "
                  "the actual step-wise decay. A wall-time decay test "
                  "requires a production-side change first.";
}

TEST_F(TrackDecayTest, VelocityEstimatedFromTwoFrames) {
  // Two frames 1.0 s apart with the same obstacle id at different positions.
  // Expected velocity = (1.0 m, 0.0 m) / 1.0 s = (1.0, 0.0) m/s.
  const std::uint64_t id = 7;
  publishReport(makeReport(0.0, {makePositionObstacle(id, 5.0, 5.0)}));
  ASSERT_TRUE(spinUntilGrid());

  publishReport(makeReport(1.0, {makePositionObstacle(id, 6.0, 5.0)}));
  ASSERT_TRUE(spinUntilGrid());

  // We cannot read velocity directly. With horizon=4.0 s, step=1.0 s and
  // sigma==resolution (~1-cell blob), the projection points land at
  // (7, 5), (8, 5), (9, 5), (current 6, 5). We expect non-zero peaks at
  // those projected x-coordinates if velocity ~= (1, 0).
  const auto &grid = latest_grid_.value();
  const auto cols = grid.info.width;

  const std::size_t idx_now = worldToCellIndex(6.0, 5.0, cols);
  const std::size_t idx_step1 = worldToCellIndex(7.0, 5.0, cols);
  const std::size_t idx_step2 = worldToCellIndex(8.0, 5.0, cols);

  ASSERT_LT(idx_step2, grid.data.size());

  EXPECT_GT(grid.data[idx_now], 0);
  EXPECT_GT(grid.data[idx_step1], 0)
      << "Projection at v*1s should land near (7, 5)";
  EXPECT_GT(grid.data[idx_step2], 0)
      << "Projection at v*2s should land near (8, 5)";
}

TEST_F(TrackDecayTest, PredictedPositionMatchesLinearExtrapolation) {
  // Build a track with vx=1 m/s. After two frames spanning dt=1 s, the next
  // projection step (dt=1 s) lands 1 m further in +x; the cell at +0.5 m
  // (mid-projection sigma=resolution Gaussian shoulder) must register below
  // the current-position cell because confidence decays.
  const std::uint64_t id = 11;
  publishReport(makeReport(0.0, {makePositionObstacle(id, 4.0, 5.0)}));
  ASSERT_TRUE(spinUntilGrid());
  publishReport(makeReport(1.0, {makePositionObstacle(id, 5.0, 5.0)}));
  ASSERT_TRUE(spinUntilGrid());

  const auto &grid = latest_grid_.value();
  const auto cols = grid.info.width;

  const std::size_t idx_now = worldToCellIndex(5.0, 5.0, cols);
  const std::size_t idx_future = worldToCellIndex(6.0, 5.0, cols);
  const std::size_t idx_late = worldToCellIndex(8.0, 5.0, cols);

  ASSERT_LT(idx_late, grid.data.size());

  // Current position is stamped with full weight, then normalised to 100.
  EXPECT_EQ(grid.data[idx_now], 100);

  // 1-step-ahead projection: confidence = 1 - 1/4 = 0.75 of full weight.
  EXPECT_GT(grid.data[idx_future], 0);
  EXPECT_LT(grid.data[idx_future], grid.data[idx_now]);

  // Late projection (3 steps ahead): confidence = 1 - 3/4 = 0.25.
  EXPECT_GT(grid.data[idx_late], 0);
  EXPECT_LT(grid.data[idx_late], grid.data[idx_future]);
}

TEST_F(TrackDecayTest, EmptyInputProducesEmptyHeatmap) {
  // No reports published; the wall timer still fires and emits an
  // all-zero grid because there are no tracked obstacles.
  ASSERT_TRUE(spinUntilGrid());
  const auto &grid = latest_grid_.value();
  ASSERT_FALSE(grid.data.empty());

  for (std::int8_t cell : grid.data) {
    ASSERT_EQ(cell, 0) << "Empty input must produce an all-zero heatmap";
  }
}

TEST_F(TrackDecayTest, StaleTrackEvicted) {
  // Eviction runs inside obstacleReportCallback against the latest msg
  // timestamp. We seed track 1 at t=0, then send track 2 at t=20 (>>
  // eviction_timeout=5). Track 1 must be evicted before publishHeatmap
  // stamps anything, so the only non-zero cell should be at track 2's
  // position.
  publishReport(makeReport(0.0, {makePositionObstacle(1, 2.0, 2.0)}));
  ASSERT_TRUE(spinUntilGrid());

  publishReport(makeReport(20.0, {makePositionObstacle(2, 8.0, 8.0)}));
  ASSERT_TRUE(spinUntilGrid());

  const auto &grid = latest_grid_.value();
  const auto cols = grid.info.width;

  const std::size_t idx_stale = worldToCellIndex(2.0, 2.0, cols);
  const std::size_t idx_fresh = worldToCellIndex(8.0, 8.0, cols);
  ASSERT_LT(idx_fresh, grid.data.size());

  EXPECT_EQ(grid.data[idx_stale], 0)
      << "Evicted track must not contribute to the heatmap";
  EXPECT_GT(grid.data[idx_fresh], 0)
      << "Fresh track must be stamped after eviction sweep";
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
