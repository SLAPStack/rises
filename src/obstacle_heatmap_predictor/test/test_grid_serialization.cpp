/// @file test_grid_serialization.cpp
/// @brief Tests for HeatmapPredictorNode grid layout / serialization:
///        resolution-to-cell mapping, out-of-bounds handling, configured grid
///        dimensions, frame_id propagation, and world<->cell coordinate
///        sanity.
///
/// All tests are single-threaded and read only the published OccupancyGrid
/// (the production code exposes no introspection accessors). Time is driven
/// by `header.stamp` on the published `obstacle_report`; the publish wall
/// timer fires at a deterministic high rate, and the executor is bounded by a
/// 500 ms deadline.
///
/// Out-of-bounds behaviour (see heatmap_predictor_node.cpp:267-271): the
/// `stampGaussian` kernel clamps the row/column iteration range to
/// `[0, grid_rows_-1]` / `[0, grid_cols_-1]`. When the centre lies far
/// outside the grid the clamped range becomes empty, so the write is
/// silently skipped (neither clamped-to-edge nor an out-of-bounds write).
/// The OutOfBoundsTrackClamped test asserts that "skipped" behaviour.

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

constexpr double kPublishRateHz = 100.0;
constexpr int kPublishWaitMs = 500;
constexpr int kSpinSliceMs = 5;

rises_interfaces::msg::Obstacle makePositionObstacle(std::uint64_t id, double x,
                                                     double y) {
  rises_interfaces::msg::Obstacle obs;
  obs.id = id;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.position.x = x;
  obs.position.y = y;
  return obs;
}

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

struct GridConfig {
  double resolution;
  double width;
  double height;
  double center_x;
  double center_y;
  double sigma;
  std::string frame_id;
};

class GridSerializationTest : public ::testing::Test {
protected:
  void initFixture(const GridConfig &cfg) {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"observation_window_sec", 10.0},
        {"prediction_horizon_sec", 1.0},
        {"prediction_step_sec", 1.0},
        {"eviction_timeout_sec", 5.0},
        {"min_observations", 1},
        {"grid_resolution", cfg.resolution},
        {"grid_width", cfg.width},
        {"grid_height", cfg.height},
        {"grid_center_x", cfg.center_x},
        {"grid_center_y", cfg.center_y},
        {"frame_id", cfg.frame_id},
        {"gaussian_sigma", cfg.sigma},
        {"publish_rate_hz", kPublishRateHz},
    });

    node_ = std::make_shared<rises::HeatmapPredictorNode>(options);
    helper_node_ = std::make_shared<rclcpp::Node>("grid_serialization_helper");

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
    if (executor_) {
      executor_->cancel();
    }
    executor_.reset();
    subscriber_.reset();
    publisher_.reset();
    helper_node_.reset();
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

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

  std::shared_ptr<rises::HeatmapPredictorNode> node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Publisher<rises_interfaces::msg::ObstacleReport>::SharedPtr
      publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_grid_;
};

TEST_F(GridSerializationTest, GridResolutionRespected) {
  // resolution=0.1, width=height=10, center=5 -> origin at (0, 0), 100x100.
  // Use a tight Gaussian (sigma = resolution) so the peak cell is the
  // centre cell of the stamp.
  GridConfig cfg{0.1, 10.0, 10.0, 5.0, 5.0, 0.1, "map"};
  initFixture(cfg);

  publisher_->publish(makeReport(0.0, {makePositionObstacle(1, 0.5, 0.5)}));
  ASSERT_TRUE(spinUntilGrid());

  const auto &grid = latest_grid_.value();
  ASSERT_EQ(grid.info.width, 100u);
  ASSERT_EQ(grid.info.height, 100u);

  // Find the strongest cell; it should be at (col=5, row=5).
  std::size_t peak_index = 0;
  std::int8_t peak_value = 0;
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] > peak_value) {
      peak_value = grid.data[i];
      peak_index = i;
    }
  }
  ASSERT_GT(peak_value, 0);
  const auto col = peak_index % grid.info.width;
  const auto row = peak_index / grid.info.width;
  EXPECT_EQ(col, 5u);
  EXPECT_EQ(row, 5u);
}

TEST_F(GridSerializationTest, OutOfBoundsTrackClamped) {
  // 100x100 grid covering world [0, 10]x[0, 10]. Place obstacle at
  // (1000, 1000): the kernel iteration range becomes empty (col_start >
  // col_end, row_start > row_end), so nothing is written. Production
  // behaviour: skipped, not clamped-to-edge.
  GridConfig cfg{0.1, 10.0, 10.0, 5.0, 5.0, 0.1, "map"};
  initFixture(cfg);

  publisher_->publish(
      makeReport(0.0, {makePositionObstacle(1, 1000.0, 1000.0)}));
  ASSERT_TRUE(spinUntilGrid());

  const auto &grid = latest_grid_.value();
  ASSERT_EQ(grid.info.width, 100u);
  ASSERT_EQ(grid.info.height, 100u);
  ASSERT_EQ(grid.data.size(), 100u * 100u);

  for (std::int8_t cell : grid.data) {
    ASSERT_EQ(cell, 0)
        << "OOB obstacle must not produce any cell write (clamped to "
           "empty range, no edge clamp).";
  }
}

TEST_F(GridSerializationTest, GridSizeBoundedByConfig) {
  // 64x64 grid: width=height=6.4, resolution=0.1 -> grid_cols_=grid_rows_=64.
  GridConfig cfg{0.1, 6.4, 6.4, 3.2, 3.2, 0.1, "map"};
  initFixture(cfg);

  ASSERT_TRUE(spinUntilGrid());
  const auto &grid = latest_grid_.value();
  EXPECT_EQ(grid.info.width, 64u);
  EXPECT_EQ(grid.info.height, 64u);
  EXPECT_EQ(grid.data.size(), 64u * 64u);
}

TEST_F(GridSerializationTest, HeaderFrameIdSetFromParameter) {
  GridConfig cfg{0.1, 10.0, 10.0, 5.0, 5.0, 0.1, "map_custom"};
  initFixture(cfg);

  ASSERT_TRUE(spinUntilGrid());
  EXPECT_EQ(latest_grid_.value().header.frame_id, "map_custom");
}

TEST_F(GridSerializationTest, ResolutionInMetersPerCell) {
  // Sanity check: with origin (0, 0) and resolution 0.1, an obstacle at
  // world (10*res, 10*res) = (1.0, 1.0) lands at cell (col=10, row=10).
  GridConfig cfg{0.1, 10.0, 10.0, 5.0, 5.0, 0.1, "map"};
  initFixture(cfg);

  publisher_->publish(makeReport(0.0, {makePositionObstacle(1, 1.0, 1.0)}));
  ASSERT_TRUE(spinUntilGrid());

  const auto &grid = latest_grid_.value();
  ASSERT_EQ(grid.info.width, 100u);
  EXPECT_FLOAT_EQ(grid.info.resolution, 0.1f);

  std::size_t peak_index = 0;
  std::int8_t peak_value = 0;
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] > peak_value) {
      peak_value = grid.data[i];
      peak_index = i;
    }
  }
  ASSERT_GT(peak_value, 0);

  const auto col = peak_index % grid.info.width;
  const auto row = peak_index / grid.info.width;
  EXPECT_EQ(col, 10u);
  EXPECT_EQ(row, 10u);

  // Origin should match the configured center - half-extent.
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, 0.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 0.0);
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
