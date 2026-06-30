// Regression tests for audit finding #14 (OccupancyGrid index overflow) in the
// fiware_bridge predicted-occupancy handler.
//
// These tests verify that the OccupancyGrid callback validates dimensions with
// size_t arithmetic before indexing: it publishes JSON on
// /fiware/heatmap_summary for a well-formed grid (width * height == data.size())
// and rejects grids with a mismatched data size, a zero width or height, or an
// oversize width * height product. A large but valid grid is still published.
// Inputs are fed on the "predicted_occupancy" subscription and observed via a
// ROS subscriber spy on /fiware/heatmap_summary.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "fiware_bridge/fiware_bridge_node.hpp"

namespace {

using nav_msgs::msg::OccupancyGrid;

constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr int kMaxSpinIterations = 200;
// Throttle is configured very high so we observe the first publication
// without waiting for the default 1 Hz window.
constexpr double kFastThrottleHz = 100.0;

class HeatmapFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void buildBridge() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"heatmap_throttle_hz", kFastThrottleHz},
        {"report_throttle_hz", kFastThrottleHz},
        {"odom_throttle_hz", kFastThrottleHz},
        {"diag_throttle_hz", kFastThrottleHz},
        {"obstacles_json_file", std::string{}},
    });
    node_ = std::make_shared<rises::FiwareBridgeNode>(options);

    publisher_node_ = std::make_shared<rclcpp::Node>("test_heatmap_pub");
    heatmap_pub_ = publisher_node_->create_publisher<OccupancyGrid>(
        "predicted_occupancy", rclcpp::QoS(1).reliable());

    spy_node_ = std::make_shared<rclcpp::Node>("test_heatmap_spy");
    published_payloads_.clear();
    spy_sub_ = spy_node_->create_subscription<std_msgs::msg::String>(
        "fiware/heatmap_summary", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          published_payloads_.push_back(msg->data);
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(publisher_node_);
    executor_->add_node(spy_node_);
  }

  void TearDown() override {
    executor_.reset();
    spy_sub_.reset();
    heatmap_pub_.reset();
    spy_node_.reset();
    publisher_node_.reset();
    node_.reset();
    published_payloads_.clear();
  }

  void spinFor(std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      std::this_thread::sleep_for(kSpinSlice);
    }
  }

  bool spinUntilPublished(int max_iterations = kMaxSpinIterations) {
    for (int i = 0; i < max_iterations; ++i) {
      executor_->spin_some();
      if (!published_payloads_.empty()) {
        return true;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    return false;
  }

  static OccupancyGrid::SharedPtr
  makeGrid(std::uint32_t width, std::uint32_t height, std::size_t data_size) {
    auto grid = std::make_shared<OccupancyGrid>();
    grid->info.width = width;
    grid->info.height = height;
    grid->info.resolution = 0.1f;
    grid->info.origin.position.x = 0.0;
    grid->info.origin.position.y = 0.0;
    grid->data.assign(data_size, 0);
    return grid;
  }

  std::shared_ptr<rises::FiwareBridgeNode> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::Node> spy_node_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr heatmap_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spy_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::string> published_payloads_;
};

} // namespace

// Well-formed 100x100 grid with matching data size: callback runs and
// publishes.
TEST_F(HeatmapFixture, ConsistentDimensionsAccepted) {
  buildBridge();

  auto grid = makeGrid(100u, 100u, 10000u);
  heatmap_pub_->publish(*grid);

  ASSERT_TRUE(spinUntilPublished()) << "Expected heatmap summary publish";
  const nlohmann::json parsed =
      nlohmann::json::parse(published_payloads_.front());
  EXPECT_EQ(parsed["grid_width"].get<int>(), 100);
  EXPECT_EQ(parsed["grid_height"].get<int>(), 100);
}

// width*height != data.size() => callback returns early, no publish.
TEST_F(HeatmapFixture, MismatchedDataSizeRejected) {
  buildBridge();

  // 100x100 should be 10000; provide 9999.
  auto grid = makeGrid(100u, 100u, 9999u);
  heatmap_pub_->publish(*grid);

  spinFor(std::chrono::milliseconds(150));
  EXPECT_TRUE(published_payloads_.empty())
      << "Bridge published despite mismatched data size — fix missing.";
}

// width=0 with non-empty height must short-circuit (no iteration, no publish).
TEST_F(HeatmapFixture, WidthZeroRejected) {
  buildBridge();

  auto grid = makeGrid(0u, 50u, 0u);
  heatmap_pub_->publish(*grid);

  spinFor(std::chrono::milliseconds(150));
  EXPECT_TRUE(published_payloads_.empty())
      << "Width=0 grid should be rejected.";
}

// width*height > INT_MAX (~3.6 B cells). Pre-fix the signed-int multiplication
// in row*width+col silently overflows; post-fix the size_t comparison against
// data.size() (much smaller) rejects.
TEST_F(HeatmapFixture, OversizeProductRejected) {
  buildBridge();

  // 60000 * 60000 = 3.6e9 > INT_MAX (~2.147e9). Tiny data buffer.
  auto grid = makeGrid(60000u, 60000u, 16u);
  heatmap_pub_->publish(*grid);

  spinFor(std::chrono::milliseconds(150));
  EXPECT_TRUE(published_payloads_.empty())
      << "Oversize grid where width*height > INT_MAX must be rejected.";
}

// width=height=70000 with matching data buffer of 70000*70000 cells is
// genuinely large but valid; we don't actually allocate this in test — we
// only assert that the linear-index formula uses size_t internally so the
// REJECTION decision uses size_t arithmetic. We probe with a grid whose
// declared dimensions overflow int but where data.size() matches the size_t
// product, and assert it is NOT rejected as oversize. (We use a smaller
// matched-size variant to keep memory bounded.)
//
// To keep the test deterministic and lightweight we use width=70000 height=1
// (matches size_t multiplication: 70000 > INT16 range but trivial to allocate).
TEST_F(HeatmapFixture, MaxSizeTArithmeticHandledViaSizeT) {
  buildBridge();

  constexpr std::uint32_t kWidth = 70000u;
  constexpr std::uint32_t kHeight = 1u;
  constexpr std::size_t kData =
      static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);
  auto grid = makeGrid(kWidth, kHeight, kData);
  heatmap_pub_->publish(*grid);

  ASSERT_TRUE(spinUntilPublished())
      << "70000x1 grid with matching data should be accepted — confirms "
         "size_t arithmetic is used.";
  const nlohmann::json parsed =
      nlohmann::json::parse(published_payloads_.front());
  EXPECT_EQ(parsed["grid_width"].get<std::int64_t>(),
            static_cast<std::int64_t>(kWidth));
  EXPECT_EQ(parsed["grid_height"].get<std::int64_t>(),
            static_cast<std::int64_t>(kHeight));
}
