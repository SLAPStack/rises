// Unit tests for FiwareBridgeNode message-to-JSON serialization.
//
// The bridge is a pure ROS-topic translator: it consumes typed messages and
// republishes std_msgs/String JSON on fiware/* topics for the DDS Enabler.
// There is no HTTP / MQTT path to mock — we just feed input on the bridge's
// subscriptions and observe the JSON string emitted on the matching fiware/*
// output topic via a spy subscriber.
//
// Scope:
//   - Golden-JSON parity for ObstacleReport (obstacle_summary).
//   - Path-to-Orion-LD is documented below as GTEST_SKIP — the production
//     bridge does NOT subscribe to nav_msgs/Path. Listed as a future-work
//     seam in the spec.
//   - Heatmap hot-cell cap (200, see fiware_bridge_node.cpp:349).
//   - Empty heatmap emits "hot_cells":[] (empty array, not null / missing).
//   - OccupancyGrid resolution is forwarded as-is in the heatmap summary.
//   - Contours: outer hull + N inner polygons forwarded with matching shapes.
//
// Throttling for each input topic is dialled up to a high frequency in the
// fixture so we observe the first publish without artificial latency.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rises_interfaces/msg/contours.hpp>
#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "fiware_bridge/fiware_bridge_node.hpp"

namespace {

using rises_interfaces::msg::Contours;
using rises_interfaces::msg::LineSegment;
using rises_interfaces::msg::Obstacle;
using rises_interfaces::msg::ObstacleReport;
using nav_msgs::msg::OccupancyGrid;

constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr int kMaxSpinIterations = 200;
// Throttle is dialled up so the first message is always emitted without
// waiting for the default 1 Hz window.
constexpr double kFastThrottleHz = 100.0;
// Production cap (see fiware_bridge_node.cpp: constexpr max_hot_cells = 200).
constexpr std::size_t kProductionHotCellCap = 200u;

class BridgeFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void buildBridge() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"report_throttle_hz", kFastThrottleHz},
        {"odom_throttle_hz", kFastThrottleHz},
        {"diag_throttle_hz", kFastThrottleHz},
        {"heatmap_throttle_hz", kFastThrottleHz},
        {"obstacles_json_file", std::string{}},
    });
    node_ = std::make_shared<rises::FiwareBridgeNode>(options);
    publisher_node_ = std::make_shared<rclcpp::Node>("test_serial_pub");
    spy_node_ = std::make_shared<rclcpp::Node>("test_serial_spy");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(publisher_node_);
    executor_->add_node(spy_node_);
  }

  void TearDown() override {
    executor_.reset();
    spy_subs_.clear();
    publishers_.clear();
    captured_.clear();
    spy_node_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr
  makePublisher(const std::string &topic, const rclcpp::QoS &qos) {
    auto pub = publisher_node_->create_publisher<MsgT>(topic, qos);
    publishers_.push_back(pub);
    return pub;
  }

  std::vector<std::string> &subscribeSpy(const std::string &topic,
                                         const rclcpp::QoS &qos) {
    auto &slot = captured_[topic];
    auto sub = spy_node_->create_subscription<std_msgs::msg::String>(
        topic, qos, [&slot](const std_msgs::msg::String::SharedPtr msg) {
          slot.push_back(msg->data);
        });
    spy_subs_.push_back(sub);
    return slot;
  }

  bool spinUntil(const std::vector<std::string> &sink,
                 int max_iterations = kMaxSpinIterations) {
    for (int i = 0; i < max_iterations; ++i) {
      executor_->spin_some();
      if (!sink.empty()) {
        return true;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    return false;
  }

  void spinFor(std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      std::this_thread::sleep_for(kSpinSlice);
    }
  }

  std::shared_ptr<rises::FiwareBridgeNode> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::Node> spy_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> spy_subs_;
  std::unordered_map<std::string, std::vector<std::string>> captured_;
};

OccupancyGrid::SharedPtr makeGrid(std::uint32_t width, std::uint32_t height,
                                  float resolution, std::int8_t fill_value) {
  auto grid = std::make_shared<OccupancyGrid>();
  grid->info.width = width;
  grid->info.height = height;
  grid->info.resolution = resolution;
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  const std::size_t cells =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  grid->data.assign(cells, fill_value);
  return grid;
}

} // namespace

// ----------------------------------------------------------------------------
// ObstacleReport -> fiware/obstacle_summary golden JSON
// ----------------------------------------------------------------------------
TEST_F(BridgeFixture, ObstacleReportToOrionLdMatchesGoldenJson) {
  buildBridge();

  auto pub = makePublisher<ObstacleReport>(
      "obstacle_report", rclcpp::SensorDataQoS().keep_last(10));
  auto &sink =
      subscribeSpy("fiware/obstacle_summary", rclcpp::QoS(1).reliable());

  ObstacleReport report;
  Obstacle matched;
  matched.id = 7;
  matched.position.x = 1.0;
  matched.position.y = 2.0;
  geometry_msgs::msg::Point v1;
  v1.x = 1.0;
  v1.y = 2.0;
  geometry_msgs::msg::Point v2;
  v2.x = 3.0;
  v2.y = 4.0;
  matched.vertices = {v1, v2};
  report.matched_obstacles = {matched};

  Obstacle unmatched;
  unmatched.id = 42;
  unmatched.position.x = 5.0;
  unmatched.position.y = 6.0;
  geometry_msgs::msg::Point u1;
  u1.x = 5.0;
  u1.y = 6.0;
  geometry_msgs::msg::Point u2;
  u2.x = 7.0;
  u2.y = 8.0;
  unmatched.vertices = {u1, u2};
  report.unmatched_obstacles = {unmatched};

  pub->publish(report);

  ASSERT_TRUE(spinUntil(sink)) << "Expected obstacle_summary publish";

  const nlohmann::json got = nlohmann::json::parse(sink.front());
  const nlohmann::json golden = nlohmann::json::parse(R"({
    "matched_count": 1,
    "unmatched_count": 1,
    "matched_segments": [{"x1": 1.0, "y1": 2.0, "x2": 3.0, "y2": 4.0}],
    "unmatched_segments": [{"x1": 5.0, "y1": 6.0, "x2": 7.0, "y2": 8.0}],
    "unmatched_ids": [42]
  })");

  EXPECT_EQ(got["matched_count"], golden["matched_count"]);
  EXPECT_EQ(got["unmatched_count"], golden["unmatched_count"]);
  EXPECT_EQ(got["matched_segments"], golden["matched_segments"]);
  EXPECT_EQ(got["unmatched_segments"], golden["unmatched_segments"]);
  EXPECT_EQ(got["unmatched_ids"], golden["unmatched_ids"]);
}

// ----------------------------------------------------------------------------
// Path -> Orion-LD: production bridge does NOT consume nav_msgs/Path.
// Documented as future-work seam so the gap is visible in the test report.
// ----------------------------------------------------------------------------
TEST_F(BridgeFixture, PathToOrionLdMatchesGoldenJson) {
  GTEST_SKIP() << "FiwareBridgeNode has no nav_msgs/Path subscription as of "
                  "fiware_bridge_node.hpp r0.1.0. The bridge translates "
                  "ObstacleReport / Contours / OccupancyGrid / Odometry / "
                  "DiagnosticArray only. If a /path translation is added in "
                  "the future, wire a golden JSON assertion here.";
}

// ----------------------------------------------------------------------------
// Heatmap hot-cell cap = 200 (see fiware_bridge_node.cpp:349
// `constexpr std::size_t max_hot_cells = 200;`).
// We publish a 1000-cell grid where every cell is above the threshold and
// assert at most kProductionHotCellCap entries land in the JSON array.
// ----------------------------------------------------------------------------
TEST_F(BridgeFixture, HeatmapTo200HotCellCap) {
  buildBridge();

  auto pub = makePublisher<OccupancyGrid>("predicted_occupancy",
                                          rclcpp::QoS(1).reliable());
  auto &sink =
      subscribeSpy("fiware/heatmap_summary", rclcpp::QoS(1).reliable());

  // 50x20 = 1000 cells, all at value 50 (well above threshold of 10).
  auto grid = makeGrid(50u, 20u, 0.1f, /*fill_value=*/50);
  pub->publish(*grid);

  ASSERT_TRUE(spinUntil(sink)) << "Expected heatmap_summary publish";

  const nlohmann::json parsed = nlohmann::json::parse(sink.front());
  ASSERT_TRUE(parsed.contains("hot_cells"));
  ASSERT_TRUE(parsed["hot_cells"].is_array());
  EXPECT_LE(parsed["hot_cells"].size(), kProductionHotCellCap)
      << "hot_cells must be capped to at most " << kProductionHotCellCap
      << " entries to keep Orion-LD payload bounded.";
  // Production also reports the full nonzero count regardless of cap.
  EXPECT_EQ(parsed["nonzero_cells"].get<int>(), 1000);
}

// ----------------------------------------------------------------------------
// Heatmap with zero hot cells emits "hot_cells":[] — empty array, never null
// or missing — so downstream consumers can rely on the shape.
// ----------------------------------------------------------------------------
TEST_F(BridgeFixture, EmptyHeatmapEmitsEmptyArray) {
  buildBridge();

  auto pub = makePublisher<OccupancyGrid>("predicted_occupancy",
                                          rclcpp::QoS(1).reliable());
  auto &sink =
      subscribeSpy("fiware/heatmap_summary", rclcpp::QoS(1).reliable());

  // 10x10, all zero -> no cell at/above threshold of 10.
  auto grid = makeGrid(10u, 10u, 0.1f, /*fill_value=*/0);
  pub->publish(*grid);

  ASSERT_TRUE(spinUntil(sink)) << "Expected heatmap_summary publish";

  const nlohmann::json parsed = nlohmann::json::parse(sink.front());
  ASSERT_TRUE(parsed.contains("hot_cells"));
  ASSERT_TRUE(parsed["hot_cells"].is_array());
  EXPECT_EQ(parsed["hot_cells"].size(), 0u);
  EXPECT_EQ(parsed["nonzero_cells"].get<int>(), 0);
}

// ----------------------------------------------------------------------------
// OccupancyGrid resolution must round-trip into the JSON unchanged.
// ----------------------------------------------------------------------------
TEST_F(BridgeFixture, OccupancyGridResolutionForwarded) {
  buildBridge();

  auto pub = makePublisher<OccupancyGrid>("predicted_occupancy",
                                          rclcpp::QoS(1).reliable());
  auto &sink =
      subscribeSpy("fiware/heatmap_summary", rclcpp::QoS(1).reliable());

  constexpr float kResolution = 0.05f;
  auto grid = makeGrid(4u, 4u, kResolution, /*fill_value=*/0);
  pub->publish(*grid);

  ASSERT_TRUE(spinUntil(sink)) << "Expected heatmap_summary publish";

  const nlohmann::json parsed = nlohmann::json::parse(sink.front());
  ASSERT_TRUE(parsed.contains("resolution"));
  EXPECT_NEAR(parsed["resolution"].get<double>(),
              static_cast<double>(kResolution), 1e-6);
}

// ----------------------------------------------------------------------------
// Contours: outer hull (one polygon) + 2 inner polygons must each appear with
// the correct array shape in the forwarded JSON.
// ----------------------------------------------------------------------------
TEST_F(BridgeFixture, ContoursForwardedAsArrays) {
  buildBridge();

  auto pub = makePublisher<Contours>(
      "warehouse_contours", rclcpp::QoS(1).reliable().transient_local());
  auto &sink = subscribeSpy("fiware/warehouse_geometry",
                            rclcpp::QoS(1).reliable().transient_local());

  Contours contours;

  // Outer hull: 4 points (a square).
  for (int i = 0; i < 4; ++i) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(i);
    p.y = static_cast<float>(i + 1);
    contours.outer_contour_hull.points.push_back(p);
  }

  // Outer wall segments: 3 segments.
  for (int i = 0; i < 3; ++i) {
    LineSegment seg;
    seg.start.x = static_cast<float>(i);
    seg.start.y = 0.0f;
    seg.end.x = static_cast<float>(i + 1);
    seg.end.y = 1.0f;
    contours.outer_contour_segments.push_back(seg);
  }

  // Two inner polygons: first 3 points, second 5 points.
  geometry_msgs::msg::Polygon inner_a;
  for (int i = 0; i < 3; ++i) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(i);
    p.y = 0.0f;
    inner_a.points.push_back(p);
  }
  geometry_msgs::msg::Polygon inner_b;
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::Point32 p;
    p.x = 0.0f;
    p.y = static_cast<float>(i);
    inner_b.points.push_back(p);
  }
  contours.inner_contours = {inner_a, inner_b};

  pub->publish(contours);

  ASSERT_TRUE(spinUntil(sink)) << "Expected warehouse_geometry publish";

  const nlohmann::json parsed = nlohmann::json::parse(sink.front());
  ASSERT_TRUE(parsed["outer_hull"].is_array());
  ASSERT_TRUE(parsed["wall_segments"].is_array());
  ASSERT_TRUE(parsed["inner_polygons"].is_array());

  EXPECT_EQ(parsed["outer_hull"].size(), 4u);
  EXPECT_EQ(parsed["wall_segments"].size(), 3u);
  ASSERT_EQ(parsed["inner_polygons"].size(), 2u);
  EXPECT_EQ(parsed["inner_polygons"][0].size(), 3u);
  EXPECT_EQ(parsed["inner_polygons"][1].size(), 5u);
}
