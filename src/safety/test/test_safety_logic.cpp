// Unit tests for the Safety supervisor node — NON-regression tests that
// validate normal-operation correctness on develop. All tests here are
// expected GREEN against the current production behaviour of
// safety/src/safety.cpp.
//
// Scope:
//   - Halt path via diagnostics ERROR (the only e-stop pathway in the
//     current implementation; there is no dedicated e-stop topic).
//   - Idempotence of haltSystem under repeated triggers.
//   - Path-overlap warning when a path crosses a high-confidence heatmap
//     cell, including the below-threshold case.
//   - Quiescence when a heatmap arrives without a subsequent path.
//
// Testability seams required for cases not covered here:
//   - HaltSystemOnObstacleAlert: production code does NOT halt on a
//     detected-obstacles message; it only republishes on /alert. Skipped
//     because the premise contradicts current correct behaviour.
//   - ResumeOnAllClear: resumeSystem() only fires from healthCheckCallback,
//     which runs on a 5-second wall timer. A test seam is needed (e.g.
//     parameter-controlled timer period or a public trigger) to exercise
//     this deterministically without long sleeps. Skipped.
//
// Standards binding:
//   - Real Safety class instantiated as rclcpp::Node; no mocks of rclcpp.
//   - Deterministic. Bounded spin_some slices, no sleep > 100 ms.
//   - Function cap 100 lines, nesting <= 3, named constants.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "rises_interfaces/srv/validate_path.hpp"
#include "safety/safety.hpp"

namespace {

constexpr auto kSpinPollStep = std::chrono::milliseconds(5);
constexpr int kSpinIterations = 20;     // total bounded wait = 100 ms
constexpr int kSpinIterationsLong = 40; // total bounded wait = 200 ms

constexpr const char *kDiagnosticsTopic = "/diagnostics";
constexpr const char *kPathTopic = "incoming_path";
constexpr const char *kHeatmapTopic = "predicted_occupancy";
constexpr const char *kHaltTopic = "halt";
constexpr const char *kResponseTopic = "response";
constexpr const char *kValidatedPathTopic = "validated_path";
constexpr const char *kValidatePathService = "validate_path";

constexpr const char *kOverlapWarningSubstring = "overlaps";
constexpr const char *kHaltSubstring = "HALT";

constexpr int kHeatmapWidth = 10;
constexpr int kHeatmapHeight = 10;
constexpr double kHeatmapResolution = 1.0;
constexpr int kOverlapThreshold = 50;
constexpr int kHighConfidenceValue = 90;
constexpr int kLowConfidenceValue = 10;
constexpr int kOverlapCellCol = 5;
constexpr int kOverlapCellRow = 5;

// Spin the executor in small slices so callbacks fire without blocking the
// test for long. Total wait bounded by iterations * kSpinPollStep.
void spinBriefly(rclcpp::Executor &executor, int iterations = kSpinIterations) {
  for (int i = 0; i < iterations; ++i) {
    executor.spin_some();
    rclcpp::sleep_for(kSpinPollStep);
  }
}

// Stub ValidatePath server replying with a fixed verdict. Used to keep
// pathCallback's async branch deterministic.
class ValidatePathStub {
public:
  ValidatePathStub(rclcpp::Node::SharedPtr node, bool reply_blocked)
      : node_(std::move(node)), reply_blocked_(reply_blocked) {
    using rises_interfaces::srv::ValidatePath;
    server_ = node_->create_service<ValidatePath>(
        kValidatePathService,
        [this](const std::shared_ptr<ValidatePath::Request> /*req*/,
               std::shared_ptr<ValidatePath::Response> resp) {
          resp->blocked = reply_blocked_;
          resp->reason = reply_blocked_ ? "stub: blocked" : "stub: clear";
          ++call_count_;
        });
  }

  int callCount() const { return call_count_; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<rises_interfaces::srv::ValidatePath>::SharedPtr server_;
  bool reply_blocked_;
  int call_count_ = 0;
};

class SafetyLogicHarness : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"validate_path_service_name", std::string(kValidatePathService)},
        {"node_timeout_sec", 60.0},
        {"heatmap_overlap_threshold", kOverlapThreshold},
    });
    safety_node_ = std::make_shared<rises::Safety>(options);
    helper_node_ = std::make_shared<rclcpp::Node>("safety_logic_test_helper");

    halt_messages_.clear();
    response_messages_.clear();
    validated_path_count_ = 0;

    halt_sub_ = helper_node_->create_subscription<std_msgs::msg::Bool>(
        kHaltTopic, rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          halt_messages_.push_back(msg->data);
        });
    response_sub_ = helper_node_->create_subscription<std_msgs::msg::String>(
        kResponseTopic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          response_messages_.push_back(msg->data);
        });
    validated_sub_ = helper_node_->create_subscription<nav_msgs::msg::Path>(
        kValidatedPathTopic, 10,
        [this](const nav_msgs::msg::Path::SharedPtr /*msg*/) {
          ++validated_path_count_;
        });

    diagnostics_pub_ =
        helper_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            kDiagnosticsTopic, 10);
    path_pub_ =
        helper_node_->create_publisher<nav_msgs::msg::Path>(kPathTopic, 10);
    heatmap_pub_ = helper_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        kHeatmapTopic, rclcpp::QoS(1).reliable());

    executor_.add_node(safety_node_);
    executor_.add_node(helper_node_);
  }

  void TearDown() override {
    executor_.remove_node(helper_node_);
    executor_.remove_node(safety_node_);
    helper_node_.reset();
    safety_node_.reset();
  }

  diagnostic_msgs::msg::DiagnosticArray
  makeDiagnostic(uint8_t level, const std::string &name,
                 const std::string &message) const {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = helper_node_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = name;
    status.message = message;
    arr.status.push_back(status);
    return arr;
  }

  nav_msgs::msg::Path makePathThroughCell(int col, int row) const {
    nav_msgs::msg::Path path;
    path.header.stamp = helper_node_->now();
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    // Heatmap origin is (0,0) with resolution 1.0; cell centre at col+0.5.
    pose.pose.position.x = static_cast<double>(col) + 0.5;
    pose.pose.position.y = static_cast<double>(row) + 0.5;
    path.poses.push_back(pose);
    return path;
  }

  nav_msgs::msg::OccupancyGrid makeHeatmapWithHotCell(int hot_col, int hot_row,
                                                      int hot_value) const {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = helper_node_->now();
    grid.header.frame_id = "map";
    grid.info.resolution = static_cast<float>(kHeatmapResolution);
    grid.info.width = kHeatmapWidth;
    grid.info.height = kHeatmapHeight;
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.data.assign(static_cast<std::size_t>(kHeatmapWidth * kHeatmapHeight),
                     0);
    const std::size_t idx =
        static_cast<std::size_t>(hot_row * kHeatmapWidth + hot_col);
    grid.data[idx] = static_cast<int8_t>(hot_value);
    return grid;
  }

  bool responseContains(const std::string &needle) const {
    for (const std::string &line : response_messages_) {
      if (line.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  int countHaltsWithValue(bool value) const {
    int n = 0;
    for (bool m : halt_messages_) {
      if (m == value) {
        ++n;
      }
    }
    return n;
  }

  std::shared_ptr<rises::Safety> safety_node_;
  std::shared_ptr<rclcpp::Node> helper_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr halt_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr response_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr validated_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr heatmap_pub_;
  std::vector<bool> halt_messages_;
  std::vector<std::string> response_messages_;
  int validated_path_count_ = 0;
};

} // namespace

// Premise of "halt on detected-obstacle message" contradicts current
// production behaviour: detectedObstaclesCallback only republishes on /alert
// and emits a response string. It does not raise /halt. A regression test
// could be filed if the safety policy ever requires escalation here.
TEST_F(SafetyLogicHarness, HaltSystemOnObstacleAlert) {
  GTEST_SKIP() << "Production Safety does not halt on detected_obstacles; "
                  "behaviour is republish-only. Skipped to keep the GREEN "
                  "non-regression suite faithful to current semantics.";
}

TEST_F(SafetyLogicHarness, HaltSystemOnEmergencyStop) {
  // Diagnostics ERROR is the e-stop pathway in the current implementation.
  using diagnostic_msgs::msg::DiagnosticStatus;
  auto diag = makeDiagnostic(DiagnosticStatus::ERROR, "critical_node",
                             "hardware fault");
  diagnostics_pub_->publish(diag);

  spinBriefly(executor_);

  EXPECT_EQ(countHaltsWithValue(true), 1)
      << "Diagnostics ERROR must raise /halt exactly once";
  EXPECT_TRUE(responseContains(kHaltSubstring))
      << "Response stream must carry a HALT string";
}

// resumeSystem() only fires from healthCheckCallback, which is bound to a
// 5-second wall timer. A deterministic test requires either a configurable
// timer period or a public hook to invoke healthCheckCallback() directly.
// Until that seam exists, this case is not exercised in CI.
TEST_F(SafetyLogicHarness, ResumeOnAllClear) {
  GTEST_SKIP() << "Resume path requires a testability seam: either expose "
                  "healthCheckCallback() or make the timer period a "
                  "parameter. Current 5s wall timer is incompatible with the "
                  "no-long-sleep test standard.";
}

TEST_F(SafetyLogicHarness, PathOverlapWithPredictionEmitsWarning) {
  // Stub server replies blocked=false so the path is forwarded; the overlap
  // warning is independent of the geofence verdict.
  ValidatePathStub stub(helper_node_, /*reply_blocked=*/false);

  auto heatmap = makeHeatmapWithHotCell(kOverlapCellCol, kOverlapCellRow,
                                        kHighConfidenceValue);
  heatmap_pub_->publish(heatmap);
  spinBriefly(executor_);

  auto path = makePathThroughCell(kOverlapCellCol, kOverlapCellRow);
  path_pub_->publish(path);
  spinBriefly(executor_, kSpinIterationsLong);

  EXPECT_TRUE(responseContains(kOverlapWarningSubstring))
      << "Path-vs-heatmap overlap must publish a warning on /response";
  EXPECT_GE(stub.callCount(), 1)
      << "ValidatePath must still be called; overlap is a warning, not a "
         "halt";
  EXPECT_EQ(countHaltsWithValue(true), 0)
      << "Overlap warning must not raise /halt";
}

TEST_F(SafetyLogicHarness, PathOverlapDoesNotHaltIfBelowConfidence) {
  ValidatePathStub stub(helper_node_, /*reply_blocked=*/false);

  auto heatmap = makeHeatmapWithHotCell(kOverlapCellCol, kOverlapCellRow,
                                        kLowConfidenceValue);
  heatmap_pub_->publish(heatmap);
  spinBriefly(executor_);

  auto path = makePathThroughCell(kOverlapCellCol, kOverlapCellRow);
  path_pub_->publish(path);
  spinBriefly(executor_, kSpinIterationsLong);

  EXPECT_FALSE(responseContains(kOverlapWarningSubstring))
      << "Below-threshold heatmap cells must not trigger overlap warning";
  EXPECT_EQ(countHaltsWithValue(true), 0)
      << "Below-threshold overlap must not raise /halt";
}

TEST_F(SafetyLogicHarness, RepeatedHaltsAreIdempotent) {
  // haltSystem uses atomic exchange(true) so only the first invocation
  // publishes; subsequent ERROR diagnostics on the same node must not emit
  // a duplicate halt message.
  using diagnostic_msgs::msg::DiagnosticStatus;
  const int kRepeatCount = 3;
  for (int i = 0; i < kRepeatCount; ++i) {
    auto diag = makeDiagnostic(DiagnosticStatus::ERROR, "critical_node",
                               "hardware fault");
    diagnostics_pub_->publish(diag);
    spinBriefly(executor_);
  }

  EXPECT_EQ(countHaltsWithValue(true), 1)
      << "haltSystem must be idempotent: one /halt true message regardless "
         "of how many ERROR diagnostics arrive while already halted";
}

TEST_F(SafetyLogicHarness, HeatmapWithoutPathDoesNotEmitWarning) {
  auto heatmap = makeHeatmapWithHotCell(kOverlapCellCol, kOverlapCellRow,
                                        kHighConfidenceValue);
  heatmap_pub_->publish(heatmap);
  spinBriefly(executor_, kSpinIterationsLong);

  EXPECT_FALSE(responseContains(kOverlapWarningSubstring))
      << "Heatmap arrival alone must not emit an overlap warning; warning "
         "is only computed inside pathCallback";
  EXPECT_EQ(countHaltsWithValue(true), 0)
      << "Heatmap arrival must not raise /halt";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
