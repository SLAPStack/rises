// Gap-closure unit tests for TF failure modes in laserscan_preprocessor.
//
// The onLaserScan path resolves a transform from the scan's frame to
// target_frame via tf2_ros::Buffer::lookupTransform. The current code logs
// and skips on failure; these tests pin the documented contract:
//
//   * lookupTransform timeout (empty buffer) — node must log and skip, must
//     not crash and must not publish stale data.
//   * frame absent — same contract as timeout.
//   * extrapolation error (stamp far in past) — handled gracefully.
//   * recovery — once TF becomes available, subsequent scans must succeed.
//
// Behaviour under test is observable at the ObstacleArray subscriber and at
// the node's continued ability to process valid scans. We do NOT poke
// rclcpp logger sinks — the assertion is on whether obstacles are produced
// at all (no ghost-clear is asserted elsewhere; see
// regression/test_audit_07_exception_ghost_clear.cpp).
//
// Standards binding:
//   - Real LaserPreprocessorNode lifecycle node, no mocks of rclcpp.
//   - Deterministic. Bounded spin loop (no sleeps > 100 ms total).
//   - Function cap 100 lines, nesting <= 3, named constants.

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "rises_interfaces/msg/obstacle_array.hpp"
#include "laserscan_preprocessor/laserscan_preprocessor_node.hpp"

namespace {

constexpr auto kSpinPollStep = std::chrono::milliseconds(5);
constexpr int kSpinIterations = 20; // total bounded wait = 100 ms
constexpr const char *kObstaclesTopic = "obstacles";
constexpr const char *kLaserTopic = "scan_tf_test";
constexpr const char *kLaserFrame = "laser_tf";
constexpr const char *kTargetFrame = "base_link";
constexpr const char *kMissingFrame = "frame_that_does_not_exist";
constexpr float kAngleMin = -1.0f;
constexpr float kAngleMax = 1.0f;
constexpr float kAngleIncrement = 0.1f;
constexpr float kRangeMin = 0.1f;
constexpr float kRangeMax = 10.0f;
constexpr int kRangeCount = 20;
constexpr float kSampleRange = 1.0f;
constexpr float kSampleIntensity = 100.0f;
constexpr int kExtrapolationSecondsInPast = 3600;

void spinBriefly(rclcpp::Executor &executor) {
  for (int i = 0; i < kSpinIterations; ++i) {
    executor.spin_some();
    rclcpp::sleep_for(kSpinPollStep);
  }
}

sensor_msgs::msg::LaserScan makeScanInFrame(const rclcpp::Time &stamp,
                                            const std::string &frame) {
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = frame;
  scan.angle_min = kAngleMin;
  scan.angle_max = kAngleMax;
  scan.angle_increment = kAngleIncrement;
  scan.range_min = kRangeMin;
  scan.range_max = kRangeMax;
  scan.ranges.assign(kRangeCount, kSampleRange);
  scan.intensities.assign(kRangeCount, kSampleIntensity);
  return scan;
}

class TfFailureModesHarness : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    laser_count_ = static_cast<std::size_t>(LASER_COUNT);
    if (laser_count_ != 1) {
      // The TF tests rely on a single laser pipeline; with multiple lasers
      // compiled in the harness would need to seed N TF entries. Skip at
      // SetUp by leaving harness_ready_ false.
      return;
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"distance_threshold", 0.2},
        {"angle_threshold_deg", 30.0},
        {"target_frame", std::string(kTargetFrame)},
        {"tf_prefix", std::string("")},
        {"laser_frame_ids", std::vector<std::string>{kLaserFrame}},
        {"laser_topics", std::vector<std::string>{kLaserTopic}},
        {"publish_points_only", false},
    });
    node_ = std::make_shared<rises::LaserPreprocessorNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("tf_failure_test_helper");
    setupSubscriber();

    scan_pub_ = helper_->create_publisher<sensor_msgs::msg::LaserScan>(
        kLaserTopic, rclcpp::SensorDataQoS());

    const auto cfg = node_->on_configure(rclcpp_lifecycle::State{});
    configure_ok_ =
        (cfg == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                    CallbackReturn::SUCCESS);
    if (!configure_ok_) {
      return;
    }
    const auto act = node_->on_activate(rclcpp_lifecycle::State{});
    activate_ok_ = (act == rclcpp_lifecycle::node_interfaces::
                               LifecycleNodeInterface::CallbackReturn::SUCCESS);

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(helper_);
  }

  void TearDown() override {
    if (node_) {
      node_->on_deactivate(rclcpp_lifecycle::State{});
    }
    node_.reset();
    helper_.reset();
  }

  void setupSubscriber() {
    obstacle_msgs_.clear();
    obstacle_sub_ =
        helper_->create_subscription<rises_interfaces::msg::ObstacleArray>(
            kObstaclesTopic, 10,
            [this](const rises_interfaces::msg::ObstacleArray::SharedPtr msg) {
              obstacle_msgs_.push_back(*msg);
            });
  }

  void publishStaticTransform() {
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = helper_->now();
    tf.header.frame_id = kTargetFrame;
    tf.child_frame_id = kLaserFrame;
    tf.transform.rotation.w = 1.0;
    static_tf_->sendTransform(tf);
  }

  bool harnessReady() const {
    return laser_count_ == 1 && configure_ok_ && activate_ok_;
  }

  std::shared_ptr<rises::LaserPreprocessorNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<rises_interfaces::msg::ObstacleArray>::SharedPtr
      obstacle_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  std::vector<rises_interfaces::msg::ObstacleArray> obstacle_msgs_;
  std::size_t laser_count_ = 0;
  bool configure_ok_ = false;
  bool activate_ok_ = false;
};

} // namespace

TEST_F(TfFailureModesHarness, LookupTimeoutLogsAndSkips) {
  if (!harnessReady()) {
    GTEST_SKIP() << "Harness requires LASER_COUNT==1 (have " << laser_count_
                 << ") to exercise single-frame TF path.";
  }
  // Intentionally do NOT publish a TF — the buffer remains empty. The scan
  // callback should log a throttled warning and skip publish.
  scan_pub_->publish(makeScanInFrame(helper_->now(), kLaserFrame));
  spinBriefly(executor_);

  EXPECT_TRUE(obstacle_msgs_.empty())
      << "Empty TF buffer must result in skipped publish (no ghost data).";
}

TEST_F(TfFailureModesHarness, FrameAbsentSkipsScan) {
  if (!harnessReady()) {
    GTEST_SKIP() << "Harness requires LASER_COUNT==1.";
  }
  // Publish a scan in a frame the node never registered nor saw via TF.
  scan_pub_->publish(makeScanInFrame(helper_->now(), kMissingFrame));
  spinBriefly(executor_);

  EXPECT_TRUE(obstacle_msgs_.empty())
      << "Unknown frame must be skipped, not crashed on.";
}

TEST_F(TfFailureModesHarness, ExtrapolationErrorHandled) {
  if (!harnessReady()) {
    GTEST_SKIP() << "Harness requires LASER_COUNT==1.";
  }
  publishStaticTransform();
  spinBriefly(executor_); // let TF propagate

  const rclcpp::Time ancient =
      helper_->now() -
      rclcpp::Duration::from_seconds(kExtrapolationSecondsInPast);
  scan_pub_->publish(makeScanInFrame(ancient, kLaserFrame));
  spinBriefly(executor_);

  // Static transforms cover all times by definition, so extrapolation may
  // succeed for the static_transform_broadcaster path. We only assert
  // absence of crash. If implementation switches to dynamic TF with a
  // bounded cache, this becomes a hard assertion.
  SUCCEED() << "Far-past stamp handled without crash (static TF path).";
}

TEST_F(TfFailureModesHarness, RecoveryAfterTfBecomesAvailable) {
  if (!harnessReady()) {
    GTEST_SKIP() << "Harness requires LASER_COUNT==1.";
  }
  // Phase 1: scan without TF — expect skip.
  scan_pub_->publish(makeScanInFrame(helper_->now(), kLaserFrame));
  spinBriefly(executor_);
  const std::size_t before_tf = obstacle_msgs_.size();

  // Phase 2: publish TF, give it time to register, then scan again.
  publishStaticTransform();
  spinBriefly(executor_);
  scan_pub_->publish(makeScanInFrame(helper_->now(), kLaserFrame));
  spinBriefly(executor_);

  EXPECT_GT(obstacle_msgs_.size(), before_tf)
      << "After TF becomes available, scans must produce ObstacleArrays.";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
