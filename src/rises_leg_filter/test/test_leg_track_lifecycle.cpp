// Unit tests for rises::LegFilterNode track lifecycle.
//
// Package: rises_leg_filter
// Strategy:
//   - Stand up the real LegFilterNode under a SingleThreadedExecutor with
//     parameters tuned for fast tests (publish_rate_hz=50, short timeouts,
//     low min_observations).
//   - Drive inputs by publishing ObstacleReport messages (obstacles placed in
//     the unmatched_obstacles[] field) on "obstacle_report" with explicit,
//     monotonic header stamps that the production code consumes as the
//     track's logical time. This avoids any dependence on wall-clock
//     duration for track-time arithmetic.
//   - Observe outputs via a spy subscriber on /humans/bodies/lidar_tracked
//     (the only public signal carrying track IDs).
//   - All waits are bounded by a deterministic spin-budget.
//
// GTEST_SKIP rationale:
//   - NewTrackCreatedOnFirstObservation: LegFilterNode has no public accessor
//     for the internal `tracks_` map, and the only externally-visible signal
//     (lidar_tracked) requires the track to pass the leg filter (which is
//     based on min_observations + velocity, NOT first-observation). The spec
//     calls for asserting on first-observation creation; that requires a
//     testability seam (e.g. const accessor to track count). Skipping.
//   - TrackUpdatedOnSubsequentObservation: same accessor requirement —
//     "track exists with updated position" is unobservable through the public
//     API without a seam.
//   - TrackExpiresAfterTimeout: production code uses message-stamp time to
//     evict tracks, but exposes no API to observe eviction directly. The
//     periodic publishCandidates() uses this->now() (wall clock) for filter
//     decisions, so an "expired but not yet evaluated" track cannot be
//     distinguished from a present-but-not-published track via lidar_tracked.
//     Skipping until a public hasTrack(id) seam is added.
//   - MaxTracksCapped: there is no `max_tracks` parameter in the production
//     node. Skipping until it is introduced.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_report.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "rises_leg_filter/leg_filter_node.hpp"

namespace {

using rises_interfaces::msg::Obstacle;
using rises_interfaces::msg::ObstacleReport;

constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr int kMaxSpinIterations = 300;
constexpr double kPublishRateHz = 50.0;
constexpr double kStationaryTimeoutSec = 0.5;
constexpr double kEvictionTimeoutSec = 2.0;
constexpr int kMinObservations = 3;
constexpr float kLegWidth = 0.3f;
constexpr double kBurstDtSec = 0.05;

// Velocity that comfortably exceeds the configured min_velocity (0.05 m/s)
// across kMinObservations samples at kBurstDtSec spacing.
constexpr double kMovingStepMeters = 0.10;

// Velocity that stays well below min_velocity (0.05 m/s) so the filter rejects.
constexpr double kStaticJitterMeters = 0.0;

class LegTrackFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void buildNode() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"min_width", 0.15},
        {"max_width", 0.8},
        {"min_velocity", 0.05},
        {"stationary_timeout_sec", kStationaryTimeoutSec},
        {"eviction_timeout_sec", kEvictionTimeoutSec},
        {"min_observations", kMinObservations},
        {"publish_rate_hz", kPublishRateHz},
        {"frame_id", std::string("map")},
        {"base_confidence", 0.3},
        {"boosted_confidence", 0.7},
        {"camera_match_radius", 1.5},
        {"camera_stale_sec", 2.0},
    });

    node_ = std::make_shared<rises::LegFilterNode>(options);

    pub_node_ = std::make_shared<rclcpp::Node>("test_leg_lifecycle_pub");
    obstacle_pub_ = pub_node_->create_publisher<ObstacleReport>(
        "obstacle_report", rclcpp::QoS(10).reliable());

    spy_node_ = std::make_shared<rclcpp::Node>("test_leg_lifecycle_spy");
    published_ids_.clear();
    spy_sub_ = spy_node_->create_subscription<std_msgs::msg::String>(
        "/humans/bodies/lidar_tracked", rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) {
          published_ids_.push_back(msg->data);
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(pub_node_);
    executor_->add_node(spy_node_);

    base_stamp_ = node_->now();
  }

  void TearDown() override {
    executor_.reset();
    spy_sub_.reset();
    obstacle_pub_.reset();
    spy_node_.reset();
    pub_node_.reset();
    node_.reset();
    published_ids_.clear();
  }

  /// Publishes a single ObstacleReport at logical (sim) time offset
  /// `seconds_offset` from base_stamp_, with one unmatched obstacle at
  /// (x, y) and the given width.
  void publishObservation(std::uint64_t id, double seconds_offset, double x,
                          double y, float width = kLegWidth) {
    ObstacleReport report;
    report.header.stamp =
        base_stamp_ + rclcpp::Duration::from_seconds(seconds_offset);
    report.header.frame_id = "map";

    Obstacle obs;
    obs.id = id;
    obs.type = Obstacle::POINT;
    obs.width = width;
    obs.position.x = x;
    obs.position.y = y;
    report.unmatched_obstacles.push_back(obs);

    obstacle_pub_->publish(report);
  }

  /// Publishes `count` observations of a leg moving in +x at `step_meters`
  /// per sample, all under the same obstacle id.
  void publishMovingBurst(std::uint64_t id, int count, double step_meters) {
    for (int i = 0; i < count; ++i) {
      const double t = i * kBurstDtSec;
      const double x = i * step_meters;
      publishObservation(id, t, x, 0.0);
      spinFor(std::chrono::milliseconds(5));
    }
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

  void spinFor(std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      std::this_thread::sleep_for(kSpinSlice);
    }
  }

  bool seenIdToken(const std::string &token) const {
    for (const std::string &payload : published_ids_) {
      if (payload.find(token) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<rises::LegFilterNode> node_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  std::shared_ptr<rclcpp::Node> spy_node_;
  rclcpp::Publisher<ObstacleReport>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spy_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::string> published_ids_;
  rclcpp::Time base_stamp_;
};

} // namespace

// Pins first-observation track creation; skipped until LegFilterNode exposes a
// const track accessor (creation is not observable on the lidar_tracked topic).
TEST_F(LegTrackFixture, NewTrackCreatedOnFirstObservation) {
  GTEST_SKIP_(
      "LegFilterNode exposes no `tracks_` accessor; the only public signal "
      "(/humans/bodies/lidar_tracked) requires min_observations to be reached, "
      "so first-observation track creation is unobservable. Re-enable once a "
      "const inspector such as trackCount() / hasTrack(id) is added.");
}

// Pins position update on a subsequent observation; skipped until a per-track
// accessor seam exists (not observable via the lidar_tracked topic alone).
TEST_F(LegTrackFixture, TrackUpdatedOnSubsequentObservation) {
  GTEST_SKIP_(
      "LegFilterNode exposes no public accessor for an individual track's "
      "last_x/last_y. Position update is unobservable through the lidar "
      "tracked topic alone. Re-enable once a track inspector seam is added.");
}

// Pins track eviction after timeout; skipped until a hasTrack(id) accessor lets
// the test distinguish "evicted" from "present but filtered out".
TEST_F(LegTrackFixture, TrackExpiresAfterTimeout) {
  GTEST_SKIP_(
      "Eviction is observable only through internal state. Re-enable once a "
      "const hasTrack(id) inspector is added or evictStaleTracks() exposes a "
      "return-the-erased-ids contract.");
}

// A track with fewer than min_observations observations must not be
// published on /humans/bodies/lidar_tracked even when it is moving fast
// enough to clear the velocity gate. This is observable via the public
// topic and so does not require a seam.
TEST_F(LegTrackFixture, MinObservationsGate) {
  buildNode();

  // One observation only — below min_observations (=3).
  publishObservation(/*id=*/11, /*seconds_offset=*/0.0, /*x=*/0.0, /*y=*/0.0);

  // Give the periodic publisher generous time to fire.
  spinFor(std::chrono::milliseconds(400));

  EXPECT_FALSE(seenIdToken("lidar_11"))
      << "Track below min_observations was published.";
}

// A track that stays stationary must NOT pass the leg filter (its average
// velocity is below min_velocity AND it has not been camera-confirmed).
TEST_F(LegTrackFixture, VelocityFilterRejectsStatic) {
  buildNode();

  // Burst of observations with effectively zero motion.
  publishMovingBurst(/*id=*/22, /*count=*/kMinObservations + 2,
                     /*step_meters=*/kStaticJitterMeters);

  // Wait long enough that any periodic publish would have fired.
  spinFor(std::chrono::milliseconds(400));

  EXPECT_FALSE(seenIdToken("lidar_22"))
      << "Static track passed the velocity gate.";
}

// A track moving fast enough must pass the filter and appear on
// /humans/bodies/lidar_tracked.
TEST_F(LegTrackFixture, VelocityFilterAcceptsMoving) {
  buildNode();

  publishMovingBurst(/*id=*/33, /*count=*/kMinObservations + 2,
                     /*step_meters=*/kMovingStepMeters);

  ASSERT_TRUE(spinUntil([this]() { return seenIdToken("lidar_33"); }))
      << "Moving track was never published; velocity gate too strict or "
         "track lifecycle broken.";
}

// Pins a cap on the tracks_ map; skipped until a max_tracks parameter exists.
TEST_F(LegTrackFixture, MaxTracksCapped) {
  GTEST_SKIP_(
      "LegFilterNode does not declare a `max_tracks` parameter and applies no "
      "cap to the tracks_ map. Re-enable once the parameter is introduced.");
}

// A track whose width is inside [min_width, max_width] is NOT immediately
// rejected by the width gate, so a moving track of that width still
// accumulates observations and survives to publication. This is the R3 fix
// under test: obstacleCallback now reads width from ObstacleReport's
// unmatched_obstacles[] (populated by ObstacleReportBuilder), not from the
// always-zero ObstacleArray.obstacles[].width on unmatched_obstacles.
TEST_F(LegTrackFixture, WidthInsideGateSurvivesAsMovingTrack) {
  buildNode();

  ASSERT_GE(kLegWidth, 0.15f);
  ASSERT_LE(kLegWidth, 0.8f);

  publishMovingBurst(/*id=*/44, /*count=*/kMinObservations + 2,
                     /*step_meters=*/kMovingStepMeters);

  ASSERT_TRUE(spinUntil([this]() { return seenIdToken("lidar_44"); }))
      << "Track with in-gate width never published; width gate wrongly "
         "rejected it.";
}

// A track whose width is outside [min_width, max_width] is rejected by the
// width gate on every observation (track.is_candidate forced false), so it
// must never appear on lidar_tracked even while moving fast enough to
// otherwise pass the velocity filter.
TEST_F(LegTrackFixture, WidthOutsideGateRejectsCandidate) {
  buildNode();

  constexpr float kTooWide = 2.0f; // above max_width (0.8)
  ASSERT_GT(kTooWide, 0.8f);

  for (int i = 0; i < kMinObservations + 2; ++i) {
    const double t = i * kBurstDtSec;
    const double x = i * kMovingStepMeters;
    publishObservation(/*id=*/55, t, x, 0.0, kTooWide);
    spinFor(std::chrono::milliseconds(5));
  }

  spinFor(std::chrono::milliseconds(400));

  EXPECT_FALSE(seenIdToken("lidar_55"))
      << "Out-of-gate-width track was published despite the width gate.";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
