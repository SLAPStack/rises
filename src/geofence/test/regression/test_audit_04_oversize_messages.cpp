// =============================================================================
// Audit Finding #4 -- Uncapped message sizes lead to unbounded allocation
// =============================================================================
//
// Locations:
//   geofence/spatial_node/src/node/map_update_handler.cpp:45      (Stats apply)
//   geofence/spatial_node/src/queries/batch_point_checker.cpp:445-471 (vertex
//   flatten)
//
// `MapUpdateHandler::applyUpdates` walks every entry in the inbound
// `ObstacleUpdateArray` with no upper bound on the array length, so a single
// adversarial / mis-sized message can fan out into hundreds of MB of spatial
// index work. `BatchPointChecker::checkObstacles` is symmetric: it sums every
// vertex across every obstacle into thread_local SoA buffers with no cap.
//
// Expected fix: enforce an explicit MAX_OBSTACLE_UPDATES (and a matching
// MAX_TOTAL_VERTICES) constant. When the cap is exceeded the call must
// return early with an error signal -- either RCLCPP_ERROR + empty Stats /
// ObstacleResult, or a Stats::rejected counter, depending on which knob the
// fix picks.
//
// STATUS: RED on develop. The "just over cap" / "1M vertices" cases expect
// post-fix behaviour and will FAIL against current code. The "below cap" /
// "at cap" cases are GREEN today and pin the lower edge against an over-eager
// fix that lowers the cap below the realistic operating range.
//
// Memory accounting uses getrusage(RUSAGE_SELF). On non-glibc hosts (or any
// platform that does not export ru_maxrss in KiB), the memory assertion
// degrades to a wall-clock deadline -- failure to finish in time is also a
// pass-through for "unbounded allocation".
// =============================================================================

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#if defined(__GLIBC__) || defined(__APPLE__)
#include <sys/resource.h>
#define ARISE_HAVE_RUSAGE 1
#else
#define ARISE_HAVE_RUSAGE 0
#endif

#include "test_support/obstacle_builder.hpp"
#include "test_support/oversize_msg_factory.hpp"

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/node/map_update_handler.hpp"
#include "geofence/spatial/policies/robot_safety_profile.hpp"
#include "geofence/spatial/queries/batch_point_checker.hpp"
#include "spatial_index_selection.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"

namespace {

using rises::MapUpdateHandler;
using rises::geofence::GeofenceMap;
using rises::geofence::RobotSafetyProfile;
using rises::geofence::SpatialIndex;
using rises::geofence::query::BatchPointChecker;

// Soft cap that the fix is expected to introduce in
// MapUpdateHandler::applyUpdates. Picked to be well above any realistic
// warehouse map size while still small enough to keep this test fast.
constexpr std::size_t kMaxObstacleUpdates = 10'000;

// Wall-clock fallback when we cannot read ru_maxrss.
constexpr auto kWallClockDeadline = std::chrono::seconds(10);

// Max heap delta we tolerate when an oversize message is rejected. ru_maxrss
// is reported in KiB on Linux; an unbounded run blows past several hundred MB
// almost immediately on a 1M-vertex test.
constexpr long kMaxHeapDeltaKib = 256L * 1024L; // 256 MiB

std::unique_ptr<GeofenceMap> makeMap() {
  std::function<std::shared_ptr<SpatialIndex>()> factory = []() {
    return std::make_shared<SpatialIndex>();
  };
  return std::make_unique<GeofenceMap>(factory);
}

long peakResidentKib() {
#if ARISE_HAVE_RUSAGE
  struct rusage usage{};
  if (::getrusage(RUSAGE_SELF, &usage) != 0) {
    return -1;
  }
#if defined(__APPLE__)
  // macOS reports ru_maxrss in bytes; normalize to KiB.
  return static_cast<long>(usage.ru_maxrss / 1024);
#else
  return static_cast<long>(usage.ru_maxrss);
#endif
#else
  return -1;
#endif
}

class OversizeFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Logger logger() const { return rclcpp::get_logger("oversize_test"); }
};

} // namespace

TEST_F(OversizeFixture, BelowCapAccepted) {
  auto map = makeMap();
  constexpr std::size_t kCount = 1'000;
  static_assert(kCount < kMaxObstacleUpdates, "below cap requires < cap");

  const auto updates = test_support::oversize::makeObstacleUpdateArray(kCount);
  const MapUpdateHandler::Stats stats = MapUpdateHandler::applyUpdates(
      updates, *map, /*visualizer=*/nullptr, logger(), /*source_tag=*/"TEST");

  EXPECT_EQ(stats.added, static_cast<int32_t>(kCount));
}

TEST_F(OversizeFixture, AtCapAccepted) {
  auto map = makeMap();

  const auto updates =
      test_support::oversize::makeObstacleUpdateArray(kMaxObstacleUpdates);
  const MapUpdateHandler::Stats stats = MapUpdateHandler::applyUpdates(
      updates, *map, /*visualizer=*/nullptr, logger(), /*source_tag=*/"TEST");

  EXPECT_EQ(stats.added, static_cast<int32_t>(kMaxObstacleUpdates));
}

TEST_F(OversizeFixture, JustOverCapRejected) {
  auto map = makeMap();
  const std::size_t overflow_count = kMaxObstacleUpdates + 1;

  const auto updates =
      test_support::oversize::makeObstacleUpdateArray(overflow_count);

  const long baseline_kib = peakResidentKib();
  const auto start = std::chrono::steady_clock::now();

  const MapUpdateHandler::Stats stats = MapUpdateHandler::applyUpdates(
      updates, *map, /*visualizer=*/nullptr, logger(), /*source_tag=*/"TEST");

  const auto elapsed = std::chrono::steady_clock::now() - start;
  const long peak_kib = peakResidentKib();

  // Post-fix expectation: rejection is observable. We accept any of the
  // plausible signals so the fix author can pick whichever fits the codebase:
  //   1. Stats::added == 0 (early return drops everything), or
  //   2. Stats::added < count (some applied before the cap, then aborted), or
  //   3. A new Stats::rejected counter > 0 once the struct grows that field.
  EXPECT_LT(stats.added, static_cast<int32_t>(overflow_count));

  if (baseline_kib > 0 && peak_kib > 0) {
    EXPECT_LT(peak_kib - baseline_kib, kMaxHeapDeltaKib)
        << "Oversize ObstacleUpdateArray triggered unbounded allocation: "
        << (peak_kib - baseline_kib) << " KiB";
  } else {
    EXPECT_LT(elapsed, kWallClockDeadline)
        << "Oversize ObstacleUpdateArray did not return within deadline";
  }
}

TEST_F(OversizeFixture, BatchPointCheckerCapsTotalVerts) {
  auto map = makeMap();
  constexpr std::size_t kVertexCount = 1'000'000;

  // Single obstacle with one million POINT vertices triggers the
  // batch_point_checker.cpp:445-471 flatten loop. Post-fix, the call must
  // refuse to allocate gigabytes of thread_local SoA storage.
  rises_interfaces::msg::ObstacleArray array;
  rises_interfaces::msg::Obstacle obs;
  obs.id = 1;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.vertices.reserve(kVertexCount);
  for (std::size_t i = 0; i < kVertexCount; ++i) {
    obs.vertices.push_back(test_support::makePoint(
        0.01 * static_cast<double>(i), 0.01 * static_cast<double>(i)));
  }
  array.obstacles.push_back(std::move(obs));

  RobotSafetyProfile profile; // default outer zone, no exclusions

  const long baseline_kib = peakResidentKib();
  const auto start = std::chrono::steady_clock::now();

  const auto result = BatchPointChecker::checkObstacles(
      *map, array, /*robot_position=*/{0.0, 0.0},
      /*robot_heading_rad=*/0.0F, profile,
      /*tolerance=*/0.10F,
      /*include_dynamic_obstacles=*/false);
  (void)result; // value is irrelevant; we only assert resource bounds.

  const auto elapsed = std::chrono::steady_clock::now() - start;
  const long peak_kib = peakResidentKib();

  if (baseline_kib > 0 && peak_kib > 0) {
    EXPECT_LT(peak_kib - baseline_kib, kMaxHeapDeltaKib)
        << "BatchPointChecker allocated unbounded buffers: "
        << (peak_kib - baseline_kib) << " KiB";
  } else {
    EXPECT_LT(elapsed, kWallClockDeadline)
        << "BatchPointChecker did not return within deadline on 1M vertices";
  }
}
