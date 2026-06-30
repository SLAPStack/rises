// =============================================================================
// Gap-closure unit tests for obstacle correspondence matching.
//
// Source under test:
//   geofence/spatial_node/src/queries/obstacle_correspondence_matcher.cpp
//   geofence/spatial_node/src/queries/obstacle_correspondence_matcher_simd.cpp
//
// Both implementations expose the same public surface but live in distinct
// classes (ObstacleCorrespondenceMatcher and
// ObstacleCorrespondenceMatcherSIMD). The parity tests below run both backends
// against the same inputs and assert the bool match outcome is identical -- the
// load-bearing signal that the SIMD rewrite did not regress correctness.
//
// When USE_SIMD is not enabled at build time the SIMD source is not compiled
// in (see geofence/CMakeLists.txt), so the parity tests GTEST_SKIP.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "test_support/obstacle_builder.hpp"

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/policies/robot_safety_profile.hpp"
#include "geofence/spatial/queries/obstacle_correspondence_matcher.hpp"
#include "spatial_index_selection.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"

#ifdef USE_SIMD
#include "geofence/spatial/queries/obstacle_correspondence_matcher_simd.hpp"
#endif

namespace {

using rises::geofence::fromObstacleMsg;
using rises::geofence::GeofenceMap;
using rises::geofence::ObstacleCorrespondenceMatcher;
using ::Point2D;
using rises::geofence::RobotSafetyProfile;
using rises::geofence::SpatialIndex;

constexpr float kPositionTolerance = 0.10f;
constexpr float kPointOnLineTolerance = 0.05f;
constexpr float kLineContainmentTolerance = 0.03f;
constexpr float kRobotHeadingRad = 0.0f;
// Robot position kept far from the test obstacles so the (empty) safety
// profile never filters them out.
constexpr float kRobotX = 0.0f;
constexpr float kRobotY = 0.0f;
constexpr int kRectangleCount = 100;

std::unique_ptr<GeofenceMap> makeMap() {
  std::function<std::shared_ptr<SpatialIndex>()> factory = []() {
    return std::make_shared<SpatialIndex>();
  };
  return std::make_unique<GeofenceMap>(factory);
}

void initBoth() {
  ObstacleCorrespondenceMatcher::Config cfg;
  cfg.position_tolerance = kPositionTolerance;
  cfg.point_on_line_tolerance = kPointOnLineTolerance;
  cfg.line_containment_tolerance = kLineContainmentTolerance;
  ObstacleCorrespondenceMatcher::initialize(cfg);
#ifdef USE_SIMD
  rises::geofence::ObstacleCorrespondenceMatcherSIMD::Config simd_cfg;
  simd_cfg.position_tolerance = kPositionTolerance;
  simd_cfg.point_on_line_tolerance = kPointOnLineTolerance;
  simd_cfg.line_containment_tolerance = kLineContainmentTolerance;
  rises::geofence::ObstacleCorrespondenceMatcherSIMD::initialize(simd_cfg);
#endif
}

// Build a RobotSafetyProfile with no detection-zone filtering: the matcher
// skips the profile check entirely when both outer and inner zones are
// disabled. The default ctor installs a 2m outer circle, so call
// clearOuterZone() explicitly.
RobotSafetyProfile makeUnfilteredProfile() {
  RobotSafetyProfile profile;
  profile.clearOuterZone();
  profile.clearInnerZone();
  return profile;
}

// Run both backends against every detected obstacle in the list and assert
// the match-outcome bool is identical for every input.
void expectParity(
    const GeofenceMap &map,
    const std::vector<rises_interfaces::msg::Obstacle> &detected_obstacles) {
#ifndef USE_SIMD
  GTEST_SKIP() << "USE_SIMD not enabled at build time -- SIMD path is not "
                  "compiled into this test binary.";
#else
  const Point2D robot_pos(kRobotX, kRobotY);
  const RobotSafetyProfile profile = makeUnfilteredProfile();

  for (const auto &detected : detected_obstacles) {
    const auto scalar =
        ObstacleCorrespondenceMatcher::findCorrespondingObstacle(
            map, detected, robot_pos, kRobotHeadingRad, profile);
    const auto simd = rises::geofence::ObstacleCorrespondenceMatcherSIMD::
        findCorrespondingObstacle(map, detected, robot_pos, kRobotHeadingRad,
                                  profile);
    EXPECT_EQ(scalar.found_match, simd.found_match)
        << "detected id " << detected.id << " type " << int(detected.type);
  }
#endif
}

} // namespace

TEST(CorrespondenceMatcher, ScalarAndSimdProduceIdenticalOutputForRectangles) {
  initBoth();
  auto map = makeMap();
  std::vector<rises_interfaces::msg::Obstacle> detected;
  detected.reserve(kRectangleCount);

  // Build kRectangleCount unit rectangles on a 10x10 grid. Each cell stores
  // both the map rectangle and a detected point on its edge that should match.
  for (int i = 0; i < kRectangleCount; ++i) {
    const double base_x = static_cast<double>(i % 10) * 5.0;
    const double base_y = static_cast<double>(i / 10) * 5.0;
    const auto obs = test_support::ObstacleBuilder::rectangle(
        static_cast<std::uint64_t>(i + 1), base_x, base_y, base_x + 1.0,
        base_y + 1.0);
    map->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));
    detected.push_back(test_support::ObstacleBuilder::point(
        static_cast<std::uint64_t>(10000 + i), base_x,
        base_y + 0.5)); // On left edge.
  }
  expectParity(*map, detected);
}

TEST(CorrespondenceMatcher, ScalarAndSimdProduceIdenticalOutputForLines) {
  initBoth();
  auto map = makeMap();
  std::vector<rises_interfaces::msg::Obstacle> detected;

  for (int i = 0; i < 50; ++i) {
    const double y = static_cast<double>(i) * 2.0;
    const auto line = test_support::ObstacleBuilder::line(
        static_cast<std::uint64_t>(i + 1), 0.0, y, 10.0, y);
    map->insertObstacle(static_cast<int64_t>(line.id), fromObstacleMsg(line));
    detected.push_back(test_support::ObstacleBuilder::point(
        static_cast<std::uint64_t>(20000 + i), 5.0, y));
  }
  expectParity(*map, detected);
}

TEST(CorrespondenceMatcher, ScalarAndSimdProduceIdenticalOutputForPolygons) {
  initBoth();
  auto map = makeMap();
  std::vector<rises_interfaces::msg::Obstacle> detected;

  // Hexagonal polygon centered at origin -- vertex count is not a power of
  // the SIMD width so both backends exercise their tail-edge handling.
  std::vector<geometry_msgs::msg::Point> hex_vertices;
  constexpr int kHexSides = 6;
  constexpr double kHexRadius = 5.0;
  constexpr double kTau = 6.2831853071795864;
  for (int i = 0; i < kHexSides; ++i) {
    const double angle =
        (kTau * static_cast<double>(i)) / static_cast<double>(kHexSides);
    hex_vertices.push_back(test_support::makePoint(
        kHexRadius * std::cos(angle), kHexRadius * std::sin(angle)));
  }
  const auto hex =
      test_support::ObstacleBuilder::polygon(/*id=*/1U, hex_vertices);
  map->insertObstacle(static_cast<int64_t>(hex.id), fromObstacleMsg(hex));

  // Detected points: hexagon midpoints, hexagon vertices, and points just off
  // the boundary (should not match).
  for (int i = 0; i < kHexSides; ++i) {
    const auto &a = hex_vertices[i];
    const auto &b = hex_vertices[(i + 1) % kHexSides];
    detected.push_back(test_support::ObstacleBuilder::point(
        static_cast<std::uint64_t>(30000 + i), 0.5 * (a.x + b.x),
        0.5 * (a.y + b.y)));
    detected.push_back(test_support::ObstacleBuilder::point(
        static_cast<std::uint64_t>(40000 + i), a.x, a.y));
    detected.push_back(test_support::ObstacleBuilder::point(
        static_cast<std::uint64_t>(50000 + i), 1.0 * a.x, 1.5 * a.y));
  }
  expectParity(*map, detected);
}

TEST(CorrespondenceMatcher, EmptyInputProducesEmptyOutput) {
  initBoth();
  auto map = makeMap(); // No obstacles inserted.

  const Point2D robot_pos(kRobotX, kRobotY);
  const RobotSafetyProfile profile = makeUnfilteredProfile();
  const auto detected =
      test_support::ObstacleBuilder::point(/*id=*/1U, 0.5, 0.5);

  const auto scalar = ObstacleCorrespondenceMatcher::findCorrespondingObstacle(
      *map, detected, robot_pos, kRobotHeadingRad, profile);
  EXPECT_FALSE(scalar.found_match);
  EXPECT_EQ(scalar.matched_id, -1);
}

TEST(CorrespondenceMatcher, DegenerateZeroLengthLineHandled) {
  initBoth();
  auto map = makeMap();

  // Insert a zero-length line as the map obstacle. The matcher must not
  // crash or spuriously match an off-line detected point.
  const auto degenerate = test_support::ObstacleBuilder::line(
      /*id=*/1U, /*x1=*/5.0, /*y1=*/5.0, /*x2=*/5.0, /*y2=*/5.0);
  map->insertObstacle(static_cast<int64_t>(degenerate.id),
                      fromObstacleMsg(degenerate));

  const Point2D robot_pos(kRobotX, kRobotY);
  const RobotSafetyProfile profile = makeUnfilteredProfile();
  const auto detected = test_support::ObstacleBuilder::point(
      /*id=*/2U, /*x=*/5.0, /*y=*/6.0); // 1m above the degenerate point.

  const auto scalar = ObstacleCorrespondenceMatcher::findCorrespondingObstacle(
      *map, detected, robot_pos, kRobotHeadingRad, profile);
  // A degenerate zero-length line has len_sq < 1e-6f; the scalar matcher
  // bails out at that guard and reports no match (line contains nothing).
  EXPECT_FALSE(scalar.found_match);
}

TEST(CorrespondenceMatcher, DegenerateCollinearTrianglePolygon) {
  initBoth();
  auto map = makeMap();

  // Three collinear points -- Boost.Geometry will close the polygon, but the
  // resulting shape has zero area. The matcher should still walk edges
  // without crashing; we assert it does not falsely report a match for an
  // off-line detection.
  std::vector<geometry_msgs::msg::Point> collinear = {
      test_support::makePoint(0.0, 0.0),
      test_support::makePoint(2.0, 0.0),
      test_support::makePoint(4.0, 0.0),
  };
  const auto poly =
      test_support::ObstacleBuilder::polygon(/*id=*/1U, collinear);
  map->insertObstacle(static_cast<int64_t>(poly.id), fromObstacleMsg(poly));

  const Point2D robot_pos(kRobotX, kRobotY);
  const RobotSafetyProfile profile = makeUnfilteredProfile();
  const auto detected = test_support::ObstacleBuilder::point(
      /*id=*/2U, /*x=*/2.0, /*y=*/5.0); // Off the line, no match expected.

  const auto scalar = ObstacleCorrespondenceMatcher::findCorrespondingObstacle(
      *map, detected, robot_pos, kRobotHeadingRad, profile);
  EXPECT_FALSE(scalar.found_match);
}
