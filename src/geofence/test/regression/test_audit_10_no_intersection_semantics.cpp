// =============================================================================
// Audit Finding #10 -- "no_intersection" naming is the opposite of its
// semantics
// =============================================================================
//
// Location: geofence/spatial_node/src/queries/path_safety_checker.cpp:124-199
//
// `doesSegmentIntersectObstacle` and `doesSegmentIntersectLockedArea` both
// assign `findFirstObstacleInRegion(...)` / `findFirstAreaInRegion(...)` to a
// variable called `no_intersection` and then `return !no_intersection`. That
// is correct given the underlying semantics ("findFirst returns true when
// iteration completed without a match"), but the *name* is the opposite of
// the value: when the search exits early -- i.e. a match was found -- the
// variable is false. A future rename that "fixes the typo" by dropping the
// negation will silently invert the path-safety contract.
//
// STATUS: GREEN on develop. Unlike the other regression tests in this
// directory, this file pins the CURRENT (correct) return values. A rename-
// only refactor that flips the logic will turn it RED, which is exactly the
// signal we want.
//
// Strategy: build several path / obstacle / locked-area pairs whose ground
// truth is obvious by inspection (line through midpoint -> blocked; parallel
// path -> clear; etc.), and assert isPathSafe / isSegmentSafe return the
// expected value. If the rename inverts the predicate, every assertion
// flips.
// =============================================================================

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "test_support/obstacle_builder.hpp"

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/queries/path_safety_checker.hpp"
#include "spatial_index_selection.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "variant_shapes.hpp"

namespace {

using rises::geofence::fromObstacleMsg;
using rises::geofence::GeofenceMap;
using rises::geofence::PathSafetyChecker;
using ::Point2D;
using rises::geofence::SpatialIndex;

// Sentinel obstacle ID for the single obstacle each TEST inserts.
constexpr std::uint64_t kObstacleId = 42U;

class NoIntersectionSnapshot : public ::testing::Test {
protected:
  void SetUp() override {
    std::function<std::shared_ptr<SpatialIndex>()> factory = []() {
      return std::make_shared<SpatialIndex>();
    };
    map_ = std::make_unique<GeofenceMap>(factory);

    PathSafetyChecker::Config cfg;
    cfg.safety_margin = 0.0F; // Snapshot tests only care about intersection,
    cfg.check_map_bounds = false; // not clearance, bounds, or locked areas.
    cfg.check_locked_areas = false;
    cfg.min_path_poses = 2;
    cfg.max_path_poses = 10000;
    PathSafetyChecker::initialize(cfg);
  }

  // Build a 2-pose nav_msgs::Path. Snapshot tests use simple A->B paths so
  // the ground truth is obvious from the coordinates.
  static nav_msgs::msg::Path makePath(const Point2D &a, const Point2D &b) {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped p1;
    p1.pose.position.x = a.x;
    p1.pose.position.y = a.y;
    path.poses.push_back(p1);
    geometry_msgs::msg::PoseStamped p2;
    p2.pose.position.x = b.x;
    p2.pose.position.y = b.y;
    path.poses.push_back(p2);
    return path;
  }

  std::unique_ptr<GeofenceMap> map_;
};

} // namespace

TEST_F(NoIntersectionSnapshot, EmptyMapPathIsClear) {
  // No obstacles inserted -- any path is safe.
  EXPECT_TRUE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 0.0}, {10.0, 0.0})));
}

TEST_F(NoIntersectionSnapshot, PathThroughObstacleIsBlocked) {
  // Rectangle obstacle centred on (5,0). The path runs from (0,0) -> (10,0)
  // and crosses it.
  const auto obs = test_support::ObstacleBuilder::rectangle(
      kObstacleId, /*min_x=*/4.0, /*min_y=*/-1.0, /*max_x=*/6.0,
      /*max_y=*/1.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_FALSE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 0.0}, {10.0, 0.0})));
}

TEST_F(NoIntersectionSnapshot, PathParallelToObstacleIsClear) {
  // Same obstacle, but the path is parallel and offset by 5m -- no
  // intersection.
  const auto obs = test_support::ObstacleBuilder::rectangle(
      kObstacleId, /*min_x=*/4.0, /*min_y=*/-1.0, /*max_x=*/6.0,
      /*max_y=*/1.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_TRUE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 5.0}, {10.0, 5.0})));
}

TEST_F(NoIntersectionSnapshot, PathTouchingObstacleEdgeIsBlocked) {
  // Path runs along y=1.0 which coincides with the top edge of the
  // rectangle. Boost.Geometry "intersects" includes boundary contact, so
  // this snapshot asserts BLOCKED.
  const auto obs = test_support::ObstacleBuilder::rectangle(
      kObstacleId, /*min_x=*/4.0, /*min_y=*/-1.0, /*max_x=*/6.0,
      /*max_y=*/1.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_FALSE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 1.0}, {10.0, 1.0})));
}

TEST_F(NoIntersectionSnapshot, PathThroughPointObstacleIsBlocked) {
  // POINT obstacle exactly on the path's midpoint.
  const auto obs = test_support::ObstacleBuilder::point(kObstacleId, 5.0, 0.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_FALSE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 0.0}, {10.0, 0.0})));
}

TEST_F(NoIntersectionSnapshot, PathAvoidingPointObstacleIsClear) {
  const auto obs = test_support::ObstacleBuilder::point(kObstacleId, 5.0, 5.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_TRUE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 0.0}, {10.0, 0.0})));
}

TEST_F(NoIntersectionSnapshot, PathCrossingLineObstacleIsBlocked) {
  // Vertical line from (5,-3) to (5,3); horizontal path crosses it.
  const auto obs =
      test_support::ObstacleBuilder::line(kObstacleId, 5.0, -3.0, 5.0, 3.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_FALSE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 0.0}, {10.0, 0.0})));
}

TEST_F(NoIntersectionSnapshot, PathParallelToLineObstacleIsClear) {
  // Horizontal line at y=5; horizontal path at y=0 -- never meet.
  const auto obs =
      test_support::ObstacleBuilder::line(kObstacleId, 0.0, 5.0, 10.0, 5.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_TRUE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 0.0}, {10.0, 0.0})));
}

TEST_F(NoIntersectionSnapshot, PathThroughPolygonIsBlocked) {
  // Triangle around the origin; path runs straight through it.
  const std::vector<geometry_msgs::msg::Point> verts = {
      test_support::makePoint(4.0, -1.0),
      test_support::makePoint(6.0, -1.0),
      test_support::makePoint(5.0, 1.0),
  };
  const auto obs = test_support::ObstacleBuilder::polygon(kObstacleId, verts);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_FALSE(
      PathSafetyChecker::isPathSafe(*map_, makePath({0.0, 0.0}, {10.0, 0.0})));
}

TEST_F(NoIntersectionSnapshot, SegmentSafeApiAgreesWithPathSafeApi) {
  // Cross-check: the helper used by external callers (isSegmentSafe) must
  // agree with isPathSafe on the same geometry. A rename that inverted the
  // semantics would only catch one of the two checkers if they used
  // different helpers; this case guards against that mismatch.
  const auto obs = test_support::ObstacleBuilder::rectangle(
      kObstacleId, /*min_x=*/4.0, /*min_y=*/-1.0, /*max_x=*/6.0,
      /*max_y=*/1.0);
  map_->insertObstacle(static_cast<int64_t>(obs.id), fromObstacleMsg(obs));

  EXPECT_FALSE(
      PathSafetyChecker::isSegmentSafe(*map_, {0.0, 0.0}, {10.0, 0.0}));
  EXPECT_TRUE(PathSafetyChecker::isSegmentSafe(*map_, {0.0, 5.0}, {10.0, 5.0}));
}
