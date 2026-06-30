// =============================================================================
// Audit Finding #1 -- Inverted return in
// ObstacleCollisionChecker::checkCollision
// =============================================================================
//
// Location: geofence/spatial_node/src/queries/obstacle_collision_checker.cpp:31
//
// The current implementation returns the bool out of
// `map.findFirstObstacleInRegion(...)` directly. That call returns *true* when
// iteration completes WITHOUT a match (see geofence_map.hpp:329, "true if
// iteration completed, false if stopped early"). As a result, `checkCollision`
// reports "no collision" when one exists, and vice versa.
//
// Expected fix: negate the return value, e.g.
//     return !map.findFirstObstacleInRegion(search_box, ...);
//
// STATUS: RED on develop. The tests below assert post-fix semantics and are
// expected to FAIL until the fix lands.
// =============================================================================

#include <gtest/gtest.h>

#include <functional>
#include <memory>

#include "test_support/obstacle_builder.hpp"

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/queries/obstacle_collision_checker.hpp"
#include "spatial_index_selection.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"

namespace {

using rises::geofence::fromObstacleMsg;
using rises::geofence::GeofenceMap;
using rises::geofence::Geometry;
using rises::geofence::ObstacleCollisionChecker;
using rises::geofence::SpatialIndex;

// Default 5cm collision margin per ObstacleCollisionChecker::Config defaults.
constexpr float kCollisionMargin = 0.05F;

std::unique_ptr<GeofenceMap> makeMap() {
  std::function<std::shared_ptr<SpatialIndex>()> factory = []() {
    return std::make_shared<SpatialIndex>();
  };
  auto map = std::make_unique<GeofenceMap>(factory);
  ObstacleCollisionChecker::Config cfg;
  cfg.collision_margin = kCollisionMargin;
  ObstacleCollisionChecker::initialize(cfg);
  return map;
}

} // namespace

TEST(CollisionInverted, ObstacleAtKnownLocationReportsCollision) {
  auto map = makeMap();

  const auto stored = test_support::ObstacleBuilder::rectangle(
      /*id=*/1U, /*min_x=*/0.0, /*min_y=*/0.0, /*max_x=*/1.0, /*max_y=*/1.0);
  map->insertObstacle(static_cast<int64_t>(stored.id), fromObstacleMsg(stored));

  // Query obstacle fully overlaps the stored rectangle -- a true collision.
  const auto query = test_support::ObstacleBuilder::rectangle(
      /*id=*/2U, /*min_x=*/0.25, /*min_y=*/0.25, /*max_x=*/0.75,
      /*max_y=*/0.75);

  EXPECT_TRUE(ObstacleCollisionChecker::checkCollision(*map, query));
}

TEST(CollisionInverted, EmptyMapReportsNoCollision) {
  auto map = makeMap();

  const auto query = test_support::ObstacleBuilder::rectangle(
      /*id=*/1U, /*min_x=*/0.0, /*min_y=*/0.0, /*max_x=*/1.0, /*max_y=*/1.0);

  EXPECT_FALSE(ObstacleCollisionChecker::checkCollision(*map, query));
}

TEST(CollisionInverted, MarginEdgeReportsCollision) {
  auto map = makeMap();

  const auto stored = test_support::ObstacleBuilder::rectangle(
      /*id=*/1U, /*min_x=*/0.0, /*min_y=*/0.0, /*max_x=*/1.0, /*max_y=*/1.0);
  map->insertObstacle(static_cast<int64_t>(stored.id), fromObstacleMsg(stored));

  // Query rectangle overlaps the (1,1) corner of the stored obstacle.
  const auto query = test_support::ObstacleBuilder::rectangle(
      /*id=*/2U, /*min_x=*/0.95, /*min_y=*/0.95, /*max_x=*/1.05,
      /*max_y=*/1.05);

  EXPECT_TRUE(ObstacleCollisionChecker::checkCollision(*map, query));
}

TEST(CollisionInverted, LongWallFarFromItsCentroidReportsCollision) {
  // Hardening for the spatial-index radius fix: a large size disparity between
  // the stored obstacle and the query box. A 20m x 0.2m "wall" spanning
  // x in [0, 20] has its centroid at (10, 0.1), far from either end. The query
  // box touches only the far (x ~ 20) end -- nowhere near the wall's centroid.
  // A centroid-only KD-tree radius search would miss this overlap; the fix
  // inflates the search radius by the largest stored obstacle's half-diagonal,
  // so the wall is still found.
  std::unique_ptr<GeofenceMap> map = makeMap();

  const rises_interfaces::msg::Obstacle wall =
      test_support::ObstacleBuilder::rectangle(
          /*id=*/1U, /*min_x=*/0.0, /*min_y=*/0.0, /*max_x=*/20.0,
          /*max_y=*/0.2);
  map->insertObstacle(static_cast<int64_t>(wall.id), fromObstacleMsg(wall));

  // Query box overlaps only the far end of the wall (x ~ 19.9..20.1).
  const rises_interfaces::msg::Obstacle query =
      test_support::ObstacleBuilder::rectangle(
          /*id=*/2U, /*min_x=*/19.9, /*min_y=*/0.0, /*max_x=*/20.1,
          /*max_y=*/0.2);

  EXPECT_TRUE(ObstacleCollisionChecker::checkCollision(*map, query));
}
