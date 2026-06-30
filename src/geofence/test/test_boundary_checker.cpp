// =============================================================================
// Gap-closure unit tests for BoundaryChecker.
//
// Source under test:
//   geofence/spatial_node/src/queries/boundary_checker.cpp
//   geofence/spatial_node/include/geofence/spatial/queries/boundary_checker.hpp
//
// BoundaryChecker delegates to MapBoundaryContours::isPointInside, which
// uses ray casting on the outer polygon and excludes points that fall inside
// any inner contour. Tests below cover inside / outside, on-edge, empty
// hull (no contours set), and a concave outer polygon.
// =============================================================================

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <vector>

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/queries/boundary_checker.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include "spatial_index_selection.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"

namespace {

using rises::geofence::BoundaryChecker;
using rises::geofence::GeofenceMap;
using ::Point2D;
using rises::geofence::SpatialIndex;
using rises::shape::MapBoundaryContours;
using rises::shape::PolygonContour;

std::unique_ptr<GeofenceMap> makeMap() {
  std::function<std::shared_ptr<SpatialIndex>()> factory = []() {
    return std::make_shared<SpatialIndex>();
  };
  return std::make_unique<GeofenceMap>(factory);
}

// 10 x 10 square centred at origin, counter-clockwise winding.
PolygonContour squareHull() {
  return PolygonContour({{-5.0, -5.0}, {5.0, -5.0}, {5.0, 5.0}, {-5.0, 5.0}});
}

} // namespace

TEST(BoundaryChecker, PointInsideHullReturnsInside) {
  auto map = makeMap();
  MapBoundaryContours contours(squareHull());
  map->setMapContours(contours);

  EXPECT_TRUE(BoundaryChecker::isPointInsideMapBounds(*map, Point2D{0.0, 0.0}));
  EXPECT_TRUE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{2.5, -1.5}));
}

TEST(BoundaryChecker, PointOutsideHullReturnsOutside) {
  auto map = makeMap();
  MapBoundaryContours contours(squareHull());
  map->setMapContours(contours);

  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{10.0, 10.0}));
  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{-5.01, 0.0}));
}

TEST(BoundaryChecker, PointOnEdgeIsDocumentedBehaviour) {
  // PolygonContour::containsPoint uses a strict ray-cast test:
  // `(vi.y > p.y) != (vj.y > p.y)` -- the half-open comparison treats
  // top-edge points and right-edge points as OUTSIDE, while bottom-edge and
  // left-edge points read as INSIDE. This test pins that convention.
  auto map = makeMap();
  MapBoundaryContours contours(squareHull());
  map->setMapContours(contours);

  // Bottom edge (y == -5): both adjacent vertices have y == -5, so the
  // half-open test never flips state -> reported as OUTSIDE.
  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{0.0, -5.0}));
  // Top edge (y == 5): same logic -> OUTSIDE.
  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{0.0, 5.0}));
  // Right edge (x == 5, y in interior): the y-window flips once but the
  // x < ... check fails when p.x equals the projected x. Reported OUTSIDE.
  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{5.0, 0.0}));
}

TEST(BoundaryChecker, EmptyHullRejectsAllPoints) {
  // The header says "Returns true if no boundary contours are defined". An
  // empty hull (vertices.size() < 3) is a contour that exists but is too
  // small to be a polygon; containsPoint returns false -> isPointInside
  // returns false. This is distinct from "no contours set at all".
  auto map = makeMap();
  MapBoundaryContours contours(PolygonContour{}); // Empty outer.
  map->setMapContours(contours);

  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{0.0, 0.0}));
  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{100.0, 100.0}));
}

TEST(BoundaryChecker, ConcaveHullHandledCorrectly) {
  // L-shaped (concave) outer hull. The "notch" at (2.5, 2.5) is OUTSIDE the
  // polygon; a point in the long arm (1.0, 1.0) is INSIDE. Ray casting
  // handles both convex and concave polygons.
  auto map = makeMap();
  PolygonContour l_shape(
      {{0.0, 0.0}, {5.0, 0.0}, {5.0, 2.0}, {2.0, 2.0}, {2.0, 5.0}, {0.0, 5.0}});
  MapBoundaryContours contours(l_shape);
  map->setMapContours(contours);

  EXPECT_TRUE(BoundaryChecker::isPointInsideMapBounds(*map, Point2D{1.0, 1.0}));
  EXPECT_TRUE(BoundaryChecker::isPointInsideMapBounds(*map, Point2D{1.0, 4.0}));
  // (3.0, 3.0) lies in the cut-out of the L -> OUTSIDE.
  EXPECT_FALSE(
      BoundaryChecker::isPointInsideMapBounds(*map, Point2D{3.0, 3.0}));
}
