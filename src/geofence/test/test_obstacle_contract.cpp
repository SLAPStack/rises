// =============================================================================
// Contract tests for rises_interfaces::msg::Obstacle field semantics.
//
// These pin behaviour that is ALREADY TRUE today; they are regression guards,
// not a proposal to change anything. In particular they document a deliberate
// divergence in how two consumers read a POINT-type obstacle's reference point:
//
//   * GeometryIntersection::intersectsCircle (safety-circle / footprint checks,
//     geofence/utils/src/geometry_intersection.cpp) reads `.position`.
//   * fromObstacleMsg (variant geometry construction,
//     geofence/common/src/geometry/variant_geometry.cpp) reads
//     `.vertices[0]`.
//
// They are DIFFERENT fields by design, for different purposes. Real production
// POINT obstacles (e.g. from laserscan_preprocessor) set BOTH to the same
// coordinates, so the two readers agree in practice -- but each reader's chosen
// field is a contract this test locks down. This also regression-guards the
// test_support::ObstacleBuilder::point() fix that now populates both fields.
//
// Drives the readers directly; no lifecycle / executor involved.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_array.hpp>

#include "test_support/obstacle_builder.hpp"

#include "geofence/spatial/node/obstacle_report_builder.hpp"
#include "geofence/spatial/queries/obstacle_match_result.hpp"
#include "geofence/utils/geometry_intersection.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"

namespace {

using rises::geofence::fromObstacleMsg;
using rises::geofence::Geometry;
using rises::geofence::GeometryType;
using rises::geofence::getPosition;
using rises::geofence::getType;
using rises::geofence::utils::GeometryIntersection;

constexpr double kEps = 1e-9;

} // namespace

// intersectsCircle's POINT case reads `.position`. A zero-radius circle exactly
// at .position must intersect; one at .vertices[0] (when that differs) must not.
TEST(ObstacleContract, IntersectsCircleReadsPositionForPoint) {
  rises_interfaces::msg::Obstacle obs;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.position.x = 3.0;
  obs.position.y = 4.0;
  // Deliberately put vertices[0] somewhere else to prove which field is read.
  geometry_msgs::msg::Point v;
  v.x = -100.0;
  v.y = -100.0;
  obs.vertices.push_back(v);

  // Circle centred on .position intersects (point is within radius).
  EXPECT_TRUE(GeometryIntersection::intersectsCircle(obs, 3.0, 4.0, 0.5));
  // Circle centred on .vertices[0] does NOT -- confirming .position is the
  // canonical reference for this reader.
  EXPECT_FALSE(GeometryIntersection::intersectsCircle(obs, -100.0, -100.0, 0.5));
}

// fromObstacleMsg's POINT case reads `.vertices[0]`. The constructed geometry's
// position must equal vertices[0], not .position, when the two differ.
TEST(ObstacleContract, FromObstacleMsgReadsFirstVertexForPoint) {
  rises_interfaces::msg::Obstacle obs;
  obs.type = rises_interfaces::msg::Obstacle::POINT;
  obs.position.x = -100.0;
  obs.position.y = -100.0;
  geometry_msgs::msg::Point v;
  v.x = 3.0;
  v.y = 4.0;
  obs.vertices.push_back(v);

  const Geometry geom = fromObstacleMsg(obs);
  EXPECT_EQ(getType(geom), GeometryType::Point);

  const Point2D pos = getPosition(geom);
  EXPECT_NEAR(pos.x, 3.0, kEps);
  EXPECT_NEAR(pos.y, 4.0, kEps);
}

// Regression guard for ObstacleBuilder::point(): it must populate BOTH .position
// and .vertices[0] with the same coordinates, so the two divergent readers above
// agree (the bug was that only .vertices was set, leaving .position at (0,0,0)).
TEST(ObstacleContract, BuilderPointPopulatesBothPositionAndVertex) {
  const rises_interfaces::msg::Obstacle obs =
      test_support::ObstacleBuilder::point(/*id=*/7U, /*x=*/3.0, /*y=*/4.0);

  ASSERT_EQ(obs.type, rises_interfaces::msg::Obstacle::POINT);

  // .position is set (this is what intersectsCircle reads).
  EXPECT_NEAR(obs.position.x, 3.0, kEps);
  EXPECT_NEAR(obs.position.y, 4.0, kEps);

  // .vertices[0] is set to the same coordinates (this is what fromObstacleMsg
  // reads).
  ASSERT_FALSE(obs.vertices.empty());
  EXPECT_NEAR(obs.vertices[0].x, 3.0, kEps);
  EXPECT_NEAR(obs.vertices[0].y, 4.0, kEps);

  // Both readers therefore agree on the reference point.
  EXPECT_TRUE(GeometryIntersection::intersectsCircle(obs, 3.0, 4.0, 0.1));
  const Point2D pos = getPosition(fromObstacleMsg(obs));
  EXPECT_NEAR(pos.x, 3.0, kEps);
  EXPECT_NEAR(pos.y, 4.0, kEps);
}

// =============================================================================
// Contract tests for the LINE-type `width` field set by ObstacleReportBuilder.
//
// width is the GROUP's transverse extent (max perpendicular deviation off the
// group's own first->last chord), computed once per pre-Douglas-Peucker point
// group and assigned to every LINE segment DP emits from that group -- not the
// individual final segment's endpoint distance. This is what makes
// rises_leg_filter's width gate ([0.15, 0.8] m by default) meaningful: a flat
// wall fragment has near-zero transverse spread, a leg-shaped blob does not.
// =============================================================================

namespace {

using rises::ObstacleReportBuilder;
using rises::geofence::query::ObstacleMatchResult;

// Builds a single-obstacle ObstacleArray whose vertices are exactly `points`,
// all marked unmatched, ready to feed to ObstacleReportBuilder::buildReport.
rises_interfaces::msg::ObstacleArray
makeUnmatchedScanGroup(const std::vector<geometry_msgs::msg::Point> &points) {
  rises_interfaces::msg::Obstacle src;
  src.type = rises_interfaces::msg::Obstacle::POLYGON;
  src.vertices = points;

  rises_interfaces::msg::ObstacleArray array;
  array.obstacles.push_back(src);
  array.angle_increment = 0.001745f;
  return array;
}

// Classifies every vertex of the single source obstacle as unmatched.
ObstacleMatchResult makeAllUnmatchedResult(std::size_t vertex_count) {
  ObstacleMatchResult result;
  result.unmatched_obstacle_indices.push_back(0);
  result.matched_vertices_per_obstacle.emplace_back();
  std::vector<std::size_t> unmatched_indices(vertex_count);
  for (std::size_t i = 0; i < vertex_count; ++i) {
    unmatched_indices[i] = i;
  }
  result.unmatched_vertices_per_obstacle.push_back(unmatched_indices);
  return result;
}

ObstacleReportBuilder::Config makeTestConfig() {
  ObstacleReportBuilder::Config config;
  // Keep every point in one group regardless of spacing/range, and disable
  // the outlier pass so test geometry is not pruned before width is computed.
  config.segment_min_gap = 100.0f;
  config.segment_gap_multiplier = 0.0f;
  config.outlier_filter_distance = 0.0f;
  config.min_segment_points = 3;
  config.enable_error_segment_tracking = false;
  return config;
}

} // namespace

// A curved/bent group (not collinear) must report width > 0, matching the
// expected max perpendicular distance from the first->last chord.
TEST(ObstacleWidthContract, CurvedGroupProducesPositiveWidth) {
  ObstacleReportBuilder::Config config = makeTestConfig();
  config.line_fit_tolerance = 10.0f; // generous: force a single LINE segment
  ObstacleReportBuilder builder(config);

  // Chord is (0,0)->(2,0) along the x-axis; the midpoint is bent up to
  // y=0.3, giving an expected perpendicular distance of exactly 0.3.
  std::vector<geometry_msgs::msg::Point> points = {
      test_support::makePoint(0.0, 0.0), test_support::makePoint(1.0, 0.3),
      test_support::makePoint(2.0, 0.0)};

  const rises_interfaces::msg::ObstacleArray::SharedPtr msg =
      std::make_shared<rises_interfaces::msg::ObstacleArray>(
          makeUnmatchedScanGroup(points));
  const ObstacleMatchResult result = makeAllUnmatchedResult(points.size());

  const rises_interfaces::msg::ObstacleReport &report = builder.buildReport(
      msg, result, /*tf_buffer=*/nullptr, rclcpp::get_logger("test"));

  ASSERT_FALSE(report.unmatched_obstacles.empty());
  for (const rises_interfaces::msg::Obstacle &seg : report.unmatched_obstacles) {
    EXPECT_NEAR(seg.width, 0.3f, 1e-4f);
  }
}

// A perfectly collinear group must report width ~= 0: no point deviates from
// the first->last chord.
TEST(ObstacleWidthContract, CollinearGroupProducesZeroWidth) {
  ObstacleReportBuilder::Config config = makeTestConfig();
  config.line_fit_tolerance = 10.0f;
  ObstacleReportBuilder builder(config);

  std::vector<geometry_msgs::msg::Point> points = {
      test_support::makePoint(0.0, 0.0), test_support::makePoint(1.0, 0.0),
      test_support::makePoint(2.0, 0.0)};

  const rises_interfaces::msg::ObstacleArray::SharedPtr msg =
      std::make_shared<rises_interfaces::msg::ObstacleArray>(
          makeUnmatchedScanGroup(points));
  const ObstacleMatchResult result = makeAllUnmatchedResult(points.size());

  const rises_interfaces::msg::ObstacleReport &report = builder.buildReport(
      msg, result, /*tf_buffer=*/nullptr, rclcpp::get_logger("test"));

  ASSERT_FALSE(report.unmatched_obstacles.empty());
  for (const rises_interfaces::msg::Obstacle &seg : report.unmatched_obstacles) {
    EXPECT_NEAR(seg.width, 0.0f, 1e-4f);
  }
}

// When Douglas-Peucker splits one curved group into multiple final LINE
// segments, every resulting segment must report the SAME group-level width,
// not a smaller per-segment value (the bug this fix prevents: a leg's curved
// arc split into near-straight sub-pieces that would each individually
// measure near-zero).
TEST(ObstacleWidthContract, SplitGroupSegmentsShareGroupWidth) {
  ObstacleReportBuilder::Config config = makeTestConfig();
  // Tight tolerance: forces Douglas-Peucker to split this curved group into
  // more than one final LINE segment, since no single chord stays within
  // 0.01 m of every intermediate point.
  config.line_fit_tolerance = 0.01f;
  ObstacleReportBuilder builder(config);

  // A gently bowed arc: each interior point sits progressively farther off
  // the (0,0)->(4,0) chord, peaking at the midpoint (matches a human leg's
  // short curved profile better than a single sharp bend).
  std::vector<geometry_msgs::msg::Point> points = {
      test_support::makePoint(0.0, 0.0),  test_support::makePoint(1.0, 0.18),
      test_support::makePoint(2.0, 0.30), test_support::makePoint(3.0, 0.18),
      test_support::makePoint(4.0, 0.0)};

  const rises_interfaces::msg::ObstacleArray::SharedPtr msg =
      std::make_shared<rises_interfaces::msg::ObstacleArray>(
          makeUnmatchedScanGroup(points));
  const ObstacleMatchResult result = makeAllUnmatchedResult(points.size());

  const rises_interfaces::msg::ObstacleReport &report = builder.buildReport(
      msg, result, /*tf_buffer=*/nullptr, rclcpp::get_logger("test"));

  // The tight tolerance must actually have forced a split; otherwise this
  // test is not exercising the multi-segment claim it makes.
  ASSERT_GT(report.unmatched_obstacles.size(), 1U)
      << "Expected Douglas-Peucker to split this group into multiple "
         "segments at line_fit_tolerance=0.01; tune the test geometry if "
         "this no longer splits.";

  const float expected_width = report.unmatched_obstacles.front().width;
  EXPECT_NEAR(expected_width, 0.3f, 1e-4f);
  for (const rises_interfaces::msg::Obstacle &seg : report.unmatched_obstacles) {
    EXPECT_NEAR(seg.width, expected_width, 1e-4f);
  }
}
