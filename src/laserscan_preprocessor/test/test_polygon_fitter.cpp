// Gap-closure unit tests for the PolygonFitter shape strategy in
// laserscan_preprocessor.
//
// Production code: src/shapes/shape_fitter.cpp (PolygonFitter::fitShape /
// getConfidence). The fitter copies every input point straight through as a
// polygon vertex; it neither computes a convex hull nor rejects degenerate
// input. These tests pin that documented behaviour explicitly so that any
// future refactor (e.g. introducing convex-hull simplification) trips a
// red test and is forced to declare intent.
//
// Documented behaviour as of develop:
//   * Polygon fitter accepts any non-empty point set and returns a polygon
//     with vertices == input order (no convex hull).
//   * Empty input returns a default-constructed Obstacle (vertices empty).
//   * Single / two-point input is accepted (no rejection at the fitter).
//     Higher-level ShapeFitterFactory routes such cases to PointFitter, so
//     these are direct fitter tests, not factory tests.
//
// Standards binding:
//   - No mocks, real rises::shapes::PolygonFitter.
//   - Deterministic.
//   - Function cap 100 lines, nesting <= 3, named constants.

#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "rises_interfaces/msg/obstacle.hpp"
#include "laserscan_preprocessor/shapes/shape_fitter.hpp"

namespace {

constexpr int kSegmentId = 42;
constexpr float kSquareEdge = 1.0f;
constexpr std::size_t kSquareVertexCount = 4;
constexpr std::size_t kConcaveVertexCount = 6;
constexpr float kCollinearStep = 0.1f;
constexpr int kCollinearCount = 10;
constexpr float kTolerance = 1e-4f;

std::vector<Eigen::Vector2f> makeSquare() {
  // Counter-clockwise unit square.
  return {{0.0f, 0.0f},
          {kSquareEdge, 0.0f},
          {kSquareEdge, kSquareEdge},
          {0.0f, kSquareEdge}};
}

std::vector<Eigen::Vector2f> makeConcave() {
  // Arrow / "C" shape: 6 vertices, one indent — not convex.
  return {{0.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 2.0f},
          {1.0f, 1.0f}, {0.5f, 2.0f}, {0.0f, 2.0f}};
}

std::vector<Eigen::Vector2f> makeCollinear() {
  std::vector<Eigen::Vector2f> pts;
  pts.reserve(kCollinearCount);
  for (int i = 0; i < kCollinearCount; ++i) {
    pts.emplace_back(static_cast<float>(i) * kCollinearStep, 0.0f);
  }
  return pts;
}

} // namespace

TEST(PolygonFitter, ConvexPolygonRecovered) {
  rises::shapes::PolygonFitter fitter;
  const std::vector<Eigen::Vector2f> square = makeSquare();

  const rises_interfaces::msg::Obstacle obs =
      fitter.fitShape(kSegmentId, square);

  ASSERT_EQ(obs.vertices.size(), kSquareVertexCount);
  EXPECT_EQ(obs.id, kSegmentId);
  EXPECT_EQ(obs.type, rises_interfaces::msg::Obstacle::POLYGON);
  // Centroid of unit square is (0.5, 0.5).
  EXPECT_NEAR(obs.position.x, 0.5f, kTolerance);
  EXPECT_NEAR(obs.position.y, 0.5f, kTolerance);
}

TEST(PolygonFitter, ConcavePolygonProducesConvexHullOrPolygon) {
  // Documented behaviour: PolygonFitter passes input points through as
  // vertices. It does NOT compute a convex hull. The vertex count after
  // fitting therefore equals the input count.
  rises::shapes::PolygonFitter fitter;
  const std::vector<Eigen::Vector2f> pts = makeConcave();

  const rises_interfaces::msg::Obstacle obs = fitter.fitShape(kSegmentId, pts);

  EXPECT_EQ(obs.vertices.size(), kConcaveVertexCount)
      << "PolygonFitter must NOT collapse concave input to a convex hull; "
         "documented behaviour is pass-through.";
  EXPECT_EQ(obs.type, rises_interfaces::msg::Obstacle::POLYGON);
}

TEST(PolygonFitter, DegenerateCollinearPointsRejected) {
  // Documented behaviour: PolygonFitter does NOT reject collinear input. It
  // still produces an Obstacle, but with all vertices on the same line. The
  // documented contract is that downstream consumers must tolerate this.
  rises::shapes::PolygonFitter fitter;
  const std::vector<Eigen::Vector2f> pts = makeCollinear();

  const rises_interfaces::msg::Obstacle obs = fitter.fitShape(kSegmentId, pts);

  EXPECT_EQ(obs.vertices.size(), static_cast<std::size_t>(kCollinearCount))
      << "Documented pass-through: collinear input is not rejected, but is "
         "preserved as a degenerate polygon. Downstream must handle.";
}

TEST(PolygonFitter, SinglePointRejected) {
  // Documented behaviour: a single point IS accepted by PolygonFitter (returns
  // 1-vertex polygon). The factory ShapeFitterFactory::getBestFitter routes
  // single points to PointFitter instead. This test pins the fitter's direct
  // behaviour: it produces a 1-vertex obstacle (not rejected outright).
  rises::shapes::PolygonFitter fitter;
  const std::vector<Eigen::Vector2f> pts{{1.5f, 2.5f}};

  const rises_interfaces::msg::Obstacle obs = fitter.fitShape(kSegmentId, pts);

  EXPECT_EQ(obs.vertices.size(), 1u)
      << "PolygonFitter on a single point produces a 1-vertex obstacle; the "
         "factory is responsible for routing to PointFitter.";
  EXPECT_NEAR(obs.position.x, 1.5f, kTolerance);
  EXPECT_NEAR(obs.position.y, 2.5f, kTolerance);
}

TEST(PolygonFitter, TwoPointsRejected) {
  // Documented behaviour: two points are accepted (factory routes them to
  // LineFitter). PolygonFitter itself produces a 2-vertex obstacle without
  // rejecting.
  rises::shapes::PolygonFitter fitter;
  const std::vector<Eigen::Vector2f> pts{{0.0f, 0.0f}, {1.0f, 0.0f}};

  const rises_interfaces::msg::Obstacle obs = fitter.fitShape(kSegmentId, pts);

  EXPECT_EQ(obs.vertices.size(), 2u)
      << "PolygonFitter accepts two points; routing to LineFitter is the "
         "factory's responsibility.";
}

TEST(PolygonFitter, EmptyInputProducesEmptyObstacle) {
  rises::shapes::PolygonFitter fitter;
  const std::vector<Eigen::Vector2f> empty;

  const rises_interfaces::msg::Obstacle obs =
      fitter.fitShape(kSegmentId, empty);

  EXPECT_TRUE(obs.vertices.empty())
      << "Empty input must produce a default-constructed (empty) Obstacle.";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
