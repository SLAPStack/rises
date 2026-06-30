// Gap-closure unit tests for the PointFitter shape strategy in
// laserscan_preprocessor.
//
// Production code: src/shapes/shape_fitter.cpp (PointFitter::fitShape /
// getConfidence).
//
// Documented behaviour as of develop:
//   * Single input point -> POINT obstacle with one vertex at that point.
//   * In points_only_mode all input points become individual vertices.
//   * Out of points_only_mode and with size() > 1 the obstacle carries the
//     centroid as the only vertex.
//   * Empty input returns a default-constructed Obstacle whose vertices are
//     empty (the implementation early-returns without filling vertices).
//
// Standards binding:
//   - No mocks, real rises::shapes::PointFitter.
//   - Deterministic.
//   - Function cap 100 lines, nesting <= 3, named constants.

#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "rises_interfaces/msg/obstacle.hpp"
#include "laserscan_preprocessor/shapes/shape_fitter.hpp"

namespace {

constexpr int kSegmentId = 7;
constexpr float kTolerance = 1e-4f;
constexpr float kPx = 0.5f;
constexpr float kPy = -1.25f;
constexpr float kQx = 2.0f;
constexpr float kQy = 3.0f;

} // namespace

TEST(PointFitter, SinglePointAccepted) {
  rises::shapes::PointFitter fitter; // default: not points_only_mode
  const std::vector<Eigen::Vector2f> pts{{kPx, kPy}};

  const rises_interfaces::msg::Obstacle obs = fitter.fitShape(kSegmentId, pts);

  ASSERT_EQ(obs.vertices.size(), 1u);
  EXPECT_EQ(obs.id, kSegmentId);
  EXPECT_EQ(obs.type, rises_interfaces::msg::Obstacle::POINT);
  EXPECT_NEAR(obs.vertices.front().x, kPx, kTolerance);
  EXPECT_NEAR(obs.vertices.front().y, kPy, kTolerance);
  EXPECT_NEAR(obs.position.x, kPx, kTolerance);
  EXPECT_NEAR(obs.position.y, kPy, kTolerance);
}

TEST(PointFitter, TwoPointsBehaviour) {
  // Documented behaviour: in non-points-only mode with size() > 1, the
  // obstacle carries the centroid as a single vertex. Caller can opt into
  // points-only mode to keep both vertices.
  rises::shapes::PointFitter fitter(/*points_only_mode=*/false);
  const std::vector<Eigen::Vector2f> pts{{kPx, kPy}, {kQx, kQy}};

  const rises_interfaces::msg::Obstacle obs = fitter.fitShape(kSegmentId, pts);

  EXPECT_EQ(obs.type, rises_interfaces::msg::Obstacle::POINT);
  EXPECT_EQ(obs.vertices.size(), 1u)
      << "Documented behaviour: two points collapse to centroid vertex in "
         "non-points-only mode.";
  const float expected_cx = (kPx + kQx) / 2.0f;
  const float expected_cy = (kPy + kQy) / 2.0f;
  EXPECT_NEAR(obs.position.x, expected_cx, kTolerance);
  EXPECT_NEAR(obs.position.y, expected_cy, kTolerance);

  rises::shapes::PointFitter points_only_fitter(/*points_only_mode=*/true);
  const rises_interfaces::msg::Obstacle obs_po =
      points_only_fitter.fitShape(kSegmentId, pts);
  EXPECT_EQ(obs_po.vertices.size(), 2u)
      << "points_only_mode preserves each input point as a vertex.";
}

TEST(PointFitter, EmptyInputRejected) {
  rises::shapes::PointFitter fitter;
  const std::vector<Eigen::Vector2f> empty;

  const rises_interfaces::msg::Obstacle obs =
      fitter.fitShape(kSegmentId, empty);

  EXPECT_TRUE(obs.vertices.empty())
      << "Empty input must not populate vertices (early-return contract).";
  EXPECT_EQ(obs.id, kSegmentId)
      << "Obstacle id still propagated even on rejection.";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
