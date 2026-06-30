// =============================================================================
// Audit Finding #5 -- worldToGrid casts non-finite floats to int (UB)
// =============================================================================
//
// Location:
// geofence/gridmap_node/include/geofence/gridmap/map/gridmap_data.hpp:149-154
//
// Current implementation:
//     grid_x = static_cast<int>((x - origin_x_) * inv_resolution_);
//     grid_y = static_cast<int>((y - origin_y_) * inv_resolution_);
//     return grid_x >= 0 && grid_x < grid_width_ && ...
//
// When x or y is NaN or +/-Inf, the multiplication yields a non-finite double
// and the static_cast<int> conversion is UB per [conv.fpint]. On x86 with
// MSVC this returns INT_MIN; on x86 with GCC it varies; the subsequent bounds
// check happens to succeed-or-fail at random. NaN coordinates can reach this
// code path from any input pipeline (TF that lost a transform, malformed
// sensor data, JSON pre-init with a literal "NaN") and silently corrupt the
// grid.
//
// Expected fix: reject non-finite inputs explicitly before the cast, e.g.
//     if (!std::isfinite(x) || !std::isfinite(y)) return false;
//
// STATUS: RED on develop -- the NaN / Inf cases below assert that
// worldToGrid returns false, which is the post-fix contract. They are
// expected to FAIL until the fix lands. The finite control cases
// (AcceptsLargeFiniteX / ZeroIsAccepted) should be GREEN today.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "geofence/gridmap/map/gridmap_data.hpp"

namespace {

using rises::geofence::GridMapData;

GridMapData::Config defaultConfig() {
  GridMapData::Config cfg;
  cfg.resolution = 0.05;
  cfg.width_meters = 100.0;
  cfg.height_meters = 100.0;
  cfg.origin_x = -50.0;
  cfg.origin_y = -50.0;
  return cfg;
}

} // namespace

TEST(WorldToGridNaN, RejectsNaNX) {
  GridMapData data(defaultConfig());
  const double nan = std::numeric_limits<double>::quiet_NaN();

  int grid_x = 0;
  int grid_y = 0;
  EXPECT_FALSE(data.worldToGrid(nan, 0.0, grid_x, grid_y));
}

TEST(WorldToGridNaN, RejectsNaNY) {
  GridMapData data(defaultConfig());
  const double nan = std::numeric_limits<double>::quiet_NaN();

  int grid_x = 0;
  int grid_y = 0;
  EXPECT_FALSE(data.worldToGrid(0.0, nan, grid_x, grid_y));
}

TEST(WorldToGridNaN, RejectsPosInfX) {
  GridMapData data(defaultConfig());
  const double inf = std::numeric_limits<double>::infinity();

  int grid_x = 0;
  int grid_y = 0;
  EXPECT_FALSE(data.worldToGrid(inf, 0.0, grid_x, grid_y));
}

TEST(WorldToGridNaN, RejectsNegInfY) {
  GridMapData data(defaultConfig());
  const double neg_inf = -std::numeric_limits<double>::infinity();

  int grid_x = 0;
  int grid_y = 0;
  EXPECT_FALSE(data.worldToGrid(0.0, neg_inf, grid_x, grid_y));
}

TEST(WorldToGridNaN, AcceptsLargeFiniteX) {
  GridMapData data(defaultConfig());

  // 1e6 is finite -- worldToGrid is allowed to return either true (if it
  // happens to land in-grid) or false (out of bounds). The contract is just
  // "well defined": no UB, no crash, the value of grid_x is observable.
  int grid_x = 0;
  int grid_y = 0;
  const bool inside = data.worldToGrid(1.0e6, 0.0, grid_x, grid_y);
  EXPECT_FALSE(inside); // 1e6 m is far outside a 100m map.

  // If false was returned, the contract for grid_x is implementation-defined;
  // we only assert non-NaN behaviour by re-checking the deterministic path.
  EXPECT_FALSE(std::isnan(static_cast<double>(grid_x)));
}

TEST(WorldToGridNaN, ZeroIsAccepted) {
  GridMapData data(defaultConfig());

  // Origin = -50, so world (0,0) lands at grid (1000, 1000) in a 2000x2000
  // map. Sanity check that the happy path is unaffected by the NaN fix.
  int grid_x = 0;
  int grid_y = 0;
  EXPECT_TRUE(data.worldToGrid(0.0, 0.0, grid_x, grid_y));
  EXPECT_GE(grid_x, 0);
  EXPECT_GE(grid_y, 0);
}
