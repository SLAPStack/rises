// Regression tests for audit finding #4 (oversize obstacle / contours DoS).
//
// Bug locations:
//   message_translator/src/aabb_converter.cpp:87
//     updates.reserve(items.size());
//   message_translator/src/contours_converter.cpp:25
//     poly.points.reserve(hull_json.size());
//   message_translator/src/contours_converter.cpp:53
//     segments_out.reserve(segs_json.size());
//   message_translator/src/contours_converter.cpp:88
//     poly.points.reserve(inner.size() + 1);
//
// In every case the size argument is taken straight from an attacker-supplied
// JSON array and used to reserve a vector without bound. A maliciously large
// "pallets" / hull / segments / inner-contour array exhausts heap before any
// per-item validation runs.
//
// Expected behaviour after the fix:
//   - Each path enforces a hard cap on the JSON array size.
//   - Oversize inputs return an empty result (empty update vector or
//     header-only Contours) and log an error.
//
// Red-on-develop status:
//   *BelowCapAccepted          — GREEN today (existing converters accept).
//   *JustOverCapRejected       — RED until the fix lands.
//
// Cap values pinned here. The fix PR MUST define matching constants in
// production headers (suggested names: MAX_AABB_OBSTACLES,
// MAX_CONTOUR_HULL_POINTS, MAX_CONTOUR_INNER_POLYGONS). Divergence between
// production and this file should fail the test by design — that is the
// contract.

#include <chrono>
#include <cstddef>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "message_translator/aabb_converter.hpp"
#include "message_translator/contours_converter.hpp"
#include "test_support/oversize_msg_factory.hpp"

#ifdef __GLIBC__
#include <sys/resource.h>
#include <sys/time.h>
#endif

namespace {

constexpr std::size_t kMaxAabbObstacles = 100000;
constexpr std::size_t kMaxContourHullPoints = 100000;
constexpr std::size_t kMaxContourInnerPolygons = 100000;
constexpr const char *kFrameId = "map";

// Wall-clock budget: a correctly capped converter rejects oversize input in
// near-constant time. An unbounded reserve at the chosen cap +1 still
// allocates ~tens of megabytes; the deadline gives ample headroom for a
// healthy machine yet flags an obvious regression.
constexpr std::chrono::milliseconds kRejectDeadline{500};

// Build a contours JSON with `outer_hull_size` hull points and
// `inner_count` inner polygons; mirrors makeContoursWithVertices in shape.
std::string makeContoursJson(std::size_t outer_hull_size,
                             std::size_t inner_count) {
  nlohmann::json hull = nlohmann::json::array();
  for (std::size_t i = 0; i < outer_hull_size; ++i) {
    const float coord = 0.01f * static_cast<float>(i);
    hull.push_back({coord, coord});
  }

  nlohmann::json segments = nlohmann::json::array();
  // One degenerate segment is sufficient to satisfy required-fields checks
  // without inflating the test's own allocation footprint.
  segments.push_back({{0.0f, 0.0f}, {1.0f, 1.0f}});

  nlohmann::json inner_contours = nlohmann::json::array();
  for (std::size_t i = 0; i < inner_count; ++i) {
    nlohmann::json single_inner = nlohmann::json::array();
    single_inner.push_back({{0.0f, 0.0f}, {1.0f, 1.0f}});
    inner_contours.push_back(std::move(single_inner));
  }

  nlohmann::json root;
  root["outer_contour_hull"] = std::move(hull);
  root["outer_contour_segments"] = std::move(segments);
  root["inner_contours"] = std::move(inner_contours);
  return root.dump();
}

// Resident-set delta in bytes (Linux/glibc only). Returns 0 on platforms
// without getrusage. ru_maxrss is in kilobytes on Linux, so multiply by 1024.
long residentBytesNow() {
#ifdef __GLIBC__
  rusage usage{};
  if (::getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0;
  }
  return usage.ru_maxrss * 1024L;
#else
  return 0;
#endif
}

// Sanity check: an oversize reject must not push max-RSS up by anywhere
// near the would-be allocation size. 64 MiB is generous yet catches the
// unbounded-reserve regression for the chosen caps.
constexpr long kMaxResidentDeltaBytes = 64L * 1024L * 1024L;

} // namespace

TEST(OversizeAabb, BelowCapAccepted) {
  const std::size_t count = 1000;
  const std::string payload = test_support::oversize::makeAabbJson(count);

  const auto updates = rises::AabbConverter::parseObstacleUpdates(payload);

  EXPECT_EQ(updates.size(), count);
}

TEST(OversizeAabb, JustOverCapRejected) {
  const std::string payload =
      test_support::oversize::makeAabbJson(kMaxAabbObstacles + 1);

  const long rss_before = residentBytesNow();
  const auto start = std::chrono::steady_clock::now();
  const auto updates = rises::AabbConverter::parseObstacleUpdates(payload);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  const long rss_after = residentBytesNow();

  EXPECT_TRUE(updates.empty())
      << "Over-cap AABB payload should be rejected; got " << updates.size()
      << " updates.";
  EXPECT_LT(elapsed, kRejectDeadline)
      << "Over-cap rejection should be cheap (no per-item parse, no reserve).";
  if (rss_before > 0 && rss_after > 0) {
    EXPECT_LT(rss_after - rss_before, kMaxResidentDeltaBytes)
        << "Resident-set growth of " << (rss_after - rss_before)
        << " bytes suggests an unbounded reserve still happened.";
  }
}

TEST(OversizeContours, OuterHullBelowCapAccepted) {
  const std::size_t hull_size = 1000;
  const std::string payload = makeContoursJson(hull_size, 0);

  const auto contours = rises::ContoursConverter::parse(payload, kFrameId);

  EXPECT_EQ(contours.outer_contour_hull.points.size(), hull_size);
  EXPECT_EQ(contours.header.frame_id, kFrameId);
}

TEST(OversizeContours, OuterHullJustOverCapRejected) {
  const std::string payload = makeContoursJson(kMaxContourHullPoints + 1, 0);

  const auto start = std::chrono::steady_clock::now();
  const auto contours = rises::ContoursConverter::parse(payload, kFrameId);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_TRUE(contours.outer_contour_hull.points.empty())
      << "Over-cap hull should be rejected; got "
      << contours.outer_contour_hull.points.size() << " points.";
  EXPECT_LT(elapsed, kRejectDeadline);
}

TEST(OversizeContours, InnerCountBelowCapAccepted) {
  const std::size_t inner_count = 1000;
  const std::string payload = makeContoursJson(4, inner_count);

  const auto contours = rises::ContoursConverter::parse(payload, kFrameId);

  EXPECT_EQ(contours.inner_contours.size(), inner_count);
}

TEST(OversizeContours, InnerCountJustOverCapRejected) {
  const std::string payload = makeContoursJson(4, kMaxContourInnerPolygons + 1);

  const long rss_before = residentBytesNow();
  const auto start = std::chrono::steady_clock::now();
  const auto contours = rises::ContoursConverter::parse(payload, kFrameId);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  const long rss_after = residentBytesNow();

  EXPECT_TRUE(contours.inner_contours.empty())
      << "Over-cap inner-contour count should be rejected; got "
      << contours.inner_contours.size() << " polygons.";
  EXPECT_LT(elapsed, kRejectDeadline);
  if (rss_before > 0 && rss_after > 0) {
    EXPECT_LT(rss_after - rss_before, kMaxResidentDeltaBytes);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
