// Smoke tests for the test_support helpers. Verifies the headers compile,
// the helpers instantiate, and the basic invariants hold (monotonic clock,
// non-empty oversize messages, temp file lifecycle). Production logic in
// other packages tests this library implicitly via use.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "test_support/fake_clock.hpp"
#include "test_support/lifecycle_harness.hpp"
#include "test_support/obstacle_builder.hpp"
#include "test_support/oversize_msg_factory.hpp"
#include "test_support/temp_json.hpp"

namespace {

constexpr std::size_t kSmallCount = 8;
constexpr std::size_t kMediumCount = 32;

} // namespace

TEST(FakeClock, StartsAtZeroByDefault) {
  test_support::FakeClock clock;
  EXPECT_EQ(clock.now().time_since_epoch().count(), 0);
}

TEST(FakeClock, AdvanceIsMonotonic) {
  test_support::FakeClock clock;
  const auto t0 = clock.now();
  clock.advance(std::chrono::milliseconds{5});
  const auto t1 = clock.now();
  clock.advance(std::chrono::milliseconds{10});
  const auto t2 = clock.now();
  EXPECT_LT(t0, t1);
  EXPECT_LT(t1, t2);
  EXPECT_EQ((t2 - t0), std::chrono::milliseconds{15});
}

TEST(FakeClock, SetIsAbsolute) {
  test_support::FakeClock clock{std::chrono::seconds{100}};
  clock.set(std::chrono::seconds{42});
  EXPECT_EQ(clock.now().time_since_epoch(), std::chrono::seconds{42});
}

TEST(FakeClock, ConcurrentReadersSeeProgression) {
  test_support::FakeClock clock;
  std::atomic<bool> stop{false};
  std::atomic<std::int64_t> last_seen_ns{0};

  std::thread reader{[&]() {
    while (!stop.load(std::memory_order_acquire)) {
      const auto v = clock.now().time_since_epoch().count();
      last_seen_ns.store(v, std::memory_order_release);
    }
  }};
  for (int i = 0; i < 100; ++i) {
    clock.advance(std::chrono::microseconds{1});
  }
  stop.store(true, std::memory_order_release);
  reader.join();

  EXPECT_GE(last_seen_ns.load(), 0);
  EXPECT_LE(last_seen_ns.load(), clock.now().time_since_epoch().count());
}

TEST(ObstacleBuilder, PointHasOneVertex) {
  const auto obs = test_support::ObstacleBuilder::point(7, 1.0, 2.0);
  EXPECT_EQ(obs.id, 7u);
  EXPECT_EQ(obs.type, rises_interfaces::msg::Obstacle::POINT);
  ASSERT_EQ(obs.vertices.size(), 1u);
  EXPECT_DOUBLE_EQ(obs.vertices[0].x, 1.0);
  EXPECT_DOUBLE_EQ(obs.vertices[0].y, 2.0);
}

TEST(ObstacleBuilder, RectanglePositionIsCentroid) {
  const auto obs =
      test_support::ObstacleBuilder::rectangle(11, -1.0, -2.0, 3.0, 4.0);
  EXPECT_EQ(obs.type, rises_interfaces::msg::Obstacle::RECTANGLE);
  EXPECT_DOUBLE_EQ(obs.position.x, 1.0);
  EXPECT_DOUBLE_EQ(obs.position.y, 1.0);
  EXPECT_FLOAT_EQ(obs.width, 4.0f);
  EXPECT_FLOAT_EQ(obs.height, 6.0f);
  EXPECT_EQ(obs.vertices.size(), 4u);
}

TEST(OversizeFactory, ObstacleUpdateArraySizeMatches) {
  const auto msg =
      test_support::oversize::makeObstacleUpdateArray(kMediumCount);
  EXPECT_EQ(msg.updates.size(), kMediumCount);
  EXPECT_EQ(msg.updates.front().obstacle.id, 0u);
  EXPECT_EQ(msg.updates.back().obstacle.id, kMediumCount - 1);
}

TEST(OversizeFactory, ContoursVertexAndPolygonCount) {
  const auto msg = test_support::oversize::makeContoursWithVertices(
      kMediumCount, kSmallCount);
  EXPECT_EQ(msg.outer_contour_hull.points.size(), kMediumCount);
  EXPECT_EQ(msg.inner_contours.size(), kSmallCount);
}

TEST(OversizeFactory, Vda5050JsonRoundTrips) {
  const std::string s =
      test_support::oversize::makeVda5050OrderJson(kSmallCount);
  EXPECT_NE(s.find("\"nodes\":["), std::string::npos);
  EXPECT_NE(s.find("\"nodeId\":\"n0\""), std::string::npos);
  EXPECT_NE(s.find("\"nodeId\":\"n7\""), std::string::npos);
}

TEST(OversizeFactory, AabbJsonHasExpectedCount) {
  const std::string s = test_support::oversize::makeAabbJson(kSmallCount);
  std::size_t pos = 0;
  std::size_t count = 0;
  while ((pos = s.find("\"id\":", pos)) != std::string::npos) {
    ++count;
    ++pos;
  }
  EXPECT_EQ(count, kSmallCount);
}

TEST(TempJsonFile, FileExistsDuringLifetime) {
  std::string path;
  {
    test_support::TempJsonFile f{"{\"hello\":\"world\"}"};
    path = f.path();
    EXPECT_TRUE(std::filesystem::exists(path));
  }
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(TempJsonFile, ContentMatches) {
  const std::string payload = "{\"x\":1,\"y\":[1,2,3]}";
  test_support::TempJsonFile f{payload};
  std::ifstream in(f.path());
  const std::string read{(std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>()};
  EXPECT_EQ(read, payload);
}

TEST(LifecycleHelpers, StatePredicatesMatchEnum) {
  EXPECT_TRUE(test_support::isActive(
      test_support::LifecycleState::PRIMARY_STATE_ACTIVE));
  EXPECT_TRUE(test_support::isInactive(
      test_support::LifecycleState::PRIMARY_STATE_INACTIVE));
  EXPECT_TRUE(test_support::isUnconfigured(
      test_support::LifecycleState::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_TRUE(test_support::configureFailed(
      test_support::LifecycleState::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_TRUE(test_support::activateFailed(
      test_support::LifecycleState::PRIMARY_STATE_INACTIVE));
}
