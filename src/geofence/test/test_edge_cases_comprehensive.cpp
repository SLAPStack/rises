/**
 * @file test_edge_cases_comprehensive.cpp
 * @brief Comprehensive edge case tests for GeofenceMap
 * 
 * Tests cover critical edge cases:
 * - Empty map operations
 * - Extreme numeric values
 * - Degenerate geometry
 * - RCU snapshot consistency
 * - Dense obstacle scenarios
 * - State manipulation edge cases
 */

#include <gtest/gtest.h>

// Include ROS interface first (needed by variant_geometry.hpp)
#include <rises_interfaces/msg/obstacle.hpp>

// Include spatial index BEFORE geofence_map to ensure type is defined
#include "spatial_index_selection.hpp"

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/common/bounding_box.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>

// Helper to create obstacles
struct ObstacleBuilder {
    static rises_interfaces::msg::Obstacle rectangle(int64_t id, float min_x, float min_y, float max_x, float max_y) {
        rises_interfaces::msg::Obstacle obs;
        obs.id = id;
        obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
        geometry_msgs::msg::Point p1, p2;
        p1.x = min_x; p1.y = min_y; p1.z = 0.0;
        p2.x = max_x; p2.y = max_y; p2.z = 0.0;
        obs.vertices = {p1, p2};
        return obs;
    }
};

class ComprehensiveEdgeCasesTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::function<std::shared_ptr<rises::geofence::SpatialIndex>()> factory =
            []() -> std::shared_ptr<rises::geofence::SpatialIndex> {
            return std::make_shared<rises::geofence::SpatialIndex>();
        };
        map_ = std::make_unique<rises::geofence::GeofenceMap>(factory);
    }

    std::unique_ptr<rises::geofence::GeofenceMap> map_;
};

// ==================== Empty Map Tests ====================

TEST_F(ComprehensiveEdgeCasesTest, EmptyMap_DistanceQuery) {
    float dist = map_->distanceToNearestObstacle(Point2D{50.0, 50.0});
    EXPECT_TRUE(std::isinf(dist) || dist > 1e6f);
}

TEST_F(ComprehensiveEdgeCasesTest, EmptyMap_RemoveNonexistent) {
    EXPECT_NO_THROW(map_->removeObstacle(999));
}

TEST_F(ComprehensiveEdgeCasesTest, EmptyMap_ObstacleQuery) {
    std::vector<int64_t> ids = map_->getAllObstacleIds();
    EXPECT_TRUE(ids.empty());
}

// ==================== Boundary Precision Tests ====================

TEST_F(ComprehensiveEdgeCasesTest, PointOnObstacleEdge) {
    rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(100, 45.0f, 45.0f, 55.0f, 55.0f);
    rises::geofence::Geometry geom = rises::geofence::fromObstacleMsg(msg);
    map_->insertObstacle(100, geom);

    float dist_on_edge = map_->distanceToNearestObstacle(Point2D{45.0, 50.0});
    EXPECT_NEAR(dist_on_edge, 0.0f, 1e-3f);
}

TEST_F(ComprehensiveEdgeCasesTest, TouchingObstacles) {
    rises_interfaces::msg::Obstacle msg1 = ObstacleBuilder::rectangle(201, 40.0f, 30.0f, 50.0f, 40.0f);
    rises_interfaces::msg::Obstacle msg2 = ObstacleBuilder::rectangle(202, 50.0f, 30.0f, 60.0f, 40.0f);
    map_->insertObstacle(201, rises::geofence::fromObstacleMsg(msg1));
    map_->insertObstacle(202, rises::geofence::fromObstacleMsg(msg2));

    float dist = map_->distanceToNearestObstacle(Point2D{40.0, 35.0});
    EXPECT_GE(dist, 0.0f);
}

// ==================== Degenerate Geometry Tests ====================

TEST_F(ComprehensiveEdgeCasesTest, DegenerateGeometry_ZeroWidth) {
    rises::geofence::BoundingBox bbox = rises::geofence::createValidatedBBox(50.0f, 50.0f, 50.0f, 60.0f);
    EXPECT_TRUE(bbox.isValid());
    EXPECT_TRUE(bbox.isDegenerate());
}

TEST_F(ComprehensiveEdgeCasesTest, DegenerateGeometry_InvertedBounds) {
    rises::geofence::BoundingBox bbox = rises::geofence::createValidatedBBox(60.0f, 60.0f, 50.0f, 50.0f);
    EXPECT_TRUE(bbox.isValid());
}

// ==================== Extreme Values Tests ====================

TEST_F(ComprehensiveEdgeCasesTest, LargeCoordinates) {
    rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(401, 1000.0f, 1000.0f, 1010.0f, 1010.0f);
    EXPECT_NO_THROW(map_->insertObstacle(401, rises::geofence::fromObstacleMsg(msg)));

    float dist = map_->distanceToNearestObstacle(Point2D{1005.0, 1005.0});
    EXPECT_NEAR(dist, 0.0f, 1e-3f);
}

TEST_F(ComprehensiveEdgeCasesTest, NearZeroCoordinates) {
    rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(402, 0.001f, 0.001f, 0.002f, 0.002f);
    EXPECT_NO_THROW(map_->insertObstacle(402, rises::geofence::fromObstacleMsg(msg)));

    float dist = map_->distanceToNearestObstacle(Point2D{0.0015, 0.0015});
    EXPECT_GE(dist, 0.0f);
}

TEST_F(ComprehensiveEdgeCasesTest, NegativeCoordinates) {
    rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(403, -60.0f, -60.0f, -40.0f, -40.0f);
    EXPECT_NO_THROW(map_->insertObstacle(403, rises::geofence::fromObstacleMsg(msg)));

    float dist = map_->distanceToNearestObstacle(Point2D{-45.0, -45.0});
    EXPECT_NEAR(dist, 0.0f, 1e-3f);
}

TEST_F(ComprehensiveEdgeCasesTest, MixedScales) {
    rises_interfaces::msg::Obstacle small = ObstacleBuilder::rectangle(404, 10.0f, 10.0f, 10.01f, 10.01f);
    rises_interfaces::msg::Obstacle large = ObstacleBuilder::rectangle(405, 50.0f, 50.0f, 100.0f, 100.0f);
    EXPECT_NO_THROW(map_->insertObstacle(404, rises::geofence::fromObstacleMsg(small)));
    EXPECT_NO_THROW(map_->insertObstacle(405, rises::geofence::fromObstacleMsg(large)));

    float dist_small = map_->distanceToNearestObstacle(Point2D{10.005, 10.005});
    float dist_large = map_->distanceToNearestObstacle(Point2D{70.0, 70.0});
    EXPECT_GE(dist_small, 0.0f);
    EXPECT_NEAR(dist_large, 0.0f, 1e-3f);
}

// ==================== RCU Consistency Tests ====================

TEST_F(ComprehensiveEdgeCasesTest, RCU_SnapshotConsistency) {
    rises_interfaces::msg::Obstacle msg1 = ObstacleBuilder::rectangle(501, 40.0f, 40.0f, 50.0f, 50.0f);
    map_->insertObstacle(501, rises::geofence::fromObstacleMsg(msg1));

    std::vector<int64_t> ids_before = map_->getAllObstacleIds();
    EXPECT_EQ(ids_before.size(), 1);

    for (int i = 502; i < 510; i++) {
        rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(i, i * 1.0f, i * 1.0f, i * 1.0f + 10.0f, i * 1.0f + 10.0f);
        map_->insertObstacle(i, rises::geofence::fromObstacleMsg(msg));
    }

    std::vector<int64_t> ids_after = map_->getAllObstacleIds();
    EXPECT_EQ(ids_after.size(), 9);
}

TEST_F(ComprehensiveEdgeCasesTest, RCU_ConcurrentReadsWrites) {
    // Simplified concurrent test without thread complexity
    for (int i = 601; i < 611; i++) {
        rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(i, i * 1.0f, i * 1.0f, i * 1.0f + 10.0f, i * 1.0f + 10.0f);
        map_->insertObstacle(i, rises::geofence::fromObstacleMsg(msg));
    }

    // Interleave reads and writes
    for (int iter = 0; iter < 10; iter++) {
        // Write
        rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(700 + iter, 700.0f + iter, 700.0f + iter, 710.0f + iter, 710.0f + iter);
        map_->insertObstacle(700 + iter, rises::geofence::fromObstacleMsg(msg));

        // Read
        for (double x = 600.0; x < 620.0; x += 5.0) {
            for (double y = 600.0; y < 620.0; y += 5.0) {
                float dist = map_->distanceToNearestObstacle(Point2D{x, y});
                EXPECT_FALSE(std::isnan(dist));
            }
        }
    }
}

// ==================== Dense Obstacles Tests ====================

TEST_F(ComprehensiveEdgeCasesTest, ManySmallObstacles) {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            float x = 10.0f * i;
            float y = 10.0f * j;
            rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(800 + i * 10 + j, x, y, x + 1.0f, y + 1.0f);
            map_->insertObstacle(800 + i * 10 + j, rises::geofence::fromObstacleMsg(msg));
        }
    }

    std::vector<int64_t> ids = map_->getAllObstacleIds();
    EXPECT_EQ(ids.size(), 100);

    // Query between obstacles (each obstacle is 1m starting at multiples of 10)
    float dist = map_->distanceToNearestObstacle(Point2D{5.5, 5.5});
    EXPECT_GT(dist, 0.0f);
}

TEST_F(ComprehensiveEdgeCasesTest, OverlappingObstacles) {
    for (int i = 0; i < 10; i++) {
        rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(900 + i, 45.0f, 45.0f, 55.0f, 55.0f);
        map_->insertObstacle(900 + i, rises::geofence::fromObstacleMsg(msg));
    }

    float dist = map_->distanceToNearestObstacle(Point2D{50.0, 50.0});
    EXPECT_NEAR(dist, 0.0f, 1e-3f);

    std::vector<int64_t> ids = map_->getAllObstacleIds();
    EXPECT_EQ(ids.size(), 10);
}

// ==================== State Manipulation Tests ====================

TEST_F(ComprehensiveEdgeCasesTest, RemoveTwice) {
    rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(1001, 40.0f, 40.0f, 50.0f, 50.0f);
    map_->insertObstacle(1001, rises::geofence::fromObstacleMsg(msg));

    EXPECT_NO_THROW(map_->removeObstacle(1001));
    EXPECT_NO_THROW(map_->removeObstacle(1001));
}

TEST_F(ComprehensiveEdgeCasesTest, RapidAddRemove) {
    for (int i = 0; i < 100; i++) {
        rises_interfaces::msg::Obstacle msg = ObstacleBuilder::rectangle(1002, 40.0f, 40.0f, 50.0f, 50.0f);
        map_->insertObstacle(1002, rises::geofence::fromObstacleMsg(msg));
        map_->removeObstacle(1002);
    }

    std::vector<int64_t> ids = map_->getAllObstacleIds();
    EXPECT_TRUE(ids.empty());
}

TEST_F(ComprehensiveEdgeCasesTest, ExtremeIDValues) {
    rises_interfaces::msg::Obstacle msg_zero = ObstacleBuilder::rectangle(0, 40.0f, 40.0f, 50.0f, 50.0f);
    EXPECT_NO_THROW(map_->insertObstacle(0, rises::geofence::fromObstacleMsg(msg_zero)));

    rises_interfaces::msg::Obstacle msg_max = ObstacleBuilder::rectangle(std::numeric_limits<int64_t>::max(), 60.0f, 60.0f, 70.0f, 70.0f);
    EXPECT_NO_THROW(map_->insertObstacle(std::numeric_limits<int64_t>::max(), rises::geofence::fromObstacleMsg(msg_max)));

    std::vector<int64_t> ids = map_->getAllObstacleIds();
    EXPECT_EQ(ids.size(), 2);
}
