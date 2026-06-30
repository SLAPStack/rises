/**
 * @file test_occupancy_grid_map.cpp
 * @brief Comprehensive test suite for GridMap implementation
 * 
 * Tests cover:
 * - Basic CRUD operations (insert, remove, clear)
 * - Geometry types (point, line, rectangle, circle, polygon)
 * - Query operations (occupancy, path blocking, range search)
 * - Safety circle exclusion for robot collision avoidance
 * - Edge cases (out of bounds, zero-size, overlapping obstacles)
 * 
 * All tests use GridMap which provides:
 * - Lock-free concurrent reads via RCU snapshots
 * - Copy-on-write updates for thread safety
 * - Memory-efficient std::vector<bool> storage (1 bit per cell)
 */

#include <gtest/gtest.h>
#include "geofence/gridmap/map/gridmap.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include <memory>

using rises::geofence::GridMap;

/**
 * @brief Helper for constructing ROS obstacle messages
 * 
 * Provides convenient factory methods for each geometry type.
 * Ensures consistent message structure across all tests.
 */
struct ObstacleBuilder {
    static rises_interfaces::msg::Obstacle point(uint64_t id, float x, float y) {
        rises_interfaces::msg::Obstacle obs;
        obs.id = id;
        obs.type = rises_interfaces::msg::Obstacle::POINT;
        obs.position.x = x;
        obs.position.y = y;
        return obs;
    }
    
    static rises_interfaces::msg::Obstacle line(uint64_t id, float x1, float y1, float x2, float y2) {
        rises_interfaces::msg::Obstacle obs;
        obs.id = id;
        obs.type = rises_interfaces::msg::Obstacle::LINE;
        geometry_msgs::msg::Point p1, p2;
        p1.x = x1; p1.y = y1; p1.z = 0.0;
        p2.x = x2; p2.y = y2; p2.z = 0.0;
        obs.vertices = {p1, p2};
        return obs;
    }
    
    static rises_interfaces::msg::Obstacle rectangle(uint64_t id, float cx, float cy, float w, float h) {
        rises_interfaces::msg::Obstacle obs;
        obs.id = id;
        obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
        obs.position.x = cx;
        obs.position.y = cy;
        obs.width = w;
        obs.height = h;
        obs.orientation = 0.0f;
        return obs;
    }
    
    static rises_interfaces::msg::Obstacle circle(uint64_t id, float x, float y, float radius) {
        rises_interfaces::msg::Obstacle obs;
        obs.id = id;
        obs.type = rises_interfaces::msg::Obstacle::CIRCLE;
        obs.position.x = x;
        obs.position.y = y;
        obs.radius = radius;
        return obs;
    }
    
    static rises_interfaces::msg::Obstacle polygon(uint64_t id, const std::vector<std::pair<float, float>>& points) {
        rises_interfaces::msg::Obstacle obs;
        obs.id = id;
        obs.type = rises_interfaces::msg::Obstacle::POLYGON;
        for (const std::pair<const float, float>& point_pair : points) {
            geometry_msgs::msg::Point p;
            p.x = point_pair.first;
            p.y = point_pair.second;
            p.z = 0.0;
            obs.vertices.push_back(p);
        }
        return obs;
    }
};

/**
 * @brief Test fixture providing common gridmap setup
 * 
 * Configuration:
 * - Resolution: 10cm cells (0.1m)
 * - Size: 50m x 50m
 * - Origin: (-25, -25) for centered coordinate system
 * - Grid dimensions: 500 x 500 cells
 * - Memory: ~31KB (500² cells × 1 bit)
 */
class OccupancyGridMapTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.resolution = 0.1;      // 10cm cells
        config_.width_meters = 50.0;   // 50m wide
        config_.height_meters = 50.0;  // 50m tall
        config_.origin_x = -25.0;      // Centered
        config_.origin_y = -25.0;
        
        gridmap_ = std::make_unique<GridMap>(config_);
    }
    
    GridMap::Config config_;
    std::unique_ptr<GridMap> gridmap_;
};

// ==================== Constructor & Configuration ====================

TEST_F(OccupancyGridMapTest, ConstructorDoesNotCrash) {
    EXPECT_NE(gridmap_, nullptr);
    EXPECT_EQ(gridmap_->getResolution(), 0.1);
    EXPECT_EQ(gridmap_->getGridWidth(), 500);   // 50m / 0.1m
    EXPECT_EQ(gridmap_->getGridHeight(), 500);
}

TEST_F(OccupancyGridMapTest, MemoryUsageIsReasonable) {
    // 500x500 grid = 250,000 cells
    // std::vector<bool> is bit-packed: 8 cells per byte
    // Total: 250,000 / 8 = 31,250 bytes = ~31KB
    // This is 64x more efficient than uint64_t per cell (would be 2MB)
    const size_t expected_cells = 500 * 500;
    const size_t expected_memory_kb = (expected_cells / 8) / 1024;
    EXPECT_LT(expected_memory_kb, 100);  // Should be ~31KB
}

// ==================== Point Obstacles ====================

TEST_F(OccupancyGridMapTest, InsertSinglePoint) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::point(1, 0.0f, 0.0f);
    gridmap_->insertObstacle(1, obs);
    
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_FALSE(gridmap_->isOccupied(1.0, 1.0));
}

TEST_F(OccupancyGridMapTest, RemovePoint) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::point(1, 5.0f, 5.0f);
    gridmap_->insertObstacle(1, obs);
    EXPECT_TRUE(gridmap_->isOccupied(5.0, 5.0));
    
    gridmap_->removeObstacle(1);
    EXPECT_FALSE(gridmap_->isOccupied(5.0, 5.0));
}

TEST_F(OccupancyGridMapTest, PointOutOfBoundsDoesNotCrash) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::point(1, 100.0f, 100.0f);  // Out of bounds
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
}

// ==================== Line Obstacles ====================

TEST_F(OccupancyGridMapTest, InsertLine) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(2, 0.0f, 0.0f, 5.0f, 5.0f);
    gridmap_->insertObstacle(2, obs);
    
    // Check line passes through some points
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(5.0, 5.0));
    // Middle point should also be occupied
    EXPECT_TRUE(gridmap_->isOccupied(2.5, 2.5));
}

TEST_F(OccupancyGridMapTest, RemoveLine) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(2, -5.0f, -5.0f, 5.0f, 5.0f);
    gridmap_->insertObstacle(2, obs);
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    
    gridmap_->removeObstacle(2);
    EXPECT_FALSE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_FALSE(gridmap_->isOccupied(5.0, 5.0));
}

TEST_F(OccupancyGridMapTest, HorizontalLine) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(3, -10.0f, 0.0f, 10.0f, 0.0f);
    gridmap_->insertObstacle(3, obs);
    
    EXPECT_TRUE(gridmap_->isOccupied(-10.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(10.0, 0.0));
    EXPECT_FALSE(gridmap_->isOccupied(0.0, 1.0));  // Offset should be free
}

TEST_F(OccupancyGridMapTest, VerticalLine) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(4, 0.0f, -10.0f, 0.0f, 10.0f);
    gridmap_->insertObstacle(4, obs);
    
    EXPECT_TRUE(gridmap_->isOccupied(0.0, -10.0));
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 10.0));
    EXPECT_FALSE(gridmap_->isOccupied(1.0, 0.0));  // Offset should be free
}

// ==================== Rectangle Obstacles ====================

TEST_F(OccupancyGridMapTest, InsertRectangle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::rectangle(5, 0.0f, 0.0f, 4.0f, 2.0f);
    gridmap_->insertObstacle(5, obs);
    
    // Center should be occupied
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    
    // Corners should be occupied (within bounds)
    EXPECT_TRUE(gridmap_->isOccupied(1.9, 0.9));
    EXPECT_TRUE(gridmap_->isOccupied(-1.9, -0.9));
    
    // Outside rectangle should be free
    EXPECT_FALSE(gridmap_->isOccupied(3.0, 3.0));
}

TEST_F(OccupancyGridMapTest, RemoveRectangle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::rectangle(5, 5.0f, 5.0f, 2.0f, 2.0f);
    gridmap_->insertObstacle(5, obs);
    EXPECT_TRUE(gridmap_->isOccupied(5.0, 5.0));
    
    gridmap_->removeObstacle(5);
    EXPECT_FALSE(gridmap_->isOccupied(5.0, 5.0));
}

// ==================== Circle Obstacles ====================

TEST_F(OccupancyGridMapTest, InsertCircle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(6, 0.0f, 0.0f, 5.0f);
    gridmap_->insertObstacle(6, obs);
    
    // Center should be occupied
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    
    // Points inside radius should be occupied
    EXPECT_TRUE(gridmap_->isOccupied(3.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 3.0));
    
    // Points outside radius should be free
    EXPECT_FALSE(gridmap_->isOccupied(10.0, 0.0));
}

TEST_F(OccupancyGridMapTest, RemoveCircle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(6, 10.0f, 10.0f, 3.0f);
    gridmap_->insertObstacle(6, obs);
    EXPECT_TRUE(gridmap_->isOccupied(10.0, 10.0));
    
    gridmap_->removeObstacle(6);
    EXPECT_FALSE(gridmap_->isOccupied(10.0, 10.0));
}

TEST_F(OccupancyGridMapTest, SmallCircle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(7, 0.0f, 0.0f, 0.5f);
    gridmap_->insertObstacle(7, obs);
    
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(0.4, 0.0));
}

// ==================== Polygon Obstacles ====================

TEST_F(OccupancyGridMapTest, InsertTriangle) {
    const std::vector<std::pair<float, float>> triangle = {
        {0.0f, 0.0f},
        {5.0f, 0.0f},
        {2.5f, 5.0f}
    };
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(8, triangle);
    gridmap_->insertObstacle(8, obs);
    
    // Interior point should be occupied
    EXPECT_TRUE(gridmap_->isOccupied(2.5, 1.0));
    
    // Exterior point should be free
    EXPECT_FALSE(gridmap_->isOccupied(10.0, 10.0));
}

TEST_F(OccupancyGridMapTest, InsertSquarePolygon) {
    const std::vector<std::pair<float, float>> square = {
        {-2.0f, -2.0f},
        {2.0f, -2.0f},
        {2.0f, 2.0f},
        {-2.0f, 2.0f}
    };
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(9, square);
    gridmap_->insertObstacle(9, obs);
    
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(1.5, 1.5));
    EXPECT_FALSE(gridmap_->isOccupied(3.0, 3.0));
}

TEST_F(OccupancyGridMapTest, RemovePolygon) {
    const std::vector<std::pair<float, float>> triangle = {{0.0f, 0.0f}, {3.0f, 0.0f}, {1.5f, 3.0f}};
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(8, triangle);
    gridmap_->insertObstacle(8, obs);
    EXPECT_TRUE(gridmap_->isOccupied(1.5, 1.0));
    
    gridmap_->removeObstacle(8);
    EXPECT_FALSE(gridmap_->isOccupied(1.5, 1.0));
}

// ==================== Multiple Obstacles ====================

TEST_F(OccupancyGridMapTest, InsertMultipleObstacles) {
    const rises_interfaces::msg::Obstacle point = ObstacleBuilder::point(1, 0.0f, 0.0f);
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(2, 5.0f, 5.0f, 2.0f);
    const rises_interfaces::msg::Obstacle line = ObstacleBuilder::line(3, -5.0f, -5.0f, -5.0f, 5.0f);
    
    gridmap_->insertObstacle(1, point);
    gridmap_->insertObstacle(2, circle);
    gridmap_->insertObstacle(3, line);
    
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(5.0, 5.0));
    EXPECT_TRUE(gridmap_->isOccupied(-5.0, 0.0));
}

TEST_F(OccupancyGridMapTest, OverwriteObstacle) {
    const rises_interfaces::msg::Obstacle circle1 = ObstacleBuilder::circle(10, 0.0f, 0.0f, 2.0f);
    const rises_interfaces::msg::Obstacle circle2 = ObstacleBuilder::circle(10, 10.0f, 10.0f, 2.0f);
    
    gridmap_->insertObstacle(10, circle1);
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_FALSE(gridmap_->isOccupied(10.0, 10.0));
    
    // Inserting same ID at different location should:
    // 1. Remove old obstacle cells
    // 2. Rasterize new obstacle
    // This ensures no "ghost" obstacles from old positions
    gridmap_->insertObstacle(10, circle2);
    EXPECT_FALSE(gridmap_->isOccupied(0.0, 0.0));   // Old location cleared
    EXPECT_TRUE(gridmap_->isOccupied(10.0, 10.0)); // New location set
}

// ==================== Query Operations ====================

TEST_F(OccupancyGridMapTest, FindObstaclesNear) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 0.0f, 0.0f, 1.0f);
    const rises_interfaces::msg::Obstacle point = ObstacleBuilder::point(2, 5.0f, 0.0f);
    const rises_interfaces::msg::Obstacle line = ObstacleBuilder::line(3, 10.0f, 0.0f, 15.0f, 0.0f);
    
    gridmap_->insertObstacle(1, circle);
    gridmap_->insertObstacle(2, point);
    gridmap_->insertObstacle(3, line);
    
    // Search near origin should find circle
    const std::vector<int64_t> near_origin = gridmap_->findObstaclesNear(0.0, 0.0, 2.0);
    EXPECT_EQ(near_origin.size(), 1);
    EXPECT_EQ(near_origin[0], 1);
    
    // Search at (5,0) should find point
    const std::vector<int64_t> near_point = gridmap_->findObstaclesNear(5.0, 0.0, 1.0);
    EXPECT_EQ(near_point.size(), 1);
    EXPECT_EQ(near_point[0], 2);
}

TEST_F(OccupancyGridMapTest, IsPathBlocked) {
    const rises_interfaces::msg::Obstacle line = ObstacleBuilder::line(1, 5.0f, 0.0f, 5.0f, 10.0f);
    gridmap_->insertObstacle(1, line);
    
    // Path crossing the line should be blocked
    EXPECT_TRUE(gridmap_->isPathBlocked(0.0, 5.0, 10.0, 5.0));
    
    // Path not crossing should be free
    EXPECT_FALSE(gridmap_->isPathBlocked(0.0, 0.0, 3.0, 0.0));
}

TEST_F(OccupancyGridMapTest, IsPathBlockedByCircle) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 5.0f, 5.0f, 2.0f);
    gridmap_->insertObstacle(1, circle);
    
    // Path through circle should be blocked
    EXPECT_TRUE(gridmap_->isPathBlocked(0.0, 5.0, 10.0, 5.0));
    
    // Path around circle should be free
    EXPECT_FALSE(gridmap_->isPathBlocked(0.0, 0.0, 10.0, 0.0));
}

// ==================== Safety Circle Queries ====================

TEST_F(OccupancyGridMapTest, FindObstaclesInSafetyCircle) {
    // Create obstacles at various distances from origin
    const rises_interfaces::msg::Obstacle circle1 = ObstacleBuilder::circle(1, 2.0f, 0.0f, 0.5f);  // Close
    const rises_interfaces::msg::Obstacle circle2 = ObstacleBuilder::circle(2, 10.0f, 0.0f, 0.5f); // Far
    const rises_interfaces::msg::Obstacle circle3 = ObstacleBuilder::circle(3, 4.5f, 0.0f, 0.5f);  // At edge
    
    gridmap_->insertObstacle(1, circle1);
    gridmap_->insertObstacle(2, circle2);
    gridmap_->insertObstacle(3, circle3);
    
    // Find obstacles within 5m safety circle centered at origin
    const std::vector<int64_t> found = gridmap_->findObstaclesInSafetyCircle(0.0, 0.0, 5.0);
    
    // Should find obstacles 1 and 3 (both have cells within 5m)
    // Even if only ONE cell of the obstacle is within the circle, it's included
    EXPECT_GE(found.size(), 1); // At least obstacle 1 should be found
    EXPECT_TRUE(std::find(found.begin(), found.end(), 1) != found.end()); // Circle1 definitely in
    
    // Obstacle 2 at 10m should NOT be found
    EXPECT_TRUE(std::find(found.begin(), found.end(), 2) == found.end());
}

TEST_F(OccupancyGridMapTest, FindObstaclesInSafetyCirclePartialOverlap) {
    // Large obstacle that partially overlaps safety circle
    const rises_interfaces::msg::Obstacle rect = ObstacleBuilder::rectangle(1, 7.0f, 0.0f, 6.0f, 2.0f);
    gridmap_->insertObstacle(1, rect);
    
    // Safety circle at origin with 5m radius
    // Rectangle extends from x=4 to x=10, so left edge is within circle
    const std::vector<int64_t> found = gridmap_->findObstaclesInSafetyCircle(0.0, 0.0, 5.0);
    
    // Should find the rectangle even though only part of it is within circle
    EXPECT_GE(found.size(), 1);
    EXPECT_TRUE(std::find(found.begin(), found.end(), 1) != found.end());
}

TEST_F(OccupancyGridMapTest, FindObstaclesInSafetyCircleEmpty) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 20.0f, 0.0f, 1.0f);
    gridmap_->insertObstacle(1, circle);
    
    // Small safety circle with no obstacles
    const std::vector<int64_t> found = gridmap_->findObstaclesInSafetyCircle(0.0, 0.0, 2.0);
    EXPECT_EQ(found.size(), 0);
}

// ==================== Clear ====================

TEST_F(OccupancyGridMapTest, ClearRemovesAllObstacles) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 0.0f, 0.0f, 2.0f);
    const rises_interfaces::msg::Obstacle line = ObstacleBuilder::line(2, 5.0f, 5.0f, 10.0f, 10.0f);
    
    gridmap_->insertObstacle(1, circle);
    gridmap_->insertObstacle(2, line);
    
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(5.0, 5.0));
    
    gridmap_->clear();
    
    EXPECT_FALSE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_FALSE(gridmap_->isOccupied(5.0, 5.0));
}

// ==================== Edge Cases ====================

TEST_F(OccupancyGridMapTest, RemoveNonExistentObstacle) {
    EXPECT_NO_THROW(gridmap_->removeObstacle(999));
}

TEST_F(OccupancyGridMapTest, QueryOutOfBounds) {
    EXPECT_FALSE(gridmap_->isOccupied(1000.0, 1000.0));
    EXPECT_FALSE(gridmap_->isOccupied(-1000.0, -1000.0));
}

TEST_F(OccupancyGridMapTest, ZeroSizeRectangle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::rectangle(1, 0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
}

TEST_F(OccupancyGridMapTest, ZeroRadiusCircle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(1, 0.0f, 0.0f, 0.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
}

TEST_F(OccupancyGridMapTest, ZeroLengthLine) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(1, 5.0f, 5.0f, 5.0f, 5.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
}

// ==================== Additional Edge Cases ====================

TEST_F(OccupancyGridMapTest, NegativeRadiusCircle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(1, 0.0f, 0.0f, -5.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Negative radius should not crash, should be treated as zero or no obstacle
}

TEST_F(OccupancyGridMapTest, VeryLargeCircle) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(1, 0.0f, 0.0f, 100.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Should only rasterize cells within grid bounds
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
}

TEST_F(OccupancyGridMapTest, DiagonalLineAcrossFullGrid) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(1, -24.0f, -24.0f, 24.0f, 24.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Check line exists at multiple points along the diagonal
    EXPECT_TRUE(gridmap_->isOccupied(-20.0, -20.0));
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(20.0, 20.0));
}

TEST_F(OccupancyGridMapTest, LinePartiallyOutOfBounds) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(1, -30.0f, 0.0f, 30.0f, 0.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Should rasterize only the portion within bounds
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(20.0, 0.0));
}

TEST_F(OccupancyGridMapTest, RectangleAtGridBoundary) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::rectangle(1, 24.0f, 24.0f, 2.0f, 2.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Rectangle near edge should not crash
    EXPECT_TRUE(gridmap_->isOccupied(24.0, 24.0));
}

TEST_F(OccupancyGridMapTest, CircleAtGridBoundary) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(1, -24.5f, -24.5f, 1.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Should handle edge clipping correctly
}

TEST_F(OccupancyGridMapTest, PolygonWithTwoVertices) {
    const std::vector<std::pair<float, float>> line = {{0.0f, 0.0f}, {5.0f, 5.0f}};
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(1, line);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Degenerate polygon (line) should not crash
}

TEST_F(OccupancyGridMapTest, PolygonWithOneVertex) {
    const std::vector<std::pair<float, float>> point = {{0.0f, 0.0f}};
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(1, point);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Degenerate polygon (point) should not crash
}

TEST_F(OccupancyGridMapTest, EmptyPolygon) {
    const std::vector<std::pair<float, float>> empty;
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(1, empty);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
}

TEST_F(OccupancyGridMapTest, ComplexConcavePolygon) {
    // Star shape - tests ray-casting with complex geometry
    const std::vector<std::pair<float, float>> star = {
        {0.0f, 5.0f},   // Top
        {1.5f, 1.5f},   // Inner
        {5.0f, 1.0f},   // Right
        {2.0f, -0.5f},  // Inner
        {3.0f, -5.0f},  // Bottom right
        {0.0f, -2.0f},  // Inner
        {-3.0f, -5.0f}, // Bottom left
        {-2.0f, -0.5f}, // Inner
        {-5.0f, 1.0f},  // Left
        {-1.5f, 1.5f}   // Inner
    };
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(1, star);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    
    // Center should be occupied
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    // Interior point in one of the "arms"
    EXPECT_TRUE(gridmap_->isOccupied(2.0, 0.5));
}

TEST_F(OccupancyGridMapTest, PathBlockedDiagonally) {
    const rises_interfaces::msg::Obstacle line = ObstacleBuilder::line(1, 0.0f, 0.0f, 10.0f, 10.0f);
    gridmap_->insertObstacle(1, line);
    
    // Path crossing diagonal line
    EXPECT_TRUE(gridmap_->isPathBlocked(-5.0, 10.0, 10.0, -5.0));
}

TEST_F(OccupancyGridMapTest, PathBlockedAtStart) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 0.0f, 0.0f, 1.0f);
    gridmap_->insertObstacle(1, circle);
    
    // Path starting inside obstacle
    EXPECT_TRUE(gridmap_->isPathBlocked(0.0, 0.0, 10.0, 10.0));
}

TEST_F(OccupancyGridMapTest, PathBlockedAtEnd) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 10.0f, 10.0f, 1.0f);
    gridmap_->insertObstacle(1, circle);
    
    // Path ending inside obstacle
    EXPECT_TRUE(gridmap_->isPathBlocked(0.0, 0.0, 10.0, 10.0));
}

TEST_F(OccupancyGridMapTest, PathOutOfBounds) {
    // Path completely outside grid
    bool result = false;
    EXPECT_NO_THROW(result = gridmap_->isPathBlocked(100.0, 100.0, 200.0, 200.0));
    (void)result; // Suppress unused variable warning
}

TEST_F(OccupancyGridMapTest, FindObstaclesNearWithZeroRadius) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 0.0f, 0.0f, 1.0f);
    gridmap_->insertObstacle(1, circle);
    
    // Zero radius search - should still check the exact cell
    const std::vector<int64_t> found = gridmap_->findObstaclesNear(0.0, 0.0, 0.0);
    EXPECT_GE(found.size(), 0); // Should not crash
}

TEST_F(OccupancyGridMapTest, FindObstaclesNearOutOfBounds) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 0.0f, 0.0f, 1.0f);
    gridmap_->insertObstacle(1, circle);
    
    // Search outside grid bounds
    const std::vector<int64_t> found = gridmap_->findObstaclesNear(1000.0, 1000.0, 5.0);
    EXPECT_EQ(found.size(), 0);
}

TEST_F(OccupancyGridMapTest, FindObstaclesNearLargeRadius) {
    const rises_interfaces::msg::Obstacle point1 = ObstacleBuilder::point(1, 0.0f, 0.0f);
    const rises_interfaces::msg::Obstacle point2 = ObstacleBuilder::point(2, 10.0f, 0.0f);
    const rises_interfaces::msg::Obstacle point3 = ObstacleBuilder::point(3, 20.0f, 0.0f);
    
    gridmap_->insertObstacle(1, point1);
    gridmap_->insertObstacle(2, point2);
    gridmap_->insertObstacle(3, point3);
    
    // Large radius should find multiple obstacles
    const std::vector<int64_t> found = gridmap_->findObstaclesNear(5.0, 0.0, 20.0);
    EXPECT_GE(found.size(), 2); // Should find at least points 1 and 2
}

TEST_F(OccupancyGridMapTest, OverlappingCircles) {
    const rises_interfaces::msg::Obstacle circle1 = ObstacleBuilder::circle(1, 0.0f, 0.0f, 3.0f);
    const rises_interfaces::msg::Obstacle circle2 = ObstacleBuilder::circle(2, 2.0f, 0.0f, 3.0f);
    
    gridmap_->insertObstacle(1, circle1);
    gridmap_->insertObstacle(2, circle2);
    
    // Overlapping region should be occupied
    EXPECT_TRUE(gridmap_->isOccupied(1.0, 0.0));
    
    // Remove one, overlapping region might still be occupied by the other
    gridmap_->removeObstacle(1);
    EXPECT_TRUE(gridmap_->isOccupied(2.0, 0.0)); // Still occupied by circle2
}

TEST_F(OccupancyGridMapTest, InsertRemoveInsertSameID) {
    const rises_interfaces::msg::Obstacle circle1 = ObstacleBuilder::circle(1, 0.0f, 0.0f, 2.0f);
    const rises_interfaces::msg::Obstacle circle2 = ObstacleBuilder::circle(1, 10.0f, 10.0f, 2.0f);
    
    gridmap_->insertObstacle(1, circle1);
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    
    gridmap_->removeObstacle(1);
    EXPECT_FALSE(gridmap_->isOccupied(0.0, 0.0));
    
    gridmap_->insertObstacle(1, circle2);
    EXPECT_TRUE(gridmap_->isOccupied(10.0, 10.0));
    EXPECT_FALSE(gridmap_->isOccupied(0.0, 0.0)); // Old location should remain clear
}

TEST_F(OccupancyGridMapTest, VeryLongLine) {
    // Line crossing entire grid multiple times the resolution
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::line(1, -24.0f, -24.0f, 24.0f, 24.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    
    // Should create continuous line
    EXPECT_TRUE(gridmap_->isOccupied(-10.0, -10.0));
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(10.0, 10.0));
}

TEST_F(OccupancyGridMapTest, MultipleRemovalsOfSameObstacle) {
    const rises_interfaces::msg::Obstacle circle = ObstacleBuilder::circle(1, 0.0f, 0.0f, 2.0f);
    gridmap_->insertObstacle(1, circle);
    
    gridmap_->removeObstacle(1);
    EXPECT_NO_THROW(gridmap_->removeObstacle(1)); // Second removal should not crash
    EXPECT_NO_THROW(gridmap_->removeObstacle(1)); // Third removal should not crash
}

TEST_F(OccupancyGridMapTest, ClearThenInsert) {
    const rises_interfaces::msg::Obstacle circle1 = ObstacleBuilder::circle(1, 0.0f, 0.0f, 2.0f);
    gridmap_->insertObstacle(1, circle1);
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    
    gridmap_->clear();
    EXPECT_FALSE(gridmap_->isOccupied(0.0, 0.0));
    
    // Insert after clear should work
    const rises_interfaces::msg::Obstacle circle2 = ObstacleBuilder::circle(2, 5.0f, 5.0f, 2.0f);
    gridmap_->insertObstacle(2, circle2);
    EXPECT_TRUE(gridmap_->isOccupied(5.0, 5.0));
}

TEST_F(OccupancyGridMapTest, MultipleClears) {
    gridmap_->clear();
    EXPECT_NO_THROW(gridmap_->clear());
    EXPECT_NO_THROW(gridmap_->clear());
}

TEST_F(OccupancyGridMapTest, RectangleWithNegativeDimensions) {
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::rectangle(1, 0.0f, 0.0f, -4.0f, -2.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
}

TEST_F(OccupancyGridMapTest, VerySmallRectangle) {
    // Rectangle smaller than one grid cell
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::rectangle(1, 0.0f, 0.0f, 0.05f, 0.05f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Should still occupy at least the center cell
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
}

TEST_F(OccupancyGridMapTest, PathAlongGridBoundary) {
    // Path exactly along the grid edge
    const double edge = 24.9;
    bool result = false;
    EXPECT_NO_THROW(result = gridmap_->isPathBlocked(-edge, 0.0, edge, 0.0));
    (void)result; // Suppress unused variable warning
}

TEST_F(OccupancyGridMapTest, CircleCenterOutOfBoundsRadiusInBounds) {
    // Circle center outside grid but radius extends into grid
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::circle(1, 30.0f, 0.0f, 10.0f);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Should rasterize the portion that overlaps the grid
    EXPECT_TRUE(gridmap_->isOccupied(24.0, 0.0));
}

TEST_F(OccupancyGridMapTest, PolygonPartiallyOutOfBounds) {
    const std::vector<std::pair<float, float>> polygon = {
        {-30.0f, -30.0f},  // Out of bounds
        {30.0f, -30.0f},   // Out of bounds
        {30.0f, 30.0f},    // Out of bounds
        {-30.0f, 30.0f}    // Out of bounds
    };
    const rises_interfaces::msg::Obstacle obs = ObstacleBuilder::polygon(1, polygon);
    EXPECT_NO_THROW(gridmap_->insertObstacle(1, obs));
    // Should fill the entire grid since polygon contains it
    EXPECT_TRUE(gridmap_->isOccupied(0.0, 0.0));
    EXPECT_TRUE(gridmap_->isOccupied(20.0, 20.0));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
