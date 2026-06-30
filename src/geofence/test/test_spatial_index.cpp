#include <gtest/gtest.h>
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "spatial_index_selection.hpp"
#include <functional>
#include <memory>

// Test fixture for spatial index queries
class SpatialIndexTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create factory
        std::function<std::shared_ptr<rises::geofence::SpatialIndex>()> factory =
            []() -> std::shared_ptr<rises::geofence::SpatialIndex> {
            return std::make_shared<rises::geofence::SpatialIndex>();
        };
        
        map_ = std::make_unique<rises::geofence::GeofenceMap>(factory);
        
        // Insert test obstacles
        // Obstacle 1: Point at (0, 0)
        map_->insertObstacle(1, rises::geofence::Point(0.0f, 0.0f));
        
        // Obstacle 2: Line from (5, 5) to (10, 5)
        map_->insertObstacle(2, rises::geofence::makeLine(5.0f, 5.0f, 10.0f, 5.0f));
        
        // Obstacle 3: Rectangle at (10, 10) to (12, 12)
        map_->insertObstacle(3, rises::geofence::makeRectangle(10.0f, 10.0f, 12.0f, 12.0f));
        
        // Obstacle 4: Far point at (100, 100)
        map_->insertObstacle(4, rises::geofence::Point(100.0f, 100.0f));
    }
    
    std::unique_ptr<rises::geofence::GeofenceMap> map_;
};

TEST_F(SpatialIndexTest, QueryNearbyFindsCloseObstacles) {
    // Query near origin - should find obstacle 1
    Point2D query_point{0.5, 0.5};
    std::vector<int64_t> results = map_->queryNearby(query_point, 1.0f);
    
    EXPECT_FALSE(results.empty());
    EXPECT_TRUE(std::find(results.begin(), results.end(), 1) != results.end());
}

TEST_F(SpatialIndexTest, QueryNearbyExcludesFarObstacles) {
    // Query near origin - should NOT find obstacle 4
    Point2D query_point{0.0, 0.0};
    std::vector<int64_t> results = map_->queryNearby(query_point, 10.0f);
    
    EXPECT_TRUE(std::find(results.begin(), results.end(), 4) == results.end());
}

TEST_F(SpatialIndexTest, QueryNearbyMultipleResults) {
    // Query in middle area - should find obstacles 2 and 3
    Point2D query_point{8.0, 8.0};
    std::vector<int64_t> results = map_->queryNearby(query_point, 5.0f);
    
    EXPECT_GE(results.size(), 2);
}

TEST_F(SpatialIndexTest, SegmentIntersectsObstacle) {
    // Segment passing through rectangle obstacle
    Point2D p1{9.0, 11.0};
    Point2D p2{13.0, 11.0};
    
    EXPECT_TRUE(map_->segmentIntersectsObstacle(p1, p2));
}

TEST_F(SpatialIndexTest, SegmentDoesNotIntersect) {
    // Segment in empty space
    Point2D p1{50.0, 50.0};
    Point2D p2{60.0, 60.0};
    
    EXPECT_FALSE(map_->segmentIntersectsObstacle(p1, p2));
}

TEST_F(SpatialIndexTest, DistanceToNearestObstacle) {
    // Point near origin
    Point2D p{1.0, 1.0};
    
    float dist = map_->distanceToNearestObstacle(p);
    
    EXPECT_GT(dist, 0.0f);
    EXPECT_LT(dist, 2.0f);  // Should be roughly sqrt(2) from obstacle 1
}

TEST_F(SpatialIndexTest, DistanceInEmptySpace) {
    // Point far from all obstacles
    Point2D p{50.0, 50.0};
    
    float dist = map_->distanceToNearestObstacle(p);
    
    EXPECT_GT(dist, 30.0f);  // Far from nearest obstacle
}

TEST_F(SpatialIndexTest, DistanceAtObstacle) {
    // Point exactly at obstacle location
    Point2D p{0.0, 0.0};
    
    float dist = map_->distanceToNearestObstacle(p);
    
    EXPECT_LT(dist, 0.1f);  // Very close to zero
}

TEST_F(SpatialIndexTest, InsertAndQuery) {
    // Insert new obstacle
    map_->insertObstacle(5, rises::geofence::Point(20.0f, 20.0f));
    
    // Query nearby
    Point2D query_point{20.0, 20.0};
    std::vector<int64_t> results = map_->queryNearby(query_point, 1.0f);
    
    EXPECT_TRUE(std::find(results.begin(), results.end(), 5) != results.end());
}

TEST_F(SpatialIndexTest, RemoveAndQuery) {
    // Remove obstacle
    map_->removeObstacle(1);
    
    // Query where it was
    Point2D query_point{0.0, 0.0};
    std::vector<int64_t> results = map_->queryNearby(query_point, 1.0f);
    
    // Should not find removed obstacle
    EXPECT_TRUE(std::find(results.begin(), results.end(), 1) == results.end());
}

TEST_F(SpatialIndexTest, ObstacleCount) {
    // Count obstacles using forEachObstacle
    int count = 0;
    map_->forEachObstacle([&](const rises::geofence::GeometryEntry&) { ++count; });
    EXPECT_EQ(count, 4);
    
    map_->insertObstacle(10, rises::geofence::Point(30.0f, 30.0f));
    count = 0;
    map_->forEachObstacle([&](const rises::geofence::GeometryEntry&) { ++count; });
    EXPECT_EQ(count, 5);
    
    map_->removeObstacle(10);
    count = 0;
    map_->forEachObstacle([&](const rises::geofence::GeometryEntry&) { ++count; });
    EXPECT_EQ(count, 4);
}

TEST_F(SpatialIndexTest, GetObstacle) {
    const rises::geofence::Geometry* geom = map_->getObstacle(1);
    EXPECT_NE(geom, nullptr);
    
    const rises::geofence::Geometry* missing = map_->getObstacle(999);
    EXPECT_EQ(missing, nullptr);
}

TEST_F(SpatialIndexTest, ExpandingRadiusSearch) {
    // Point at reasonable distance from obstacles
    Point2D test_point{15.0, 15.0};
    
    float dist = map_->distanceToNearestObstacle(test_point);
    
    // Should find obstacle 3 at (10, 10) to (12, 12)
    // Distance should be roughly 3-5 meters
    EXPECT_GT(dist, 0.0f);
    EXPECT_LT(dist, 10.0f);
    EXPECT_FALSE(std::isinf(dist));
}

TEST_F(SpatialIndexTest, EmptyMapDistance) {
    // Create empty map
    std::function<std::shared_ptr<rises::geofence::SpatialIndex>()> factory =
        []() -> std::shared_ptr<rises::geofence::SpatialIndex> {
        return std::make_shared<rises::geofence::SpatialIndex>();
    };
    std::unique_ptr<rises::geofence::GeofenceMap> empty_map =
        std::make_unique<rises::geofence::GeofenceMap>(factory);
    
    Point2D p{10.0, 10.0};
    float dist = empty_map->distanceToNearestObstacle(p);
    
    // Empty map should return infinity
    EXPECT_TRUE(std::isinf(dist));
}

TEST_F(SpatialIndexTest, QueryBoundaryConditions) {
    // Query with zero radius
    Point2D query_point{0.0, 0.0};
    std::vector<int64_t> results = map_->queryNearby(query_point, 0.0f);
    
    // Should either be empty or contain only exact matches
    EXPECT_LE(results.size(), 1);
}

TEST_F(SpatialIndexTest, LargeRadiusQuery) {
    // Query with very large radius - should find all obstacles
    Point2D query_point{50.0, 50.0};
    std::vector<int64_t> results = map_->queryNearby(query_point, 1000.0f);
    
    // Should find all 4 obstacles
    EXPECT_EQ(results.size(), 4);
}

TEST_F(SpatialIndexTest, NegativeCoordinates) {
    // Insert obstacle at negative coordinates
    map_->insertObstacle(6, rises::geofence::Point(-10.0f, -10.0f));
    
    Point2D query_point{-10.0, -10.0};
    std::vector<int64_t> results = map_->queryNearby(query_point, 1.0f);
    
    EXPECT_TRUE(std::find(results.begin(), results.end(), 6) != results.end());
}

TEST_F(SpatialIndexTest, OverwriteObstacle) {
    // Insert obstacle at ID that already exists
    map_->insertObstacle(1, rises::geofence::Point(50.0f, 50.0f));
    
    // Query old location - should not find it
    Point2D old_location{0.0, 0.0};
    std::vector<int64_t> old_results = map_->queryNearby(old_location, 1.0f);
    EXPECT_TRUE(std::find(old_results.begin(), old_results.end(), 1) == old_results.end());
    
    // Query new location - should find it
    Point2D new_location{50.0, 50.0};
    std::vector<int64_t> new_results = map_->queryNearby(new_location, 1.0f);
    EXPECT_TRUE(std::find(new_results.begin(), new_results.end(), 1) != new_results.end());
}

TEST_F(SpatialIndexTest, SegmentThroughMultipleObstacles) {
    // Segment that passes through multiple obstacles
    Point2D p1{0.0, 0.0};
    Point2D p2{15.0, 15.0};
    
    // This should intersect multiple obstacles along its path
    EXPECT_TRUE(map_->segmentIntersectsObstacle(p1, p2));
}

TEST_F(SpatialIndexTest, ParallelSegmentToLine) {
    // Segment parallel to line obstacle but not intersecting
    Point2D p1{5.0, 3.0};  // Parallel to line at (5,5) to (10,5)
    Point2D p2{10.0, 3.0};
    
    EXPECT_FALSE(map_->segmentIntersectsObstacle(p1, p2));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
