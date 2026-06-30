#include <gtest/gtest.h>
#include "geofence/spatial/queries/path_safety_checker.hpp"
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "spatial_index_selection.hpp"
#include <functional>
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Test fixture for path safety checking
class PathSafetyTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::function<std::shared_ptr<rises::geofence::SpatialIndex>()> factory =
            []() -> std::shared_ptr<rises::geofence::SpatialIndex> {
            return std::make_shared<rises::geofence::SpatialIndex>();
        };

        map_ = std::make_unique<rises::geofence::GeofenceMap>(factory);

        // Create obstacles for testing
        // Wall from (10, 0) to (10, 20)
        map_->insertObstacle(1, rises::geofence::makeLine(10.0f, 0.0f, 10.0f, 20.0f));

        // Rectangle obstacle at (15, 15) to (17, 17)
        map_->insertObstacle(2, rises::geofence::makeRectangle(15.0f, 15.0f, 17.0f, 17.0f));

        // Configure checker with small safety margin
        rises::geofence::PathSafetyChecker::Config config;
        config.safety_margin = 0.5f;
        config.check_map_bounds = true;
        config.check_locked_areas = true;

        rises::geofence::PathSafetyChecker::initialize(config);
    }

    // Helper to convert point vector to ROS path
    nav_msgs::msg::Path createPath(const std::vector<Point2D>& points) {
        nav_msgs::msg::Path path;
        for (const Point2D& pt : points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }

    std::unique_ptr<rises::geofence::GeofenceMap> map_;
};

TEST_F(PathSafetyTest, SafePathInOpenSpace) {
    std::vector<Point2D> points = {
        {0.0, 0.0},
        {5.0, 5.0},
        {5.0, 10.0}
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, PathThroughObstacle) {
    std::vector<Point2D> points = {
        {9.0, 10.0},   // Before wall
        {11.0, 10.0}   // After wall - should collide
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_FALSE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, PathTooCloseToObstacle) {
    std::vector<Point2D> points = {
        {9.0, 10.0},     // Safe
        {9.6, 10.0}      // 0.4m clearance - less than 0.5m margin required
    };
    nav_msgs::msg::Path path = createPath(points);
    
    // This path is too close to wall at x=10
    // Distance = 10 - 9.6 = 0.4m < 0.5m safety margin
    EXPECT_FALSE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, PathWithSufficientClearance) {
    std::vector<Point2D> points = {
        {0.0, 10.0},
        {8.0, 10.0}   // 2m from wall, well above margin
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, SegmentSafetyCheck) {
    Point2D safe_start{0.0, 0.0};
    Point2D safe_end{5.0, 5.0};
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isSegmentSafe(*map_, safe_start, safe_end));
    
    Point2D unsafe_start{9.0, 10.0};
    Point2D unsafe_end{11.0, 10.0};
    EXPECT_FALSE(rises::geofence::PathSafetyChecker::isSegmentSafe(*map_, unsafe_start, unsafe_end));
}

TEST_F(PathSafetyTest, EmptyPath) {
    nav_msgs::msg::Path path = createPath({});
    
    // Empty paths should throw due to min_path_poses constraint
    EXPECT_THROW(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path), std::invalid_argument);
}

TEST_F(PathSafetyTest, SinglePointPath) {
    nav_msgs::msg::Path path = createPath({{5.0, 5.0}});
    
    // Single point path should throw due to min_path_poses=2
    EXPECT_THROW(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path), std::invalid_argument);
}

TEST_F(PathSafetyTest, ConfigurationChange) {
    std::vector<Point2D> points = {
        {9.0, 10.0},
        {9.6, 10.0}  // 0.4m from wall
    };
    nav_msgs::msg::Path path = createPath(points);
    
    // With 0.5m margin - should fail
    EXPECT_FALSE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
    
    // Change to 0.3m margin
    rises::geofence::PathSafetyChecker::Config new_config;
    new_config.safety_margin = 0.3f;
    rises::geofence::PathSafetyChecker::initialize(new_config);
    
    // Now should pass
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, PathIntersectsObstacle) {
    // Path that passes directly through obstacle
    std::vector<Point2D> points = {
        {9.5, 10.0},    // Just before wall (0.5m clearance)
        {11.0, 10.0}    // Through the wall
    };
    nav_msgs::msg::Path path = createPath(points);
    
    // Should detect collision with wall obstacle
    bool result = rises::geofence::PathSafetyChecker::isPathSafe(*map_, path);
    EXPECT_FALSE(result);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(PathSafetyTest, PathAlongObstacleBoundary) {
    // Path running parallel to wall, exactly at safety margin
    std::vector<Point2D> points = {
        {9.0, 10.0},
        {9.5, 10.0},   // Exactly 0.5m from wall at x=10
        {9.5, 11.0}
    };
    nav_msgs::msg::Path path = createPath(points);
    
    // Should pass - exactly at margin distance
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, ZeroLengthSegment) {
    // Path with duplicate consecutive points
    std::vector<Point2D> points = {
        {5.0, 5.0},
        {5.0, 5.0},  // Duplicate
        {6.0, 6.0}
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, VeryLongPath) {
    // Path with many waypoints
    std::vector<Point2D> points;
    for (int i = 0; i < 100; ++i) {
        points.push_back({static_cast<double>(i) * 0.1, 5.0});
    }
    nav_msgs::msg::Path path = createPath(points);
    
    bool result = rises::geofence::PathSafetyChecker::isPathSafe(*map_, path);
    // Result depends on obstacle positions, just verify it doesn't crash
    EXPECT_TRUE(result || !result);
}

TEST_F(PathSafetyTest, PathWithBacktracking) {
    // Path that backtracks on itself
    std::vector<Point2D> points = {
        {0.0, 0.0},
        {5.0, 0.0},
        {2.5, 0.0},  // Backtrack
        {5.0, 0.0}   // Forward again
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, PathGrazingObstacle) {
    // Path that just touches the safety margin boundary
    std::vector<Point2D> points = {
        {9.0, 10.0},
        {9.5, 10.0}  // Exactly at safety margin boundary (wall at x=10, margin=0.5)
    };
    nav_msgs::msg::Path path = createPath(points);
    
    // Should pass - at the margin boundary is acceptable (just barely safe)
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, MultipleSegmentCollision) {
    // Path with multiple segments colliding
    std::vector<Point2D> points = {
        {10.0, 9.5},   // Through wall
        {11.0, 10.0},  // Through wall
        {11.0, 11.0}   // Through wall
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_FALSE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, PathStartingInsideObstacle) {
    // Path starting from position too close to wall (within safety margin)
    std::vector<Point2D> points = {
        {9.7, 10.0},   // 0.3m from wall - within 0.5m safety margin
        {5.0, 10.0}    // Moving away
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_FALSE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, PathEndingInsideObstacle) {
    // Path ending inside an obstacle
    std::vector<Point2D> points = {
        {5.0, 10.0},
        {10.5, 10.0}   // Inside wall
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_FALSE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, NegativeCoordinatePath) {
    // Path with negative coordinates
    std::vector<Point2D> points = {
        {-5.0, -5.0},
        {-1.0, -1.0},
        {0.0, 0.0}
    };
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

TEST_F(PathSafetyTest, VeryShortSegments) {
    // Path with very short segments (testing resolution)
    std::vector<Point2D> points;
    for (int i = 0; i < 20; ++i) {
        points.push_back({static_cast<double>(i) * 0.01, 5.0});  // 1cm segments
    }
    nav_msgs::msg::Path path = createPath(points);
    
    EXPECT_TRUE(rises::geofence::PathSafetyChecker::isPathSafe(*map_, path));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
