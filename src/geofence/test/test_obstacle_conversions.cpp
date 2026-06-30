#include <gtest/gtest.h>
#include "geofence/common/geometry/variant_geometry.hpp"
#include <rises_interfaces/msg/obstacle.hpp>
#include <cmath>


// ============================================================================
// Point Conversion Tests
// ============================================================================

TEST(ConversionTest, FromObstaclePoint_BasicConversion) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    obs.position.x = 5.5;
    obs.position.y = 3.2;
    
    rises::geofence::Point result = rises::geofence::fromObstaclePoint(obs);
    
    EXPECT_FLOAT_EQ(result.x(), 5.5f);
    EXPECT_FLOAT_EQ(result.y(), 3.2f);
}

TEST(ConversionTest, FromObstaclePoint_NegativeCoordinates) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    obs.position.x = -10.5;
    obs.position.y = -7.3;
    
    rises::geofence::Point result = rises::geofence::fromObstaclePoint(obs);
    
    EXPECT_FLOAT_EQ(result.x(), -10.5f);
    EXPECT_FLOAT_EQ(result.y(), -7.3f);
}

TEST(ConversionTest, FromObstaclePoint_Origin) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    obs.position.x = 0.0;
    obs.position.y = 0.0;
    
    rises::geofence::Point result = rises::geofence::fromObstaclePoint(obs);
    
    EXPECT_FLOAT_EQ(result.x(), 0.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.0f);
}

// ============================================================================
// Line Conversion Tests
// ============================================================================

TEST(ConversionTest, FromObstacleLine_HorizontalLine) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 5.0; p1.z = 0.0;
    p2.x = 10.0; p2.y = 5.0; p2.z = 0.0;
    obs.vertices = {p1, p2};
    
    rises::geofence::Line result = rises::geofence::fromObstacleLine(obs);
    
    EXPECT_FLOAT_EQ(result.first.x(), 0.0f);
    EXPECT_FLOAT_EQ(result.first.y(), 5.0f);
    EXPECT_FLOAT_EQ(result.second.x(), 10.0f);
    EXPECT_FLOAT_EQ(result.second.y(), 5.0f);
}

TEST(ConversionTest, FromObstacleLine_VerticalLine) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = 3.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 3.0; p2.y = 8.0; p2.z = 0.0;
    obs.vertices = {p1, p2};
    
    rises::geofence::Line result = rises::geofence::fromObstacleLine(obs);
    
    EXPECT_FLOAT_EQ(result.first.x(), 3.0f);
    EXPECT_FLOAT_EQ(result.first.y(), 0.0f);
    EXPECT_FLOAT_EQ(result.second.x(), 3.0f);
    EXPECT_FLOAT_EQ(result.second.y(), 8.0f);
}

TEST(ConversionTest, FromObstacleLine_DiagonalLine) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = 1.0; p1.y = 1.0; p1.z = 0.0;
    p2.x = 5.0; p2.y = 5.0; p2.z = 0.0;
    obs.vertices = {p1, p2};
    
    rises::geofence::Line result = rises::geofence::fromObstacleLine(obs);
    
    EXPECT_FLOAT_EQ(result.first.x(), 1.0f);
    EXPECT_FLOAT_EQ(result.first.y(), 1.0f);
    EXPECT_FLOAT_EQ(result.second.x(), 5.0f);
    EXPECT_FLOAT_EQ(result.second.y(), 5.0f);
}

TEST(ConversionTest, FromObstacleLine_InvalidVertexCount) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    // Only one vertex (invalid for line)
    geometry_msgs::msg::Point p1;
    p1.x = 1.0; p1.y = 1.0; p1.z = 0.0;
    obs.vertices = {p1};
    
    // Should throw or handle error gracefully
    EXPECT_THROW(rises::geofence::fromObstacleLine(obs), std::invalid_argument);
}

// ============================================================================
// Rectangle Conversion Tests
// ============================================================================

TEST(ConversionTest, FromObstacleRectangle_AxisAligned) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 5.0;
    obs.position.y = 3.0;
    obs.width = 4.0;
    obs.height = 2.0;
    obs.orientation = 0.0;  // No rotation
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    EXPECT_FLOAT_EQ(rises::geofence::center(result).x(), 5.0f);
    EXPECT_FLOAT_EQ(rises::geofence::center(result).y(), 3.0f);
    float width = result.max_corner().x() - result.min_corner().x();
    float height = result.max_corner().y() - result.min_corner().y();
    EXPECT_NEAR(width, 4.0f, 0.01f);
    EXPECT_NEAR(height, 2.0f, 0.01f);
}

TEST(ConversionTest, FromObstacleRectangle_SmallRectangle) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 0.0;
    obs.position.y = 0.0;
    obs.width = 0.1;
    obs.height = 0.1;
    obs.orientation = 0.0;
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    float width = result.max_corner().x() - result.min_corner().x();
    float height = result.max_corner().y() - result.min_corner().y();
    EXPECT_NEAR(width, 0.1f, 0.01f);
    EXPECT_NEAR(height, 0.1f, 0.01f);
}

TEST(ConversionTest, FromObstacleRectangle_Rotated45Degrees) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 0.0;
    obs.position.y = 0.0;
    obs.width = 2.0;
    obs.height = 2.0;
    obs.orientation = M_PI / 4.0;  // 45 degrees
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    // For a rotated square, the AABB should be larger
    // At 45 degrees, width and height should be ~sqrt(2) * side
    float expected_aabb_size = 2.0f * std::sqrt(2.0f);
    float width = result.max_corner().x() - result.min_corner().x();
    float height = result.max_corner().y() - result.min_corner().y();
    EXPECT_NEAR(width, expected_aabb_size, 0.01f);
    EXPECT_NEAR(height, expected_aabb_size, 0.01f);
}

TEST(ConversionTest, FromObstacleRectangle_Rotated90Degrees) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 5.0;
    obs.position.y = 5.0;
    obs.width = 4.0;  // Longer in x
    obs.height = 2.0;  // Shorter in y
    obs.orientation = M_PI / 2.0;  // 90 degrees
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    // After 90 degree rotation, width and height should swap
    float width = result.max_corner().x() - result.min_corner().x();
    float height = result.max_corner().y() - result.min_corner().y();
    EXPECT_NEAR(width, 2.0f, 0.01f);
    EXPECT_NEAR(height, 4.0f, 0.01f);
}

TEST(ConversionTest, FromObstacleRectangle_NegativePosition) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = -10.0;
    obs.position.y = -5.0;
    obs.width = 3.0;
    obs.height = 1.5;
    obs.orientation = 0.0;
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    // AABB center should account for negative position
    EXPECT_FLOAT_EQ(rises::geofence::center(result).x(), -10.0f);
    EXPECT_FLOAT_EQ(rises::geofence::center(result).y(), -5.0f);
}

// ============================================================================
// Polygon Conversion Tests
// ============================================================================

TEST(ConversionTest, FromObstaclePolygon_Triangle) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 4.0; p2.y = 0.0; p2.z = 0.0;
    p3.x = 2.0; p3.y = 3.0; p3.z = 0.0;
    obs.vertices = {p1, p2, p3};
    
    rises::geofence::Polygon result = rises::geofence::fromObstaclePolygon(obs);
    
    ASSERT_EQ(result.outer().size(), 3);
    EXPECT_FLOAT_EQ(result.outer()[0].x(), 0.0f);
    EXPECT_FLOAT_EQ(result.outer()[0].y(), 0.0f);
    EXPECT_FLOAT_EQ(result.outer()[1].x(), 4.0f);
    EXPECT_FLOAT_EQ(result.outer()[1].y(), 0.0f);
    EXPECT_FLOAT_EQ(result.outer()[2].x(), 2.0f);
    EXPECT_FLOAT_EQ(result.outer()[2].y(), 3.0f);
}

TEST(ConversionTest, FromObstaclePolygon_Square) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 2.0; p2.y = 0.0; p2.z = 0.0;
    p3.x = 2.0; p3.y = 2.0; p3.z = 0.0;
    p4.x = 0.0; p4.y = 2.0; p4.z = 0.0;
    obs.vertices = {p1, p2, p3, p4};
    
    rises::geofence::Polygon result = rises::geofence::fromObstaclePolygon(obs);
    
    ASSERT_EQ(result.outer().size(), 4);
    EXPECT_FLOAT_EQ(result.outer()[0].x(), 0.0f);
    EXPECT_FLOAT_EQ(result.outer()[3].y(), 2.0f);
}

TEST(ConversionTest, FromObstaclePolygon_ComplexShape) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    // Pentagon
    std::vector<geometry_msgs::msg::Point> vertices;
    for (int i = 0; i < 5; ++i) {
        geometry_msgs::msg::Point p;
        double angle = 2.0 * M_PI * i / 5.0;
        p.x = std::cos(angle);
        p.y = std::sin(angle);
        p.z = 0.0;
        vertices.push_back(p);
    }
    obs.vertices = vertices;
    
    rises::geofence::Polygon result = rises::geofence::fromObstaclePolygon(obs);
    
    ASSERT_EQ(result.outer().size(), 5);
    // Check first vertex (should be at angle 0)
    EXPECT_NEAR(result.outer()[0].x(), 1.0f, 0.01f);
    EXPECT_NEAR(result.outer()[0].y(), 0.0f, 0.01f);
}

TEST(ConversionTest, FromObstaclePolygon_EmptyVertices) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    obs.vertices = {};  // Empty
    
    // Should throw or return empty polygon
    EXPECT_THROW(rises::geofence::fromObstaclePolygon(obs), std::invalid_argument);
}

TEST(ConversionTest, FromObstaclePolygon_SingleVertex) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    geometry_msgs::msg::Point p1;
    p1.x = 1.0; p1.y = 1.0; p1.z = 0.0;
    obs.vertices = {p1};
    
    // Single vertex polygon is degenerate
    EXPECT_THROW(rises::geofence::fromObstaclePolygon(obs), std::invalid_argument);
}

// ============================================================================
// Round-Trip Conversion Tests
// ============================================================================

TEST(ConversionTest, RoundTrip_Point) {
    // Create backend point
    rises::geofence::Point original{7.5f, 3.2f};
    
    // Convert to Obstacle message
    rises_interfaces::msg::Obstacle msg = rises::geofence::toObstacle(original, 123);
    
    // Convert back
    rises::geofence::Point recovered = rises::geofence::fromObstaclePoint(msg);
    
    EXPECT_FLOAT_EQ(original.x(), recovered.x());
    EXPECT_FLOAT_EQ(original.y(), recovered.y());
    EXPECT_EQ(msg.id, 123);
    EXPECT_EQ(msg.type, rises_interfaces::msg::Obstacle::POINT);
}

TEST(ConversionTest, RoundTrip_Line) {
    // Create backend line
    rises::geofence::Line original = rises::geofence::makeLine(1.0f, 2.0f, 5.0f, 8.0f);
    
    // Convert to Obstacle message
    rises_interfaces::msg::Obstacle msg = rises::geofence::toObstacle(original, 456);
    
    // Convert back
    rises::geofence::Line recovered = rises::geofence::fromObstacleLine(msg);
    
    EXPECT_FLOAT_EQ(original.first.x(), recovered.first.x());
    EXPECT_FLOAT_EQ(original.first.y(), recovered.first.y());
    EXPECT_FLOAT_EQ(original.second.x(), recovered.second.x());
    EXPECT_FLOAT_EQ(original.second.y(), recovered.second.y());
    EXPECT_EQ(msg.id, 456);
    EXPECT_EQ(msg.type, rises_interfaces::msg::Obstacle::LINE);
}

TEST(ConversionTest, RoundTrip_Rectangle) {
    // Create backend rectangle (axis-aligned)
    rises::geofence::Rectangle original = rises::geofence::makeRectangle(7.0f, 3.5f, 13.0f, 6.5f);
    
    // Convert to Obstacle message
    rises_interfaces::msg::Obstacle msg = rises::geofence::toObstacle(original, 789);
    
    // Convert back (will be AABB of potentially rotated rectangle)
    rises::geofence::Rectangle recovered = rises::geofence::fromObstacleRectangle(msg);
    
    // For axis-aligned rectangles, should be exact
    EXPECT_NEAR(rises::geofence::center(original).x(), rises::geofence::center(recovered).x(), 0.01f);
    EXPECT_NEAR(rises::geofence::center(original).y(), rises::geofence::center(recovered).y(), 0.01f);
    float orig_width = original.max_corner().x() - original.min_corner().x();
    float orig_height = original.max_corner().y() - original.min_corner().y();
    float rec_width = recovered.max_corner().x() - recovered.min_corner().x();
    float rec_height = recovered.max_corner().y() - recovered.min_corner().y();
    EXPECT_NEAR(orig_width, rec_width, 0.01f);
    EXPECT_NEAR(orig_height, rec_height, 0.01f);
    EXPECT_EQ(msg.id, 789);
    EXPECT_EQ(msg.type, rises_interfaces::msg::Obstacle::RECTANGLE);
}

TEST(ConversionTest, RoundTrip_Polygon) {
    // Create backend polygon
    std::vector<bg::model::d2::point_xy<float>> verts = {
        {0.0f, 0.0f}, {3.0f, 0.0f}, {3.0f, 2.0f}, {0.0f, 2.0f}
    };
    rises::geofence::Polygon original = rises::geofence::makePolygon(verts);
    
    // Convert to Obstacle message
    rises_interfaces::msg::Obstacle msg = rises::geofence::toObstacle(original, 999);
    
    // Convert back
    rises::geofence::Polygon recovered = rises::geofence::fromObstaclePolygon(msg);
    
    ASSERT_EQ(original.outer().size(), recovered.outer().size());
    for (size_t i = 0; i < original.outer().size(); ++i) {
        EXPECT_FLOAT_EQ(original.outer()[i].x(), recovered.outer()[i].x());
        EXPECT_FLOAT_EQ(original.outer()[i].y(), recovered.outer()[i].y());
    }
    EXPECT_EQ(msg.id, 999);
    EXPECT_EQ(msg.type, rises_interfaces::msg::Obstacle::POLYGON);
}

// ============================================================================
// Type Validation Tests
// ============================================================================

TEST(ConversionTest, WrongType_PointAsLine) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    obs.position.x = 5.0;
    obs.position.y = 3.0;
    
    // Trying to convert a POINT type as LINE should fail
    EXPECT_THROW(rises::geofence::fromObstacleLine(obs), std::invalid_argument);
}

TEST(ConversionTest, WrongType_RectangleAsPoint) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 5.0;
    obs.position.y = 3.0;
    obs.width = 2.0;
    obs.height = 1.0;
    
    // Rectangle should not be convertible to Point
    // (depending on implementation, might use position only)
    // This test documents expected behavior
}

// ============================================================================
// Additional Edge Case Tests
// ============================================================================

TEST(ConversionTest, Rectangle_Rotation180Degrees) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 5.0;
    obs.position.y = 3.0;
    obs.width = 4.0;
    obs.height = 2.0;
    obs.orientation = M_PI;  // 180 degrees
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    // 180° rotation should still produce valid AABB
    EXPECT_GT(result.max_corner().x() - result.min_corner().x(), 0.0f);
    EXPECT_GT(result.max_corner().y() - result.min_corner().y(), 0.0f);
    
    // Center should be preserved
    EXPECT_NEAR(rises::geofence::center(result).x(), 5.0f, 0.01f);
    EXPECT_NEAR(rises::geofence::center(result).y(), 3.0f, 0.01f);
}

TEST(ConversionTest, Rectangle_Rotation270Degrees) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 10.0;
    obs.position.y = 5.0;
    obs.width = 6.0;
    obs.height = 2.0;
    obs.orientation = 3.0f * M_PI / 2.0f;  // 270 degrees
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    // Should swap dimensions (90° rotation effect)
    float width = result.max_corner().x() - result.min_corner().x();
    float height = result.max_corner().y() - result.min_corner().y();
    
    // For 270° rotation, AABB should be ~2x6 (swapped from 6x2)
    EXPECT_NEAR(width, 2.0f, 0.01f);
    EXPECT_NEAR(height, 6.0f, 0.01f);
}

TEST(ConversionTest, Rectangle_VerySmallDimensions) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 1.0;
    obs.position.y = 1.0;
    obs.width = 0.001f;  // 1mm
    obs.height = 0.001f;
    obs.orientation = 0.0f;
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    EXPECT_NEAR(rises::geofence::center(result).x(), 1.0f, 0.0001f);
    EXPECT_NEAR(rises::geofence::center(result).y(), 1.0f, 0.0001f);
    EXPECT_NEAR(result.max_corner().x() - result.min_corner().x(), 0.001f, 0.0001f);
}

TEST(ConversionTest, Rectangle_VeryLargeDimensions) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = 0.0;
    obs.position.y = 0.0;
    obs.width = 1000.0f;  // 1km
    obs.height = 500.0f;
    obs.orientation = 0.0f;
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    EXPECT_NEAR(result.max_corner().x() - result.min_corner().x(), 1000.0f, 0.01f);
    EXPECT_NEAR(result.max_corner().y() - result.min_corner().y(), 500.0f, 0.01f);
}

TEST(ConversionTest, Rectangle_NegativePosition) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    obs.position.x = -50.0;
    obs.position.y = -30.0;
    obs.width = 10.0;
    obs.height = 5.0;
    obs.orientation = 0.0f;
    
    rises::geofence::Rectangle result = rises::geofence::fromObstacleRectangle(obs);
    
    EXPECT_NEAR(rises::geofence::center(result).x(), -50.0f, 0.01f);
    EXPECT_NEAR(rises::geofence::center(result).y(), -30.0f, 0.01f);
    EXPECT_NEAR(result.min_corner().x(), -55.0f, 0.01f);
    EXPECT_NEAR(result.max_corner().x(), -45.0f, 0.01f);
}

TEST(ConversionTest, Line_VeryShort) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = 1.0; p1.y = 1.0; p1.z = 0.0;
    p2.x = 1.001; p2.y = 1.001; p2.z = 0.0;  // 1.4mm line
    obs.vertices = {p1, p2};
    
    rises::geofence::Line result = rises::geofence::fromObstacleLine(obs);
    
    float length = std::sqrt(
        std::pow(result.second.x() - result.first.x(), 2) +
        std::pow(result.second.y() - result.first.y(), 2)
    );
    
    EXPECT_NEAR(length, 0.001414f, 0.0001f);
}

TEST(ConversionTest, Line_VeryLong) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 1000.0; p2.y = 1000.0; p2.z = 0.0;  // 1.4km line
    obs.vertices = {p1, p2};
    
    rises::geofence::Line result = rises::geofence::fromObstacleLine(obs);
    
    EXPECT_FLOAT_EQ(result.first.x(), 0.0f);
    EXPECT_FLOAT_EQ(result.second.x(), 1000.0f);
}

TEST(ConversionTest, Polygon_Triangle) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 1.0; p2.y = 0.0; p2.z = 0.0;
    p3.x = 0.5; p3.y = 1.0; p3.z = 0.0;
    obs.vertices = {p1, p2, p3};
    
    rises::geofence::Polygon result = rises::geofence::fromObstaclePolygon(obs);
    
    ASSERT_EQ(result.outer().size(), 3);
    EXPECT_FLOAT_EQ(result.outer()[0].x(), 0.0f);
    EXPECT_FLOAT_EQ(result.outer()[1].x(), 1.0f);
    EXPECT_FLOAT_EQ(result.outer()[2].x(), 0.5f);
}

TEST(ConversionTest, Polygon_TwoVertices_Invalid) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 1.0; p2.y = 1.0; p2.z = 0.0;
    obs.vertices = {p1, p2};
    
    // Should throw - polygon needs at least 3 vertices
    EXPECT_THROW(rises::geofence::fromObstaclePolygon(obs), std::invalid_argument);
}

TEST(ConversionTest, Polygon_ManyVertices) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    // Create 100-sided polygon
    for (int i = 0; i < 100; ++i) {
        geometry_msgs::msg::Point pt;
        float angle = i * 2.0f * M_PI / 100.0f;
        pt.x = std::cos(angle) * 10.0;
        pt.y = std::sin(angle) * 10.0;
        pt.z = 0.0;
        obs.vertices.push_back(pt);
    }
    
    rises::geofence::Polygon result = rises::geofence::fromObstaclePolygon(obs);
    
    EXPECT_EQ(result.outer().size(), 100);
}

TEST(ConversionTest, Point_AtOrigin) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    obs.position.x = 0.0;
    obs.position.y = 0.0;
    
    rises::geofence::Point result = rises::geofence::fromObstaclePoint(obs);
    
    EXPECT_FLOAT_EQ(result.x(), 0.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.0f);
}

TEST(ConversionTest, Point_VeryFarAway) {
    rises_interfaces::msg::Obstacle obs;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    obs.position.x = 10000.0;
    obs.position.y = -10000.0;
    
    rises::geofence::Point result = rises::geofence::fromObstaclePoint(obs);
    
    EXPECT_FLOAT_EQ(result.x(), 10000.0f);
    EXPECT_FLOAT_EQ(result.y(), -10000.0f);
}

TEST(ConversionTest, RoundTrip_RotatedRectangle) {
    // Create rotated rectangle
    rises::geofence::Rectangle original = rises::geofence::makeRectangle(-1.0f, -1.0f, 1.0f, 1.0f);
    
    // Convert to message (will be axis-aligned)
    rises_interfaces::msg::Obstacle msg = rises::geofence::toObstacle(original, 999);
    
    // Convert back
    rises::geofence::Rectangle recovered = rises::geofence::fromObstacleRectangle(msg);
    
    // Centers should match
    EXPECT_NEAR(rises::geofence::center(original).x(), rises::geofence::center(recovered).x(), 0.01f);
    EXPECT_NEAR(rises::geofence::center(original).y(), rises::geofence::center(recovered).y(), 0.01f);
    
    // Sizes should match
    float orig_width = original.max_corner().x() - original.min_corner().x();
    float orig_height = original.max_corner().y() - original.min_corner().y();
    float rec_width = recovered.max_corner().x() - recovered.min_corner().x();
    float rec_height = recovered.max_corner().y() - recovered.min_corner().y();
    EXPECT_NEAR(orig_width, rec_width, 0.01f);
    EXPECT_NEAR(orig_height, rec_height, 0.01f);
}

TEST(ConversionTest, RoundTrip_LineWithNegativeCoords) {
    rises::geofence::Line original = rises::geofence::makeLine(-5.0f, -3.0f, -1.0f, -7.0f);
    
    rises_interfaces::msg::Obstacle msg = rises::geofence::toObstacle(original, 888);
    rises::geofence::Line recovered = rises::geofence::fromObstacleLine(msg);
    
    EXPECT_FLOAT_EQ(original.first.x(), recovered.first.x());
    EXPECT_FLOAT_EQ(original.first.y(), recovered.first.y());
    EXPECT_FLOAT_EQ(original.second.x(), recovered.second.x());
    EXPECT_FLOAT_EQ(original.second.y(), recovered.second.y());
}

TEST(ConversionTest, RoundTrip_ComplexPolygon) {
    // Star-shaped polygon
    std::vector<bg::model::d2::point_xy<float>> star_points;
    for (int i = 0; i < 10; ++i) {
        float angle = i * 2.0f * M_PI / 10.0f;
        float radius = (i % 2 == 0) ? 2.0f : 1.0f;
        star_points.push_back({radius * std::cos(angle), radius * std::sin(angle)});
    }
    
    rises::geofence::Polygon original = rises::geofence::makePolygon(star_points);
    
    rises_interfaces::msg::Obstacle msg = rises::geofence::toObstacle(original, 777);
    rises::geofence::Polygon recovered = rises::geofence::fromObstaclePolygon(msg);
    
    ASSERT_EQ(original.outer().size(), recovered.outer().size());
    for (size_t i = 0; i < original.outer().size(); ++i) {
        EXPECT_NEAR(original.outer()[i].x(), recovered.outer()[i].x(), 0.01f);
        EXPECT_NEAR(original.outer()[i].y(), recovered.outer()[i].y(), 0.01f);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
