#pragma once

#include <cmath>
#include <vector>
#include "rises_interfaces/msg/obstacle.hpp"

namespace rises::geofence::utils {

/**
 * @brief Utility class for geometry intersection and collision detection
 * 
 * Provides methods to check if obstacles intersect with circular regions,
 * useful for safety circle filtering and proximity queries.
 */
class GeometryIntersection {
public:
    /**
     * @brief Check if any part of an obstacle geometry intersects with a circular region
     * 
     * This method properly handles all obstacle types:
     * - POINT: Checks if point is within circle
     * - CIRCLE: Checks for circle-circle overlap
     * - LINE: Checks endpoints and closest point on segment
     * - POLYGON/RECTANGLE: Checks vertices and if circle center is inside polygon
     * 
     * @param obstacle The obstacle to check
     * @param circle_center_x Circle center X coordinate
     * @param circle_center_y Circle center Y coordinate
     * @param circle_radius Circle radius
     * @return true if any part of the obstacle intersects the circle
     */
    [[nodiscard]] static bool intersectsCircle(
        const rises_interfaces::msg::Obstacle& obstacle,
        double circle_center_x,
        double circle_center_y,
        double circle_radius);

private:
    /**
     * @brief Check if a point intersects a circle
     */
    static bool pointIntersectsCircle(
        double point_x, double point_y,
        double circle_x, double circle_y,
        double radius);

    /**
     * @brief Check if two circles overlap
     */
    static bool circleIntersectsCircle(
        double c1_x, double c1_y, double c1_radius,
        double c2_x, double c2_y, double c2_radius);

    /**
     * @brief Check if a line segment intersects a circle
     */
    static bool lineIntersectsCircle(
        double line_x1, double line_y1,
        double line_x2, double line_y2,
        double circle_x, double circle_y,
        double radius);

    /**
     * @brief Check if a polygon intersects a circle
     * 
     * Checks:
     * 1. If any vertex is inside the circle
     * 2. If the circle center is inside the polygon
     */
    static bool polygonIntersectsCircle(
        const std::vector<geometry_msgs::msg::Point>& vertices,
        double circle_x, double circle_y,
        double radius);

    /**
     * @brief Point-in-polygon test using ray casting
     */
    static bool pointInPolygon(
        double point_x, double point_y,
        const std::vector<geometry_msgs::msg::Point>& vertices);

    /**
     * @brief Get closest point on line segment to a point
     * @return Distance squared from point to closest point on segment
     */
    static double closestPointOnSegmentDistSq(
        double seg_x1, double seg_y1,
        double seg_x2, double seg_y2,
        double point_x, double point_y);
};

} // namespace rises::geofence::utils
