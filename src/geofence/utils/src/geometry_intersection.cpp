#include "geofence/utils/geometry_intersection.hpp"
#include "geofence/utils/compiler_hints.hpp"
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace rises::geofence::utils {

bool GeometryIntersection::intersectsCircle(
    const rises_interfaces::msg::Obstacle& obstacle,
    const double circle_center_x,
    const double circle_center_y,
    const double circle_radius) {
    
    switch (obstacle.type) {
        case rises_interfaces::msg::Obstacle::POINT:
            return pointIntersectsCircle(
                obstacle.position.x, obstacle.position.y,
                circle_center_x, circle_center_y, circle_radius);
        
        case rises_interfaces::msg::Obstacle::CIRCLE:
            return circleIntersectsCircle(
                obstacle.position.x, obstacle.position.y, obstacle.radius,
                circle_center_x, circle_center_y, circle_radius);
        
        case rises_interfaces::msg::Obstacle::LINE:
            if (obstacle.vertices.size() >= 2) {
                return lineIntersectsCircle(
                    obstacle.vertices[0].x, obstacle.vertices[0].y,
                    obstacle.vertices[1].x, obstacle.vertices[1].y,
                    circle_center_x, circle_center_y, circle_radius);
            }
            // Fallback to position if no vertices
            return pointIntersectsCircle(
                obstacle.position.x, obstacle.position.y,
                circle_center_x, circle_center_y, circle_radius);
        
        case rises_interfaces::msg::Obstacle::POLYGON:
        case rises_interfaces::msg::Obstacle::RECTANGLE:
        case rises_interfaces::msg::Obstacle::CONVEX_POLYGON:
        case rises_interfaces::msg::Obstacle::FREEFORM:
            if (LIKELY(!obstacle.vertices.empty())) {
                return polygonIntersectsCircle(
                    obstacle.vertices,
                    circle_center_x, circle_center_y, circle_radius);
            }
            // Fallback to position if no vertices
            return pointIntersectsCircle(
                obstacle.position.x, obstacle.position.y,
                circle_center_x, circle_center_y, circle_radius);
        
        default:
            // Unknown type: use center point check as fallback
            return pointIntersectsCircle(
                obstacle.position.x, obstacle.position.y,
                circle_center_x, circle_center_y, circle_radius);
    }
}

// Check if a point intersects with a circle (within radius)
bool GeometryIntersection::pointIntersectsCircle(
    const double point_x, const double point_y,
    const double circle_x, const double circle_y,
    const double radius) {
    
    const boost::geometry::model::d2::point_xy<double> p(point_x, point_y);
    const boost::geometry::model::d2::point_xy<double> c(circle_x, circle_y);
    return boost::geometry::distance(p, c) <= radius;
}

// Check if two circles intersect
bool GeometryIntersection::circleIntersectsCircle(
    const double c1_x, const double c1_y, const double c1_radius,
    const double c2_x, const double c2_y, const double c2_radius) {
    
    const boost::geometry::model::d2::point_xy<double> p1(c1_x, c1_y);
    const boost::geometry::model::d2::point_xy<double> p2(c2_x, c2_y);
    const double center_dist = boost::geometry::distance(p1, p2);
    const double combined_radius = c1_radius + c2_radius;
    return center_dist <= combined_radius;
}

// Check if a line segment intersects with a circle
bool GeometryIntersection::lineIntersectsCircle(
    const double line_x1, const double line_y1,
    const double line_x2, const double line_y2,
    const double circle_x, const double circle_y,
    const double radius) {
    
    const boost::geometry::model::segment<boost::geometry::model::d2::point_xy<double>> seg(
        boost::geometry::model::d2::point_xy<double>(line_x1, line_y1),
        boost::geometry::model::d2::point_xy<double>(line_x2, line_y2));
    const boost::geometry::model::d2::point_xy<double> center(circle_x, circle_y);
    
    return boost::geometry::distance(seg, center) <= radius;
}

// Check if a polygon intersects with a circle
bool GeometryIntersection::polygonIntersectsCircle(
    const std::vector<geometry_msgs::msg::Point>& vertices,
    const double circle_x, const double circle_y,
    const double radius) {
    
    if (UNLIKELY(vertices.empty())) {
        return false;
    }
    
    // Build Boost.Geometry polygon
    boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly;
    for (const geometry_msgs::msg::Point& v : vertices) {
        boost::geometry::append(poly.outer(), boost::geometry::model::d2::point_xy<double>(v.x, v.y));
    }
    boost::geometry::correct(poly);  // Ensure correct orientation
    
    const boost::geometry::model::d2::point_xy<double> center(circle_x, circle_y);
    
    // Check if circle center is inside polygon
    if (boost::geometry::within(center, poly)) {
        return true;
    }
    
    // Check distance from center to polygon boundary
    return boost::geometry::distance(center, poly) <= radius;
}

// Check if a point is inside a polygon
bool GeometryIntersection::pointInPolygon(
    const double point_x, const double point_y,
    const std::vector<geometry_msgs::msg::Point>& vertices) {
    
    if (vertices.size() < 3) {
        return false;
    }
    
    // Build Boost.Geometry polygon
    boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly;
    for (const geometry_msgs::msg::Point& v : vertices) {
        boost::geometry::append(poly.outer(), boost::geometry::model::d2::point_xy<double>(v.x, v.y));
    }
    boost::geometry::correct(poly);
    
    const boost::geometry::model::d2::point_xy<double> pt(point_x, point_y);
    return boost::geometry::within(pt, poly);
}

// Calculate squared distance from point to closest point on line segment
double GeometryIntersection::closestPointOnSegmentDistSq(
    const double seg_x1, const double seg_y1,
    const double seg_x2, const double seg_y2,
    const double point_x, const double point_y) {
    
    const boost::geometry::model::segment<boost::geometry::model::d2::point_xy<double>> seg(
        boost::geometry::model::d2::point_xy<double>(seg_x1, seg_y1),
        boost::geometry::model::d2::point_xy<double>(seg_x2, seg_y2));
    const boost::geometry::model::d2::point_xy<double> pt(point_x, point_y);
    
    const double dist = boost::geometry::distance(seg, pt);
    return dist * dist;
}

} // namespace rises::geofence::utils
