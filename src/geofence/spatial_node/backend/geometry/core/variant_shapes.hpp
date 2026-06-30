#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <vector>
#include <cmath>
#include "rises_interfaces/msg/obstacle.hpp"

namespace bg = boost::geometry;

/**
 * @file variant_shapes.hpp
 * @brief Geometry type definitions for geofencing using Boost.Geometry
 * 
 * This is special-purpose code for warehouse AGV geofencing, not a reusable library.
 * We use Boost.Geometry directly and provide type aliases for convenience.
 * 
 * Type aliases vs direct boost types:
 * - Shorter: Point vs boost::geometry::model::d2::point_xy<float>
 * - Extensible: Added Circle which Boost.Geometry doesn't provide
 * - Documented: Centralized list of geometry types used throughout codebase
 * 
 * This is NOT an abstraction layer - we're committed to Boost.Geometry.
 * If you need a different geometry library, you'll need to update code throughout.
 * 
 * Dependencies: Boost.Geometry 1.74+ (ships with Boost 1.74.0+)
 */
namespace rises::geofence {

// ============================================================================
// Geometry Types (Boost.Geometry aliases + custom extensions)
// ============================================================================

// Boost.Geometry type aliases (zero overhead - just shorter names)
using Point = bg::model::d2::point_xy<float>;
using Line = bg::model::segment<Point>;
using Rectangle = bg::model::box<Point>;
using Polygon = bg::model::polygon<Point>;

// Circle - not natively supported by Boost.Geometry, so we define it ourselves
struct Circle {
    Point center;
    float radius;
    
    Circle() : center(0.0f, 0.0f), radius(0.0f) {}
    Circle(const Point& c, float r) : center(c), radius(r) {}
    Circle(float x, float y, float r) : center(x, y), radius(r) {}
};

// ============================================================================
// Helper Functions (replacing former member methods)
// ============================================================================

// Point construction helpers
inline Point makePoint(float x, float y) {
    return Point(x, y);
}

// Line helpers
inline Line makeLine(float x1, float y1, float x2, float y2) {
    return Line(Point(x1, y1), Point(x2, y2));
}

inline Line makeLine(const Point& p1, const Point& p2) {
    return Line(p1, p2);
}

inline float length(const Line& l) {
    return bg::distance(l.first, l.second);
}

/**
 * @brief Computes the normalized direction vector of a line segment
 * 
 * Returns a unit vector pointing from the start to the end of the line.
 * For degenerate lines (length < epsilon), returns a zero vector.
 * 
 * @param l The line segment
 * @return Normalized direction vector, or (0,0) for degenerate lines
 * 
 * @note Uses epsilon = 1e-6f to detect degenerate lines
 * @note Safe for zero-length lines (no division by zero)
 */
inline Point direction(const Line& l) {
    const float dx = l.second.x() - l.first.x();
    const float dy = l.second.y() - l.first.y();
    const float len = std::sqrt(dx*dx + dy*dy);
    if (len < 1e-6f) return Point(0, 0);
    return Point(dx/len, dy/len);
}

// Rectangle helpers
/**
 * @brief Creates a rectangle from corner coordinates
 * 
 * Creates an axis-aligned bounding box from minimum and maximum corners.
 * 
 * @param min_x Minimum X coordinate (left edge)
 * @param min_y Minimum Y coordinate (bottom edge)  
 * @param max_x Maximum X coordinate (right edge)
 * @param max_y Maximum Y coordinate (top edge)
 * @return Rectangle with specified bounds
 * 
 * @warning Caller must ensure min_x <= max_x and min_y <= max_y
 * @note Boost.Geometry may produce undefined behavior if min > max
 */
inline Rectangle makeRectangle(float min_x, float min_y, float max_x, float max_y) {
    return Rectangle(Point(min_x, min_y), Point(max_x, max_y));
}

inline Rectangle makeRectangle(const Point& min, const Point& max) {
    return Rectangle(min, max);
}

inline Point center(const Rectangle& r) {
    const auto& min = r.min_corner();
    const auto& max = r.max_corner();
    return Point((min.x() + max.x()) * 0.5f, (min.y() + max.y()) * 0.5f);
}

/**
 * @brief Computes the area of a rectangle
 * 
 * Calculates area as width * height. Always returns a non-negative value
 * for valid rectangles (where min <= max).
 * 
 * @param r The rectangle
 * @return Area in square units (same units as coordinates)
 * 
 * @warning Returns negative or zero for invalid rectangles (min > max)
 */
inline float area(const Rectangle& r) {
    const auto& min = r.min_corner();
    const auto& max = r.max_corner();
    return (max.x() - min.x()) * (max.y() - min.y());
}

// Polygon helpers
/**
 * @brief Creates a polygon from a vector of vertices
 * 
 * Constructs a closed polygon from the given vertices. The polygon is
 * automatically closed (last vertex connects to first) and corrected for
 * proper winding order per Boost.Geometry requirements.
 * 
 * @param verts Vector of vertices defining the polygon boundary
 * @return Closed, corrected polygon ready for geometric operations
 * 
 * @note Boost.Geometry expects counter-clockwise winding for outer rings
 * @note Empty vertex lists produce an empty (invalid) polygon
 * @note bg::correct() fixes winding order and closes the ring if needed
 * @note Minimum 3 non-collinear vertices required for valid polygon
 */
inline Polygon makePolygon(const std::vector<Point>& verts) {
    Polygon poly;
    for (const auto& v : verts) {
        bg::append(poly.outer(), v);
    }
    if (!verts.empty()) {
        bg::correct(poly);
    }
    return poly;
}

inline const auto& vertices(const Polygon& poly) {
    return poly.outer();
}

inline Point centroid(const Polygon& poly) {
    Point c;
    bg::centroid(poly, c);
    return c;
}

// Circle helpers
/**
 * @brief Creates a circle from center point and radius
 * 
 * @param x X-coordinate of center
 * @param y Y-coordinate of center
 * @param r Radius of the circle
 * @return Circle object
 */
inline Circle makeCircle(float x, float y, float r) {
    return Circle(x, y, r);
}

inline Circle makeCircle(const Point& center, float radius) {
    return Circle(center, radius);
}

inline const Point& center(const Circle& c) {
    return c.center;
}

inline float radius(const Circle& c) {
    return c.radius;
}

inline float area(const Circle& c) {
    return 3.14159265359f * c.radius * c.radius;
}

// ============================================================================
// Collision Detection (using Boost.Geometry algorithms directly)
// ============================================================================

// Point-Point
inline bool collision(const Point& a, const Point& b, float epsilon = 1e-6f) {
    return bg::distance(a, b) < epsilon;
}

// Point-Line (segment)
inline bool collision(const Point& p, const Line& l, float epsilon = 0.01f) {
    return bg::distance(p, l) < epsilon;
}

inline bool collision(const Line& l, const Point& p, float epsilon = 0.01f) {
    return collision(p, l, epsilon);
}

// Point-Rectangle (box)
inline bool collision(const Point& p, const Rectangle& r) {
    return bg::within(p, r);
}

inline bool collision(const Rectangle& r, const Point& p) {
    return collision(p, r);
}

// Point-Polygon
inline bool collision(const Point& p, const Polygon& poly) {
    return bg::within(p, poly);
}

inline bool collision(const Polygon& poly, const Point& p) {
    return collision(p, poly);
}

// Line-Line
inline bool collision(const Line& l1, const Line& l2) {
    return bg::intersects(l1, l2);
}

// Line-Rectangle
inline bool collision(const Line& l, const Rectangle& r) {
    return bg::intersects(l, r);
}

inline bool collision(const Rectangle& r, const Line& l) {
    return collision(l, r);
}

// Line-Polygon
inline bool collision(const Line& l, const Polygon& poly) {
    return bg::intersects(l, poly);
}

inline bool collision(const Polygon& poly, const Line& l) {
    return collision(l, poly);
}

// Rectangle-Rectangle
inline bool collision(const Rectangle& r1, const Rectangle& r2) {
    return bg::intersects(r1, r2);
}

// Rectangle-Polygon
inline bool collision(const Rectangle& r, const Polygon& poly) {
    return bg::intersects(r, poly);
}

inline bool collision(const Polygon& poly, const Rectangle& r) {
    return collision(r, poly);
}

// Polygon-Polygon
inline bool collision(const Polygon& p1, const Polygon& p2) {
    return bg::intersects(p1, p2);
}

// Circle-Point
inline bool collision(const Circle& c, const Point& p) {
    return bg::distance(c.center, p) <= c.radius;
}

inline bool collision(const Point& p, const Circle& c) {
    return collision(c, p);
}

// Circle-Line
inline bool collision(const Circle& c, const Line& l) {
    return bg::distance(c.center, l) <= c.radius;
}

inline bool collision(const Line& l, const Circle& c) {
    return collision(c, l);
}

// Circle-Rectangle
inline bool collision(const Circle& c, const Rectangle& r) {
    // Find closest point on rectangle to circle center
    const auto& min = r.min_corner();
    const auto& max = r.max_corner();
    const float cx = std::clamp(c.center.x(), min.x(), max.x());
    const float cy = std::clamp(c.center.y(), min.y(), max.y());
    
    const float dx = c.center.x() - cx;
    const float dy = c.center.y() - cy;
    return (dx*dx + dy*dy) <= (c.radius * c.radius);
}

inline bool collision(const Rectangle& r, const Circle& c) {
    return collision(c, r);
}

// Circle-Polygon
inline bool collision(const Circle& c, const Polygon& poly) {
    return bg::distance(c.center, poly) <= c.radius;
}

inline bool collision(const Polygon& poly, const Circle& c) {
    return collision(c, poly);
}

// Circle-Circle
inline bool collision(const Circle& c1, const Circle& c2) {
    const float dist = bg::distance(c1.center, c2.center);
    return dist <= (c1.radius + c2.radius);
}

// ============================================================================
// Distance Functions
// ============================================================================
// Distance Functions
// ============================================================================

// Squared distance helpers (avoid sqrt for comparisons - performance optimization)
inline float distanceSquared(const Point& a, const Point& b) {
    float dx = a.x() - b.x();
    float dy = a.y() - b.y();
    return dx*dx + dy*dy;
}

/**
 * @brief Computes squared distance from point to line segment (no sqrt)
 * 
 * Calculates the minimum squared distance from a point to the closest
 * point on a line segment. This is a performance optimization to avoid
 * the expensive sqrt operation when only comparisons are needed.
 * 
 * Algorithm:
 * 1. If point projects before segment start, return distance to start
 * 2. If point projects after segment end, return distance to end  
 * 3. Otherwise, return perpendicular distance to segment
 * 
 * @param p The point
 * @param l The line segment
 * @return Squared distance (always non-negative)
 * 
 * @note For degenerate lines (length ≈ 0), returns distance to start point
 * @note No division by zero: c2 check ensures denominator is non-zero
 */
inline float distanceSquared(const Point& p, const Line& l) {
    // Compute point-to-segment squared distance manually
    const float dx = l.second.x() - l.first.x();
    const float dy = l.second.y() - l.first.y();
    const float wx = p.x() - l.first.x();
    const float wy = p.y() - l.first.y();
    
    const float c1 = wx * dx + wy * dy;
    if (c1 <= 0) {
        return wx*wx + wy*wy;  // Distance to start point
    }
    
    const float c2 = dx*dx + dy*dy;
    if (c1 >= c2) {
        const float ex = p.x() - l.second.x();
        const float ey = p.y() - l.second.y();
        return ex*ex + ey*ey;  // Distance to end point
    }
    
    const float b = c1 / c2;
    const float pbx = l.first.x() + b * dx;
    const float pby = l.first.y() + b * dy;
    const float distx = p.x() - pbx;
    const float disty = p.y() - pby;
    return distx*distx + disty*disty;
}

/**
 * @brief Computes squared distance from point to rectangle (no sqrt)
 * 
 * Uses clamping to find the closest point on or inside the rectangle.
 * If the point is inside the rectangle, the distance is zero.
 * 
 * @param p The point
 * @param r The rectangle
 * @return Squared distance (0 if point is inside rectangle)
 * 
 * @note Clamping ensures closest point is on rectangle boundary
 * @note Always returns non-negative value
 */
inline float distanceSquared(const Point& p, const Rectangle& r) {
    // Clamp point to rectangle bounds
    const auto& min = r.min_corner();
    const auto& max = r.max_corner();
    const float cx = std::clamp(p.x(), min.x(), max.x());
    const float cy = std::clamp(p.y(), min.y(), max.y());
    
    const float dx = p.x() - cx;
    const float dy = p.y() - cy;
    return dx*dx + dy*dy;
}

/**
 * @brief Computes squared distance from point to polygon
 * 
 * Falls back to Boost.Geometry distance calculation and squares the result.
 * This is less efficient than other distanceSquared variants since
 * Boost.Geometry doesn't provide a native squared distance function for polygons.
 * 
 * @param p The point
 * @param poly The polygon
 * @return Squared distance
 * 
 * @warning Slower than other distanceSquared variants (requires sqrt internally)
 * @note Consider using distance() directly if you need the actual distance
 */
inline float distanceSquared(const Point& p, const Polygon& poly) {
    // Boost.Geometry doesn't provide distance_squared, so compute and square it
    const float dist = bg::distance(p, poly);
    return dist * dist;
}

// Public distance API (with sqrt)
inline float distance(const Point& a, const Point& b) {
    return std::sqrt(distanceSquared(a, b));
}

inline float distance(const Point& p, const Line& l) {
    return std::sqrt(distanceSquared(p, l));
}

inline float distance(const Point& p, const Rectangle& r) {
    return std::sqrt(distanceSquared(p, r));
}

inline float distance(const Point& p, const Polygon& poly) {
    return bg::distance(p, poly);
}

// Circle distance functions
inline float distanceSquared(const Point& p, const Circle& c) {
    const float centerDist = bg::distance(p, c.center);
    const float diff = centerDist - c.radius;
    return (diff > 0.0f) ? (diff * diff) : 0.0f;  // 0 if point inside circle
}

inline float distance(const Point& p, const Circle& c) {
    const float centerDist = bg::distance(p, c.center);
    return std::max(0.0f, centerDist - c.radius);  // 0 if point inside circle
}

inline float distance(const Circle& c, const Point& p) {
    return distance(p, c);
}

// ============================================================================
// Bounding Box Computation
// ============================================================================

inline Rectangle boundingBox(const Point& p) {
    return Rectangle(p, p);
}

inline Rectangle boundingBox(const Line& l) {
    float min_x = std::min(l.first.x(), l.second.x());
    float min_y = std::min(l.first.y(), l.second.y());
    float max_x = std::max(l.first.x(), l.second.x());
    float max_y = std::max(l.first.y(), l.second.y());
    return makeRectangle(min_x, min_y, max_x, max_y);
}

inline Rectangle boundingBox(const Rectangle& r) {
    return r;
}

inline Rectangle boundingBox(const Polygon& poly) {
    Rectangle box;
    bg::envelope(poly, box);
    return box;
}

inline Rectangle boundingBox(const Circle& c) {
    return makeRectangle(
        c.center.x() - c.radius, c.center.y() - c.radius,
        c.center.x() + c.radius, c.center.y() + c.radius
    );
}

// ============================================================================
// Contains Predicate
// ============================================================================

inline bool contains(const Point& p1, const Point& p2, float epsilon = 1e-6f) {
    return bg::distance(p1, p2) < epsilon;
}

inline bool contains(const Line& l, const Point& p, float epsilon = 0.01f) {
    return bg::distance(p, l) < epsilon;
}

inline bool contains(const Rectangle& r, const Point& p) {
    return bg::within(p, r);
}

inline bool contains(const Polygon& poly, const Point& p) {
    return bg::within(p, poly);
}

inline bool contains(const Circle& c, const Point& p) {
    return bg::distance(c.center, p) <= c.radius;
}

// ============================================================================
// Message Conversion
// ============================================================================

inline rises_interfaces::msg::Obstacle toObstacle(const Point& p, int64_t id) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    
    // Set position field for round-trip conversion
    obs.position.x = p.x();
    obs.position.y = p.y();
    obs.position.z = 0.0;
    
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = 0.0;
    obs.vertices = {pt};
    
    return obs;
}

inline rises_interfaces::msg::Obstacle toObstacle(const Line& l, int64_t id) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = l.first.x(); p1.y = l.first.y(); p1.z = 0.0;
    p2.x = l.second.x(); p2.y = l.second.y(); p2.z = 0.0;
    obs.vertices = {p1, p2};
    
    return obs;
}

inline rises_interfaces::msg::Obstacle toObstacle(const Rectangle& r, int64_t id) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
    
    const auto& min = r.min_corner();
    const auto& max = r.max_corner();
    const auto ctr = center(r);
    
    // Set position, width, height, orientation for round-trip conversion
    obs.position.x = ctr.x();
    obs.position.y = ctr.y();
    obs.position.z = 0.0;
    obs.width = max.x() - min.x();
    obs.height = max.y() - min.y();
    obs.orientation = 0.0;  // AABB rectangles are always axis-aligned
    
    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = min.x(); p1.y = min.y(); p1.z = 0.0;
    p2.x = max.x(); p2.y = min.y(); p2.z = 0.0;
    p3.x = max.x(); p3.y = max.y(); p3.z = 0.0;
    p4.x = min.x(); p4.y = max.y(); p4.z = 0.0;
    obs.vertices = {p1, p2, p3, p4};
    
    return obs;
}

inline rises_interfaces::msg::Obstacle toObstacle(const Polygon& poly, int64_t id) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    for (const Point& pt : poly.outer()) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = 0.0;
        obs.vertices.push_back(p);
    }
    
    return obs;
}

inline rises_interfaces::msg::Obstacle toObstacle(const Circle& c, int64_t id) {
    rises_interfaces::msg::Obstacle obs;
    obs.id = id;
    obs.type = rises_interfaces::msg::Obstacle::CIRCLE;
    
    // Set position and radius for round-trip conversion
    obs.position.x = c.center.x();
    obs.position.y = c.center.y();
    obs.position.z = 0.0;
    obs.radius = c.radius;
    
    return obs;
}

// ============================================================================
// Conversion FROM Obstacle Messages (for convenience)
// ============================================================================

/**
 * @brief Convert Obstacle message to Point
 * @note Only valid for POINT type obstacles
 */
inline Point fromObstaclePoint(const rises_interfaces::msg::Obstacle& obs) {
    return Point(obs.position.x, obs.position.y);
}

/**
 * @brief Convert Obstacle message to Line
 * @note Only valid for LINE type obstacles with 2 vertices
 */
inline Line fromObstacleLine(const rises_interfaces::msg::Obstacle& obs) {
    if (obs.vertices.size() != 2) {
        throw std::invalid_argument("LINE obstacle must have exactly 2 vertices");
    }
    return makeLine(
        obs.vertices[0].x, obs.vertices[0].y,
        obs.vertices[1].x, obs.vertices[1].y
    );
}

/**
 * @brief Convert Obstacle message to Rectangle
 * @note Only valid for RECTANGLE type obstacles
 */
inline Rectangle fromObstacleRectangle(const rises_interfaces::msg::Obstacle& obs) {
    // Rectangle uses width, height, and orientation
    const float half_w = obs.width * 0.5f;
    const float half_h = obs.height * 0.5f;
    const float cos_theta = std::cos(obs.orientation);
    const float sin_theta = std::sin(obs.orientation);
    
    const Point center_pt(obs.position.x, obs.position.y);
    
    // For axis-aligned (orientation=0)
    if (std::abs(obs.orientation) < 1e-6f) {
        return makeRectangle(
            center_pt.x() - half_w, center_pt.y() - half_h,
            center_pt.x() + half_w, center_pt.y() + half_h
        );
    }
    
    // For rotated rectangles, compute AABB of all 4 corners
    const std::array<Point, 4> corners = {{
        Point(center_pt.x() + cos_theta * (-half_w) - sin_theta * (-half_h),
              center_pt.y() + sin_theta * (-half_w) + cos_theta * (-half_h)),
        Point(center_pt.x() + cos_theta * half_w - sin_theta * (-half_h),
              center_pt.y() + sin_theta * half_w + cos_theta * (-half_h)),
        Point(center_pt.x() + cos_theta * half_w - sin_theta * half_h,
              center_pt.y() + sin_theta * half_w + cos_theta * half_h),
        Point(center_pt.x() + cos_theta * (-half_w) - sin_theta * half_h,
              center_pt.y() + sin_theta * (-half_w) + cos_theta * half_h)
    }};
    
    float min_x = corners[0].x(), max_x = corners[0].x();
    float min_y = corners[0].y(), max_y = corners[0].y();
    for (int i = 1; i < 4; ++i) {
        min_x = std::min(min_x, corners[i].x());
        max_x = std::max(max_x, corners[i].x());
        min_y = std::min(min_y, corners[i].y());
        max_y = std::max(max_y, corners[i].y());
    }
    
    return makeRectangle(min_x, min_y, max_x, max_y);
}

/**
 * @brief Convert Obstacle message to Polygon
 * @note Valid for POLYGON, CONVEX_POLYGON, FREEFORM types
 */
inline Polygon fromObstaclePolygon(const rises_interfaces::msg::Obstacle& obs) {
    // Validate: polygon must have at least 3 vertices
    if (obs.vertices.size() < 3) {
        throw std::invalid_argument("Polygon must have at least 3 vertices, got " + 
                                  std::to_string(obs.vertices.size()));
    }
    
    Polygon result;
    Polygon::ring_type& outer = result.outer();
    outer.reserve(obs.vertices.size());
    for (const geometry_msgs::msg::Point& v : obs.vertices) {
        bg::append(outer, Point(v.x, v.y));
    }
    return result;
}

/**
 * @brief Convert Obstacle message to Circle
 * @note Only valid for CIRCLE type obstacles
 */
inline Circle fromObstacleCircle(const rises_interfaces::msg::Obstacle& obs) {
    return Circle(obs.position.x, obs.position.y, obs.radius);
}

} // namespace rises::geofence
