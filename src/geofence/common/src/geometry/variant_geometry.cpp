/**
 * @file variant_geometry.cpp
 * @brief Geometry operations via std::variant and visitors
 * 
 * This file implements the Visitor pattern for type-safe, zero-cost polymorphic
 * operations on geometry variants. All operations are resolved at compile-time
 * via template instantiation.
 * 
 * Key Design Patterns:
 * - Visitor Pattern: Type-safe double dispatch without virtual functions
 * - Compile-time polymorphism: Zero overhead via template instantiation
 * - Generic programming: Uses Boost.Geometry types for spatial operations
 * 
 * Visitor Categories:
 * - Type identification: getType()
 * - Spatial queries: getPosition(), getBoundingBox(), getOrientation()
 * - Collision detection: intersects(), contains()
 * - Distance computation: distance(), distanceToPoint()
 * - Message conversion: toObstacleMsg(), fromObstacleMsg()
 * 
 * @note All visitors are stateless functors for optimal inlining
 */

#include "geofence/common/geometry/variant_geometry.hpp"

namespace rises::geofence {

// ============================================================================
// Visitor Implementations (type-safe dispatch)
// ============================================================================

/**
 * @brief Visitor that identifies the concrete type in a geometry variant
 * 
 * Returns the enum tag corresponding to the active variant alternative.
 * Compiled to direct vtable-free dispatch via template instantiation.
 */
struct TypeVisitor {
    GeometryType operator()(const Point&) const { return GeometryType::Point; }
    GeometryType operator()(const Line&) const { return GeometryType::Line; }
    GeometryType operator()(const Rectangle&) const { return GeometryType::Rectangle; }
    GeometryType operator()(const Polygon&) const { return GeometryType::Polygon; }
    GeometryType operator()(const Circle&) const { return GeometryType::Circle; }
};

/**
 * @brief Returns the type of geometry stored in the variant
 * 
 * @param g Geometry variant to query
 * @return Enum identifying the concrete geometry type
 */
GeometryType getType(const Geometry& g) {
    return std::visit(TypeVisitor{}, g);
}

// ============================================================================
// Position/Bounding Box Visitors
// ============================================================================

/**
 * @brief Visitor that computes the representative position of a geometry
 * 
 * Returns:
 * - Point: the point itself
 * - Line: midpoint
 * - Rectangle: center
 * - Polygon: centroid
 */
struct PositionVisitor {
    Point2D operator()(const Point& p) const {
        return {p.x(), p.y()};
    }
    
    Point2D operator()(const Line& l) const {
        return {(l.first.x() + l.second.x()) * 0.5f, (l.first.y() + l.second.y()) * 0.5f};
    }
    
    Point2D operator()(const Rectangle& r) const {
        const Point c = center(r);
        return {c.x(), c.y()};
    }
    
    Point2D operator()(const Polygon& poly) const {
        const Point c = centroid(poly);
        return {c.x(), c.y()};
    }
    
    Point2D operator()(const Circle& c) const {
        return {c.center.x(), c.center.y()};
    }
};

/**
 * @brief Returns the representative position of a geometry
 * 
 * @param g Geometry variant to query
 * @return Position as backend-independent Point2D
 */
Point2D getPosition(const Geometry& g) {
    return std::visit(PositionVisitor{}, g);
}

/**
 * @brief Visitor that computes the axis-aligned bounding box of a geometry
 * 
 * Returns the tightest AABB containing the geometry. Used for spatial
 * indexing and conservative collision detection.
 */
struct BoundingBoxVisitor {
    BoundingBox operator()(const rises::geofence::Point& p) const {
        return {p.x(), p.y(), p.x(), p.y()};
    }
    
    BoundingBox operator()(const rises::geofence::Line& l) const {
        const float min_x = std::min(l.first.x(), l.second.x());
        const float min_y = std::min(l.first.y(), l.second.y());
        const float max_x = std::max(l.first.x(), l.second.x());
        const float max_y = std::max(l.first.y(), l.second.y());
        return {min_x, min_y, max_x, max_y};
    }
    
    BoundingBox operator()(const rises::geofence::Rectangle& r) const {
        const rises::geofence::Point& min = r.min_corner();
        const rises::geofence::Point& max = r.max_corner();
        return {min.x(), min.y(), max.x(), max.y()};
    }
    
    BoundingBox operator()(const rises::geofence::Polygon& poly) const {
        const rises::geofence::Polygon::ring_type& verts = poly.outer();
        if (verts.empty()) return {0, 0, 0, 0};
        
        float min_x = verts[0].x();
        float min_y = verts[0].y();
        float max_x = min_x;
        float max_y = min_y;
        
        for (const rises::geofence::Point& v : verts) {
            min_x = std::min(min_x, v.x());
            min_y = std::min(min_y, v.y());
            max_x = std::max(max_x, v.x());
            max_y = std::max(max_y, v.y());
        }
        
        return {min_x, min_y, max_x, max_y};
    }
    
    BoundingBox operator()(const rises::geofence::Circle& c) const {
        return {c.center.x() - c.radius, c.center.y() - c.radius,
                c.center.x() + c.radius, c.center.y() + c.radius};
    }
};

/**
 * @brief Returns the axis-aligned bounding box of a geometry
 * 
 * @param g Geometry variant to query
 * @return Bounding box as common BoundingBox struct
 */
BoundingBox getBoundingBox(const Geometry& g) {
    return std::visit(BoundingBoxVisitor{}, g);
}

/**
 * @brief Visitor that computes the orientation angle of a geometry
 * 
 * Returns:
 * - Point: 0 (no orientation)
 * - Line: angle from start to end in radians
 * - Rectangle: 0 (axis-aligned)
 * - Polygon: 0 (orientation not well-defined for general polygons)
 */
struct OrientationVisitor {
    float operator()(const Point&) const { return 0.0f; }
    
    float operator()(const Line& l) const {
        const Point dir = direction(l);
        return std::atan2(dir.y(), dir.x());
    }
    
    float operator()(const Rectangle&) const { return 0.0f; }
    
    float operator()(const Polygon&) const { return 0.0f; }
    
    float operator()(const Circle&) const { return 0.0f; }
};

/**
 * @brief Returns the orientation angle of a geometry in radians
 * 
 * @param g Geometry variant to query
 * @return Orientation angle in radians, 0 if not applicable
 */
float getOrientation(const Geometry& g) {
    return std::visit(OrientationVisitor{}, g);
}

// ============================================================================
// Collision Detection (double dispatch via std::visit)
// ============================================================================

/**
 * @brief Visitor that performs collision detection between two geometries
 * 
 * Uses ADL (Argument Dependent Lookup) to find the correct collision
 * function from the rises::geofence namespace for all geometry type combinations.
 */
struct CollisionVisitor {
    template<typename T, typename U>
    bool operator()(const T& a, const U& b) const {
        return collision(a, b);  // ADL finds correct overload
    }
};

/**
 * @brief Tests if two geometries intersect
 * 
 * @param a First geometry variant
 * @param b Second geometry variant
 * @return true if geometries intersect or overlap
 * 
 * @note Uses backend-specific collision detection (Eigen/Boost/CUDA)
 * @note Exact collision test, not bounding box approximation
 */
bool intersects(const Geometry& a, const Geometry& b) {
    return std::visit(CollisionVisitor{}, a, b);
}

/**
 * @brief Visitor that tests if a geometry contains a point
 * 
 * Converts the backend-independent Point2D to a backend-specific Point
 * and delegates to the backend's collision function.
 */
struct ContainsVisitor {
    Point2D point;
    
    template<typename T>
    bool operator()(const T& shape) const {
        const Point p(point.x, point.y);
        return collision(p, shape);
    }
};

/**
 * @brief Tests if a geometry contains a point
 * 
 * @param container Geometry variant to test
 * @param point Point to test for containment
 * @return true if point is inside or on the boundary
 * 
 * @note For lines, returns true if point is on the line
 * @note For polygons, uses ray-casting or winding number algorithm
 */
bool contains(const Geometry& container, const Point2D& point) {
    return std::visit(ContainsVisitor{point}, container);
}

// ============================================================================
// Distance Queries
// ============================================================================

/**
 * @brief Visitor that computes distance between two geometries
 * 
 * Uses ADL to find the correct backend-specific distance function.
 * Returns minimum Euclidean distance between the geometries.
 */
struct DistanceVisitor {
    template<typename T, typename U>
    float operator()(const T& a, const U& b) const {
        return distance(a, b);
    }
};

/**
 * @brief Computes minimum distance between two geometries
 * 
 * @param a First geometry variant
 * @param b Second geometry variant
 * @return Minimum Euclidean distance in meters
 * 
 * @note Returns 0 if geometries intersect
 * @note Uses backend-specific distance algorithms (Eigen/Boost/CUDA)
 */
float distance(const Geometry& a, const Geometry& b) {
    return std::visit(DistanceVisitor{}, a, b);
}

/**
 * @brief Visitor that computes distance from a geometry to a point
 * 
 * Converts the backend-independent Point2D to a backend-specific Point
 * and delegates to the backend's distance function.
 */
struct DistanceToPointVisitor {
    Point2D point;
    
    template<typename T>
    float operator()(const T& shape) const {
        const Point p(point.x, point.y);
        return distance(p, shape);
    }
};

/**
 * @brief Computes minimum distance from a geometry to a point
 * 
 * @param g Geometry variant
 * @param point Point to measure distance to
 * @return Minimum Euclidean distance in meters
 * 
 * @note Returns 0 if point is inside or on the boundary
 */
float distanceToPoint(const Geometry& g, const Point2D& point) {
    return std::visit(DistanceToPointVisitor{point}, g);
}

/**
 * @brief Visitor that computes SQUARED distance from a geometry to a point
 * 
 * Used for optimization when comparing multiple distances - avoids sqrt in loop.
 * Converts the backend-independent Point2D to a backend-specific Point
 * and delegates to the backend's distanceSquared function.
 */
struct DistanceSquaredToPointVisitor {
    Point2D point;
    
    template<typename T>
    float operator()(const T& shape) const {
        const Point p(point.x, point.y);
        return distanceSquared(p, shape);
    }
};

/**
 * @brief Computes minimum SQUARED distance from a geometry to a point
 * 
 * Performance optimization: Use this when comparing distances to avoid sqrt.
 * Example: finding minimum distance to multiple geometries.
 * 
 * @param g Geometry variant
 * @param point Point to measure distance to
 * @return Minimum Euclidean distance SQUARED in meters²
 * 
 * @note Returns 0 if point is inside or on the boundary
 * @note To get actual distance, use std::sqrt() on the result
 */
float distanceSquaredToPoint(const Geometry& g, const Point2D& point) {
    return std::visit(DistanceSquaredToPointVisitor{point}, g);
}

// ============================================================================
// Message Conversion
// ============================================================================

/**
 * @brief Visitor that converts a geometry to ROS2 obstacle message
 * 
 * Each backend provides toObstacle() functions that serialize geometry
 * to the rises_interfaces::msg::Obstacle format.
 */
struct ToObstacleVisitor {
    int64_t id;
    
    template<typename T>
    rises_interfaces::msg::Obstacle operator()(const T& shape) const {
        return toObstacle(shape, id);
    }
};

/**
 * @brief Converts a geometry variant to ROS2 obstacle message
 * 
 * @param g Geometry variant to convert
 * @param id Obstacle identifier to include in message
 * @return ROS2 obstacle message ready for publishing
 * 
 * @note Backend-specific serialization preserves geometry type information
 */
rises_interfaces::msg::Obstacle toObstacleMsg(const Geometry& g, const int64_t id) {
    return std::visit(ToObstacleVisitor{id}, g);
}

/**
 * @brief Converts ROS2 obstacle message to geometry variant
 * 
 * Deserializes obstacle messages from ROS2 topics into the appropriate
 * backend-specific geometry type. Handles multiple representation formats
 * for rectangles (vertices vs width/height).
 * 
 * @param msg ROS2 obstacle message to convert
 * @return Geometry variant containing the deserialized shape
 * 
 * @note CUDA backend uses fromPoints() for GPU allocation
 * @note Eigen/Boost backends use addVertex() for incremental construction
 * @note Falls back to Point(0,0) for invalid/unknown types
 */
Geometry fromObstacleMsg(const rises_interfaces::msg::Obstacle& msg) {
    switch (msg.type) {
        case rises_interfaces::msg::Obstacle::POINT: {
            if (!msg.vertices.empty()) {
                return Point(msg.vertices[0].x, msg.vertices[0].y);
            }
            return Point();
        }
        
        case rises_interfaces::msg::Obstacle::LINE: {
            if (msg.vertices.size() >= 2) {
                return makeLine(
                    static_cast<float>(msg.vertices[0].x), static_cast<float>(msg.vertices[0].y),
                    static_cast<float>(msg.vertices[1].x), static_cast<float>(msg.vertices[1].y)
                );
            }
            return Line();
        }
        
        case rises_interfaces::msg::Obstacle::RECTANGLE: {
            // Try vertices-based representation first (2+ corner vertices)
            if (msg.vertices.size() >= 2) {
                float min_x = std::min(msg.vertices[0].x, msg.vertices[1].x);
                float min_y = std::min(msg.vertices[0].y, msg.vertices[1].y);
                float max_x = std::max(msg.vertices[0].x, msg.vertices[1].x);
                float max_y = std::max(msg.vertices[0].y, msg.vertices[1].y);
                
                // Consider all vertices if there are more than 2
                for (size_t i = 2; i < msg.vertices.size(); ++i) {
                    min_x = std::min(min_x, static_cast<float>(msg.vertices[i].x));
                    max_x = std::max(max_x, static_cast<float>(msg.vertices[i].x));
                    min_y = std::min(min_y, static_cast<float>(msg.vertices[i].y));
                    max_y = std::max(max_y, static_cast<float>(msg.vertices[i].y));
                }
                
                return makeRectangle(min_x, min_y, max_x, max_y);
            }
            // Fall back to position/width/height representation
            else if (msg.width > 0 && msg.height > 0) {
                return fromObstacleRectangle(msg);
            }
            return Rectangle();
        }
        
        case rises_interfaces::msg::Obstacle::POLYGON: {
            std::vector<Point> points;
            points.reserve(msg.vertices.size());
            for (const geometry_msgs::msg::Point& v : msg.vertices) {
                points.emplace_back(static_cast<float>(v.x), static_cast<float>(v.y));
            }
            return makePolygon(points);
        }
        
        case rises_interfaces::msg::Obstacle::CIRCLE: {
            return makeCircle(
                static_cast<float>(msg.position.x), 
                static_cast<float>(msg.position.y), 
                static_cast<float>(msg.radius)
            );
        }
        
        default:
            return Point();  // Fallback for invalid/unknown types
    }
}

} // namespace rises::geofence
