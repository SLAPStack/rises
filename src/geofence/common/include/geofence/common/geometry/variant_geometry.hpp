#pragma once

#include <variant>
#include <vector>
#include <cstdint>
#include "rises_interfaces/msg/obstacle.hpp"
#include "geometry_types.hpp"
#include "geofence/spatial/common/bounding_box.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include "variant_shapes.hpp"

namespace rises::geofence {

// ============================================================================
// Geometry Variant (Zero-cost polymorphism)
// ============================================================================

// All shapes in one variant - no virtual functions!
// Note: Types must be complete (not forward declared) for std::variant
// Uses Boost.Geometry types (defined in variant_shapes.hpp)
using Geometry = std::variant<
    Point,
    Line,
    Rectangle,
    Polygon,
    Circle
>;

// ============================================================================
// Primitive Query APIs (what Map exposes)
// ============================================================================

// Type queries
enum class GeometryType : uint8_t { Point, Line, Rectangle, Polygon, Circle };
GeometryType getType(const Geometry& g);

// Spatial queries
Point2D getPosition(const Geometry& g);
BoundingBox getBoundingBox(const Geometry& g);
float getOrientation(const Geometry& g);

// Collision detection (fully inlined, zero virtual overhead)
bool intersects(const Geometry& a, const Geometry& b);
bool contains(const Geometry& container, const Point2D& point);

// Distance queries
float distance(const Geometry& a, const Geometry& b);
float distanceToPoint(const Geometry& g, const Point2D& point);
float distanceSquaredToPoint(const Geometry& g, const Point2D& point);  // For optimized loops

// Conversion
rises_interfaces::msg::Obstacle toObstacleMsg(const Geometry& g, int64_t id);
Geometry fromObstacleMsg(const rises_interfaces::msg::Obstacle& msg);

// ============================================================================
// Visitor Helpers (for custom operations)
// ============================================================================

// Zero-overhead visitor - compiler optimizes to jump table
template<typename Visitor>
inline auto visit(Visitor&& vis, const Geometry& g) {
    return std::visit(std::forward<Visitor>(vis), g);
}

template<typename Visitor>
inline auto visit(Visitor&& vis, const Geometry& g1, const Geometry& g2) {
    return std::visit(std::forward<Visitor>(vis), g1, g2);
}

// ============================================================================
// Storage & Iteration (zero-cost abstractions with caching)
// ============================================================================

/**
 * @brief Geometry entry with cached spatial properties
 * 
 * Caches commonly-accessed properties to avoid repeated computation:
 * - Bounding box: Used for spatial index queries
 * - Area: Used for collision priority/filtering
 * - Center: Used for distance checks and spatial hashing
 * 
 * Memory overhead: ~32 bytes per geometry (worth it for >1 access)
 */
struct GeometryEntry {
    int64_t id;
    Geometry geometry;
    ::Rectangle bbox;
    float cached_area;
    ::Point2D cached_center;
    ObstacleLayer layer;
    
    GeometryEntry(int64_t id_, Geometry geom_, ObstacleLayer layer_ = ObstacleLayer::STATIC)
        : id(id_)
        , geometry(std::move(geom_))
        , bbox(convertToRectangle(getBoundingBox(this->geometry)))
        , cached_area(computeArea(this->bbox))
        , cached_center(computeCenter(this->bbox))
        , layer(layer_) {}

private:
    static ::Rectangle convertToRectangle(const rises::geofence::BoundingBox& box) {
        return ::Rectangle(
            ::Point2D(static_cast<double>(box.min_x), static_cast<double>(box.min_y)),
            ::Point2D(static_cast<double>(box.max_x), static_cast<double>(box.max_y))
        );
    }
    
    static float computeArea(const ::Rectangle& bbox) {
        const double width = bbox.max.x - bbox.min.x;
        const double height = bbox.max.y - bbox.min.y;
        return static_cast<float>(width * height);
    }
    
    static ::Point2D computeCenter(const ::Rectangle& bbox) {
        return ::Point2D(
            (bbox.min.x + bbox.max.x) * 0.5,
            (bbox.min.y + bbox.max.y) * 0.5
        );
    }
};

} // namespace rises::geofence
