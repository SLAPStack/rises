#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "geometry_types.hpp"

namespace rises {
namespace geofence {

// ============================================================================
// Common BoundingBox Type - Shared Between Geometry and SpatialIndex
// ============================================================================

/**
 * @brief Axis-aligned bounding box in 2D space
 * 
 * Design rationale:
 * - Defined in common header (neither geometry nor spatial index owns it)
 * - Simple POD type for optimal performance
 * - Used by both geometry (getBoundingBox) and spatial index (queries)
 * - Avoids type conversions and coupling
 */
struct BoundingBox {
    float min_x;
    float min_y;
    float max_x;
    float max_y;
    
    // Default constructor for uninitialized box
    BoundingBox() = default;
    
    // Parameterized constructor
    BoundingBox(float min_x_, float min_y_, float max_x_, float max_y_)
        : min_x(min_x_), min_y(min_y_), max_x(max_x_), max_y(max_y_) {}
    
    // Construct from center and half-extents
    [[nodiscard]] static BoundingBox fromCenterAndSize(float cx, float cy, float half_width, float half_height) {
        return BoundingBox{
            cx - half_width, cy - half_height,
            cx + half_width, cy + half_height
        };
    }
    
    // Construct from single point (zero-area box)
    [[nodiscard]] static BoundingBox fromPoint(float x, float y) {
        return BoundingBox{x, y, x, y};
    }
    
    // Check if box is valid (min <= max)
    [[nodiscard]] bool isValid() const {
        return this->min_x <= max_x && min_y <= max_y;
    }
    
    // Check if box is degenerate (zero area)
    [[nodiscard]] bool isDegenerate() const {
        return this->min_x == max_x || min_y == max_y;
    }
    
    // Get center point
    void getCenter(float& cx, float& cy) const {
        cx = (min_x + max_x) * 0.5f;
        cy = (min_y + max_y) * 0.5f;
    }
    
    // Get dimensions
    [[nodiscard]] float width() const { return this->max_x - min_x; }
    [[nodiscard]] float height() const { return this->max_y - min_y; }
    [[nodiscard]] float area() const { return width() * height(); }
    
    // Expand box to include point
    void expandToInclude(float x, float y) {
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
    }
    
    // Expand box to include another box
    void expandToInclude(const BoundingBox& other) {
        min_x = std::min(min_x, other.min_x);
        min_y = std::min(min_y, other.min_y);
        max_x = std::max(max_x, other.max_x);
        max_y = std::max(max_y, other.max_y);
    }
    
    // Check intersection with another box
    [[nodiscard]] bool intersects(const BoundingBox& other) const {
        return !(max_x < other.min_x || other.max_x < min_x ||
                 max_y < other.min_y || other.max_y < min_y);
    }
    
    // Check if contains point
    [[nodiscard]] bool contains(float x, float y) const {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    }
    
    // Check if fully contains another box
    [[nodiscard]] bool contains(const BoundingBox& other) const {
        return this->min_x <= other.min_x && max_x >= other.max_x &&
               min_y <= other.min_y && max_y >= other.max_y;
    }
    
    // Grow box by margin on all sides
    [[nodiscard]] BoundingBox expanded(float margin) const {
        return BoundingBox{
            min_x - margin, min_y - margin,
            max_x + margin, max_y + margin
        };
    }
};

// ============================================================================
// Rationale for Common BoundingBox Type
// ============================================================================

/* ALTERNATIVE APPROACHES CONSIDERED:
   
   1. Geometry owns BoundingBox, SpatialIndex converts
   --------------------------------------------------
   struct BoundingBox { ... };  // In variant_shapes.hpp
   
   // Geometry defines it
   BoundingBox getBoundingBox(const Geometry& geom);
   
   // SpatialIndex must convert
   class BoostRTreeAdapter {
       void insert(int64_t id, const Rectangle& rect) {
           // Convert Rectangle (geometry type) to Boost::Box (spatial index type)
           Box box = convertToBoostBox(rect);
       }
   };
   
   Issues:
   ✗ Tight coupling: SpatialIndex depends on geometry types
   ✗ Type conversions on every insert/query
   ✗ Rectangle is both a shape AND a bounding box (confusing)
   
   
   2. SpatialIndex owns BoundingBox, Geometry converts
   ---------------------------------------------------
   // SpatialIndex defines it
   class SpatialIndexInterface {
       struct BoundingBox { ... };
       virtual void insert(int64_t id, const BoundingBox& bbox) = 0;
   };
   
   // Geometry must convert
   BoundingBox getBoundingBox(const Geometry& geom) {
       // Convert to SpatialIndex::BoundingBox
   }
   
   Issues:
   ✗ Geometry depends on spatial index
   ✗ Cannot use geometry without spatial index
   ✗ Wrong dependency direction
   
   
   3. Each backend defines its own type (CURRENT OLD DESIGN)
   ---------------------------------------------------------
   class BoostRTreeAdapter {
       using Box = bg::model::box<Point>;
       void insert(int64_t id, const Rectangle& rect) {
           Box box(Point(rect.min_x, rect.min_y), ...);  // Convert
       }
   };
   
   class NanoflannAdapter {
       void insert(int64_t id, const Rectangle& rect) {
           float center[2] = { (rect.min_x + rect.max_x) * 0.5f, ... };  // Convert
       }
   };
   
   Issues:
   ✗ Each adapter does its own conversion
   ✗ Duplicated conversion logic
   ✗ No common representation
   
   
   4. Common BoundingBox type (RECOMMENDED)
   -----------------------------------------
   // Common header (e.g., geofence/common/types.hpp)
   struct BoundingBox { float min_x, min_y, max_x, max_y; };
   
   // Geometry produces it
   BoundingBox getBoundingBox(const Geometry& geom) {
       return std::visit([](const auto& shape) {
           return computeBoundingBox(shape);
       }, geom);
   }
   
   // SpatialIndex consumes it
   class SpatialIndexInterface {
       virtual void insert(int64_t id, const BoundingBox& bbox) = 0;
   };
   
   // Adapters convert to backend-specific type
   class BoostRTreeAdapter {
       void insert(int64_t id, const BoundingBox& bbox) override {
           Box box(Point(bbox.min_x, bbox.min_y), Point(bbox.max_x, bbox.max_y));
           rtree_.insert({box, id});
       }
   };
   
   Advantages:
   ✓ Clean separation: geometry produces, spatial index consumes
   ✓ Single conversion point: in adapter, not at every call site
   ✓ Backend-agnostic: common type doesn't depend on Boost/nanoflann/etc
   ✓ Testing: can test getBoundingBox independently
   ✓ Performance: conversion happens once in adapter (can be inlined)
*/


// ============================================================================
// DESIGN DECISION: Common BoundingBox
// ============================================================================

/*
   RATIONALE:
   ----------
   
   1. SEPARATION OF CONCERNS
      - Geometry: Produces bounding boxes
      - SpatialIndex: Consumes bounding boxes
      - Neither owns the concept
   
   2. MINIMAL COUPLING
      - Geometry doesn't know about spatial indexing
      - SpatialIndex doesn't know about geometry shapes
      - Both depend on simple POD struct
   
   3. SINGLE CONVERSION POINT
      - Geometry → BoundingBox: Once, in getBoundingBox()
      - BoundingBox → Backend type: Once, in adapter
      - No conversions at call sites
   
   4. TYPE SAFETY
      - BoundingBox is distinct from Rectangle
      - Rectangle = geometric shape with collision detection
      - BoundingBox = axis-aligned query region
      - Clear semantic difference
   
   5. PERFORMANCE
      - BoundingBox is POD (passed by value efficiently)
      - Conversions are inlined in adapters
      - No heap allocations
      - Compiler can optimize away temporary objects
   
   
   USAGE PATTERN:
   --------------
   
   // 1. Create geometry
   Geometry obstacle = Rectangle{0.0f, 0.0f, 2.0f, 1.0f};
   
   // 2. Extract bounding box (happens in GeofenceMap)
   BoundingBox bbox = getBoundingBox(obstacle);
   
   // 3. Insert into spatial index (adapter converts to backend type)
   spatialIndex.insert(id, bbox);
   //   ↓
   // Adapter translates:
   //   Boost: Box(Point(bbox.min_x, ...), Point(bbox.max_x, ...))
   //   Nanoflann: float center[2] = {(bbox.min_x + bbox.max_x) * 0.5f, ...}
   //   libspatialindex: double low[2] = {bbox.min_x, ...}, Region(low, high)
   
   
   COMPILE-TIME COST:
   ------------------
   With optimization, the BoundingBox is completely optimized away:
   
   BoundingBox bbox = getBoundingBox(rect);
   index.insert(id, bbox);
   
   Becomes:
   rtree_.insert({Box(Point(rect.min_x, rect.min_y), Point(rect.max_x, rect.max_y)), id});
   
   Zero runtime overhead - just a type safety boundary!
*/

// ============================================================================
// Validated BoundingBox Creation Utilities
// ============================================================================

/**
 * @brief Create a bounding box from two points with validation
 * 
 * Ensures the resulting box is valid (no NaN, no infinity, min <= max)
 * 
 * @param x1 First x coordinate
 * @param y1 First y coordinate
 * @param x2 Second x coordinate
 * @param y2 Second y coordinate
 * @param margin Optional margin to expand box by (default 0)
 * @return Valid bounding box
 * @throws std::invalid_argument if any coordinate is NaN or infinite
 */
inline BoundingBox createValidatedBBox(float x1, float y1, float x2, float y2, float margin = 0.0f) {
    // Check for invalid float values
    if (!std::isfinite(x1) || !std::isfinite(y1) || 
        !std::isfinite(x2) || !std::isfinite(y2) || !std::isfinite(margin)) {
        throw std::invalid_argument("BoundingBox coordinates must be finite");
    }
    
    // Compute bbox with margin
    float min_x = std::min(x1, x2) - margin;
    float min_y = std::min(y1, y2) - margin;
    float max_x = std::max(x1, x2) + margin;
    float max_y = std::max(y1, y2) + margin;
    
    // Handle degenerate case (point or line segment)
    // Add small epsilon to ensure min < max for spatial index compatibility
    constexpr float epsilon = std::numeric_limits<float>::epsilon() * 10.0f;
    if (min_x >= max_x) {
        max_x = min_x + epsilon;
    }
    if (min_y >= max_y) {
        max_y = min_y + epsilon;
    }
    
    return BoundingBox{min_x, min_y, max_x, max_y};
}

/**
 * @brief Create a bounding box from two Point2D with validation
 * 
 * @param p1 First point
 * @param p2 Second point
 * @param margin Optional margin to expand box by (default 0)
 * @return Valid bounding box
 * @throws std::invalid_argument if any coordinate is NaN or infinite
 */
inline BoundingBox createValidatedBBox(const Point2D& p1, const Point2D& p2, float margin = 0.0f) {
    return createValidatedBBox(
        static_cast<float>(p1.x), 
        static_cast<float>(p1.y), 
        static_cast<float>(p2.x), 
        static_cast<float>(p2.y), 
        margin
    );
}

} // namespace geofence
} // namespace rises
