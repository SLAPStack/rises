#pragma once
#include <cstdint>

/**
 * @brief Shared geometry value types (axis-aligned point and bounding box).
 *
 * These are plain value types used across the geometry, map, and query layers.
 * They live in the global namespace and carry no spatial-index dependency, so
 * any layer can include them without pulling in a backend (e.g. nanoflann).
 *
 * This Rectangle is distinct from geometry-backend Rectangle shapes:
 * - Purpose: spatial-index queries and bounding boxes.
 * - Always axis-aligned, no rotation.
 *
 * Geometry-backend rectangles (variant_shapes.hpp) may carry rotation or
 * backend-specific data.
 */
struct Point2D {
    double x{};
    double y{};
    Point2D() = default;
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

struct Rectangle {
    Point2D min;
    Point2D max;
    Rectangle() = default;
    Rectangle(Point2D _min, Point2D _max) : min(_min), max(_max) {}
};
