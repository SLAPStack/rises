# Geometry Backend

Boost.Geometry-based geometry implementation for the geofence system.

## Namespace

All geometry types live directly in `rises::geofence`:

```cpp
rises::geofence::Point
rises::geofence::Line
rises::geofence::Rectangle
rises::geofence::Polygon
rises::geofence::Circle
```

Factory functions: `makeLine()`, `makeRectangle()`, `makePolygon()`, `makeCircle()`

## Design

This is NOT an abstraction layer - we are committed to Boost.Geometry as the geometry backend.
The types use `std::variant` for zero-cost polymorphism (compiles to jump table, no vtables).

## Key Files

- `core/variant_shapes.hpp` - All geometry types, collision, distance, bounding box, ROS conversion
- `core/variant_geometry.hpp` - Variant typedef and visitor utilities
