#pragma once

#include "geofence/spatial/map/geofence_map.hpp"
#include "spatial_index_selection.hpp"

namespace rises::geofence {

/**
 * @brief Shared boundary checking policy
 * 
 * Provides common map boundary validation logic that is used by multiple
 * checker classes (PathSafetyChecker, ObstacleMatchChecker, etc.).
 * 
 * This is policy logic that uses data from the map. The map provides the
 * contours data, and this class implements the business logic of deciding
 * whether points/shapes are within valid bounds.
 * 
 * Design Rationale:
 * - Extracted from individual checkers to avoid duplication
 * - Policy layer: implements business logic using map data
 * - Map provides data access only (getMapContours())
 * - Stateless: can be used by multiple checkers concurrently
 */
class BoundaryChecker {
public:
    /**
     * @brief Check if a point is inside map boundary contours
     * 
     * @param map The geofence map containing boundary contours
     * @param point Point to test
     * @return true if point is inside map boundary
     * 
     * @note Returns true if no boundary contours are defined (unbounded map)
     * @note Thread-safe: reads immutable map data
     */
    [[nodiscard]] static bool isPointInsideMapBounds(
        const GeofenceMap& map,
        const Point2D& point);
    
    /**
     * @brief Check if both endpoints of a segment are inside map bounds
     * 
     * @param map The geofence map containing boundary contours
     * @param start Start point of segment
     * @param end End point of segment
     * @return true if both points are inside map boundary
     * 
     * @note Returns true if no boundary contours are defined (unbounded map)
     * @note Thread-safe: reads immutable map data
     */
    [[nodiscard]] static bool isSegmentInsideMapBounds(
        const GeofenceMap& map,
        const Point2D& start,
        const Point2D& end);
};

} // namespace rises::geofence
