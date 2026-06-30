#pragma once

#include "geofence/common/geometry/variant_geometry.hpp"
#include "geofence/spatial/common/bounding_box.hpp"
#include <vector>
#include <variant>
#include <string>

namespace rises::geofence {

/**
 * @brief Robot safety profile defining detection and exclusion zones
 * 
 * Encapsulates all robot-geometry-related safety parameters using the
 * existing Geometry variant (Circle, Rectangle, Polygon, Point, Line):
 * - Outer zone: Detection area where obstacles are checked
 * - Inner zone: Exclusion area where obstacles are ignored (optional)
 * - Physical dimensions: Robot body size for visualization/debugging
 * 
 * Design patterns:
 * - Outer circle + inner rectangle: Ignore pallet in fork pickup area
 * - Outer rectangle + inner circle: Rectangular AGV with circular exclusion
 * - Outer only: Detect everything within outer boundary
 * - Inner only: Detect everything EXCEPT inner zone (inverse exclusion)
 * 
 * Thread-safe: All methods are const and read-only after construction.
 * Intended usage: Create once during configuration, reuse for all queries.
 * 
 * Example:
 * @code
 * // Create profile with 3m outer circle, 0.5m inner rectangle (fork area)
 * RobotSafetyProfile profile;
 * profile.setOuterZone(rises::geofence::makeCircle(0.0f, 0.0f, 3.0f));
 * profile.setInnerZone(rises::geofence::makeRectangle(-0.4f, -0.2f, 0.4f, 0.2f));
 * 
 * // Or: Inner-only mode (process everything except loading area)
 * profile.setInnerZone(rises::geofence::makeRectangle(-1.0f, -0.5f, 1.0f, 0.5f));
 * // Now processes all points except those in the rectangle
 * 
 * // Check if point is in detection zone
 * Point2D robot_pos(10.0f, 5.0f);
 * Point2D scan_point(11.5f, 6.0f);
 * if (profile.isInDetectionZone(scan_point, robot_pos, robot_heading)) {
 *     // Process this point
 * }
 * @endcode
 */
class RobotSafetyProfile {
public:
    RobotSafetyProfile();
    
    /**
     * @brief Set outer safety zone (detection area)
     * 
     * Points within this zone are candidates for obstacle matching.
     * Typically larger than inner zone (e.g., 2-5m for warehouse AGVs).
     * If no outer zone is set, all points are considered (infinite range).
     * 
     * @param zone Circle, rectangle, polygon, or other geometry defining outer boundary
     */
    void setOuterZone(const Geometry& zone);
    
    /**
     * @brief Set inner safety zone (exclusion area)
     * 
     * Points within this zone are ignored during obstacle matching.
     * Can be used standalone (inner-only mode) to exclude a region while
     * processing everything else, or combined with outer zone for donut pattern.
     * 
     * @param zone Circle, rectangle, polygon, or other geometry defining inner boundary
     */
    void setInnerZone(const Geometry& zone);
    
    /**
     * @brief Clear outer zone (process all points, unlimited range)
     * 
     * After calling this, detection zone is unlimited unless constrained by inner zone.
     * Useful for inner-only mode where you want to exclude one area but process everything else.
     */
    void clearOuterZone();
    
    /**
     * @brief Clear inner zone (no exclusion area)
     * 
     * After calling this, no points are excluded from detection.
     */
    void clearInnerZone();
    
    /**
     * @brief Set robot physical dimensions (for visualization/debugging)
     * 
     * @param length Length in meters (X-axis in robot frame)
     * @param width Width in meters (Y-axis in robot frame)
     * @param height Height in meters (Z-axis, optional)
     */
    void setPhysicalDimensions(const float length, const float width, const float height = 0.0f);
    
    /**
     * @brief Check if point is in detection zone (outer zone but not inner zone)
     * 
     * Returns true if:
     * 1. Point is inside outer zone, AND
     * 2. Point is NOT inside inner zone (or no inner zone exists)
     * 
     * @param point Point to check (world coordinates)
     * @param robot_pos Robot center position (world coordinates)
     * @param robot_heading_rad Robot heading in radians (0 = +X axis)
     * @return true if point should be processed for obstacle matching
     */
    [[nodiscard]] bool isInDetectionZone(
        const Point2D& point,
        const Point2D& robot_pos,
        const float robot_heading_rad) const;
    
    /**
     * @brief Batch check if points are in detection zone (SoA layout, SIMD-optimized)
     *
     * Accepts point coordinates as separate x/y arrays (Structure-of-Arrays) to allow
     * direct SIMD loads without scatter/gather. When USE_SIMD is enabled the
     * outer circle check is fully vectorized; an inner circle exclusion zone is also
     * vectorized. Non-circle inner zones fall back to a scalar per-lane check.
     *
     * Writes 1 (in zone) or 0 (out of zone) into results_out for each point.
     *
     * @param x_coords  World-space X coordinates (float array, length count)
     * @param y_coords  World-space Y coordinates (float array, length count)
     * @param results_out Output array, 0/1 per point (must hold count elements)
     * @param count       Number of points to check
     * @param robot_pos   Robot center in world coordinates
     * @param robot_heading_rad Robot heading in radians (0 = +X axis)
     */
    void isInDetectionZoneBatch(
        const float* __restrict__ x_coords,
        const float* __restrict__ y_coords,
        uint8_t* __restrict__ results_out,
        std::size_t count,
        const Point2D& robot_pos,
        float robot_heading_rad) const;
    
    /**
     * @brief Check if point is in outer zone
     * 
     * @param point Point to check (world coordinates)
     * @param robot_pos Robot center position (world coordinates)
     * @param robot_heading_rad Robot heading in radians
     * @return true if point is inside outer zone boundary
     */
    [[nodiscard]] bool isInOuterZone(
        const Point2D& point,
        const Point2D& robot_pos,
        const float robot_heading_rad) const;
    
    /**
     * @brief Check if point is in inner zone (exclusion area)
     * 
     * @param point Point to check (world coordinates)
     * @param robot_pos Robot center position (world coordinates)
     * @param robot_heading_rad Robot heading in radians
     * @return true if point is inside inner zone boundary
     */
    [[nodiscard]] bool isInInnerZone(
        const Point2D& point,
        const Point2D& robot_pos,
        const float robot_heading_rad) const;
    
    /**
     * @brief Get bounding box for spatial queries (based on outer zone)
     * 
     * Returns conservative bounding box containing entire outer zone.
     * Used to optimize spatial index queries before point-by-point checks.
     * 
     * @param robot_pos Robot center position (world coordinates)
     * @param robot_heading_rad Robot heading in radians
     * @return Axis-aligned bounding box in world coordinates
     */
    [[nodiscard]] BoundingBox getSearchBoundingBox(
        const Point2D& robot_pos,
        const float robot_heading_rad) const;
    
    /**
     * @brief Get maximum search radius (for quick distance checks)
     * 
     * Returns maximum distance from robot center to outer zone boundary.
     * Useful for quickly rejecting points that are definitely outside.
     * 
     * @return Maximum radius in meters
     */
    [[nodiscard]] float getMaxSearchRadius() const;
    
    /**
     * @brief Check if profile has an outer detection zone
     * 
     * @return true if outer zone is configured, false if unlimited range
     */
    [[nodiscard]] bool hasOuterZone() const;
    
    /**
     * @brief Check if profile has an inner exclusion zone
     * 
     * @return true if inner zone is configured
     */
    [[nodiscard]] bool hasInnerZone() const;
    
    // Getters for configuration inspection
    [[nodiscard]] const Geometry* getOuterZone() const { 
        return this->has_outer_zone_ ? &this->outer_zone_ : nullptr;
    }
    [[nodiscard]] const Geometry* getInnerZone() const { 
        return this->has_inner_zone_ ? &this->inner_zone_ : nullptr;
    }
    [[nodiscard]] float getLength() const { return this->length_; }
    [[nodiscard]] float getWidth() const { return this->width_; }
    [[nodiscard]] float getHeight() const { return this->height_; }
    
private:
    Geometry outer_zone_;   // Detection zone (optional - if not set, unlimited range)
    Geometry inner_zone_;   // Exclusion zone (optional)
    bool has_outer_zone_;   // Whether outer zone is active
    bool has_inner_zone_;   // Whether inner zone is active
    
    float length_;   // Robot length in meters (X-axis in robot frame)
    float width_;    // Robot width in meters (Y-axis in robot frame)
    float height_;   // Robot height in meters (Z-axis)
    
    // Cached values for optimization
    float max_search_radius_;  // Maximum distance to outer zone boundary
    
    /**
     * @brief Check if point is inside a safety zone geometry
     * 
     * Handles all geometry types (circle, rectangle, polygon, etc.) with rotation.
     * Uses existing rises::geofence geometric operations.
     * 
     * @param zone Geometry to check against
     * @param point Point to check (world coordinates)
     * @param robot_pos Robot center position (world coordinates)
     * @param robot_heading_rad Robot heading in radians
     * @return true if point is inside zone
     */
    [[nodiscard]] bool isPointInZone(
        const Geometry& zone,
        const Point2D& point,
        const Point2D& robot_pos,
        const float robot_heading_rad) const;
    
    /**
     * @brief Update cached optimization values after zone changes
     */
    void updateCachedValues();
};

} // namespace rises::geofence
