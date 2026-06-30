#include "geofence/spatial/policies/robot_safety_profile.hpp"
#include "variant_shapes.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

#ifdef USE_SIMD
#include "geofence/spatial/common/simd_types.hpp"
#endif

namespace rises::geofence {

RobotSafetyProfile::RobotSafetyProfile()
    : outer_zone_(rises::geofence::makeCircle(0.0f, 0.0f, 2.0f))  // Default: 2m circle at origin
    , inner_zone_(rises::geofence::makeCircle(0.0f, 0.0f, 0.0f))  // Placeholder
    , has_outer_zone_(true)   // Default: outer zone enabled
    , has_inner_zone_(false)  // Default: no inner zone
    , length_(1.0f)
    , width_(0.6f)
    , height_(0.5f)
    , max_search_radius_(2.0f)
{
}

void RobotSafetyProfile::setOuterZone(const Geometry& zone) {
    this->outer_zone_ = zone;
    this->has_outer_zone_ = true;
    this->updateCachedValues();
}

void RobotSafetyProfile::setInnerZone(const Geometry& zone) {
    this->inner_zone_ = zone;
    this->has_inner_zone_ = true;
}

void RobotSafetyProfile::clearOuterZone() {
    this->has_outer_zone_ = false;
    this->max_search_radius_ = std::numeric_limits<float>::infinity();
}

void RobotSafetyProfile::clearInnerZone() {
    this->has_inner_zone_ = false;
}

void RobotSafetyProfile::setPhysicalDimensions(const float length, const float width, const float height) {
    this->length_ = length;
    this->width_ = width;
    this->height_ = height;
}

bool RobotSafetyProfile::isInDetectionZone(
    const Point2D& point,
    const Point2D& robot_pos,
    const float robot_heading_rad) const
{
    // Three modes:
    // 1. Outer + Inner: Must be in outer AND not in inner (donut)
    // 2. Outer only: Must be in outer (simple boundary)
    // 3. Inner only: Must NOT be in inner (inverse - process everything except inner)
    
    // Check outer zone constraint (if active)
    if (this->has_outer_zone_) {
        if (!this->isPointInZone(this->outer_zone_, point, robot_pos, robot_heading_rad)) {
            return false;  // Outside outer boundary
        }
    }
    // If no outer zone: unlimited range, all points pass this check
    
    // Check inner zone exclusion (if active)
    if (this->has_inner_zone_) {
        if (this->isPointInZone(this->inner_zone_, point, robot_pos, robot_heading_rad)) {
            return false;  // Point in exclusion zone, ignore it
        }
    }
    
    return true;
}

// Batch zone check with SoA input. SIMD path vectorizes the outer circle check and,
// when the inner exclusion zone is also a circle, the exclusion check as well.
// Non-circle inner zones fall back per-lane to the scalar isInDetectionZone path.
void RobotSafetyProfile::isInDetectionZoneBatch(
    const float* __restrict__ x_coords,
    const float* __restrict__ y_coords,
    uint8_t* __restrict__ results_out,
    const std::size_t count,
    const Point2D& robot_pos,
    const float robot_heading_rad) const
{
    const float cos_theta = std::cos(-robot_heading_rad);
    const float sin_theta = std::sin(-robot_heading_rad);

    if (this->has_outer_zone_ && std::holds_alternative<Circle>(this->outer_zone_)) {
        const Circle& outer_circle = std::get<Circle>(this->outer_zone_);
        const float outer_r_sq = outer_circle.radius * outer_circle.radius;

        // Determine if inner zone is also a circle so we can keep the inner check vectorized.
        const bool inner_is_circle = this->has_inner_zone_ && std::holds_alternative<Circle>(this->inner_zone_);
        float inner_cx = 0.0f, inner_cy = 0.0f, inner_r_sq = 0.0f;
        if (inner_is_circle) {
            const Circle& ic = std::get<Circle>(this->inner_zone_);
            inner_cx = ic.center.x();
            inner_cy = ic.center.y();
            inner_r_sq = ic.radius * ic.radius;
        }

#ifdef USE_SIMD
        using simd_batch = rises::geofence::simd::float_batch;
        constexpr std::size_t simd_size = simd_batch::size;
        const std::size_t simd_end = count - (count % simd_size);

        const simd_batch cos_b(cos_theta), sin_b(sin_theta);
        const simd_batch rx_b(robot_pos.x), ry_b(robot_pos.y);
        const simd_batch outer_r_sq_b(outer_r_sq);
        const simd_batch inner_cx_b(inner_cx), inner_cy_b(inner_cy);
        const simd_batch inner_r_sq_b(inner_r_sq);

        for (std::size_t i = 0; i < simd_end; i += simd_size) {
            // Inputs are already SoA: no scatter/gather needed.
            const simd_batch x = xsimd::load_unaligned(&x_coords[i]);
            const simd_batch y = xsimd::load_unaligned(&y_coords[i]);

            // Transform to robot-local frame.
            const simd_batch dx = x - rx_b;
            const simd_batch dy = y - ry_b;
            const simd_batch lx = dx * cos_b - dy * sin_b;
            const simd_batch ly = dx * sin_b + dy * cos_b;

            // Outer circle containment.
            const simd_batch dist_sq = xsimd::fma(lx, lx, ly * ly);
            auto in_zone = dist_sq <= outer_r_sq_b;

            if (inner_is_circle) {
                // Inner circle exclusion: vectorized using local-frame coordinates.
                const simd_batch dxi = lx - inner_cx_b;
                const simd_batch dyi = ly - inner_cy_b;
                const simd_batch inner_dist_sq = xsimd::fma(dxi, dxi, dyi * dyi);
                in_zone = in_zone & (inner_dist_sq > inner_r_sq_b);
            }

            alignas(rises::geofence::simd::simd_alignment) bool lane[simd_size];
            in_zone.store_aligned(lane);

            // Non-circle inner zone: scalar fallback per-lane (rare case).
            if (this->has_inner_zone_ && !inner_is_circle) {
                alignas(rises::geofence::simd::simd_alignment) float lx_arr[simd_size];
                alignas(rises::geofence::simd::simd_alignment) float ly_arr[simd_size];
                lx.store_aligned(lx_arr);
                ly.store_aligned(ly_arr);
                for (std::size_t j = 0; j < simd_size; ++j) {
                    if (lane[j]) {
                        const Point2D local_pt{lx_arr[j], ly_arr[j]};
                        if (std::visit([&local_pt](const auto& geom) { return contains(geom, local_pt); }, this->inner_zone_)) {
                            lane[j] = false;
                        }
                    }
                }
            }

            for (std::size_t j = 0; j < simd_size; ++j) {
                results_out[i + j] = lane[j] ? 1u : 0u;
            }
        }

        // Scalar remainder.
        for (std::size_t i = simd_end; i < count; ++i) {
            const float dx = x_coords[i] - robot_pos.x;
            const float dy = y_coords[i] - robot_pos.y;
            const float lx = dx * cos_theta - dy * sin_theta;
            const float ly = dx * sin_theta + dy * cos_theta;
            const float dist_sq = lx * lx + ly * ly;
            bool in_zone = dist_sq <= outer_r_sq;
            if (in_zone && this->has_inner_zone_) {
                const Point2D local_pt{lx, ly};
                if (std::visit([&local_pt](const auto& geom) { return contains(geom, local_pt); }, this->inner_zone_)) {
                    in_zone = false;
                }
            }
            results_out[i] = in_zone ? 1u : 0u;
        }
#else
        for (std::size_t i = 0; i < count; ++i) {
            const float dx = x_coords[i] - robot_pos.x;
            const float dy = y_coords[i] - robot_pos.y;
            const float lx = dx * cos_theta - dy * sin_theta;
            const float ly = dx * sin_theta + dy * cos_theta;
            const float dist_sq = lx * lx + ly * ly;
            bool in_zone = dist_sq <= outer_r_sq;
            if (in_zone && this->has_inner_zone_) {
                const Point2D local_pt{lx, ly};
                if (std::visit([&local_pt](const auto& geom) { return contains(geom, local_pt); }, this->inner_zone_)) {
                    in_zone = false;
                }
            }
            results_out[i] = in_zone ? 1u : 0u;
        }
#endif
    } else {
        // Generic path for non-circle outer zones (rectangles, polygons, no outer zone).
        for (std::size_t i = 0; i < count; ++i) {
            const Point2D pt{x_coords[i], y_coords[i]};
            results_out[i] = this->isInDetectionZone(pt, robot_pos, robot_heading_rad) ? 1u : 0u;
        }
    }
}

bool RobotSafetyProfile::isInOuterZone(
    const Point2D& point,
    const Point2D& robot_pos,
    const float robot_heading_rad) const
{
    if (!this->has_outer_zone_) {
        return true;  // No outer zone = infinite range, all points are "in"
    }
    return this->isPointInZone(this->outer_zone_, point, robot_pos, robot_heading_rad);
}

bool RobotSafetyProfile::isInInnerZone(
    const Point2D& point,
    const Point2D& robot_pos,
    const float robot_heading_rad) const
{
    if (!this->has_inner_zone_) {
        return false;
    }
    return this->isPointInZone(this->inner_zone_, point, robot_pos, robot_heading_rad);
}

BoundingBox RobotSafetyProfile::getSearchBoundingBox(
    const Point2D& robot_pos,
    const float robot_heading_rad) const
{
    // Conservative bounding box based on max search radius
    // This is guaranteed to contain the entire outer zone
    const float radius = this->max_search_radius_;
    return BoundingBox(
        robot_pos.x - radius,
        robot_pos.y - radius,
        robot_pos.x + radius,
        robot_pos.y + radius
    );
}

float RobotSafetyProfile::getMaxSearchRadius() const {
    return this->max_search_radius_;
}

bool RobotSafetyProfile::hasOuterZone() const {
    return this->has_outer_zone_;
}

bool RobotSafetyProfile::hasInnerZone() const {
    return this->has_inner_zone_;
}

bool RobotSafetyProfile::isPointInZone(
    const Geometry& zone,
    const Point2D& point,
    const Point2D& robot_pos,
    const float robot_heading_rad) const
{
    // Transform point to robot-local coordinates
    const float cos_theta = std::cos(-robot_heading_rad);
    const float sin_theta = std::sin(-robot_heading_rad);
    
    // Translate to robot center
    const float dx = point.x - robot_pos.x;
    const float dy = point.y - robot_pos.y;
    
    // Rotate to robot frame
    const float local_x = dx * cos_theta - dy * sin_theta;
    const float local_y = dx * sin_theta + dy * cos_theta;
    
    // Use existing contains() operation
    // Zone geometries are defined in robot-local coordinates, so we test the transformed point
    const Point2D test_point{local_x, local_y};
    
    return std::visit([&test_point](const auto& geometry) -> bool {
        return contains(geometry, test_point);
    }, zone);
}

void RobotSafetyProfile::updateCachedValues() {
    // Calculate maximum search radius based on outer zone bounding box
    if (!this->has_outer_zone_) {
        this->max_search_radius_ = std::numeric_limits<float>::infinity();
        return;
    }
    
    // Use existing getBoundingBox operation from rises::geofence
    const BoundingBox bbox = getBoundingBox(this->outer_zone_);
    
    // Maximum radius is distance from origin (robot center) to farthest corner
    const float max_x = std::max(std::abs(bbox.min_x), std::abs(bbox.max_x));
    const float max_y = std::max(std::abs(bbox.min_y), std::abs(bbox.max_y));
    this->max_search_radius_ = std::sqrt(max_x * max_x + max_y * max_y);
}

} // namespace rises::geofence
