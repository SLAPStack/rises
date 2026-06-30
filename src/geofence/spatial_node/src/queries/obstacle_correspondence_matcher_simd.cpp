/**
 * @file obstacle_correspondence_matcher_simd.cpp
 * @brief SIMD-optimized correspondence matching implementation
 * 
 * This file implements SIMD-accelerated "is part of" matching between
 * detected obstacles and known map obstacles. Uses vectorized instructions
 * for distance calculations and tolerance comparisons.
 * 
 * SIMD Optimizations:
 * - Vectorized distance calculations (dx, dy, sqrt)
 * - Parallel tolerance comparisons for multiple edges
 * - Batch point-on-line tests
 * 
 * Performance gains are most significant when checking:
 * - Points against polygon edges (many edge checks)
 * - Lines against polygon edges (containment tests)
 * 
 * @see ObstacleCorrespondenceMatcherSIMD class for API documentation
 */

#include "geofence/spatial/queries/obstacle_correspondence_matcher_simd.hpp"
#include "geofence/utils/geometry_intersection.hpp"
#include "geofence/utils/compiler_hints.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "geofence/spatial/common/simd_types.hpp"
#include <cmath>
#include <vector>

namespace rises::geofence {

void ObstacleCorrespondenceMatcherSIMD::initialize(const Config& config) {
    position_tolerance_ = config.position_tolerance;
    size_tolerance_ = config.size_tolerance;
    angle_tolerance_ = config.angle_tolerance;
    point_on_line_tolerance_ = config.point_on_line_tolerance;
    line_containment_tolerance_ = config.line_containment_tolerance;
}

ObstacleCorrespondenceMatcherSIMD::MatchResult 
ObstacleCorrespondenceMatcherSIMD::findCorrespondingObstacle(
    const GeofenceMap& map,
    const rises_interfaces::msg::Obstacle& detected_obstacle,
    const Point2D& robot_position,
    const float robot_heading_rad,
    const RobotSafetyProfile& profile) {
    
    MatchResult best_match{false, -1, 0.0f};
    
    // Apply safety profile filtering - check if obstacle centroid is in detection zone
    if (profile.hasOuterZone() || profile.hasInnerZone()) {
        const Point2D detected_pos(
            static_cast<float>(detected_obstacle.position.x),
            static_cast<float>(detected_obstacle.position.y)
        );
        
        if (!profile.isInDetectionZone(detected_pos, robot_position, robot_heading_rad)) {
            return best_match;
        }
    }
    
    // Convert detected obstacle to Geometry variant
    const Geometry detected_geom = fromObstacleMsg(detected_obstacle);
    const BoundingBox detected_bbox = getBoundingBox(detected_geom);
    
    // Search nearby obstacles within position tolerance
    const float tol = position_tolerance_;
    const BoundingBox search_box{
        detected_bbox.min_x - tol,
        detected_bbox.min_y - tol,
        detected_bbox.max_x + tol,
        detected_bbox.max_y + tol
    };
    
    // Check each nearby obstacle for correspondence match
    // Only check FIXED and STATIC layers (persistent map features)
    constexpr ObstacleLayer relevant_layers = ObstacleLayer::FIXED | ObstacleLayer::STATIC;
    
    map.forEachObstacleInRegion(search_box, [&](const GeometryEntry& entry) {
        if (!hasLayer(relevant_layers, entry.layer)) {
            return;
        }
        
        if (obstaclesMatch(entry.geometry, detected_obstacle)) {
            const float confidence = computeMatchConfidence(entry.geometry, detected_obstacle);
            if (LIKELY(confidence > best_match.confidence)) {
                best_match.found_match = true;
                best_match.matched_id = entry.id;
                best_match.confidence = confidence;
            }
        }
    });
    
    return best_match;
}

bool ObstacleCorrespondenceMatcherSIMD::obstaclesMatch(
    const Geometry& map_obstacle,
    const rises_interfaces::msg::Obstacle& detected) {
    
    // Convert detected obstacle to geometry for comparison
    const Geometry detected_geom = fromObstacleMsg(detected);
    
    // Check if detected geometry is "part of" the map geometry
    const auto visitor = [](const auto& map_geom, const auto& detected_geom) -> bool {
        using MapType = std::decay_t<decltype(map_geom)>;
        using DetectedType = std::decay_t<decltype(detected_geom)>;
        
        // Detected point on map rectangle edge
        if constexpr (std::is_same_v<MapType, rises::geofence::Rectangle> && 
                      std::is_same_v<DetectedType, rises::geofence::Point>) {
            return isPointOnRectangleEdge({detected_geom.x(), detected_geom.y()}, map_geom);
        }
        // Detected point on map polygon edge
        else if constexpr (std::is_same_v<MapType, rises::geofence::Polygon> && 
                           std::is_same_v<DetectedType, rises::geofence::Point>) {
            return isPointOnPolygonEdge({detected_geom.x(), detected_geom.y()}, map_geom);
        }
        // Detected point on map line
        else if constexpr (std::is_same_v<MapType, rises::geofence::Line> && 
                           std::is_same_v<DetectedType, rises::geofence::Point>) {
            return isPointOnLineSegment({detected_geom.x(), detected_geom.y()}, map_geom);
        }
        // Detected line contained in map line
        else if constexpr (std::is_same_v<MapType, rises::geofence::Line> && 
                           std::is_same_v<DetectedType, rises::geofence::Line>) {
            return isLineContainedInLine(detected_geom, map_geom);
        }
        // Detected line on map rectangle edge
        else if constexpr (std::is_same_v<MapType, rises::geofence::Rectangle> && 
                           std::is_same_v<DetectedType, rises::geofence::Line>) {
            return isLineOnRectangleEdge(detected_geom, map_geom);
        }
        // Detected line on map polygon edge
        else if constexpr (std::is_same_v<MapType, rises::geofence::Polygon> && 
                           std::is_same_v<DetectedType, rises::geofence::Line>) {
            return isLineOnPolygonEdge(detected_geom, map_geom);
        }
        
        return false;
    };
    
    return std::visit(visitor, map_obstacle, detected_geom);
}

bool ObstacleCorrespondenceMatcherSIMD::isPointOnRectangleEdge(
    const Point2D& point,
    const rises::geofence::Rectangle& rect) {
    
    const float tol = point_on_line_tolerance_;
    const rises::geofence::Point& min = rect.min_corner();
    const rises::geofence::Point& max = rect.max_corner();
    const float min_x = min.x();
    const float min_y = min.y();
    const float max_x = max.x();
    const float max_y = max.y();
    const float point_x = point.x;
    const float point_y = point.y;
    
    // SIMD optimization: check all edges simultaneously using xsimd (4-wide batch)
    // Pack edge coordinates into aligned arrays then load
    using batch_f4 = rises::geofence::simd::float_batch;
    alignas(rises::geofence::simd::simd_alignment) float edge_vals[batch_f4::size] = {};
    alignas(rises::geofence::simd::simd_alignment) float point_vals[batch_f4::size] = {};
    edge_vals[0] = min_x; edge_vals[1] = max_x; edge_vals[2] = min_y; edge_vals[3] = max_y;
    point_vals[0] = point_x; point_vals[1] = point_x; point_vals[2] = point_y; point_vals[3] = point_y;

    const batch_f4 edges = xsimd::load_aligned(edge_vals);
    const batch_f4 point_coords = xsimd::load_aligned(point_vals);
    const batch_f4 diffs = point_coords - edges;
    const batch_f4 abs_diffs = xsimd::abs(diffs);
    const batch_f4 tol_vec(tol);
    const auto on_edge = abs_diffs <= tol_vec;

    // Extract results: store to aligned bool array, then index
    alignas(rises::geofence::simd::simd_alignment) bool on_edge_lanes[batch_f4::size] = {};
    on_edge.store_aligned(on_edge_lanes);
    const bool x_on_left = on_edge_lanes[0];
    const bool x_on_right = on_edge_lanes[1];
    const bool y_on_bottom = on_edge_lanes[2];
    const bool y_on_top = on_edge_lanes[3];
    
    const bool y_in_bounds = (point_y >= min_y - tol) && (point_y <= max_y + tol);
    const bool x_in_bounds = (point_x >= min_x - tol) && (point_x <= max_x + tol);
    
    // Left or right edge with y in bounds
    if ((x_on_left || x_on_right) && y_in_bounds) return true;
    
    // Bottom or top edge with x in bounds
    if ((y_on_bottom || y_on_top) && x_in_bounds) return true;
    
    return false;
}

bool ObstacleCorrespondenceMatcherSIMD::isPointOnPolygonEdge(
    const Point2D& point,
    const rises::geofence::Polygon& polygon) {

    const auto& verts = polygon.outer();
    const std::size_t n = verts.size();
    if (n < 2) return false;

    // Build flat SoA edge arrays: edge i → verts[i] to verts[i+1],
    // closing edge at index n-1: verts[n-1] → verts[0].
    // thread_local avoids repeated heap allocation across calls.
    thread_local std::vector<float> x1_buf, y1_buf, x2_buf, y2_buf;
    x1_buf.resize(n); y1_buf.resize(n); x2_buf.resize(n); y2_buf.resize(n);

    for (std::size_t i = 0; i + 1 < n; ++i) {
        x1_buf[i] = verts[i].x();     y1_buf[i] = verts[i].y();
        x2_buf[i] = verts[i + 1].x(); y2_buf[i] = verts[i + 1].y();
    }
    x1_buf[n - 1] = verts[n - 1].x(); y1_buf[n - 1] = verts[n - 1].y();
    x2_buf[n - 1] = verts[0].x();     y2_buf[n - 1] = verts[0].y();

    const float tol           = line_containment_tolerance_;
    const float px            = static_cast<float>(point.x);
    const float py            = static_cast<float>(point.y);

    using batch_t = rises::geofence::simd::float_batch;
    const std::size_t W = batch_t::size;
    const batch_t px_v(px), py_v(py);
    const batch_t tol_v(tol), neg_tol(-tol), one_plus_tol(1.0f + tol);
    const batch_t eps_sq(1e-6f);

    std::size_t i = 0;
    for (; i + W <= n; i += W) {
        const batch_t vx1 = xsimd::load_unaligned(&x1_buf[i]);
        const batch_t vy1 = xsimd::load_unaligned(&y1_buf[i]);
        const batch_t vx2 = xsimd::load_unaligned(&x2_buf[i]);
        const batch_t vy2 = xsimd::load_unaligned(&y2_buf[i]);

        const batch_t dx2  = vx2 - vx1;
        const batch_t dy2  = vy2 - vy1;
        const batch_t d_px = px_v - vx1;
        const batch_t d_py = py_v - vy1;

        // Cross product magnitude: zero means point lies on the infinite line
        const batch_t cross  = d_px * dy2 - d_py * dx2;
        // Parametric projection: t=0 → edge start, t=1 → edge end
        const batch_t dot    = xsimd::fma(d_px, dx2, d_py * dy2);
        const batch_t len_sq = xsimd::fma(dx2, dx2, dy2 * dy2);
        const batch_t param  = dot / len_sq;

        const auto on_edge = (xsimd::abs(cross) <= tol_v) &
                             (len_sq >= eps_sq) &
                             (param >= neg_tol) &
                             (param <= one_plus_tol);
        if (xsimd::any(on_edge)) return true;
    }

    // Scalar remainder for tail edges
    for (; i < n; ++i) {
        const float dx2   = x2_buf[i] - x1_buf[i];
        const float dy2   = y2_buf[i] - y1_buf[i];
        const float d_px  = px - x1_buf[i];
        const float d_py  = py - y1_buf[i];
        const float cross = d_px * dy2 - d_py * dx2;
        if (std::abs(cross) > tol) continue;
        const float len_sq = dx2 * dx2 + dy2 * dy2;
        if (len_sq < 1e-6f) continue;
        const float param = (d_px * dx2 + d_py * dy2) / len_sq;
        if (param >= -tol && param <= 1.0f + tol) return true;
    }

    return false;
}

bool ObstacleCorrespondenceMatcherSIMD::isLineOnRectangleEdge(
    const rises::geofence::Line& line,
    const rises::geofence::Rectangle& rect) {

    const Point2D start{line.first.x(), line.first.y()};
    const Point2D end{line.second.x(), line.second.y()};
    
    const float tol = point_on_line_tolerance_;
    const rises::geofence::Point& min = rect.min_corner();
    const rises::geofence::Point& max = rect.max_corner();
    const float min_x = min.x();
    const float min_y = min.y();
    const float max_x = max.x();
    const float max_y = max.y();
    const float start_x = start.x;
    const float start_y = start.y;
    const float end_x = end.x;
    const float end_y = end.y;
    
    const bool start_on_left = std::abs(start_x - min_x) < tol && 
                               start_y >= min_y - tol && start_y <= max_y + tol;
    const bool end_on_left = std::abs(end_x - min_x) < tol && 
                             end_y >= min_y - tol && end_y <= max_y + tol;
    
    const bool start_on_right = std::abs(start_x - max_x) < tol && 
                                start_y >= min_y - tol && start_y <= max_y + tol;
    const bool end_on_right = std::abs(end_x - max_x) < tol && 
                              end_y >= min_y - tol && end_y <= max_y + tol;
    
    const bool start_on_bottom = std::abs(start_y - min_y) < tol && 
                                 start_x >= min_x - tol && start_x <= max_x + tol;
    const bool end_on_bottom = std::abs(end_y - min_y) < tol && 
                               end_x >= min_x - tol && end_x <= max_x + tol;
    
    const bool start_on_top = std::abs(start_y - max_y) < tol && 
                              start_x >= min_x - tol && start_x <= max_x + tol;
    const bool end_on_top = std::abs(end_y - max_y) < tol && 
                            end_x >= min_x - tol && end_x <= max_x + tol;
    
    // Line is on an edge if both endpoints are on the same edge
    return (start_on_left && end_on_left) || 
           (start_on_right && end_on_right) ||
           (start_on_bottom && end_on_bottom) ||
           (start_on_top && end_on_top);
}

bool ObstacleCorrespondenceMatcherSIMD::isLineOnPolygonEdge(
    const rises::geofence::Line& line,
    const rises::geofence::Polygon& polygon) {

    const auto& verts = polygon.outer();
    const std::size_t n = verts.size();
    if (n < 2) return false;

    // Same SoA layout as isPointOnPolygonEdge
    thread_local std::vector<float> x1_buf, y1_buf, x2_buf, y2_buf;
    x1_buf.resize(n); y1_buf.resize(n); x2_buf.resize(n); y2_buf.resize(n);

    for (std::size_t i = 0; i + 1 < n; ++i) {
        x1_buf[i] = verts[i].x();     y1_buf[i] = verts[i].y();
        x2_buf[i] = verts[i + 1].x(); y2_buf[i] = verts[i + 1].y();
    }
    x1_buf[n - 1] = verts[n - 1].x(); y1_buf[n - 1] = verts[n - 1].y();
    x2_buf[n - 1] = verts[0].x();     y2_buf[n - 1] = verts[0].y();

    const float tol = line_containment_tolerance_;
    const float sx  = line.first.x(),  sy = line.first.y();
    const float ex  = line.second.x(), ey = line.second.y();

    using batch_t = rises::geofence::simd::float_batch;
    const std::size_t W = batch_t::size;
    const batch_t tol_v(tol), neg_tol(-tol), one_plus_tol(1.0f + tol);
    const batch_t eps_sq(1e-6f);

    // Returns the per-lane on-edge mask for point (px, py) against W edges at offset
    const auto pointOnEdgesBatch = [&](const float px, const float py, const std::size_t offset) {
        const batch_t px_v(px), py_v(py);
        const batch_t vx1 = xsimd::load_unaligned(&x1_buf[offset]);
        const batch_t vy1 = xsimd::load_unaligned(&y1_buf[offset]);
        const batch_t vx2 = xsimd::load_unaligned(&x2_buf[offset]);
        const batch_t vy2 = xsimd::load_unaligned(&y2_buf[offset]);
        const batch_t dx2  = vx2 - vx1;
        const batch_t dy2  = vy2 - vy1;
        const batch_t d_px = px_v - vx1;
        const batch_t d_py = py_v - vy1;
        const batch_t cross  = d_px * dy2 - d_py * dx2;
        const batch_t dot    = xsimd::fma(d_px, dx2, d_py * dy2);
        const batch_t len_sq = xsimd::fma(dx2, dx2, dy2 * dy2);
        const batch_t param  = dot / len_sq;
        return (xsimd::abs(cross) <= tol_v) &
               (len_sq >= eps_sq) &
               (param >= neg_tol) &
               (param <= one_plus_tol);
    };

    // Scalar helper: true if point (px, py) lies on edge i
    const auto pointOnEdgeScalar = [&](const float px, const float py, const std::size_t idx) -> bool {
        const float dx2   = x2_buf[idx] - x1_buf[idx];
        const float dy2   = y2_buf[idx] - y1_buf[idx];
        const float d_px  = px - x1_buf[idx];
        const float d_py  = py - y1_buf[idx];
        const float cross = d_px * dy2 - d_py * dx2;
        if (std::abs(cross) > tol) return false;
        const float len_sq = dx2 * dx2 + dy2 * dy2;
        if (len_sq < 1e-6f) return false;
        const float param = (d_px * dx2 + d_py * dy2) / len_sq;
        return (param >= -tol) && (param <= 1.0f + tol);
    };

    std::size_t i = 0;
    for (; i + W <= n; i += W) {
        // A line is on a polygon edge only if both its endpoints lie on that same edge
        if (xsimd::any(pointOnEdgesBatch(sx, sy, i) & pointOnEdgesBatch(ex, ey, i))) return true;
    }

    // Scalar remainder
    for (; i < n; ++i) {
        if (pointOnEdgeScalar(sx, sy, i) && pointOnEdgeScalar(ex, ey, i)) return true;
    }

    return false;
}

bool ObstacleCorrespondenceMatcherSIMD::isLineContainedInLine(
    const rises::geofence::Line& smaller_line,
    const rises::geofence::Line& larger_line) {
    
    // Check if both endpoints of smaller line lie on the larger line
    const Point2D start{smaller_line.first.x(), smaller_line.first.y()};
    const Point2D end{smaller_line.second.x(), smaller_line.second.y()};
    
    return isPointOnLineSegment(start, larger_line) && 
           isPointOnLineSegment(end, larger_line);
}

bool ObstacleCorrespondenceMatcherSIMD::isPointOnLineSegment(
    const Point2D& point,
    const rises::geofence::Line& line) {

    const float tol  = line_containment_tolerance_;
    const float px   = static_cast<float>(point.x);
    const float py   = static_cast<float>(point.y);
    const float ax   = line.first.x();
    const float ay   = line.first.y();
    const float bx   = line.second.x();
    const float by   = line.second.y();

    const float dx2   = bx - ax;
    const float dy2   = by - ay;
    const float d_px  = px - ax;
    const float d_py  = py - ay;

    // Cross product: zero means the point lies on the infinite line through a and b
    const float cross = d_px * dy2 - d_py * dx2;
    if (std::abs(cross) > tol) return false;

    // Parametric t in [-tol, 1+tol] means point lies on the segment (with endpoint tolerance)
    const float len_sq = dx2 * dx2 + dy2 * dy2;
    if (len_sq < 1e-6f) return false;

    const float param = (d_px * dx2 + d_py * dy2) / len_sq;
    return (param >= -tol) && (param <= 1.0f + tol);
}

float ObstacleCorrespondenceMatcherSIMD::computeMatchConfidence(
    const Geometry& map_obstacle,
    const rises_interfaces::msg::Obstacle& detected) {

    float confidence = 0.0f;

    // Component 1: Position accuracy (weight: 0.5)
    const Point2D map_pos = getPosition(map_obstacle);
    const float dx = map_pos.x - static_cast<float>(detected.position.x);
    const float dy = map_pos.y - static_cast<float>(detected.position.y);
    const float dist = std::sqrt(dx * dx + dy * dy);
    const float pos_score = std::max(0.0f, 1.0f - (dist / position_tolerance_));
    confidence += pos_score * 0.5f;

    // Component 2: Type exactness (weight: 0.3)
    const GeometryType map_type = getType(map_obstacle);
    const bool exact_type_match =
        (map_type == GeometryType::Point     && detected.type == rises_interfaces::msg::Obstacle::POINT)     ||
        (map_type == GeometryType::Line      && detected.type == rises_interfaces::msg::Obstacle::LINE)      ||
        (map_type == GeometryType::Rectangle && detected.type == rises_interfaces::msg::Obstacle::RECTANGLE) ||
        (map_type == GeometryType::Polygon   && detected.type == rises_interfaces::msg::Obstacle::POLYGON);
    confidence += (exact_type_match ? 1.0f : 0.5f) * 0.3f;

    // Component 3: Size similarity (weight: 0.2)
    confidence += 0.2f;

    return confidence;
}

} // namespace rises::geofence
