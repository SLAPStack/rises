/**
 * @file obstacle_match_checker.cpp
 * @brief Scan matching implementation for obstacle correspondence
 * 
 * This file implements laser scan matching against known obstacles and
 * map boundaries. Used for:
 * - Scan registration: Matching scan segments to map features
 * - Unknown obstacle detection: Identifying scans that don't match any known geometry
 * - Data association: Creating correspondences between scans and obstacles
 * 
 * Matching Algorithm:
 * 1. Check if scan segment matches any known obstacle (within tolerances)
 * 2. If not, check if it matches map contours
 * 3. If neither, classify as unknown obstacle (potential new object)
 * 
 * Layer Filtering:
 * - Only matches against FIXED and STATIC layers
 * - Ignores DYNAMIC layer (transient objects shouldn't be reference features)
 * 
 * Tolerances:
 * - Position tolerance: Maximum distance between scan and feature
 * - Angular tolerance: Maximum angle difference for line features
 * 
 * @see ObstacleMatchChecker class for API documentation
 */

#include "geofence/spatial/queries/obstacle_match_checker.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include <cmath>

namespace rises::geofence {

/**
 * @brief Initialize static configuration
 * 
 * @param config Configuration specifying position/angular tolerances and
 *               which feature types to match against
 */
void ObstacleMatchChecker::initialize(const Config& config) {
    position_tolerance_ = config.position_tolerance;
    angular_tolerance_ = config.angular_tolerance;
    match_obstacles_ = config.match_obstacles;
    match_warehouse_ = config.match_warehouse;
}

/**
 * @brief Helper: Check if scan line matches a geometry shape
 * 
 * Uses position and angular tolerances to determine if scan segment
 * corresponds to an edge of the given shape.
 * 
 * @param scan_line Laser scan segment in backend format
 * @param shape Obstacle geometry to match against
 * @param eps_pos Position tolerance (meters)
 * @param eps_angle Angular tolerance (radians)
 * @return true if scan matches shape edge
 */
bool ObstacleMatchChecker::matchesScanToShape(
    const rises::geofence::Line& scan_line,
    const Geometry& shape,
    const float eps_pos,
    const float eps_angle) {
    
    return std::visit([&](const auto& geom) -> bool {
        using GeomType = std::decay_t<decltype(geom)>;
        
        // Point: Check if point is on scan line
        if constexpr (std::is_same_v<GeomType, rises::geofence::Point>) {
            return rises::geofence::distance(geom, scan_line) <= eps_pos;
        }
        // Line: Check if lines match (close + parallel)
        else if constexpr (std::is_same_v<GeomType, rises::geofence::Line>) {
            return matchScanToLine(scan_line, geom, eps_pos, eps_angle);
        }
        // Rectangle: Check if scan matches any rectangle edge
        else if constexpr (std::is_same_v<GeomType, rises::geofence::Rectangle>) {
            return matchScanToRectangle(scan_line, geom, eps_pos, eps_angle);
        }
        // Polygon: Check if scan matches any polygon edge  
        else if constexpr (std::is_same_v<GeomType, rises::geofence::Polygon>) {
            return matchScanToPolygon(scan_line, geom, eps_pos, eps_angle);
        }
        else {
            return false;
        }
    }, shape);
}

/**
 * @brief Check if scan matches any obstacle in the map
 * 
 * Queries spatial index for nearby obstacles and tests each for match.
 * 
 * @param map Geofence map
 * @param scan_start Scan start point
 * @param scan_end Scan end point
 * @param matched_id Output parameter for matched obstacle ID (optional)
 * @return true if scan matches any obstacle
 */
bool ObstacleMatchChecker::matchesAnyObstacle(
    const GeofenceMap& map,
    const Point2D& scan_start,
    const Point2D& scan_end,
    int64_t* matched_id) {
    
    // Create scan line in backend format
    const rises::geofence::Line scan_line = rises::geofence::makeLine(
        static_cast<float>(scan_start.x), static_cast<float>(scan_start.y),
        static_cast<float>(scan_end.x), static_cast<float>(scan_end.y)
    );
    
    // Create bounding box for spatial query
    const float tol = position_tolerance_;
    const float min_x = std::min(scan_start.x, scan_end.x) - tol;
    const float min_y = std::min(scan_start.y, scan_end.y) - tol;
    const float max_x = std::max(scan_start.x, scan_end.x) + tol;
    const float max_y = std::max(scan_start.y, scan_end.y) + tol;
    const BoundingBox query_box{
        static_cast<float>(min_x), static_cast<float>(min_y),
        static_cast<float>(max_x), static_cast<float>(max_y)
    };
    
    // Match against FIXED and STATIC obstacles only
    // Dynamic obstacles are transient and shouldn't be reference features for scan matching
    constexpr ObstacleLayer relevant_layers = ObstacleLayer::FIXED | ObstacleLayer::STATIC;
    
    bool found_match = false;
    map.forEachObstacleInRegion(query_box, [&](const GeometryEntry& entry) {
        if (!hasLayer(relevant_layers, entry.layer)) {
            return;
        }
        
        if (matchesScanToShape(scan_line, entry.geometry, 
                               position_tolerance_,
                               angular_tolerance_)) {
            if (matched_id) {
                *matched_id = entry.id;
            }
            found_match = true;
        }
    });
    
    return found_match;
}

/**
 * @brief Attempts to match a laser scan segment against map features
 * 
 * Checks in priority order:
 * 1. Known obstacles (if enabled in config)
 * 2. Map contours (if enabled in config)
 * 3. Returns UNKNOWN if no match found
 * 
 * @param map Geofence map containing obstacles and map boundaries
 * @param scan_start Start point of the scan segment
 * @param scan_end End point of the scan segment
 * @return Match result with type (OBSTACLE/WAREHOUSE/UNKNOWN) and ID
 * 
 * @note Position and angular tolerances come from config_
 * @note Obstacle matches return the matched obstacle ID
 * @note Unknown matches indicate potential new obstacles in the environment
 */
ObstacleMatchChecker::MatchResult ObstacleMatchChecker::matchScanSegment(
    const GeofenceMap& map,
    const Point2D& scan_start,
    const Point2D& scan_end) {
    
    // Priority 1: Check obstacles (dynamic/movable things)
    if (match_obstacles_) {
        int64_t matched_id = -1;
        if (matchesAnyObstacle(map, scan_start, scan_end, &matched_id)) {
            return MatchResult{MatchType::OBSTACLE, matched_id};
        }
    }
    
    // Priority 2: Check map contours (static walls)
    if (match_warehouse_) {
        const rises::shape::MapBoundaryContours* contours = map.getMapContours();
        if (contours && matchesWarehouse(*contours, scan_start, scan_end)) {
            return MatchResult{MatchType::WAREHOUSE, -1};
        }
    }
    
    // No match found - this is an unknown obstacle!
    return MatchResult{MatchType::UNKNOWN, -1};
}

/**
 * @brief Finds all scan segments that don't match any known map features
 * 
 * Iterates through all scan segments and identifies those that don't
 * correspond to any known obstacle or warehouse boundary. These unmatched
 * scans likely represent new or moved obstacles.
 * 
 * @param map Geofence map containing known obstacles and boundaries
 * @param scan_segments Vector of scan segments as (start, end) point pairs
 * @return Indices of scan segments that didn't match any known features
 * 
 * @note Pre-allocates assuming ~25% of scans will be unmatched
 * @note Used for dynamic obstacle detection and map updates
 */
std::vector<std::size_t> ObstacleMatchChecker::findUnmatchedScans(
    const GeofenceMap& map,
    const std::vector<std::pair<Point2D, Point2D>>& scan_segments) {
    
    std::vector<std::size_t> unmatched_indices;
    unmatched_indices.reserve(scan_segments.size() / 4);  // Guess ~25% unmatched
    
    for (std::size_t i = 0; i < scan_segments.size(); ++i) {
        const std::pair<const Point2D, const Point2D>& scan_pair = scan_segments[i];
        const Point2D& start = scan_pair.first;
        const Point2D& end = scan_pair.second;
        
        const MatchResult result = matchScanSegment(map, start, end);
        
        if (result.isUnknown()) {
            unmatched_indices.push_back(i);
        }
    }
    
    return unmatched_indices;
}

/**
 * @brief Helper: Match scan line to line obstacle
 */
bool ObstacleMatchChecker::matchScanToLine(
    const rises::geofence::Line& scan,
    const rises::geofence::Line& edge,
    const float eps_pos,
    const float eps_angle) {
    
    // Check if endpoints are close to edge
    const float dist_start = rises::geofence::distance(scan.first, edge);
    const float dist_end = rises::geofence::distance(scan.second, edge);
    
    if (dist_start >= eps_pos || dist_end >= eps_pos) {
        return false;
    }
    
    // Check if lines are parallel (similar angle)
    const rises::geofence::Point scan_dir = rises::geofence::direction(scan);
    const rises::geofence::Point edge_dir = rises::geofence::direction(edge);
    
    const float dot = scan_dir.x() * edge_dir.x() + scan_dir.y() * edge_dir.y();
    const float angle_diff = std::acos(std::clamp(std::abs(dot), 0.0f, 1.0f));
    
    return angle_diff < eps_angle;
}

/**
 * @brief Helper: Match scan line to rectangle obstacle
 */
bool ObstacleMatchChecker::matchScanToRectangle(
    const rises::geofence::Line& scan,
    const rises::geofence::Rectangle& rect,
    const float eps_pos,
    const float eps_angle) {
    
    // Rectangle has 4 edges
    const rises::geofence::Point& min = rect.min_corner();
    const rises::geofence::Point& max = rect.max_corner();
    const rises::geofence::Line edges[4] = {
        rises::geofence::makeLine(min, rises::geofence::Point(max.x(), min.y())),     // Bottom
        rises::geofence::makeLine(rises::geofence::Point(max.x(), min.y()), max),     // Right
        rises::geofence::makeLine(max, rises::geofence::Point(min.x(), max.y())),     // Top
        rises::geofence::makeLine(rises::geofence::Point(min.x(), max.y()), min)      // Left
    };
    
    for (const rises::geofence::Line& edge : edges) {
        if (matchScanToLine(scan, edge, eps_pos, eps_angle)) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Helper: Match scan line to polygon obstacle
 */
bool ObstacleMatchChecker::matchScanToPolygon(
    const rises::geofence::Line& scan,
    const rises::geofence::Polygon& poly,
    const float eps_pos,
    const float eps_angle) {
    
    const boost::geometry::ring_type<rises::geofence::Polygon>::type& verts = poly.outer();
    const std::size_t n = verts.size();
    if (n < 2) {
        return false;
    }
    
    // Check each polygon edge
    for (std::size_t i = 0; i < n - 1; ++i) {
        const rises::geofence::Line edge = rises::geofence::makeLine(verts[i], verts[i + 1]);
        
        if (matchScanToLine(scan, edge, eps_pos, eps_angle)) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Tests if a scan segment matches map boundary contours
 * 
 * Checks if the scan segment aligns with any edge of the boundary contours
 * within position and angular tolerances.
 * 
 * @param contours Map boundary polygon(s)
 * @param scan_start Start point of the scan segment
 * @param scan_end End point of the scan segment
 * @return true if scan matches map boundary within tolerances
 * 
 * @note Uses position and angular tolerances from config_
 */
bool ObstacleMatchChecker::matchesWarehouse(
    const shape::MapBoundaryContours& contours,
    const Point2D& scan_start,
    const Point2D& scan_end) {
    
    const double eps_pos = position_tolerance_;
    const double eps_pos_sq = eps_pos * eps_pos;
    const double eps_angle = angular_tolerance_;

    // Helper to compute squared point-to-line-segment distance (avoids sqrt)
    const auto pointLineDistanceSq = [](const Point2D& p, const Point2D& a, const Point2D& b) -> double {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len_sq = dx * dx + dy * dy;

        if (len_sq < 1e-12) {
            const double ex = p.x - a.x, ey = p.y - a.y;
            return ex * ex + ey * ey;
        }

        const double t = std::clamp(
            ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq,
            0.0, 1.0
        );

        const double ex = p.x - (a.x + t * dx);
        const double ey = p.y - (a.y + t * dy);
        return ex * ex + ey * ey;
    };

    // Helper to check if scan segment matches a contour edge
    const auto matchesEdge = [&](const Point2D& edge_start, const Point2D& edge_end) -> bool {
        const double scan_angle = std::atan2(scan_end.y - scan_start.y,
                                             scan_end.x - scan_start.x);
        const double edge_angle = std::atan2(edge_end.y - edge_start.y,
                                             edge_end.x - edge_start.x);

        double angle_diff = std::abs(scan_angle - edge_angle);
        if (angle_diff > M_PI) angle_diff = 2.0 * 3.14159265358979323846 - angle_diff;

        return pointLineDistanceSq(scan_start, edge_start, edge_end) < eps_pos_sq &&
               pointLineDistanceSq(scan_end,   edge_start, edge_end) < eps_pos_sq &&
               angle_diff < eps_angle;
    };

    // Check outer contour (i+1 < n loop + explicit closing edge, no modulo)
    const std::vector<Point2D>& outer_verts = contours.getOuterContour().getVertices();
    const std::size_t n_outer = outer_verts.size();
    for (std::size_t i = 0; i + 1 < n_outer; ++i) {
        if (matchesEdge(outer_verts[i], outer_verts[i + 1])) return true;
    }
    if (n_outer >= 2 && matchesEdge(outer_verts[n_outer - 1], outer_verts[0])) return true;

    // Check inner contours
    for (const rises::shape::PolygonContour& inner : contours.getInnerContours()) {
        const std::vector<Point2D>& inner_verts = inner.getVertices();
        const std::size_t n_inner = inner_verts.size();
        for (std::size_t i = 0; i + 1 < n_inner; ++i) {
            if (matchesEdge(inner_verts[i], inner_verts[i + 1])) return true;
        }
        if (n_inner >= 2 && matchesEdge(inner_verts[n_inner - 1], inner_verts[0])) return true;
    }
    
    return false;
}

/**
 * @brief Creates correspondences between scan segments and matched obstacles
 * 
 * Returns all scan-to-obstacle matches with their indices and IDs.
 * Used for scan registration, localization, and data association.
 * 
 * @param map Geofence map containing known obstacles
 * @param scan_segments Vector of scan segments as (start, end) point pairs
 * @return Vector of (scan_index, obstacle_id) pairs for all matches
 * 
 * @note Pre-allocates assuming ~50% of scans will match obstacles
 * @note Only includes OBSTACLE matches, not map boundary or unknown
 */
std::vector<std::pair<std::size_t, int64_t>> ObstacleMatchChecker::getObstacleMatches(
    const GeofenceMap& map,
    const std::vector<std::pair<Point2D, Point2D>>& scan_segments) {
    
    std::vector<std::pair<std::size_t, int64_t>> result_matches;
    result_matches.reserve(scan_segments.size() / 2);  // Guess ~50% match obstacles
    
    for (std::size_t i = 0; i < scan_segments.size(); ++i) {
        const std::pair<const Point2D, const Point2D>& scan_pair = scan_segments[i];
        const Point2D& start = scan_pair.first;
        const Point2D& end = scan_pair.second;
        
        int64_t matched_id = -1;
        if (matchesAnyObstacle(map, start, end, &matched_id)) {
            result_matches.emplace_back(i, matched_id);
        }
    }
    
    return result_matches;
}

/**
 * @brief Match a detected obstacle against map obstacles
 * 
 * Takes a detected obstacle (e.g., points from laser scan) and checks if it matches
 * any obstacle in the geofence map. This is the correct direction: scan -> map.
 */
ObstacleMatchChecker::MatchResult ObstacleMatchChecker::matchDetectedObstacle(
    const GeofenceMap& map,
    const rises_interfaces::msg::Obstacle& detected_obstacle) {
    
    // Convert detected obstacle to geometry
    const Geometry detected_geom = rises::geofence::fromObstacleMsg(detected_obstacle);
    
    // Create bounding box for detected obstacle to query nearby map obstacles
    BoundingBox query_box;
    std::visit([&](const auto& geom) {
        rises::geofence::Rectangle bbox = rises::geofence::boundingBox(geom);
        query_box.min_x = bbox.min_corner().x();
        query_box.min_y = bbox.min_corner().y();
        query_box.max_x = bbox.max_corner().x();
        query_box.max_y = bbox.max_corner().y();
    }, detected_geom);
    
    // Expand query box by tolerance
    const float expansion = position_tolerance_;
    query_box.min_x -= expansion;
    query_box.min_y -= expansion;
    query_box.max_x += expansion;  
    query_box.max_y += expansion;
    
    // Query map for nearby obstacles (FIXED + STATIC only, same as matchesAnyObstacle)
    constexpr ObstacleLayer relevant_layers = ObstacleLayer::FIXED | ObstacleLayer::STATIC;
    MatchResult result;
    result.type = MatchType::UNKNOWN;
    result.obstacle_id = -1;

    map.forEachObstacleInRegion(query_box, [&](const GeometryEntry& entry) {
        if (!hasLayer(relevant_layers, entry.layer)) {
            return true;  // Skip dynamic obstacles
        }
        if (matchObstacleGeometries(detected_geom, entry.geometry,
                                  position_tolerance_,
                                  angular_tolerance_)) {
            result.type = MatchType::OBSTACLE;
            result.obstacle_id = entry.id;
            return false;  // Stop searching
        }
        return true;  // Continue searching
    });
    
    // If no obstacle match found, check warehouse boundaries
    if (result.type == MatchType::UNKNOWN && match_warehouse_) {
        const rises::shape::MapBoundaryContours* contours = map.getMapContours();
        if (contours) {
            bool warehouse_match = std::visit([&](const auto& geom) -> bool {
                using T = std::decay_t<decltype(geom)>;

                if constexpr (std::is_same_v<T, rises::geofence::Line>) {
                    // Line: check endpoints directly against contour edges
                    Point2D start(static_cast<double>(geom.first.x()), static_cast<double>(geom.first.y()));
                    Point2D end(static_cast<double>(geom.second.x()), static_cast<double>(geom.second.y()));
                    return matchesWarehouse(*contours, start, end);
                }
                else if constexpr (std::is_same_v<T, rises::geofence::Point>) {
                    // Point: check proximity to any contour edge
                    Point2D pt(static_cast<double>(geom.x()), static_cast<double>(geom.y()));
                    return isPointNearContourEdge(*contours, pt, static_cast<double>(position_tolerance_));
                }
                else if constexpr (std::is_same_v<T, rises::geofence::Rectangle>) {
                    // Rectangle: check each of the 4 edges
                    const auto& min_c = geom.min_corner();
                    const auto& max_c = geom.max_corner();
                    Point2D p1(min_c.x(), min_c.y()), p2(max_c.x(), min_c.y());
                    Point2D p3(max_c.x(), max_c.y()), p4(min_c.x(), max_c.y());
                    return matchesWarehouse(*contours, p1, p2) ||
                           matchesWarehouse(*contours, p2, p3) ||
                           matchesWarehouse(*contours, p3, p4) ||
                           matchesWarehouse(*contours, p4, p1);
                }
                else if constexpr (std::is_same_v<T, rises::geofence::Polygon>) {
                    // Polygon: check each edge
                    const auto& verts = geom.outer();
                    for (std::size_t i = 0; i + 1 < verts.size(); ++i) {
                        Point2D start(verts[i].x(), verts[i].y());
                        Point2D end(verts[i + 1].x(), verts[i + 1].y());
                        if (matchesWarehouse(*contours, start, end)) return true;
                    }
                    return false;
                }
                else if constexpr (std::is_same_v<T, rises::geofence::Circle>) {
                    // Circle: check center proximity to contour edges (expanded by radius)
                    Point2D pt(static_cast<double>(geom.center.x()), static_cast<double>(geom.center.y()));
                    return isPointNearContourEdge(*contours, pt, static_cast<double>(position_tolerance_ + geom.radius));
                }
                return false;
            }, detected_geom);

            if (warehouse_match) {
                result.type = MatchType::WAREHOUSE;
                result.obstacle_id = -1;
            }
        }
    }

    return result;
}

/**
 * @brief Check if detected obstacle geometry collides with map obstacle geometry
 *
 * Uses a simple collision/intersection check instead of edge-matching:
 * - Point types: distance-based proximity check (points have no area)
 * - All other types: Boost.Geometry intersects() (covers overlap, containment,
 *   and touching). Distance fallback for near-miss within tolerance.
 */
bool ObstacleMatchChecker::matchObstacleGeometries(
    const Geometry& detected_geom,
    const Geometry& map_geom,
    const float eps_pos,
    const float /*eps_angle*/) {

    // Point detected types need distance-based check (zero-area geometry)
    const bool detected_is_point = std::holds_alternative<rises::geofence::Point>(detected_geom);
    if (detected_is_point) {
        return rises::geofence::distance(detected_geom, map_geom) <= eps_pos;
    }

    // For all other geometry combinations: direct intersection test.
    // If geometries overlap or touch, it's a match.
    if (rises::geofence::intersects(detected_geom, map_geom)) {
        return true;
    }

    // Near-miss: geometries don't intersect but are within tolerance.
    return rises::geofence::distance(detected_geom, map_geom) <= eps_pos;
}

bool ObstacleMatchChecker::isPointNearContourEdge(
    const shape::MapBoundaryContours& contours,
    const Point2D& point,
    double tolerance) {

    const double tolerance_sq = tolerance * tolerance;

    // Squared distance from `point` to segment [a, b] (avoids sqrt)
    const auto distSq = [&](const Point2D& a, const Point2D& b) -> double {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len_sq = dx * dx + dy * dy;
        if (len_sq < 1e-12) {
            const double ex = point.x - a.x, ey = point.y - a.y;
            return ex * ex + ey * ey;
        }
        const double t = std::clamp(
            ((point.x - a.x) * dx + (point.y - a.y) * dy) / len_sq,
            0.0, 1.0
        );
        const double ex = point.x - (a.x + t * dx);
        const double ey = point.y - (a.y + t * dy);
        return ex * ex + ey * ey;
    };

    // Check polygon ring edges (i+1 < n loop + explicit closing edge, no modulo)
    const auto checkEdges = [&](const std::vector<Point2D>& verts) -> bool {
        const std::size_t n = verts.size();
        for (std::size_t i = 0; i + 1 < n; ++i) {
            if (distSq(verts[i], verts[i + 1]) < tolerance_sq) return true;
        }
        if (n >= 2 && distSq(verts[n - 1], verts[0]) < tolerance_sq) return true;
        return false;
    };

    // Check outer contour edges
    if (checkEdges(contours.getOuterContour().getVertices()))
        return true;

    // Check inner contour edges
    for (const rises::shape::PolygonContour& inner : contours.getInnerContours()) {
        if (checkEdges(inner.getVertices()))
            return true;
    }

    // Check outer segments (wall segments with gaps/doors)
    for (const rises::shape::LineSegment2D& seg : contours.getOuterSegments()) {
        if (distSq(seg.start, seg.end) < tolerance_sq) return true;
    }

    return false;
}

} // namespace rises::geofence
