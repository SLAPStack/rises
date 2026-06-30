/**
 * @file obstacle_correspondence_matcher.cpp
 * @brief Correspondence matching implementation for data association
 * 
 * This file implements "is part of" matching between detected obstacles
 * and known map obstacles. Unlike collision detection (geometric intersection),
 * correspondence matching checks if a detected primitive is contained within
 * or on the boundary of a larger known obstacle.
 * 
 * Matching Examples:
 * - Point on rectangle edge → match
 * - Line segment within larger line → match
 * - Line on polygon edge → match
 * 
 * Used for:
 * - Data association: Identifying which map obstacle a detection came from
 * - Sensor validation: Verifying detections against known geometry
 * - Unknown detection: Identifying readings that don't match anything
 * 
 * @see ObstacleCorrespondenceMatcher class for API documentation
 */

#include "geofence/spatial/queries/obstacle_correspondence_matcher.hpp"
#include "geofence/utils/geometry_intersection.hpp"
#include "geofence/utils/compiler_hints.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include <cmath>

namespace rises::geofence {

void ObstacleCorrespondenceMatcher::initialize(const Config& config) {
    position_tolerance_ = config.position_tolerance;
    size_tolerance_ = config.size_tolerance;
    angle_tolerance_ = config.angle_tolerance;
    point_on_line_tolerance_ = config.point_on_line_tolerance;
    line_containment_tolerance_ = config.line_containment_tolerance;
}

ObstacleCorrespondenceMatcher::MatchResult 
ObstacleCorrespondenceMatcher::findCorrespondingObstacle(
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
    // Dynamic obstacles shouldn't be reference features for matching
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

bool ObstacleCorrespondenceMatcher::obstaclesMatch(
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

bool ObstacleCorrespondenceMatcher::isPointOnRectangleEdge(
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
    
    const bool on_left_edge = (std::abs(point_x - min_x) < tol) && 
                              (point_y >= min_y - tol) && (point_y <= max_y + tol);
    const bool on_right_edge = (std::abs(point_x - max_x) < tol) && 
                               (point_y >= min_y - tol) && (point_y <= max_y + tol);
    const bool on_bottom_edge = (std::abs(point_y - min_y) < tol) && 
                                (point_x >= min_x - tol) && (point_x <= max_x + tol);
    const bool on_top_edge = (std::abs(point_y - max_y) < tol) && 
                             (point_x >= min_x - tol) && (point_x <= max_x + tol);
    
    return on_left_edge || on_right_edge || on_bottom_edge || on_top_edge;
}

bool ObstacleCorrespondenceMatcher::isPointOnPolygonEdge(
    const Point2D& point,
    const rises::geofence::Polygon& polygon) {
    
    const auto& verts = polygon.outer();
    if (verts.empty()) return false;
    
    // Check if point lies on any edge of the polygon
    for (std::size_t i = 0; i < verts.size(); ++i) {
        const std::size_t next_i = (i + 1) % verts.size();
        const rises::geofence::Line edge(verts[i], verts[next_i]);
        
        if (isPointOnLineSegment(point, edge)) {
            return true;
        }
    }
    
    return false;
}

bool ObstacleCorrespondenceMatcher::isLineOnRectangleEdge(
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

bool ObstacleCorrespondenceMatcher::isLineOnPolygonEdge(
    const rises::geofence::Line& line,
    const rises::geofence::Polygon& polygon) {
    
    const auto& verts = polygon.outer();
    if (verts.empty()) return false;
    
    // Check if line overlaps any edge of the polygon
    for (std::size_t i = 0; i < verts.size(); ++i) {
        const std::size_t next_i = (i + 1) % verts.size();
        const rises::geofence::Line poly_edge(verts[i], verts[next_i]);
        
        // Check if detected line is contained within this polygon edge
        if (isLineContainedInLine(line, poly_edge)) {
            return true;
        }
    }
    
    return false;
}

bool ObstacleCorrespondenceMatcher::isLineContainedInLine(
    const rises::geofence::Line& smaller_line,
    const rises::geofence::Line& larger_line) {
    
    // Check if both endpoints of smaller line lie on the larger line
    const Point2D start{smaller_line.first.x(), smaller_line.first.y()};
    const Point2D end{smaller_line.second.x(), smaller_line.second.y()};
    
    return isPointOnLineSegment(start, larger_line) && 
           isPointOnLineSegment(end, larger_line);
}

bool ObstacleCorrespondenceMatcher::isPointOnLineSegment(
    const Point2D& point,
    const rises::geofence::Line& line) {
    
    const float tol = line_containment_tolerance_;
    const float point_x = point.x;
    const float point_y = point.y;
    const float line_start_x = line.first.x();
    const float line_start_y = line.first.y();
    const float line_end_x = line.second.x();
    const float line_end_y = line.second.y();

    // Vector from line start to point
    const float dx1 = point_x - line_start_x;
    const float dy1 = point_y - line_start_y;

    // Vector from line start to end
    const float dx2 = line_end_x - line_start_x;
    const float dy2 = line_end_y - line_start_y;

    // Check if point is collinear with line (cross product should be ~0)
    const float cross = dx1 * dy2 - dy1 * dx2;
    if (std::abs(cross) > tol) return false;

    // Check if point lies within the line segment bounds
    const float dot = dx1 * dx2 + dy1 * dy2;
    const float len_sq = dx2 * dx2 + dy2 * dy2;

    constexpr float MIN_LINE_LENGTH_SQ = 1e-6f;
    if (len_sq < MIN_LINE_LENGTH_SQ) return false;

    const float param = dot / len_sq;
    return (param >= -tol) && (param <= 1.0f + tol);
}

float ObstacleCorrespondenceMatcher::computeMatchConfidence(
    const Geometry& map_obstacle,
    const rises_interfaces::msg::Obstacle& detected) {
    
    float confidence = 0.0f;
    
    // Component 1: Position accuracy (weight: 0.5)
    const Point2D map_pos = getPosition(map_obstacle);
    const float dx = map_pos.x - detected.position.x;
    const float dy = map_pos.y - detected.position.y;
    const float dist = std::sqrt(dx*dx + dy*dy);
    const float position_tolerance = position_tolerance_;
    const float pos_score = std::max(0.0f, 1.0f - (dist / position_tolerance));
    confidence += pos_score * 0.5f;
    
    // Component 2: Type exactness (weight: 0.3)
    const GeometryType map_type = getType(map_obstacle);
    const bool exact_type_match = 
        (map_type == GeometryType::Point && detected.type == rises_interfaces::msg::Obstacle::POINT) ||
        (map_type == GeometryType::Line && detected.type == rises_interfaces::msg::Obstacle::LINE) ||
        (map_type == GeometryType::Rectangle && detected.type == rises_interfaces::msg::Obstacle::RECTANGLE) ||
        (map_type == GeometryType::Polygon && detected.type == rises_interfaces::msg::Obstacle::POLYGON);
    confidence += (exact_type_match ? 1.0f : 0.5f) * 0.3f;
    
    // Component 3: Size similarity (weight: 0.2)
    // Simplified for now - full implementation would compare all dimensions
    confidence += 0.2f;  // Assume reasonable match if we got here
    
    return confidence;
}

} // namespace rises::geofence
