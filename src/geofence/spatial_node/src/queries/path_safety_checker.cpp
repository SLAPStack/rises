#include "geofence/spatial/queries/path_safety_checker.hpp"
#include "spatial_index_selection.hpp"
#include "geofence/spatial/common/bounding_box.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include <cmath>
#include <stdexcept>

namespace rises {
namespace geofence {

void PathSafetyChecker::initialize(const Config& config) {
    safety_margin_ = config.safety_margin;
    check_map_bounds_ = config.check_map_bounds;
    check_locked_areas_ = config.check_locked_areas;
    min_path_poses_ = config.min_path_poses;
    max_path_poses_ = config.max_path_poses;
}

bool PathSafetyChecker::isPathSafe(
    const GeofenceMap& map,
    const nav_msgs::msg::Path& path) {
    
    validatePathSize(path.poses.size());
    
    // Validate each segment between consecutive poses
    // Note: Path must be pre-sampled at appropriate resolution by the planner
    for (std::size_t i = 0; i < path.poses.size() - 1; ++i) {
        const geometry_msgs::msg::PoseStamped& current_pose = path.poses[i];
        const geometry_msgs::msg::PoseStamped& next_pose = path.poses[i + 1];
        
        const Point2D start{
            current_pose.pose.position.x,
            current_pose.pose.position.y
        };
        
        const Point2D end{
            next_pose.pose.position.x,
            next_pose.pose.position.y
        };
        
        if (!isSegmentSafe(map, start, end)) {
            return false;
        }
    }
    
    return true;
}

bool PathSafetyChecker::isSegmentSafe(
    const GeofenceMap& map,
    const Point2D& start, const Point2D& end) {
    
    // Phase 1: Map boundary validation (fast rejection)
    if (check_map_bounds_) {
        if (!BoundaryChecker::isSegmentInsideMapBounds(map, start, end)) {
            return false;
        }
    }
    
    // Phase 2: Collision detection - segment must not intersect any obstacles
    if (doesSegmentIntersectObstacle(map, start, end)) {
        return false;
    }
    
    // Phase 3: Clearance validation - ensure minimum safety margin at endpoints
    // Only checks endpoints, assumes path planner provided adequate sampling density
    const float clearance_start = map.distanceToNearestObstacle(start);
    if (clearance_start < safety_margin_) {
        return false;
    }
    
    const float clearance_end = map.distanceToNearestObstacle(end);
    if (clearance_end < safety_margin_) {
        return false;
    }
    
    // Phase 4: Locked area validation - areas restricted for navigation
    // (e.g., zones reserved for other robots, maintenance areas, etc.)
    if (check_locked_areas_) {
        if (doesSegmentIntersectLockedArea(map, start, end)) {
            return false;
        }
    }
    
    return true;
}

void PathSafetyChecker::validatePathSize(const std::size_t num_poses) {
    if (num_poses < min_path_poses_) {
        throw std::invalid_argument(
            "[validatePath] Path has " + std::to_string(num_poses) + " poses, minimum required: " + 
            std::to_string(min_path_poses_)
        );
    }
    
    if (num_poses > max_path_poses_) {
        throw std::invalid_argument(
            "[validatePath] Path has " + std::to_string(num_poses) + " poses, maximum allowed: " + 
            std::to_string(max_path_poses_)
        );
    }
}

bool PathSafetyChecker::doesSegmentIntersectObstacle(
    const GeofenceMap& map,
    const Point2D& start, const Point2D& end) {
    
    const BoundingBox segment_box{
        std::min(static_cast<float>(start.x), static_cast<float>(end.x)),
        std::min(static_cast<float>(start.y), static_cast<float>(end.y)),
        std::max(static_cast<float>(start.x), static_cast<float>(end.x)),
        std::max(static_cast<float>(start.y), static_cast<float>(end.y))
    };
    
    const Geometry segment_geom = Line(
        Point(start.x, start.y),
        Point(end.x, end.y)
    );
    
    // Path safety only checks FIXED and STATIC obstacles
    // Dynamic obstacles (robots, people) are handled by collision avoidance
    constexpr ObstacleLayer relevant_layers = ObstacleLayer::FIXED | ObstacleLayer::STATIC;
    
    const bool no_intersection = map.findFirstObstacleInRegion(segment_box, 
        [&segment_geom, relevant_layers](const GeometryEntry& entry) {
            if (!hasLayer(relevant_layers, entry.layer)) {
                return false;
            }
            return intersects(segment_geom, entry.geometry);
        });
    
    return !no_intersection;
}

bool PathSafetyChecker::doesSegmentIntersectLockedArea(
    const GeofenceMap& map,
    const Point2D& start, const Point2D& end) {
    
    // Query spatial index for potential locked area intersections
    const BoundingBox segment_box{
        std::min(static_cast<float>(start.x), static_cast<float>(end.x)),
        std::min(static_cast<float>(start.y), static_cast<float>(end.y)),
        std::max(static_cast<float>(start.x), static_cast<float>(end.x)),
        std::max(static_cast<float>(start.y), static_cast<float>(end.y))
    };
    
    const bool no_locked_intersection = map.findFirstAreaInRegion(segment_box,
        [&map, &start, &end](const int64_t area_id, const ::Rectangle& area) {
            if (!map.isAreaLocked(area_id)) {
                return false;
            }
            
            // Check if either endpoint is inside the locked area
            const bool start_inside = (start.x >= area.min.x && start.x <= area.max.x &&
                                       start.y >= area.min.y && start.y <= area.max.y);
            
            const bool end_inside = (end.x >= area.min.x && end.x <= area.max.x &&
                                     end.y >= area.min.y && end.y <= area.max.y);
            
            if (start_inside || end_inside) {
                return true;  // Stop - segment enters locked area
            }
            
            // Check if segment crosses any rectangle edge
            // Uses parametric line segment intersection test
            const auto segments_intersect = [](const Point2D& p1, const Point2D& p2,
                                               const Point2D& p3, const Point2D& p4) -> bool {
                const double x1 = p1.x, y1 = p1.y;
                const double x2 = p2.x, y2 = p2.y;
                const double x3 = p3.x, y3 = p3.y;
                const double x4 = p4.x, y4 = p4.y;
                
                const double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
                if (std::abs(denom) < 1e-10) {
                    return false;
                }
                
                const double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
                const double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
                
                return (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0);
            };
            
            const Point2D bottom_left{area.min.x, area.min.y};
            const Point2D bottom_right{area.max.x, area.min.y};
            const Point2D top_right{area.max.x, area.max.y};
            const Point2D top_left{area.min.x, area.max.y};
            
            if (segments_intersect(start, end, bottom_left, bottom_right) ||
                segments_intersect(start, end, bottom_right, top_right) ||
                segments_intersect(start, end, top_right, top_left) ||
                segments_intersect(start, end, top_left, bottom_left)) {
                return true;
            }
            
            return false;
        });
    
    return !no_locked_intersection;
}

} // namespace geofence
} // namespace rises
