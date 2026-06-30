#include "geofence/common/policies/robot_tracking.hpp"
#include "geofence/utils/geometry_intersection.hpp"
#include <chrono>
#include <cmath>

namespace rises {
namespace geofence {
namespace policies {

// Initialize static member
RobotTrackingPolicy::RobotTrackerPtr RobotTrackingPolicy::tracker_ = nullptr;

// ============================================================================
// RobotFootprint implementation
// ============================================================================

bool RobotFootprint::containsObstacle(const rises_interfaces::msg::Obstacle& obstacle, 
                                     const RobotPose& pose,
                                     const double expansion_margin) const {
    const double total_margin = this->margin + expansion_margin;
    
    // Use conservative circumscribed circle approach for all footprint types
    // Faster than precise geometry checks, acceptable for inter-robot collision avoidance
    switch (this->type) {
        case Type::CIRCLE: {
            const double effective_radius = this->radius + total_margin;
            return utils::GeometryIntersection::intersectsCircle(
                obstacle, pose.x, pose.y, effective_radius);
        }
        
        case Type::RECTANGLE: {
            // Conservative approximation: use circumscribed circle around rectangle
            // Slightly over-estimates collision area, but ensures safety and improves performance
            const double diag = std::sqrt(this->width * this->width + this->height * this->height) * 0.5;
            const double effective_radius = diag + total_margin;
            return utils::GeometryIntersection::intersectsCircle(
                obstacle, pose.x, pose.y, effective_radius);
        }
        
        case Type::POLYGON: {
            // Conservative approximation: find maximum vertex distance for circumscribed circle
            // Faster than precise polygon-obstacle intersection tests
            double max_dist_sq = 0.0;
            for (const Point2D& vertex : this->vertices) {
                const double dx = vertex.x;
                const double dy = vertex.y;
                const double dist_sq = dx * dx + dy * dy;
                max_dist_sq = std::max(max_dist_sq, dist_sq);
            }
            const double effective_radius = std::sqrt(max_dist_sq) + total_margin;
            return utils::GeometryIntersection::intersectsCircle(
                obstacle, pose.x, pose.y, effective_radius);
        }
    }
    
    return false;
}

// ============================================================================
// RobotTracker implementation
// ============================================================================

void RobotTracker::updateRobotPose(const std::string& robot_id, const RobotPose& pose) {
    // Thread-safe pose update for multi-robot tracking
    const std::lock_guard<std::mutex> lock(this->mutex_);
    this->robots_[robot_id].pose = pose;
}

void RobotTracker::setRobotFootprint(const std::string& robot_id, const RobotFootprint& footprint) {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    this->robots_[robot_id].footprint = footprint;
}

void RobotTracker::removeRobot(const std::string& robot_id) {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    this->robots_.erase(robot_id);
}

void RobotTracker::clear() {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    this->robots_.clear();
}

std::size_t RobotTracker::getRobotCount() const {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    return this->robots_.size();
}

// Determines if detected obstacle corresponds to a tracked robot
bool RobotTracker::isObstacleRobot(const rises_interfaces::msg::Obstacle& obstacle, 
                                   const uint64_t max_age_ms) const {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    
    const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
    
    const uint64_t max_age_ns = max_age_ms * 1000000ULL;
    
    for (const std::pair<const std::string, RobotInfo>& robot_pair : this->robots_) {
        const RobotInfo& robot_info = robot_pair.second;
        
        if (!robot_info.pose.isValid()) continue;
        if (now_ns - robot_info.pose.timestamp_ns > max_age_ns) continue;
        
        if (this->checkOverlap(obstacle, robot_info)) {
            return true;
        }
    }
    
    return false;
}

bool RobotTracker::checkOverlap(const rises_interfaces::msg::Obstacle& obstacle,
                                const RobotInfo& robot) const {
    const RobotPose& pose = robot.pose;
    const RobotFootprint& footprint = robot.footprint;
    
    double effective_radius = 0.0;
    
    switch (footprint.type) {
        case RobotFootprint::Type::CIRCLE: {
            effective_radius = footprint.radius + footprint.margin;
            
            const bool intersects = utils::GeometryIntersection::intersectsCircle(
                obstacle, pose.x, pose.y, effective_radius);
            
            return intersects;
        }
        
        case RobotFootprint::Type::RECTANGLE: {
            // Conservative approximation using circumscribed circle
            // Trades precision for performance - acceptable for robot filtering
            const double diag = std::sqrt(footprint.width * footprint.width + 
                                   footprint.height * footprint.height) * 0.5;
            effective_radius = diag + footprint.margin;
            
            // Conservative overlap test: may over-estimate but guarantees safety
            const bool intersects = utils::GeometryIntersection::intersectsCircle(
                obstacle, pose.x, pose.y, effective_radius);
            
            return intersects;
        }
        
        case RobotFootprint::Type::POLYGON: {
            // Conservative approximation: circumscribed circle around polygon
            // Finds maximum vertex distance from center
            double max_dist_sq = 0.0;
            for (const RobotFootprint::Point2D& v : footprint.vertices) {
                const double dx = v.x;
                const double dy = v.y;
                const double dist_sq = dx * dx + dy * dy;
                max_dist_sq = std::max(max_dist_sq, dist_sq);
            }
            effective_radius = std::sqrt(max_dist_sq) + footprint.margin;
            
            // Conservative check using circumscribed circle
            const bool intersects = utils::GeometryIntersection::intersectsCircle(
                obstacle, pose.x, pose.y, effective_radius);
            
            return intersects;
        }
    }
    
    return false;
}

} // namespace policies
} // namespace geofence
} // namespace rises
