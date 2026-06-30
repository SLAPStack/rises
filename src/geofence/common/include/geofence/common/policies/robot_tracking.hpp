#pragma once

#include "rises_interfaces/msg/obstacle.hpp"
#include <memory>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <mutex>

namespace rises {
namespace geofence {
namespace policies {

// Forward declarations
struct RobotPose;

/**
 * @brief Robot footprint representation for collision checking
 */
struct RobotFootprint {
    enum class Type {
        CIRCLE,
        RECTANGLE,
        POLYGON
    };
    
    Type type;
    
    // Circle parameters
    double radius = 0.0;
    
    // Rectangle parameters (axis-aligned)
    double width = 0.0;
    double height = 0.0;
    
    // Polygon parameters
    struct Point2D {
        double x;
        double y;
    };
    std::vector<Point2D> vertices;
    
    // Margin to add to footprint (safety buffer)
    double margin = 0.0;
    
    /**
     * @brief Create circle footprint
     */
    [[nodiscard]] static RobotFootprint createCircle(double radius, double margin = 0.0) {
        RobotFootprint fp;
        fp.type = Type::CIRCLE;
        fp.radius = radius;
        fp.margin = margin;
        return fp;
    }
    
    /**
     * @brief Create rectangle footprint
     */
    [[nodiscard]] static RobotFootprint createRectangle(double width, double height, double margin = 0.0) {
        RobotFootprint fp;
        fp.type = Type::RECTANGLE;
        fp.width = width;
        fp.height = height;
        fp.margin = margin;
        return fp;
    }
    
    /**
     * @brief Create polygon footprint
     */
    [[nodiscard]] static RobotFootprint createPolygon(const std::vector<Point2D>& vertices, double margin = 0.0) {
        RobotFootprint fp;
        fp.type = Type::POLYGON;
        fp.vertices = vertices;
        fp.margin = margin;
        return fp;
    }
    
    /**
     * @brief Check if obstacle is contained within this robot footprint at given pose
     * @param obstacle Obstacle to check
     * @param pose Robot pose (position and orientation)  
     * @param expansion_margin Additional margin to expand footprint
     * @return true if obstacle intersects with robot footprint
     */
    [[nodiscard]] bool containsObstacle(const rises_interfaces::msg::Obstacle& obstacle, 
                         const RobotPose& pose,
                         double expansion_margin = 0.0) const;
};

/**
 * @brief Robot position with timestamp
 */
struct RobotPose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;  // Orientation in radians
    uint64_t timestamp_ns = 0;  // Nanoseconds since epoch
    
    [[nodiscard]] bool isValid() const {
        return timestamp_ns > 0;
    }
};

/**
 * @brief Thread-safe tracker for multiple robot positions
 * 
 * Maintains current pose for each robot and provides obstacle filtering
 * based on robot footprints.
 */
class RobotTracker {
public:
    /**
     * @brief Update robot position
     * @param robot_id Unique robot identifier
     * @param pose Current robot pose
     */
    void updateRobotPose(const std::string& robot_id, const RobotPose& pose);
    
    /**
     * @brief Set robot footprint
     * @param robot_id Unique robot identifier
     * @param footprint Robot footprint definition
     */
    void setRobotFootprint(const std::string& robot_id, const RobotFootprint& footprint);
    
    /**
     * @brief Check if obstacle overlaps with any known robot
     * 
     * @param obstacle Detected obstacle to check
     * @param max_age_ms Maximum age of robot poses to consider (ms)
     * @return true if obstacle matches a known robot position
     */
    [[nodiscard]] bool isObstacleRobot(const rises_interfaces::msg::Obstacle& obstacle, 
                        uint64_t max_age_ms = 1000) const;
    
    /**
     * @brief Remove robot from tracker
     */
    void removeRobot(const std::string& robot_id);
    
    /**
     * @brief Clear all robots
     */
    void clear();
    
    /**
     * @brief Get number of tracked robots
     */
    std::size_t getRobotCount() const;
    
private:
    struct RobotInfo {
        RobotPose pose;
        RobotFootprint footprint;
    };
    
    mutable std::mutex mutex_;
    std::unordered_map<std::string, RobotInfo> robots_;
    
    /**
     * @brief Check if obstacle overlaps with specific robot footprint
     */
    bool checkOverlap(const rises_interfaces::msg::Obstacle& obstacle,
                     const RobotInfo& robot) const;
};

/**
 * @brief Policy class for robot tracking and filtering
 * 
 * Provides static methods for tracking robot positions and filtering
 * robot-related obstacles from the map.
 * 
 * Uses compile-time polymorphism for zero-cost abstraction.
 */
class RobotTrackingPolicy {
public:
    using RobotTrackerPtr = std::shared_ptr<RobotTracker>;
    
    /**
     * @brief Initialize robot tracking with tracker instance
     * @param tracker Shared pointer to robot tracker
     */
    static void initialize(RobotTrackerPtr tracker) {
        tracker_ = tracker;
    }
    
    /**
     * @brief Update robot position
     * @param robot_id Unique robot identifier
     * @param pose Current robot pose
     */
    static void updateRobotPose(const std::string& robot_id, 
                               const RobotPose& pose) {
        if (tracker_) {
            tracker_->updateRobotPose(robot_id, pose);
        }
    }
    
    /**
     * @brief Set robot footprint
     * @param robot_id Unique robot identifier
     * @param footprint Robot footprint definition
     */
    static void setRobotFootprint(const std::string& robot_id, 
                                 const RobotFootprint& footprint) {
        if (tracker_) {
            tracker_->setRobotFootprint(robot_id, footprint);
        }
    }
    
    /**
     * @brief Check if obstacle should be filtered (is a robot)
     * @param obstacle Obstacle to check
     * @param max_age_ms Maximum age of robot poses to consider (ms)
     * @return true if obstacle matches a known robot
     */
    static bool shouldFilterObstacle(const rises_interfaces::msg::Obstacle& obstacle,
                                    uint64_t max_age_ms = 1000) {
        if (tracker_) {
            return tracker_->isObstacleRobot(obstacle, max_age_ms);
        }
        return false;
    }
    
    /**
     * @brief Filter robot obstacles from obstacle array
     * @param obstacles Input obstacles
     * @param max_age_ms Maximum age of robot poses to consider (ms)
     * @return Filtered obstacles (robots removed)
     */
    static std::vector<rises_interfaces::msg::Obstacle> filterRobots(
        const std::vector<rises_interfaces::msg::Obstacle>& obstacles,
        uint64_t max_age_ms = 1000) {
        
        if (!tracker_) {
            return obstacles;
        }
        
        std::vector<rises_interfaces::msg::Obstacle> filtered;
        filtered.reserve(obstacles.size());
        
        for (const auto& obstacle : obstacles) {
            if (!tracker_->isObstacleRobot(obstacle, max_age_ms)) {
                filtered.push_back(obstacle);
            }
        }
        
        return filtered;
    }
    
    /**
     * @brief Remove robot from tracker
     * @param robot_id Robot identifier
     */
    static void removeRobot(const std::string& robot_id) {
        if (tracker_) {
            tracker_->removeRobot(robot_id);
        }
    }
    
    /**
     * @brief Clear all tracked robots
     */
    static void clear() {
        if (tracker_) {
            tracker_->clear();
        }
    }
    
    /**
     * @brief Get number of tracked robots
     * @return Robot count
     */
    static std::size_t getRobotCount() {
        if (tracker_) {
            return tracker_->getRobotCount();
        }
        return 0;
    }

private:
    static RobotTrackerPtr tracker_;
};

/**
 * @brief No-op robot tracking policy (disabled robot filtering)
 * 
 * Provides same interface but all methods are empty or return defaults.
 * Compiler will optimize away all calls (zero cost).
 */
class NoRobotTrackingPolicy {
public:
    using RobotTrackerPtr = std::shared_ptr<RobotTracker>;
    
    static void initialize(RobotTrackerPtr) {}
    static void updateRobotPose(const std::string&, const RobotPose&) {}
    static void setRobotFootprint(const std::string&, const RobotFootprint&) {}
    static bool shouldFilterObstacle(const rises_interfaces::msg::Obstacle&, uint64_t = 1000) { return false; }
    static std::vector<rises_interfaces::msg::Obstacle> filterRobots(
        const std::vector<rises_interfaces::msg::Obstacle>& obstacles, uint64_t = 1000) {
        return obstacles;
    }
    static void removeRobot(const std::string&) {}
    static void clear() {}
    static std::size_t getRobotCount() { return 0; }
};

} // namespace policies
} // namespace geofence
} // namespace rises
