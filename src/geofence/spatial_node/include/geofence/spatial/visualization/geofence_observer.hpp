#pragma once

#include <memory>
#include <vector>
#include <algorithm>

#include "nav_msgs/msg/path.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include "geofence/spatial/shape/contour.hpp"

namespace rises {
namespace geofence {

/**
 * @brief Observer interface for geofence visualization and other notifications
 * 
 * Classes implementing this interface will receive automatic notifications
 * when the geofence map changes, dynamic obstacles are detected, or path
 * validation results are available.
 */
class GeofenceObserver {
public:
    virtual ~GeofenceObserver() = default;
    
    /**
     * @brief Called when the geofence map structure changes
     * 
     * This includes areas or obstacles being added, removed, or modified
     * in the static geofence map.
     */
    virtual void onMapChanged() = 0;
    
    /**
     * @brief Called when dynamic obstacles are detected or updated
     * 
     * @param obstacle The dynamic obstacle that was detected
     */
    virtual void onDynamicObstacleUpdate(const rises_interfaces::msg::Obstacle& obstacle) = 0;
    
    /**
     * @brief Called when path validation results are available
     * 
     * @param path The path that was validated
     * @param is_safe Whether the path is considered safe
     */
    virtual void onPathValidationUpdate(const nav_msgs::msg::Path& path, bool is_safe) = 0;
    
    /**
     * @brief Called when obstacle correspondence check is complete
     * 
     * @param obstacle The detected obstacle
     * @param matched Whether it matched a known map obstacle
     */
    virtual void onObstacleCorrespondence(const rises_interfaces::msg::Obstacle& obstacle, bool matched) = 0;
    
    /**
     * @brief Called when map boundary contours are updated
     * 
     * @param contours The new map boundary contours
     */
    virtual void onMapBoundaryUpdate(const rises::shape::MapBoundaryContours& contours) = 0;
};

/**
 * @brief Base class for components that can notify observers
 * 
 * This implements the Subject part of the Observer pattern.
 * Classes that want to notify observers should inherit from this.
 */
class GeofenceSubject {
public:
    virtual ~GeofenceSubject() = default;
    
    /**
     * @brief Add an observer to receive notifications
     * 
     * @param observer Observer to add (weak reference stored to avoid cycles)
     */
    void addObserver(std::shared_ptr<GeofenceObserver> observer) {
        this->observers_.push_back(observer);
    }
    
    /**
     * @brief Remove an observer from notifications
     * 
     * @param observer Observer to remove
     */
    void removeObserver(std::shared_ptr<GeofenceObserver> observer) {
        this->observers_.erase(
            std::remove_if(this->observers_.begin(), this->observers_.end(),
                [&observer](const std::weak_ptr<GeofenceObserver>& weak_obs) {
                    return weak_obs.lock() == observer;
                }), 
            this->observers_.end()
        );
    }
    
    /**
     * @brief Get the number of active observers
     * 
     * @return Number of observers still alive
     */
    std::size_t getObserverCount() const {
        std::size_t count = 0;
        for (const auto& obs : this->observers_) {
            if (obs.lock()) {
                ++count;
            }
        }
        return count;
    }
    
    /**
     * @brief Notify all observers that the map has changed
     */
    void notifyMapChanged() {
        this->cleanupDeadObservers();
        for (const auto& obs : this->observers_) {
            if (auto observer = obs.lock()) {
                observer->onMapChanged();
            }
        }
    }
    
    /**
     * @brief Notify all observers about a dynamic obstacle
     * 
     * @param obstacle The dynamic obstacle detected
     */
    void notifyDynamicObstacle(const rises_interfaces::msg::Obstacle& obstacle) {
        this->cleanupDeadObservers();
        for (const auto& obs : this->observers_) {
            if (auto observer = obs.lock()) {
                observer->onDynamicObstacleUpdate(obstacle);
            }
        }
    }
    
    /**
     * @brief Notify all observers about path validation results
     * 
     * @param path The path that was validated
     * @param is_safe Whether the path is safe
     */
    void notifyPathValidation(const nav_msgs::msg::Path& path, bool is_safe) {
        this->cleanupDeadObservers();
        for (const auto& obs : this->observers_) {
            if (auto observer = obs.lock()) {
                observer->onPathValidationUpdate(path, is_safe);
            }
        }
    }
    
    /**
     * @brief Notify all observers about obstacle correspondence results
     * 
     * @param obstacle The obstacle that was checked
     * @param matched Whether it matched a known obstacle
     */
    void notifyObstacleCorrespondence(const rises_interfaces::msg::Obstacle& obstacle, bool matched) {
        this->cleanupDeadObservers();
        for (const auto& obs : this->observers_) {
            if (auto observer = obs.lock()) {
                observer->onObstacleCorrespondence(obstacle, matched);
            }
        }
    }
    
    /**
     * @brief Notify all observers about map boundary contours updates
     * 
     * @param contours The new map boundary contours
     */
    void notifyMapBoundaryUpdate(const rises::shape::MapBoundaryContours& contours) {
        this->cleanupDeadObservers();
        for (const auto& obs : this->observers_) {
            if (auto observer = obs.lock()) {
                observer->onMapBoundaryUpdate(contours);
            }
        }
    }

protected:

private:
    std::vector<std::weak_ptr<GeofenceObserver>> observers_;
    
    /**
     * @brief Remove dead weak_ptr entries from observers list
     */
    void cleanupDeadObservers() {
        this->observers_.erase(
            std::remove_if(this->observers_.begin(), this->observers_.end(),
                [](const std::weak_ptr<GeofenceObserver>& obs) {
                    return obs.expired();
                }),
            this->observers_.end()
        );
    }
};

} // namespace geofence
} // namespace rises