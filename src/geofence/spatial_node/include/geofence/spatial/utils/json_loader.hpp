#pragma once

#include "rises_interfaces/msg/obstacle.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include <string>
#include <vector>

namespace rises::shape { class MapBoundaryContours; }

namespace rises::geofence::utils
{

/**
 * @brief Static utility for loading geofencing data from JSON files
 * 
 * Pure parser utility - returns data without side effects.
 * Caller is responsible for inserting into map and visualizer.
 */
class JsonLoader
{
public:
    /**
     * @brief Parse obstacles from JSON file
     * 
     * Supports two JSON formats:
     * 1. Array format: [{id, aabb}, {id, aabb}, ...]
     * 2. Object format: {pallets: [{id, aabb}, ...]}
     * 
     * AABB format: [[x_min, y_min], [x_max, y_max]]
     * 
     * @param filepath Path to JSON file containing obstacle definitions
     * @param apply_transform Whether to apply coordinate transformations
     * @return Vector of obstacle messages ready for insertion
     * @throws std::runtime_error on file/parse errors
     */
    static std::vector<rises_interfaces::msg::Obstacle> loadObstacles(
        const std::string& filepath,
        bool apply_transform = true);

    /**
     * @brief Parse map boundary contours from JSON file
     * 
     * Expected JSON format:
     * {
     *   "outer_contour_hull": [[x1, y1], [x2, y2], ...],
     *   "inner_contours": [
     *     [[[x1, y1], [x2, y2]], [[x2, y2], [x3, y3]], ...],
     *     ...
     *   ]
     * }
     * 
     * @param filepath Path to JSON file containing contour definitions
     * @param apply_transform Whether to apply coordinate transformations
     * @return MapBoundaryContours object ready for insertion
     * @throws std::runtime_error on file/parse errors
     */
    static rises::shape::MapBoundaryContours loadContours(
        const std::string& filepath,
        bool apply_transform = true);

    /**
     * @brief Convert obstacle message to geometry
     * 
     * Useful for testing or pre-processing obstacles before insertion
     * 
     * @param obstacle_msg ROS obstacle message
     * @param apply_transform Whether to apply coordinate transformations
     * @return Converted geometry object
     */
    static rises::geofence::Geometry obstacleToGeometry(
        rises_interfaces::msg::Obstacle obstacle_msg,
        bool apply_transform = true);
};

} // namespace rises::geofence::utils

// Include implementation
#include "json_loader_impl.hpp"
