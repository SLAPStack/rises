#pragma once

#include "geofence/common/policies/coordinate_transform.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <vector>

namespace rises::geofence::utils
{

inline std::vector<rises_interfaces::msg::Obstacle> JsonLoader::loadObstacles(
    const std::string& filepath,
    bool apply_transform)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open obstacles file: " + filepath);
    }

    try {
        nlohmann::json j;
        file >> j;

        std::vector<nlohmann::json> items;
        if (j.is_array()) {
            items = j.get<std::vector<nlohmann::json>>();
        } else if (j.contains("pallets") && j["pallets"].is_array()) {
            items = j["pallets"].get<std::vector<nlohmann::json>>();
        } else {
            throw std::runtime_error("Invalid obstacles JSON format: expected array or {pallets: [...]}");
        }

        std::vector<rises_interfaces::msg::Obstacle> obstacles;
        obstacles.reserve(items.size());

        for (const nlohmann::json& item : items) {
            if (!item.contains("id") || !item.contains("aabb")) {
                continue;
            }

            const int64_t id = item["id"];
            const nlohmann::json& aabb = item["aabb"];

            if (!aabb.is_array() || aabb.size() != 2) {
                continue;
            }

            const float x_min = aabb[0][0].get<float>();
            const float y_min = aabb[0][1].get<float>();
            const float x_max = aabb[1][0].get<float>();
            const float y_max = aabb[1][1].get<float>();

            const float center_x = (x_min + x_max) / 2.0f;
            const float center_y = (y_min + y_max) / 2.0f;
            const float width = std::abs(x_max - x_min);
            const float height = std::abs(y_max - y_min);

            rises_interfaces::msg::Obstacle obs_msg;
            obs_msg.id = id;
            obs_msg.type = rises_interfaces::msg::Obstacle::RECTANGLE;
            obs_msg.position.x = center_x;
            obs_msg.position.y = center_y;
            obs_msg.position.z = 0.0f;
            obs_msg.width = width;
            obs_msg.height = height;
            obs_msg.orientation = 0.0f;

            if (apply_transform) {
                rises::geofence::CoordinateTransform::transformObstacle(obs_msg);
            }

            obstacles.push_back(obs_msg);
        }

        return obstacles;

    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing obstacles JSON: " + std::string(e.what()));
    }
}

inline rises::shape::MapBoundaryContours JsonLoader::loadContours(
    const std::string& filepath,
    bool apply_transform)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open contours file: " + filepath);
    }

    try {
        nlohmann::json j;
        file >> j;

        if (!j.contains("outer_contour_hull") || !j.contains("inner_contours")) {
            throw std::runtime_error("Invalid contours JSON format: missing outer_contour_hull or inner_contours");
        }

        // Load outer hull (for fast containment checks)
        std::vector<Point2D> outer_vertices;
        const nlohmann::json& hull = j["outer_contour_hull"];
        for (const nlohmann::json& point : hull) {
            if (point.is_array() && point.size() >= 2) {
                double x = static_cast<double>(point[0].get<float>());
                double y = static_cast<double>(point[1].get<float>());
                double z = 0.0;

                if (apply_transform) {
                    rises::geofence::CoordinateTransform::transformBoundaryPoint(x, y, z);
                }

                outer_vertices.push_back({x, y});
            }
        }

        // Load outer segments (actual walls with potential gaps/doors)
        std::vector<shape::LineSegment2D> outer_segments;
        if (j.contains("outer_contour_segments")) {
            const nlohmann::json& segments = j["outer_contour_segments"];
            for (const nlohmann::json& seg : segments) {
                if (!seg.is_array() || seg.size() != 2) continue;
                if (!seg[0].is_array() || seg[0].size() < 2) continue;
                if (!seg[1].is_array() || seg[1].size() < 2) continue;

                double x1 = static_cast<double>(seg[0][0].get<float>());
                double y1 = static_cast<double>(seg[0][1].get<float>());
                double z1 = 0.0;

                double x2 = static_cast<double>(seg[1][0].get<float>());
                double y2 = static_cast<double>(seg[1][1].get<float>());
                double z2 = 0.0;

                if (apply_transform) {
                    rises::geofence::CoordinateTransform::transformBoundaryPoint(x1, y1, z1);
                    rises::geofence::CoordinateTransform::transformBoundaryPoint(x2, y2, z2);
                }

                outer_segments.emplace_back(Point2D{x1, y1}, Point2D{x2, y2});
            }
        }

        // Load inner contours (holes)
        std::vector<shape::PolygonContour> inner_polygons;
        const nlohmann::json& inner_contours = j["inner_contours"];
        for (const nlohmann::json& inner : inner_contours) {
            if (!inner.is_array()) continue;

            std::vector<Point2D> hole_vertices;
            for (const nlohmann::json& segment : inner) {
                if (!segment.is_array() || segment.size() != 2) continue;
                
                double x1 = static_cast<double>(segment[0][0].get<float>());
                double y1 = static_cast<double>(segment[0][1].get<float>());
                double z1 = 0.0;
                
                if (apply_transform) {
                    rises::geofence::CoordinateTransform::transformBoundaryPoint(x1, y1, z1);
                }
                
                hole_vertices.push_back({x1, y1});
            }
            
            if (!inner.empty()) {
                const nlohmann::json& last_segment = inner.back();
                
                double x2 = static_cast<double>(last_segment[1][0].get<float>());
                double y2 = static_cast<double>(last_segment[1][1].get<float>());
                double z2 = 0.0;
                
                if (apply_transform) {
                    rises::geofence::CoordinateTransform::transformBoundaryPoint(x2, y2, z2);
                }
                
                hole_vertices.push_back({x2, y2});
            }

            if (hole_vertices.size() >= 3) {
                inner_polygons.emplace_back(hole_vertices);
            }
        }

        const shape::PolygonContour outer_poly(outer_vertices);
        return rises::shape::MapBoundaryContours(outer_poly, inner_polygons, outer_segments);

    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing contours JSON: " + std::string(e.what()));
    }
}

} // namespace rises::geofence::utils
