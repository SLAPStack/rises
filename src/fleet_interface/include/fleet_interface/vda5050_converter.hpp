#pragma once

#include <rclcpp/time.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "rises_interfaces/msg/obstacle_report.hpp"
#include <string>

namespace rises {

struct AgvPosition {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;  // radians
    bool valid = false;
};

/**
 * Stateless, thread-safe converter between VDA5050 protocol messages (JSON-encoded)
 * and ROS2 typed messages used within the Rises system.
 *
 * VDA5050 is the communication interface standard for AGV systems (VDA 5050:2020).
 * All methods are static; this class is not instantiable and carries no state.
 * Errors during parsing are logged to the named logger "vda5050_converter".
 */
class Vda5050Converter
{
public:
    Vda5050Converter() = delete;

    /**
     * Convert a VDA5050 "order" message to a ROS2 nav_msgs/Path.
     */
    static nav_msgs::msg::Path orderToPath(
        const std::string& json_str,
        const std::string& frame_id,
        const rclcpp::Time& stamp);

    /**
     * Convert an ObstacleReport to a VDA5050 state JSON fragment.
     * Includes agvPosition if provided and valid.
     */
    static std::string obstacleReportToStateJson(
        const rises_interfaces::msg::ObstacleReport& report,
        const AgvPosition& position = AgvPosition{});

    /**
     * Generate a minimal VDA5050 state JSON with only safetyState (fieldViolation=true)
     * and agvPosition. Used for immediate alert on first unmatched obstacle detection.
     */
    static std::string alertStateJson(const AgvPosition& position = AgvPosition{});
};

} // namespace rises
