#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>

#include "rises_interfaces/msg/obstacle_report.hpp"
#include "message_translator/vda5050_converter.hpp"

#include <mutex>
#include <string>
#include <memory>

namespace rises {

/**
 * Bridges geofence obstacle reports and alerts to VDA5050 state JSON,
 * publishing on both a ROS topic and an MQTT bridge topic.
 *
 * Maintains a cached state that is republished at a configurable rate
 * with an up-to-date robot position (from TF or a pose topic).
 * Obstacle reports and alerts trigger an immediate publish in addition
 * to the periodic timer.
 */
class Vda5050StateBridge
{
public:
    struct Config {
        std::string position_source = "tf";   // "tf" or "topic"
        double state_publish_rate = 1.0;      // Hz, 0 disables periodic publish
        std::string global_frame = "map";
        std::string target_frame = "base_link";
        std::string tf_prefix;
    };

    Vda5050StateBridge(
        rclcpp::Node* node,
        const Config& config,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer);

private:
    void obstacleReportCallback(const rises_interfaces::msg::ObstacleReport::SharedPtr msg);
    void obstacleAlertCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void stateTimerCallback();
    AgvPosition getRobotPosition() const;
    void publishStateJson(const std::string& json);

    rclcpp::Node* node_;
    Config config_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_mqtt_pub_;

    // Subscriptions
    rclcpp::Subscription<rises_interfaces::msg::ObstacleReport>::SharedPtr report_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alert_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr state_timer_;

    // Cached state (protected by mutable mutex for const getRobotPosition)
    mutable std::mutex state_mutex_;
    rises_interfaces::msg::ObstacleReport::SharedPtr last_report_;
    geometry_msgs::msg::PoseStamped::SharedPtr last_pose_;
};

} // namespace rises
