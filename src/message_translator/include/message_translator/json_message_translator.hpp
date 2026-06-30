#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>

#include "rises_interfaces/msg/obstacle_array.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/msg/contours.hpp"
#include "rises_interfaces/srv/set_area_state.hpp"

#include <string>
#include <memory>

namespace rises {

/**
 * Simplified ROS2 node that translates JSON string messages to typed ROS messages.
 *
 * Unlike MessageTranslatorNode this node has no buffering, replay, or multi-AGV
 * coordination. It delegates all JSON-to-ROS conversion to the static converter
 * classes (AabbConverter, ContoursConverter, Vda5050Converter) and publishes the
 * results directly.
 */
class JsonMessageTranslator : public rclcpp::Node {
public:
    explicit JsonMessageTranslator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~JsonMessageTranslator() = default;

private:
    // === Subscription callbacks ===
    void mapUpdatesCallback(const std_msgs::msg::String::SharedPtr msg);
    void orderCallback(const std_msgs::msg::String::SharedPtr msg);
    void contoursCallback(const std_msgs::msg::String::SharedPtr msg);
    void validationCallback(const std_msgs::msg::String::SharedPtr msg);
    void areaLocksCallback(const std_msgs::msg::String::SharedPtr msg);

    // === Setup ===
    void setupPublishers();
    void setupSubscribers();

    // === Configuration ===
    std::string map_frame_;

    // === Publishers ===
    rclcpp::Publisher<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<rises_interfaces::msg::ObstacleArray>::SharedPtr       validation_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                        path_pub_;
    rclcpp::Publisher<rises_interfaces::msg::Contours>::SharedPtr            contours_pub_;

    // === Subscribers ===
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_updates_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr contours_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr validation_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr area_locks_sub_;

    // === Service clients ===
    rclcpp::Client<rises_interfaces::srv::SetAreaState>::SharedPtr area_state_client_;
};

} // namespace rises
