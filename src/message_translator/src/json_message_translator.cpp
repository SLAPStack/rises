#include "message_translator/json_message_translator.hpp"
#include "message_translator/aabb_converter.hpp"
#include "message_translator/area_locks_converter.hpp"
#include "message_translator/contours_converter.hpp"
#include "message_translator/vda5050_converter.hpp"

#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rises_interfaces/msg/obstacle_update_array.hpp>
#include <rises_interfaces/msg/obstacle_array.hpp>
#include <rises_interfaces/msg/contours.hpp>

#include <memory>

namespace rises {

JsonMessageTranslator::JsonMessageTranslator(const rclcpp::NodeOptions& options)
    : rclcpp::Node("json_message_translator", options)
{
    this->map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

    this->setupPublishers();
    this->setupSubscribers();

    RCLCPP_INFO(this->get_logger(), "JsonMessageTranslator initialized (map_frame: %s)",
        this->map_frame_.c_str());
}

void JsonMessageTranslator::setupPublishers()
{
    this->path_pub_       = this->create_publisher<nav_msgs::msg::Path>("incoming_path", 10);
    this->contours_pub_   = this->create_publisher<rises_interfaces::msg::Contours>("warehouse_contours", 10);
    this->obstacle_pub_   = this->create_publisher<rises_interfaces::msg::ObstacleUpdateArray>("map_updates", 10);
    this->validation_pub_ = this->create_publisher<rises_interfaces::msg::ObstacleArray>("validation", 10);
    this->area_state_client_ =
        this->create_client<rises_interfaces::srv::SetAreaState>("set_area_state");
}

void JsonMessageTranslator::setupSubscribers()
{
    this->map_updates_sub_ = this->create_subscription<std_msgs::msg::String>(
        "obstacle_json", rclcpp::QoS(5000).reliable().transient_local(),
        std::bind(&JsonMessageTranslator::mapUpdatesCallback, this, std::placeholders::_1));

    this->order_sub_ = this->create_subscription<std_msgs::msg::String>(
        "order", 10,
        std::bind(&JsonMessageTranslator::orderCallback, this, std::placeholders::_1));

    this->contours_sub_ = this->create_subscription<std_msgs::msg::String>(
        "warehouse_contours_json", rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&JsonMessageTranslator::contoursCallback, this, std::placeholders::_1));

    this->validation_sub_ = this->create_subscription<std_msgs::msg::String>(
        "validation_json", 10,
        std::bind(&JsonMessageTranslator::validationCallback, this, std::placeholders::_1));

    this->area_locks_sub_ = this->create_subscription<std_msgs::msg::String>(
        "area_locks_json", 10,
        std::bind(&JsonMessageTranslator::areaLocksCallback, this, std::placeholders::_1));
}

void JsonMessageTranslator::mapUpdatesCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty obstacle update message");
        return;
    }

    const std::vector<rises_interfaces::msg::ObstacleUpdate> updates =
        rises::AabbConverter::parseObstacleUpdates(msg->data);

    if (updates.empty()) {
        return;
    }

    std::unique_ptr<rises_interfaces::msg::ObstacleUpdateArray> arr =
        std::make_unique<rises_interfaces::msg::ObstacleUpdateArray>();
    arr->header.stamp    = this->now();
    arr->header.frame_id = this->map_frame_;
    arr->updates         = updates;

    this->obstacle_pub_->publish(std::move(arr));
    RCLCPP_DEBUG(this->get_logger(), "Published %zu obstacle updates", updates.size());
}

void JsonMessageTranslator::validationCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty validation message");
        return;
    }

    const rises_interfaces::msg::Obstacle obs =
        rises::AabbConverter::parseValidationObstacle(msg->data);

    if (obs.type == 0) {
        return;  // AabbConverter already logged the failure.
    }

    std::unique_ptr<rises_interfaces::msg::ObstacleArray> arr =
        std::make_unique<rises_interfaces::msg::ObstacleArray>();
    arr->header.stamp    = this->now();
    arr->header.frame_id = this->map_frame_;
    arr->obstacles.push_back(obs);

    this->validation_pub_->publish(std::move(arr));
    RCLCPP_DEBUG(this->get_logger(),
        "Published validation obstacle id=%lu type=%u at (%.3f, %.3f)",
        obs.id, obs.type, obs.position.x, obs.position.y);
}

void JsonMessageTranslator::orderCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty VDA5050 order message");
        return;
    }

    const nav_msgs::msg::Path path =
        rises::Vda5050Converter::orderToPath(msg->data, this->map_frame_, this->now());

    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "VDA5050 order produced no path waypoints");
        return;
    }

    this->path_pub_->publish(path);
    RCLCPP_DEBUG(this->get_logger(),
        "Published path with %zu waypoints from VDA5050 order", path.poses.size());
}

void JsonMessageTranslator::contoursCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty warehouse contours message");
        return;
    }

    const rises_interfaces::msg::Contours contours =
        rises::ContoursConverter::parse(msg->data, this->map_frame_);

    RCLCPP_INFO(this->get_logger(),
        "Publishing warehouse contours: %zu outer segments, %zu hull points, %zu inner polygons",
        contours.outer_contour_segments.size(),
        contours.outer_contour_hull.points.size(),
        contours.inner_contours.size());

    this->contours_pub_->publish(contours);
}

void JsonMessageTranslator::areaLocksCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty area_locks message");
        return;
    }

    bool ok = false;
    const rises_interfaces::msg::AreaState area_state =
        rises::AreaLocksConverter::parse(msg->data, ok);

    if (!ok) {
        return;  // AreaLocksConverter already logged the failure.
    }

    if (!this->area_state_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(),
            "set_area_state service not available; dropping area_locks message (id=%ld)",
            area_state.id);
        return;
    }

    auto request = std::make_shared<rises_interfaces::srv::SetAreaState::Request>();
    request->area_id = area_state.id;
    request->lock = (area_state.operation == rises_interfaces::msg::AreaState::LOCK);

    this->area_state_client_->async_send_request(request,
        [this, area_state](rclcpp::Client<rises_interfaces::srv::SetAreaState>::SharedFuture future) {
            try {
                const auto response = future.get();
                if (response->success) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "Area %s successful (id=%ld): %s",
                        area_state.operation == rises_interfaces::msg::AreaState::LOCK ? "lock" : "unlock",
                        area_state.id, response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Area %s failed (id=%ld): %s",
                        area_state.operation == rises_interfaces::msg::AreaState::LOCK ? "lock" : "unlock",
                        area_state.id, response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                    "set_area_state service call failed (id=%ld): %s",
                    area_state.id, e.what());
            }
        });
}

} // namespace rises

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rises::JsonMessageTranslator)
