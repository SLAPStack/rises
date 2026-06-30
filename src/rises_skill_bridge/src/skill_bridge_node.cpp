/// @file skill_bridge_node.cpp
/// @brief Implementation of the ARISE skill bridge node.
///
/// Bridges geofence ROS2 services to ROS4HRI action servers. Each action's accepted
/// handler calls the corresponding geofence service via callService(), which uses
/// future.wait_for() polling instead of spin_until_future_complete() to avoid
/// executor deadlocks.

#include "rises_skill_bridge/skill_bridge_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <functional>
#include <string>
#include <thread>

namespace rises
{

SkillBridgeNode::SkillBridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("skill_bridge_node", options)
{
    this->geofence_node_name_ = this->declare_parameter<std::string>(
        "geofence_node_name", "geofence_node");
    const int timeout_sec = this->declare_parameter<int>("service_timeout_sec", 10);
    this->service_timeout_ = std::chrono::seconds(timeout_sec);

    // Dedicated callback group for service clients. When the executor processes
    // an action callback on the default group, service responses can still be
    // delivered on this group — preventing deadlock on MultiThreadedExecutor.
    this->service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Service clients — all placed in the dedicated callback group.
    // The geofence's USER services are created with relative names, so they resolve
    // under the geofence node's NAMESPACE (e.g. /agv_0/validate_path), NOT under the
    // node name. The skill bridge runs in that same namespace, so it must reference
    // them by their bare relative names. (The lifecycle change_state below IS
    // node-namespaced — /<ns>/<node>/change_state — so it keeps the node-name prefix.)
    this->validate_path_client_ = this->create_client<rises_interfaces::srv::ValidatePath>(
        "validate_path",
        rmw_qos_profile_services_default, this->service_callback_group_);
    this->update_map_client_ = this->create_client<rises_interfaces::srv::UpdateMap>(
        "update_map",
        rmw_qos_profile_services_default, this->service_callback_group_);
    this->set_area_state_client_ = this->create_client<rises_interfaces::srv::SetAreaState>(
        "set_area_state",
        rmw_qos_profile_services_default, this->service_callback_group_);
    this->set_safety_radius_client_ = this->create_client<rises_interfaces::srv::SetSafetyCircleRadius>(
        "set_safety_circle_radius",
        rmw_qos_profile_services_default, this->service_callback_group_);
    this->update_warehouse_layout_client_ = this->create_client<rises_interfaces::srv::UpdateWarehouseLayout>(
        "update_warehouse_layout",
        rmw_qos_profile_services_default, this->service_callback_group_);
    this->get_area_state_client_ = this->create_client<rises_interfaces::srv::GetAreaState>(
        "get_area_state",
        rmw_qos_profile_services_default, this->service_callback_group_);
    this->get_safety_radius_client_ = this->create_client<rises_interfaces::srv::GetSafetyRadius>(
        "get_safety_radius",
        rmw_qos_profile_services_default, this->service_callback_group_);
    this->get_map_info_client_ = this->create_client<rises_interfaces::srv::GetMapInfo>(
        "get_map_info",
        rmw_qos_profile_services_default, this->service_callback_group_);

    // Action servers (skills)
    this->validate_path_action_ = rclcpp_action::create_server<ValidatePathAction>(
        this, "skill/validate_path",
        std::bind(&SkillBridgeNode::handleValidatePathGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleValidatePathCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleValidatePathAccepted, this, std::placeholders::_1));

    this->update_map_action_ = rclcpp_action::create_server<UpdateMapAction>(
        this, "skill/update_map",
        std::bind(&SkillBridgeNode::handleUpdateMapGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleUpdateMapCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleUpdateMapAccepted, this, std::placeholders::_1));

    this->set_area_state_action_ = rclcpp_action::create_server<SetAreaStateAction>(
        this, "skill/set_area_state",
        std::bind(&SkillBridgeNode::handleSetAreaStateGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleSetAreaStateCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleSetAreaStateAccepted, this, std::placeholders::_1));

    this->set_safety_radius_action_ = rclcpp_action::create_server<SetSafetyRadiusAction>(
        this, "skill/set_safety_radius",
        std::bind(&SkillBridgeNode::handleSetSafetyRadiusGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleSetSafetyRadiusCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleSetSafetyRadiusAccepted, this, std::placeholders::_1));

    this->update_warehouse_layout_action_ = rclcpp_action::create_server<UpdateWarehouseLayoutAction>(
        this, "skill/update_warehouse_layout",
        std::bind(&SkillBridgeNode::handleUpdateWarehouseLayoutGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleUpdateWarehouseLayoutCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleUpdateWarehouseLayoutAccepted, this, std::placeholders::_1));

    this->get_area_state_action_ = rclcpp_action::create_server<GetAreaStateAction>(
        this, "skill/get_area_state",
        std::bind(&SkillBridgeNode::handleGetAreaStateGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleGetAreaStateCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleGetAreaStateAccepted, this, std::placeholders::_1));

    this->get_safety_radius_action_ = rclcpp_action::create_server<GetSafetyRadiusAction>(
        this, "skill/get_safety_radius",
        std::bind(&SkillBridgeNode::handleGetSafetyRadiusGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleGetSafetyRadiusCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleGetSafetyRadiusAccepted, this, std::placeholders::_1));

    this->get_map_info_action_ = rclcpp_action::create_server<GetMapInfoAction>(
        this, "skill/get_map_info",
        std::bind(&SkillBridgeNode::handleGetMapInfoGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleGetMapInfoCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleGetMapInfoAccepted, this, std::placeholders::_1));

    // Lifecycle transition skill — uses lifecycle_msgs/srv/ChangeState on the geofence node
    this->lifecycle_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        this->geofence_node_name_ + "/change_state", rmw_qos_profile_services_default,
        this->service_callback_group_);

    this->set_geofence_enabled_action_ = rclcpp_action::create_server<SetGeofenceEnabledAction>(
        this, "skill/set_geofence_enabled",
        std::bind(&SkillBridgeNode::handleSetGeofenceEnabledGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SkillBridgeNode::handleSetGeofenceEnabledCancel, this, std::placeholders::_1),
        std::bind(&SkillBridgeNode::handleSetGeofenceEnabledAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
        "[SkillBridgeNode] Started (geofence_node='%s', timeout=%ds). "
        "Skills: validate_path, update_map, set_area_state, set_safety_radius, "
        "update_warehouse_layout, get_area_state, get_safety_radius, get_map_info, "
        "set_geofence_enabled",
        this->geofence_node_name_.c_str(), timeout_sec);
}

// ---- Service call helper ----

template <typename ServiceT>
typename ServiceT::Response::SharedPtr SkillBridgeNode::callService(
    typename rclcpp::Client<ServiceT>::SharedPtr& client,
    typename ServiceT::Request::SharedPtr request)
{
    if (!client->wait_for_service(this->service_timeout_)) {
        RCLCPP_ERROR(this->get_logger(),
            "[SkillBridgeNode::callService] Service '%s' not available after %lds timeout",
            client->get_service_name(), this->service_timeout_.count());
        return nullptr;
    }

    std::shared_future<typename ServiceT::Response::SharedPtr> future =
        client->async_send_request(request);

    // Poll the future instead of calling spin_until_future_complete().
    // The service callback group is processed by a separate executor thread,
    // so the future will resolve without us needing to spin.
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + this->service_timeout_;

    while (std::chrono::steady_clock::now() < deadline) {
        if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
            return future.get();
        }
    }

    RCLCPP_ERROR(this->get_logger(),
        "[SkillBridgeNode::callService] Service call to '%s' timed out after %lds",
        client->get_service_name(), this->service_timeout_.count());
    return nullptr;
}

// ============================================================
//  ValidatePath
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleValidatePathGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const ValidatePathAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleValidatePathCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ValidatePathAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleValidatePathAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ValidatePathAction>> goal_handle)
{
    std::shared_ptr<ValidatePathAction::Feedback> feedback =
        std::make_shared<ValidatePathAction::Feedback>();
    feedback->status = "validating path";
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::ValidatePath::Request> request =
        std::make_shared<rises_interfaces::srv::ValidatePath::Request>();
    request->path = goal_handle->get_goal()->path;

    std::shared_ptr<ValidatePathAction::Result> result =
        std::make_shared<ValidatePathAction::Result>();
    const rises_interfaces::srv::ValidatePath::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::ValidatePath>(this->validate_path_client_, request);

    if (response) {
        result->blocked = response->blocked;
        result->message = response->blocked ? "Path is blocked" : "Path is clear";
    } else {
        result->blocked = true;
        result->message = "Service call to validate_path failed";
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ============================================================
//  UpdateMap
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleUpdateMapGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const UpdateMapAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleUpdateMapCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateMapAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleUpdateMapAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateMapAction>> goal_handle)
{
    std::shared_ptr<UpdateMapAction::Feedback> feedback =
        std::make_shared<UpdateMapAction::Feedback>();
    feedback->status = "updating map";
    feedback->progress = 0.0f;
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::UpdateMap::Request> request =
        std::make_shared<rises_interfaces::srv::UpdateMap::Request>();
    request->updates = goal_handle->get_goal()->updates;

    std::shared_ptr<UpdateMapAction::Result> result =
        std::make_shared<UpdateMapAction::Result>();
    const rises_interfaces::srv::UpdateMap::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::UpdateMap>(this->update_map_client_, request);

    if (response) {
        result->success = response->success;
        result->message = response->message;
        result->obstacles_added = response->obstacles_added;
        result->obstacles_removed = response->obstacles_removed;
    } else {
        result->success = false;
        result->message = "Service call to update_map failed";
    }

    feedback->status = "complete";
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ============================================================
//  SetAreaState
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleSetAreaStateGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const SetAreaStateAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleSetAreaStateCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetAreaStateAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleSetAreaStateAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetAreaStateAction>> goal_handle)
{
    std::shared_ptr<SetAreaStateAction::Feedback> feedback =
        std::make_shared<SetAreaStateAction::Feedback>();
    feedback->status = "setting area state";
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::SetAreaState::Request> request =
        std::make_shared<rises_interfaces::srv::SetAreaState::Request>();
    request->area_id = goal_handle->get_goal()->area_id;
    request->lock = goal_handle->get_goal()->lock;

    std::shared_ptr<SetAreaStateAction::Result> result =
        std::make_shared<SetAreaStateAction::Result>();
    const rises_interfaces::srv::SetAreaState::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::SetAreaState>(this->set_area_state_client_, request);

    if (response) {
        result->success = response->success;
        result->message = response->message;
    } else {
        result->success = false;
        result->message = "Service call to set_area_state failed";
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ============================================================
//  SetSafetyRadius
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleSetSafetyRadiusGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const SetSafetyRadiusAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleSetSafetyRadiusCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetSafetyRadiusAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleSetSafetyRadiusAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetSafetyRadiusAction>> goal_handle)
{
    std::shared_ptr<SetSafetyRadiusAction::Feedback> feedback =
        std::make_shared<SetSafetyRadiusAction::Feedback>();
    feedback->status = "updating safety radius";
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::SetSafetyCircleRadius::Request> request =
        std::make_shared<rises_interfaces::srv::SetSafetyCircleRadius::Request>();
    request->radius = goal_handle->get_goal()->radius;

    std::shared_ptr<SetSafetyRadiusAction::Result> result =
        std::make_shared<SetSafetyRadiusAction::Result>();
    const rises_interfaces::srv::SetSafetyCircleRadius::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::SetSafetyCircleRadius>(this->set_safety_radius_client_, request);

    if (response) {
        result->success = response->success;
        result->message = response->message;
    } else {
        result->success = false;
        result->message = "Service call to set_safety_circle_radius failed";
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ============================================================
//  UpdateWarehouseLayout
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleUpdateWarehouseLayoutGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const UpdateWarehouseLayoutAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleUpdateWarehouseLayoutCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateWarehouseLayoutAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleUpdateWarehouseLayoutAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateWarehouseLayoutAction>> goal_handle)
{
    std::shared_ptr<UpdateWarehouseLayoutAction::Feedback> feedback =
        std::make_shared<UpdateWarehouseLayoutAction::Feedback>();
    feedback->status = "updating warehouse layout";
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::UpdateWarehouseLayout::Request> request =
        std::make_shared<rises_interfaces::srv::UpdateWarehouseLayout::Request>();
    request->contours = goal_handle->get_goal()->contours;

    std::shared_ptr<UpdateWarehouseLayoutAction::Result> result =
        std::make_shared<UpdateWarehouseLayoutAction::Result>();
    const rises_interfaces::srv::UpdateWarehouseLayout::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::UpdateWarehouseLayout>(this->update_warehouse_layout_client_, request);

    if (response) {
        result->success = response->success;
        result->message = response->message;
        result->segments_count = response->segments_count;
        result->inner_polygons_count = response->inner_polygons_count;
    } else {
        result->success = false;
        result->message = "Service call to update_warehouse_layout failed";
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ============================================================
//  GetAreaState
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleGetAreaStateGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const GetAreaStateAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleGetAreaStateCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetAreaStateAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleGetAreaStateAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetAreaStateAction>> goal_handle)
{
    std::shared_ptr<GetAreaStateAction::Feedback> feedback =
        std::make_shared<GetAreaStateAction::Feedback>();
    feedback->status = "querying area state";
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::GetAreaState::Request> request =
        std::make_shared<rises_interfaces::srv::GetAreaState::Request>();
    request->area_id = goal_handle->get_goal()->area_id;

    std::shared_ptr<GetAreaStateAction::Result> result =
        std::make_shared<GetAreaStateAction::Result>();
    const rises_interfaces::srv::GetAreaState::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::GetAreaState>(this->get_area_state_client_, request);

    if (response) {
        result->locked = response->locked;
        result->found = response->found;
        result->message = response->message;
    } else {
        result->locked = false;
        result->found = false;
        result->message = "Service call to get_area_state failed";
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ============================================================
//  GetSafetyRadius
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleGetSafetyRadiusGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const GetSafetyRadiusAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleGetSafetyRadiusCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetSafetyRadiusAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleGetSafetyRadiusAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetSafetyRadiusAction>> goal_handle)
{
    std::shared_ptr<GetSafetyRadiusAction::Feedback> feedback =
        std::make_shared<GetSafetyRadiusAction::Feedback>();
    feedback->status = "querying safety radius";
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::GetSafetyRadius::Request> request =
        std::make_shared<rises_interfaces::srv::GetSafetyRadius::Request>();

    std::shared_ptr<GetSafetyRadiusAction::Result> result =
        std::make_shared<GetSafetyRadiusAction::Result>();
    const rises_interfaces::srv::GetSafetyRadius::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::GetSafetyRadius>(this->get_safety_radius_client_, request);

    if (response) {
        result->radius = response->radius;
    } else {
        result->radius = -1.0f;
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ============================================================
//  GetMapInfo
// ============================================================

rclcpp_action::GoalResponse SkillBridgeNode::handleGetMapInfoGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const GetMapInfoAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleGetMapInfoCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetMapInfoAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleGetMapInfoAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetMapInfoAction>> goal_handle)
{
    std::shared_ptr<GetMapInfoAction::Feedback> feedback =
        std::make_shared<GetMapInfoAction::Feedback>();
    feedback->status = "querying map info";
    goal_handle->publish_feedback(feedback);

    std::shared_ptr<rises_interfaces::srv::GetMapInfo::Request> request =
        std::make_shared<rises_interfaces::srv::GetMapInfo::Request>();

    std::shared_ptr<GetMapInfoAction::Result> result =
        std::make_shared<GetMapInfoAction::Result>();
    const rises_interfaces::srv::GetMapInfo::Response::SharedPtr response =
        this->callService<rises_interfaces::srv::GetMapInfo>(this->get_map_info_client_, request);

    if (response) {
        result->obstacle_count = response->obstacle_count;
        result->contours_loaded = response->contours_loaded;
        result->contour_segment_count = response->contour_segment_count;
        result->inner_polygon_count = response->inner_polygon_count;
    } else {
        result->obstacle_count = -1;
        result->contours_loaded = false;
        result->contour_segment_count = 0;
        result->inner_polygon_count = 0;
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// ---- SetGeofenceEnabled (lifecycle transition) ----

rclcpp_action::GoalResponse SkillBridgeNode::handleSetGeofenceEnabledGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const SetGeofenceEnabledAction::Goal>)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SkillBridgeNode::handleSetGeofenceEnabledCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetGeofenceEnabledAction>>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SkillBridgeNode::handleSetGeofenceEnabledAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetGeofenceEnabledAction>> goal_handle)
{
    auto result = std::make_shared<SetGeofenceEnabledAction::Result>();
    const bool enabled = goal_handle->get_goal()->enabled;

    // Lifecycle transition IDs from lifecycle_msgs/msg/Transition
    // activate = 3, deactivate = 4
    constexpr uint8_t TRANSITION_ACTIVATE = 3;
    constexpr uint8_t TRANSITION_DEACTIVATE = 4;

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = enabled ? TRANSITION_ACTIVATE : TRANSITION_DEACTIVATE;

    RCLCPP_INFO(this->get_logger(), "Requesting lifecycle transition: %s",
        enabled ? "activate" : "deactivate");

    if (!this->lifecycle_client_->wait_for_service(this->service_timeout_)) {
        RCLCPP_ERROR(this->get_logger(),
            "Lifecycle change_state service not available for '%s'",
            this->geofence_node_name_.c_str());
        result->success = false;
        result->message = "Lifecycle service not available";
        goal_handle->succeed(result);
        return;
    }

    auto future = this->lifecycle_client_->async_send_request(request);

    // Poll the future (same pattern as callService)
    const auto deadline = std::chrono::steady_clock::now() + this->service_timeout_;
    while (std::chrono::steady_clock::now() < deadline) {
        if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
            break;
        }
    }

    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Lifecycle transition timed out");
        result->success = false;
        result->message = "Lifecycle transition timed out";
        goal_handle->succeed(result);
        return;
    }

    auto response = future.get();
    result->success = response->success;
    result->previous_state = !enabled;  // Best guess — we don't query state beforehand

    if (response->success) {
        result->message = enabled ? "Geofence activated" : "Geofence deactivated";
        RCLCPP_INFO(this->get_logger(), "Lifecycle transition succeeded: %s",
            result->message.c_str());
    } else {
        result->message = enabled ? "Failed to activate geofence" : "Failed to deactivate geofence";
        RCLCPP_ERROR(this->get_logger(), "Lifecycle transition failed: %s",
            result->message.c_str());
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

} // namespace rises

RCLCPP_COMPONENTS_REGISTER_NODE(rises::SkillBridgeNode)
