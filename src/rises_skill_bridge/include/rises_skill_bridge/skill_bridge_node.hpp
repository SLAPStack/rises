/// @file skill_bridge_node.hpp
/// @brief Optional ARISE middleware bridge that exposes geofence services as ROS4HRI skill action servers.
///
/// This node is entirely optional. The geofence system runs without it.
/// Launch it alongside the geofence node to expose skills under the /skill/* namespace.
/// Requires a MultiThreadedExecutor (or composable container with MultiThreadedExecutor)
/// because service calls from within action callbacks need a separate callback group.

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rises_interfaces/srv/validate_path.hpp"
#include "rises_interfaces/srv/update_map.hpp"
#include "rises_interfaces/srv/set_area_state.hpp"
#include "rises_interfaces/srv/set_safety_circle_radius.hpp"
#include "rises_interfaces/srv/update_warehouse_layout.hpp"
#include "rises_interfaces/srv/get_area_state.hpp"
#include "rises_interfaces/srv/get_safety_radius.hpp"
#include "rises_interfaces/srv/get_map_info.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rises_interfaces/srv/set_geofence_enabled.hpp"

#include "rises_interfaces/action/validate_path.hpp"
#include "rises_interfaces/action/update_map.hpp"
#include "rises_interfaces/action/set_area_state.hpp"
#include "rises_interfaces/action/set_safety_radius.hpp"
#include "rises_interfaces/action/update_warehouse_layout.hpp"
#include "rises_interfaces/action/get_area_state.hpp"
#include "rises_interfaces/action/get_safety_radius.hpp"
#include "rises_interfaces/action/get_map_info.hpp"
#include "rises_interfaces/action/set_geofence_enabled.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace rises
{

/// @brief Bridges geofence ROS2 services to ROS4HRI skill action servers.
///
/// Creates service clients pointing at the geofence node's existing services and
/// exposes them as action servers under skill/*. Each action goal triggers a service
/// call and returns the response as the action result.
///
/// @note Must run on a MultiThreadedExecutor. Service clients use a dedicated
///       MutuallyExclusiveCallbackGroup so they can be processed while action
///       callbacks are executing on the main group.
class SkillBridgeNode : public rclcpp::Node
{
public:
    /// @param options Node options (remappings, parameters, etc.)
    explicit SkillBridgeNode(const rclcpp::NodeOptions& options);

private:
    // Convenience type aliases for action types (permitted by code_style: "using" for
    // class-level convenience aliases like classname::iterator)
    using ValidatePathAction = rises_interfaces::action::ValidatePath;
    using UpdateMapAction = rises_interfaces::action::UpdateMap;
    using SetAreaStateAction = rises_interfaces::action::SetAreaState;
    using SetSafetyRadiusAction = rises_interfaces::action::SetSafetyRadius;
    using UpdateWarehouseLayoutAction = rises_interfaces::action::UpdateWarehouseLayout;
    using GetAreaStateAction = rises_interfaces::action::GetAreaState;
    using GetSafetyRadiusAction = rises_interfaces::action::GetSafetyRadius;
    using GetMapInfoAction = rises_interfaces::action::GetMapInfo;
    using SetGeofenceEnabledAction = rises_interfaces::action::SetGeofenceEnabled;

    // Callback group for service clients (separate from the default group that runs
    // action callbacks). Prevents deadlock: service futures can be processed on a
    // different executor thread than the action callback that initiated them.
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // Service clients (to the geofence node)
    rclcpp::Client<rises_interfaces::srv::ValidatePath>::SharedPtr validate_path_client_;
    rclcpp::Client<rises_interfaces::srv::UpdateMap>::SharedPtr update_map_client_;
    rclcpp::Client<rises_interfaces::srv::SetAreaState>::SharedPtr set_area_state_client_;
    rclcpp::Client<rises_interfaces::srv::SetSafetyCircleRadius>::SharedPtr set_safety_radius_client_;
    rclcpp::Client<rises_interfaces::srv::UpdateWarehouseLayout>::SharedPtr update_warehouse_layout_client_;
    rclcpp::Client<rises_interfaces::srv::GetAreaState>::SharedPtr get_area_state_client_;
    rclcpp::Client<rises_interfaces::srv::GetSafetyRadius>::SharedPtr get_safety_radius_client_;
    rclcpp::Client<rises_interfaces::srv::GetMapInfo>::SharedPtr get_map_info_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr lifecycle_client_;

    // Action servers (skills)
    rclcpp_action::Server<ValidatePathAction>::SharedPtr validate_path_action_;
    rclcpp_action::Server<UpdateMapAction>::SharedPtr update_map_action_;
    rclcpp_action::Server<SetAreaStateAction>::SharedPtr set_area_state_action_;
    rclcpp_action::Server<SetSafetyRadiusAction>::SharedPtr set_safety_radius_action_;
    rclcpp_action::Server<UpdateWarehouseLayoutAction>::SharedPtr update_warehouse_layout_action_;
    rclcpp_action::Server<GetAreaStateAction>::SharedPtr get_area_state_action_;
    rclcpp_action::Server<GetSafetyRadiusAction>::SharedPtr get_safety_radius_action_;
    rclcpp_action::Server<GetMapInfoAction>::SharedPtr get_map_info_action_;
    rclcpp_action::Server<SetGeofenceEnabledAction>::SharedPtr set_geofence_enabled_action_;

    std::chrono::seconds service_timeout_{10};
    std::string geofence_node_name_;

    /// @brief Send a service request and wait for the response.
    ///
    /// Waits for the service to become available, sends the request asynchronously,
    /// then polls the future until completion or timeout. Uses future.wait_for() instead
    /// of spin_until_future_complete() to avoid deadlocking the executor.
    ///
    /// @tparam ServiceT The service type (e.g., rises_interfaces::srv::ValidatePath).
    /// @param client Service client to use.
    /// @param request Populated request message.
    /// @return Response shared pointer, or nullptr on timeout/failure.
    template <typename ServiceT>
    [[nodiscard]] typename ServiceT::Response::SharedPtr callService(
        typename rclcpp::Client<ServiceT>::SharedPtr& client,
        typename ServiceT::Request::SharedPtr request);

    // --- Action handler triplets (goal / cancel / accepted) ---
    // Each skill has three handlers. The goal handler always accepts.
    // The cancel handler always accepts. The accepted handler calls
    // the corresponding geofence service via callService().

    rclcpp_action::GoalResponse handleValidatePathGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ValidatePathAction::Goal> goal);
    rclcpp_action::CancelResponse handleValidatePathCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ValidatePathAction>> goal_handle);
    void handleValidatePathAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ValidatePathAction>> goal_handle);

    rclcpp_action::GoalResponse handleUpdateMapGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const UpdateMapAction::Goal> goal);
    rclcpp_action::CancelResponse handleUpdateMapCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateMapAction>> goal_handle);
    void handleUpdateMapAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateMapAction>> goal_handle);

    rclcpp_action::GoalResponse handleSetAreaStateGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const SetAreaStateAction::Goal> goal);
    rclcpp_action::CancelResponse handleSetAreaStateCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetAreaStateAction>> goal_handle);
    void handleSetAreaStateAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetAreaStateAction>> goal_handle);

    rclcpp_action::GoalResponse handleSetSafetyRadiusGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const SetSafetyRadiusAction::Goal> goal);
    rclcpp_action::CancelResponse handleSetSafetyRadiusCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetSafetyRadiusAction>> goal_handle);
    void handleSetSafetyRadiusAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetSafetyRadiusAction>> goal_handle);

    rclcpp_action::GoalResponse handleUpdateWarehouseLayoutGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const UpdateWarehouseLayoutAction::Goal> goal);
    rclcpp_action::CancelResponse handleUpdateWarehouseLayoutCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateWarehouseLayoutAction>> goal_handle);
    void handleUpdateWarehouseLayoutAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<UpdateWarehouseLayoutAction>> goal_handle);

    rclcpp_action::GoalResponse handleGetAreaStateGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GetAreaStateAction::Goal> goal);
    rclcpp_action::CancelResponse handleGetAreaStateCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetAreaStateAction>> goal_handle);
    void handleGetAreaStateAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetAreaStateAction>> goal_handle);

    rclcpp_action::GoalResponse handleGetSafetyRadiusGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GetSafetyRadiusAction::Goal> goal);
    rclcpp_action::CancelResponse handleGetSafetyRadiusCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetSafetyRadiusAction>> goal_handle);
    void handleGetSafetyRadiusAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetSafetyRadiusAction>> goal_handle);

    rclcpp_action::GoalResponse handleGetMapInfoGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GetMapInfoAction::Goal> goal);
    rclcpp_action::CancelResponse handleGetMapInfoCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetMapInfoAction>> goal_handle);
    void handleGetMapInfoAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetMapInfoAction>> goal_handle);

    rclcpp_action::GoalResponse handleSetGeofenceEnabledGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const SetGeofenceEnabledAction::Goal> goal);
    rclcpp_action::CancelResponse handleSetGeofenceEnabledCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetGeofenceEnabledAction>> goal_handle);
    void handleSetGeofenceEnabledAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetGeofenceEnabledAction>> goal_handle);
};

} // namespace rises
