/// @file task_examples_node.hpp
/// @brief Demo node that runs example tasks composing ARISE geofence skills.
///
/// Provides three example tasks triggered via a ROS service:
///   - safe_navigation: validate_path → navigate (mock)
///   - corridor_crossing: lock area → tighten safety → validate → navigate → restore → unlock
///   - maintenance_mode: disable geofence → navigate home → wait → re-enable
///
/// These demonstrate how an integrator would compose geofence skills into tasks
/// from their own mission controller. This is a reference implementation, not
/// a production component.

#pragma once

#include "rises_task_examples/task_runner.hpp"

#include "rises_interfaces/action/validate_path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rises_interfaces/action/set_area_state.hpp"
#include "rises_interfaces/action/get_area_state.hpp"
#include "rises_interfaces/action/set_safety_radius.hpp"
#include "rises_interfaces/action/get_safety_radius.hpp"
#include "rises_interfaces/action/get_map_info.hpp"
#include "rises_interfaces/action/set_geofence_enabled.hpp"
#include "rises_interfaces/msg/mission_status.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace rises
{

class TaskExamplesNode : public rclcpp::Node
{
public:
    explicit TaskExamplesNode(const rclcpp::NodeOptions& options);

private:
    // Skill action clients
    rclcpp_action::Client<rises_interfaces::action::ValidatePath>::SharedPtr validate_path_client_;
    rclcpp_action::Client<rises_interfaces::action::SetAreaState>::SharedPtr set_area_state_client_;
    rclcpp_action::Client<rises_interfaces::action::GetAreaState>::SharedPtr get_area_state_client_;
    rclcpp_action::Client<rises_interfaces::action::SetSafetyRadius>::SharedPtr set_safety_radius_client_;
    rclcpp_action::Client<rises_interfaces::action::GetSafetyRadius>::SharedPtr get_safety_radius_client_;
    rclcpp_action::Client<rises_interfaces::action::GetMapInfo>::SharedPtr get_map_info_client_;
    rclcpp_action::Client<rises_interfaces::action::SetGeofenceEnabled>::SharedPtr set_geofence_enabled_client_;

    // Status publisher
    rclcpp::Publisher<rises_interfaces::msg::MissionStatus>::SharedPtr status_pub_;

    // Task runner
    std::unique_ptr<TaskRunner> runner_;

    // Callback group for action clients (separate from service callbacks)
    rclcpp::CallbackGroup::SharedPtr action_group_;

    // Trigger services — one per example task
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safe_nav_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr corridor_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr maintenance_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;

    std::chrono::seconds skill_timeout_{30};
    std::thread task_thread_;

    // Rule of Five: destructor must join the task thread to avoid std::terminate.
    // Copy/move are deleted because std::thread is not copyable and the node
    // holds non-movable ROS resources (publishers, subscriptions, services).
public:
    ~TaskExamplesNode();
    TaskExamplesNode(const TaskExamplesNode&) = delete;
    TaskExamplesNode& operator=(const TaskExamplesNode&) = delete;
    TaskExamplesNode(TaskExamplesNode&&) = delete;
    TaskExamplesNode& operator=(TaskExamplesNode&&) = delete;
private:

    // Service handlers
    void handleSafeNavigation(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void handleCorridorCrossing(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void handleMaintenanceMode(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void handleCancel(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);

    // Task builders — return step sequences for the runner
    [[nodiscard]] std::vector<TaskStep> buildSafeNavigationTask();
    [[nodiscard]] std::vector<TaskStep> buildCorridorCrossingTask();
    [[nodiscard]] std::vector<TaskStep> buildMaintenanceModeTask();

    /// @brief Launch a task on a background thread.
    void launchTask(const std::string& mission_type, std::vector<TaskStep> steps,
                    std_srvs::srv::Trigger::Response::SharedPtr response);
};

} // namespace rises
