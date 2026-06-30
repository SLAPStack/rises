/// @file task_runner.hpp
/// @brief Reusable task runner that executes sequential skill calls with status reporting.
///
/// Provides the generic machinery for composing ARISE skills into tasks:
/// sequential execution, timeout handling, cancellation, and MissionStatus publishing.
/// Concrete tasks (safe_navigation, corridor_crossing, etc.) build task lists
/// using this runner.

#pragma once

#include "rises_interfaces/msg/mission_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rises
{

/// @brief A single step in a task sequence.
struct TaskStep
{
    std::string name;
    std::function<bool()> execute;
};

/// @brief Executes a sequence of TaskSteps, publishing MissionStatus progress.
class TaskRunner
{
public:
    /// @param node The owning node (for logging and clock).
    /// @param status_pub Publisher for MissionStatus messages.
    explicit TaskRunner(
        rclcpp::Node& node,
        rclcpp::Publisher<rises_interfaces::msg::MissionStatus>::SharedPtr status_pub);

    /// @brief Run a named mission (sequence of steps). Blocking call.
    /// @return true if all steps succeeded, false on failure or cancellation.
    bool run(const std::string& mission_type, std::vector<TaskStep> steps);

    /// @brief Cancel the currently running mission.
    void cancel();

    /// @return true if a mission is currently executing.
    [[nodiscard]] bool isActive() const;

private:
    rclcpp::Node& node_;
    rclcpp::Publisher<rises_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
    std::atomic<bool> active_{false};
    std::atomic<bool> cancel_requested_{false};
    std::string active_mission_id_;
    std::mutex mutex_;

    void publishStatus(
        const std::string& mission_id,
        const std::string& mission_type,
        uint8_t status,
        const std::string& current_task,
        uint8_t total_tasks,
        uint8_t completed_tasks,
        const std::string& message);

    [[nodiscard]] std::string generateMissionId() const;
};

/// @brief Helper to call an action skill and block until result. Returns nullptr on failure.
///
/// Works with any action type. Handles goal send, acceptance, and result wait
/// with configurable timeout.
template <typename ActionT>
[[nodiscard]] typename ActionT::Result::SharedPtr callSkillBlocking(
    rclcpp::Node& node,
    typename rclcpp_action::Client<ActionT>::SharedPtr& client,
    typename ActionT::Goal goal,
    std::chrono::seconds timeout = std::chrono::seconds(30))
{
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node.get_logger(), "Action server not available");
        return nullptr;
    }

    // auto is necessary here: rclcpp_action future/handle types are deeply
    // nested and vary across ROS 2 distributions. The return types of
    // async_send_goal() and async_get_result() are implementation-defined.
    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(timeout) != std::future_status::ready) {
        RCLCPP_ERROR(node.get_logger(), "[callSkillBlocking] Goal send timed out");
        return nullptr;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node.get_logger(), "[callSkillBlocking] Goal rejected by server");
        return nullptr;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
        RCLCPP_ERROR(node.get_logger(), "[callSkillBlocking] Result timed out");
        return nullptr;
    }

    auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node.get_logger(), "[callSkillBlocking] Action did not succeed (code: %d)",
            static_cast<int>(wrapped.code));
        return nullptr;
    }

    return wrapped.result;
}

} // namespace rises
