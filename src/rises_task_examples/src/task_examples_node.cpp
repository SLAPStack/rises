/// @file task_examples_node.cpp
/// @brief Example tasks that compose ARISE geofence skills.
///
/// Each task is triggered by a Trigger service call. This makes them easy
/// to invoke from the command line without needing the full intent pipeline:
///
///   ros2 service call /task_examples/run_safe_navigation std_srvs/srv/Trigger
///   ros2 service call /task_examples/run_corridor_crossing std_srvs/srv/Trigger
///   ros2 service call /task_examples/run_maintenance_mode std_srvs/srv/Trigger
///   ros2 service call /task_examples/cancel std_srvs/srv/Trigger

#include "rises_task_examples/task_examples_node.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cinttypes>

namespace rises
{

TaskExamplesNode::~TaskExamplesNode()
{
    if (this->runner_) {
        this->runner_->cancel();
    }
    if (this->task_thread_.joinable()) {
        this->task_thread_.join();
    }
}

TaskExamplesNode::TaskExamplesNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("task_examples", options)
{
    this->declare_parameter("skill_timeout_sec", 30);
    this->skill_timeout_ = std::chrono::seconds(
        this->get_parameter("skill_timeout_sec").as_int());

    // Action clients need a separate callback group to avoid executor deadlock
    this->action_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create skill action clients
    this->validate_path_client_ =
        rclcpp_action::create_client<rises_interfaces::action::ValidatePath>(
            this, "skill/validate_path", this->action_group_);
    this->set_area_state_client_ =
        rclcpp_action::create_client<rises_interfaces::action::SetAreaState>(
            this, "skill/set_area_state", this->action_group_);
    this->get_area_state_client_ =
        rclcpp_action::create_client<rises_interfaces::action::GetAreaState>(
            this, "skill/get_area_state", this->action_group_);
    this->set_safety_radius_client_ =
        rclcpp_action::create_client<rises_interfaces::action::SetSafetyRadius>(
            this, "skill/set_safety_radius", this->action_group_);
    this->get_safety_radius_client_ =
        rclcpp_action::create_client<rises_interfaces::action::GetSafetyRadius>(
            this, "skill/get_safety_radius", this->action_group_);
    this->get_map_info_client_ =
        rclcpp_action::create_client<rises_interfaces::action::GetMapInfo>(
            this, "skill/get_map_info", this->action_group_);
    this->set_geofence_enabled_client_ =
        rclcpp_action::create_client<rises_interfaces::action::SetGeofenceEnabled>(
            this, "skill/set_geofence_enabled", this->action_group_);

    // Mission status publisher
    this->status_pub_ = this->create_publisher<rises_interfaces::msg::MissionStatus>(
        "/mission_status", 10);

    // Task runner
    this->runner_ = std::make_unique<TaskRunner>(*this, this->status_pub_);

    // Trigger services
    this->safe_nav_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/run_safe_navigation",
        std::bind(&TaskExamplesNode::handleSafeNavigation, this,
            std::placeholders::_1, std::placeholders::_2));
    this->corridor_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/run_corridor_crossing",
        std::bind(&TaskExamplesNode::handleCorridorCrossing, this,
            std::placeholders::_1, std::placeholders::_2));
    this->maintenance_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/run_maintenance_mode",
        std::bind(&TaskExamplesNode::handleMaintenanceMode, this,
            std::placeholders::_1, std::placeholders::_2));
    this->cancel_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/cancel",
        std::bind(&TaskExamplesNode::handleCancel, this,
            std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "Task examples node ready. Trigger tasks via:\n"
        "  ros2 service call ~/run_safe_navigation std_srvs/srv/Trigger\n"
        "  ros2 service call ~/run_corridor_crossing std_srvs/srv/Trigger\n"
        "  ros2 service call ~/run_maintenance_mode std_srvs/srv/Trigger\n"
        "  ros2 service call ~/cancel std_srvs/srv/Trigger");
}

void TaskExamplesNode::launchTask(
    const std::string& mission_type,
    std::vector<TaskStep> steps,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->runner_->isActive()) {
        response->success = false;
        response->message = "A task is already running. Call ~/cancel first.";
        return;
    }

    // Join previous thread if finished
    if (this->task_thread_.joinable()) {
        this->task_thread_.join();
    }

    response->success = true;
    response->message = "Task '" + mission_type + "' started";

    this->task_thread_ = std::thread([this, mission_type, steps = std::move(steps)]() {
        this->runner_->run(mission_type, std::move(steps));
    });
}

// ---------------------------------------------------------------------------
// Task: safe_navigation
// Validates path with geofence before simulating navigation.
// ---------------------------------------------------------------------------

void TaskExamplesNode::handleSafeNavigation(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    this->launchTask("safe_navigation", this->buildSafeNavigationTask(), response);
}

std::vector<TaskStep> TaskExamplesNode::buildSafeNavigationTask()
{
    std::vector<TaskStep> steps;

    // Step 1: Check geofence is healthy
    steps.push_back({"get_map_info", [this]() -> bool {
        rises_interfaces::action::GetMapInfo::Goal goal;
        auto result = callSkillBlocking<rises_interfaces::action::GetMapInfo>(
            *this, this->get_map_info_client_, goal, this->skill_timeout_);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "get_map_info failed — geofence not reachable");
            return false;
        }
        if (!result->contours_loaded) {
            RCLCPP_ERROR(this->get_logger(), "Warehouse contours not loaded — cannot navigate safely");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Geofence healthy: %d obstacles, %d contour segments",
            result->obstacle_count, result->contour_segment_count);
        return true;
    }});

    // Step 2: Validate the planned path
    steps.push_back({"validate_path", [this]() -> bool {
        rises_interfaces::action::ValidatePath::Goal goal;

        // Build a test path (straight line from (1,1) to (10,5))
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        for (int i = 0; i <= 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = 1.0 + static_cast<double>(i) * 0.9;
            pose.pose.position.y = 1.0 + static_cast<double>(i) * 0.4;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        goal.path = path;

        auto result = callSkillBlocking<rises_interfaces::action::ValidatePath>(
            *this, this->validate_path_client_, goal, this->skill_timeout_);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "validate_path skill call failed");
            return false;
        }
        if (result->blocked) {
            RCLCPP_WARN(this->get_logger(), "Path blocked: %s — would reroute in production",
                result->message.c_str());
            // In a real task this would trigger replanning. For the demo we continue.
        } else {
            RCLCPP_INFO(this->get_logger(), "Path validated — safe to navigate");
        }
        return true;
    }});

    // Step 3: Navigate (mock — just log and sleep)
    steps.push_back({"navigate", [this]() -> bool {
        RCLCPP_INFO(this->get_logger(),
            "Navigating to target (mock — in production this calls /skill/navigate)...");
        // Simulate navigation time
        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(this->get_logger(), "Navigation complete (mock)");
        return true;
    }});

    return steps;
}

// ---------------------------------------------------------------------------
// Task: corridor_crossing
// Locks an area, tightens safety, validates, navigates, restores, unlocks.
// ---------------------------------------------------------------------------

void TaskExamplesNode::handleCorridorCrossing(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    this->launchTask("corridor_crossing", this->buildCorridorCrossingTask(), response);
}

std::vector<TaskStep> TaskExamplesNode::buildCorridorCrossingTask()
{
    std::vector<TaskStep> steps;
    constexpr int64_t corridor_area_id = 1;
    constexpr float narrow_radius = 0.3f;

    // Step 1: Check if corridor is free
    steps.push_back({"check_corridor", [this]() -> bool {
        rises_interfaces::action::GetAreaState::Goal goal;
        goal.area_id = corridor_area_id;
        auto result = callSkillBlocking<rises_interfaces::action::GetAreaState>(
            *this, this->get_area_state_client_, goal, this->skill_timeout_);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "get_area_state failed");
            return false;
        }
        if (result->locked) {
            RCLCPP_WARN(this->get_logger(), "Corridor %" PRId64 " is locked by another AGV — waiting",
                corridor_area_id);
            // In production: wait/retry loop or reroute. For demo: fail.
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Corridor %" PRId64 " is free", corridor_area_id);
        return true;
    }});

    // Step 2: Lock the corridor
    steps.push_back({"lock_corridor", [this]() -> bool {
        rises_interfaces::action::SetAreaState::Goal goal;
        goal.area_id = corridor_area_id;
        goal.lock = true;
        auto result = callSkillBlocking<rises_interfaces::action::SetAreaState>(
            *this, this->set_area_state_client_, goal, this->skill_timeout_);
        if (!result || !result->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to lock corridor");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Corridor locked");
        return true;
    }});

    // Step 3: Tighten safety radius for narrow corridor
    steps.push_back({"tighten_safety", [this]() -> bool {
        rises_interfaces::action::SetSafetyRadius::Goal goal;
        goal.radius = narrow_radius;
        auto result = callSkillBlocking<rises_interfaces::action::SetSafetyRadius>(
            *this, this->set_safety_radius_client_, goal, this->skill_timeout_);
        if (!result || !result->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set safety radius");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Safety radius set to %.2fm for narrow corridor",
            narrow_radius);
        return true;
    }});

    // Step 4: Validate path through corridor
    steps.push_back({"validate_path", [this]() -> bool {
        rises_interfaces::action::ValidatePath::Goal goal;
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        for (int i = 0; i <= 5; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = 5.0 + static_cast<double>(i) * 2.0;
            pose.pose.position.y = 7.5;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        goal.path = path;
        auto result = callSkillBlocking<rises_interfaces::action::ValidatePath>(
            *this, this->validate_path_client_, goal, this->skill_timeout_);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "validate_path failed");
            return false;
        }
        if (result->blocked) {
            RCLCPP_WARN(this->get_logger(), "Corridor path blocked: %s", result->message.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Corridor path validated");
        }
        return true;
    }});

    // Step 5: Navigate through corridor (mock)
    steps.push_back({"navigate_corridor", [this]() -> bool {
        RCLCPP_INFO(this->get_logger(), "Navigating through corridor (mock)...");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(this->get_logger(), "Corridor crossing complete (mock)");
        return true;
    }});

    // Step 6: Restore safety radius
    steps.push_back({"restore_safety", [this]() -> bool {
        rises_interfaces::action::SetSafetyRadius::Goal goal;
        goal.radius = 0.5f;
        auto result = callSkillBlocking<rises_interfaces::action::SetSafetyRadius>(
            *this, this->set_safety_radius_client_, goal, this->skill_timeout_);
        if (!result || !result->success) {
            RCLCPP_WARN(this->get_logger(), "Failed to restore safety radius — non-fatal");
        } else {
            RCLCPP_INFO(this->get_logger(), "Safety radius restored to 0.5m");
        }
        return true;
    }});

    // Step 7: Unlock corridor
    steps.push_back({"unlock_corridor", [this]() -> bool {
        rises_interfaces::action::SetAreaState::Goal goal;
        goal.area_id = corridor_area_id;
        goal.lock = false;
        auto result = callSkillBlocking<rises_interfaces::action::SetAreaState>(
            *this, this->set_area_state_client_, goal, this->skill_timeout_);
        if (!result || !result->success) {
            RCLCPP_WARN(this->get_logger(), "Failed to unlock corridor — non-fatal");
        } else {
            RCLCPP_INFO(this->get_logger(), "Corridor unlocked");
        }
        return true;
    }});

    return steps;
}

// ---------------------------------------------------------------------------
// Task: maintenance_mode
// Disables geofence, navigates home, waits, re-enables.
// ---------------------------------------------------------------------------

void TaskExamplesNode::handleMaintenanceMode(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    this->launchTask("maintenance_mode", this->buildMaintenanceModeTask(), response);
}

std::vector<TaskStep> TaskExamplesNode::buildMaintenanceModeTask()
{
    std::vector<TaskStep> steps;

    // Step 1: Disable geofence enforcement
    steps.push_back({"disable_geofence", [this]() -> bool {
        rises_interfaces::action::SetGeofenceEnabled::Goal goal;
        goal.enabled = false;
        auto result = callSkillBlocking<rises_interfaces::action::SetGeofenceEnabled>(
            *this, this->set_geofence_enabled_client_, goal, this->skill_timeout_);
        if (!result || !result->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to disable geofence");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Geofence disabled for maintenance");
        return true;
    }});

    // Step 2: Navigate to home position (mock)
    steps.push_back({"navigate_home", [this]() -> bool {
        RCLCPP_INFO(this->get_logger(), "Navigating to home position (mock)...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "At home position (mock)");
        return true;
    }});

    // Step 3: Wait for maintenance to complete (simulated)
    steps.push_back({"wait_maintenance", [this]() -> bool {
        RCLCPP_INFO(this->get_logger(),
            "Waiting for maintenance (5s mock — in production waits for STOP_ACTIVITY intent)...");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        return true;
    }});

    // Step 4: Re-enable geofence
    steps.push_back({"enable_geofence", [this]() -> bool {
        rises_interfaces::action::SetGeofenceEnabled::Goal goal;
        goal.enabled = true;
        auto result = callSkillBlocking<rises_interfaces::action::SetGeofenceEnabled>(
            *this, this->set_geofence_enabled_client_, goal, this->skill_timeout_);
        if (!result || !result->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to re-enable geofence");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Geofence re-enabled");
        return true;
    }});

    // Step 5: Verify geofence is healthy
    steps.push_back({"verify_health", [this]() -> bool {
        rises_interfaces::action::GetMapInfo::Goal goal;
        auto result = callSkillBlocking<rises_interfaces::action::GetMapInfo>(
            *this, this->get_map_info_client_, goal, this->skill_timeout_);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "Health check failed after maintenance");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Post-maintenance health check passed: %d obstacles, contours %s",
            result->obstacle_count, result->contours_loaded ? "loaded" : "NOT loaded");
        return result->contours_loaded;
    }});

    return steps;
}

// ---------------------------------------------------------------------------
// Cancel
// ---------------------------------------------------------------------------

void TaskExamplesNode::handleCancel(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->runner_->isActive()) {
        this->runner_->cancel();
        response->success = true;
        response->message = "Cancel requested";
    } else {
        response->success = false;
        response->message = "No task running";
    }
}

} // namespace rises
