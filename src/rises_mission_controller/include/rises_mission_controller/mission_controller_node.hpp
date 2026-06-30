/// @file mission_controller_node.hpp
/// @brief ARISE mission controller that maps intents to tasks and executes
/// skills.
///
/// This node is entirely optional. The core geofence system runs without it.
/// It bridges the ROS4HRI intent system to the geofence skill layer by:
///   1. Subscribing to /intents (Intent messages)
///   2. Decomposing intents into ordered tasks
///   3. Calling /skill/* action servers for each task
///   4. Publishing mission status on /mission_status

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rises_interfaces/msg/mission_status.hpp"
#include "hri_actions_msgs/msg/intent.hpp"

#include "rises_interfaces/action/get_area_state.hpp"
#include "rises_interfaces/action/get_map_info.hpp"
#include "rises_interfaces/action/get_safety_radius.hpp"
#include "rises_interfaces/action/set_area_state.hpp"
#include "rises_interfaces/action/set_safety_radius.hpp"
#include "rises_interfaces/action/update_map.hpp"
#include "rises_interfaces/action/update_warehouse_layout.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

namespace rises {

/// @brief A single task within a mission. Holds a name and an execute function.
struct Task {
  std::string name;
  std::function<bool()> execute;
};

/// @brief Maps intents to missions (task sequences) and executes them via skill
/// action clients.
class MissionControllerNode : public rclcpp::Node {
public:
  explicit MissionControllerNode(const rclcpp::NodeOptions &options);
  ~MissionControllerNode() override;

  MissionControllerNode(const MissionControllerNode &) = delete;
  MissionControllerNode &operator=(const MissionControllerNode &) = delete;
  MissionControllerNode(MissionControllerNode &&) = delete;
  MissionControllerNode &operator=(MissionControllerNode &&) = delete;

  /// @brief Returns a snapshot of the currently active mission id.
  ///
  /// Acquires mission_mutex_ and returns by value so callers (including tests)
  /// observe a consistent value without racing the mission-execution thread
  /// that writes active_mission_id_ in executeMission().
  /// @return The active mission id, or an empty string if no mission is active.
  [[nodiscard]] std::string activeMissionId() const;

private:
  // Intent subscription
  rclcpp::Subscription<hri_actions_msgs::msg::Intent>::SharedPtr intent_sub_;

  // Mission status publisher
  rclcpp::Publisher<rises_interfaces::msg::MissionStatus>::SharedPtr
      mission_status_pub_;

  // Skill action clients (geofence-specific operations only)
  rclcpp_action::Client<rises_interfaces::action::SetAreaState>::SharedPtr
      set_area_state_client_;
  rclcpp_action::Client<rises_interfaces::action::GetAreaState>::SharedPtr
      get_area_state_client_;
  rclcpp_action::Client<rises_interfaces::action::UpdateMap>::SharedPtr
      update_map_client_;
  rclcpp_action::Client<rises_interfaces::action::UpdateWarehouseLayout>::
      SharedPtr update_warehouse_layout_client_;
  rclcpp_action::Client<rises_interfaces::action::SetSafetyRadius>::SharedPtr
      set_safety_radius_client_;
  rclcpp_action::Client<rises_interfaces::action::GetMapInfo>::SharedPtr
      get_map_info_client_;
  rclcpp_action::Client<rises_interfaces::action::GetSafetyRadius>::SharedPtr
      get_safety_radius_client_;

  // Callback group for action clients (separate from subscription callbacks)
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  // Mission execution thread
  std::thread mission_thread_;
  std::atomic<bool> mission_active_{false};
  // Mutable so the const activeMissionId() accessor can lock it. The mutex
  // serialises writes from the mission-execution thread (executeMission) with
  // reads from intentCallback() and external callers.
  mutable std::mutex mission_mutex_;
  std::string active_mission_id_;

  // Configuration
  std::chrono::seconds action_timeout_{30};

  /// @brief Called when an intent is received on /intents.
  void intentCallback(const hri_actions_msgs::msg::Intent::ConstSharedPtr &msg);

  /// @brief Maps a START_ACTIVITY("lock_area"/"unlock_area") intent to
  /// AREA_ACCESS_CONTROL.
  /// @param data JSON-encoded thematic roles (expects "object" = area_id).
  /// @param lock True to lock, false to unlock.
  /// @return Ordered list of tasks.
  std::vector<Task> buildAreaControlMission(const std::string &data, bool lock);

  /// @brief Maps a START_ACTIVITY("update_map") intent to UPDATE_GEOFENCE_MAP.
  /// @param data JSON-encoded thematic roles.
  /// @return Ordered list of tasks.
  std::vector<Task> buildMapUpdateMission(const std::string &data);

  /// @brief Executes a mission (ordered task list) on a background thread.
  /// @param mission_id Unique identifier for this mission execution.
  /// @param mission_type Human-readable mission type name.
  /// @param tasks Ordered list of tasks to execute.
  void executeMission(const std::string &mission_id,
                      const std::string &mission_type, std::vector<Task> tasks);

  /// @brief Publishes a MissionStatus message.
  void publishStatus(const std::string &mission_id,
                     const std::string &mission_type, uint8_t status,
                     const std::string &current_task, uint8_t total_tasks,
                     uint8_t completed_tasks, const std::string &message);

  /// @brief Generates a unique mission ID from timestamp.
  [[nodiscard]] std::string generateMissionId() const;

  /// @brief Calls a skill action server and blocks until result or timeout.
  /// @tparam ActionT The action type.
  /// @param client Action client to use.
  /// @param goal Populated goal message.
  /// @return Result shared pointer, or nullptr on timeout/failure.
  template <typename ActionT>
  [[nodiscard]] typename ActionT::Result::SharedPtr
  callSkill(typename rclcpp_action::Client<ActionT>::SharedPtr &client,
            typename ActionT::Goal goal);
};

} // namespace rises
