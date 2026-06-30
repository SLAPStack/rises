/// @file mission_controller_node.cpp
/// @brief ARISE mission controller for geofence-specific operations.
///
/// This controller handles only geofence-related intents (area locking, map
/// updates, safety parameter changes). Navigation intents like MOVE_TO are
/// handled by a separate navigation mission controller which may call our
/// /skill/* servers as part of its own mission decomposition (e.g.,
/// validate_path before navigating).

#include "rises_mission_controller/mission_controller_node.hpp"

#include <chrono>
#include <sstream>

namespace rises {

MissionControllerNode::MissionControllerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("rises_mission_controller", options) {
  this->declare_parameter("action_timeout_sec", 30);
  this->action_timeout_ =
      std::chrono::seconds(this->get_parameter("action_timeout_sec").as_int());

  // Separate callback group for action clients to avoid deadlock with intent
  // callback
  this->action_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Subscribe to intents
  this->intent_sub_ = this->create_subscription<hri_actions_msgs::msg::Intent>(
      "/intents", 10,
      std::bind(&MissionControllerNode::intentCallback, this,
                std::placeholders::_1));

  // Mission status publisher
  this->mission_status_pub_ =
      this->create_publisher<rises_interfaces::msg::MissionStatus>(
          "/mission_status", 10);

  // Skill action clients (geofence operations only)
  this->set_area_state_client_ =
      rclcpp_action::create_client<rises_interfaces::action::SetAreaState>(
          this, "skill/set_area_state", this->action_callback_group_);
  this->get_area_state_client_ =
      rclcpp_action::create_client<rises_interfaces::action::GetAreaState>(
          this, "skill/get_area_state", this->action_callback_group_);
  this->update_map_client_ =
      rclcpp_action::create_client<rises_interfaces::action::UpdateMap>(
          this, "skill/update_map", this->action_callback_group_);
  this->update_warehouse_layout_client_ = rclcpp_action::create_client<
      rises_interfaces::action::UpdateWarehouseLayout>(
      this, "skill/update_warehouse_layout", this->action_callback_group_);
  this->set_safety_radius_client_ =
      rclcpp_action::create_client<rises_interfaces::action::SetSafetyRadius>(
          this, "skill/set_safety_radius", this->action_callback_group_);
  this->get_map_info_client_ =
      rclcpp_action::create_client<rises_interfaces::action::GetMapInfo>(
          this, "skill/get_map_info", this->action_callback_group_);
  this->get_safety_radius_client_ =
      rclcpp_action::create_client<rises_interfaces::action::GetSafetyRadius>(
          this, "skill/get_safety_radius", this->action_callback_group_);

  RCLCPP_INFO(
      this->get_logger(),
      "ARISE Mission Controller initialized (geofence operations only). "
      "Listening on /intents, action timeout: %ld seconds",
      this->action_timeout_.count());
}

MissionControllerNode::~MissionControllerNode() {
  // The mission thread is owned by this node. If we destroy without joining,
  // std::thread's destructor calls std::terminate() per the standard. Signal
  // cancellation via mission_active_ so any executing skill call observes
  // shutdown, then join. Best-effort: we do not block indefinitely -- the
  // action timeout caps the wait.
  this->mission_active_.store(false);
  if (this->mission_thread_.joinable()) {
    this->mission_thread_.join();
  }
}

void MissionControllerNode::intentCallback(
    const hri_actions_msgs::msg::Intent::ConstSharedPtr &msg) {
  RCLCPP_INFO(this->get_logger(),
              "Received intent: '%s' from '%s' via '%s' (confidence: %.2f, "
              "priority: %u)",
              msg->intent.c_str(), msg->source.c_str(), msg->modality.c_str(),
              msg->confidence, msg->priority);

  if (this->mission_active_.load()) {
    // Snapshot active_mission_id_ under the mutex: it is a non-atomic
    // std::string written by executeMission() on the mission-execution thread.
    // Reading it bare here is a data race (TSan flags it). Copy out and log
    // from the snapshot so the lock window stays small.
    std::string mission_id_snapshot;
    {
      std::lock_guard<std::mutex> guard(this->mission_mutex_);
      mission_id_snapshot = this->active_mission_id_;
    }
    RCLCPP_WARN(this->get_logger(),
                "Mission already active (id: %s). Rejecting intent '%s'",
                mission_id_snapshot.c_str(), msg->intent.c_str());
    return;
  }

  std::vector<Task> tasks;
  std::string mission_type;

  if (msg->intent == hri_actions_msgs::msg::Intent::START_ACTIVITY) {
    if (msg->data.find("lock_area") != std::string::npos) {
      mission_type = "AREA_ACCESS_CONTROL";
      tasks = this->buildAreaControlMission(msg->data, true);
    } else if (msg->data.find("unlock_area") != std::string::npos) {
      mission_type = "AREA_ACCESS_CONTROL";
      tasks = this->buildAreaControlMission(msg->data, false);
    } else if (msg->data.find("update_map") != std::string::npos) {
      mission_type = "UPDATE_GEOFENCE_MAP";
      tasks = this->buildMapUpdateMission(msg->data);
    } else {
      RCLCPP_DEBUG(
          this->get_logger(),
          "START_ACTIVITY '%s' not handled by geofence controller, ignoring",
          msg->data.c_str());
      return;
    }
  } else if (msg->intent == hri_actions_msgs::msg::Intent::STOP_ACTIVITY) {
    RCLCPP_INFO(this->get_logger(),
                "STOP_ACTIVITY received. Cancelling active mission.");
    this->mission_active_.store(false);
    return;
  } else {
    // Intents like MOVE_TO are not our responsibility. A navigation mission
    // controller handles those and may call our skills (validate_path,
    // get_area_state) as part of its own task decomposition.
    RCLCPP_DEBUG(this->get_logger(),
                 "Intent '%s' not handled by geofence controller, ignoring",
                 msg->intent.c_str());
    return;
  }

  if (tasks.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to build task list for mission '%s'",
                 mission_type.c_str());
    return;
  }

  std::string mission_id = this->generateMissionId();

  // Detach previous mission thread if it finished
  if (this->mission_thread_.joinable()) {
    this->mission_thread_.join();
  }

  this->mission_thread_ =
      std::thread(&MissionControllerNode::executeMission, this, mission_id,
                  mission_type, std::move(tasks));
}

std::string MissionControllerNode::activeMissionId() const {
  std::lock_guard<std::mutex> guard(this->mission_mutex_);
  return this->active_mission_id_;
}

std::vector<Task>
MissionControllerNode::buildAreaControlMission(const std::string &data,
                                               bool lock) {
  std::vector<Task> tasks;

  // Extract area_id from data JSON. Look for digits after "area_id" or "goal".
  int64_t area_id = 0;
  {
    std::size_t pos = data.find("area_id");
    if (pos == std::string::npos) {
      pos = data.find("goal");
    }
    if (pos != std::string::npos) {
      while (pos < data.size() && (data[pos] < '0' || data[pos] > '9')) {
        ++pos;
      }
      if (pos < data.size()) {
        area_id = std::stoll(data.substr(pos));
      }
    }
  }

  // Task 1: Query current area state
  tasks.push_back(
      {"get_area_state", [this, area_id]() -> bool {
         if (!this->get_area_state_client_->wait_for_action_server(
                 std::chrono::seconds(5))) {
           RCLCPP_ERROR(this->get_logger(),
                        "skill/get_area_state action server not available");
           return false;
         }

         rises_interfaces::action::GetAreaState::Goal goal;
         goal.area_id = area_id;
         auto result = this->callSkill<rises_interfaces::action::GetAreaState>(
             this->get_area_state_client_, goal);
         if (!result) {
           RCLCPP_ERROR(this->get_logger(),
                        "get_area_state skill call failed or timed out");
           return false;
         }

         if (!result->found) {
           // Not finding the area is NOT fatal: the following set_area_state task is
           // authoritative and creates/locks the area as needed (locking does not
           // require the area to pre-exist). Treat this as an informational query.
           RCLCPP_INFO(this->get_logger(),
                       "Area %ld not yet defined; proceeding to set its state",
                       area_id);
           return true;
         }

         RCLCPP_INFO(this->get_logger(), "Area %ld current state: %s", area_id,
                     result->locked ? "locked" : "unlocked");
         return true;
       }});

  // Task 2: Set area state
  tasks.push_back(
      {"set_area_state", [this, area_id, lock]() -> bool {
         if (!this->set_area_state_client_->wait_for_action_server(
                 std::chrono::seconds(5))) {
           RCLCPP_ERROR(this->get_logger(),
                        "skill/set_area_state action server not available");
           return false;
         }

         rises_interfaces::action::SetAreaState::Goal goal;
         goal.area_id = area_id;
         goal.lock = lock;
         auto result = this->callSkill<rises_interfaces::action::SetAreaState>(
             this->set_area_state_client_, goal);
         if (!result) {
           RCLCPP_ERROR(this->get_logger(),
                        "set_area_state skill call failed or timed out");
           return false;
         }

         if (!result->success) {
           RCLCPP_ERROR(this->get_logger(), "Failed to %s area %ld: %s",
                        lock ? "lock" : "unlock", area_id,
                        result->message.c_str());
           return false;
         }

         RCLCPP_INFO(this->get_logger(), "Area %ld %s successfully", area_id,
                     lock ? "locked" : "unlocked");
         return true;
       }});

  return tasks;
}

std::vector<Task>
MissionControllerNode::buildMapUpdateMission(const std::string & /*data*/) {
  std::vector<Task> tasks;

  // Task 1: Query current map state
  tasks.push_back(
      {"get_map_info", [this]() -> bool {
         if (!this->get_map_info_client_->wait_for_action_server(
                 std::chrono::seconds(5))) {
           RCLCPP_ERROR(this->get_logger(),
                        "skill/get_map_info action server not available");
           return false;
         }

         rises_interfaces::action::GetMapInfo::Goal goal;
         auto result = this->callSkill<rises_interfaces::action::GetMapInfo>(
             this->get_map_info_client_, goal);
         if (!result) {
           RCLCPP_ERROR(this->get_logger(),
                        "get_map_info skill call failed or timed out");
           return false;
         }

         RCLCPP_INFO(this->get_logger(),
                     "Current map: %d obstacles, contours %s, %d segments, %d "
                     "inner polygons",
                     result->obstacle_count,
                     result->contours_loaded ? "loaded" : "not loaded",
                     result->contour_segment_count,
                     result->inner_polygon_count);
         return true;
       }});

  // Task 2: Get current safety radius
  tasks.push_back(
      {"get_safety_radius", [this]() -> bool {
         if (!this->get_safety_radius_client_->wait_for_action_server(
                 std::chrono::seconds(5))) {
           RCLCPP_ERROR(this->get_logger(),
                        "skill/get_safety_radius action server not available");
           return false;
         }

         rises_interfaces::action::GetSafetyRadius::Goal goal;
         auto result =
             this->callSkill<rises_interfaces::action::GetSafetyRadius>(
                 this->get_safety_radius_client_, goal);
         if (!result) {
           RCLCPP_ERROR(this->get_logger(),
                        "get_safety_radius skill call failed or timed out");
           return false;
         }

         RCLCPP_INFO(this->get_logger(), "Current safety radius: %.2f m",
                     result->radius);
         return true;
       }});

  return tasks;
}

void MissionControllerNode::executeMission(const std::string &mission_id,
                                           const std::string &mission_type,
                                           std::vector<Task> tasks) {
  std::lock_guard<std::mutex> lock(this->mission_mutex_);
  this->mission_active_.store(true);
  this->active_mission_id_ = mission_id;

  const uint8_t total = static_cast<uint8_t>(tasks.size());

  RCLCPP_INFO(this->get_logger(),
              "Starting mission '%s' (id: %s) with %u tasks",
              mission_type.c_str(), mission_id.c_str(), total);

  this->publishStatus(mission_id, mission_type,
                      rises_interfaces::msg::MissionStatus::STATUS_ACTIVE,
                      tasks.front().name, total, 0, "Mission started");

  uint8_t completed = 0;
  for (const Task &task : tasks) {
    if (!this->mission_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "Mission '%s' cancelled",
                  mission_id.c_str());
      this->publishStatus(
          mission_id, mission_type,
          rises_interfaces::msg::MissionStatus::STATUS_CANCELLED, task.name,
          total, completed, "Mission cancelled");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Executing task '%s' (%u/%u)",
                task.name.c_str(), completed + 1, total);

    this->publishStatus(mission_id, mission_type,
                        rises_interfaces::msg::MissionStatus::STATUS_ACTIVE,
                        task.name, total, completed,
                        "Executing task: " + task.name);

    bool task_result = task.execute();
    if (!task_result) {
      RCLCPP_ERROR(this->get_logger(),
                   "Task '%s' failed. Aborting mission '%s'", task.name.c_str(),
                   mission_id.c_str());
      this->publishStatus(mission_id, mission_type,
                          rises_interfaces::msg::MissionStatus::STATUS_FAILED,
                          task.name, total, completed,
                          "Task failed: " + task.name);
      this->mission_active_.store(false);
      return;
    }

    ++completed;
  }

  RCLCPP_INFO(this->get_logger(), "Mission '%s' completed successfully",
              mission_id.c_str());
  this->publishStatus(mission_id, mission_type,
                      rises_interfaces::msg::MissionStatus::STATUS_SUCCEEDED,
                      "", total, completed, "Mission completed");

  this->mission_active_.store(false);
}

void MissionControllerNode::publishStatus(
    const std::string &mission_id, const std::string &mission_type,
    uint8_t status, const std::string &current_task, uint8_t total_tasks,
    uint8_t completed_tasks, const std::string &message) {
  rises_interfaces::msg::MissionStatus msg;
  msg.header.stamp = this->now();
  msg.mission_id = mission_id;
  msg.mission_type = mission_type;
  msg.status = status;
  msg.current_task = current_task;
  msg.total_tasks = total_tasks;
  msg.completed_tasks = completed_tasks;
  msg.message = message;
  this->mission_status_pub_->publish(msg);
}

std::string MissionControllerNode::generateMissionId() const {
  const std::chrono::system_clock::time_point now =
      std::chrono::system_clock::now();
  const int64_t epoch_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch())
          .count();
  std::ostringstream oss;
  oss << "mission_" << epoch_ms;
  return oss.str();
}

template <typename ActionT>
typename ActionT::Result::SharedPtr MissionControllerNode::callSkill(
    typename rclcpp_action::Client<ActionT>::SharedPtr &client,
    typename ActionT::Goal goal) {
  auto goal_handle_future = client->async_send_goal(goal);

  auto goal_status = goal_handle_future.wait_for(this->action_timeout_);
  if (goal_status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Skill goal send timed out");
    return nullptr;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Skill goal was rejected");
    return nullptr;
  }

  auto result_future = client->async_get_result(goal_handle);
  auto result_status = result_future.wait_for(this->action_timeout_);
  if (result_status != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Skill result timed out");
    return nullptr;
  }

  auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Skill action did not succeed (code: %d)",
                 static_cast<int>(wrapped_result.code));
    return nullptr;
  }

  return wrapped_result.result;
}

// Explicit template instantiations
template rises_interfaces::action::SetAreaState::Result::SharedPtr
MissionControllerNode::callSkill<rises_interfaces::action::SetAreaState>(
    rclcpp_action::Client<rises_interfaces::action::SetAreaState>::SharedPtr &,
    rises_interfaces::action::SetAreaState::Goal);

template rises_interfaces::action::GetAreaState::Result::SharedPtr
MissionControllerNode::callSkill<rises_interfaces::action::GetAreaState>(
    rclcpp_action::Client<rises_interfaces::action::GetAreaState>::SharedPtr &,
    rises_interfaces::action::GetAreaState::Goal);

template rises_interfaces::action::UpdateMap::Result::SharedPtr
MissionControllerNode::callSkill<rises_interfaces::action::UpdateMap>(
    rclcpp_action::Client<rises_interfaces::action::UpdateMap>::SharedPtr &,
    rises_interfaces::action::UpdateMap::Goal);

template rises_interfaces::action::UpdateWarehouseLayout::Result::SharedPtr
MissionControllerNode::callSkill<
    rises_interfaces::action::UpdateWarehouseLayout>(
    rclcpp_action::Client<
        rises_interfaces::action::UpdateWarehouseLayout>::SharedPtr &,
    rises_interfaces::action::UpdateWarehouseLayout::Goal);

template rises_interfaces::action::SetSafetyRadius::Result::SharedPtr
MissionControllerNode::callSkill<rises_interfaces::action::SetSafetyRadius>(
    rclcpp_action::Client<rises_interfaces::action::SetSafetyRadius>::SharedPtr
        &,
    rises_interfaces::action::SetSafetyRadius::Goal);

template rises_interfaces::action::GetMapInfo::Result::SharedPtr
MissionControllerNode::callSkill<rises_interfaces::action::GetMapInfo>(
    rclcpp_action::Client<rises_interfaces::action::GetMapInfo>::SharedPtr &,
    rises_interfaces::action::GetMapInfo::Goal);

template rises_interfaces::action::GetSafetyRadius::Result::SharedPtr
MissionControllerNode::callSkill<rises_interfaces::action::GetSafetyRadius>(
    rclcpp_action::Client<rises_interfaces::action::GetSafetyRadius>::SharedPtr
        &,
    rises_interfaces::action::GetSafetyRadius::Goal);

} // namespace rises
