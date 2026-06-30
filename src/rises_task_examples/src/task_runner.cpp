/// @file task_runner.cpp
/// @brief Sequential task runner with MissionStatus reporting.

#include "rises_task_examples/task_runner.hpp"

#include <chrono>
#include <sstream>

namespace rises {

TaskRunner::TaskRunner(
    rclcpp::Node &node,
    rclcpp::Publisher<rises_interfaces::msg::MissionStatus>::SharedPtr
        status_pub)
    : node_(node), status_pub_(std::move(status_pub)) {}

bool TaskRunner::run(const std::string &mission_type,
                     std::vector<TaskStep> steps) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  this->active_.store(true);
  this->cancel_requested_.store(false);
  this->active_mission_id_ = this->generateMissionId();

  const uint8_t total = static_cast<uint8_t>(steps.size());
  uint8_t completed = 0;

  RCLCPP_INFO(this->node_.get_logger(),
              "Starting task '%s' (id: %s) with %u steps", mission_type.c_str(),
              this->active_mission_id_.c_str(), total);

  // Empty step list resolves to immediate success. Without this guard the
  // status publish below dereferences steps.front() and crashes.
  if (steps.empty()) {
    RCLCPP_INFO(this->node_.get_logger(),
                "Task '%s' has no steps; completing immediately",
                mission_type.c_str());
    this->publishStatus(this->active_mission_id_, mission_type,
                        rises_interfaces::msg::MissionStatus::STATUS_SUCCEEDED,
                        "", 0, 0, "Completed (no steps)");
    this->active_.store(false);
    return true;
  }

  this->publishStatus(this->active_mission_id_, mission_type,
                      rises_interfaces::msg::MissionStatus::STATUS_ACTIVE,
                      steps.front().name, total, 0, "Task started");

  for (const TaskStep &step : steps) {
    if (this->cancel_requested_.load()) {
      RCLCPP_WARN(this->node_.get_logger(), "Task '%s' cancelled",
                  mission_type.c_str());
      this->publishStatus(
          this->active_mission_id_, mission_type,
          rises_interfaces::msg::MissionStatus::STATUS_CANCELLED, step.name,
          total, completed, "Cancelled");
      this->active_.store(false);
      return false;
    }

    RCLCPP_INFO(this->node_.get_logger(), "  Step %u/%u: %s", completed + 1,
                total, step.name.c_str());

    this->publishStatus(this->active_mission_id_, mission_type,
                        rises_interfaces::msg::MissionStatus::STATUS_ACTIVE,
                        step.name, total, completed, "Executing: " + step.name);

    const bool ok = step.execute();
    if (!ok) {
      RCLCPP_ERROR(this->node_.get_logger(),
                   "Step '%s' failed. Aborting task '%s'", step.name.c_str(),
                   mission_type.c_str());
      this->publishStatus(this->active_mission_id_, mission_type,
                          rises_interfaces::msg::MissionStatus::STATUS_FAILED,
                          step.name, total, completed,
                          "Failed at: " + step.name);
      this->active_.store(false);
      return false;
    }

    ++completed;
  }

  RCLCPP_INFO(this->node_.get_logger(), "Task '%s' completed successfully",
              mission_type.c_str());
  this->publishStatus(this->active_mission_id_, mission_type,
                      rises_interfaces::msg::MissionStatus::STATUS_SUCCEEDED,
                      "", total, completed, "Completed");

  this->active_.store(false);
  return true;
}

void TaskRunner::cancel() { this->cancel_requested_.store(true); }

bool TaskRunner::isActive() const { return this->active_.load(); }

void TaskRunner::publishStatus(const std::string &mission_id,
                               const std::string &mission_type, uint8_t status,
                               const std::string &current_task,
                               uint8_t total_tasks, uint8_t completed_tasks,
                               const std::string &message) {
  rises_interfaces::msg::MissionStatus msg;
  msg.header.stamp = this->node_.now();
  msg.mission_id = mission_id;
  msg.mission_type = mission_type;
  msg.status = status;
  msg.current_task = current_task;
  msg.total_tasks = total_tasks;
  msg.completed_tasks = completed_tasks;
  msg.message = message;
  this->status_pub_->publish(msg);
}

std::string TaskRunner::generateMissionId() const {
  const int64_t epoch_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  std::ostringstream oss;
  oss << "task_" << epoch_ms;
  return oss.str();
}

} // namespace rises
