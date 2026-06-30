// Helpers for driving rclcpp_lifecycle::LifecycleNode transitions in unit
// tests. The geofence audit regression tests need to assert that
// on_configure / on_activate return ERROR for invalid configurations; rclcpp
// exposes this via Transition + State id comparisons which are verbose at
// every call site. Use these helpers instead.

#pragma once

#include <memory>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace test_support {

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using LifecycleState = lifecycle_msgs::msg::State;
using LifecycleTransition = lifecycle_msgs::msg::Transition;

inline std::uint8_t configure(const std::shared_ptr<LifecycleNode> &node) {
  return node->configure().id();
}

inline std::uint8_t activate(const std::shared_ptr<LifecycleNode> &node) {
  return node->activate().id();
}

inline std::uint8_t deactivate(const std::shared_ptr<LifecycleNode> &node) {
  return node->deactivate().id();
}

inline std::uint8_t cleanup(const std::shared_ptr<LifecycleNode> &node) {
  return node->cleanup().id();
}

inline std::uint8_t shutdown(const std::shared_ptr<LifecycleNode> &node) {
  return node->shutdown().id();
}

inline bool isInactive(std::uint8_t state) noexcept {
  return state == LifecycleState::PRIMARY_STATE_INACTIVE;
}

inline bool isActive(std::uint8_t state) noexcept {
  return state == LifecycleState::PRIMARY_STATE_ACTIVE;
}

inline bool isUnconfigured(std::uint8_t state) noexcept {
  return state == LifecycleState::PRIMARY_STATE_UNCONFIGURED;
}

// Lifecycle transition results: an unsuccessful on_configure callback (returns
// ERROR / FAILURE) leaves the state machine in UNCONFIGURED. An unsuccessful
// on_activate leaves it in INACTIVE. These predicates name those expectations.
inline bool configureFailed(std::uint8_t state) noexcept {
  return isUnconfigured(state);
}

inline bool activateFailed(std::uint8_t state) noexcept {
  return isInactive(state);
}

} // namespace test_support
