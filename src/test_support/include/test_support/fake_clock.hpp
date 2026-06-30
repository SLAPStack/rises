// Deterministic monotonic clock for unit tests. Atomic-counter backed so a
// test thread can advance time while another thread reads it without
// introducing a mutex. Drop-in for code paths that accept any clock type
// satisfying the `now() -> time_point` + `advance(duration)` shape, or for
// raw std::chrono-style usage via the nested duration / time_point typedefs.
//
// For ROS-aware tests that need /clock published with sim time, see
// RosSimTimePublisher below.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace test_support {

class FakeClock {
public:
  using rep = std::int64_t;
  using period = std::nano;
  using duration = std::chrono::nanoseconds;
  using time_point = std::chrono::time_point<FakeClock, duration>;

  static constexpr bool is_steady = true;

  FakeClock() noexcept : now_ns_{0} {}
  explicit FakeClock(duration initial) noexcept
      : now_ns_{static_cast<std::int64_t>(initial.count())} {}

  FakeClock(const FakeClock &) = delete;
  FakeClock &operator=(const FakeClock &) = delete;
  FakeClock(FakeClock &&) = delete;
  FakeClock &operator=(FakeClock &&) = delete;

  time_point now() const noexcept {
    return time_point{duration{now_ns_.load(std::memory_order_acquire)}};
  }

  void advance(duration delta) noexcept {
    now_ns_.fetch_add(static_cast<std::int64_t>(delta.count()),
                      std::memory_order_acq_rel);
  }

  void set(duration absolute) noexcept {
    now_ns_.store(static_cast<std::int64_t>(absolute.count()),
                  std::memory_order_release);
  }

private:
  std::atomic<std::int64_t> now_ns_;
};

// Publishes /clock messages so a node configured with `use_sim_time:=true`
// advances along a deterministic timeline driven by the test. Owns a small
// dedicated rclcpp::Node; do not reuse the system-under-test node for this.
class RosSimTimePublisher {
public:
  explicit RosSimTimePublisher(const std::string &owner_node_name)
      : node_{std::make_shared<rclcpp::Node>(owner_node_name)},
        pub_{node_->create_publisher<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).reliable())},
        current_{0, 0, RCL_ROS_TIME} {}

  RosSimTimePublisher(const RosSimTimePublisher &) = delete;
  RosSimTimePublisher &operator=(const RosSimTimePublisher &) = delete;

  void set(rclcpp::Time t) {
    current_ = t;
    rosgraph_msgs::msg::Clock msg;
    msg.clock = t;
    pub_->publish(msg);
  }

  void advance(rclcpp::Duration delta) { set(current_ + delta); }

  rclcpp::Time current() const noexcept { return current_; }
  rclcpp::Node::SharedPtr node() const noexcept { return node_; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::Time current_;
};

} // namespace test_support
