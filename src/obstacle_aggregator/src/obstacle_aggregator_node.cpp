/// @file obstacle_aggregator_node.cpp
/// @brief Fleet-wide obstacle aggregation.
///
/// See obstacle_aggregator_node.hpp for the ID-offset contract: each AGV
/// namespace owns the half-open ID range
///   [N * kIdSpacePerAgv, (N + 1) * kIdSpacePerAgv)
/// where N is a deterministic per-namespace index (numeric suffix when the
/// namespace ends in "_<digits>", otherwise std::hash mod kMaxAgvIndex).

#include "obstacle_aggregator/obstacle_aggregator_node.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace rises {

namespace {

/// @brief Parse the trailing digit run of @p ns into an AGV index.
/// @return Parsed index in [0, kMaxAgvIndex) on success, -1 on failure
///         (no trailing digits, or value out of range).
std::int64_t parseTrailingIndex(const std::string &ns) {
  if (ns.empty()) {
    return -1;
  }
  std::size_t digit_begin = ns.size();
  while (digit_begin > 0 && ns[digit_begin - 1] >= '0' &&
         ns[digit_begin - 1] <= '9') {
    --digit_begin;
  }
  if (digit_begin == ns.size()) {
    return -1;
  }
  // Cap parsed length to avoid std::stoll overflow on absurd inputs.
  constexpr std::size_t kMaxDigits = 18;
  const std::size_t digit_len = ns.size() - digit_begin;
  if (digit_len > kMaxDigits) {
    return -1;
  }
  try {
    const long long value = std::stoll(ns.substr(digit_begin));
    if (value < 0 || value >= kMaxAgvIndex) {
      return -1;
    }
    return static_cast<std::int64_t>(value);
  } catch (const std::exception &) {
    return -1;
  }
}

/// @brief Fallback deterministic index for namespaces without a numeric suffix.
std::int64_t hashFallbackIndex(const std::string &ns) {
  const std::size_t hashed = std::hash<std::string>{}(ns);
  return static_cast<std::int64_t>(hashed %
                                   static_cast<std::size_t>(kMaxAgvIndex));
}

} // namespace

ObstacleAggregatorNode::ObstacleAggregatorNode(
    const rclcpp::NodeOptions &options)
    : rclcpp::Node("obstacle_aggregator", options) {
  this->declare_parameter("agv_namespaces",
                          std::vector<std::string>{"agv_0", "agv_1"});
  this->declare_parameter("publish_rate_hz", 5.0);

  const std::vector<std::string> agv_namespaces =
      this->get_parameter("agv_namespaces").as_string_array();
  this->publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

  // Subscribe to each AGV's obstacle_report
  for (const std::string &ns : agv_namespaces) {
    const std::string topic = "/" + ns + "/obstacle_report";
    auto sub = this->create_subscription<rises_interfaces::msg::ObstacleReport>(
        topic, rclcpp::SensorDataQoS().keep_last(10),
        [this,
         ns](const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg) {
          this->reportCallback(ns, msg);
        });
    this->subs_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
  }

  // Fleet-wide combined publisher
  this->fleet_pub_ =
      this->create_publisher<rises_interfaces::msg::ObstacleReport>(
          "/fleet/obstacle_report", rclcpp::SensorDataQoS().keep_last(10));

  // Periodic publish timer
  const auto period =
      std::chrono::duration<double>(1.0 / this->publish_rate_hz_);
  this->publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObstacleAggregatorNode::publishCombined, this));

  RCLCPP_INFO(this->get_logger(),
              "Obstacle Aggregator initialized. Monitoring %zu AGVs, "
              "publishing at %.0f Hz",
              agv_namespaces.size(), this->publish_rate_hz_);
}

void ObstacleAggregatorNode::reportCallback(
    const std::string &agv_ns,
    const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(this->reports_mutex_);
  this->latest_reports_[agv_ns] = *msg;
}

void ObstacleAggregatorNode::publishCombined() {
  std::lock_guard<std::mutex> lock(this->reports_mutex_);

  if (this->latest_reports_.empty()) {
    return;
  }

  rises_interfaces::msg::ObstacleReport combined;
  combined.header.stamp = this->now();
  combined.header.frame_id = "map";

  for (const auto &[agv_ns, report] : this->latest_reports_) {
    // Apply the per-AGV ID offset documented in the header so two AGVs that
    // emit obstacles with the same internal id end up as distinct entries in
    // the combined report. uint64 arithmetic is sufficient: the index is
    // capped at kMaxAgvIndex, so offset stays within int64_max.
    const std::int64_t agv_index = this->agvIndexFor(agv_ns);
    const std::uint64_t offset = static_cast<std::uint64_t>(agv_index) *
                                 static_cast<std::uint64_t>(kIdSpacePerAgv);

    for (const rises_interfaces::msg::Obstacle &obs :
         report.unmatched_obstacles) {
      rises_interfaces::msg::Obstacle shifted = obs;
      shifted.id = obs.id + offset;
      combined.unmatched_obstacles.push_back(shifted);
    }

    // Matched obstacles are per-AGV map context — include them for full
    // picture.
    for (const rises_interfaces::msg::Obstacle &obs :
         report.matched_obstacles) {
      rises_interfaces::msg::Obstacle shifted = obs;
      shifted.id = obs.id + offset;
      combined.matched_obstacles.push_back(shifted);
    }
  }

  this->fleet_pub_->publish(combined);
}

std::int64_t ObstacleAggregatorNode::agvIndexFor(const std::string &agv_ns) {
  const auto cached = this->agv_index_cache_.find(agv_ns);
  if (cached != this->agv_index_cache_.end()) {
    return cached->second;
  }
  std::int64_t index = parseTrailingIndex(agv_ns);
  if (index < 0) {
    index = hashFallbackIndex(agv_ns);
  }
  this->agv_index_cache_.emplace(agv_ns, index);
  return index;
}

} // namespace rises
