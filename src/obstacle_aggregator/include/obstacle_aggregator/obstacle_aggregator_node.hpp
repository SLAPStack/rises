/// @file obstacle_aggregator_node.hpp
/// @brief Aggregates unmatched obstacle reports from multiple AGVs into a
/// fleet-wide view.
///
/// Subscribes to /<ns>/obstacle_report for each configured AGV namespace and
/// publishes a combined report on /fleet/obstacle_report. This allows any AGV
/// to see obstacles detected by other AGVs before encountering them.
///
/// Optional node — only needed in multi-AGV deployments.
///
/// ID-offset contract
/// ------------------
/// Each AGV reports obstacles with namespace-local IDs (e.g. agv_0 and agv_1
/// may both emit an obstacle with id=1 to refer to two physically distinct
/// obstacles). To keep those IDs unique in the combined fleet report, every AGV
/// namespace is mapped to a non-negative integer index N, and the per-AGV ID
/// range
///   [N * kIdSpacePerAgv, (N + 1) * kIdSpacePerAgv)
/// is reserved exclusively for that AGV. The combined obstacle ID is
///   combined_id = original_id + N * kIdSpacePerAgv.
///
/// Index derivation is a pure function of the namespace string and therefore
/// deterministic across runs:
///   1. If the namespace matches "<prefix>_<digits>" (e.g. "agv_0", "agv_17"),
///      the trailing digits are used as the index when they fit within
///      kMaxAgvIndex.
///   2. Otherwise the index is `std::hash<std::string>{}(ns) % kMaxAgvIndex`.
///   3. The chosen index is cached on first use so subsequent reports from the
///      same namespace get the same offset without re-parsing.
///
/// Capacity: kMaxAgvIndex = int64_max / kIdSpacePerAgv (~9.22e12), which bounds
/// the hash fallback and prevents the offset multiplication from overflowing
/// uint64. In practice fleets are well below this ceiling.

#pragma once

#include "rises_interfaces/msg/obstacle_report.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rises {

/// @brief Size of the ID range reserved for one AGV namespace.
constexpr std::int64_t kIdSpacePerAgv = 1'000'000;

/// @brief Maximum number of distinguishable AGV namespaces.
///
/// Bounded so that `index * kIdSpacePerAgv` fits in std::int64_t (and therefore
/// trivially in the unsigned uint64 Obstacle::id field).
constexpr std::int64_t kMaxAgvIndex =
    std::numeric_limits<std::int64_t>::max() / kIdSpacePerAgv;

class ObstacleAggregatorNode : public rclcpp::Node {
public:
  explicit ObstacleAggregatorNode(const rclcpp::NodeOptions &options);

private:
  // Per-AGV latest obstacle report
  std::unordered_map<std::string, rises_interfaces::msg::ObstacleReport>
      latest_reports_;
  std::mutex reports_mutex_;

  // Cached namespace -> AGV index lookup. Populated lazily on first use of a
  // namespace and never mutated afterwards, so subsequent reports from the
  // same AGV get the same offset.
  std::unordered_map<std::string, std::int64_t> agv_index_cache_;

  // Subscriptions (one per AGV)
  std::vector<
      rclcpp::Subscription<rises_interfaces::msg::ObstacleReport>::SharedPtr>
      subs_;

  // Combined fleet publisher
  rclcpp::Publisher<rises_interfaces::msg::ObstacleReport>::SharedPtr
      fleet_pub_;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double publish_rate_hz_;

  /// @brief Callback for per-AGV obstacle reports.
  void reportCallback(
      const std::string &agv_ns,
      const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg);

  /// @brief Combines all latest reports and publishes on
  /// /fleet/obstacle_report.
  void publishCombined();

  /// @brief Returns the cached AGV index for @p agv_ns, computing it on first
  /// call.
  /// @note Caller must hold reports_mutex_.
  std::int64_t agvIndexFor(const std::string &agv_ns);
};

} // namespace rises
