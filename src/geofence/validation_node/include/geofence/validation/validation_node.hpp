#pragma once

#include "rises_interfaces/msg/obstacle_report.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rises {

/**
 * Standalone validation node measuring geofence detection latency and ratio.
 *
 * Detections are read from the geofence's "obstacle_report" message, field
 * unmatched_obstacles[] -- gap-segmented LINE segments whose .position is the
 * segment midpoint at the obstacle's REAL location. (The geofence's
 * unmatched_obstacle TOPIC reports a single whole-scan centroid in points-only
 * mode, which is meaningless for per-obstacle matching; the report carries the
 * correct per-segment positions, so this node uses those.)
 *
 * Latency is anchored on safety-circle entry (the moment the robot first comes
 * within eligibility_radius of a spawn), not on spawn time, and matching uses a
 * short post-spawn window (match_window) so a moving obstacle is paired with its
 * prompt detection near the spawn rather than a later unrelated segment that
 * wanders back across the abandoned spawn point.
 *
 * Subscribes to:
 *   - "obstacle_report"   (ObstacleReport)  -- geofence per-segment detections
 *   - "obstacle_spawn"    (std_msgs/String) -- JSON ground-truth spawn events
 *   - "obstacle_validate" (std_msgs/String) -- JSON validate events with spawn
 * timestamp
 *
 * Publishes:
 *   - "validation_result" (std_msgs/String) -- JSON cumulative stats
 *
 * Optionally writes per-detection CSV rows (parameter: output_file).
 *
 * All topic names are remappable via standard ROS2 topic remapping.
 */
class ValidationNode : public rclcpp::Node {
public:
  explicit ValidationNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ValidationNode() override;

private:
  struct SpawnedObstacle {
    rclcpp::Time embedded_time;
    double x{0.0};
    double y{0.0};
    double radius{0.0};
    bool detected{false};
    /// True once the robot has passed within eligibility_radius_ of this spawn.
    /// Only eligible spawns count toward the detection-ratio denominator: an
    /// obstacle the robot never came near was never in the geofence's detection
    /// zone, so counting it as a "miss" would understate real performance.
    bool eligible{false};
    rclcpp::Time first_detection_stamp;
    double match_distance_m{0.0}; ///< Spawn->detection distance at first match.

    /// Latency is measured from the moment the obstacle ENTERED the safety
    /// circle, not from spawn. circle_entry_stamp is the time the robot first
    /// came within eligibility_radius (== safety_circle_radius) of this spawn.
    bool entered_circle{false};
    rclcpp::Time circle_entry_stamp;
    /// Signed latency = first_detection - circle_entry. Negative means the
    /// geofence detected the obstacle BEFORE it reached the safety circle (lead
    /// time, the desired case); positive means it reacted after entry. Defined
    /// only once the obstacle has both been detected and observed entering the
    /// circle, so it is finalized at whichever of the two events happens last.
    bool latency_finalized{false};
    double signed_latency_ms{0.0};

    /// Diagnostic (A2): closest any unmatched detection ever came to this spawn,
    /// regardless of the match window. For a spawn that is eligible but never
    /// detected, this distinguishes "the geofence saw something nearby but the
    /// association missed it" (small value -> validation issue) from "the
    /// geofence never reported anything near it" (stays max -> geofence issue).
    double nearest_unmatched_dist_m{std::numeric_limits<double>::max()};
  };

  /// Buffered detection that arrived before any matching spawn message.
  struct PendingDetection {
    double x{0.0};
    double y{0.0};
    rclcpp::Time first_seen; ///< Timestamp of first detection
    rclcpp::Time last_seen;  ///< Timestamp of most recent confirmation
    uint32_t consecutive_scans{1}; ///< Consecutive reports confirming this
  };

  void reportCallback(
      const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg);
  void spawnCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
  void validateCallback(const std_msgs::msg::String::ConstSharedPtr &msg);

  /// Register a spawned obstacle and retroactively match pending detections.
  void registerSpawnedObstacle(SpawnedObstacle obs);
  /// Try to match a single detection against known spawned obstacles.
  /// Returns true if matched, false if buffered as pending.
  bool tryMatchDetection(double obs_x, double obs_y,
                         const rclcpp::Time &detection_stamp);
  /// Diagnostic classifier (read-only w.r.t. matching state): returns true if an
  /// unmatched segment at (x,y) sits within match_tolerance of ANY spawn (a
  /// legitimate intruder segment -- moving or stationary -- that SHOULD be
  /// unmatched), false if it is an "unclaimed" segment (a genuine static miss or
  /// noise). No time gate: a spawn position stays intruder-occupied for the run.
  /// Also updates each spawn's nearest_unmatched_dist_m for A2.
  bool classifyAndTrackUnmatched(double x, double y);
  /// Record a successful match (detection + stats) and publish the result.
  void reportMatch(SpawnedObstacle &spawned, double best_dist,
                   const rclcpp::Time &detection_stamp);
  /// Remove pending detections older than max age.
  void prunePendingDetections(const rclcpp::Time &now);
  /// Bound unmatched_id_seen_: when it exceeds kIdSeenSoftCap, drop transient
  /// ids still below the persistence threshold (they will never be counted; a
  /// genuine static feature reappears and re-accumulates). Prevents unbounded
  /// growth on long runs.
  void pruneIdSeen();
  /// Sample the robot pose from TF and record safety-circle entry for any spawn
  /// the robot has come within eligibility_radius_ of.
  void sampleRobotPose();
  /// Mark a spawn eligible (once) and update the eligible-spawn counter.
  void markEligible(SpawnedObstacle &spawned);
  /// Record the time the robot first came within the safety circle of this
  /// spawn (the latency anchor), and finalize latency if already detected.
  void recordCircleEntry(SpawnedObstacle &spawned,
                         const rclcpp::Time &entry_stamp);
  /// Compute and accumulate the signed safety-circle-relative latency once the
  /// obstacle has both been detected and observed entering the circle. Returns
  /// true the one time it finalizes, false otherwise.
  bool finalizeLatencyIfReady(SpawnedObstacle &spawned);
  /// Publish a validation_result with the current cumulative stats. `spawned`
  /// is the spawn whose event triggered the publish (for the per-event fields),
  /// or nullptr.
  void publishResult(const SpawnedObstacle *spawned);

  double match_tolerance_m_;
  double pending_max_age_s_;
  /// Max spawn->detection latency for a valid match. Kept SHORT (~seconds): the
  /// spawn position is only a valid spatial anchor while the obstacle is still
  /// near it, so a long window would re-admit a later unrelated segment that
  /// wanders back across the abandoned spawn point (the ~5s artifact).
  double match_window_s_;
  /// A spawn is eligible (counts toward the detection ratio) and its safety-
  /// circle entry is recorded once the robot passes within this radius of it.
  /// Set to the geofence safety_circle_radius so the KPI denominator matches
  /// the geofence's actual detection zone.
  double eligibility_radius_m_;
  std::string target_frame_; ///< Fixed frame the robot pose is read in (map).
  std::string robot_base_frame_; ///< Robot base frame (tf_prefix + base link).

  // TF -- robot pose for safety-circle-entry tracking
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  // Subscriptions
  rclcpp::Subscription<rises_interfaces::msg::ObstacleReport>::SharedPtr
      report_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spawn_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr validate_sub_;

  // Result publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;

  // CSV file output
  std::ofstream output_file_;

  std::mutex mutex_;
  std::vector<SpawnedObstacle> spawned_obstacles_;
  std::vector<PendingDetection> pending_detections_;

  // Running statistics
  uint32_t total_spawned_{0};    ///< All spawn events seen (informational).
  uint32_t eligible_spawned_{0}; ///< Spawns the robot came within radius of.
  uint32_t total_detected_{0};   ///< Eligible spawns that were detected.
  /// Number of obstacles with a finalized circle-relative latency. The average
  /// is over these, not over total_detected_: a detected obstacle that never
  /// entered the safety circle has no latency sample.
  uint32_t latency_samples_{0};
  double sum_latency_ms_{0.0};
  double min_latency_ms_{std::numeric_limits<double>::max()};
  double max_latency_ms_{std::numeric_limits<double>::lowest()};

  // --- Diagnostic tallies (population attribution; see plan Part A) ----------
  /// A1: cumulative matched segments across all reports (numerator of the
  /// Static Structure Match Rate).
  uint64_t matched_segments_total_{0};
  /// A1: unmatched segments that sit near a ground-truth spawn -- legitimate
  /// intruders that SHOULD be unmatched (they feed the Detection Ratio).
  uint64_t unmatched_intruder_segments_{0};
  /// A1: unmatched segments NOT near any spawn -- genuine static misses or
  /// noise. A material count here means the geofence is failing to match real
  /// static structure (a geofence issue); ~0 means the low raw match rate was
  /// purely the conflated/averaged metric (a measurement issue).
  uint64_t unmatched_unclaimed_segments_{0};
  /// A2: distance (m) within which an unmatched detection counts as "near" a
  /// missed spawn when attributing the intruder-detection gap.
  double missed_near_radius_m_{3.0};

  // --- Static Structure Match Rate (ground-truth fused) ---------------------
  /// An unmatched segment is a true STATIC MISS (real structure the map lacks)
  /// only if its persistent tracker id recurs for >= this many reports AND it is
  /// never near a ground-truth spawn. Persistence excludes moving intruders
  /// (transient ids); the spawn-proximity test excludes stationary intruders
  /// (which sit at their spawn). The residual is genuine unmapped structure.
  int persistent_min_reports_;
  /// Per persistent-tracker-id: how many reports it has appeared in. Bounded by
  /// pruneIdSeen() (transient ids, stuck below the persistence threshold, are
  /// dropped once the map grows large) so a long run does not leak memory.
  std::unordered_map<uint64_t, uint32_t> unmatched_id_seen_;
  /// Soft cap on unmatched_id_seen_ before transient ids are pruned.
  static constexpr std::size_t kIdSeenSoftCap = 50000;
  /// Cumulative unmatched segment-instances classified as true static misses.
  /// NOTE: the metric has a warm-up of persistent_min_reports_ scans -- before
  /// any id crosses the threshold it reads 0 static misses (rate 100%), then
  /// steps up. This is fine for end-of-run KPI export; live readers should
  /// expect the warm-up.
  uint64_t static_miss_segments_{0};
};

} // namespace rises
