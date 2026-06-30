#include "geofence/validation/validation_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rises {

ValidationNode::ValidationNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("validation_node", options) {
  // 2.5 m (was 1.5): moving obstacles drift from their spawn point before the
  // geofence pairs them; 2.5 m recovers those associations while the short
  // window + nearest-spawn-wins prevents cross-matching.
  this->declare_parameter<double>("match_tolerance", 2.5);
  this->declare_parameter<double>("pending_max_age", 10.0);
  // Short by design (see match_window_s_): the spawn position is only a valid
  // spatial anchor while the obstacle is still near it. 3.0 s is the largest
  // value keeping all paired latencies within the 2 s target.
  this->declare_parameter<double>("match_window", 3.0);
  this->declare_parameter<double>("eligibility_radius", 3.0);
  // Min reports an unmatched tracker id must recur in to count as a persistent
  // (static) miss rather than a transient/loitering intruder. ~12 s at 10 Hz:
  // empirically (K-sweep) the static-miss count keeps collapsing as K rises
  // (11060@K20 -> 488@K120), i.e. the residual at low K is slow/loitering
  // intruders, not map gaps. 120 is long enough that no intruder holds a single
  // tracker id at a fixed spot (while >match_tolerance from its spawn) this long,
  // isolating genuine unmapped static structure. Static match rate ~0.99 here.
  this->declare_parameter<int>("persistent_min_reports", 120);
  this->declare_parameter<std::string>("target_frame", "map");
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->declare_parameter<std::string>("tf_prefix", "");
  this->declare_parameter<std::string>("output_file", "");

  this->match_tolerance_m_ = this->get_parameter("match_tolerance").as_double();
  this->pending_max_age_s_ = this->get_parameter("pending_max_age").as_double();
  this->match_window_s_ = this->get_parameter("match_window").as_double();
  this->eligibility_radius_m_ =
      this->get_parameter("eligibility_radius").as_double();
  // A2 "near a missed spawn" radius tracks the eligibility radius so the two
  // never silently diverge when eligibility is retuned.
  this->missed_near_radius_m_ = this->eligibility_radius_m_;
  this->persistent_min_reports_ =
      static_cast<int>(this->get_parameter("persistent_min_reports").as_int());
  if (this->persistent_min_reports_ < 1) {
    RCLCPP_WARN(this->get_logger(),
                "[validation_node] persistent_min_reports=%d invalid "
                "(must be >= 1); clamping to 1",
                this->persistent_min_reports_);
    this->persistent_min_reports_ = 1;
  }
  this->target_frame_ = this->get_parameter("target_frame").as_string();
  this->robot_base_frame_ = this->get_parameter("tf_prefix").as_string() +
                            this->get_parameter("base_link_frame").as_string();
  const std::string output_path =
      this->get_parameter("output_file").as_string();

  // TF for robot-pose-based safety-circle-entry tracking.
  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  this->pose_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { this->sampleRobotPose(); });

  // Subscriptions. Detections come from the geofence's obstacle_report
  // (per-segment positions), NOT the unmatched_obstacle topic (whole-scan
  // centroid in points-only mode).
  this->report_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleReport>(
          "obstacle_report", rclcpp::QoS(10).reliable(),
          [this](
              const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg) {
            this->reportCallback(msg);
          });

  this->spawn_sub_ = this->create_subscription<std_msgs::msg::String>(
      "obstacle_spawn", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::ConstSharedPtr &msg) {
        this->spawnCallback(msg);
      });

  this->validate_sub_ = this->create_subscription<std_msgs::msg::String>(
      "obstacle_validate", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::ConstSharedPtr &msg) {
        this->validateCallback(msg);
      });

  // Publisher
  this->result_pub_ = this->create_publisher<std_msgs::msg::String>(
      "validation_result", rclcpp::QoS(10));

  // CSV file output
  if (!output_path.empty()) {
    this->output_file_.open(output_path, std::ios::out | std::ios::app);
    if (this->output_file_.is_open()) {
      this->output_file_.seekp(0, std::ios::end);
      if (this->output_file_.tellp() == 0) {
        this->output_file_ << "spawn_x,spawn_y,spawn_radius,"
                           << "spawn_time_sec,spawn_time_nsec,"
                           << "circle_entry_sec,circle_entry_nsec,"
                           << "detection_time_sec,detection_time_nsec,"
                           << "latency_ms,detected_count,eligible_count\n";
      }
      RCLCPP_INFO(this->get_logger(), "CSV output: %s", output_path.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s",
                   output_path.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "ValidationNode initialized (match_tolerance=%.2fm, "
              "pending_max_age=%.1fs, match_window=%.1fs, "
              "eligibility_radius=%.2fm, robot_frame=%s->%s)",
              this->match_tolerance_m_, this->pending_max_age_s_,
              this->match_window_s_, this->eligibility_radius_m_,
              this->target_frame_.c_str(), this->robot_base_frame_.c_str());
}

ValidationNode::~ValidationNode() {
  if (this->output_file_.is_open()) {
    this->output_file_.flush();
    this->output_file_.close();
  }
}

void ValidationNode::spawnCallback(
    const std_msgs::msg::String::ConstSharedPtr &msg) {
  try {
    const nlohmann::json j = nlohmann::json::parse(msg->data);

    SpawnedObstacle obs;
    obs.embedded_time =
        rclcpp::Time(j["timestamp"]["sec"].get<int64_t>(),
                     j["timestamp"]["nanosec"].get<uint32_t>(), RCL_ROS_TIME);
    obs.x = j["position"][0].get<double>();
    obs.y = j["position"][1].get<double>();
    obs.radius = j["radius"].get<double>();
    obs.detected = false;

    std::lock_guard<std::mutex> lock(this->mutex_);
    this->registerSpawnedObstacle(obs);

  } catch (const nlohmann::json::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse spawn JSON: %s", e.what());
  }
}

void ValidationNode::validateCallback(
    const std_msgs::msg::String::ConstSharedPtr &msg) {
  try {
    const nlohmann::json j = nlohmann::json::parse(msg->data);

    SpawnedObstacle obs;
    obs.embedded_time =
        rclcpp::Time(j["timestamp"]["sec"].get<int64_t>(),
                     j["timestamp"]["nanosec"].get<uint32_t>(), RCL_ROS_TIME);
    obs.x = j["position"][0].get<double>();
    obs.y = j["position"][1].get<double>();
    obs.radius = j["radius"].get<double>();
    obs.detected = false;

    std::lock_guard<std::mutex> lock(this->mutex_);

    // Skip if this obstacle was already registered via the spawn topic.
    for (const SpawnedObstacle &existing : this->spawned_obstacles_) {
      const double dx = obs.x - existing.x;
      const double dy = obs.y - existing.y;
      if (std::sqrt(dx * dx + dy * dy) < 0.1) {
        return;
      }
    }

    this->registerSpawnedObstacle(obs);

  } catch (const nlohmann::json::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse validate JSON: %s",
                e.what());
  }
}

void ValidationNode::registerSpawnedObstacle(SpawnedObstacle obs) {
  // Caller must hold mutex_
  this->spawned_obstacles_.push_back(obs);
  ++this->total_spawned_;

  RCLCPP_INFO(
      this->get_logger(),
      "[VALIDATION] Obstacle spawned at (%.2f, %.2f) r=%.2f [total: %u]", obs.x,
      obs.y, obs.radius, this->total_spawned_);

  // Retroactively match pending detections that arrived before this spawn.
  // Same gate as the forward path: at-or-after the spawn and within
  // match_window_s_; smallest latency wins.
  SpawnedObstacle &spawned = this->spawned_obstacles_.back();
  double best_latency_s = std::numeric_limits<double>::max();
  double best_dist = 0.0;
  std::size_t best_idx = 0;
  bool found = false;

  for (std::size_t i = 0; i < this->pending_detections_.size(); ++i) {
    const PendingDetection &pd = this->pending_detections_[i];
    const double dx = pd.x - spawned.x;
    const double dy = pd.y - spawned.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > this->match_tolerance_m_)
      continue;

    const double latency_s = (pd.first_seen - spawned.embedded_time).seconds();
    if (latency_s < 0.0 || latency_s > this->match_window_s_)
      continue;

    if (latency_s < best_latency_s) {
      best_latency_s = latency_s;
      best_dist = dist;
      best_idx = i;
      found = true;
    }
  }

  if (found) {
    const rclcpp::Time matched_stamp =
        this->pending_detections_[best_idx].first_seen;
    this->reportMatch(spawned, best_dist, matched_stamp);
    this->pending_detections_.erase(this->pending_detections_.begin() +
                                    static_cast<std::ptrdiff_t>(best_idx));
  }
}

bool ValidationNode::tryMatchDetection(double obs_x, double obs_y,
                                       const rclcpp::Time &detection_stamp) {
  // Caller must hold mutex_.
  // Match against known spawned obstacles. A detection is paired only with a
  // spawn it occurred at-or-after and within match_window_s_; among the valid
  // candidates the most recent spawn (smallest latency) wins. The short window
  // is essential: on a long recording the same location is revisited, so a wide
  // window pairs a detection to an unrelated same-position spawn.
  double best_latency_s = std::numeric_limits<double>::max();
  double best_dist = 0.0;
  SpawnedObstacle *best_match = nullptr;

  for (SpawnedObstacle &spawned : this->spawned_obstacles_) {
    if (spawned.detected)
      continue;

    const double dx = obs_x - spawned.x;
    const double dy = obs_y - spawned.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > this->match_tolerance_m_)
      continue;

    const double latency_s =
        (detection_stamp - spawned.embedded_time).seconds();
    if (latency_s < 0.0 || latency_s > this->match_window_s_)
      continue;

    if (latency_s < best_latency_s) {
      best_latency_s = latency_s;
      best_dist = dist;
      best_match = &spawned;
    }
  }

  if (best_match != nullptr) {
    this->reportMatch(*best_match, best_dist, detection_stamp);
    return true;
  }

  // No spawn match found -- buffer as pending (a detection may arrive a few
  // scans before its spawn message due to QoS/clock reordering).
  for (PendingDetection &pd : this->pending_detections_) {
    const double dx = obs_x - pd.x;
    const double dy = obs_y - pd.y;
    if (std::sqrt(dx * dx + dy * dy) <= this->match_tolerance_m_) {
      pd.x = (pd.x * pd.consecutive_scans + obs_x) / (pd.consecutive_scans + 1);
      pd.y = (pd.y * pd.consecutive_scans + obs_y) / (pd.consecutive_scans + 1);
      pd.last_seen = detection_stamp;
      ++pd.consecutive_scans;
      return false;
    }
  }

  PendingDetection pd;
  pd.x = obs_x;
  pd.y = obs_y;
  pd.first_seen = detection_stamp;
  pd.last_seen = detection_stamp;
  pd.consecutive_scans = 1;
  this->pending_detections_.push_back(pd);
  return false;
}

void ValidationNode::reportMatch(SpawnedObstacle &spawned, double best_dist,
                                 const rclcpp::Time &detection_stamp) {
  // Caller must hold mutex_.
  // A detection means the obstacle was inside the geofence's view, so it is
  // eligible by definition (covers a fast fly-by between pose samples).
  this->markEligible(spawned);
  spawned.detected = true;
  spawned.first_detection_stamp = detection_stamp;
  spawned.match_distance_m = best_dist;
  ++this->total_detected_;

  RCLCPP_INFO(this->get_logger(),
              "[VALIDATION] DETECTED spawned obstacle at (%.2f, %.2f) "
              "[detected %u/%u eligible]",
              spawned.x, spawned.y, this->total_detected_,
              this->eligible_spawned_);

  // Finalize latency now if circle entry was already observed; otherwise it is
  // finalized later in recordCircleEntry (detected-before-entry = lead time).
  this->finalizeLatencyIfReady(spawned);
  this->publishResult(&spawned);
}

void ValidationNode::recordCircleEntry(SpawnedObstacle &spawned,
                                       const rclcpp::Time &entry_stamp) {
  // Caller must hold mutex_. The robot has come within the safety circle
  // (eligibility_radius) of this spawn -- the latency anchor t0.
  this->markEligible(spawned);
  if (spawned.entered_circle)
    return;
  spawned.entered_circle = true;
  spawned.circle_entry_stamp = entry_stamp;

  if (this->finalizeLatencyIfReady(spawned))
    this->publishResult(&spawned);
}

bool ValidationNode::finalizeLatencyIfReady(SpawnedObstacle &spawned) {
  // Caller must hold mutex_. Latency is defined only once the obstacle has both
  // been detected AND observed entering the safety circle; recorded once.
  if (spawned.latency_finalized || !spawned.detected || !spawned.entered_circle)
    return false;

  // Signed: detection BEFORE circle entry -> negative (lead time, the desired
  // case); detection AFTER entry -> positive (reaction lag). Approach time
  // before the obstacle reached the circle is excluded.
  const double latency_ms =
      (spawned.first_detection_stamp - spawned.circle_entry_stamp).seconds() *
      1000.0;
  spawned.signed_latency_ms = latency_ms;
  spawned.latency_finalized = true;

  this->sum_latency_ms_ += latency_ms;
  this->min_latency_ms_ = std::min(this->min_latency_ms_, latency_ms);
  this->max_latency_ms_ = std::max(this->max_latency_ms_, latency_ms);
  ++this->latency_samples_;

  RCLCPP_INFO(this->get_logger(),
              "[VALIDATION] latency at (%.2f, %.2f): %.1fms (%s safety-circle "
              "entry) [%u samples, avg=%.1fms]",
              spawned.x, spawned.y, latency_ms,
              latency_ms < 0.0 ? "before" : "after", this->latency_samples_,
              this->sum_latency_ms_ /
                  static_cast<double>(this->latency_samples_));

  // One CSV row per finalized latency sample.
  if (this->output_file_.is_open()) {
    // Split each stamp via integer ns to avoid the double rounding of
    // Time::seconds() (epoch ns exceeds the 53-bit double mantissa).
    const int64_t spawn_sec = spawned.embedded_time.nanoseconds() / 1000000000LL;
    const uint32_t spawn_nsec = static_cast<uint32_t>(
        spawned.embedded_time.nanoseconds() % 1000000000LL);
    const int64_t entry_sec =
        spawned.circle_entry_stamp.nanoseconds() / 1000000000LL;
    const uint32_t entry_nsec = static_cast<uint32_t>(
        spawned.circle_entry_stamp.nanoseconds() % 1000000000LL);
    const int64_t det_sec =
        spawned.first_detection_stamp.nanoseconds() / 1000000000LL;
    const uint32_t det_nsec = static_cast<uint32_t>(
        spawned.first_detection_stamp.nanoseconds() % 1000000000LL);
    this->output_file_ << spawned.x << "," << spawned.y << "," << spawned.radius
                       << "," << spawn_sec << "," << spawn_nsec << ","
                       << entry_sec << "," << entry_nsec << "," << det_sec
                       << "," << det_nsec << "," << latency_ms << ","
                       << this->total_detected_ << "," << this->eligible_spawned_
                       << "\n";
    this->output_file_.flush();
  }
  return true;
}

void ValidationNode::publishResult(const SpawnedObstacle *spawned) {
  // Caller must hold mutex_. Emits current cumulative stats so the latest
  // validation_result always carries up-to-date KPI inputs.
  const bool have_latency = this->latency_samples_ > 0;
  const double avg_latency_ms =
      have_latency
          ? this->sum_latency_ms_ / static_cast<double>(this->latency_samples_)
          : 0.0;

  nlohmann::json result;
  result["event"] = "detection";
  if (spawned != nullptr) {
    result["spawn_position"] = {spawned->x, spawned->y};
    result["spawn_radius"] = spawned->radius;
    result["distance_m"] = spawned->match_distance_m;
    result["latency_ms"] =
        spawned->latency_finalized ? nlohmann::json(spawned->signed_latency_ms)
                                   : nlohmann::json(nullptr);
  }
  // Diagnostic (A2): of the eligible-but-undetected spawns, how many had an
  // unmatched detection come near them (association missed it -> validation
  // issue) vs none at all (geofence never saw it -> geofence issue).
  uint32_t missed_near_assoc = 0;
  uint32_t missed_never_seen = 0;
  for (const SpawnedObstacle &s : this->spawned_obstacles_) {
    if (!s.eligible || s.detected)
      continue;
    if (s.nearest_unmatched_dist_m <= this->missed_near_radius_m_)
      ++missed_near_assoc;
    else
      ++missed_never_seen;
  }

  // avg_latency_ms = safety-circle-relative detection latency (the KPI
  // export_kpis.py reads). spawned / detected feed Detection Ratio.
  // matched_segments_total + unmatched_unclaimed_segments feed the Static
  // Structure Match Rate; unmatched_intruder_segments are legitimate intruders.
  result["stats"] = {
      {"detected", this->total_detected_},
      {"spawned", this->eligible_spawned_},
      {"total_spawned", this->total_spawned_},
      {"latency_samples", this->latency_samples_},
      {"avg_latency_ms", avg_latency_ms},
      {"min_latency_ms", have_latency ? this->min_latency_ms_ : 0.0},
      {"max_latency_ms", have_latency ? this->max_latency_ms_ : 0.0},
      {"matched_segments_total", this->matched_segments_total_},
      {"unmatched_intruder_segments", this->unmatched_intruder_segments_},
      {"unmatched_unclaimed_segments", this->unmatched_unclaimed_segments_},
      {"static_miss_segments", this->static_miss_segments_},
      {"missed_near_assoc", missed_near_assoc},
      {"missed_never_seen", missed_never_seen}};

  std::unique_ptr<std_msgs::msg::String> result_msg =
      std::make_unique<std_msgs::msg::String>();
  result_msg->data = result.dump();
  this->result_pub_->publish(std::move(result_msg));
}

void ValidationNode::prunePendingDetections(const rclcpp::Time &now) {
  // Caller must hold mutex_
  this->pending_detections_.erase(
      std::remove_if(
          this->pending_detections_.begin(), this->pending_detections_.end(),
          [&](const PendingDetection &pd) {
            return (now - pd.last_seen).seconds() > this->pending_max_age_s_;
          }),
      this->pending_detections_.end());
}

void ValidationNode::pruneIdSeen() {
  // Caller must hold mutex_. Only triggers on long runs; transient ids stuck
  // below the persistence threshold are dropped (a genuine static feature
  // reappears and re-accumulates), bounding memory.
  if (this->unmatched_id_seen_.size() <= kIdSeenSoftCap)
    return;
  const uint32_t k = static_cast<uint32_t>(this->persistent_min_reports_);
  for (std::unordered_map<uint64_t, uint32_t>::iterator it =
           this->unmatched_id_seen_.begin();
       it != this->unmatched_id_seen_.end();) {
    if (it->second < k)
      it = this->unmatched_id_seen_.erase(it);
    else
      ++it;
  }
}

void ValidationNode::markEligible(SpawnedObstacle &spawned) {
  // Caller must hold mutex_
  if (spawned.eligible)
    return;
  spawned.eligible = true;
  ++this->eligible_spawned_;
}

void ValidationNode::sampleRobotPose() {
  // Read the latest robot pose and record safety-circle entry for any spawn the
  // robot is now within eligibility_radius of. Spawns are retained for the
  // whole run, so entry means the robot came within range at ANY time.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = this->tf_buffer_->lookupTransform(
        this->target_frame_, this->robot_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &e) {
    // Transient at startup, but a persistent failure (e.g. misconfigured frame
    // names) silently leaves eligible_spawned_ at 0 and zeroes the detection
    // ratio -- so surface it, rate-limited.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[validation_node] sampleRobotPose: TF lookup "
                         "%s->%s failed: %s",
                         this->target_frame_.c_str(),
                         this->robot_base_frame_.c_str(), e.what());
    return;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double r2 = this->eligibility_radius_m_ * this->eligibility_radius_m_;

  // Time of this pose = the safety-circle crossing time (the latency anchor).
  // Fall back to the node clock if the transform carries no stamp.
  rclcpp::Time entry_stamp(tf.header.stamp);
  if (entry_stamp.nanoseconds() == 0)
    entry_stamp = this->now();

  std::lock_guard<std::mutex> lock(this->mutex_);
  for (SpawnedObstacle &spawned : this->spawned_obstacles_) {
    if (spawned.entered_circle)
      continue;
    const double dx = rx - spawned.x;
    const double dy = ry - spawned.y;
    if (dx * dx + dy * dy <= r2)
      this->recordCircleEntry(spawned, entry_stamp);
  }
}

bool ValidationNode::classifyAndTrackUnmatched(double x, double y) {
  // Caller holds mutex_. Read-only w.r.t. matching state: classifies an
  // unmatched segment as a legitimate intruder (spatially near any spawn) vs an
  // "unclaimed" segment (genuine static miss / noise), and tracks the closest
  // any unmatched detection ever came to each spawn (A2).
  const double tol2 = this->match_tolerance_m_ * this->match_tolerance_m_;
  bool near_spawn = false;
  for (SpawnedObstacle &s : this->spawned_obstacles_) {
    const double dx = x - s.x;
    const double dy = y - s.y;
    const double dist2 = dx * dx + dy * dy;
    const double dist = std::sqrt(dist2);
    if (dist < s.nearest_unmatched_dist_m)
      s.nearest_unmatched_dist_m = dist;
    // Spatial proximity only -- NO time gate. A spawn position is "occupied" by
    // its intruder for the whole run (a stationary intruder never leaves it; a
    // moving one started there), so any unmatched segment near a spawn is
    // intruder-attributable and must not be counted as a static miss. A time
    // gate here would wrongly reclassify a stationary intruder as a static miss
    // once the match window elapsed.
    if (dist2 <= tol2)
      near_spawn = true;
  }
  return near_spawn;
}

void ValidationNode::reportCallback(
    const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg) {
  if (!msg)
    return;

  const rclcpp::Time detection_stamp(msg->header.stamp);

  std::lock_guard<std::mutex> lock(this->mutex_);

  // Count matched segments from EVERY report (including all-matched reports with
  // no unmatched entries) so the Static Structure Match Rate denominator is
  // complete -- hence this runs before the empty-unmatched early return.
  this->matched_segments_total_ +=
      static_cast<uint64_t>(msg->matched_obstacles.size());

  if (msg->unmatched_obstacles.empty())
    return;

  // Each unmatched_obstacles entry is a per-cluster segment whose .position is
  // the segment midpoint at the obstacle's real location -- the correct signal
  // for per-obstacle matching (unlike the whole-scan-centroid topic).
  for (const rises_interfaces::msg::Obstacle &obstacle : msg->unmatched_obstacles) {
    // Is this unmatched segment near a ground-truth spawn (a legitimate
    // intruder) or not (an "unclaimed" segment)?
    if (this->classifyAndTrackUnmatched(obstacle.position.x,
                                        obstacle.position.y)) {
      ++this->unmatched_intruder_segments_;
    } else {
      ++this->unmatched_unclaimed_segments_;
      // True STATIC MISS: an unclaimed segment whose persistent tracker id
      // recurs for >= persistent_min_reports_ reports is real unmapped static
      // structure. A moving intruder's id is transient (re-IDed on drift); a
      // stationary intruder sits at its spawn (caught by the near-spawn test
      // above). Count every instance of a confirmed-persistent id -- crediting
      // the pre-threshold ones retroactively -- so it is comparable to
      // matched_segments_total.
      const uint64_t tid = obstacle.id;
      if (tid != static_cast<uint64_t>(-1)) {
        const uint32_t seen = ++this->unmatched_id_seen_[tid];
        const uint32_t k = static_cast<uint32_t>(this->persistent_min_reports_);
        if (seen == k)
          this->static_miss_segments_ += k;
        else if (seen > k)
          ++this->static_miss_segments_;
      }
    }

    this->tryMatchDetection(obstacle.position.x, obstacle.position.y,
                            detection_stamp);
  }

  this->prunePendingDetections(detection_stamp);
  this->pruneIdSeen();
}

} // namespace rises

RCLCPP_COMPONENTS_REGISTER_NODE(rises::ValidationNode)
