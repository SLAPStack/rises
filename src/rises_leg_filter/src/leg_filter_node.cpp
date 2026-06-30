/// @file leg_filter_node.cpp
/// @brief Filters unmatched geofence obstacles for likely human leg-pairs.

#include "rises_leg_filter/leg_filter_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rises {

LegFilterNode::LegFilterNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("rises_leg_filter", options) {
  // Width filter: typical human leg-pair seen by 2D LiDAR
  this->declare_parameter("min_width", 0.15);
  this->declare_parameter("max_width", 0.8);

  // Velocity and persistence thresholds
  this->declare_parameter("min_velocity", 0.05);
  this->declare_parameter("stationary_timeout_sec", 5.0);
  this->declare_parameter("eviction_timeout_sec", 3.0);
  this->declare_parameter("min_observations", 5);

  // Publishing
  this->declare_parameter("publish_rate_hz", 5.0);
  this->declare_parameter("frame_id", std::string("map"));

  // Confidence levels
  this->declare_parameter("base_confidence", 0.3);
  this->declare_parameter("boosted_confidence", 0.7);

  // Camera fusion
  this->declare_parameter("camera_match_radius", 1.5);
  this->declare_parameter("camera_stale_sec", 2.0);
  // Camera-fusion gate. Default OFF until the TF lookup on body_<id> frames
  // replaces the (0,0) placeholder in humansTrackedCallback; with the
  // placeholder live, any leg near map origin would be spuriously marked
  // confirmed_by_camera. Audit finding #12.
  this->declare_parameter("enable_camera_fusion", false);

  // Cache all parameters
  this->min_width_ = this->get_parameter("min_width").as_double();
  this->max_width_ = this->get_parameter("max_width").as_double();
  this->min_velocity_ = this->get_parameter("min_velocity").as_double();
  this->stationary_timeout_ =
      this->get_parameter("stationary_timeout_sec").as_double();
  this->eviction_timeout_ =
      this->get_parameter("eviction_timeout_sec").as_double();
  this->min_observations_ = this->get_parameter("min_observations").as_int();
  this->publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
  this->frame_id_ = this->get_parameter("frame_id").as_string();
  this->base_confidence_ = this->get_parameter("base_confidence").as_double();
  this->boosted_confidence_ =
      this->get_parameter("boosted_confidence").as_double();
  this->camera_match_radius_ =
      this->get_parameter("camera_match_radius").as_double();
  this->camera_stale_sec_ = this->get_parameter("camera_stale_sec").as_double();
  this->enable_camera_fusion_ =
      this->get_parameter("enable_camera_fusion").as_bool();

  // Subscribe to the geofence's obstacle_report; we only read its
  // unmatched_obstacles[] array. RELIABLE QoS matches the producer
  // (lifecycle_geofence_node_base.cpp) and this topic's other subscribers
  // (validation_node, fiware_bridge_node).
  this->obstacle_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleReport>(
          "obstacle_report", rclcpp::QoS(10).reliable(),
          std::bind(&LegFilterNode::obstacleCallback, this,
                    std::placeholders::_1));

  // Subscribe to camera-confirmed humans (ROS4HRI /humans/bodies/tracked).
  // When hri_msgs is available, change type to hri_msgs::msg::IdsList.
  // For now we accept a comma-separated string of body IDs and resolve
  // positions via TF lookup at match time.
  this->humans_tracked_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/humans/bodies/tracked", rclcpp::QoS(10).reliable(),
      std::bind(&LegFilterNode::humansTrackedCallback, this,
                std::placeholders::_1));

  // Publish detected body IDs
  this->bodies_tracked_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/humans/bodies/lidar_tracked", rclcpp::QoS(10).reliable());

  // TF broadcaster for body frames
  this->tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Periodic evaluation and publishing
  const std::chrono::duration<double> period(1.0 / this->publish_rate_hz_);
  this->publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LegFilterNode::publishCandidates, this));

  RCLCPP_INFO(
      this->get_logger(),
      "Leg filter initialized. Width: [%.2f, %.2f]m, min velocity: %.2f m/s, "
      "base confidence: %.1f, boosted: %.1f, camera match radius: %.1fm",
      this->min_width_, this->max_width_, this->min_velocity_,
      this->base_confidence_, this->boosted_confidence_,
      this->camera_match_radius_);
}

void LegFilterNode::obstacleCallback(
    const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg) {
  const double timestamp_sec = rclcpp::Time(msg->header.stamp).seconds();

  // Hold state_mutex_ across the entire batch update + eviction sequence.
  // No ROS I/O happens here; the critical section is bounded by
  // msg->unmatched_obstacles.size().
  std::lock_guard<std::mutex> guard(this->state_mutex_);

  // Only unmatched obstacles are candidate legs -- a matched obstacle is a
  // known map feature, not something the geofence failed to explain.
  for (const rises_interfaces::msg::Obstacle &obs : msg->unmatched_obstacles) {
    LegTrack &track = this->tracks_[obs.id];

    // Width gate: reject immediately if obstacle is too wide or too narrow
    const double width = static_cast<double>(obs.width);
    if (width < this->min_width_ || width > this->max_width_) {
      track.is_candidate = false;
      track.last_seen_sec = timestamp_sec;
      continue;
    }

    const double x = obs.position.x;
    const double y = obs.position.y;

    // First observation for this track
    if (track.observation_count == 0) {
      track.first_seen_sec = timestamp_sec;
    }

    // Accumulate displacement for velocity estimation
    if (track.has_prev) {
      const double dx = x - track.prev_x;
      const double dy = y - track.prev_y;
      track.displacement_sum += std::sqrt(dx * dx + dy * dy);
    }

    track.prev_x = track.last_x;
    track.prev_y = track.last_y;
    track.has_prev = track.observation_count > 0;

    track.last_x = x;
    track.last_y = y;
    track.last_seen_sec = timestamp_sec;
    track.time_span = timestamp_sec - track.first_seen_sec;
    ++track.observation_count;
  }

  this->evictStaleTracks(timestamp_sec);
}

void LegFilterNode::humansTrackedCallback(
    const std_msgs::msg::String::ConstSharedPtr &msg) {
  // Camera-confirmed human body IDs arrive here.
  // In a full ROS4HRI setup this would be hri_msgs::msg::IdsList.
  // For now we just note that camera-confirmed humans exist — their
  // positions are resolved via TF (body_<id> frame) in matchesCameraHuman().
  //
  // We store the IDs with a timestamp so we can expire stale confirmations.
  const double now_sec = this->now().seconds();

  // Parse comma-separated IDs (placeholder format). Do the parse outside the
  // lock; only the map writes need protection.
  std::istringstream stream(msg->data);
  std::string body_id;

  std::lock_guard<std::mutex> guard(this->state_mutex_);
  while (std::getline(stream, body_id, ',')) {
    if (!body_id.empty()) {
      ConfirmedHuman &human = this->confirmed_humans_[body_id];
      human.timestamp_sec = now_sec;
      // Position would be resolved from TF frame body_<id>.
      // For now we store (0,0) — matchesCameraHuman() is a placeholder
      // until TF buffer integration is added with the full hri_msgs.
    }
  }
}

bool LegFilterNode::passesLegFilter(const LegTrack &track,
                                    double current_sec) const {
  // Need enough observations to judge
  if (track.observation_count < this->min_observations_) {
    return false;
  }

  // Must have moved above minimum velocity
  if (track.time_span > 0.1) {
    const double avg_velocity = track.displacement_sum / track.time_span;
    if (avg_velocity >= this->min_velocity_) {
      return true;
    }
  }

  // If stationary for too long, reject — likely a pallet or pole
  const double stationary_duration = current_sec - track.first_seen_sec;
  if (track.displacement_sum < 0.1 &&
      stationary_duration > this->stationary_timeout_) {
    return false;
  }

  // Camera-confirmed obstacles pass even if stationary (human standing still)
  if (track.confirmed_by_camera) {
    return true;
  }

  return false;
}

bool LegFilterNode::matchesCameraHuman(double x, double y,
                                       double current_sec) const {
  // Camera-fusion gate. With the TF lookup on body_<id> still unimplemented,
  // confirmed_humans_ entries hold the (0,0) placeholder position. Treating
  // every leg near map origin as camera-confirmed is a safety hazard, so this
  // path stays disabled until the real TF lookup lands. See audit #12.
  if (!this->enable_camera_fusion_) {
    return false;
  }

  // Check if any camera-confirmed human is within match radius.
  // In a full implementation this would do a TF lookup for each body_<id>
  // frame. For now, confirmed_humans_ stores positions from TF or (0,0) as
  // placeholder.
  const double radius_sq =
      this->camera_match_radius_ * this->camera_match_radius_;

  for (const std::pair<const std::string, ConfirmedHuman> &entry :
       this->confirmed_humans_) {
    const std::string &id = entry.first;
    const ConfirmedHuman &human = entry.second;
    if (current_sec - human.timestamp_sec > this->camera_stale_sec_) {
      continue; // stale
    }
    const double dx = x - human.x;
    const double dy = y - human.y;
    if (dx * dx + dy * dy < radius_sq) {
      return true;
    }
  }
  return false;
}

void LegFilterNode::publishCandidates() {
  const double current_sec = this->now().seconds();

  // Snapshot of the data we need to publish for one passing track. Storing a
  // raw value snapshot (not pointers into tracks_) means the rest of this
  // function can drop state_mutex_ before doing any ROS I/O.
  struct CandidateSnapshot {
    uint64_t id;
    double last_x;
    double last_y;
    bool confirmed_by_camera;
  };
  std::vector<CandidateSnapshot> candidates;

  {
    std::lock_guard<std::mutex> guard(this->state_mutex_);
    candidates.reserve(this->tracks_.size());
    for (std::pair<const uint64_t, LegTrack> &entry : this->tracks_) {
      const uint64_t id = entry.first;
      LegTrack &track = entry.second;
      // Update camera confirmation status (writes back into the live track).
      track.confirmed_by_camera =
          this->matchesCameraHuman(track.last_x, track.last_y, current_sec);

      track.is_candidate = this->passesLegFilter(track, current_sec);

      if (track.is_candidate) {
        candidates.push_back(
            {id, track.last_x, track.last_y, track.confirmed_by_camera});
      }
    }
  }

  if (candidates.empty()) {
    return;
  }

  // Publish body IDs list (lock released — safe to do I/O here).
  std_msgs::msg::String ids_msg;
  std::ostringstream ids_stream;
  bool first = true;
  for (const CandidateSnapshot &candidate : candidates) {
    if (!first) {
      ids_stream << ",";
    }
    ids_stream << "lidar_" << candidate.id;
    first = false;
  }
  ids_msg.data = ids_stream.str();
  this->bodies_tracked_pub_->publish(ids_msg);

  // Broadcast TF frames for each candidate
  const rclcpp::Time stamp = this->now();
  for (const CandidateSnapshot &candidate : candidates) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = this->frame_id_;
    tf.child_frame_id = "body_lidar_" + std::to_string(candidate.id);

    tf.transform.translation.x = candidate.last_x;
    tf.transform.translation.y = candidate.last_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    this->tf_broadcaster_->sendTransform(tf);
  }

  RCLCPP_DEBUG(
      this->get_logger(), "Published %zu leg candidates (%zu camera-boosted)",
      candidates.size(),
      static_cast<std::size_t>(std::count_if(
          candidates.begin(), candidates.end(),
          [](const CandidateSnapshot &c) { return c.confirmed_by_camera; })));
}

void LegFilterNode::evictStaleTracks(double current_sec) {
  auto it = this->tracks_.begin();
  while (it != this->tracks_.end()) {
    if (current_sec - it->second.last_seen_sec > this->eviction_timeout_) {
      it = this->tracks_.erase(it);
    } else {
      ++it;
    }
  }

  // Also evict stale camera confirmations
  auto cit = this->confirmed_humans_.begin();
  while (cit != this->confirmed_humans_.end()) {
    if (current_sec - cit->second.timestamp_sec >
        this->camera_stale_sec_ * 2.0) {
      cit = this->confirmed_humans_.erase(cit);
    } else {
      ++cit;
    }
  }
}

} // namespace rises
