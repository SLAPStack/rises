/// @file heatmap_predictor_node.cpp
/// @brief Obstacle heatmap predictor implementation.
///
/// Algorithm:
///   1. Receive unmatched obstacles from geofence node (with persistent IDs)
///   2. Maintain a sliding window of position observations per obstacle ID
///   3. Estimate linear velocity via least-squares regression on the trajectory
///   4. Project positions forward at prediction_step intervals up to
///   prediction_horizon
///   5. Stamp each projected position as a Gaussian blob on an OccupancyGrid
///   6. Normalize and publish the grid

#include "obstacle_heatmap_predictor/heatmap_predictor_node.hpp"

#include <algorithm>
#include <cmath>

namespace rises {

HeatmapPredictorNode::HeatmapPredictorNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("obstacle_heatmap_predictor", options) {
  // Observation and prediction parameters
  this->declare_parameter("observation_window_sec", 10.0);
  this->declare_parameter("prediction_horizon_sec", 30.0);
  this->declare_parameter("prediction_step_sec", 2.0);
  this->declare_parameter("eviction_timeout_sec", 5.0);
  this->declare_parameter("min_observations", 3);

  // Grid parameters
  this->declare_parameter("grid_resolution", 0.25);
  this->declare_parameter("grid_width", 200.0);
  this->declare_parameter("grid_height", 200.0);
  // grid_center_x/y: world-space center of the occupancy grid.
  // Set these to the approximate center of your warehouse.
  // grid_origin (bottom-left) = center - half_size, so origin defaults to
  // (-100, -100).
  this->declare_parameter("grid_center_x", 0.0);
  this->declare_parameter("grid_center_y", 0.0);
  this->declare_parameter("frame_id", std::string("map"));

  // Prediction spread
  this->declare_parameter("gaussian_sigma", 1.0);
  this->declare_parameter("publish_rate_hz", 2.0);

  // Cache parameters
  this->observation_window_sec_ =
      this->get_parameter("observation_window_sec").as_double();
  this->prediction_horizon_sec_ =
      this->get_parameter("prediction_horizon_sec").as_double();
  this->prediction_step_sec_ =
      this->get_parameter("prediction_step_sec").as_double();
  this->eviction_timeout_sec_ =
      this->get_parameter("eviction_timeout_sec").as_double();
  this->min_observations_ = this->get_parameter("min_observations").as_int();
  this->grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  this->grid_width_ = this->get_parameter("grid_width").as_double();
  this->grid_height_ = this->get_parameter("grid_height").as_double();
  const double grid_center_x = this->get_parameter("grid_center_x").as_double();
  const double grid_center_y = this->get_parameter("grid_center_y").as_double();
  this->grid_origin_x_ = grid_center_x - this->grid_width_ * 0.5;
  this->grid_origin_y_ = grid_center_y - this->grid_height_ * 0.5;
  this->frame_id_ = this->get_parameter("frame_id").as_string();
  this->gaussian_sigma_ = this->get_parameter("gaussian_sigma").as_double();
  this->publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

  this->grid_cols_ =
      static_cast<int>(this->grid_width_ / this->grid_resolution_);
  this->grid_rows_ =
      static_cast<int>(this->grid_height_ / this->grid_resolution_);

  // Subscribe to obstacle reports from geofence node
  this->obstacle_report_sub_ =
      this->create_subscription<rises_interfaces::msg::ObstacleReport>(
          "obstacle_report", rclcpp::SensorDataQoS().keep_last(10),
          std::bind(&HeatmapPredictorNode::obstacleReportCallback, this,
                    std::placeholders::_1));

  // Publish prediction heatmap (volatile QoS so RViz updates every frame)
  this->heatmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "predicted_occupancy", rclcpp::QoS(1).reliable());

  // Periodic publishing timer
  const auto period =
      std::chrono::duration<double>(1.0 / this->publish_rate_hz_);
  this->publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HeatmapPredictorNode::publishHeatmap, this));

  RCLCPP_INFO(
      this->get_logger(),
      "Obstacle Heatmap Predictor initialized. Grid: %dx%d (%.1fm x %.1fm @ "
      "%.2fm/cell), "
      "observation window: %.0fs, prediction horizon: %.0fs, sigma: %.1fm",
      this->grid_cols_, this->grid_rows_, this->grid_width_, this->grid_height_,
      this->grid_resolution_, this->observation_window_sec_,
      this->prediction_horizon_sec_, this->gaussian_sigma_);
}

void HeatmapPredictorNode::obstacleReportCallback(
    const rises_interfaces::msg::ObstacleReport::ConstSharedPtr &msg) {
  const double timestamp_sec = rclcpp::Time(msg->header.stamp).seconds();

  // Hold tracks_mutex_ across the entire update + eviction sequence so the
  // map's internal structure cannot be observed mid-rehash by publishHeatmap.
  // updateTrack / evictStaleTracks do not perform any ROS I/O, so the
  // critical section is short and bounded by msg->unmatched_obstacles.size().
  std::lock_guard<std::mutex> guard(this->tracks_mutex_);

  for (const rises_interfaces::msg::Obstacle &obstacle :
       msg->unmatched_obstacles) {
    this->updateTrack(obstacle.id, obstacle.position.x, obstacle.position.y,
                      timestamp_sec);
  }

  this->evictStaleTracks(timestamp_sec);
}

void HeatmapPredictorNode::updateTrack(uint64_t id, double x, double y,
                                       double timestamp_sec) {
  TrackedObstacle &track = this->tracked_obstacles_[id];
  track.history.push_back({x, y, timestamp_sec});
  track.last_seen_sec = timestamp_sec;

  // Trim observations outside the sliding window
  const double cutoff = timestamp_sec - this->observation_window_sec_;
  while (!track.history.empty() &&
         track.history.front().timestamp_sec < cutoff) {
    track.history.pop_front();
  }

  // Re-estimate velocity when we have enough observations
  if (static_cast<int>(track.history.size()) >= this->min_observations_) {
    this->estimateVelocity(track);
  }
}

void HeatmapPredictorNode::estimateVelocity(TrackedObstacle &track) const {
  // Least-squares linear regression: position = p0 + velocity * (t - t0)
  // Using the first observation as t0 to reduce floating-point error.
  const double t0 = track.history.front().timestamp_sec;
  const int n = static_cast<int>(track.history.size());

  double sum_dt = 0.0;
  double sum_dt2 = 0.0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_dt_x = 0.0;
  double sum_dt_y = 0.0;

  for (const Observation &obs : track.history) {
    const double dt = obs.timestamp_sec - t0;
    sum_dt += dt;
    sum_dt2 += dt * dt;
    sum_x += obs.x;
    sum_y += obs.y;
    sum_dt_x += dt * obs.x;
    sum_dt_y += dt * obs.y;
  }

  const double denom = static_cast<double>(n) * sum_dt2 - sum_dt * sum_dt;

  // If denominator is near zero, all observations are at the same time — no
  // velocity
  if (std::abs(denom) < 1e-9) {
    track.velocity_x = 0.0;
    track.velocity_y = 0.0;
    return;
  }

  track.velocity_x =
      (static_cast<double>(n) * sum_dt_x - sum_dt * sum_x) / denom;
  track.velocity_y =
      (static_cast<double>(n) * sum_dt_y - sum_dt * sum_y) / denom;

  // Clamp velocity to reasonable human walking speed (2.0 m/s max)
  constexpr double max_speed = 2.0;
  const double speed = std::sqrt(track.velocity_x * track.velocity_x +
                                 track.velocity_y * track.velocity_y);
  if (speed > max_speed) {
    const double scale = max_speed / speed;
    track.velocity_x *= scale;
    track.velocity_y *= scale;
  }
}

void HeatmapPredictorNode::evictStaleTracks(double current_time_sec) {
  auto it = this->tracked_obstacles_.begin();
  while (it != this->tracked_obstacles_.end()) {
    if (current_time_sec - it->second.last_seen_sec >
        this->eviction_timeout_sec_) {
      it = this->tracked_obstacles_.erase(it);
    } else {
      ++it;
    }
  }
}

void HeatmapPredictorNode::publishHeatmap() {
  const int total_cells = this->grid_rows_ * this->grid_cols_;
  std::vector<float> probability_grid(static_cast<std::size_t>(total_cells),
                                      0.0f);

  const int prediction_steps = static_cast<int>(this->prediction_horizon_sec_ /
                                                this->prediction_step_sec_);

  // Snapshot tracked_obstacles_ under tracks_mutex_ into a flat vector of the
  // fields we actually need (latest position + velocity). This keeps the
  // critical section to a single linear pass over the map and releases the
  // lock before the O(tracks * steps * radius^2) Gaussian stamping and before
  // any ROS publish. obstacleReportCallback can therefore mutate the live map
  // in parallel with our stamping/publish, which is the whole point.
  struct TrackSnapshot {
    double latest_x;
    double latest_y;
    double velocity_x;
    double velocity_y;
  };
  std::vector<TrackSnapshot> snapshots;
  {
    std::lock_guard<std::mutex> guard(this->tracks_mutex_);
    snapshots.reserve(this->tracked_obstacles_.size());
    for (const auto &[id, track] : this->tracked_obstacles_) {
      if (static_cast<int>(track.history.size()) < this->min_observations_) {
        continue;
      }
      const Observation &latest = track.history.back();
      snapshots.push_back(
          {latest.x, latest.y, track.velocity_x, track.velocity_y});
    }
  }

  for (const TrackSnapshot &snap : snapshots) {
    // Stamp current position with full weight
    this->stampGaussian(probability_grid, snap.latest_x, snap.latest_y, 1.0f);

    // Project forward in time
    for (int step = 1; step <= prediction_steps; ++step) {
      const double dt = static_cast<double>(step) * this->prediction_step_sec_;
      const double predicted_x = snap.latest_x + snap.velocity_x * dt;
      const double predicted_y = snap.latest_y + snap.velocity_y * dt;

      // Confidence decays linearly with prediction time
      const float confidence =
          static_cast<float>(1.0 - dt / this->prediction_horizon_sec_);

      // Prediction spread increases with time (wider Gaussian for further
      // predictions) This is handled by stampGaussian using gaussian_sigma_ as
      // base, but we scale weight down so farther predictions contribute less.
      this->stampGaussian(probability_grid, predicted_x, predicted_y,
                          std::max(0.05f, confidence));
    }
  }

  // Find max value for normalization
  float max_val = 0.0f;
  for (float val : probability_grid) {
    max_val = std::max(max_val, val);
  }

  // Build OccupancyGrid message
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = this->frame_id_;

  grid_msg.info.resolution = static_cast<float>(this->grid_resolution_);
  grid_msg.info.width = static_cast<uint32_t>(this->grid_cols_);
  grid_msg.info.height = static_cast<uint32_t>(this->grid_rows_);
  grid_msg.info.origin.position.x = this->grid_origin_x_;
  grid_msg.info.origin.position.y = this->grid_origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data.resize(static_cast<std::size_t>(total_cells));

  if (max_val > 0.0f) {
    const float inv_max = 100.0f / max_val;
    for (int i = 0; i < total_cells; ++i) {
      grid_msg.data[static_cast<std::size_t>(i)] = static_cast<int8_t>(
          probability_grid[static_cast<std::size_t>(i)] * inv_max);
    }
  }

  this->heatmap_pub_->publish(grid_msg);
}

void HeatmapPredictorNode::stampGaussian(std::vector<float> &grid, double x,
                                         double y, float weight) const {
  // Convert world coordinates to grid coordinates
  const double gx = (x - this->grid_origin_x_) / this->grid_resolution_;
  const double gy = (y - this->grid_origin_y_) / this->grid_resolution_;

  // Gaussian kernel radius in cells (3 sigma covers 99.7% of distribution)
  const double sigma_cells = this->gaussian_sigma_ / this->grid_resolution_;
  const int radius = static_cast<int>(std::ceil(3.0 * sigma_cells));
  const double inv_2sigma2 = 1.0 / (2.0 * sigma_cells * sigma_cells);

  const int cx = static_cast<int>(std::round(gx));
  const int cy = static_cast<int>(std::round(gy));

  const int row_start = std::max(0, cy - radius);
  const int row_end = std::min(this->grid_rows_ - 1, cy + radius);
  const int col_start = std::max(0, cx - radius);
  const int col_end = std::min(this->grid_cols_ - 1, cx + radius);

  for (int row = row_start; row <= row_end; ++row) {
    const double dy = static_cast<double>(row) - gy;
    const double dy2 = dy * dy;
    for (int col = col_start; col <= col_end; ++col) {
      const double dx = static_cast<double>(col) - gx;
      const double dist2 = dx * dx + dy2;
      const float gaussian_val =
          static_cast<float>(std::exp(-dist2 * inv_2sigma2));
      grid[static_cast<std::size_t>(row * this->grid_cols_ + col)] +=
          weight * gaussian_val;
    }
  }
}

} // namespace rises
