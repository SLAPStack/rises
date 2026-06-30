#include "geofence/spatial/node/obstacle_report_builder.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <algorithm>

namespace rises
{

ObstacleReportBuilder::ObstacleReportBuilder(const Config& config)
    : config_(config),
      error_segment_tracker_(config.error_segment_tracker_cell_size,
                             config.error_segment_tracker_max_drift)
{
}

void ObstacleReportBuilder::beginScanCycle()
{
    if (this->config_.enable_error_segment_tracking) {
        this->error_segment_tracker_.beginScanCycle();
    }
}

void ObstacleReportBuilder::endScanCycle()
{
    if (this->config_.enable_error_segment_tracking) {
        this->error_segment_tracker_.endScanCycle();
    }
}

const rises_interfaces::msg::ObstacleReport& ObstacleReportBuilder::buildReport(
    const rises_interfaces::msg::ObstacleArray::ConstSharedPtr& msg,
    const rises::geofence::query::ObstacleMatchResult& result,
    const std::shared_ptr<tf2_ros::Buffer>& /*tf_buffer*/,
    rclcpp::Logger logger)
{
    this->last_report_ = rises_interfaces::msg::ObstacleReport();
    this->last_report_.header = msg->header;

    rises_interfaces::msg::ObstacleReport& report_ref = this->last_report_;

    std::size_t total_matched_pts = 0;
    std::size_t total_unmatched_pts = 0;

    const float min_gap = this->config_.segment_min_gap;
    const float gap_mul = this->config_.segment_gap_multiplier;
    // Prefer angle_increment from the message (set by laserscan_preprocessor);
    // fall back to the ROS parameter if the message field is zero (e.g. older publisher).
    const float angle_inc = (msg->angle_increment > 0.0f)
        ? msg->angle_increment
        : this->config_.lidar_angle_increment;
    const float line_tol = this->config_.line_fit_tolerance;
    const std::size_t min_points = this->config_.min_segment_points;
    const float outlier_dist = this->config_.outlier_filter_distance;

    // Split an ordered vertex list on distance gaps, remove spike outliers,
    // then fit each group into LINE segments using Douglas-Peucker simplification.
    // Groups with fewer than min_points vertices are discarded as scan noise.
    const auto emitSegments = [min_gap, gap_mul, angle_inc, line_tol, min_points, outlier_dist](
        const std::vector<geometry_msgs::msg::Point>& verts,
        std::vector<rises_interfaces::msg::Obstacle>& out,
        std::size_t& total_pts)
    {
        if (verts.empty()) return;

        // Phase 1: group by distance gap only
        // Gap threshold = max(min_gap, multiplier * range * angle_increment)
        // This tracks the actual lidar point spacing at each range.
        std::vector<std::size_t> splits;
        splits.push_back(0);

        for (std::size_t i = 1; i < verts.size(); ++i) {
            const float dx = static_cast<float>(verts[i].x - verts[i - 1].x);
            const float dy = static_cast<float>(verts[i].y - verts[i - 1].y);
            const float dist = std::sqrt(dx * dx + dy * dy);
            const float range = static_cast<float>(std::sqrt(
                verts[i - 1].x * verts[i - 1].x +
                verts[i - 1].y * verts[i - 1].y));
            const float gap_threshold = std::max(min_gap, gap_mul * range * angle_inc);

            if (dist > gap_threshold) {
                splits.push_back(i);
            }
        }
        splits.push_back(verts.size());

        // Phase 1.5: outlier spike removal within each group.
        // For each interior point, compute perpendicular distance to the line
        // between its neighbors. If it exceeds outlier_dist, mark it for removal.
        // This catches multipath reflections and edge-effect spikes that would
        // otherwise cause Douglas-Peucker to produce spurious tiny segments.
        std::vector<bool> outlier(verts.size(), false);
        if (outlier_dist > 0.0f) {
            for (std::size_t s = 0; s + 1 < splits.size(); ++s) {
                const std::size_t begin = splits[s];
                const std::size_t end = splits[s + 1];
                if (end - begin < 3) continue;  // need at least 3 points to detect spikes

                for (std::size_t i = begin + 1; i + 1 < end; ++i) {
                    const float ax = static_cast<float>(verts[i - 1].x);
                    const float ay = static_cast<float>(verts[i - 1].y);
                    const float bx = static_cast<float>(verts[i + 1].x);
                    const float by = static_cast<float>(verts[i + 1].y);
                    const float px = static_cast<float>(verts[i].x);
                    const float py = static_cast<float>(verts[i].y);

                    const float ldx = bx - ax;
                    const float ldy = by - ay;
                    const float len_sq = ldx * ldx + ldy * ldy;

                    float perp;
                    if (len_sq > 1e-12f) {
                        perp = std::abs(ldx * (py - ay) - ldy * (px - ax)) / std::sqrt(len_sq);
                    } else {
                        // Neighbors coincident — distance from point to neighbor
                        perp = std::sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
                    }

                    if (perp > outlier_dist) {
                        outlier[i] = true;
                    }
                }
            }
        }

        // Build filtered index lists per group (skipping outliers)
        // Phase 2: emit each group as LINE segments using Douglas-Peucker
        // recursive simplification. This finds the minimal set of lines that
        // approximate the shape within line_tol.

        // Group-level transverse extent, recomputed once per group (see the
        // assignment at the top of the `for (s...)` loop below) and read by
        // makeLine via its [&] capture. Declaring it here, before makeLine,
        // means every makeLine call made while processing one group sees that
        // group's value.
        float current_group_width = 0.0f;

        // Helper: create a LINE obstacle from two points
        const auto makeLine = [&](const geometry_msgs::msg::Point& p0,
                                  const geometry_msgs::msg::Point& p1)
        {
            rises_interfaces::msg::Obstacle seg;
            seg.type = rises_interfaces::msg::Obstacle::LINE;
            seg.vertices.push_back(p0);
            seg.vertices.push_back(p1);
            seg.position.x = (p0.x + p1.x) * 0.5;
            seg.position.y = (p0.y + p1.y) * 0.5;
            seg.position.z = 0.0;
            // width is the WHOLE GROUP's transverse extent (max perpendicular
            // deviation off the group's own first->last chord), not this
            // individual segment's endpoint distance. Endpoint distance would
            // be wrong: it would feed a wall fragment's length to
            // width-gating consumers (e.g. rises_leg_filter, gate 0.15-0.8 m),
            // passing short straight wall fragments as legs. Using the whole
            // group's spread instead means a leg's curved arc still reports a
            // meaningful width even if Douglas-Peucker below splits it into
            // multiple near-straight sub-segments that would each individually
            // measure near-zero. The segment's length remains recoverable
            // from vertices[0]..vertices[1].
            seg.width = current_group_width;
            total_pts += 2;
            out.push_back(std::move(seg));
        };

        // Douglas-Peucker: find the point with maximum perpendicular distance
        // from the first->last line. If it exceeds tolerance, split there and
        // recurse on each half. Otherwise emit a single line first->last.
        // Uses an explicit stack to avoid deep recursion on large groups.
        struct DPRange { std::size_t first; std::size_t last; };
        std::vector<DPRange> dp_stack;
        std::vector<geometry_msgs::msg::Point> filtered;  // reused per group

        for (std::size_t s = 0; s + 1 < splits.size(); ++s) {
            const std::size_t begin = splits[s];
            const std::size_t end = splits[s + 1];

            // Collect non-outlier points for this group
            filtered.clear();
            for (std::size_t i = begin; i < end; ++i) {
                if (!outlier[i]) {
                    filtered.push_back(verts[i]);
                }
            }
            const std::size_t len = filtered.size();

            if (len < min_points) continue;

            // Group-level transverse extent: max perpendicular distance of any point in
            // this group from the group's own first->last chord. Computed ONCE per group
            // (not per final DP segment) so a leg's whole curved arc gets a meaningful
            // width even if Douglas-Peucker later splits it into straighter sub-pieces
            // that would individually measure near-zero. O(n) per group -- same cost
            // class as the existing outlier-detection pass above.
            current_group_width = 0.0f;
            if (len >= 2) {
                const float gax = static_cast<float>(filtered[0].x);
                const float gay = static_cast<float>(filtered[0].y);
                const float gbx = static_cast<float>(filtered[len - 1].x);
                const float gby = static_cast<float>(filtered[len - 1].y);
                const float gldx = gbx - gax;
                const float gldy = gby - gay;
                const float glen_sq = gldx * gldx + gldy * gldy;
                if (glen_sq > 1e-12f) {
                    const float ginv_len = 1.0f / std::sqrt(glen_sq);
                    for (std::size_t k = 1; k + 1 < len; ++k) {
                        const float kpx = static_cast<float>(filtered[k].x) - gax;
                        const float kpy = static_cast<float>(filtered[k].y) - gay;
                        const float gperp = std::abs(gldx * kpy - gldy * kpx) * ginv_len;
                        current_group_width = std::max(current_group_width, gperp);
                    }
                }
            }

            if (len == 1) {
                makeLine(filtered[0], filtered[0]);
            } else {
                // Seed the Douglas-Peucker stack with the full range
                dp_stack.clear();
                dp_stack.push_back({0, len - 1});

                while (!dp_stack.empty()) {
                    const DPRange range = dp_stack.back();
                    dp_stack.pop_back();

                    if (range.last <= range.first + 1) {
                        // Two adjacent points or same point — emit directly
                        makeLine(filtered[range.first], filtered[range.last]);
                        continue;
                    }

                    const float ax = static_cast<float>(filtered[range.first].x);
                    const float ay = static_cast<float>(filtered[range.first].y);
                    const float bx = static_cast<float>(filtered[range.last].x);
                    const float by = static_cast<float>(filtered[range.last].y);
                    const float ldx = bx - ax;
                    const float ldy = by - ay;
                    const float len_sq = ldx * ldx + ldy * ldy;

                    float max_dist = 0.0f;
                    std::size_t max_idx = range.first;

                    if (len_sq > 1e-12f) {
                        const float inv_len = 1.0f / std::sqrt(len_sq);
                        for (std::size_t k = range.first + 1; k < range.last; ++k) {
                            const float px = static_cast<float>(filtered[k].x) - ax;
                            const float py = static_cast<float>(filtered[k].y) - ay;
                            const float perp = std::abs(ldx * py - ldy * px) * inv_len;
                            if (perp > max_dist) {
                                max_dist = perp;
                                max_idx = k;
                            }
                        }
                    } else {
                        // First and last coincident — pick farthest point from first
                        for (std::size_t k = range.first + 1; k < range.last; ++k) {
                            const float pdx = static_cast<float>(filtered[k].x) - ax;
                            const float pdy = static_cast<float>(filtered[k].y) - ay;
                            const float d = pdx * pdx + pdy * pdy;
                            if (d > max_dist) {
                                max_dist = d;
                                max_idx = k;
                            }
                        }
                        max_dist = std::sqrt(max_dist);
                    }

                    if (max_dist <= line_tol) {
                        // All intermediate points within tolerance — single line
                        makeLine(filtered[range.first], filtered[range.last]);
                    } else {
                        // Split at the worst point and recurse on both halves.
                        // Push right half first so left half is processed first
                        // (preserves scan order in the output).
                        dp_stack.push_back({max_idx, range.last});
                        dp_stack.push_back({range.first, max_idx});
                    }
                }
            }

        }
    };

    for (std::size_t obs_idx = 0; obs_idx < msg->obstacles.size(); ++obs_idx) {
        const rises_interfaces::msg::Obstacle& src = msg->obstacles[obs_idx];

        // Build sets of matched and unmatched vertex indices.
        // Vertices in neither set were outside the safety circle (filtered out)
        // and must be ignored — not treated as unmatched.
        std::vector<bool> is_matched(src.vertices.size(), false);
        std::vector<bool> is_unmatched(src.vertices.size(), false);
        if (obs_idx < result.matched_vertices_per_obstacle.size()) {
            for (const std::size_t vi : result.matched_vertices_per_obstacle[obs_idx]) {
                is_matched[vi] = true;
            }
        }
        if (obs_idx < result.unmatched_vertices_per_obstacle.size()) {
            for (const std::size_t vi : result.unmatched_vertices_per_obstacle[obs_idx]) {
                is_unmatched[vi] = true;
            }
        }

        std::vector<geometry_msgs::msg::Point> matched_verts;
        std::vector<geometry_msgs::msg::Point> unmatched_verts;
        for (std::size_t vi = 0; vi < src.vertices.size(); ++vi) {
            if (is_matched[vi]) {
                matched_verts.push_back(src.vertices[vi]);
            } else if (is_unmatched[vi]) {
                unmatched_verts.push_back(src.vertices[vi]);
            }
            // else: vertex was outside safety circle, skip it
        }

        if (this->config_.report_error_segments_as_points) {
            const auto emitAsDegenerateLines = [](
                const std::vector<geometry_msgs::msg::Point>& verts,
                std::vector<rises_interfaces::msg::Obstacle>& out,
                std::size_t& total_pts)
            {
                for (const geometry_msgs::msg::Point& pt : verts) {
                    rises_interfaces::msg::Obstacle seg;
                    seg.type = rises_interfaces::msg::Obstacle::LINE;
                    seg.vertices.push_back(pt);
                    seg.vertices.push_back(pt);
                    seg.position = pt;
                    seg.position.z = 0.0;
                    total_pts += 2;
                    out.push_back(std::move(seg));
                }
            };
            emitAsDegenerateLines(matched_verts, report_ref.matched_obstacles, total_matched_pts);
            emitAsDegenerateLines(unmatched_verts, report_ref.unmatched_obstacles, total_unmatched_pts);
        } else {
            emitSegments(matched_verts, report_ref.matched_obstacles, total_matched_pts);
            emitSegments(unmatched_verts, report_ref.unmatched_obstacles, total_unmatched_pts);
        }
    }

    // Assign sequential IDs to matched segments
    for (std::size_t i = 0; i < report_ref.matched_obstacles.size(); ++i) {
        report_ref.matched_obstacles[i].id = i;
    }

    // Assign persistent IDs to each unmatched line individually.
    // Each line's midpoint (position) and AABB from its two endpoints are used
    // for centroid-proximity and overlap matching in the tracker.
    if (this->config_.enable_error_segment_tracking) {
        for (rises_interfaces::msg::Obstacle& obs : report_ref.unmatched_obstacles) {
            SegmentAABB aabb{};
            if (obs.vertices.size() >= 2) {
                const float x0 = static_cast<float>(obs.vertices[0].x);
                const float y0 = static_cast<float>(obs.vertices[0].y);
                const float x1 = static_cast<float>(obs.vertices[1].x);
                const float y1 = static_cast<float>(obs.vertices[1].y);
                aabb.min_x = std::min(x0, x1);
                aabb.min_y = std::min(y0, y1);
                aabb.max_x = std::max(x0, x1);
                aabb.max_y = std::max(y0, y1);
            } else if (!obs.vertices.empty()) {
                const float px = static_cast<float>(obs.vertices[0].x);
                const float py = static_cast<float>(obs.vertices[0].y);
                aabb = {px, py, px, py};
            } else {
                const float px = static_cast<float>(obs.position.x);
                const float py = static_cast<float>(obs.position.y);
                aabb = {px, py, px, py};
            }

            // Persistent ID is stable across scans for a stationary feature;
            // a moving obstacle drifts and is re-IDed. The validation node uses
            // this ID's recurrence (+ ground-truth spawns) to tell a true static
            // miss from a moving/stationary intruder.
            const int32_t persistent_id = this->error_segment_tracker_.getOrAssignId(
                static_cast<float>(obs.position.x),
                static_cast<float>(obs.position.y),
                aabb);
            obs.id = static_cast<uint64_t>(persistent_id);
        }
    } else {
        for (rises_interfaces::msg::Obstacle& obs : report_ref.unmatched_obstacles) {
            obs.id = static_cast<uint64_t>(-1);
        }
    }

    RCLCPP_DEBUG(logger,
        "ObstacleReport: %zu matched lines (%zu pts), %zu unmatched lines (%zu pts), tracking=%s",
        report_ref.matched_obstacles.size(), total_matched_pts,
        report_ref.unmatched_obstacles.size(), total_unmatched_pts,
        this->config_.enable_error_segment_tracking ? "on" : "off");

    return this->last_report_;
}

void ObstacleReportBuilder::publishReport(
    const rclcpp_lifecycle::LifecyclePublisher<rises_interfaces::msg::ObstacleReport>::SharedPtr& publisher,
    const bool publish_always,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    rclcpp::Logger logger)
{
    const rises_interfaces::msg::ObstacleReport& report_ref = this->last_report_;
    const bool has_unmatched = !report_ref.unmatched_obstacles.empty();

    if (!publisher || !publisher->is_activated()) return;
    if (!publish_always && !has_unmatched) return;

    if (this->config_.publish_report_in_local_frame) {
        // Use report_output_frame if set, otherwise fall back to robot_frame_id
        const std::string& target_frame = this->config_.report_output_frame.empty()
            ? this->config_.robot_frame_id
            : this->config_.report_output_frame;

        // Transform in-place on last_report_, publish, then reverse-transform to restore.
        // Avoids copying the entire report (all obstacle vectors + vertex vectors) every frame.
        try {
            const geometry_msgs::msg::TransformStamped tf_map_to_target =
                tf_buffer->lookupTransform(
                    target_frame, this->config_.map_frame_id,
                    tf2::TimePointZero);

            // Precompute rotation matrix from quaternion (2D only)
            const geometry_msgs::msg::Vector3& t = tf_map_to_target.transform.translation;
            const geometry_msgs::msg::Quaternion& q = tf_map_to_target.transform.rotation;

            const double r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            const double r01 = 2.0 * (q.x * q.y - q.z * q.w);
            const double r10 = 2.0 * (q.x * q.y + q.z * q.w);
            const double r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
            const double tx = t.x;
            const double ty = t.y;

            // Inverse rotation matrix (transpose of orthogonal matrix) + inverse translation
            const double ir00 = r00;
            const double ir01 = r10;
            const double ir10 = r01;
            const double ir11 = r11;
            const double itx = -(ir00 * tx + ir01 * ty);
            const double ity = -(ir10 * tx + ir11 * ty);

            const auto applyTransform = [](
                rises_interfaces::msg::ObstacleReport& report,
                double m00, double m01, double m10, double m11,
                double dx, double dy)
            {
                const auto xformObs = [m00, m01, m10, m11, dx, dy](rises_interfaces::msg::Obstacle& obs) {
                    const double ox = obs.position.x;
                    const double oy = obs.position.y;
                    obs.position.x = m00 * ox + m01 * oy + dx;
                    obs.position.y = m10 * ox + m11 * oy + dy;
                    for (geometry_msgs::msg::Point& v : obs.vertices) {
                        const double vx = v.x;
                        const double vy = v.y;
                        v.x = m00 * vx + m01 * vy + dx;
                        v.y = m10 * vx + m11 * vy + dy;
                    }
                };
                for (rises_interfaces::msg::Obstacle& obs : report.matched_obstacles) {
                    xformObs(obs);
                }
                for (rises_interfaces::msg::Obstacle& obs : report.unmatched_obstacles) {
                    xformObs(obs);
                }
            };

            // Forward transform: map → target frame
            const std::string original_frame = this->last_report_.header.frame_id;
            this->last_report_.header.frame_id = target_frame;
            applyTransform(this->last_report_, r00, r01, r10, r11, tx, ty);

            // Restore map frame for other consumers (e.g. lastReport()).
            const auto restore_map_frame = [&]() {
                this->last_report_.header.frame_id = original_frame;
                applyTransform(this->last_report_, ir00, ir01, ir10, ir11, itx, ity);
            };

            // publish() in the target frame, then ALWAYS restore -- if it throws
            // a non-tf2 exception, last_report_ must not be left target-framed
            // for the next reader.
            try {
                publisher->publish(
                    std::make_unique<rises_interfaces::msg::ObstacleReport>(this->last_report_));
            } catch (...) {
                restore_map_frame();
                throw;
            }
            restore_map_frame();
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(logger,
                "Cannot transform obstacle report to '%s': %s", target_frame.c_str(), ex.what());
            publisher->publish(
                std::make_unique<rises_interfaces::msg::ObstacleReport>(report_ref));
        }
    } else {
        publisher->publish(
            std::make_unique<rises_interfaces::msg::ObstacleReport>(report_ref));
    }
}

} // namespace rises
