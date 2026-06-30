#pragma once

// Project headers
#include "geofence/spatial/policies/error_segment_tracker.hpp"
// Only the plain match-result struct -- NOT batch_point_checker.hpp -- so this
// builder (and its consumers, e.g. the gridmap node) stay free of any
// spatial-index dependency.
#include "geofence/spatial/queries/obstacle_match_result.hpp"

// Third-party (ROS 2)
#include "rises_interfaces/msg/obstacle.hpp"
#include "rises_interfaces/msg/obstacle_array.hpp"
#include "rises_interfaces/msg/obstacle_report.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

// Standard library
#include <string>
#include <vector>

namespace rises
{

/**
 * @brief Builds ObstacleReport messages from BatchPointChecker results.
 *
 * Extracted from GeofenceNode::publishObstacleReport() and segmentPoints().
 * Handles gap-based segmentation of matched/unmatched vertices,
 * error segment ID tracking, and optional local-frame transformation.
 */
class ObstacleReportBuilder
{
public:
    struct Config {
        float segment_min_gap = 0.05f;          // minimum gap to split regardless of range (meters)
        float segment_gap_multiplier = 8.0f;    // gap = multiplier * range * angle_increment
        float lidar_angle_increment = 0.001745f; // angular resolution of the lidar (radians)
        float line_fit_tolerance = 0.05f;        // max perpendicular distance from first-to-last line before POLYGON (meters)
        std::size_t min_segment_points = 1;  // raise to filter scan noise (e.g. 3 discards 1-2 point clusters)
        bool report_error_segments_as_points = false;
        bool enable_error_segment_tracking = true;
        float error_segment_tracker_cell_size = 0.3f;
        float error_segment_tracker_max_drift = 1.0f;
        float outlier_filter_distance = 0.10f;  // remove spike outliers > this from neighbor line (meters, 0 = disabled)
        bool publish_report_in_local_frame = false;
        std::string map_frame_id;
        std::string robot_frame_id;
        std::string report_output_frame;  // TF frame for report output (empty = use robot_frame_id)
    };

    explicit ObstacleReportBuilder(const Config& config);

    /**
     * @brief Build a report from the detection result.
     *
     * Classifies vertices per obstacle as matched/unmatched, splits on
     * spatial gaps, assigns persistent segment IDs, and optionally
     * transforms to the robot local frame.
     *
     * @param msg         Original obstacle array (scan input).
     * @param result      Per-obstacle match classification from BatchPointChecker.
     * @param tf_buffer   TF buffer for local-frame transform (may be null if not needed).
     * @param logger      Logger for diagnostics.
     * @return The built report in map frame (stored internally as last_report).
     */
    const rises_interfaces::msg::ObstacleReport& buildReport(
        const rises_interfaces::msg::ObstacleArray::ConstSharedPtr& msg,
        const rises::geofence::query::ObstacleMatchResult& result,
        const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
        rclcpp::Logger logger);

    /**
     * @brief Publish the report if conditions are met.
     *
     * Handles the local-frame transformation when configured.
     *
     * @param publisher        Lifecycle publisher for the report.
     * @param publish_always   Whether to publish even when no unmatched obstacles.
     * @param tf_buffer        TF buffer for local-frame transform.
     * @param logger           Logger for diagnostics.
     */
    void publishReport(
        const rclcpp_lifecycle::LifecyclePublisher<rises_interfaces::msg::ObstacleReport>::SharedPtr& publisher,
        bool publish_always,
        const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
        rclcpp::Logger logger);

    /** @brief Access the last built report (in map frame, for visualizer). */
    [[nodiscard]] const rises_interfaces::msg::ObstacleReport& lastReport() const { return this->last_report_; }

    /** @brief Begin a new scan cycle for error segment tracking. */
    void beginScanCycle();

    /** @brief End the scan cycle (evicts unseen segments). */
    void endScanCycle();

private:
    Config config_;
    ErrorSegmentTracker error_segment_tracker_;
    rises_interfaces::msg::ObstacleReport last_report_;
};

} // namespace rises
