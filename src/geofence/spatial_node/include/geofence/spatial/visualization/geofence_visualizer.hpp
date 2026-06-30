#pragma once

#include "rises_interfaces/msg/obstacle.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include "geofence/spatial/visualization/geofence_observer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rises {
/**
 * @class GeofenceVisualizer
 * @brief Publishes geofence-related visualization markers for RViz.
 *
 * This class manages and publishes various visualization elements, including:
 * - Obstacles (circles, rectangles, polygons)
 * - Safety circles (cylindrical regions around the robot)
 * - Operational areas (rectangular zones)
 * - Map boundary contours (outer and inner boundaries)
 * - Matched and error segments (for diagnostics)
 *
 * All visualizations are published as `visualization_msgs::msg::MarkerArray`
 * to ROS 2 topics using transient-local and reliable QoS profiles.
 *
 * Thread safety:
 *  - All modification and publishing functions acquire a shared mutex
 * (`mutex_`).
 *  - The class is safe for concurrent calls from multiple threads.
 */
class GeofenceVisualizer : public rises::geofence::GeofenceObserver {
public:
  /**
   * @brief Construct a new GeofenceVisualizer.
   *
   * @param node Pointer to a lifecycle node used for publisher creation.
   * @param tf_prefix Prefix to prepend to all published frame IDs.
   * @param target_frame Global map or world frame for visualization.
   * @param base_link_frame Robot’s local base frame (for safety circle
   * attachment).
   * @param enable_safety_circle Whether to visualize the safety circle.
   * @param safety_circle_radius Radius of the safety circle in meters.
   */
  explicit GeofenceVisualizer(rclcpp_lifecycle::LifecycleNode *node,
                              std::string tf_prefix, std::string target_frame,
                              std::string base_link_frame,
                              bool enable_safety_circle = false,
                              float safety_circle_radius = 0.5f);

  /**
   * @brief Add an obstacle to the visualization.
   *
   * Creates a marker (SPHERE, CUBE, or LINE_STRIP) depending on the obstacle
   * type. Existing obstacles with the same ID are replaced.
   *
   * @param obstacle Obstacle message containing geometry and ID.
   */
  void addObstacle(const rises_interfaces::msg::Obstacle &obstacle);

  /**
   * @brief Remove a previously published obstacle by ID.
   *
   * Converts the marker to a DELETE action and publishes it on the next update
   * cycle.
   *
   * @param id 64-bit obstacle ID to remove.
   */
  void removeObstacle(int64_t id);

  /**
   * @brief Add a cylindrical safety circle marker centered on the robot.
   *
   * @param radius Radius of the safety circle in meters.
   * @param ns Marker namespace, defaults to "safety_circle".
   */
  void addSafetyCircle(float radius, const std::string &ns = "safety_circle");

  /**
   * @brief Clear existing safety circle markers.
   *
   * Used before adding a new safety circle with different radius.
   */
  void clearSafetyCircle();

  /**
   * @brief Add a rectangular area marker for operational or restricted zones.
   *
   * @param id Unique identifier for the area.
   * @param x X-coordinate of the area center (in target_frame).
   * @param y Y-coordinate of the area center (in target_frame).
   * @param width Width of the area in meters.
   * @param height Height of the area in meters.
   * @param ns Marker namespace, defaults to "areas".
   */
  void addArea(int64_t id, float x, float y, float width, float height,
               const std::string &ns = "areas");

  /**
   * @brief Update the color of an existing area marker by ID.
   *
   * @param id Marker ID to modify.
   * @param r Red component (0–1).
   * @param g Green component (0–1).
   * @param b Blue component (0–1).
   */
  void updateAreaColor(int64_t id, float r, float g, float b);

  /**
   * @brief Add a matched segment marker for visualization (typically blue).
   *
   * Used to indicate successfully matched geometric segments in localization or
   * perception.
   *
   * @param obstacle A line or polygon obstacle representing the matched
   * segment.
   */
  void addMatchedSegment(const rises_interfaces::msg::Obstacle &obstacle);

  /**
   * @brief Add an error segment marker for visualization (typically red).
   *
   * Used to highlight mismatched or failed correspondences in geofencing
   * diagnostics.
   *
   * @param obstacle A line or polygon obstacle representing the error segment.
   */
  void addErrorSegment(const rises_interfaces::msg::Obstacle &obstacle);

  /**
   * @brief Set how error segments are visualized.
   *
   * @param as_lines If true, render 2+ vertex segments as LINE_STRIP.
   *                 If false (default), render as POINTS.
   */
  void setErrorSegmentLineMode(const bool as_lines) {
    this->error_segments_as_lines_ = as_lines;
  }

  /**
   * @brief Process obstacle correspondence result and visualize appropriately.
   *
   * Adds matched (blue) or error (red) segment based on match status.
   * Automatically publishes segment visualization.
   *
   * @param obstacle The detected obstacle
   * @param matched Whether the obstacle matched a known map obstacle
   */
  void
  processCorrespondenceResult(const rises_interfaces::msg::Obstacle &obstacle,
                              bool matched);

  /**
   * @brief Add and visualize map outer and inner boundary contours.
   *
   * The outer contour is drawn as a green LINE_STRIP.
   * Inner contours (holes) are also rendered in green.
   *
   * @param contours Map boundary contour structure.
   * @param ns Marker namespace, defaults to "map_boundary".
   */
  void addMapBoundary(const rises::shape::MapBoundaryContours &contours,
                      const std::string &ns = "map_boundary");

  /**
   * @brief Publish obstacle map markers if dirty.
   */
  void publishMap();

  /**
   * @brief Publish safety circle markers if dirty.
   */
  void publishSafetyCircle();

  /**
   * @brief Publish area markers if dirty.
   */
  void publishAreas();

  /**
   * @brief Publish map boundary contour markers if dirty.
   */
  void publishContours();

  /**
   * @brief Publish matched and error segment markers (ephemeral).
   */
  void publishSegments();

  /**
   * @brief Publish path validation markers (ephemeral).
   */
  void publishPath();

  /**
   * @brief Publish all dirty markers to their respective topics.
   *
   * Calls all individual publish methods. Use specific publish methods
   * when you know exactly what changed for better performance.
   */
  void publishAll();

  /**
   * @brief Activate all lifecycle publishers.
   *
   * Should be called during node activation.
   */
  void activate();

  /**
   * @brief Deactivate all lifecycle publishers.
   *
   * Should be called during node deactivation.
   */
  void deactivate();

  /**
   * @brief Check if visualizer is currently activated.
   *
   * @return true if publishers are active, false otherwise
   */
  [[nodiscard]] bool isActivated() const;

  /**
   * @brief Cleanup all resources and reset publishers.
   *
   * Typically called during lifecycle cleanup transition.
   */
  void cleanup();

  /**
   * @brief Clear all marker data and reset internal IDs.
   *
   * Does not affect ROS publishers, but removes all stored markers.
   */
  void clear();

  /**
   * @brief Clears all obstacle markers belonging to an old update.
   *
   */
  void clearOldObstacleUpdates();

  // ============================================================================
  // GeofenceObserver Interface Implementation
  // ============================================================================

  /**
   * @brief Called when the geofence map structure changes
   *
   * Automatically refreshes all static visualization elements.
   */
  void onMapChanged() override;

  /**
   * @brief Called when dynamic obstacles are detected
   *
   * @param obstacle The dynamic obstacle to visualize
   */
  void onDynamicObstacleUpdate(
      const rises_interfaces::msg::Obstacle &obstacle) override;

  /**
   * @brief Called when path validation results are available
   *
   * @param path The validated path
   * @param is_safe Whether the path is safe
   */
  void onPathValidationUpdate(const nav_msgs::msg::Path &path,
                              bool is_safe) override;

  /**
   * @brief Called when obstacle correspondence check is complete
   *
   * @param obstacle The detected obstacle
   * @param matched Whether it matched a known map obstacle
   */
  void onObstacleCorrespondence(const rises_interfaces::msg::Obstacle &obstacle,
                                bool matched) override;

  /**
   * @brief Called when map boundary contours are updated
   *
   * @param contours The new map boundary contours
   */
  void onMapBoundaryUpdate(
      const rises::shape::MapBoundaryContours &contours) override;

private:
  /**
   * @brief Create a pre-initialized base marker.
   *
   * Sets up header, timestamp, namespace, orientation, and unique ID.
   *
   * @param ns Marker namespace.
   * @param tf_prefix TF prefix to prepend to the frame name.
   * @param target_frame Frame ID for visualization.
   * @return Initialized visualization_msgs::msg::Marker.
   */
  visualization_msgs::msg::Marker
  createBaseMarker(const std::string &ns, const std::string tf_prefix,
                   const std::string target_frame);

  // === Node and configuration ===
  rclcpp_lifecycle::LifecycleNode
      *node_;                   ///< Node pointer for publishers and logging.
  std::string tf_prefix_;       ///< TF prefix for frame IDs.
  std::string target_frame_;    ///< Global/map frame for visualization.
  std::string base_link_frame_; ///< Frame attached to the robot's base.
  bool enable_safety_circle_;   ///< Whether to show safety circle.
  float safety_circle_radius_;  ///< Safety circle radius in meters.
  bool is_activated_ = false;   ///< Whether publishers are currently activated.
  bool error_segments_as_lines_ =
      false; ///< Render error segments as lines instead of points.
  mutable std::mutex mutex_; ///< Thread synchronization.
  // === Publishers ===
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  ///< Obstacle markers.
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr safety_circle_pub_;
  ///< Safety circles.
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr area_pub_;
  ///< Area markers.
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr contour_pub_;
  ///< Map boundary contours.
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::
      SharedPtr matched_segments_pub_; ///< Matched segments.
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::
      SharedPtr error_segments_pub_; ///< Error segments.
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::
      SharedPtr path_pub_; ///< Path visualization with safety color coding.

  // === Marker Containers (with dirty flags) ===
  std::unordered_map<int64_t, visualization_msgs::msg::Marker>
      obstacle_markers_;         ///< Active obstacle markers keyed by ID.
  bool obstacles_dirty_ = false; ///< Indicates new/removed obstacles.

  std::vector<visualization_msgs::msg::Marker> safety_circle_markers_;
  ///< Cylindrical safety zones.
  bool safety_circles_dirty_ = false;

  std::vector<visualization_msgs::msg::Marker> area_markers_;
  ///< Operational/restricted areas.
  bool areas_dirty_ = false;

  std::vector<visualization_msgs::msg::Marker> contour_markers_;
  ///< Outer and inner map boundary contours.
  bool contours_dirty_ = false;

  std::vector<visualization_msgs::msg::Marker> path_markers_;
  ///< Path visualization with color coding.
  bool path_dirty_ = false;

  // === Ephemeral markers (cleared each publish) ===
  std::unordered_map<int64_t, visualization_msgs::msg::Marker>
      matched_segment_markers_; ///< Diagnostic matched segments.
  std::unordered_map<int64_t, visualization_msgs::msg::Marker>
      error_segment_markers_; ///< Diagnostic error segments.

  // === Obstacle match state tracking ===
  std::unordered_set<int64_t>
      currently_matched_obstacles_; ///< Obstacles matched in current scan.
  std::unordered_set<int64_t>
      previously_matched_obstacles_; ///< Obstacles matched in previous scan.

  // === ID Management ===
  uint64_t next_id_ = 0; ///< Incremental ID counter for unnamed markers.

  // === 64→32 bit marker ID mapping ===
  // RViz markers use int32_t IDs. We assign sequential 32-bit IDs
  // and maintain a bidirectional mapping to/from 64-bit obstacle IDs.
  int32_t next_marker_id_ =
      1; ///< Next available 32-bit marker ID (0 reserved).
  std::unordered_map<int64_t, int32_t>
      obstacle_to_marker_id_; ///< obstacle ID → RViz marker ID

  /**
   * @brief Get or assign a 32-bit RViz marker ID for a 64-bit obstacle ID.
   * Must be called under mutex_.
   */
  int32_t getOrAssignMarkerId(int64_t obstacle_id);

  /**
   * @brief Get existing 32-bit marker ID, or -1 if not mapped.
   * Must be called under mutex_.
   */
  int32_t getMarkerId(int64_t obstacle_id) const;

public:
  /**
   * @brief Clear match state for new scan cycle.
   *
   * Call this before processing obstacles from a new scan to reset match
   * tracking. Moves current matches to previous, clears current matches.
   */
  void beginNewScanCycle();

  /**
   * @brief Mark an obstacle as matched in current scan.
   *
   * @param id Obstacle ID that was matched
   */
  void markObstacleMatched(const int64_t id);

  /**
   * @brief Refresh obstacle colors based on current match state.
   *
   * Updates all obstacle markers with colors reflecting their match status:
   * - Green: currently matched in this scan
   * - Gray: previously matched but not in current scan, or never matched
   *
   * Call this after processing all obstacles in a scan cycle, before
   * publishMap().
   */
  void refreshMapColors();
};
} // namespace rises
