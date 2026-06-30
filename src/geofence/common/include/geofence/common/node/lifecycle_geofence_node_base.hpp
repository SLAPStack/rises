#pragma once

// Project headers
#include "geofence/common/node/geofence_common_config.hpp"
#include "geofence/spatial/node/obstacle_report_builder.hpp"
#include "geofence/common/policies/robot_tracking.hpp"
#include "geofence/spatial/queries/obstacle_match_result.hpp"
#include "geofence/common/queries/robot_tracking_checker.hpp"
#include "geofence/spatial/visualization/geofence_visualizer.hpp"
#include "geometry_types.hpp"

// Third-party (ROS 2)
#include "rises_interfaces/msg/contours.hpp"
#include "rises_interfaces/msg/obstacle_array.hpp"
#include "rises_interfaces/msg/obstacle_report.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/srv/set_area_state.hpp"
#include "rises_interfaces/srv/set_safety_circle_radius.hpp"
#include "rises_interfaces/srv/update_map.hpp"
#include "rises_interfaces/srv/update_warehouse_layout.hpp"
#include "rises_interfaces/srv/validate_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Standard library
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rises
{

/**
 * @brief Common lifecycle scaffolding shared by every geofence node backend.
 *
 * Both geofence backends -- the spatial GeofenceNode (nanoflann KD-tree) and the
 * GeofenceGridmapNode (occupancy grid) -- carry >90% identical ROS plumbing: TF2
 * setup, the auto-activate transition timer, the shared subscriptions /
 * publishers / services, the per-scan callback skeleton, robot-footprint
 * helpers, and the lifecycle skeletons. This base class owns all of that and
 * dispatches the backend-specific bits through a small set of virtual hooks.
 *
 * Per-scan dispatch is a single matchScan() call at ~20 Hz, so virtual-call cost
 * is irrelevant. The base accesses shared parameters only through the pure
 * virtual commonConfig() (each derived returns its own cfg_, which derives from
 * GeofenceCommonConfig), so there is a single source of truth for the safety
 * radius, frames, etc.
 *
 * Construction ordering note: virtual commonConfig() is NOT valid inside the
 * base constructor (the derived object is not yet constructed). The base ctor
 * therefore does only TF2 buffer/listener setup. Each derived ctor must:
 *   1) construct the base,
 *   2) load its config into its own cfg_,
 *   3) call the protected initCommon() to build the RobotTrackingChecker and the
 *      auto-activate timer from commonConfig().
 */
class LifecycleGeofenceNodeBase : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// Result of a single per-scan match dispatch.
    struct ScanMatchResult {
        /// True iff at least one unmatched (intruder) obstacle was found this
        /// scan. Drives the centralized obstacle_alert publish.
        bool has_unmatched = false;
        /// Per-segment matched/unmatched classification consumed by the shared
        /// ObstacleReportBuilder to emit the obstacle_report.
        rises::geofence::query::ObstacleMatchResult report_result;
        /// Whether this scan feeds the per-segment ObstacleReportBuilder. True
        /// for the points-only and gridmap paths (report_result is populated).
        /// The spatial shape-matching path sets this false: it classifies
        /// per-obstacle and publishes unmatched obstacles directly, never using
        /// the report builder -- matching pre-unification behaviour (otherwise
        /// it would emit an empty report every scan).
        bool publish_report = true;
    };

    LifecycleGeofenceNodeBase(const std::string& node_name,
                              const rclcpp::NodeOptions& options);
    ~LifecycleGeofenceNodeBase() override = default;

    // Lifecycle callbacks (shared skeletons; call the *Extra hooks).
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&) override;

protected:
    // ========================================================================
    // Pure virtual hooks -- every derived backend MUST implement.
    // ========================================================================

    /// Single source of truth for the shared parameters. Each derived returns a
    /// reference to its own cfg_ (which derives from GeofenceCommonConfig).
    [[nodiscard]] virtual GeofenceCommonConfig& commonConfig() = 0;
    [[nodiscard]] virtual const GeofenceCommonConfig& commonConfig() const = 0;

    /// Per-scan correspondence match against the backend. Called once per
    /// obstacle array at ~20 Hz. @p have_pos is false when the robot position
    /// could not be resolved but the scan is still processed.
    [[nodiscard]] virtual ScanMatchResult
    matchScan(const rises_interfaces::msg::ObstacleArray::ConstSharedPtr& msg,
              const Point2D& robot_pos, bool have_pos) = 0;

    /// Pre-load obstacles from a JSON file into the backend.
    [[nodiscard]] virtual bool loadObstaclesFromJson(const std::string& filepath) = 0;
    /// Pre-load map boundary contours from a JSON file.
    [[nodiscard]] virtual bool loadContoursFromJson(const std::string& filepath) = 0;

    /// Apply a batch of map obstacle updates (topic + service share this path).
    virtual void
    applyMapUpdates(const rises_interfaces::msg::ObstacleUpdateArray& updates,
                    const char* source_tag) = 0;

    // Service handlers (backend-specific behavior).
    virtual void validatePathService(
        const std::shared_ptr<rises_interfaces::srv::ValidatePath::Request> request,
        const std::shared_ptr<rises_interfaces::srv::ValidatePath::Response> response) = 0;
    virtual void updateMapService(
        const std::shared_ptr<rises_interfaces::srv::UpdateMap::Request> request,
        const std::shared_ptr<rises_interfaces::srv::UpdateMap::Response> response) = 0;
    virtual void updateWarehouseLayoutService(
        const std::shared_ptr<rises_interfaces::srv::UpdateWarehouseLayout::Request> request,
        const std::shared_ptr<rises_interfaces::srv::UpdateWarehouseLayout::Response> response) = 0;
    virtual void areaStateService(
        const std::shared_ptr<rises_interfaces::srv::SetAreaState::Request> request,
        const std::shared_ptr<rises_interfaces::srv::SetAreaState::Response> response) = 0;

    // ========================================================================
    // Optional virtual hooks -- sensible defaults; override as needed.
    // ========================================================================

    /// Extra setup during on_configure (spatial: diagnostics + latency + the 4
    /// GET services). Default: nothing.
    virtual void onConfigureExtra() {}
    /// Extra setup during on_activate / on_deactivate / on_cleanup.
    virtual void onActivateExtra() {}
    virtual void onDeactivateExtra() {}
    virtual void onCleanupExtra() {}

    /// Apply parsed boundary contours (spatial: stores + visualizes; gridmap:
    /// logs + ignores). Default: log once that contours are not applied.
    virtual void setContours(const rises_interfaces::msg::Contours& contours);

    /// Called once per scan after the obstacle_report is built, before the
    /// throttled publishSegments(). Lets a backend populate visualizer segments
    /// from the segmented report rather than from raw scan obstacles (the
    /// spatial points-only path needs the report's per-segment geometry).
    /// Default: nothing (backends that add segments inside matchScan).
    virtual void onReportBuilt(const rises_interfaces::msg::ObstacleReport& report);

    /// Number of obstacles currently in the backend (for the ready-signal log).
    [[nodiscard]] virtual std::size_t backendObstacleCount() const { return 0; }
    /// Reset the backend during cleanup. Default: nothing.
    virtual void resetBackend() {}
    /// True iff the backend is initialized (null-check). Drives the per-scan
    /// early-out. Default: always ready.
    [[nodiscard]] virtual bool backendReady() const { return true; }

    /// Update the active safety circle radius. Base default updates the shared
    /// config radius + visualizer. Spatial overrides to also update its
    /// safety_profile_.
    virtual void setSafetyCircleRadius(float radius);

    // ========================================================================
    // Shared services for derived classes.
    // ========================================================================

    /// Build the RobotTrackingChecker + auto-activate timer from commonConfig().
    /// Each derived ctor calls this AFTER loading its config.
    void initCommon();

    /// canTransform-first TF lookup of the robot position in the map frame.
    [[nodiscard]] bool getRobotPosition(Point2D& position,
                                        const rclcpp::Time& timestamp) const;

    /// Build a robot footprint from per-robot or default parameters.
    [[nodiscard]] rises::geofence::policies::RobotFootprint
    createRobotFootprint(const std::string& robot_id) const;
    [[nodiscard]] static std::string
    getFootprintTypeString(rises::geofence::policies::RobotFootprint::Type type);

    /// Publish the latched geofence_ready signal (if enabled).
    void publishReadySignal();

    /// Running sum/count of the per-scan alert-latency metrics (processing_ms,
    /// scan_to_alert_ms), accumulated every scan in obstaclesCallback().
    /// Guarded by alert_latency_mutex_ since produceDiagnostics() (1 Hz, on the
    /// diagnostic_updater thread) reads/resets it while obstaclesCallback()
    /// (~20 Hz, on the subscription callback thread) writes it.
    struct AlertLatencyAccumulator {
        double processing_ms_sum = 0.0;
        double scan_to_alert_ms_sum = 0.0;
        int64_t count = 0;
    };

    /// Result of getAndResetAlertLatencyStats(): the rolling averages since the
    /// last call (zero/zero if no scans were processed in that window).
    struct AlertLatencyStats {
        double avg_processing_ms = 0.0;
        double avg_scan_to_alert_ms = 0.0;
    };

    /// Returns the average processing_ms / scan_to_alert_ms accumulated since
    /// the last call, then resets the accumulator to zero. Must be called
    /// exactly once per diagnostic_updater tick (1 Hz) -- NOT per scan -- so
    /// this is an aggregate periodic KPI rather than per-scan write
    /// amplification.
    [[nodiscard]] AlertLatencyStats getAndResetAlertLatencyStats();

    // ========================================================================
    // Shared state owned by the base.
    // ========================================================================

    // Constructed TF frame names (built in on_configure from commonConfig()).
    std::string map_frame_id_;
    std::string robot_frame_id_;

    // TF2 for robot position.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Shared internal components.
    std::unique_ptr<rises::geofence::RobotTrackingChecker> robot_tracking_checker_;
    std::shared_ptr<GeofenceVisualizer> visualizer_;
    std::unique_ptr<ObstacleReportBuilder> report_builder_;

    // Shared ROS interfaces.
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
    rclcpp::Subscription<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr map_updates_sub_;
    rclcpp::Subscription<rises_interfaces::msg::ObstacleArray>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<rises_interfaces::msg::Contours>::SharedPtr map_boundary_sub_;
    std::unordered_map<std::string,
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> robot_pose_subs_;

    rclcpp_lifecycle::LifecyclePublisher<rises_interfaces::msg::ObstacleArray>::SharedPtr unmatched_obstacle_pub_;
    rclcpp_lifecycle::LifecyclePublisher<rises_interfaces::msg::ObstacleReport>::SharedPtr obstacle_report_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr obstacle_alert_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;

    rclcpp::Service<rises_interfaces::srv::ValidatePath>::SharedPtr validate_path_srv_;
    rclcpp::Service<rises_interfaces::srv::SetAreaState>::SharedPtr area_state_srv_;
    rclcpp::Service<rises_interfaces::srv::UpdateMap>::SharedPtr update_map_srv_;
    rclcpp::Service<rises_interfaces::srv::UpdateWarehouseLayout>::SharedPtr update_warehouse_layout_srv_;
    rclcpp::Service<rises_interfaces::srv::SetSafetyCircleRadius>::SharedPtr set_safety_circle_radius_srv_;

    rclcpp::TimerBase::SharedPtr auto_transition_timer_;
    rclcpp::TimerBase::SharedPtr safety_circle_timer_;

    // Visualization throttle -- wall clock based (not affected by sim time).
    std::chrono::steady_clock::time_point last_map_viz_time_{};
    std::chrono::steady_clock::time_point last_seg_viz_time_{};
    static constexpr std::chrono::milliseconds MAP_VIZ_INTERVAL{1000};  // 1 Hz
    static constexpr std::chrono::milliseconds SEG_VIZ_INTERVAL{200};   // 5 Hz

    // Alert-latency rolling-average accumulator (see getAndResetAlertLatencyStats()).
    std::mutex alert_latency_mutex_;
    AlertLatencyAccumulator alert_latency_accumulator_;

private:
    // Shared callbacks.
    void updatesCallback(const rises_interfaces::msg::ObstacleUpdateArray::ConstSharedPtr& msg);
    void obstaclesCallback(const rises_interfaces::msg::ObstacleArray::ConstSharedPtr& msg);
    void mapBoundaryCallback(const rises_interfaces::msg::Contours::ConstSharedPtr& msg);
    void robotPoseCallback(const std::string& robot_id,
                           const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void auto_transition();

    void setSafetyCircleRadiusService(
        const std::shared_ptr<rises_interfaces::srv::SetSafetyCircleRadius::Request> request,
        const std::shared_ptr<rises_interfaces::srv::SetSafetyCircleRadius::Response> response);

    uint8_t last_transition_state_ = 0;
};

} // namespace rises
