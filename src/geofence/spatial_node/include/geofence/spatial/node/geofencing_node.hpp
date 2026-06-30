#pragma once

// Project headers
#include "geofence/common/node/lifecycle_geofence_node_base.hpp"
#include "geofence/common/latency_recorder.hpp"
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/node/geofence_config.hpp"
#include "geofence/spatial/node/map_update_handler.hpp"
#include "geofence/spatial/node/obstacle_report_builder.hpp"
#include "geofence/common/policies/coordinate_transform.hpp"
#include "geofence/spatial/policies/robot_safety_profile.hpp"
#include "geofence/common/policies/robot_tracking.hpp"
#include "geofence/spatial/queries/batch_point_checker.hpp"
#include "geofence/spatial/queries/obstacle_collision_checker.hpp"
#include "geofence/spatial/queries/obstacle_correspondence_matcher.hpp"
#include "geofence/spatial/queries/obstacle_match_checker.hpp"
#include "geofence/spatial/queries/path_safety_checker.hpp"
#include "geofence/common/queries/robot_tracking_checker.hpp"
#include "geofence/spatial/visualization/geofence_visualizer.hpp"
#include "spatial_index_selection.hpp"

// Third-party (ROS 2)
#include "rises_interfaces/msg/contours.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include "rises_interfaces/msg/obstacle_array.hpp"
#include "rises_interfaces/msg/obstacle_report.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/srv/get_area_state.hpp"
#include "rises_interfaces/srv/get_map_info.hpp"
#include "rises_interfaces/srv/get_safety_radius.hpp"
#include "rises_interfaces/srv/get_warehouse_contours.hpp"
#include "rises_interfaces/srv/set_area_state.hpp"
#include "rises_interfaces/srv/update_map.hpp"
#include "rises_interfaces/srv/update_warehouse_layout.hpp"
#include "rises_interfaces/srv/validate_path.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Standard library
#include <atomic>
#include <memory>

namespace rises
{

/**
 * @brief Spatial (nanoflann KD-tree) geofence lifecycle node.
 *
 * Backend specialization of LifecycleGeofenceNodeBase. The base owns all the
 * shared ROS scaffolding (TF2, subscriptions/publishers/services, lifecycle
 * skeletons, the centralized per-scan alert + report). This class adds the
 * spatial-index backend (GeofenceMap), the safety profile, diagnostics, the
 * latency recorder, and the four read-only query services.
 */
class GeofenceNode : public LifecycleGeofenceNodeBase
{
public:
    explicit GeofenceNode(const rclcpp::NodeOptions &options);

protected:
    // ---- Pure-virtual hooks from the base ----
    [[nodiscard]] GeofenceCommonConfig& commonConfig() override { return this->cfg_; }
    [[nodiscard]] const GeofenceCommonConfig& commonConfig() const override { return this->cfg_; }

    [[nodiscard]] ScanMatchResult matchScan(
        const rises_interfaces::msg::ObstacleArray::ConstSharedPtr &msg,
        const Point2D &robot_pos, bool have_pos) override;

    [[nodiscard]] bool loadObstaclesFromJson(const std::string &filepath) override;
    [[nodiscard]] bool loadContoursFromJson(const std::string &filepath) override;

    void applyMapUpdates(const rises_interfaces::msg::ObstacleUpdateArray &updates,
                         const char *source_tag) override;

    void validatePathService(
        const std::shared_ptr<rises_interfaces::srv::ValidatePath::Request> request,
        const std::shared_ptr<rises_interfaces::srv::ValidatePath::Response> response) override;
    void updateMapService(
        const std::shared_ptr<rises_interfaces::srv::UpdateMap::Request> request,
        const std::shared_ptr<rises_interfaces::srv::UpdateMap::Response> response) override;
    void updateWarehouseLayoutService(
        const std::shared_ptr<rises_interfaces::srv::UpdateWarehouseLayout::Request> request,
        const std::shared_ptr<rises_interfaces::srv::UpdateWarehouseLayout::Response> response) override;
    void areaStateService(
        const std::shared_ptr<rises_interfaces::srv::SetAreaState::Request> request,
        const std::shared_ptr<rises_interfaces::srv::SetAreaState::Response> response) override;

    // ---- Optional hooks from the base ----
    void onConfigureExtra() override;
    void onCleanupExtra() override;
    void setContours(const rises_interfaces::msg::Contours &contours) override;
    void onReportBuilt(const rises_interfaces::msg::ObstacleReport &report) override;
    [[nodiscard]] std::size_t backendObstacleCount() const override;
    void resetBackend() override;
    [[nodiscard]] bool backendReady() const override { return this->geofence_map_ != nullptr; }
    void setSafetyCircleRadius(float radius) override;

private:
    // Configuration (all ROS parameters)
    GeofenceConfig cfg_;

    // Spatial backend + safety profile.
    std::unique_ptr<rises::geofence::GeofenceMap> geofence_map_;
    rises::geofence::RobotSafetyProfile safety_profile_;

    // Read-only query services (spatial-only; registered in onConfigureExtra).
    rclcpp::Service<rises_interfaces::srv::GetAreaState>::SharedPtr get_area_state_srv_;
    rclcpp::Service<rises_interfaces::srv::GetSafetyRadius>::SharedPtr get_safety_radius_srv_;
    rclcpp::Service<rises_interfaces::srv::GetMapInfo>::SharedPtr get_map_info_srv_;
    rclcpp::Service<rises_interfaces::srv::GetWarehouseContours>::SharedPtr get_warehouse_contours_srv_;

    /// @brief Query whether a specific area is locked. Returns found=false if the ID does not exist.
    void getAreaStateService(
        const std::shared_ptr<rises_interfaces::srv::GetAreaState::Request> request,
        const std::shared_ptr<rises_interfaces::srv::GetAreaState::Response> response);
    /// @brief Returns the current safety circle outer radius in meters.
    void getSafetyRadiusService(
        const std::shared_ptr<rises_interfaces::srv::GetSafetyRadius::Request> request,
        const std::shared_ptr<rises_interfaces::srv::GetSafetyRadius::Response> response);
    /// @brief Returns obstacle count, contour status, and segment/polygon counts.
    void getMapInfoService(
        const std::shared_ptr<rises_interfaces::srv::GetMapInfo::Request> request,
        const std::shared_ptr<rises_interfaces::srv::GetMapInfo::Response> response);
    /// @brief Returns the current warehouse contours.
    void getWarehouseContoursService(
        const std::shared_ptr<rises_interfaces::srv::GetWarehouseContours::Request> request,
        const std::shared_ptr<rises_interfaces::srv::GetWarehouseContours::Response> response);

    /// Per-obstacle unmatched alert on the unmatched_obstacle topic (shape-mode only).
    void alertUnmatchedObstacle(const rises_interfaces::msg::Obstacle &obstacle,
                                const std_msgs::msg::Header &detection_header);

    // Diagnostics (spatial-only; registered in onConfigureExtra).
    std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    std::atomic<int64_t> last_scan_ns_{0};
    std::atomic<int64_t> scan_count_{0};
    std::atomic<int64_t> alert_count_{0};
    std::atomic<int64_t> last_diag_ns_{0};

    /// @brief Publishes geofence node health to /diagnostics.
    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    // Latency recording (optional, enabled via parameter).
    std::unique_ptr<rises::geofence::LatencyRecorder> latency_recorder_;
};

} // namespace rises
