#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Messages
#include "rises_interfaces/msg/contours.hpp"
#include "rises_interfaces/msg/obstacle_array.hpp"
#include "rises_interfaces/msg/obstacle_report.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"

// Services
#include "rises_interfaces/srv/set_area_state.hpp"
#include "rises_interfaces/srv/update_map.hpp"
#include "rises_interfaces/srv/update_warehouse_layout.hpp"
#include "rises_interfaces/srv/validate_path.hpp"

// Shared base + config
#include "geofence/common/node/lifecycle_geofence_node_base.hpp"
#include "geofence/gridmap/node/gridmap_config.hpp"
#include "geofence/common/latency_recorder.hpp"

// Core map
#include "geofence/gridmap/map/gridmap.hpp"

// Shared spatial-node components (no spatial-index types - safe for gridmap)
#include "geofence/spatial/queries/obstacle_match_result.hpp"

namespace rises {

/**
 * @brief Gridmap-based geofence lifecycle node.
 *
 * Backend specialization of LifecycleGeofenceNodeBase using a 2-D occupancy
 * grid instead of an R-tree/KD-tree spatial index. Provides the same ROS
 * interface (topics, services, lifecycle) as GeofenceNode so the two
 * implementations are drop-in replaceable. The base owns all the shared ROS
 * scaffolding; this class adds the occupancy-grid backend and its
 * correspondence logic.
 *
 * Performance characteristics:
 *   + O(1) point queries (direct cell lookup)
 *   + Predictable, bounded memory (grid_w x grid_h x 1 bit)
 *   - Higher memory for sparse warehouse layouts
 *   - Discretisation artefacts at coarse resolutions (tune grid_resolution)
 */
class GeofenceGridmapNode : public LifecycleGeofenceNodeBase {
public:
    explicit GeofenceGridmapNode(const rclcpp::NodeOptions &options);

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
    [[nodiscard]] bool backendReady() const override { return this->gridmap_ != nullptr; }
    void resetBackend() override;
    /// Build the shared ObstacleReportBuilder so the gridmap node publishes the
    /// per-segment obstacle_report consumed by validation / FIWARE / KPIs. The
    /// base only declares report_builder_; each backend constructs it.
    void onConfigureExtra() override;
    void onCleanupExtra() override;

private:
    // Configuration (all ROS parameters).
    GridmapConfig cfg_;

    // Occupancy-grid backend.
    std::shared_ptr<geofence::GridMap> gridmap_;

    // Diagnostics (registered in onConfigureExtra).
    std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    std::atomic<int64_t> last_scan_ns_{0};
    std::atomic<int64_t> scan_count_{0};
    std::atomic<int64_t> alert_count_{0};

    /// @brief Publishes geofence gridmap node health to /diagnostics.
    void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    // Latency recording (optional, enabled via parameter).
    std::unique_ptr<rises::geofence::LatencyRecorder> latency_recorder_;

    /// Gridmap-specific correspondence check:
    ///   1. Safety-circle pre-filter (geometry intersection)
    ///   2. Grid cell lookup for each shape type (O(1) per sample point)
    [[nodiscard]] bool checkCorrespondence(const rises_interfaces::msg::Obstacle& obstacle,
                                           const Point2D& robot_pos) const;

    /// Classify each in-zone scan vertex (matched/unmatched vs the occupancy
    /// grid) per obstacle, producing the result ObstacleReportBuilder needs to
    /// emit a per-segment ObstacleReport with real positions for the KPI
    /// pipeline. Vertices outside the safety circle are left in neither set.
    [[nodiscard]] rises::geofence::query::ObstacleMatchResult buildMatchResult(
        const rises_interfaces::msg::ObstacleArray::ConstSharedPtr& msg,
        const Point2D& robot_pos) const;
};

} // namespace rises
