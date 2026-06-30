#pragma once

#include "rises_interfaces/msg/obstacle.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include "geometry_types.hpp"
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace rises {

// Use the shared backend-agnostic point type (geometry_types.hpp) rather than a
// second, distinct rises::Point2D. The two were layout-identical, but as two
// separate types they poisoned unqualified Point2D name resolution inside
// namespace rises once both headers were in scope (e.g. via the shared node
// base), breaking the json_loader contour templates and the base matchScan
// override. Aliasing to the global ::Point2D unifies the type everywhere.
using Point2D = ::Point2D;

namespace geofence {

/**
 * @brief Immutable occupancy grid data structure
 *
 * Read-only data class containing the grid state and obstacle metadata.
 * All modifications create new instances via copy-on-write.
 * Thread-safe for concurrent reads via RCU snapshots.
 *
 * Coordinate System:
 * - World coordinates: floating-point (x, y) in meters
 * - Grid coordinates: integer (grid_x, grid_y) cell indices
 * - Origin: configurable offset (default: -50, -50)
 * - Resolution: meters per cell (default: 0.05m = 5cm)
 *
 * Memory Layout:
 * - Grid stored as std::vector<uint8_t> (1 byte per cell; SIMD-friendly)
 * - Per-layer ref counts track how many obstacles occupy each cell (enables
 * O(1) removal)
 * - Obstacles stored in hash map with occupied cell indices
 * - Example: 100m×100m @ 5cm = 2000×2000 cells = 4MB per grid layer
 */
class GridMapData {
public:
  struct Config {
    double resolution = 0.05;     // Grid cell size in meters (5cm default)
    double width_meters = 100.0;  // Map width in meters
    double height_meters = 100.0; // Map height in meters
    double origin_x = -50.0;      // Map origin X (world coordinates)
    double origin_y = -50.0;      // Map origin Y (world coordinates)
  };

  /**
   * @brief Obstacle metadata stored separately from grid
   *
   * Design rationale:
   * - Grid stores only occupied/free (1 bit per cell)
   * - ObstacleInfo maps obstacle ID to its occupied cells
   * - Allows efficient removal (clear only cells belonging to obstacle)
   * - Supports overlapping obstacles (multiple IDs can occupy same cell)
   * - Uses uint32_t indices: sufficient for grids up to 2^32 cells (~65k×65k)
   */
  struct ObstacleInfo {
    rises_interfaces::msg::Obstacle obstacle;
    std::vector<uint32_t> occupied_cells; // 1D grid indices (uint32_t
                                          // sufficient for reasonable grids)
    ObstacleLayer layer{
        ObstacleLayer::STATIC}; ///< FIXED / STATIC / DYNAMIC classification
  };

  /**
   * @brief Construct empty grid
   */
  explicit GridMapData(const Config &config);

  /**
   * @brief Copy constructor for copy-on-write
   */
  GridMapData(const GridMapData &other) = default;

  // Query methods (const, thread-safe)
  bool isOccupied(const double x, const double y) const;

  /**
   * @brief Layer-aware occupancy check.
   * @param mask Bitmask of layers to include (e.g. ObstacleLayer::FIXED |
   * ObstacleLayer::STATIC)
   * @return true if the cell is occupied by an obstacle in any of the requested
   * layers
   */
  bool isOccupied(const double x, const double y,
                  const ObstacleLayer mask) const;

  /**
   * @brief Check if line path is blocked by obstacles in the given layer mask.
   */
  bool isPathBlocked(const double x1, const double y1, const double x2,
                     const double y2, const ObstacleLayer mask) const;

  std::vector<int64_t> findObstaclesNear(const double x, const double y,
                                         const double radius) const;

  /**
   * @brief Find all obstacles that have at least one cell within the safety
   * circle.
   *
   * An obstacle is included if ANY of its occupied cells intersects with the
   * circle. This is useful for finding obstacles near the robot for safety
   * checks.
   */
  std::vector<int64_t> findObstaclesInSafetyCircle(const double center_x,
                                                   const double center_y,
                                                   const double radius) const;

  bool isPathBlocked(const double x1, const double y1, const double x2,
                     const double y2) const;

  /**
   * @brief Clear all obstacles belonging to a specific layer and rebuild the
   * merged grid.
   */
  void clearLayer(const ObstacleLayer layer);

  /**
   * @brief Count obstacles in a specific layer.
   */
  std::size_t getObstacleCount(const ObstacleLayer layer) const;

  // Grid metadata accessors
  inline std::size_t getGridWidth() const { return this->grid_width_; }
  inline std::size_t getGridHeight() const { return this->grid_height_; }
  inline double getResolution() const { return this->resolution_; }
  inline double getOriginX() const { return this->origin_x_; }
  inline double getOriginY() const { return this->origin_y_; }
  inline const Config &getConfig() const { return this->config_; }

  // Obstacle accessors (const)
  inline const std::unordered_map<int64_t, ObstacleInfo> &getObstacles() const {
    return this->obstacles_;
  }

  // Direct grid access for policy classes only.
  // Mutable accessors allow in-place modification during copy-on-write updates.
  // External code receives const snapshots via RCU and never calls these.
  inline const std::vector<uint8_t> &getGrid() const { return this->grid_; }
  inline std::vector<uint8_t> &getMutableGrid() { return this->grid_; }
  inline std::unordered_map<int64_t, ObstacleInfo> &getMutableObstacles() {
    return this->obstacles_;
  }

  /// Per-layer grid accessor — FIXED/STATIC/DYNAMIC. Policy classes only.
  std::vector<uint8_t> &getMutableLayerGrid(const ObstacleLayer layer);
  const std::vector<uint8_t> &getLayerGrid(const ObstacleLayer layer) const;

  /// Per-layer reference count accessor. Counts how many obstacles occupy each
  /// cell. Non-zero means at least one obstacle is present; used for O(1)
  /// removal checks.
  std::vector<uint16_t> &getMutableRefCount(const ObstacleLayer layer);

  // Coordinate conversion helpers.
  // Uses pre-computed inverse resolution for fast multiply instead of divide.
  // Returns false if the point falls outside grid bounds OR if either
  // coordinate is non-finite (NaN / +Inf / -Inf). Casting a non-finite float
  // to int is undefined behaviour on x86 (and on MSVC), so the guard must come
  // before the static_cast.
  inline bool worldToGrid(const double x, const double y, int &grid_x,
                          int &grid_y) const {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return false;
    }
    grid_x = static_cast<int>((x - this->origin_x_) * this->inv_resolution_);
    grid_y = static_cast<int>((y - this->origin_y_) * this->inv_resolution_);
    return grid_x >= 0 && grid_x < static_cast<int>(this->grid_width_) &&
           grid_y >= 0 && grid_y < static_cast<int>(this->grid_height_);
  }

  inline uint32_t gridToIndex(const int grid_x, const int grid_y) const {
    return static_cast<uint32_t>(grid_y) *
               static_cast<uint32_t>(this->grid_width_) +
           static_cast<uint32_t>(grid_x);
  }

private:
  // Grid parameters
  Config config_;
  double resolution_;
  double inv_resolution_;
  std::size_t grid_width_;
  std::size_t grid_height_;
  double origin_x_;
  double origin_y_;

  // Grid storage: occupied/free state (1 byte per cell; SIMD-friendly)
  // grid_  = merged (ALL layers – used for backward-compatible isOccupied())
  // grid_fixed_/static_/dynamic_ = per-layer grids for layer-filtered queries
  std::vector<uint8_t> grid_;
  std::vector<uint8_t> grid_fixed_;
  std::vector<uint8_t> grid_static_;
  std::vector<uint8_t> grid_dynamic_;

  // Per-layer reference counts: ref_X_[cell_idx] = number of obstacles in layer
  // X that occupy this cell. Enables O(cells_per_obstacle) removal without full
  // grid scan.
  std::vector<uint16_t> ref_fixed_;
  std::vector<uint16_t> ref_static_;
  std::vector<uint16_t> ref_dynamic_;

  // Obstacle metadata
  std::unordered_map<int64_t, ObstacleInfo> obstacles_;
};

} // namespace geofence
} // namespace rises
