#include "geofence/spatial/queries/obstacle_collision_checker.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include "geofence/utils/geometry_intersection.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"

namespace rises::geofence {

void ObstacleCollisionChecker::initialize(const Config &config) {
  collision_margin_ = config.collision_margin;
}

bool ObstacleCollisionChecker::checkCollision(
    const GeofenceMap &map,
    const rises_interfaces::msg::Obstacle &obstacle) {

  // Convert to Geometry variant
  const Geometry detected_geom = fromObstacleMsg(obstacle);
  const BoundingBox detected_bbox = getBoundingBox(detected_geom);

  // Expand bounding box by collision margin
  const float margin = collision_margin_;
  const BoundingBox search_box{
      detected_bbox.min_x - margin, detected_bbox.min_y - margin,
      detected_bbox.max_x + margin, detected_bbox.max_y + margin};

  // findFirstObstacleInRegion returns false on early exit (match) and true on
  // full iteration (no match). Negate so checkCollision returns true iff at
  // least one obstacle intersects the search region.
  return !map.findFirstObstacleInRegion(
      search_box, [&](const GeometryEntry &entry) {
        return intersects(detected_geom, entry.geometry);
      });
}

std::vector<int64_t> ObstacleCollisionChecker::findCollidingObstacles(
    const GeofenceMap &map,
    const rises_interfaces::msg::Obstacle &obstacle) {

  std::vector<int64_t> colliding_ids;

  // Convert to Geometry variant
  const Geometry detected_geom = fromObstacleMsg(obstacle);
  const BoundingBox detected_bbox = getBoundingBox(detected_geom);

  // Expand bounding box by collision margin
  const float margin = collision_margin_;
  const BoundingBox search_box{
      detected_bbox.min_x - margin, detected_bbox.min_y - margin,
      detected_bbox.max_x + margin, detected_bbox.max_y + margin};

  // Find all intersecting obstacles across ALL layers
  map.forEachObstacleInRegion(search_box, [&](const GeometryEntry &entry) {
    if (intersects(detected_geom, entry.geometry)) {
      colliding_ids.push_back(entry.id);
    }
  });

  return colliding_ids;
}

} // namespace rises::geofence
