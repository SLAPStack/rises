#include "geofence/spatial/utils/json_loader.hpp"
#include "geofence/common/policies/coordinate_transform.hpp"
#include "geofence/spatial/map/geofence_map.hpp"
#include "rises_interfaces/msg/obstacle.hpp"

namespace rises::geofence::utils
{

rises::geofence::Geometry JsonLoader::obstacleToGeometry(
    rises_interfaces::msg::Obstacle obstacle_msg,
    bool apply_transform)
{
    if (apply_transform) {
        rises::geofence::CoordinateTransform::transformObstacle(obstacle_msg);
    }
    return rises::geofence::fromObstacleMsg(obstacle_msg);
}

} // namespace rises::geofence::utils
