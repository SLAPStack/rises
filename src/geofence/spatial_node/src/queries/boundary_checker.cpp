#include "geofence/spatial/queries/boundary_checker.hpp"
#include "geofence/spatial/shape/contour.hpp"

namespace rises::geofence {

bool BoundaryChecker::isPointInsideMapBounds(
    const GeofenceMap& map,
    const Point2D& point) {
    
    const rises::shape::MapBoundaryContours* contours = map.getMapContours();
    
    if (!contours) {
        return true;
    }
    
    return contours->isPointInside(point);
}

bool BoundaryChecker::isSegmentInsideMapBounds(
    const GeofenceMap& map,
    const Point2D& start,
    const Point2D& end) {
    
    return isPointInsideMapBounds(map, start) && 
           isPointInsideMapBounds(map, end);
}

} // namespace rises::geofence
