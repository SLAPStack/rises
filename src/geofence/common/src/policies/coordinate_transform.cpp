#include "geofence/common/policies/coordinate_transform.hpp"

#include <cmath>

namespace rises::geofence
{

// Static member initialization
bool CoordinateTransform::obstacle_enabled_ = false;
bool CoordinateTransform::boundary_enabled_ = false;
bool CoordinateTransform::area_enabled_ = false;
bool CoordinateTransform::negate_y_ = false;

std::unique_ptr<CoordinateTransform::Matrix3DTransformer> CoordinateTransform::obstacle_transformer_;
std::unique_ptr<CoordinateTransform::Matrix3DTransformer> CoordinateTransform::boundary_transformer_;
std::unique_ptr<CoordinateTransform::Matrix3DTransformer> CoordinateTransform::area_transformer_;

void CoordinateTransform::initialize(const Config& config)
{
    negate_y_ = config.negate_y;
    
    obstacle_enabled_ = config.obstacle.enabled;
    if (config.obstacle.enabled && config.obstacle.matrix.size() == 16) {
        const std::vector<double>& matrix = config.obstacle.matrix;
        obstacle_transformer_ = std::make_unique<Matrix3DTransformer>(
            matrix[0], matrix[1], matrix[2], matrix[3],     // row 1: a, b, c, tx
            matrix[4], matrix[5], matrix[6], matrix[7],     // row 2: d, e, f, ty
            matrix[8], matrix[9], matrix[10], matrix[11],   // row 3: g, h, i, tz
            matrix[12], matrix[13], matrix[14], matrix[15]  // row 4: p, q, r, w (homogeneous)
        );
    }

    boundary_enabled_ = config.boundary.enabled;
    if (config.boundary.enabled && config.boundary.matrix.size() == 16) {
        const std::vector<double>& matrix = config.boundary.matrix;
        boundary_transformer_ = std::make_unique<Matrix3DTransformer>(
            matrix[0], matrix[1], matrix[2], matrix[3],
            matrix[4], matrix[5], matrix[6], matrix[7],
            matrix[8], matrix[9], matrix[10], matrix[11],
            matrix[12], matrix[13], matrix[14], matrix[15]
        );
    }

    area_enabled_ = config.area.enabled;
    if (config.area.enabled && config.area.matrix.size() == 16) {
        const std::vector<double>& matrix = config.area.matrix;
        area_transformer_ = std::make_unique<Matrix3DTransformer>(
            matrix[0], matrix[1], matrix[2], matrix[3],
            matrix[4], matrix[5], matrix[6], matrix[7],
            matrix[8], matrix[9], matrix[10], matrix[11],
            matrix[12], matrix[13], matrix[14], matrix[15]
        );
    }
}

void CoordinateTransform::reset()
{
    obstacle_enabled_ = false;
    boundary_enabled_ = false;
    area_enabled_ = false;
    negate_y_ = false;
    obstacle_transformer_.reset();
    boundary_transformer_.reset();
    area_transformer_.reset();
}

void CoordinateTransform::applyTransform(double& x, double& y, double& z, const Matrix3DTransformer& transformer)
{
    Point3D input(x, y, z);
    Point3D output;
    boost::geometry::transform(input, output, transformer);

    x = boost::geometry::get<0>(output);
    y = boost::geometry::get<1>(output);
    z = boost::geometry::get<2>(output);
}

void CoordinateTransform::transformObstaclePoint(double& x, double& y, double& z)
{
    if (obstacle_enabled_ && obstacle_transformer_) {
        applyTransform(x, y, z, *obstacle_transformer_);
    }
    if (negate_y_) {
        y = -y;
    }
}

void CoordinateTransform::transformBoundaryPoint(double& x, double& y, double& z)
{
    if (boundary_enabled_ && boundary_transformer_) {
        applyTransform(x, y, z, *boundary_transformer_);
    }
    // negate_y is intentionally NOT applied to boundaries — it only
    // transforms obstacle coordinates into the contour reference frame.
}

void CoordinateTransform::transformAreaPoint(double& x, double& y, double& z)
{
    if (area_enabled_ && area_transformer_) {
        applyTransform(x, y, z, *area_transformer_);
    }
    // negate_y is intentionally NOT applied to areas — same rationale
    // as boundaries.
}

void CoordinateTransform::transformGeometryPoint(geometry_msgs::msg::Point& point, const bool is_obstacle)
{
    double x = point.x;
    double y = point.y;
    double z = point.z;
    
    if (is_obstacle) {
        transformObstaclePoint(x, y, z);
    } else {
        transformAreaPoint(x, y, z);
    }
    
    point.x = x;
    point.y = y;
    point.z = z;
}

void CoordinateTransform::transformGeometryPoint32(geometry_msgs::msg::Point32& point)
{
    double x = static_cast<double>(point.x);
    double y = static_cast<double>(point.y);
    double z = static_cast<double>(point.z);
    
    transformBoundaryPoint(x, y, z);
    
    point.x = static_cast<float>(x);
    point.y = static_cast<float>(y);
    point.z = static_cast<float>(z);
}

void CoordinateTransform::transformObstacle(rises_interfaces::msg::Obstacle& obstacle)
{
    if (!obstacle_enabled_ && !negate_y_) return;

    // RECTANGLE obstacles from the AABB converter have center + width/height
    // but empty vertices. Generate corner vertices, transform them, and
    // reconstruct the AABB so both spatial node (vertices path) and
    // gridmap node (width/height path) get correct transformed geometry.
    // Only needed when a matrix transform is active (which may rotate/skew).
    // Pure Y-negation preserves axis-alignment and is handled per-point below.
    if (obstacle_enabled_ && obstacle_transformer_ &&
        obstacle.type == rises_interfaces::msg::Obstacle::RECTANGLE &&
        obstacle.vertices.empty() &&
        obstacle.width > 0.0f && obstacle.height > 0.0f) {

        const double px = obstacle.position.x;
        const double py = obstacle.position.y;
        const double pz = obstacle.position.z;
        const double half_width = static_cast<double>(obstacle.width) * 0.5;
        const double half_height = static_cast<double>(obstacle.height) * 0.5;
        const double cos_orientation = std::cos(static_cast<double>(obstacle.orientation));
        const double sin_orientation = std::sin(static_cast<double>(obstacle.orientation));

        // Generate 4 corner vertices from center + dimensions + orientation
        obstacle.vertices.resize(4);
        obstacle.vertices[0].x = px + half_width * cos_orientation - half_height * sin_orientation;
        obstacle.vertices[0].y = py + half_width * sin_orientation + half_height * cos_orientation;
        obstacle.vertices[0].z = pz;
        obstacle.vertices[1].x = px - half_width * cos_orientation - half_height * sin_orientation;
        obstacle.vertices[1].y = py - half_width * sin_orientation + half_height * cos_orientation;
        obstacle.vertices[1].z = pz;
        obstacle.vertices[2].x = px - half_width * cos_orientation + half_height * sin_orientation;
        obstacle.vertices[2].y = py - half_width * sin_orientation - half_height * cos_orientation;
        obstacle.vertices[2].z = pz;
        obstacle.vertices[3].x = px + half_width * cos_orientation + half_height * sin_orientation;
        obstacle.vertices[3].y = py + half_width * sin_orientation - half_height * cos_orientation;
        obstacle.vertices[3].z = pz;

        // Transform all 4 corners
        for (geometry_msgs::msg::Point& vertex : obstacle.vertices) {
            transformGeometryPoint(vertex, true);
        }

        // Reconstruct AABB from transformed corners for gridmap compatibility
        double min_x = obstacle.vertices[0].x;
        double max_x = obstacle.vertices[0].x;
        double min_y = obstacle.vertices[0].y;
        double max_y = obstacle.vertices[0].y;
        for (std::size_t i = 1; i < obstacle.vertices.size(); ++i) {
            const geometry_msgs::msg::Point& v = obstacle.vertices[i];
            min_x = std::min(min_x, v.x);
            max_x = std::max(max_x, v.x);
            min_y = std::min(min_y, v.y);
            max_y = std::max(max_y, v.y);
        }

        obstacle.position.x = (min_x + max_x) * 0.5;
        obstacle.position.y = (min_y + max_y) * 0.5;
        obstacle.position.z = obstacle.vertices[0].z;
        obstacle.width = static_cast<float>(max_x - min_x);
        obstacle.height = static_cast<float>(max_y - min_y);
        obstacle.orientation = 0.0f;
        return;
    }

    transformGeometryPoint(obstacle.position, true);

    for (geometry_msgs::msg::Point& vertex : obstacle.vertices) {
        transformGeometryPoint(vertex, true);
    }
}

void CoordinateTransform::transformContours(rises_interfaces::msg::Contours& contours)
{
    if (!boundary_enabled_) return;
    
    for (geometry_msgs::msg::Point32& point : contours.outer_contour_hull.points) {
        transformGeometryPoint32(point);
    }
    
    for (rises_interfaces::msg::LineSegment& segment : contours.outer_contour_segments) {
        transformGeometryPoint32(segment.start);
        transformGeometryPoint32(segment.end);
    }
    
    for (geometry_msgs::msg::Polygon& polygon : contours.inner_contours) {
        for (geometry_msgs::msg::Point32& point : polygon.points) {
            transformGeometryPoint32(point);
        }
    }
}

} // namespace rises::geofence
