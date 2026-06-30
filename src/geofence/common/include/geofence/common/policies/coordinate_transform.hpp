#pragma once

#include <vector>
#include <memory>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include "rises_interfaces/msg/obstacle.hpp"
#include "rises_interfaces/msg/contours.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace rises::geofence
{

/**
 * @brief Utility class for coordinate transformations using 3D affine matrices.
 * 
 * Provides static transformation methods for obstacles, boundaries, and areas.
 * Uses Boost.Geometry matrix transformers for efficient 3D affine transformations.
 * 
 * Matrix format: [a, b, c, tx, d, e, f, ty, g, h, i, tz, p, q, r, w]
 * Represents:
 *   | a  b  c  tx |
 *   | d  e  f  ty |
 *   | g  h  i  tz |
 *   | p  q  r  w  |
 * 
 * Applied as (homogeneous coordinates):
 *   x' = (a*x + b*y + c*z + tx) / w'
 *   y' = (d*x + e*y + f*z + ty) / w'
 *   z' = (g*x + h*y + i*z + tz) / w'
 *   w' = p*x + q*y + r*z + w
 * 
 * For standard affine transformations, use p=0, q=0, r=0, w=1
 */
class CoordinateTransform
{
public:
    using Matrix3DTransformer = boost::geometry::strategy::transform::matrix_transformer<double, 3, 3>;
    using Point3D = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;

    /**
     * @brief Configuration for a single coordinate transformation.
     */
    struct TransformConfig {
        bool enabled = false;
        std::vector<double> matrix;  // 16-element matrix [a,b,c,tx, d,e,f,ty, g,h,i,tz, p,q,r,w]
        
        // Default: identity matrix
        TransformConfig() : enabled(false), matrix{
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        } {}
        
        TransformConfig(bool en, const std::vector<double>& mat) : enabled(en), matrix(mat) {}
    };

    /**
     * @brief Configuration for all coordinate transformations.
     */
    struct Config {
        TransformConfig obstacle;   // Transformation for obstacle coordinates
        TransformConfig boundary;   // Transformation for boundary/contour coordinates
        TransformConfig area;       // Transformation for area coordinates
        bool negate_y = false;      // Negate Y coordinate in ROS space (flip forward/backward)
    };

    /**
     * @brief Initialize transformation from configuration.
     * @param config Configuration containing all three transformation matrices
     */
    static void initialize(const Config& config);

    /**
     * @brief Reset all transformations to identity (disabled).
     */
    static void reset();

    // ========== Low-level coordinate transformation ==========
    
    static void transformObstaclePoint(double& x, double& y, double& z);
    static void transformBoundaryPoint(double& x, double& y, double& z);
    static void transformAreaPoint(double& x, double& y, double& z);

    // ========== ROS message transformation ==========
    
    /**
     * @brief Transform geometry_msgs::Point (obstacle or area based on flag).
     */
    static void transformGeometryPoint(geometry_msgs::msg::Point& point, bool is_obstacle);
    
    /**
     * @brief Transform geometry_msgs::Point32 (boundary data).
     */
    static void transformGeometryPoint32(geometry_msgs::msg::Point32& point);
    
    /**
     * @brief Transform complete Obstacle message including all coordinate fields.
     * Handles position, vertices, and all geometry types (POINT, CIRCLE, RECTANGLE, POLYGON, LINE).
     */
    static void transformObstacle(rises_interfaces::msg::Obstacle& obstacle);
    
    /**
     * @brief Transform Contours message including outer hull, segments, and inner contours.
     */
    static void transformContours(rises_interfaces::msg::Contours& contours);

    // ========== Query methods ==========
    
    [[nodiscard]] static bool isObstacleTransformEnabled() { return obstacle_enabled_; }
    [[nodiscard]] static bool isBoundaryTransformEnabled() { return boundary_enabled_; }
    [[nodiscard]] static bool isAreaTransformEnabled() { return area_enabled_; }

private:
    // Transformation state
    static bool obstacle_enabled_;
    static bool boundary_enabled_;
    static bool area_enabled_;
    static bool negate_y_;

    // Cached transformers (recreated on initialize())
    static std::unique_ptr<Matrix3DTransformer> obstacle_transformer_;
    static std::unique_ptr<Matrix3DTransformer> boundary_transformer_;
    static std::unique_ptr<Matrix3DTransformer> area_transformer_;

    // Helper: apply transformer to point
    static void applyTransform(double& x, double& y, double& z, const Matrix3DTransformer& transformer);
};

} // namespace rises::geofence
