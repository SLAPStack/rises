#pragma once

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include "geometry_types.hpp"  // Provides Point2D definition
#include "rises_interfaces/msg/obstacle.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace rises {
namespace shape {

/**
 * @class PolygonContour
 * @brief Backend-agnostic closed polygonal contour for geometric containment and matching.
 *
 * Uses Point2D (double x, y) for backend independence.
 * No Eigen or Boost dependencies - pure standard library.
 */
class PolygonContour {
public:
    /// Default constructor: creates an empty contour.
    PolygonContour() = default;

    /// Construct from Point2D vertices
    explicit PolygonContour(const std::vector<Point2D>& vertices)
        : vertices_(vertices) {}

    // -------------------------------------------------------------------------
    // Vertex Accessors / Mutators
    // -------------------------------------------------------------------------

    void setVertices(const std::vector<Point2D>& vertices) {
        vertices_ = vertices;
    }

    const std::vector<Point2D>& getVertices() const {
        return vertices_;
    }

    void addVertex(const Point2D& v) {
        vertices_.push_back(v);
    }

    void addVertex(double x, double y) {
        vertices_.push_back({x, y});
    }

    // -------------------------------------------------------------------------
    // Geometric Utilities
    // -------------------------------------------------------------------------

    /**
     * @brief Compute the centroid (average position) of the polygon vertices.
     */
    Point2D computeCentroid() const {
        if (vertices_.empty()) return {0.0, 0.0};
        
        double sum_x = 0.0;
        double sum_y = 0.0;
        for (const Point2D& v : vertices_) {
            sum_x += v.x;
            sum_y += v.y;
        }
        return {sum_x / vertices_.size(), sum_y / vertices_.size()};
    }

    /**
     * @brief Compute the axis-aligned bounding box of this polygon.
     */
    void computeBoundingBox(Point2D& min_out, Point2D& max_out) const {
        min_out = {std::numeric_limits<double>::infinity(), 
                   std::numeric_limits<double>::infinity()};
        max_out = {-std::numeric_limits<double>::infinity(), 
                   -std::numeric_limits<double>::infinity()};

        for (const Point2D& v : vertices_) {
            min_out.x = std::min(min_out.x, v.x);
            min_out.y = std::min(min_out.y, v.y);
            max_out.x = std::max(max_out.x, v.x);
            max_out.y = std::max(max_out.y, v.y);
        }
    }

    /**
     * @brief Ray casting algorithm for point-in-polygon test.
     * @return true if point is strictly inside the polygon.
     *
     * Boundary convention: a point lying exactly on an edge is reported as
     * OUTSIDE (half-open). This is a deliberate, documented safety convention
     * for a geofence -- ambiguous boundary points are conservatively excluded
     * rather than included. It also sidesteps the PNPOLY edge case where a
     * point on one edge can spuriously "cross" an unrelated edge that shares
     * the test point's y-coordinate at an endpoint.
     */
    bool containsPoint(const Point2D& p) const {
        if (vertices_.size() < 3) return false;

        // Boundary points are treated as outside (see method docs).
        if (closestEdgeDistanceSq(p) < 1e-9 * 1e-9) return false;

        bool inside = false;
        const std::size_t n = vertices_.size();

        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const Point2D& vi = vertices_[i];
            const Point2D& vj = vertices_[j];

            if ((vi.y > p.y) != (vj.y > p.y) &&
                p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x) {
                inside = !inside;
            }
        }
        return inside;
    }

    /**
     * @brief Check if point is inside or within epsilon distance to edge.
     */
    bool containsPointWithMargin(const Point2D& point, const double epsilon = 1e-6) const {
        if (this->containsPoint(point)) return true;

        // Near-or-on the boundary counts as contained for the margin test,
        // independent of what containsPoint() returned for exact-boundary points.
        return closestEdgeDistanceSq(point) < epsilon * epsilon;
    }

private:
    /**
     * @brief Minimum squared distance from @p point to any polygon edge.
     *
     * Shared by containsPoint() (boundary test) and containsPointWithMargin()
     * (near-edge test). Returns +inf for an empty contour (no edges).
     */
    double closestEdgeDistanceSq(const Point2D& point) const {
        double min_dist_sq = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < vertices_.size(); ++i) {
            const Point2D& a = vertices_[i];
            const Point2D& b = vertices_[(i + 1) % vertices_.size()];

            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double len_sq = dx * dx + dy * dy;

            if (len_sq < 1e-12) continue; // Degenerate edge

            const double t = std::clamp(
                ((point.x - a.x) * dx + (point.y - a.y) * dy) / len_sq,
                0.0, 1.0
            );

            const Point2D closest = {a.x + t * dx, a.y + t * dy};
            const double dist_sq = (point.x - closest.x) * (point.x - closest.x) +
                                   (point.y - closest.y) * (point.y - closest.y);

            min_dist_sq = std::min(min_dist_sq, dist_sq);
        }
        return min_dist_sq;
    }

    std::vector<Point2D> vertices_;
};

// ============================================================================

/**
 * @struct LineSegment2D
 * @brief Represents a line segment between two points.
 * Used for representing wall segments with gaps (doors).
 */
struct LineSegment2D {
    Point2D start;
    Point2D end;

    LineSegment2D() = default;
    LineSegment2D(const Point2D& s, const Point2D& e) : start(s), end(e) {}
    LineSegment2D(double x1, double y1, double x2, double y2)
        : start{x1, y1}, end{x2, y2} {}
};

// ============================================================================

/**
 * @class MapBoundaryContours
 * @brief Complete map boundary with outer contours and optional segments.
 *
 * Represents the navigable area boundary with:
 * - outer_contour_: Closed polygon hull (for fast containment checks)
 * - outer_segments_: Line segments representing actual walls (may have gaps/doors)
 * - inner_contours_: Interior holes/obstacles
 *
 * Backend-agnostic implementation using Point2D.
 */
class MapBoundaryContours {
public:
    MapBoundaryContours() = default;

    MapBoundaryContours(const PolygonContour& outer,
                        const std::vector<PolygonContour>& inners = {},
                        const std::vector<LineSegment2D>& segments = {})
        : outer_contour_(outer), inner_contours_(inners), outer_segments_(segments) {}

    // -------------------------------------------------------------------------
    // Accessors / Mutators
    // -------------------------------------------------------------------------

    void setOuterContour(const PolygonContour& outer) {
        outer_contour_ = outer;
    }

    void addInnerContour(const PolygonContour& inner) {
        inner_contours_.push_back(inner);
    }

    void setOuterSegments(const std::vector<LineSegment2D>& segments) {
        outer_segments_ = segments;
    }

    void addOuterSegment(const LineSegment2D& segment) {
        outer_segments_.push_back(segment);
    }

    const PolygonContour& getOuterContour() const {
        return outer_contour_;
    }

    const std::vector<PolygonContour>& getInnerContours() const {
        return inner_contours_;
    }

    const std::vector<LineSegment2D>& getOuterSegments() const {
        return outer_segments_;
    }

    bool hasSegments() const {
        return !outer_segments_.empty();
    }

    // -------------------------------------------------------------------------
    // Containment Tests
    // -------------------------------------------------------------------------

    /**
     * @brief Check if a point lies inside the map boundary area.
     * Point must be inside outer contour and outside all inner (hole) contours.
     */
    bool isPointInside(const Point2D& point) const {
        if (!outer_contour_.containsPoint(point)) return false;
        
        for (const PolygonContour& hole : inner_contours_) {
            if (hole.containsPoint(point)) return false;
        }
        return true;
    }

private:
    PolygonContour outer_contour_;                    // Convex hull for fast containment
    std::vector<PolygonContour> inner_contours_;      // Interior holes
    std::vector<LineSegment2D> outer_segments_;       // Actual wall segments (may have gaps/doors)
};

} // namespace shape
} // namespace rises
