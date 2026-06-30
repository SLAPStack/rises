#include "laserscan_preprocessor/shapes/shape_fitter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <random>

#ifdef USE_SIMD
#include <xsimd/xsimd.hpp>
#endif

namespace rises::shapes {

// ---------------- Point Fitter Implementation ----------------

rises_interfaces::msg::Obstacle PointFitter::fitShape(
    int segment_id, 
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) {
    
    (void)accumulated_angle; // Unused for point fitting
    
    rises_interfaces::msg::Obstacle obs;
    obs.id = segment_id;
    obs.type = rises_interfaces::msg::Obstacle::POINT;
    
    if (points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("PointFitter"),
                   "Empty points vector for segment %d", segment_id);
        return obs;
    }

    // Compute centroid once — used for both position and single-point vertex
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const Eigen::Vector2f& p : points) {
        centroid += p;
    }
    centroid /= static_cast<float>(points.size());

    // Add all points as individual vertices in points-only mode
    if (this->points_only_mode_ || points.size() == 1) {
        for (const Eigen::Vector2f& p : points) {
            geometry_msgs::msg::Point pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.z = 0.0f;
            obs.vertices.push_back(pt);
        }
    } else {
        // Single representative point (centroid)
        geometry_msgs::msg::Point pt;
        pt.x = centroid.x();
        pt.y = centroid.y();
        pt.z = 0.0f;
        obs.vertices.push_back(pt);
    }

    obs.position.x = centroid.x();
    obs.position.y = centroid.y();
    obs.position.z = 0.0f;

    return obs;
}

float PointFitter::getConfidence(
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) const {
    
    (void)accumulated_angle; // Unused
    
    if (this->points_only_mode_) return 1.0f; // Always best for points-only mode
    if (points.size() == 1) return 1.0f; // Perfect for single points
    
    return 0.1f; // Low confidence for multi-point clusters (fallback only)
}

// ---------------- Line Fitter Implementation ----------------

rises_interfaces::msg::Obstacle LineFitter::fitShape(
    int segment_id,
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) {
    (void)accumulated_angle;  // Unused parameter
    
    rises_interfaces::msg::Obstacle obs;
    obs.id = segment_id;
    obs.type = rises_interfaces::msg::Obstacle::LINE;
    
    if (points.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("LineFitter"),
                   "Not enough points for line fitting: %zu", points.size());
        return obs;
    }
    
    // Try RANSAC line fitting first
    std::pair<Eigen::Vector3f, std::vector<std::size_t>> line_result = this->ransacLineFitting(points);
    const Eigen::Vector3f& line_params = line_result.first;
    const std::vector<std::size_t>& inliers = line_result.second;
    const float inlier_ratio = static_cast<float>(inliers.size()) / points.size();
    
    // Compute centroid
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const Eigen::Vector2f& p : points) {
        centroid += p;
    }
    centroid /= static_cast<float>(points.size());

    obs.position.x = centroid.x();
    obs.position.y = centroid.y();
    obs.position.z = 0.0f;
    
    Eigen::Vector2f direction;
    if (inlier_ratio > 0.6f && line_params.norm() > 1e-6f) {
        // Use RANSAC result: line equation ax + by + c = 0 -> direction = (-b, a)
        direction = Eigen::Vector2f(-line_params[1], line_params[0]).normalized();
    } else {
        // Fallback to PCA
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const Eigen::Vector2f& p : points) {
            const Eigen::Vector2f d = p - centroid;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(points.size());
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
        direction = solver.eigenvectors().col(1).normalized();
    }
    
    // Project points onto direction to find line extent
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();
    
    for (const Eigen::Vector2f& p : points) {
        const float proj = (p - centroid).dot(direction);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }
    
    Eigen::Vector2f start = centroid + min_proj * direction;
    Eigen::Vector2f end = centroid + max_proj * direction;
    
    geometry_msgs::msg::Point pt1, pt2;
    pt1.x = start.x(); pt1.y = start.y(); pt1.z = 0.0f;
    pt2.x = end.x(); pt2.y = end.y(); pt2.z = 0.0f;
    
    obs.vertices.push_back(pt1);
    obs.vertices.push_back(pt2);
    
    return obs;
}

float LineFitter::getConfidence(
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) const {
    
    if (points.size() < 2) return 0.0f;
    
    // High confidence for low accumulated angle (linear segments)
    const float max_linear_angle = M_PI / 6.0f; // 30 degrees
    if (accumulated_angle < max_linear_angle) {
        return 0.8f;
    }
    
    // Medium confidence for moderate angles
    return 0.4f;
}

std::pair<Eigen::Vector3f, std::vector<std::size_t>> LineFitter::ransacLineFitting(
    const std::vector<Eigen::Vector2f>& points) const {
    
    Eigen::Vector3f best_line = Eigen::Vector3f::Zero();
    std::vector<std::size_t> best_inliers;
    
    if (points.size() < 2) {
        return std::make_pair(best_line, best_inliers);
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<std::size_t> dis(0, points.size() - 1);

#ifdef USE_SIMD
    const std::size_t n = points.size();
    std::vector<float> pts_x(n), pts_y(n), dists(n);
    for (std::size_t i = 0; i < n; ++i) {
        pts_x[i] = points[i].x();
        pts_y[i] = points[i].y();
    }
    using simd_t = xsimd::batch<float>;
    constexpr std::size_t batch_size = simd_t::size;
    const std::size_t simd_end = n - (n % batch_size);
#endif

    for (int iter = 0; iter < this->max_iterations_; ++iter) {
        // Sample two different points
        std::size_t idx1 = dis(gen);
        std::size_t idx2 = dis(gen);
        while (idx2 == idx1 && points.size() > 1) {
            idx2 = dis(gen);
        }

        const Eigen::Vector2f& p1 = points[idx1];
        const Eigen::Vector2f& p2 = points[idx2];

        if ((p2 - p1).norm() < 1e-6f) continue;

        // Line equation: ax + by + c = 0
        Eigen::Vector3f line;
        line[0] = p2.y() - p1.y();  // a
        line[1] = p1.x() - p2.x();  // b
        line[2] = p2.x() * p1.y() - p1.x() * p2.y(); // c

        const float norm = std::sqrt(line[0] * line[0] + line[1] * line[1]);
        if (norm < 1e-6f) continue;
        line /= norm;

        // Count inliers
        std::vector<std::size_t> inliers;
#ifdef USE_SIMD
        {
            const simd_t a_v(line[0]);
            const simd_t b_v(line[1]);
            const simd_t c_v(line[2]);
            for (std::size_t i = 0; i < simd_end; i += batch_size) {
                const simd_t x_v = xsimd::load_unaligned(&pts_x[i]);
                const simd_t y_v = xsimd::load_unaligned(&pts_y[i]);
                xsimd::store_unaligned(&dists[i],
                    xsimd::abs(xsimd::fma(b_v, y_v, xsimd::fma(a_v, x_v, c_v))));
            }
        }
        for (std::size_t i = simd_end; i < n; ++i) {
            dists[i] = std::abs(line[0] * pts_x[i] + line[1] * pts_y[i] + line[2]);
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (dists[i] <= this->inlier_threshold_) {
                inliers.push_back(i);
            }
        }
#else
        for (std::size_t i = 0; i < points.size(); ++i) {
            const float dist = std::abs(line[0] * points[i].x() + line[1] * points[i].y() + line[2]);
            if (dist <= this->inlier_threshold_) {
                inliers.push_back(i);
            }
        }
#endif

        if (inliers.size() > best_inliers.size() && inliers.size() >= this->min_inliers_) {
            best_line = line;
            best_inliers = inliers;
        }
    }

    return std::make_pair(best_line, best_inliers);
}

// ---------------- Circle Fitter Implementation ----------------

rises_interfaces::msg::Obstacle CircleFitter::fitShape(
    int segment_id,
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) {
    (void)accumulated_angle;  // Unused parameter
    
    rises_interfaces::msg::Obstacle obs;
    obs.id = segment_id;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    if (points.size() < 3) {
        RCLCPP_WARN(rclcpp::get_logger("CircleFitter"),
                   "Not enough points for circle fitting: %zu", points.size());
        return obs;
    }
    
    std::pair<Eigen::Vector3f, std::vector<std::size_t>> circle_result = this->ransacCircleFitting(points);
    const Eigen::Vector3f& circle_params = circle_result.first;
    const std::vector<std::size_t>& inliers = circle_result.second;

    if (inliers.size() < this->min_inliers_) {
        RCLCPP_DEBUG(rclcpp::get_logger("CircleFitter"),
                    "Circle fitting failed, not enough inliers: %zu", inliers.size());
        return obs;
    }

    const float center_x = circle_params[0];
    const float center_y = circle_params[1];
    const float radius = circle_params[2];

    // Create circle approximation with vertices
    int num_vertices = std::max(8, static_cast<int>(2 * M_PI * radius / 0.2f));
    num_vertices = std::min(num_vertices, 20);

    for (int i = 0; i < num_vertices; ++i) {
        const float angle = 2.0f * M_PI * i / num_vertices;
        geometry_msgs::msg::Point pt;
        pt.x = center_x + radius * std::cos(angle);
        pt.y = center_y + radius * std::sin(angle);
        pt.z = 0.0f;
        obs.vertices.push_back(pt);
    }
    
    obs.position.x = center_x;
    obs.position.y = center_y;
    obs.position.z = 0.0f;
    
    return obs;
}

float CircleFitter::getConfidence(
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) const {
    
    if (points.size() < 5) return 0.0f;
    
    const float circle_hint_threshold = 3.0f * M_PI / 4.0f; // 135 degrees
    if (accumulated_angle > circle_hint_threshold) {
        return 0.9f; // High confidence for curved segments
    }
    
    return 0.2f; // Low confidence otherwise
}

std::pair<Eigen::Vector3f, std::vector<std::size_t>> CircleFitter::ransacCircleFitting(
    const std::vector<Eigen::Vector2f>& points) const {
    
    Eigen::Vector3f best_circle = Eigen::Vector3f::Zero();
    std::vector<std::size_t> best_inliers;
    
    if (points.size() < 3) {
        return std::make_pair(best_circle, best_inliers);
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<std::size_t> dis(0, points.size() - 1);

#ifdef USE_SIMD
    const std::size_t n = points.size();
    std::vector<float> pts_x(n), pts_y(n), dists(n);
    for (std::size_t i = 0; i < n; ++i) {
        pts_x[i] = points[i].x();
        pts_y[i] = points[i].y();
    }
    using simd_t = xsimd::batch<float>;
    constexpr std::size_t batch_size = simd_t::size;
    const std::size_t simd_end = n - (n % batch_size);
#endif

    for (int iter = 0; iter < this->max_iterations_; ++iter) {
        // Sample three different points
        std::vector<std::size_t> sample_indices;
        while (sample_indices.size() < 3) {
            const std::size_t idx = dis(gen);
            if (std::find(sample_indices.begin(), sample_indices.end(), idx) == sample_indices.end()) {
                sample_indices.push_back(idx);
            }
        }

        const Eigen::Vector2f& p1 = points[sample_indices[0]];
        const Eigen::Vector2f& p2 = points[sample_indices[1]];
        const Eigen::Vector2f& p3 = points[sample_indices[2]];

        // Solve for circle center and radius
        Eigen::Matrix3f A;
        Eigen::Vector3f b;

        A << 2 * (p2.x() - p1.x()), 2 * (p2.y() - p1.y()), 1,
             2 * (p3.x() - p1.x()), 2 * (p3.y() - p1.y()), 1,
             0, 0, 1;

        b << p2.x() * p2.x() - p1.x() * p1.x() + p2.y() * p2.y() - p1.y() * p1.y(),
             p3.x() * p3.x() - p1.x() * p1.x() + p3.y() * p3.y() - p1.y() * p1.y(),
             p1.x() * p1.x() + p1.y() * p1.y();

        const Eigen::Vector3f solution = A.colPivHouseholderQr().solve(b);

        const Eigen::Vector2f center(solution[0], solution[1]);
        const float radius = std::sqrt((center - p1).squaredNorm());

        if (radius < 0.05f || radius > 10.0f) continue;

        // Count inliers
        std::vector<std::size_t> inliers;
#ifdef USE_SIMD
        {
            const simd_t cx_v(center.x());
            const simd_t cy_v(center.y());
            const simd_t r_v(radius);
            for (std::size_t i = 0; i < simd_end; i += batch_size) {
                const simd_t dx = xsimd::load_unaligned(&pts_x[i]) - cx_v;
                const simd_t dy = xsimd::load_unaligned(&pts_y[i]) - cy_v;
                const simd_t dist_to_center = xsimd::sqrt(xsimd::fma(dx, dx, dy * dy));
                xsimd::store_unaligned(&dists[i], xsimd::abs(dist_to_center - r_v));
            }
        }
        for (std::size_t i = simd_end; i < n; ++i) {
            const float dx = pts_x[i] - center.x();
            const float dy = pts_y[i] - center.y();
            dists[i] = std::abs(std::sqrt(dx * dx + dy * dy) - radius);
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (dists[i] <= this->inlier_threshold_) {
                inliers.push_back(i);
            }
        }
#else
        for (std::size_t i = 0; i < points.size(); ++i) {
            const float dist = std::abs((points[i] - center).norm() - radius);
            if (dist <= this->inlier_threshold_) {
                inliers.push_back(i);
            }
        }
#endif

        if (inliers.size() > best_inliers.size() && inliers.size() >= this->min_inliers_) {
            best_circle = Eigen::Vector3f(center.x(), center.y(), radius);
            best_inliers = inliers;
        }
    }

    return std::make_pair(best_circle, best_inliers);
}

// ---------------- Polygon Fitter Implementation ----------------

rises_interfaces::msg::Obstacle PolygonFitter::fitShape(
    int segment_id,
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) {
    
    (void)accumulated_angle; // Unused
    
    rises_interfaces::msg::Obstacle obs;
    obs.id = segment_id;
    obs.type = rises_interfaces::msg::Obstacle::POLYGON;
    
    if (points.empty()) return obs;
    
    // Simple polygon: use all points as vertices
    for (const Eigen::Vector2f& p : points) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0.0f;
        obs.vertices.push_back(pt);
    }

    // Set position to centroid
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const Eigen::Vector2f& p : points) {
        centroid += p;
    }
    centroid /= static_cast<float>(points.size());
    
    obs.position.x = centroid.x();
    obs.position.y = centroid.y();
    obs.position.z = 0.0f;
    
    return obs;
}

float PolygonFitter::getConfidence(
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) const {
    
    (void)accumulated_angle; // Unused
    
    if (points.empty()) return 0.0f;
    
    return 0.3f; // Medium confidence - always works as fallback
}

// ---------------- Shape Fitter Factory Implementation ----------------

std::unique_ptr<ShapeFitter> ShapeFitterFactory::getBestFitter(
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle,
    bool points_only_mode) {
    
    if (points_only_mode) {
        return std::make_unique<PointFitter>(true);
    }
    
    if (points.size() <= 1) {
        return std::make_unique<PointFitter>();
    }
    
    // Get all fitters and their confidences
    std::vector<std::unique_ptr<ShapeFitter>> all_fitters = getAllFitters(points, accumulated_angle);
    
    if (!all_fitters.empty()) {
        return std::move(all_fitters[0]); // Return highest confidence
    }
    
    // Fallback
    return std::make_unique<PolygonFitter>();
}

std::vector<std::unique_ptr<ShapeFitter>> ShapeFitterFactory::getAllFitters(
    const std::vector<Eigen::Vector2f>& points,
    float accumulated_angle) {
    
    std::vector<std::pair<float, std::unique_ptr<ShapeFitter>>> candidates;
    
    // Create all fitter types and evaluate confidence
    std::unique_ptr<PointFitter> point_fitter = std::make_unique<PointFitter>();
    const float point_confidence = point_fitter->getConfidence(points, accumulated_angle);
    candidates.emplace_back(point_confidence, std::move(point_fitter));

    std::unique_ptr<LineFitter> line_fitter = std::make_unique<LineFitter>();
    const float line_confidence = line_fitter->getConfidence(points, accumulated_angle);
    candidates.emplace_back(line_confidence, std::move(line_fitter));

    std::unique_ptr<CircleFitter> circle_fitter = std::make_unique<CircleFitter>();
    const float circle_confidence = circle_fitter->getConfidence(points, accumulated_angle);
    candidates.emplace_back(circle_confidence, std::move(circle_fitter));

    std::unique_ptr<PolygonFitter> polygon_fitter = std::make_unique<PolygonFitter>();
    const float polygon_confidence = polygon_fitter->getConfidence(points, accumulated_angle);
    candidates.emplace_back(polygon_confidence, std::move(polygon_fitter));

    // Sort by confidence (highest first)
    std::sort(candidates.begin(), candidates.end(),
              [](const std::pair<float, std::unique_ptr<ShapeFitter>>& a,
                 const std::pair<float, std::unique_ptr<ShapeFitter>>& b) {
                  return a.first > b.first;
              });

    // Extract sorted fitters
    std::vector<std::unique_ptr<ShapeFitter>> result;
    for (std::pair<float, std::unique_ptr<ShapeFitter>>& candidate : candidates) {
        result.push_back(std::move(candidate.second));
    }
    
    return result;
}

} // namespace rises::shapes