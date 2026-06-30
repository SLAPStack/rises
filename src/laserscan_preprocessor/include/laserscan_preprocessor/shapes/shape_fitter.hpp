#pragma once

#include <memory>
#include <Eigen/Dense>
#include "rises_interfaces/msg/obstacle.hpp"

namespace rises::shapes {

/**
 * @brief Abstract base class for shape fitting strategies
 */
class ShapeFitter {
public:
    virtual ~ShapeFitter() = default;
    
    /**
     * @brief Fit points to a specific shape
     * @param segment_id Unique identifier for the segment
     * @param points Input 2D points
     * @param accumulated_angle Total angular change (hint for shape type)
     * @return Obstacle message with fitted shape
     */
    virtual rises_interfaces::msg::Obstacle fitShape(
        int segment_id, 
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) = 0;
        
    /**
     * @brief Check if this fitter can handle the given points
     * @param points Input points
     * @param accumulated_angle Angular hint
     * @return Confidence score [0.0, 1.0]
     */
    virtual float getConfidence(
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) const = 0;
        
    virtual std::string getName() const = 0;
};

/**
 * @brief Point shape fitter (single points or points-only mode)
 */
class PointFitter : public ShapeFitter {
public:
    explicit PointFitter(bool points_only_mode = false) 
        : points_only_mode_(points_only_mode) {}
        
    rises_interfaces::msg::Obstacle fitShape(
        int segment_id, 
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) override;
        
    float getConfidence(
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) const override;
        
    std::string getName() const override { return "Point"; }

private:
    bool points_only_mode_;
};

/**
 * @brief Line shape fitter using RANSAC and PCA
 */
class LineFitter : public ShapeFitter {
public:
    LineFitter(int max_iterations = 100, float inlier_threshold = 0.05f, std::size_t min_inliers = 2)
        : max_iterations_(max_iterations), inlier_threshold_(inlier_threshold), min_inliers_(min_inliers) {}
        
    rises_interfaces::msg::Obstacle fitShape(
        int segment_id, 
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) override;
        
    float getConfidence(
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) const override;
        
    std::string getName() const override { return "Line"; }

private:
    int max_iterations_;
    float inlier_threshold_;
    std::size_t min_inliers_;
    
    std::pair<Eigen::Vector3f, std::vector<std::size_t>> ransacLineFitting(
        const std::vector<Eigen::Vector2f>& points) const;
};

/**
 * @brief Circle shape fitter using RANSAC
 */
class CircleFitter : public ShapeFitter {
public:
    CircleFitter(int max_iterations = 50, float inlier_threshold = 0.1f, std::size_t min_inliers = 5)
        : max_iterations_(max_iterations), inlier_threshold_(inlier_threshold), min_inliers_(min_inliers) {}
        
    rises_interfaces::msg::Obstacle fitShape(
        int segment_id, 
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) override;
        
    float getConfidence(
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) const override;
        
    std::string getName() const override { return "Circle"; }

private:
    int max_iterations_;
    float inlier_threshold_;
    std::size_t min_inliers_;
    
    std::pair<Eigen::Vector3f, std::vector<std::size_t>> ransacCircleFitting(
        const std::vector<Eigen::Vector2f>& points) const;
};

/**
 * @brief Polygon shape fitter (fallback for complex shapes)
 */
class PolygonFitter : public ShapeFitter {
public:
    PolygonFitter() = default;
    
    rises_interfaces::msg::Obstacle fitShape(
        int segment_id, 
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) override;
        
    float getConfidence(
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f) const override;
        
    std::string getName() const override { return "Polygon"; }
};

/**
 * @brief Factory class for creating and managing shape fitters
 */
class ShapeFitterFactory {
public:
    /**
     * @brief Get the best shape fitter for given points
     * @param points Input points
     * @param accumulated_angle Angular hint
     * @param points_only_mode Force point-only output
     * @return Best fitting strategy
     */
    static std::unique_ptr<ShapeFitter> getBestFitter(
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f,
        bool points_only_mode = false);
        
    /**
     * @brief Get all available fitters ordered by confidence
     * @param points Input points
     * @param accumulated_angle Angular hint
     * @return Vector of fitters ordered by confidence (highest first)
     */
    static std::vector<std::unique_ptr<ShapeFitter>> getAllFitters(
        const std::vector<Eigen::Vector2f>& points,
        float accumulated_angle = 0.0f);
};

} // namespace rises::shapes