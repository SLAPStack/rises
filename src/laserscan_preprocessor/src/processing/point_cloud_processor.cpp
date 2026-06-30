#include "laserscan_preprocessor/processing/point_cloud_processor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#ifdef USE_SIMD
#include <xsimd/xsimd.hpp>
#endif

namespace rises::processing {

// Compile-time optimization for point count using Eigen fixed-size types
// When SCAN_POINT_COUNT is known at build time, use stack-allocated fixed-size arrays
// for perfect autovectorization and zero heap allocation overhead
#ifdef SCAN_POINT_COUNT
namespace {
    constexpr std::size_t EXPECTED_POINT_COUNT = SCAN_POINT_COUNT;
    constexpr bool HAS_KNOWN_POINT_COUNT = true;
    
    // Only use fixed-size for reasonable stack sizes (< 2048 floats ≈ 8KB per array)
    constexpr bool USE_FIXED_SIZE = (EXPECTED_POINT_COUNT > 0 && EXPECTED_POINT_COUNT < 2048);
    
    // Fixed-size array types for compile-time optimization
    using FixedFloatArray = Eigen::Array<float, EXPECTED_POINT_COUNT, 1>;
    
    // Helper to create optimal array type based on compile-time knowledge
    template<bool UseFixed>
    struct ArrayTypeSelector {
        using type = std::vector<float>;  // Fallback for large sizes
    };
    
    template<>
    struct ArrayTypeSelector<true> {
        using type = FixedFloatArray;  // Stack-allocated for small sizes
    };
    
    using OptimalFloatArray = typename ArrayTypeSelector<USE_FIXED_SIZE>::type;
}
#else
namespace {
    constexpr bool HAS_KNOWN_POINT_COUNT = false;
    constexpr bool USE_FIXED_SIZE = false;
    
    // Runtime fallback: always use dynamic allocation
    using OptimalFloatArray = std::vector<float>;
}
#endif

#ifdef USE_SIMD
// SIMD-optimized polar to Cartesian conversion
// Processes batches of points using SIMD instructions
namespace {
    using simd_batch = xsimd::batch<float>;
    constexpr std::size_t simd_size = simd_batch::size;
    
    // Batch polar to Cartesian conversion: (range, angle) -> (x, y)
    inline void polarToCartesianSIMD(
        const float* ranges,
        const float* angles,
        float* x_out,
        float* y_out,
        const std::size_t count) {
        
        const std::size_t simd_end = count - (count % simd_size);
        
        // Process full SIMD batches
        for (std::size_t i = 0; i < simd_end; i += simd_size) {
            const simd_batch range_batch = xsimd::load_unaligned(&ranges[i]);
            const simd_batch angle_batch = xsimd::load_unaligned(&angles[i]);
            
            const simd_batch x_batch = range_batch * xsimd::cos(angle_batch);
            const simd_batch y_batch = range_batch * xsimd::sin(angle_batch);
            
            xsimd::store_unaligned(&x_out[i], x_batch);
            xsimd::store_unaligned(&y_out[i], y_batch);
        }
        
        // Process remainder with scalar code
        for (std::size_t i = simd_end; i < count; ++i) {
            x_out[i] = ranges[i] * std::cos(angles[i]);
            y_out[i] = ranges[i] * std::sin(angles[i]);
        }
    }
    
    // Batch 3D transformation: apply rotation + translation
    inline void transform3DSIMD(
        const float* x_in,
        const float* y_in, 
        const float* z_in,
        float* x_out,
        float* y_out,
        float* z_out,
        const float r00, const float r01, const float r02, const float tx,
        const float r10, const float r11, const float r12, const float ty,
        const float r20, const float r21, const float r22, const float tz,
        const std::size_t count) {
        
        const std::size_t simd_end = count - (count % simd_size);
        
        // Broadcast matrix elements
        const simd_batch r00_b(r00), r01_b(r01), r02_b(r02), tx_b(tx);
        const simd_batch r10_b(r10), r11_b(r11), r12_b(r12), ty_b(ty);
        const simd_batch r20_b(r20), r21_b(r21), r22_b(r22), tz_b(tz);
        
        // Process full SIMD batches
        for (std::size_t i = 0; i < simd_end; i += simd_size) {
            const simd_batch x = xsimd::load_unaligned(&x_in[i]);
            const simd_batch y = xsimd::load_unaligned(&y_in[i]);
            const simd_batch z = xsimd::load_unaligned(&z_in[i]);
            
            const simd_batch x_new = xsimd::fma(r00_b, x, xsimd::fma(r01_b, y, xsimd::fma(r02_b, z, tx_b)));
            const simd_batch y_new = xsimd::fma(r10_b, x, xsimd::fma(r11_b, y, xsimd::fma(r12_b, z, ty_b)));
            const simd_batch z_new = xsimd::fma(r20_b, x, xsimd::fma(r21_b, y, xsimd::fma(r22_b, z, tz_b)));
            
            xsimd::store_unaligned(&x_out[i], x_new);
            xsimd::store_unaligned(&y_out[i], y_new);
            xsimd::store_unaligned(&z_out[i], z_new);
        }
        
        // Process remainder with scalar code
        for (std::size_t i = simd_end; i < count; ++i) {
            const float xi = x_in[i];
            const float yi = y_in[i];
            const float zi = z_in[i];
            x_out[i] = r00 * xi + r01 * yi + r02 * zi + tx;
            y_out[i] = r10 * xi + r11 * yi + r12 * zi + ty;
            z_out[i] = r20 * xi + r21 * yi + r22 * zi + tz;
        }
    }
}
#endif

sensor_msgs::msg::PointCloud2 PointCloudProcessor::convertToPointCloud2(
    const std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr>& scans,
    const std::vector<LaserConfig>& laser_configs,
    const std::string& global_frame) const {
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    
    // Calculate total number of valid points
    std::size_t total_points = 0;
    for (const auto& scan : scans) {
        if (scan) {
            for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
                float range = scan->ranges[i];
                if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
                    total_points++;
                }
            }
        }
    }
    
    if (total_points == 0) {
        RCLCPP_WARN(rclcpp::get_logger("PointCloudProcessor"), "[convertToPointCloud2] No valid points found in scans");
        return cloud_msg;
    }
    
    // Set up PointCloud2 message - use first scan's frame and timestamp
    // Find first valid scan for header info
    const sensor_msgs::msg::LaserScan::ConstSharedPtr* first_scan = nullptr;
    for (const auto& scan : scans) {
        if (scan) {
            first_scan = &scan;
            break;
        }
    }
    
    if (!first_scan) {
        RCLCPP_WARN(rclcpp::get_logger("PointCloudProcessor"), "[convertToPointCloud2] No valid scans found");
        return cloud_msg;
    }
    
    cloud_msg.header.frame_id = (*first_scan)->header.frame_id;  // Keep in laser frame
    cloud_msg.header.stamp = (*first_scan)->header.stamp;        // Use scan timestamp
    cloud_msg.height = 1;
    cloud_msg.width = total_points;
    
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(total_points);
    
    // Iterators for filling point data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    
#ifdef USE_SIMD
    // SIMD-optimized conversion path
    for (std::size_t scan_idx = 0; scan_idx < scans.size() && scan_idx < laser_configs.size(); ++scan_idx) {
        const auto& scan = scans[scan_idx];
        const auto& config = laser_configs[scan_idx];
        
        if (!scan) continue;
        
        // Prepare batches for SIMD processing using optimal array type
#if defined(SCAN_POINT_COUNT) && USE_FIXED_SIZE
        // Compile-time known size: use stack-allocated Eigen arrays for autovectorization
        OptimalFloatArray valid_ranges;
        OptimalFloatArray valid_angles;
        OptimalFloatArray x_coords;
        OptimalFloatArray y_coords;
        
        // Fill arrays using index-based access (works for fixed-size)
        std::size_t valid_count = 0;
        float angle = scan->angle_min;
        for (std::size_t i = 0; i < scan->ranges.size() && valid_count < EXPECTED_POINT_COUNT; 
             ++i, angle += scan->angle_increment) {
            const float range = scan->ranges[i];
            if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
                valid_ranges[valid_count] = range;
                valid_angles[valid_count] = angle;
                ++valid_count;
            }
        }
        
        if (valid_count == 0) continue;
        
        // Batch polar-to-Cartesian conversion using SIMD
        polarToCartesianSIMD(
            valid_ranges.data(),
            valid_angles.data(),
            x_coords.data(),
            y_coords.data(),
            valid_count
        );
        
        // Copy to point cloud
        for (std::size_t i = 0; i < valid_count; ++i) {
            *iter_x = x_coords[i];
            *iter_y = y_coords[i];
            *iter_z = config.height;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
#else
        // Runtime size or large point count: use dynamic allocation
        std::vector<float> valid_ranges;
        std::vector<float> valid_angles;
        valid_ranges.reserve(scan->ranges.size());
        valid_angles.reserve(scan->ranges.size());
        
        float angle = scan->angle_min;
        for (std::size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
            const float range = scan->ranges[i];
            if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
                valid_ranges.push_back(range);
                valid_angles.push_back(angle);
            }
        }
        
        if (valid_ranges.empty()) continue;
        
        // Allocate output buffers for SIMD results
        std::vector<float> x_coords(valid_ranges.size());
        std::vector<float> y_coords(valid_ranges.size());
        
        // Batch polar-to-Cartesian conversion using SIMD
        polarToCartesianSIMD(
            valid_ranges.data(),
            valid_angles.data(),
            x_coords.data(),
            y_coords.data(),
            valid_ranges.size()
        );
        
        // Copy to point cloud
        for (std::size_t i = 0; i < valid_ranges.size(); ++i) {
            *iter_x = x_coords[i];
            *iter_y = y_coords[i];
            *iter_z = config.height;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
#endif
    }
#else
    // Scalar fallback path
    for (std::size_t scan_idx = 0; scan_idx < scans.size() && scan_idx < laser_configs.size(); ++scan_idx) {
        const auto& scan = scans[scan_idx];
        const auto& config = laser_configs[scan_idx];
        
        if (!scan) continue;
        
        float angle = scan->angle_min;
        for (std::size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
            const float range = scan->ranges[i];
            
            if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
                continue;
            }
            
            // Convert to Cartesian coordinates in laser frame
            *iter_x = range * std::cos(angle);
            *iter_y = range * std::sin(angle);
            *iter_z = config.height;
            
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }
#endif
    
    return cloud_msg;
}

Eigen::Matrix<float, 3, Eigen::Dynamic> PointCloudProcessor::convertToEigenCloud(
    const std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr>& scans,
    const std::vector<LaserConfig>& laser_configs) const {
    
    // First pass: count valid points
    std::size_t total_points = 0;
    for (const auto& scan : scans) {
        if (scan) {
            for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
                float range = scan->ranges[i];
                if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
                    total_points++;
                }
            }
        }
    }
    
    if (total_points == 0) {
        return Eigen::Matrix<float, 3, Eigen::Dynamic>(3, 0);
    }
    
    // Create Eigen matrix
    Eigen::Matrix<float, 3, Eigen::Dynamic> points(3, total_points);
    
    std::size_t point_idx = 0;
    
    // Convert each scan to points
    for (std::size_t scan_idx = 0; scan_idx < scans.size() && scan_idx < laser_configs.size(); ++scan_idx) {
        const auto& scan = scans[scan_idx];
        const auto& config = laser_configs[scan_idx];
        
        if (!scan) continue;
        
#ifdef USE_SIMD
        // Prepare batches for SIMD processing using optimal array type
#if defined(SCAN_POINT_COUNT) && USE_FIXED_SIZE
        // Compile-time known size: use stack-allocated Eigen arrays
        OptimalFloatArray valid_ranges;
        OptimalFloatArray valid_angles;
        OptimalFloatArray x_coords;
        OptimalFloatArray y_coords;
        
        std::size_t valid_count = 0;
        float angle = scan->angle_min;
        for (std::size_t i = 0; i < scan->ranges.size() && valid_count < EXPECTED_POINT_COUNT; 
             ++i, angle += scan->angle_increment) {
            const float range = scan->ranges[i];
            if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
                valid_ranges[valid_count] = range;
                valid_angles[valid_count] = angle;
                ++valid_count;
            }
        }
        
        if (valid_count == 0) continue;
        
        // Batch SIMD conversion
        polarToCartesianSIMD(
            valid_ranges.data(),
            valid_angles.data(),
            x_coords.data(),
            y_coords.data(),
            valid_count
        );
        
        // Copy to Eigen matrix
        for (std::size_t i = 0; i < valid_count; ++i) {
            points(0, point_idx) = x_coords[i];
            points(1, point_idx) = y_coords[i];
            points(2, point_idx) = config.height;
            point_idx++;
        }
#else
        // Runtime size or large point count: use dynamic allocation
        std::vector<float> valid_ranges;
        std::vector<float> valid_angles;
        valid_ranges.reserve(scan->ranges.size());
        valid_angles.reserve(scan->ranges.size());
        
        float angle = scan->angle_min;
        for (std::size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
            const float range = scan->ranges[i];
            if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
                valid_ranges.push_back(range);
                valid_angles.push_back(angle);
            }
        }
        
        if (valid_ranges.empty()) continue;
        
        std::vector<float> x_coords(valid_ranges.size());
        std::vector<float> y_coords(valid_ranges.size());
        
        // Batch SIMD conversion
        polarToCartesianSIMD(
            valid_ranges.data(),
            valid_angles.data(),
            x_coords.data(),
            y_coords.data(),
            valid_ranges.size()
        );
        
        // Copy to Eigen matrix
        for (std::size_t i = 0; i < valid_ranges.size(); ++i) {
            points(0, point_idx) = x_coords[i];
            points(1, point_idx) = y_coords[i];
            points(2, point_idx) = config.height;
            point_idx++;
        }
#endif
#else
        float angle = scan->angle_min;
        for (std::size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
            const float range = scan->ranges[i];
            
            if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
                continue;
            }
            
            // Convert to Cartesian coordinates
            const float x = range * std::cos(angle);
            const float y = range * std::sin(angle);
            const float z = config.height;
            
            points(0, point_idx) = x;
            points(1, point_idx) = y;
            points(2, point_idx) = z;
            
            point_idx++;
        }
#endif
    }
    
    return points;
}

std::vector<Eigen::Vector2f> PointCloudProcessor::extractPoints2D(
    const sensor_msgs::msg::PointCloud2& cloud) {
    
    std::vector<Eigen::Vector2f> points;
    
    if (cloud.width == 0 || cloud.height == 0) {
        return points;
    }
    
    points.reserve(cloud.width * cloud.height);
    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    
    for (std::size_t i = 0; i < cloud.width * cloud.height; ++i, ++iter_x, ++iter_y) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y)) {
            points.emplace_back(*iter_x, *iter_y);
        }
    }
    
    return points;
}

sensor_msgs::msg::PointCloud2 PointCloudProcessor::segmentPointCloud(
    const sensor_msgs::msg::PointCloud2& cloud_in,
    float base_distance_threshold,
    float angle_threshold_rad) {
    
    const std::size_t n_points = cloud_in.width * cloud_in.height;
    
    // Build output cloud with explicit xyz + segment_id fields.
    // NOTE: setPointCloud2FieldsByString does NOT recognise custom field names
    // like "segment_id", so we use setPointCloud2Fields with explicit descriptors.
    sensor_msgs::msg::PointCloud2 cloud_out;
    cloud_out.header   = cloud_in.header;
    cloud_out.height   = cloud_in.height;
    cloud_out.width    = cloud_in.width;
    cloud_out.is_dense = cloud_in.is_dense;
    
    sensor_msgs::PointCloud2Modifier modifier(cloud_out);
    modifier.setPointCloud2Fields(4,
        "x",          1, sensor_msgs::msg::PointField::FLOAT32,
        "y",          1, sensor_msgs::msg::PointField::FLOAT32,
        "z",          1, sensor_msgs::msg::PointField::FLOAT32,
        "segment_id", 1, sensor_msgs::msg::PointField::INT32);
    modifier.resize(n_points);
    
    // Copy xyz from input and initialise every point to segment 0
    sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(cloud_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(cloud_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(cloud_in, "z");
    
    sensor_msgs::PointCloud2Iterator<float>   iter_x_out(cloud_out, "x");
    sensor_msgs::PointCloud2Iterator<float>   iter_y_out(cloud_out, "y");
    sensor_msgs::PointCloud2Iterator<float>   iter_z_out(cloud_out, "z");
    sensor_msgs::PointCloud2Iterator<int32_t> iter_seg_out(cloud_out, "segment_id");
    
    for (std::size_t i = 0; i < n_points; ++i,
         ++iter_x_in, ++iter_y_in, ++iter_z_in,
         ++iter_x_out, ++iter_y_out, ++iter_z_out, ++iter_seg_out) {
        *iter_x_out   = *iter_x_in;
        *iter_y_out   = *iter_y_in;
        *iter_z_out   = *iter_z_in;
        *iter_seg_out = 0;  // single segment for now
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("PointCloudProcessor"),
                "Basic segmentation applied (distance: %.3f, angle: %.3f rad)",
                base_distance_threshold, angle_threshold_rad);
    
    return cloud_out;
}

bool PointCloudProcessor::transformToFrame(
    sensor_msgs::msg::PointCloud2& cloud,
    const std::string& target_frame) const {
    
    if (cloud.header.frame_id == target_frame) {
        return true; // Already in target frame
    }
    
    try {
        // Wait up to 200ms for transform at exact scan timestamp
        // Accounts for TF arriving slightly after scan during rosbag playback
        auto transform = tf_buffer_->lookupTransform(
            target_frame, 
            cloud.header.frame_id, 
            cloud.header.stamp,
            rclcpp::Duration::from_seconds(0.2)  // 200ms timeout
        );
        tf2::doTransform(cloud, cloud, transform);
        cloud.header.frame_id = target_frame;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("PointCloudProcessor"),
                    "Failed to transform cloud from %s to %s: %s",
                    cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

} // namespace rises::processing
