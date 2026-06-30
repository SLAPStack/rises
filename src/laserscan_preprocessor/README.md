# Laser Scan Preprocessor

A ROS 2 node for preprocessing laser scan data from multiple LIDAR sensors, performing segmentation, and publishing obstacle information.

## Features

### Core Functionality
- Multi-laser synchronization and fusion
- Point cloud conversion and transformation
- Advanced segmentation algorithms (DBSCAN, region growing)
- Shape fitting (lines, circles, polygons)
- Configurable points-only publishing mode
- Real-time obstacle detection and publishing

### Segmentation Algorithms
- Distance-based segmentation
- Angle-based segmentation  
- DBSCAN clustering
- Region growing
- Outlier removal
- Adaptive thresholding

### Performance Optimizations
- KD-tree spatial indexing for O(log n) neighbor queries
- Optional SIMD acceleration (when Highway library available)
- Parallel processing for shape fitting
- Efficient memory management

## Parameters

### Basic Parameters
- `segment_distance_threshold` (float, default: 0.2): Distance threshold for segmentation
- `segment_angle_threshold_deg` (float, default: 30.0): Angle threshold in degrees
- `target_frame` (string, default: "map"): Target coordinate frame
- `tf_prefix` (string, default: ""): TF prefix for all frames
- `laser_frames` (string array): List of laser frame IDs
- `laser_heights` (double array): Heights of corresponding lasers

### New Feature: Points-Only Mode
- `publish_points_only` (bool, default: false): When enabled, publishes segments as individual points instead of fitted shapes, forcing downstream nodes to check every point separately

### Advanced Segmentation Parameters
- `dbscan_eps` (float, default: 0.15): DBSCAN neighborhood radius
- `dbscan_min_points` (int, default: 3): Minimum points for DBSCAN core
- `region_grow_threshold` (float, default: 0.1): Region growing distance threshold
- `min_segment_size` (int, default: 3): Minimum points per segment
- `outlier_removal_factor` (float, default: 1.5): Outlier detection threshold
- `use_adaptive_thresholding` (bool, default: true): Enable adaptive thresholds

## Topics

### Subscribed Topics
- `scan/<laser_frame>` (sensor_msgs/LaserScan): Input laser scans for each configured laser

### Published Topics
- `world_scan` (sensor_msgs/PointCloud2): Unified point cloud in target frame
- `processed_scan` (sensor_msgs/PointCloud2): Segmented point cloud with segment IDs
- `lidar_segments` (rises_interfaces/ObstacleArray): Detected obstacles/segments

## Build and Installation

```bash
# Install dependencies
sudo apt-get install libnanoflann-dev

# Build
colcon build --packages-select laserscan_preprocessor

# Source
source install/setup.bash
```

## Usage

### Basic Usage
```bash
# Launch with default parameters
ros2 launch laserscan_preprocessor laser_preprocessor.launch.py

# Enable points-only mode
ros2 launch laserscan_preprocessor laser_preprocessor.launch.py publish_points_only:=true
```

### Configuration Example
```yaml
laser_preprocessor:
  ros__parameters:
    # Basic segmentation
    segment_distance_threshold: 0.3
    segment_angle_threshold_deg: 45.0
    target_frame: "base_link"
    
    # Points-only mode for downstream point-by-point processing
    publish_points_only: true
    
    # Laser configuration
    laser_frames: ["front_laser", "rear_laser"]
    laser_heights: [0.2, 0.2]
    
    # Advanced segmentation
    dbscan_eps: 0.2
    dbscan_min_points: 5
    use_adaptive_thresholding: true
```

## Testing

### Run All Tests
```bash
# Unit tests
colcon test --packages-select laserscan_preprocessor

# View test results
colcon test-result --verbose
```

### Test Categories

#### 1. Unit Tests (`test_laserscan_preprocessor_node.cpp`)
- Parameter validation and clamping
- Lifecycle state transitions
- Transform handling
- Points-only mode functionality
- Error handling

#### 2. Algorithm Tests (`test_algorithms.cpp`)
- DBSCAN clustering
- RANSAC line/circle fitting
- Region growing segmentation
- Outlier removal
- Spatial indexing

#### 3. Integration Tests (`test_integration.cpp`)
- Multi-laser synchronization
- Full processing pipeline
- Transform failure handling
- High-frequency data processing
- Edge case handling

#### 4. Laser Manager Tests (`test_laser_manager.cpp`)
- Laser synchronization
- Time tolerance handling
- Timeout behavior
- Multi-laser coordination

### Manual Testing
```bash
# Start test environment
ros2 launch laserscan_preprocessor test_laser_preprocessor.launch.py

# Publish test data
ros2 run laserscan_preprocessor test_scan_publisher.py

# Test points-only mode
ros2 launch laserscan_preprocessor test_laser_preprocessor.launch.py test_points_only_mode:=true
```

## Points-Only Mode Details

When `publish_points_only` is enabled:

1. **Behavior**: Segments are published as `POINT` type obstacles with all individual points as vertices
2. **Use Case**: Forces downstream nodes to process every point individually rather than using geometric approximations
3. **Data Structure**: Each obstacle contains multiple vertices representing individual laser points
4. **Downstream Impact**: Nodes must iterate through all vertices instead of using fitted shape parameters

### Example Usage of Points-Only Mode
```cpp
// Downstream node processing obstacles
for (const auto& obstacle : obstacle_array.obstacles) {
    if (obstacle.type == rises_interfaces::msg::Obstacle::POINT) {
        // Points-only mode: check every vertex individually
        for (const auto& vertex : obstacle.vertices) {
            // Process each individual laser point
            processPoint(vertex.x, vertex.y, vertex.z);
        }
    } else {
        // Normal mode: use fitted shape
        processShape(obstacle);
    }
}
```

## Performance Notes

### Optimization Features
- **KD-tree Indexing**: O(log n) neighbor queries instead of O(n²) brute force
- **SIMD Support**: Enable with `-DUSE_SIMD=ON` when Highway library is available
- **Parallel Processing**: Automatic parallelization for large segment sets
- **Adaptive Algorithms**: Performance scales with data complexity

### Memory Management
- Efficient point cloud iterators
- Minimal copying with move semantics
- RAII for resource management
- Aligned memory allocation for SIMD operations

## Troubleshooting

### Common Issues

1. **Transform Errors**: Ensure all laser frames have transforms to target frame
2. **Sync Issues**: Check timestamp alignment between multiple lasers
3. **Performance**: Enable KD-tree indexing for large point clouds
4. **Parameter Validation**: Parameters are automatically clamped to valid ranges

### Debug Output
```bash
# Enable debug logging
ros2 run laserscan_preprocessor laserscan_preprocessor_node --ros-args --log-level debug
```

## Dependencies

- ROS 2 (Humble/Iron/Rolling)
- Eigen3
- nanoflann (for spatial indexing)
- rises_interfaces (custom message types)
- tf2_ros, tf2_geometry_msgs, tf2_sensor_msgs
- Optional: Highway library (for SIMD optimization)

## License

Apache 2.0