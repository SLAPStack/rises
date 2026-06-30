# Message Translator

A ROS 2 node that translates JSON AABB messages into ROS messages compatible with the Geofence node.

## Features

### Core Functionality
- **JSON to ROS Translation**: Converts JSON obstacle updates to ROS obstacle messages
- **Path Processing**: Transforms order JSON into navigation paths
- **Warehouse Contour Processing**: Handles warehouse boundary contours
- **TF Frame Management**: Publishes robot pose transforms

## Dependencies

### System Dependencies
```bash
sudo apt-get install pkg-config nlohmann-json3-dev
```

### ROS 2 Dependencies
- `rclcpp`
- `rclcpp_components`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`
- `tf2_msgs`
- `rises_interfaces` (custom message package)

## Configuration

### Parameters

The node supports configuration through ROS parameters. See `config/params.yaml` for a complete example.

#### Core Parameters
- `map_frame`: Base coordinate frame (default: "map")
- `target_frame`: Target frame for TF transforms (default: "base_link")
- `tf_prefix`: Prefix for TF frame names (default: "")
- `prefix_global_frame`: Whether to apply the prefix to the global frame (default: false)

## Usage

### Building
```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select message_translator
```

### Running with Default Configuration
```bash
ros2 launch message_translator geofence_translator.launch.py
```

### Running with Custom Configuration
```bash
ros2 launch message_translator geofence_translator.launch.py \
    config_file:=/path/to/your/config.yaml \
    log_level:=DEBUG
```

### Running as Component
```bash
ros2 component standalone message_translator slapstack::MessageTranslatorNode
```

## Topics

### Subscribed Topics
- `obstacle_json` (std_msgs/String): JSON obstacle updates
- `order` (std_msgs/String): Order JSON for path generation
- `warehouse_contours_mqtt` (std_msgs/String): Warehouse contour data
- `validation_mqtt` (std_msgs/String): Validation obstacle data
- `/tf_raw` (tf2_msgs/TFMessage): Raw TF data for filtering

### Published Topics
- `map_updates` (rises_interfaces/ObstacleUpdateArray): Processed obstacle updates
- `incoming_path` (nav_msgs/Path): Generated navigation paths
- `warehouse_contours` (rises_interfaces/Contours): Processed warehouse contours
- `validation` (rises_interfaces/ObstacleArray): Validation obstacles
- `base_link_pose` (geometry_msgs/PoseStamped): Robot pose
- `/tf` (tf2_msgs/TFMessage): Filtered TF data

## Error Handling

The node implements comprehensive error handling:
- **JSON Parsing**: Safe parsing with detailed error messages
- **Field Validation**: Checks for required JSON fields
- **Type Safety**: Safe type conversions with fallback values
- **Logging**: Detailed logging at multiple levels

## Architecture

### Class Structure
```cpp
class slapstack::MessageTranslatorNode : public rclcpp::Node
├── Core Functionality
│   ├── JSON message processing
│   ├── ROS message publishing
│   └── TF transform management
└── Error Handling
    ├── Safe JSON parsing
    ├── Field validation
    └── Exception management
```

### Data Flow
```
JSON Input → Parsing → Validation → ROS Messages → Publishing
     ↓           ↓          ↓           ↓             ↓
Error Handling → Logging → Fallback → Publishing → Downstream nodes
```

## Troubleshooting

### Common Issues
1. **JSON Parse Errors**: Validate JSON format against expected schema
2. **Missing Dependencies**: Ensure all system packages are installed
3. **TF Transform Errors**: Verify frame names and TF tree structure

### Debug Mode
Enable debug logging for detailed information:
```bash
ros2 launch message_translator geofence_translator.launch.py log_level:=DEBUG
```

## License

Apache-2.0

## Contributing

When contributing to this project:
1. Follow the existing code style and organization
2. Add appropriate error handling for new functionality
3. Update documentation and configuration files
4. Test the JSON translation functionality
5. Ensure backward compatibility when possible
