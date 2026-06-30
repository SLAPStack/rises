cmake_minimum_required(VERSION 3.22)
project(message_translator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
  include
)

# Component library
add_library(${PROJECT_NAME} SHARED
  src/message_translator_node.cpp
  src/json_message_translator.cpp
  src/tf_replay_manager.cpp
  src/delivery_strategy.cpp
  src/aabb_converter.cpp
  src/area_locks_converter.cpp
  src/contours_converter.cpp
  src/vda5050_converter.cpp
  src/vda5050_state_bridge.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  tf2_ros
  tf2_msgs
  rises_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# Enable LTO for release builds
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
  set_target_properties(${PROJECT_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Register as a composable node
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "slapstack::MessageTranslatorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install config and launch files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  
  # Core functionality tests
  ament_add_gtest(test_geofence_translator
    test/test_geofence_translator.cpp
  )
  target_include_directories(test_geofence_translator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_geofence_translator
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_geofence_translator
    rclcpp
    std_msgs
    rises_interfaces
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_msgs
  )

  # Error handling tests
  ament_add_gtest(test_error_handling
    test/test_error_handling.cpp
  )
  target_include_directories(test_error_handling PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_error_handling
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_error_handling
    rclcpp
    std_msgs
    rises_interfaces
    tf2_ros
  )

  # Contour processing tests
  ament_add_gtest(test_contour_processing
    test/test_contour_processing.cpp
  )
  target_include_directories(test_contour_processing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_contour_processing
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_contour_processing
    rclcpp
    std_msgs
    rises_interfaces
  )

  # TF handling tests
  ament_add_gtest(test_tf_handling
    test/test_tf_handling.cpp
  )
  target_include_directories(test_tf_handling PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_tf_handling
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_tf_handling
    rclcpp
    geometry_msgs
    tf2_ros
    tf2_msgs
  )

  # Integration tests
  ament_add_gtest(test_integration
    test/test_integration.cpp
  )
  target_include_directories(test_integration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_integration
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_integration
    rclcpp
    std_msgs
    rises_interfaces
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_msgs
  )

  # Audit #4 oversize-input regression tests (AABB + contours)
  find_package(test_support REQUIRED)
  ament_add_gtest(test_audit_04_oversize_contours
    test/regression/test_audit_04_oversize_contours.cpp
  )
  target_include_directories(test_audit_04_oversize_contours PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_audit_04_oversize_contours
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_audit_04_oversize_contours
    rclcpp
    rises_interfaces
    geometry_msgs
    test_support
  )

  # Gap-closure: AabbConverter contract tests.
  ament_add_gtest(test_aabb_converter
    test/test_aabb_converter.cpp
  )
  target_include_directories(test_aabb_converter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_aabb_converter
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_aabb_converter
    rclcpp
    rises_interfaces
  )

  # Gap-closure: AreaLocksConverter contract tests.
  ament_add_gtest(test_area_locks
    test/test_area_locks.cpp
  )
  target_include_directories(test_area_locks PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_area_locks
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_area_locks
    rclcpp
    rises_interfaces
  )

  # Gap-closure: Vda5050Converter contract tests.
  # Linked only against message_translator — fleet_interface has a
  # duplicate symbol; ODR forbids linking both into one TU. See test header.
  ament_add_gtest(test_vda5050_converter
    test/test_vda5050_converter.cpp
  )
  target_include_directories(test_vda5050_converter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_vda5050_converter
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_vda5050_converter
    rclcpp
    rises_interfaces
    geometry_msgs
    nav_msgs
  )
endif()

ament_package()
