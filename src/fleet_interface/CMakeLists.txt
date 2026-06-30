cmake_minimum_required(VERSION 3.22)
project(fleet_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/fleet_interface_node.cpp
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
  rises_interfaces
  geometry_msgs
  nav_msgs
  tf2_ros
)

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
  set_target_properties(${PROJECT_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rises::FleetInterfaceNode"
  EXECUTABLE fleet_interface_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_support REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(tf2 REQUIRED)

  ament_add_gtest(test_audit_04_oversize_vda5050
    test/regression/test_audit_04_oversize_vda5050.cpp
    src/vda5050_converter.cpp)
  target_include_directories(test_audit_04_oversize_vda5050 PRIVATE include)
  target_link_libraries(test_audit_04_oversize_vda5050
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_audit_04_oversize_vda5050
    rclcpp
    rises_interfaces
    nav_msgs
    test_support
  )

  ament_add_gtest(test_audit_13_frame_param_swap
    test/regression/test_audit_13_frame_param_swap.cpp
    src/fleet_interface_node.cpp
    src/vda5050_converter.cpp
    src/vda5050_state_bridge.cpp)
  target_include_directories(test_audit_13_frame_param_swap PRIVATE include)
  target_link_libraries(test_audit_13_frame_param_swap
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_audit_13_frame_param_swap
    rclcpp
    rclcpp_components
    std_msgs
    rises_interfaces
    geometry_msgs
    nav_msgs
    tf2_ros
    test_support
  )

  ament_add_gtest(test_state_bridge
    test/test_state_bridge.cpp
    src/vda5050_state_bridge.cpp
    src/vda5050_converter.cpp)
  target_include_directories(test_state_bridge PRIVATE include)
  target_link_libraries(test_state_bridge
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_state_bridge
    rclcpp
    rises_interfaces
    nav_msgs
    std_msgs
    sensor_msgs
    diagnostic_msgs
    geometry_msgs
    tf2
    tf2_ros
    test_support
  )
endif()

ament_package()
