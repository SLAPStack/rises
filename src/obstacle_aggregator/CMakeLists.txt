cmake_minimum_required(VERSION 3.22)
project(obstacle_aggregator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rises_interfaces REQUIRED)

add_executable(obstacle_aggregator_node
  src/obstacle_aggregator_node.cpp
  src/main.cpp
)

target_include_directories(obstacle_aggregator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(obstacle_aggregator_node
  rclcpp
  rises_interfaces
)

install(TARGETS obstacle_aggregator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_support REQUIRED)

  ament_add_gtest(test_audit_11_id_offset
    test/regression/test_audit_11_id_offset.cpp
    src/obstacle_aggregator_node.cpp
  )
  target_include_directories(test_audit_11_id_offset PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_audit_11_id_offset
    rclcpp
    rises_interfaces
    test_support
  )

  ament_add_gtest(test_combine
    test/test_combine.cpp
    src/obstacle_aggregator_node.cpp
  )
  target_include_directories(test_combine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_combine
    rclcpp
    rises_interfaces
    std_msgs
    test_support
  )
endif()

ament_package()
