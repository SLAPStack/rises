cmake_minimum_required(VERSION 3.16)
project(test_support LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp
  rclcpp_lifecycle
  rosgraph_msgs
  rises_interfaces
  lifecycle_msgs
  std_msgs
  geometry_msgs
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rosgraph_msgs
  rises_interfaces
  lifecycle_msgs
  std_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_support_smoke test/test_smoke.cpp)
  target_link_libraries(test_support_smoke ${PROJECT_NAME})
  ament_target_dependencies(test_support_smoke
    rclcpp
    rclcpp_lifecycle
    rosgraph_msgs
    rises_interfaces
    lifecycle_msgs
  )
endif()

ament_package()
