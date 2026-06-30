cmake_minimum_required(VERSION 3.22)
project(obstacle_heatmap_predictor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

option(ARIS_ENABLE_TSAN "Enable ThreadSanitizer for tests" OFF)

add_executable(heatmap_predictor_node
  src/heatmap_predictor_node.cpp
  src/main.cpp
)

target_include_directories(heatmap_predictor_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(heatmap_predictor_node
  rclcpp
  rclcpp_lifecycle
  rises_interfaces
  nav_msgs
  std_msgs
)

install(TARGETS heatmap_predictor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_support REQUIRED)

  ament_add_gtest(test_track_decay
    test/test_track_decay.cpp
    src/heatmap_predictor_node.cpp)
  target_include_directories(test_track_decay PRIVATE include)
  ament_target_dependencies(test_track_decay
    rclcpp rises_interfaces nav_msgs std_msgs test_support)

  ament_add_gtest(test_grid_serialization
    test/test_grid_serialization.cpp
    src/heatmap_predictor_node.cpp)
  target_include_directories(test_grid_serialization PRIVATE include)
  ament_target_dependencies(test_grid_serialization
    rclcpp rises_interfaces nav_msgs std_msgs test_support)

  # Phase 3 concurrency test: tracked_obstacles_ writer + timer reader.
  ament_add_gtest(test_tracks_race
    test/concurrency/test_tracks_race.cpp
    src/heatmap_predictor_node.cpp)
  target_include_directories(test_tracks_race PRIVATE include)
  ament_target_dependencies(test_tracks_race
    rclcpp rises_interfaces nav_msgs std_msgs test_support)
  if(ARIS_ENABLE_TSAN)
    target_compile_options(test_tracks_race PRIVATE -fsanitize=thread -O1 -g)
    target_link_options(test_tracks_race PRIVATE -fsanitize=thread)
  endif()
  set_tests_properties(test_tracks_race PROPERTIES TIMEOUT 30)
endif()

ament_package()
