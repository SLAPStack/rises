cmake_minimum_required(VERSION 3.22)
project(rises_mission_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(hri_actions_msgs REQUIRED)
find_package(std_msgs REQUIRED)

option(ARIS_ENABLE_TSAN "Enable ThreadSanitizer for tests" OFF)

add_executable(mission_controller_node
  src/mission_controller_node.cpp
  src/main.cpp
)

target_include_directories(mission_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(mission_controller_node
  rclcpp
  rclcpp_action
  rises_interfaces
  hri_actions_msgs
  std_msgs
)

install(TARGETS mission_controller_node
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

  ament_add_gtest(test_mission_state_machine
    test/test_mission_state_machine.cpp
    src/mission_controller_node.cpp)
  target_include_directories(test_mission_state_machine PRIVATE include)
  ament_target_dependencies(test_mission_state_machine
    rclcpp
    rclcpp_action
    rises_interfaces
    hri_actions_msgs
    std_msgs
    test_support)

  ament_add_gtest(test_active_mission_id
    test/test_active_mission_id.cpp
    src/mission_controller_node.cpp)
  target_include_directories(test_active_mission_id PRIVATE include)
  ament_target_dependencies(test_active_mission_id
    rclcpp
    rclcpp_action
    rises_interfaces
    hri_actions_msgs
    std_msgs
    test_support)

  # Phase 3 concurrency test: active_mission_id_ read without mutex.
  ament_add_gtest(test_active_mission_race
    test/concurrency/test_active_mission_race.cpp
    src/mission_controller_node.cpp)
  target_include_directories(test_active_mission_race PRIVATE include)
  ament_target_dependencies(test_active_mission_race
    rclcpp
    rclcpp_action
    rises_interfaces
    hri_actions_msgs
    std_msgs
    test_support)
  if(ARIS_ENABLE_TSAN)
    target_compile_options(test_active_mission_race PRIVATE -fsanitize=thread -O1 -g)
    target_link_options(test_active_mission_race PRIVATE -fsanitize=thread)
  endif()
  set_tests_properties(test_active_mission_race PROPERTIES TIMEOUT 30)
endif()

ament_package()
