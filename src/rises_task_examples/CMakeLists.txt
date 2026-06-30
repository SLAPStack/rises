cmake_minimum_required(VERSION 3.22)
project(rises_task_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(task_examples_node
    src/task_examples_node.cpp
    src/task_runner.cpp
    src/main.cpp
)

target_include_directories(task_examples_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(task_examples_node
    rclcpp
    rclcpp_action
    rises_interfaces
    nav_msgs
    geometry_msgs
    std_srvs
)

install(TARGETS task_examples_node
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

    ament_add_gtest(test_task_runner_steps
        test/test_task_runner_steps.cpp
        src/task_runner.cpp
        src/task_examples_node.cpp)
    target_include_directories(test_task_runner_steps PRIVATE include)
    ament_target_dependencies(test_task_runner_steps
        rclcpp
        rclcpp_action
        rises_interfaces
        nav_msgs
        geometry_msgs
        std_srvs
        test_support)
endif()

ament_package()
