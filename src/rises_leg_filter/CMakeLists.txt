cmake_minimum_required(VERSION 3.22)
project(rises_leg_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

option(ARIS_ENABLE_TSAN "Enable ThreadSanitizer for tests" OFF)

add_executable(leg_filter_node
    src/leg_filter_node.cpp
    src/main.cpp
)

target_include_directories(leg_filter_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(leg_filter_node
    rclcpp
    rises_interfaces
    geometry_msgs
    std_msgs
    tf2_ros
)

install(TARGETS leg_filter_node
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
    find_package(sensor_msgs REQUIRED)
    find_package(tf2 REQUIRED)

    ament_add_gtest(test_audit_12_origin_placeholder
        test/regression/test_audit_12_origin_placeholder.cpp
        src/leg_filter_node.cpp
    )
    target_include_directories(test_audit_12_origin_placeholder PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_audit_12_origin_placeholder
        rclcpp
        rises_interfaces
        geometry_msgs
        std_msgs
        tf2_ros
        test_support
    )

    ament_add_gtest(test_leg_track_lifecycle
        test/test_leg_track_lifecycle.cpp
        src/leg_filter_node.cpp
    )
    target_include_directories(test_leg_track_lifecycle PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_leg_track_lifecycle
        rclcpp
        rises_interfaces
        sensor_msgs
        geometry_msgs
        std_msgs
        tf2
        tf2_ros
        test_support
    )

    # Phase 3 concurrency test: tracks_ + confirmed_humans_ across callbacks.
    ament_add_gtest(test_leg_filter_race
        test/concurrency/test_leg_filter_race.cpp
        src/leg_filter_node.cpp
    )
    target_include_directories(test_leg_filter_race PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_leg_filter_race
        rclcpp
        rises_interfaces
        geometry_msgs
        std_msgs
        tf2_ros
        test_support
    )
    if(ARIS_ENABLE_TSAN)
        target_compile_options(test_leg_filter_race PRIVATE -fsanitize=thread -O1 -g)
        target_link_options(test_leg_filter_race PRIVATE -fsanitize=thread)
    endif()
    set_tests_properties(test_leg_filter_race PROPERTIES TIMEOUT 30)
endif()

ament_package()
