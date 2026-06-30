cmake_minimum_required(VERSION 3.22)
project(rises_skill_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/skill_bridge_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    ${rises_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
)

# Register the node as a composable component (for use in a component_container_mt).
# The macro requires an EXECUTABLE name, but its generated standalone main uses a
# SingleThreadedExecutor, which deadlocks this node (see skill_bridge_main.cpp). We
# therefore point it at an unused name and ship our own multi-threaded executable below.
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "rises::SkillBridgeNode"
    EXECUTABLE skill_bridge_node_singlethreaded_unused
)

# Standalone executable with an explicit MultiThreadedExecutor (required by this node).
# This is the executable the launch files run.
add_executable(skill_bridge_node src/skill_bridge_main.cpp)
target_link_libraries(skill_bridge_node ${PROJECT_NAME} rclcpp::rclcpp)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS skill_bridge_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY skills/
    DESTINATION share/${PROJECT_NAME}/skills
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(test_support REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(nlohmann_json REQUIRED)

    ament_add_gtest(test_skill_loader
        test/test_skill_loader.cpp)
    target_include_directories(test_skill_loader PRIVATE include)
    target_link_libraries(test_skill_loader nlohmann_json::nlohmann_json)
    ament_target_dependencies(test_skill_loader
        rclcpp rclcpp_action rises_interfaces ament_index_cpp test_support)

    ament_add_gtest(test_action_routing
        test/test_action_routing.cpp)
    target_include_directories(test_action_routing PRIVATE include)
    target_link_libraries(test_action_routing ${PROJECT_NAME})
    ament_target_dependencies(test_action_routing
        rclcpp rclcpp_action rises_interfaces test_support)
endif()

ament_package()
