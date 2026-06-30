cmake_minimum_required(VERSION 3.22)
project(fiware_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

option(ARIS_ENABLE_TSAN "Enable ThreadSanitizer for tests" OFF)

# nlohmann-json (header-only, already in workspace or system)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(json)
endif()

add_library(${PROJECT_NAME} SHARED
  src/fiware_bridge_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_components::component
  ${rises_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  nlohmann_json::nlohmann_json
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rises::FiwareBridgeNode"
  EXECUTABLE fiware_bridge_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_support REQUIRED)
  find_package(geometry_msgs REQUIRED)

  ament_add_gtest(test_audit_14_grid_index_overflow
    test/regression/test_audit_14_grid_index_overflow.cpp
  )
  target_include_directories(test_audit_14_grid_index_overflow PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_audit_14_grid_index_overflow
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_audit_14_grid_index_overflow
    rclcpp
    rises_interfaces
    std_msgs
    nav_msgs
    diagnostic_msgs
    test_support
  )

  ament_add_gtest(test_audit_15_diag_json_injection
    test/regression/test_audit_15_diag_json_injection.cpp
  )
  target_include_directories(test_audit_15_diag_json_injection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_audit_15_diag_json_injection
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_audit_15_diag_json_injection
    rclcpp
    rises_interfaces
    std_msgs
    nav_msgs
    diagnostic_msgs
    test_support
  )

  ament_add_gtest(test_message_serialization
    test/test_message_serialization.cpp
  )
  target_include_directories(test_message_serialization PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_message_serialization
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_message_serialization
    rclcpp
    rises_interfaces
    std_msgs
    nav_msgs
    diagnostic_msgs
    geometry_msgs
    test_support
  )

  ament_add_gtest(test_throttling
    test/test_throttling.cpp
  )
  target_include_directories(test_throttling PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_throttling
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_throttling
    rclcpp
    rises_interfaces
    std_msgs
    nav_msgs
    test_support
  )

  # Phase 3 concurrency test: pallet_map_ + pallet_map_dirty_ across callbacks.
  ament_add_gtest(test_pallet_map_race
    test/concurrency/test_pallet_map_race.cpp
  )
  target_include_directories(test_pallet_map_race PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_pallet_map_race
    ${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )
  ament_target_dependencies(test_pallet_map_race
    rclcpp
    rises_interfaces
    std_msgs
    nav_msgs
    diagnostic_msgs
    test_support
  )
  if(ARIS_ENABLE_TSAN)
    target_compile_options(test_pallet_map_race PRIVATE -fsanitize=thread -O1 -g)
    target_link_options(test_pallet_map_race PRIVATE -fsanitize=thread)
  endif()
  set_tests_properties(test_pallet_map_race PROPERTIES TIMEOUT 30)
endif()

ament_package()
