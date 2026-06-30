cmake_minimum_required(VERSION 3.22)
project(safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------
# Find dependencies
# ------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rises_interfaces REQUIRED)

option(ARIS_ENABLE_TSAN "Enable ThreadSanitizer for tests" OFF)

# ------------------------------
# Build as a shared library for composable node
# ------------------------------
add_library(${PROJECT_NAME} SHARED src/safety.cpp)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  rises_interfaces
)

# Enable LTO for release builds
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
  set_target_properties(${PROJECT_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# ------------------------------
# Register as a composable node
# ------------------------------
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rises::Safety"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ------------------------------
# Install targets
# ------------------------------
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ------------------------------
# Linting & testing
# ------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_support REQUIRED)

  ament_add_gtest(test_audit_06_pathcallback_silent_drop
    test/regression/test_audit_06_pathcallback_silent_drop.cpp
  )
  target_include_directories(test_audit_06_pathcallback_silent_drop PRIVATE include)
  target_link_libraries(test_audit_06_pathcallback_silent_drop ${PROJECT_NAME})
  ament_target_dependencies(test_audit_06_pathcallback_silent_drop
    rclcpp
    rclcpp_components
    std_msgs
    nav_msgs
    geometry_msgs
    diagnostic_msgs
    rises_interfaces
    test_support
  )

  ament_add_gtest(test_safety_logic
    test/test_safety_logic.cpp
  )
  target_include_directories(test_safety_logic PRIVATE include)
  target_link_libraries(test_safety_logic ${PROJECT_NAME})
  ament_target_dependencies(test_safety_logic
    rclcpp
    rclcpp_components
    std_msgs
    nav_msgs
    geometry_msgs
    diagnostic_msgs
    rises_interfaces
    test_support
  )

  # Phase 3 concurrency test: heatmap SharedPtr race in pathCallback.
  ament_add_gtest(test_heatmap_race
    test/concurrency/test_heatmap_race.cpp
  )
  target_include_directories(test_heatmap_race PRIVATE include)
  target_link_libraries(test_heatmap_race ${PROJECT_NAME})
  ament_target_dependencies(test_heatmap_race
    rclcpp
    rclcpp_components
    std_msgs
    nav_msgs
    geometry_msgs
    diagnostic_msgs
    rises_interfaces
    test_support
  )
  if(ARIS_ENABLE_TSAN)
    target_compile_options(test_heatmap_race PRIVATE -fsanitize=thread -O1 -g)
    target_link_options(test_heatmap_race PRIVATE -fsanitize=thread)
  endif()
  set_tests_properties(test_heatmap_race PROPERTIES TIMEOUT 30)
endif()

ament_package()
