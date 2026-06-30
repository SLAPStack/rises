cmake_minimum_required(VERSION 3.22)
project(laserscan_preprocessor)

# Add custom CMake modules directory
set(LASERSCAN_CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${LASERSCAN_CMAKE_MODULE_PATH}")

# --- Build Configuration ---
set(LASER_COUNT "" CACHE STRING "Exact number of laser scanners (compile-time)")
set(SCAN_POINT_COUNT "" CACHE STRING "Expected number of points per laser scan (compile-time optimization, optional)")

# Enforce LASER_COUNT requirement
if(NOT LASER_COUNT OR LASER_COUNT STREQUAL "")
    message(FATAL_ERROR 
        "\n"
        "=====================================================\n"
        "ERROR: LASER_COUNT must be specified at build time!\n"
        "=====================================================\n"
        "\n"
        "This package requires compile-time configuration of the number of lasers.\n"
        "\n"
        "Usage:\n"
        "  colcon build --cmake-args -DLASER_COUNT=<N>\n"
        "\n"
        "Example for 4 lasers:\n"
        "  colcon build --cmake-args -DLASER_COUNT=4\n"
        "\n"
        "Or when using cmake directly:\n"
        "  cmake -DLASER_COUNT=4 ..\n"
        "\n"
        "=====================================================\n"
    )
endif()

# SCAN_POINT_COUNT is optional - provides compile-time optimization
if(SCAN_POINT_COUNT AND NOT SCAN_POINT_COUNT STREQUAL "")
    message(STATUS "SCAN_POINT_COUNT=${SCAN_POINT_COUNT} - enabling compile-time optimizations")
else()
    message(STATUS "SCAN_POINT_COUNT not specified - using runtime point count calculation")
endif()

# --- Find required packages ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Try to find rises_interfaces, make it optional for testing
find_package(rises_interfaces QUIET)
if(rises_interfaces_FOUND)
    message(STATUS "Found rises_interfaces")
    set(HAS_ARISE_INTERFACES TRUE)
else()
    message(WARNING "rises_interfaces not found, using fallback for testing")
    set(HAS_ARISE_INTERFACES FALSE)
endif()

# nanoflann for spatial indexing (header-only) - with FetchContent fallback
find_package(Nanoflann REQUIRED)

# XSIMD for SIMD optimizations (required when USE_SIMD=ON)
if(USE_SIMD)
    find_package(XSIMD REQUIRED)
endif()

# --- Create library ---
add_library(${PROJECT_NAME} SHARED
  src/laserscan_preprocessor_node.cpp
  src/segmentation/segmentation_strategy.cpp
  src/shapes/shape_fitter.cpp
  src/spatial/spatial_indexer.cpp
  src/processing/point_cloud_processor.cpp
)

# Compiler flags
target_compile_definitions(${PROJECT_NAME} PUBLIC 
  "LASER_COUNT=${LASER_COUNT}"
)
if(SCAN_POINT_COUNT AND NOT SCAN_POINT_COUNT STREQUAL "")
  target_compile_definitions(${PROJECT_NAME} PUBLIC 
    "SCAN_POINT_COUNT=${SCAN_POINT_COUNT}"
  )
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE
  $<$<BOOL:${USE_SIMD}>:USE_SIMD>
  $<$<BOOL:${HAS_ARISE_INTERFACES}>:HAS_ARISE_INTERFACES>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
  if(USE_SIMD)
    target_compile_options(${PROJECT_NAME} PRIVATE -mavx2 -mfma)
  endif()
endif()
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link nanoflann (header-only)
if(TARGET nanoflann::nanoflann)
  target_link_libraries(${PROJECT_NAME} nanoflann::nanoflann)
else()
  target_include_directories(${PROJECT_NAME} PRIVATE ${NANOFLANN_INCLUDE_DIR})
endif()

# Link xsimd (header-only)
if(USE_SIMD)
  if(TARGET xsimd)
    target_link_libraries(${PROJECT_NAME} xsimd)
  else()
    target_include_directories(${PROJECT_NAME} PRIVATE ${XSIMD_INCLUDE_DIR})
  endif()
endif()

# --- Link to ROS 2 targets (correct way) ---
set(AMENT_DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  message_filters
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  diagnostic_updater
)

if(HAS_ARISE_INTERFACES)
  list(APPEND AMENT_DEPENDENCIES rises_interfaces)
endif()

ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})

# Link TBB for parallel execution
find_package(TBB REQUIRED)
target_link_libraries(${PROJECT_NAME} TBB::tbb)
message(STATUS "Found TBB, enabling parallel execution")

# Enable LTO for release builds
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
  set_target_properties(${PROJECT_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Register as a composable node
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rises::LaserPreprocessorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# --- Install rules ---
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/test_scan_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test_node
    test/test_laserscan_preprocessor_node.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_node PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_node
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    rises_interfaces
    tf2_ros
    tf2_geometry_msgs
  )
  
  ament_add_gtest(${PROJECT_NAME}_test_algorithms
    test/test_algorithms.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_algorithms ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_algorithms PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_algorithms
    rclcpp
    rclcpp_lifecycle
    Eigen3
  )
  
  ament_add_gtest(${PROJECT_NAME}_test_segmentation_logic
    test/test_segmentation_logic.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_segmentation_logic ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_segmentation_logic PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_segmentation_logic
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    rises_interfaces
    Eigen3
  )

  find_package(test_support REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_audit_07_exception_ghost_clear
    test/regression/test_audit_07_exception_ghost_clear.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_audit_07_exception_ghost_clear ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_audit_07_exception_ghost_clear PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_audit_07_exception_ghost_clear
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    rises_interfaces
    tf2_ros
    tf2_geometry_msgs
    test_support
  )

  # Gap-closure: multi-laser synchronization observable contract.
  ament_add_gtest(${PROJECT_NAME}_test_multi_laser_sync
    test/test_multi_laser_sync.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_multi_laser_sync ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_multi_laser_sync PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_multi_laser_sync
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    rises_interfaces
    tf2_ros
    tf2_geometry_msgs
  )

  # Gap-closure: TF lookup failure / recovery contract.
  ament_add_gtest(${PROJECT_NAME}_test_tf_failure_modes
    test/test_tf_failure_modes.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_tf_failure_modes ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_tf_failure_modes PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_tf_failure_modes
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    rises_interfaces
    tf2_ros
    tf2_geometry_msgs
  )

  # Gap-closure: PolygonFitter shape strategy contract.
  ament_add_gtest(${PROJECT_NAME}_test_polygon_fitter
    test/test_polygon_fitter.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_polygon_fitter ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_polygon_fitter PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_polygon_fitter
    rises_interfaces
    Eigen3
  )

  # Gap-closure: PointFitter shape strategy contract.
  ament_add_gtest(${PROJECT_NAME}_test_point_fitter
    test/test_point_fitter.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_point_fitter ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_point_fitter PRIVATE "LASER_COUNT=${LASER_COUNT}")
  ament_target_dependencies(${PROJECT_NAME}_test_point_fitter
    rises_interfaces
    Eigen3
  )
endif()

# Export the package
ament_package()
