cmake_minimum_required(VERSION 3.22)
project(geofence)

# Add custom CMake modules directory
set(GEOFENCE_CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${GEOFENCE_CMAKE_MODULE_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rises_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Find Eigen3 (with FetchContent fallback)
find_package(Eigen3Custom REQUIRED)

# Find liburcu (with FetchContent fallback)
find_package(Liburcu REQUIRED)

# Find nlohmann_json (with FetchContent fallback)
find_package(Nlohmann_json REQUIRED)

if(USE_SIMD)
    find_package(XSIMD REQUIRED)
    message(STATUS "SIMD collision checking: ENABLED")
else()
    message(STATUS "SIMD collision checking: DISABLED")
endif()

# ============================================================================
# Build Configuration Options
# ============================================================================

# Node implementation selection: spatial, gridmap, or both
set(BUILD_NODE "both" CACHE STRING "Select which node to build (spatial, gridmap, or both)")
set_property(CACHE BUILD_NODE PROPERTY STRINGS spatial gridmap both)

# Validate selection
if(NOT BUILD_NODE MATCHES "^(spatial|gridmap|both)$")
    message(FATAL_ERROR "Invalid BUILD_NODE '${BUILD_NODE}'. Valid options are: spatial, gridmap, both")
endif()

message(STATUS "Building node(s): ${BUILD_NODE}")

option(GEOFENCE_PROFILE "Enable profiling timers in hot path" OFF)
option(ARIS_ENABLE_TSAN "Enable ThreadSanitizer for tests" OFF)

# Spatial index: Nanoflann only (header-only KD-tree). The pluggable
# Boost-rtree / libspatialindex / CUDA-BVH backends were removed (unused,
# untested, and not faster for 2D in-memory queries).

# Dynamic obstacle capacity configuration
set(DYNAMIC_OBSTACLE_CAPACITY "128" CACHE STRING "Maximum number of dynamic obstacles in the obstacle layer")
message(STATUS "Dynamic obstacle capacity: ${DYNAMIC_OBSTACLE_CAPACITY}")

# Validate capacity (must be power of 2 for optimal performance, but not strictly required)
if(NOT DYNAMIC_OBSTACLE_CAPACITY MATCHES "^[0-9]+$")
    message(FATAL_ERROR "DYNAMIC_OBSTACLE_CAPACITY must be a positive integer, got: ${DYNAMIC_OBSTACLE_CAPACITY}")
endif()

if(DYNAMIC_OBSTACLE_CAPACITY LESS 16 OR DYNAMIC_OBSTACLE_CAPACITY GREATER 1024)
    message(WARNING "DYNAMIC_OBSTACLE_CAPACITY=${DYNAMIC_OBSTACLE_CAPACITY} is outside recommended range [16, 1024]")
endif()

if(USE_SIMD)
    message(STATUS "Collision checker: SIMD-optimized")
else()
    message(STATUS "Collision checker: Standard")
endif()

# Configure config.hpp from template
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/include/geofence/spatial/map/config.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/geofence/spatial/map/config.hpp
    @ONLY
)

# ============================================================================
# Shared Utilities Library
# ============================================================================

# Shared utilities (RCU, geometry_intersection, etc.)
add_library(geofence_utils INTERFACE)
target_include_directories(geofence_utils INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils/include>
    $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Backend Libraries
# ============================================================================

# Build spatial backend (only if building spatial node)
if(BUILD_NODE STREQUAL "spatial" OR BUILD_NODE STREQUAL "both")
    add_subdirectory(spatial_node/backend/spatialindex/nanoflann)
    set(SELECTED_SPATIAL_BACKEND datastructure_backend_nanoflann)
endif()

# Build shared geometry utilities (always needed, uses Boost.Geometry)
add_library(geometry_utils STATIC
    utils/src/geometry_intersection.cpp
)
target_include_directories(geometry_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(geometry_utils
    ${rises_interfaces_TARGETS}
)
set_target_properties(geometry_utils PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Enable LTO for release builds
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
    set_target_properties(geometry_utils PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# ============================================================================
# Common Library (backend-agnostic scaffolding shared by both nodes)
# ============================================================================

# geofence_common holds backend-agnostic sources used by BOTH the spatial and
# gridmap nodes (and by most tests): the geometry variant, robot tracking, the
# robot-tracking correspondence checker, coordinate transforms, the lifecycle
# base class, and latency recording. Physically lives under geofence/common/
# (not spatial_node/) since it is shared by both backends. Declared
# UNCONDITIONALLY (like geometry_utils) so it builds under
# BUILD_NODE=spatial|gridmap|both. Both node libraries link it instead of each
# recompiling these .cpp files. Crucially this also gives geofence_spatial the
# robot_tracking_checker.cpp it USES but never compiled (the .so previously
# carried an undefined RobotTrackingChecker symbol, breaking any consumer -- the
# audit tests -- that did not separately compile that translation unit).
#
# Only macro-clean files live here: robot_safety_profile.cpp (#ifdef USE_SIMD)
# deliberately stays in the spatial node so this library needs no USE_SIMD /
# USE_RTREE_NODE compile context. lifecycle_geofence_node_base.hpp still pulls
# in a few spatial-only headers (obstacle_report_builder.hpp,
# geofence_visualizer.hpp, obstacle_match_result.hpp) that have not moved, so
# spatial_node/include stays on this target's include path.
add_library(geofence_common STATIC
    common/src/geometry/variant_geometry.cpp
    common/src/policies/robot_tracking.cpp
    common/src/queries/robot_tracking_checker.cpp
    common/src/policies/coordinate_transform.cpp
    common/src/node/geofence_common_config.cpp
    common/src/node/lifecycle_geofence_node_base.cpp
    common/src/latency_recorder.cpp
)
target_include_directories(geofence_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/geometry/core>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/core>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${URCU_INCLUDE_DIRS}
)
target_link_libraries(geofence_common
    geometry_utils
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${lifecycle_msgs_TARGETS}
    ${rises_interfaces_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${tf2_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    Eigen3::Eigen
    ${URCU_LIBRARIES}
)
target_compile_features(geofence_common PUBLIC cxx_std_17)
set_target_properties(geofence_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(ipo_supported)
    set_target_properties(geofence_common PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# ============================================================================
# Spatial Node (R-tree based)
# ============================================================================

if(BUILD_NODE STREQUAL "spatial" OR BUILD_NODE STREQUAL "both")
    set(SPATIAL_NODE_NAME geofence_spatial)
    
    # SIMD collision source (conditionally compiled)
    set(COLLISION_CHECKER_SOURCES "")
    set(CORRESPONDENCE_MATCHER_SOURCES "")
    if(USE_SIMD)
        list(APPEND COLLISION_CHECKER_SOURCES spatial_node/src/queries/obstacle_collision_checker_simd.cpp)
        list(APPEND CORRESPONDENCE_MATCHER_SOURCES spatial_node/src/queries/obstacle_correspondence_matcher_simd.cpp)
    else()
        list(APPEND COLLISION_CHECKER_SOURCES spatial_node/src/queries/obstacle_collision_checker.cpp)
        list(APPEND CORRESPONDENCE_MATCHER_SOURCES spatial_node/src/queries/obstacle_correspondence_matcher.cpp)
    endif()
    
    add_library(${SPATIAL_NODE_NAME} SHARED
        spatial_node/src/node/geofencing_node.cpp
        spatial_node/src/node/geofence_config.cpp
        spatial_node/src/node/obstacle_report_builder.cpp
        spatial_node/src/node/map_update_handler.cpp
        spatial_node/src/visualization/geofence_visualizer.cpp
        spatial_node/src/map/geofence_map.cpp
        spatial_node/src/policies/robot_safety_profile.cpp
        spatial_node/src/queries/boundary_checker.cpp
        spatial_node/src/queries/path_safety_checker.cpp
        spatial_node/src/queries/obstacle_match_checker.cpp
        spatial_node/src/queries/batch_point_checker.cpp
        spatial_node/src/utils/json_loader.cpp
        ${COLLISION_CHECKER_SOURCES}
        ${CORRESPONDENCE_MATCHER_SOURCES}
    )
    
    target_include_directories(${SPATIAL_NODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/core>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/nanoflann>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/geometry/core>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        ${URCU_INCLUDE_DIRS}
    )
    
    # Add xsimd include directory when SIMD is enabled
    if(USE_SIMD)
        if(xsimd_FOUND)
            target_link_libraries(${SPATIAL_NODE_NAME} xsimd)
        elseif(XSIMD_INCLUDE_DIR)
            target_include_directories(${SPATIAL_NODE_NAME} PRIVATE ${XSIMD_INCLUDE_DIR})
        endif()
    endif()
    
    target_compile_definitions(${SPATIAL_NODE_NAME} PUBLIC USE_RTREE_NODE)
    
    if(USE_SIMD)
        target_compile_definitions(${SPATIAL_NODE_NAME} PRIVATE USE_SIMD)
    endif()
    
    # Spatial index is Nanoflann (default in spatial_index_selection.hpp; no
    # define needed). Geometry uses Boost.Geometry unconditionally.
    target_compile_features(${SPATIAL_NODE_NAME} PUBLIC cxx_std_17)

    if(GEOFENCE_PROFILE)
        target_compile_definitions(${SPATIAL_NODE_NAME} PRIVATE GEOFENCE_PROFILE)
        message(STATUS "Geofence profiling: ENABLED")
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${SPATIAL_NODE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    
    target_link_libraries(${SPATIAL_NODE_NAME}
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        rclcpp_components::component
        ${lifecycle_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${rises_interfaces_TARGETS}
        ${std_msgs_TARGETS}
        ${tf2_TARGETS}
        ${tf2_ros_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
        ${diagnostic_updater_TARGETS}
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        ${SELECTED_SPATIAL_BACKEND}
        ${URCU_LIBRARIES}
        geometry_utils
        geofence_common
    )

    if(ipo_supported)
        set_target_properties(${SPATIAL_NODE_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()

    rclcpp_components_register_node(${SPATIAL_NODE_NAME}
        PLUGIN "rises::GeofenceNode"
        EXECUTABLE ${SPATIAL_NODE_NAME}_node
    )
    
    install(TARGETS ${SPATIAL_NODE_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()

# ============================================================================
# Gridmap Node
# ============================================================================

if(BUILD_NODE STREQUAL "gridmap" OR BUILD_NODE STREQUAL "both")
    set(GRIDMAP_NODE_NAME geofence_gridmap)
    
    add_library(${GRIDMAP_NODE_NAME} SHARED
        gridmap_node/src/node/geofencing_gridmap_node.cpp
        gridmap_node/src/node/gridmap_config.cpp
        gridmap_node/src/map/gridmap_data.cpp
        gridmap_node/src/map/gridmap.cpp
        gridmap_node/src/policies/gridmap_policies.cpp
        # Backend-agnostic shared sources (variant_geometry, robot_tracking,
        # robot_tracking_checker, coordinate_transform) now come from
        # geofence_common (linked below) instead of being recompiled here.
        # NOTE: json_loader.cpp NOT included – it depends on fromObstacleMsg() /
        # geofence_map.hpp (spatial-index types). loadObstacles() / loadContours()
        # are header-only in json_loader_impl.hpp and work fine without it.
        spatial_node/src/visualization/geofence_visualizer.cpp
        # Shared report builder: turns per-vertex matched/unmatched classification
        # into per-segment ObstacleReport (real positions) so the gridmap node
        # feeds the same KPI pipeline as the spatial node. It only uses the plain
        # ObstacleResult struct (no spatial-index calls), but transitively pulls
        # geofence_map.hpp headers, which compile header-only with the include
        # paths/URCU already configured for this target.
        spatial_node/src/node/obstacle_report_builder.cpp
    )
    
    target_include_directories(${GRIDMAP_NODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gridmap_node/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/geometry/core>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/core>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        ${URCU_INCLUDE_DIRS}
    )
    
    target_compile_definitions(${GRIDMAP_NODE_NAME} PUBLIC USE_GRIDMAP_NODE)
    
    
    target_compile_features(${GRIDMAP_NODE_NAME} PUBLIC cxx_std_17)
    
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${GRIDMAP_NODE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    
    target_link_libraries(${GRIDMAP_NODE_NAME}
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        rclcpp_components::component
        ${lifecycle_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${rises_interfaces_TARGETS}
        ${std_msgs_TARGETS}
        ${tf2_TARGETS}
        ${tf2_ros_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
        ${diagnostic_updater_TARGETS}
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        ${URCU_LIBRARIES}
        geometry_utils
        geofence_common
    )

    if(ipo_supported)
        set_target_properties(${GRIDMAP_NODE_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()

    rclcpp_components_register_node(${GRIDMAP_NODE_NAME}
        PLUGIN "rises::GeofenceGridmapNode"
        EXECUTABLE ${GRIDMAP_NODE_NAME}_node
    )
    
    install(TARGETS ${GRIDMAP_NODE_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()

# ============================================================================
# Validation Node (standalone, works with either geofence implementation)
# ============================================================================

set(VALIDATION_NODE_NAME geofence_validation)

add_library(${VALIDATION_NODE_NAME} SHARED
    validation_node/src/validation_node.cpp
)

target_include_directories(${VALIDATION_NODE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/validation_node/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${VALIDATION_NODE_NAME}
    rclcpp::rclcpp
    rclcpp_components::component
    ${std_msgs_TARGETS}
    ${rises_interfaces_TARGETS}
    ${tf2_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    nlohmann_json::nlohmann_json
)

target_compile_features(${VALIDATION_NODE_NAME} PUBLIC cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${VALIDATION_NODE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(ipo_supported)
    set_target_properties(${VALIDATION_NODE_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

rclcpp_components_register_node(${VALIDATION_NODE_NAME}
    PLUGIN "rises::ValidationNode"
    EXECUTABLE ${VALIDATION_NODE_NAME}_node
)

install(TARGETS ${VALIDATION_NODE_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# ============================================================================
# Installation
# ============================================================================

install(TARGETS geometry_utils geofence_common
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY utils/include/
    DESTINATION include
)

install(DIRECTORY common/include/
    DESTINATION include
)

if(BUILD_NODE STREQUAL "spatial" OR BUILD_NODE STREQUAL "both")
    install(DIRECTORY spatial_node/include/
        DESTINATION include
    )
endif()

if(BUILD_NODE STREQUAL "gridmap" OR BUILD_NODE STREQUAL "both")
    install(DIRECTORY gridmap_node/include/
        DESTINATION include
    )
endif()

install(DIRECTORY validation_node/include/
    DESTINATION include
)

# ============================================================================
# Testing
# ============================================================================

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(test_support REQUIRED)

    # Common include paths shared by every regression test (saves repetition).
    set(GEOFENCE_REGRESSION_SPATIAL_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/common/include
        ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/include
        ${CMAKE_CURRENT_BINARY_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/geometry/core
        ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/core
        ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/nanoflann
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
        ${EIGEN3_INCLUDE_DIRS}
    )

    # ------------------------------------------------------------------
    # geofence_add_test — DRY wrapper around the repetitive
    # ament_add_gtest + target_include_directories + target_link_libraries
    # + ament_target_dependencies boilerplate that every plain test target
    # repeats. Per-target includes/links/deps stay explicit (passed as
    # multi-value args) so this is a pure behaviour-preserving refactor.
    #
    #   geofence_add_test(<name>
    #       SOURCES  <srcs...>          # test + production .cpp files
    #       [INCLUDES <dirs...>]        # PRIVATE include directories
    #       [LIBS     <libs...>]        # target_link_libraries
    #       [DEPS     <ament_deps...>]) # ament_target_dependencies
    #
    # Targets with special needs (SIMD-conditional sources/defs, TSAN
    # options, TIMEOUT properties) stay as explicit calls below.
    # ------------------------------------------------------------------
    function(geofence_add_test test_name)
        cmake_parse_arguments(GAT "" "" "SOURCES;INCLUDES;LIBS;DEPS" ${ARGN})
        if(NOT GAT_SOURCES)
            message(FATAL_ERROR "geofence_add_test(${test_name}) requires SOURCES")
        endif()
        ament_add_gtest(${test_name} ${GAT_SOURCES})
        if(GAT_INCLUDES)
            target_include_directories(${test_name} PRIVATE ${GAT_INCLUDES})
        endif()
        if(GAT_LIBS)
            target_link_libraries(${test_name} ${GAT_LIBS})
        endif()
        if(GAT_DEPS)
            ament_target_dependencies(${test_name} ${GAT_DEPS})
        endif()
    endfunction()

    # Spatial node tests (only if building spatial node)
    if(BUILD_NODE STREQUAL "spatial" OR BUILD_NODE STREQUAL "both")
        # Spatial index query tests
        # (inline include list equals GEOFENCE_REGRESSION_SPATIAL_INCLUDES)
        geofence_add_test(test_spatial_index
            SOURCES
                test/test_spatial_index.cpp
                spatial_node/src/map/geofence_map.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SELECTED_SPATIAL_BACKEND} ${URCU_LIBRARIES}
            DEPS rclcpp rises_interfaces
        )

        # Path safety checker tests
        geofence_add_test(test_path_safety
            SOURCES
                test/test_path_safety.cpp
                spatial_node/src/map/geofence_map.cpp
                spatial_node/src/queries/boundary_checker.cpp
                spatial_node/src/queries/path_safety_checker.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SELECTED_SPATIAL_BACKEND} ${URCU_LIBRARIES}
            DEPS rclcpp rises_interfaces
        )

        # Obstacle conversion tests
        # (no nanoflann / binary-dir includes; no link libs)
        geofence_add_test(test_obstacle_conversions
            SOURCES
                test/test_obstacle_conversions.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES
                ${CMAKE_CURRENT_SOURCE_DIR}/common/include
                ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/include
                ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/geometry/core
                ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/core
                ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
                ${EIGEN3_INCLUDE_DIRS}
            DEPS rises_interfaces
        )

        # =====================================================================
        # Gap-closure tests (cover modules previously without dedicated tests)
        # =====================================================================

        # GeofenceConfig::fromNode + applyCoordinateTransform parameter loading.
        # geofence_common provides GeofenceCommonConfig + coordinate_transform +
        # variant_geometry (and URCU/geometry_utils transitively).
        geofence_add_test(test_geofence_config
            SOURCES
                test/test_geofence_config.cpp
                spatial_node/src/node/geofence_config.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS geofence_common
            DEPS rclcpp rclcpp_lifecycle rises_interfaces geometry_msgs
        )

        # ObstacleCorrespondenceMatcher scalar/SIMD parity tests.
        set(CORRESPONDENCE_TEST_SOURCES
            test/test_obstacle_correspondence_matcher.cpp
            spatial_node/src/map/geofence_map.cpp
            spatial_node/src/queries/obstacle_correspondence_matcher.cpp
            spatial_node/src/policies/robot_safety_profile.cpp
            common/src/geometry/variant_geometry.cpp
        )
        if(USE_SIMD)
            list(APPEND CORRESPONDENCE_TEST_SOURCES
                spatial_node/src/queries/obstacle_correspondence_matcher_simd.cpp
            )
        endif()
        ament_add_gtest(test_obstacle_correspondence_matcher
            ${CORRESPONDENCE_TEST_SOURCES}
        )
        target_include_directories(test_obstacle_correspondence_matcher PRIVATE
            ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
        )
        if(USE_SIMD)
            target_compile_definitions(test_obstacle_correspondence_matcher PRIVATE USE_SIMD)
            if(xsimd_FOUND)
                target_link_libraries(test_obstacle_correspondence_matcher xsimd)
            elseif(XSIMD_INCLUDE_DIR)
                target_include_directories(test_obstacle_correspondence_matcher PRIVATE ${XSIMD_INCLUDE_DIR})
            endif()
        endif()
        target_link_libraries(test_obstacle_correspondence_matcher
            ${SELECTED_SPATIAL_BACKEND}
            ${URCU_LIBRARIES}
            geometry_utils
        )
        ament_target_dependencies(test_obstacle_correspondence_matcher
            rclcpp
            rises_interfaces
            test_support
        )

        # RobotTrackingChecker single-threaded correctness tests.
        geofence_add_test(test_robot_tracking_checker
            SOURCES
                test/test_robot_tracking_checker.cpp
                common/src/queries/robot_tracking_checker.cpp
                common/src/policies/robot_tracking.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${URCU_LIBRARIES} geometry_utils
            DEPS rclcpp rises_interfaces geometry_msgs test_support
        )

        # BoundaryChecker containment tests.
        geofence_add_test(test_boundary_checker
            SOURCES
                test/test_boundary_checker.cpp
                spatial_node/src/map/geofence_map.cpp
                spatial_node/src/queries/boundary_checker.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SELECTED_SPATIAL_BACKEND} ${URCU_LIBRARIES} geometry_utils
            DEPS rclcpp rises_interfaces
        )

        # GeofenceVisualizer public-surface smoke tests.
        geofence_add_test(test_visualizer
            SOURCES
                test/test_visualizer.cpp
                spatial_node/src/visualization/geofence_visualizer.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${URCU_LIBRARIES} geometry_utils
            DEPS rclcpp rclcpp_lifecycle rises_interfaces
                 visualization_msgs geometry_msgs nav_msgs test_support
        )

        # GeofenceMap edge cases (empty map, extreme values, dense/overlapping
        # obstacles, rapid add/remove). Was an orphaned, unregistered file;
        # compiles against the current variant API and is registered here.
        geofence_add_test(test_edge_cases_comprehensive
            SOURCES
                test/test_edge_cases_comprehensive.cpp
                spatial_node/src/map/geofence_map.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SELECTED_SPATIAL_BACKEND} ${URCU_LIBRARIES}
            DEPS rclcpp rises_interfaces
        )

        # Obstacle field-semantics contract: POINT reference point is read from
        # .position by intersectsCircle and from .vertices[0] by fromObstacleMsg
        # (deliberately divergent fields). Also covers ObstacleReportBuilder's
        # LINE-type `width` contract (group transverse extent, shared across
        # Douglas-Peucker splits of the same group) -- regression-guards both
        # the builder fix and the original POINT-field divergence.
        geofence_add_test(test_obstacle_contract
            SOURCES
                test/test_obstacle_contract.cpp
                spatial_node/src/node/obstacle_report_builder.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS geometry_utils
            DEPS rclcpp rclcpp_lifecycle rises_interfaces tf2_ros tf2_geometry_msgs test_support
        )

        # LifecycleGeofenceNodeBase configure/activate/deactivate/cleanup cycle,
        # transition idempotency, and the Fix-4 failed-activation recovery path.
        # Same harness/pattern as test_audit_02 / test_audit_03 (no executor).
        geofence_add_test(test_lifecycle_node_base
            SOURCES
                test/test_lifecycle_node_base.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SPATIAL_NODE_NAME} ${URCU_LIBRARIES}
            DEPS rclcpp rclcpp_lifecycle rises_interfaces lifecycle_msgs test_support
        )

        # =====================================================================
        # Regression tests (audit findings -- some RED-on-develop until fixed)
        # =====================================================================

        # Audit #1: ObstacleCollisionChecker inverted return.
        geofence_add_test(test_audit_01_collision_inverted
            SOURCES
                test/regression/test_audit_01_collision_inverted.cpp
                spatial_node/src/map/geofence_map.cpp
                spatial_node/src/queries/obstacle_collision_checker.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SELECTED_SPATIAL_BACKEND} ${URCU_LIBRARIES} geometry_utils
            DEPS rclcpp rises_interfaces test_support
        )

        # Audit #2: JSON load failure must abort lifecycle activation.
        geofence_add_test(test_audit_02_json_load_aborts_activate
            SOURCES
                test/regression/test_audit_02_json_load_aborts_activate.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SPATIAL_NODE_NAME} ${URCU_LIBRARIES}
            DEPS rclcpp rclcpp_lifecycle rises_interfaces lifecycle_msgs test_support
        )

        # Audit #3: invalid obstacle_transform_matrix must fail activation.
        geofence_add_test(test_audit_03_bad_transform_matrix
            SOURCES
                test/regression/test_audit_03_bad_transform_matrix.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SPATIAL_NODE_NAME} ${URCU_LIBRARIES}
            DEPS rclcpp rclcpp_lifecycle rises_interfaces lifecycle_msgs test_support
        )

        # Audit #4: uncapped ObstacleUpdateArray / vertex sums.
        geofence_add_test(test_audit_04_oversize_messages
            SOURCES
                test/regression/test_audit_04_oversize_messages.cpp
                spatial_node/src/map/geofence_map.cpp
                spatial_node/src/node/map_update_handler.cpp
                spatial_node/src/queries/batch_point_checker.cpp
                common/src/policies/coordinate_transform.cpp
                spatial_node/src/policies/robot_safety_profile.cpp
                spatial_node/src/visualization/geofence_visualizer.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SELECTED_SPATIAL_BACKEND} ${URCU_LIBRARIES} geometry_utils
            DEPS rclcpp rclcpp_lifecycle rises_interfaces
                 tf2 tf2_ros tf2_geometry_msgs visualization_msgs test_support
        )

        # Audit #10: pin path-safety semantics against rename-only refactors.
        geofence_add_test(test_audit_10_no_intersection_semantics
            SOURCES
                test/regression/test_audit_10_no_intersection_semantics.cpp
                spatial_node/src/map/geofence_map.cpp
                spatial_node/src/queries/boundary_checker.cpp
                spatial_node/src/queries/path_safety_checker.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
            LIBS ${SELECTED_SPATIAL_BACKEND} ${URCU_LIBRARIES} geometry_utils
            DEPS rclcpp rises_interfaces test_support
        )
    endif()

    # Gridmap tests (only if building gridmap node)
    if(BUILD_NODE STREQUAL "gridmap" OR BUILD_NODE STREQUAL "both")
        # Gridmap include set (no nanoflann / binary-dir; gridmap_node first).
        set(GEOFENCE_GRIDMAP_INCLUDES
            ${CMAKE_CURRENT_SOURCE_DIR}/gridmap_node/include
            ${CMAKE_CURRENT_SOURCE_DIR}/common/include
            ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/include
            ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/geometry/core
            ${CMAKE_CURRENT_SOURCE_DIR}/spatial_node/backend/spatialindex/core
            ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
        )

        geofence_add_test(test_occupancy_grid_map
            SOURCES
                test/test_occupancy_grid_map.cpp
                gridmap_node/src/map/gridmap_data.cpp
                gridmap_node/src/map/gridmap.cpp
                gridmap_node/src/policies/gridmap_policies.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_GRIDMAP_INCLUDES}
            LIBS ${URCU_LIBRARIES}
            DEPS rises_interfaces
        )

        # Audit #5: worldToGrid NaN guard.
        geofence_add_test(test_audit_05_grid_nan_guard
            SOURCES
                test/regression/test_audit_05_grid_nan_guard.cpp
                gridmap_node/src/map/gridmap_data.cpp
                common/src/geometry/variant_geometry.cpp
            INCLUDES ${GEOFENCE_GRIDMAP_INCLUDES}
            LIBS ${URCU_LIBRARIES}
            DEPS rclcpp rises_interfaces test_support
        )
    endif()

    # =====================================================================
    # Phase 3 concurrency tests
    # =====================================================================

    # RcuThreadGuard + AtomicSharedPtr — header-only consumer plus URCU.
    ament_add_gtest(test_rcu_thread_guard
        test/concurrency/test_rcu_thread_guard.cpp
    )
    target_include_directories(test_rcu_thread_guard PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
        ${URCU_INCLUDE_DIRS}
    )
    target_link_libraries(test_rcu_thread_guard
        ${URCU_LIBRARIES}
    )
    if(ARIS_ENABLE_TSAN)
        target_compile_options(test_rcu_thread_guard PRIVATE -fsanitize=thread -O1 -g)
        target_link_options(test_rcu_thread_guard PRIVATE -fsanitize=thread)
    endif()
    set_tests_properties(test_rcu_thread_guard PROPERTIES TIMEOUT 30)

    # RobotTrackingChecker robot_poses_ / footprints_ — spatial-node only.
    if(BUILD_NODE STREQUAL "spatial" OR BUILD_NODE STREQUAL "both")
        ament_add_gtest(test_robot_tracker_race
            test/concurrency/test_robot_tracker_race.cpp
            common/src/queries/robot_tracking_checker.cpp
            common/src/policies/robot_tracking.cpp
            spatial_node/src/map/geofence_map.cpp
            common/src/geometry/variant_geometry.cpp
        )
        target_include_directories(test_robot_tracker_race PRIVATE
            ${GEOFENCE_REGRESSION_SPATIAL_INCLUDES}
        )
        target_link_libraries(test_robot_tracker_race
            ${URCU_LIBRARIES}
            geometry_utils
        )
        ament_target_dependencies(test_robot_tracker_race
            rclcpp
            rises_interfaces
            geometry_msgs
            test_support
        )
        if(ARIS_ENABLE_TSAN)
            target_compile_options(test_robot_tracker_race PRIVATE -fsanitize=thread -O1 -g)
            target_link_options(test_robot_tracker_race PRIVATE -fsanitize=thread)
        endif()
        set_tests_properties(test_robot_tracker_race PROPERTIES TIMEOUT 30)
    endif()
endif()

ament_package()
