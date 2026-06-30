#pragma once

/**
 * @file collision_checker_selector.hpp
 * @brief Compile-time selection of collision checker implementation
 * 
 * This header provides type aliases for collision checking based on CMake build options.
 * 
 * CMake usage:
 *   if(USE_SIMD)
 *     target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SIMD)
 *   endif()
 * 
 * C++ usage:
 *   #include "geofence/spatial/policies/collision_checker_selector.hpp"
 *   
 *   // Use the selected implementation transparently
 *   rises::geofence::CollisionChecker::initialize(config);
 *   bool collides = rises::geofence::CollisionChecker::checkCollision(...);
 */

#ifdef USE_SIMD
    #include "geofence/spatial/queries/obstacle_collision_checker_simd.hpp"
    namespace rises::geofence {
        using CollisionChecker = ObstacleCollisionCheckerSIMD;
    }
    #define COLLISION_CHECKER_IMPL "SIMD"
#else
    #include "geofence/spatial/queries/obstacle_collision_checker.hpp"
    namespace rises::geofence {
        using CollisionChecker = ObstacleCollisionChecker;
    }
    #define COLLISION_CHECKER_IMPL "Standard"
#endif
