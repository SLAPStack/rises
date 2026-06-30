#pragma once

/**
 * @file correspondence_matcher_selector.hpp
 * @brief Compile-time selection of correspondence matcher implementation
 * 
 * This header provides type aliases for obstacle correspondence matching
 * based on CMake build options.
 * 
 * CMake usage:
 *   if(USE_SIMD)
 *     target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SIMD)
 *   endif()
 * 
 * C++ usage:
 *   #include "geofence/spatial/policies/correspondence_matcher_selector.hpp"
 *   
 *   // Use the selected implementation transparently
 *   rises::geofence::CorrespondenceMatcher::initialize(config);
 *   auto result = rises::geofence::CorrespondenceMatcher::findCorrespondingObstacle(...);
 */

#ifdef USE_SIMD
    #include "geofence/spatial/queries/obstacle_correspondence_matcher_simd.hpp"
    namespace rises::geofence {
        using CorrespondenceMatcher = ObstacleCorrespondenceMatcherSIMD;
    }
    #define CORRESPONDENCE_MATCHER_IMPL "SIMD"
#else
    #include "geofence/spatial/queries/obstacle_correspondence_matcher.hpp"
    namespace rises::geofence {
        using CorrespondenceMatcher = ObstacleCorrespondenceMatcher;
    }
    #define CORRESPONDENCE_MATCHER_IMPL "Standard"
#endif
