# FindEigen3Custom.cmake
# Finds Eigen3 library with FetchContent fallback
#
# Sets:
#   Eigen3_FOUND - TRUE if found
#   Creates target: Eigen3::Eigen

include(FindPackageHandleStandardArgs)

# Try standard find_package first
find_package(Eigen3 QUIET CONFIG)

if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found via find_package, checking for header...")
    
    # Try to find header manually. Honor CMAKE_PREFIX_PATH (e.g. overlay
    # installs / custom prefixes) in addition to the standard system dirs.
    find_path(EIGEN3_INCLUDE_DIR Eigen/Core
        PATHS /usr/include /usr/local/include ${CMAKE_PREFIX_PATH}
        PATH_SUFFIXES eigen3
    )
    
    if(EIGEN3_INCLUDE_DIR)
        # Create imported target manually
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
        )
        set(Eigen3_FOUND TRUE)
        set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
        message(STATUS "Found Eigen3 header: ${EIGEN3_INCLUDE_DIR}")
    else()
        # Fetch from GitLab as last resort
        message(STATUS "Eigen3 not found on system, fetching from GitLab...")
        include(FetchContent)
        FetchContent_Declare(
            eigen3
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4.0
            GIT_SHALLOW TRUE
        )
        
        # Configure Eigen3 options
        set(BUILD_TESTING OFF CACHE BOOL "Disable Eigen3 tests" FORCE)
        set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen3 docs" FORCE)
        set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Disable Eigen3 pkgconfig" FORCE)
        
        FetchContent_MakeAvailable(eigen3)
        
        set(Eigen3_FOUND TRUE)
        set(EIGEN3_INCLUDE_DIRS ${eigen3_SOURCE_DIR})
        message(STATUS "Eigen3 fetched and configured")
    endif()
else()
    message(STATUS "Found Eigen3 package")
    # Get include dirs if available
    get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
endif()

find_package_handle_standard_args(Eigen3Custom
    REQUIRED_VARS Eigen3_FOUND
)

mark_as_advanced(EIGEN3_INCLUDE_DIR EIGEN3_INCLUDE_DIRS)
