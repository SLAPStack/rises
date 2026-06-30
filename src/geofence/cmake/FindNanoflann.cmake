# FindNanoflann.cmake
# Finds nanoflann library with FetchContent fallback
#
# Sets:
#   nanoflann_FOUND - TRUE if found
#   NANOFLANN_INCLUDE_DIR - Include directory (for header-only usage)

include(FindPackageHandleStandardArgs)

# Try standard find_package first
find_package(nanoflann QUIET CONFIG)

if(NOT nanoflann_FOUND)
    message(STATUS "nanoflann not found via find_package, checking for header...")
    
    # Try to find header manually. Honor CMAKE_PREFIX_PATH (e.g. overlay
    # installs / custom prefixes) in addition to the standard system dirs.
    find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp
        PATHS /usr/include /usr/local/include ${CMAKE_PREFIX_PATH}
    )
    
    if(NANOFLANN_INCLUDE_DIR)
        set(nanoflann_FOUND TRUE)
        message(STATUS "Found nanoflann header: ${NANOFLANN_INCLUDE_DIR}")
    else()
        # Fetch from GitHub as last resort
        message(STATUS "nanoflann not found on system, fetching from GitHub...")
        include(FetchContent)
        FetchContent_Declare(
            nanoflann
            GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
            GIT_TAG v1.5.5
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(nanoflann)
        set(nanoflann_FOUND TRUE)
        set(NANOFLANN_INCLUDE_DIR ${nanoflann_SOURCE_DIR}/include)
        message(STATUS "nanoflann fetched and configured")
    endif()
else()
    message(STATUS "Found nanoflann package")
endif()

find_package_handle_standard_args(Nanoflann
    REQUIRED_VARS nanoflann_FOUND
)

mark_as_advanced(NANOFLANN_INCLUDE_DIR)
