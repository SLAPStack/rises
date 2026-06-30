# FindXSIMD.cmake
# Finds xsimd library with FetchContent fallback
#
# Sets:
#   xsimd_FOUND - TRUE if found
#   XSIMD_INCLUDE_DIR - Include directory (for header-only usage)

include(FindPackageHandleStandardArgs)

# Try standard find_package first
find_package(xsimd QUIET CONFIG)

if(NOT xsimd_FOUND)
    message(STATUS "xsimd not found via find_package, checking for header...")
    
    # Try to find header manually. Honor CMAKE_PREFIX_PATH (e.g. overlay
    # installs / custom prefixes) in addition to the standard system dirs.
    find_path(XSIMD_INCLUDE_DIR xsimd/xsimd.hpp
        PATHS /usr/include /usr/local/include ${CMAKE_PREFIX_PATH}
    )
    
    if(XSIMD_INCLUDE_DIR)
        set(xsimd_FOUND TRUE)
        message(STATUS "Found xsimd header: ${XSIMD_INCLUDE_DIR}")
    else()
        # Fetch from GitHub as last resort
        message(STATUS "xsimd not found on system, fetching from GitHub...")
        include(FetchContent)
        FetchContent_Declare(
            xsimd
            GIT_REPOSITORY https://github.com/xtensor-stack/xsimd.git
            GIT_TAG 11.1.0
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(xsimd)
        set(xsimd_FOUND TRUE)
        set(XSIMD_INCLUDE_DIR ${xsimd_SOURCE_DIR}/include)
        message(STATUS "xsimd fetched and configured")
    endif()
else()
    message(STATUS "Found xsimd package")
endif()

find_package_handle_standard_args(XSIMD
    REQUIRED_VARS xsimd_FOUND
)

mark_as_advanced(XSIMD_INCLUDE_DIR)
