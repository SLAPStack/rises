# FindNanoflann.cmake
# Finds nanoflann header-only library with FetchContent fallback
#
# Sets:
#   Nanoflann_FOUND - TRUE if found
#   Creates target: nanoflann::nanoflann

include(FindPackageHandleStandardArgs)

# Try standard find_package first
find_package(nanoflann QUIET CONFIG)

if(NOT nanoflann_FOUND)
    message(STATUS "nanoflann not found via find_package, checking for header...")
    
    # Try to find header manually
    find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp
        PATHS /usr/include /usr/local/include
    )
    
    if(NANOFLANN_INCLUDE_DIR)
        # Create imported target manually
        if(NOT TARGET nanoflann::nanoflann)
            add_library(nanoflann::nanoflann INTERFACE IMPORTED)
            set_target_properties(nanoflann::nanoflann PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${NANOFLANN_INCLUDE_DIR}"
            )
        endif()
        set(Nanoflann_FOUND TRUE)
        message(STATUS "Found nanoflann header: ${NANOFLANN_INCLUDE_DIR}")
    else()
        # Fetch from GitHub as last resort
        message(STATUS "nanoflann not found on system, fetching from GitHub...")
        include(FetchContent)
        FetchContent_Declare(
            nanoflann
            GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
            GIT_TAG v1.5.0
            GIT_SHALLOW TRUE
        )
        
        # Configure nanoflann options
        set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "Disable nanoflann examples" FORCE)
        set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "Disable nanoflann tests" FORCE)
        
        FetchContent_MakeAvailable(nanoflann)
        
        set(Nanoflann_FOUND TRUE)
        message(STATUS "nanoflann fetched and configured from GitHub")
    endif()
else()
    set(Nanoflann_FOUND TRUE)
    message(STATUS "Found nanoflann via CONFIG")
endif()

find_package_handle_standard_args(Nanoflann
    REQUIRED_VARS Nanoflann_FOUND
)
