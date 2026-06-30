# FindTBB.cmake
# Finds Intel Threading Building Blocks (TBB) library with FetchContent fallback
#
# Sets:
#   TBB_FOUND - TRUE if found
#   Creates target: TBB::tbb

include(FindPackageHandleStandardArgs)

# Try standard find_package first (TBB provides TBBConfig.cmake)
find_package(TBB QUIET CONFIG COMPONENTS tbb)

if(NOT TBB_FOUND)
    message(STATUS "TBB not found via find_package, trying pkg-config...")
    
    # Try pkg-config
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(TBB QUIET tbb)
        
        if(TBB_FOUND)
            # Create imported target from pkg-config info
            if(NOT TARGET TBB::tbb)
                add_library(TBB::tbb INTERFACE IMPORTED)
                set_target_properties(TBB::tbb PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES "${TBB_INCLUDE_DIRS}"
                    INTERFACE_LINK_LIBRARIES "${TBB_LINK_LIBRARIES}"
                    INTERFACE_COMPILE_OPTIONS "${TBB_CFLAGS_OTHER}"
                )
            endif()
            message(STATUS "Found TBB via pkg-config: ${TBB_VERSION}")
            return()
        endif()
    endif()
    
    # Try manual header/library search
    find_path(TBB_INCLUDE_DIR tbb/tbb.h
        PATHS /usr/include /usr/local/include
    )
    
    find_library(TBB_LIBRARY
        NAMES tbb
        PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
    )
    
    if(TBB_INCLUDE_DIR AND TBB_LIBRARY)
        # Create imported target manually
        if(NOT TARGET TBB::tbb)
            add_library(TBB::tbb INTERFACE IMPORTED)
            set_target_properties(TBB::tbb PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${TBB_INCLUDE_DIR}"
                INTERFACE_LINK_LIBRARIES "${TBB_LIBRARY}"
            )
        endif()
        set(TBB_FOUND TRUE)
        message(STATUS "Found TBB manually: ${TBB_LIBRARY}")
    else()
        # Fetch from GitHub as last resort
        message(STATUS "TBB not found on system, fetching from GitHub (oneAPI)...")
        include(FetchContent)
        
        FetchContent_Declare(
            tbb
            GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
            GIT_TAG v2021.11.0
            GIT_SHALLOW TRUE
        )
        
        # Configure TBB options
        set(TBB_TEST OFF CACHE BOOL "Disable TBB tests" FORCE)
        set(TBB_EXAMPLES OFF CACHE BOOL "Disable TBB examples" FORCE)
        set(TBB_STRICT OFF CACHE BOOL "Disable strict mode" FORCE)
        
        FetchContent_MakeAvailable(tbb)
        
        set(TBB_FOUND TRUE)
        message(STATUS "TBB fetched and configured from GitHub")
    endif()
endif()

find_package_handle_standard_args(TBB
    REQUIRED_VARS TBB_FOUND
)
