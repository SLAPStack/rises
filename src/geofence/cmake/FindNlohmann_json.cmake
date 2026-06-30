# FindNlohmann_json.cmake
# Finds nlohmann-json library with FetchContent fallback
#
# Sets:
#   nlohmann_json_FOUND - TRUE if found
#   Creates target: nlohmann_json::nlohmann_json

include(FindPackageHandleStandardArgs)

# Try standard find_package first (looks for nlohmann_jsonConfig.cmake)
find_package(nlohmann_json QUIET CONFIG)

if(NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json not found via find_package, checking for header...")
    
    # Try to find header manually. Honor CMAKE_PREFIX_PATH (e.g. overlay
    # installs / custom prefixes) in addition to the standard system dirs.
    find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
        PATHS /usr/include /usr/local/include ${CMAKE_PREFIX_PATH}
    )
    
    if(NLOHMANN_JSON_INCLUDE_DIR)
        # Create imported target manually
        add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
        set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_INCLUDE_DIR}"
        )
        set(nlohmann_json_FOUND TRUE)
        message(STATUS "Found nlohmann_json header: ${NLOHMANN_JSON_INCLUDE_DIR}")
    else()
        # Fetch from GitHub as last resort
        message(STATUS "nlohmann_json not found on system, fetching from GitHub...")
        include(FetchContent)
        FetchContent_Declare(
            nlohmann_json
            GIT_REPOSITORY https://github.com/nlohmann/json.git
            GIT_TAG v3.11.3
            GIT_SHALLOW TRUE
        )
        
        # Configure nlohmann_json options
        set(JSON_BuildTests OFF CACHE BOOL "Disable nlohmann_json tests" FORCE)
        set(JSON_Install OFF CACHE BOOL "Disable nlohmann_json install" FORCE)
        
        FetchContent_MakeAvailable(nlohmann_json)
        
        set(nlohmann_json_FOUND TRUE)
        message(STATUS "nlohmann_json fetched and configured")
    endif()
else()
    message(STATUS "Found nlohmann_json package")
endif()

find_package_handle_standard_args(Nlohmann_json
    REQUIRED_VARS nlohmann_json_FOUND
)

mark_as_advanced(NLOHMANN_JSON_INCLUDE_DIR)
