cmake_minimum_required(VERSION 3.16)

project(datastructure_backend_nanoflann LANGUAGES CXX)

# Use the parent's cmake module path
list(APPEND CMAKE_MODULE_PATH "${GEOFENCE_CMAKE_MODULE_PATH}")

# Find nanoflann using the custom module (with FetchContent fallback)
find_package(Nanoflann REQUIRED)

# Header-only adapter library
add_library(datastructure_backend_nanoflann INTERFACE)

# Include directories
target_include_directories(datastructure_backend_nanoflann
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Link nanoflann if it provides a target, otherwise use include dir
if(TARGET nanoflann::nanoflann)
    target_link_libraries(datastructure_backend_nanoflann
        INTERFACE
        nanoflann::nanoflann
    )
elseif(NANOFLANN_INCLUDE_DIR)
    target_include_directories(datastructure_backend_nanoflann
        INTERFACE
        ${NANOFLANN_INCLUDE_DIR}
    )
endif()

# Alias for consistent naming
add_library(datastructure_backend ALIAS datastructure_backend_nanoflann)
