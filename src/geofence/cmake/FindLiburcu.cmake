# FindLiburcu.cmake
# Finds liburcu (userspace RCU) library, memb flavor.
#
# liburcu ships multiple flavors as separate .so files:
#   liburcu        — memb   (default; balanced read/write cost)
#   liburcu-mb     — explicit memory barriers (faster reads, slower writes)
#   liburcu-bp     — bulletproof (no register required, slowest reads)
#   liburcu-qsbr   — quiescent-state-based (fastest reads, requires
#                    periodic rcu_quiescent_state() calls)
#   liburcu-signal — signal-based (legacy)
#
# Mixing flavors in one process is UNDEFINED BEHAVIOUR. This project pins
# to MEMB because:
#   - matches `#include <urcu.h>` in geofence/utils/rcu.hpp (the default
#     header maps to memb)
#   - balanced read/write cost suitable for the geofence map use case
#   - does not require periodic rcu_quiescent_state() calls (qsbr does;
#     poor fit for the rclcpp executor model)
#   - allows unregistered writer threads to call synchronize_rcu()
#
# To switch flavor, change BOTH:
#   1. the include in rcu.hpp (e.g. `#include <urcu/urcu-mb.h>`)
#   2. the library name below (e.g. `urcu-mb`)
# in lockstep. Failing to do so is UB.
#
# Reader threads MUST call rcu_register_thread() once before any
# rcu_read_lock(). The project's RcuThreadGuard (in rcu.hpp) handles this
# automatically via a thread_local; do not call rcu_register_thread()
# manually elsewhere.
#
# Sets:
#   URCU_FOUND        - TRUE if found
#   URCU_LIBRARIES    - libraries to link (memb + common)
#   URCU_INCLUDE_DIRS - include directories

include(FindPackageHandleStandardArgs)

# pkg-config first — liburcu installs liburcu.pc for the memb flavor.
find_package(PkgConfig)
if(PkgConfig_FOUND)
    pkg_check_modules(URCU liburcu)
endif()

if(NOT URCU_FOUND)
    message(STATUS "liburcu not found via pkg-config, falling back to find_library")

    find_library(URCU_LIB NAMES urcu)
    find_path(URCU_INCLUDE_DIR urcu.h)

    if(URCU_LIB AND URCU_INCLUDE_DIR)
        set(URCU_LIBRARIES ${URCU_LIB})
        set(URCU_INCLUDE_DIRS ${URCU_INCLUDE_DIR})
        set(URCU_FOUND TRUE)
        message(STATUS "Found liburcu: ${URCU_LIB}")
    else()
        message(FATAL_ERROR
            "liburcu (memb flavor) not found.\n"
            "Install:\n"
            "  Ubuntu/Debian: sudo apt install liburcu-dev\n"
            "  Fedora:        sudo dnf install userspace-rcu-devel\n"
            "  Arch:          sudo pacman -S userspace-rcu\n"
            "liburcu uses autotools (not CMake) and cannot be FetchContent'd."
        )
    endif()
else()
    message(STATUS "Found liburcu via pkg-config: ${URCU_LIBRARIES}")
endif()

# liburcu-common provides shared runtime symbols (call_rcu worker thread,
# defer_rcu, urcu_die, etc.) used by every flavor. pkg-config usually
# pulls it transitively, but custom packaging (Buildroot, Yocto, manual
# autotools install) sometimes does not — link explicitly when present.
find_library(URCU_COMMON_LIB NAMES urcu-common)
if(URCU_COMMON_LIB)
    list(APPEND URCU_LIBRARIES ${URCU_COMMON_LIB})
    message(STATUS "Linking liburcu-common: ${URCU_COMMON_LIB}")
endif()

# Sanity check: warn if a non-memb flavor library is also discoverable.
# The linker will not error on this, but if anything in the dependency
# graph links a different flavor in the same process, behaviour is UB.
foreach(_other_flavor mb bp qsbr signal)
    find_library(_URCU_${_other_flavor}_LIB NAMES urcu-${_other_flavor})
    if(_URCU_${_other_flavor}_LIB)
        message(STATUS
            "(info) liburcu-${_other_flavor} also installed at "
            "${_URCU_${_other_flavor}_LIB}. The geofence build links memb; "
            "do NOT also link liburcu-${_other_flavor} into the same process."
        )
    endif()
    mark_as_advanced(_URCU_${_other_flavor}_LIB)
endforeach()

find_package_handle_standard_args(Liburcu
    REQUIRED_VARS URCU_LIBRARIES URCU_INCLUDE_DIRS
)

mark_as_advanced(URCU_LIBRARIES URCU_INCLUDE_DIRS URCU_COMMON_LIB)
