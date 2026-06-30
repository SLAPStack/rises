# Builds the eProsima FIWARE DDS Enabler v1.0.0 on Vulcanexus Jazzy.
# Uses the same Fast-DDS 3.x as the AGV containers so DDS type discovery
# works automatically — no XML type definitions needed.
#
# Two fixes applied over a vanilla v1.0.0 build:
#  1. ddsenabler.repos pins DDS-Pipe v1.3.0 but DDS-Enabler v1.0.0 code calls
#     APIs (create_reader_with_filter, add_topic_partition, ...) that were added
#     in DDS-Pipe v1.4.0. Override to v1.4.0 after vcs import.
#  2. fastdds (3.4.2) and fastcdr (2.3.5) live under /opt/vulcanexus/jazzy, not
#     /opt/ros/jazzy. Colcon's cmake prefix path misses them, so pin both _DIR
#     explicitly to bypass find_package.

FROM eprosima/vulcanexus:jazzy-base AS builder

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake g++ make ninja-build \
    libcurl4-openssl-dev libjsoncpp-dev libyaml-cpp-dev \
    libssl-dev libasio-dev libtinyxml2-dev \
    python3-vcstool python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ros-jazzy-fastcdr ships fastcdr 2.2.5 under /opt/ros/jazzy/lib.
# Vulcanexus ships fastcdr 2.3.5 under /opt/vulcanexus/jazzy/lib.
# libfastdds.so.3.4.2.0 was compiled against 2.3.5 and requires its newer API.
# cmake's -L search hits /opt/ros/jazzy/lib before /opt/vulcanexus/jazzy/lib
# regardless of LIBRARY_PATH or cmake _DIR hints, so the linker picks 2.2.5
# and fails with undefined reference errors.
#
# Fix: redirect all ros-jazzy fastcdr .so symlinks to the vulcanexus 2.3.5
# binary. The ros-jazzy cmake config files remain intact (fastcdr-shared-
# targets.cmake checks the IMPORTED_LOCATION path still exists), but the
# linker always gets the 2.3.5 binary when it follows any of these symlinks.
# This is safe in the builder stage — the runtime stage is a separate image.
RUN ln -sf /opt/vulcanexus/jazzy/lib/libfastcdr.so.2.3.5 /opt/ros/jazzy/lib/libfastcdr.so.2.2.5 && \
    ln -sf /opt/vulcanexus/jazzy/lib/libfastcdr.so.2.3.5 /opt/ros/jazzy/lib/libfastcdr.so.2 && \
    ln -sf /opt/vulcanexus/jazzy/lib/libfastcdr.so.2.3.5 /opt/ros/jazzy/lib/libfastcdr.so

WORKDIR /ws

# Clone DDS Enabler v1.0.0 and its dependencies (DDS-Pipe, Dev-Utils).
# The ddsenabler.repos pins DDS-Pipe v1.3.0 which is missing APIs used by
# DDS-Enabler v1.0.0. Override to v1.4.0 immediately after import.
RUN mkdir -p src && \
    git clone --recursive --branch v1.0.0 \
      https://github.com/eProsima/FIWARE-DDS-Enabler.git src/fiware_dds_enabler && \
    vcs import src < src/fiware_dds_enabler/ddsenabler.repos && \
    git -C src/DDS-Pipe fetch --tags && \
    git -C src/DDS-Pipe checkout v1.4.0

# Remove Fast-DDS, Fast-CDR, foonathan_memory from source — use Vulcanexus system versions.
# This ensures the enabler links against the SAME Fast-DDS as the ROS 2 nodes.
RUN rm -rf src/Fast-DDS src/Fast-CDR src/foonathan_memory_vendor

# Symlink only the enabler packages and their direct dependencies.
RUN mkdir -p src/colcon_pkgs && \
    ln -s /ws/src/Dev-Utils/cmake_utils src/colcon_pkgs/cmake_utils && \
    ln -s /ws/src/Dev-Utils/cpp_utils src/colcon_pkgs/cpp_utils && \
    ln -s /ws/src/DDS-Pipe/ddspipe_core src/colcon_pkgs/ddspipe_core && \
    ln -s /ws/src/DDS-Pipe/ddspipe_participants src/colcon_pkgs/ddspipe_participants && \
    ln -s /ws/src/DDS-Pipe/ddspipe_yaml src/colcon_pkgs/ddspipe_yaml && \
    ln -s /ws/src/DDS-Enabler/ddsenabler src/colcon_pkgs/ddsenabler && \
    ln -s /ws/src/DDS-Enabler/ddsenabler_participants src/colcon_pkgs/ddsenabler_participants && \
    ln -s /ws/src/DDS-Enabler/ddsenabler_yaml src/colcon_pkgs/ddsenabler_yaml

# Build against Vulcanexus's system Fast-DDS and Fast-CDR.
# Both live under /opt/vulcanexus/jazzy, not /opt/ros/jazzy.
#
# Two problems:
#   1. Colcon's cmake prefix path drops /opt/vulcanexus/jazzy → find_package
#      picks wrong versions. Fixed by pinning *_DIR explicitly.
#   2. libfastdds.so.3.4.2.0 has a transitive SONAME dep on libfastcdr.so.2.
#      The linker resolves it from the first matching lib in its search path.
#      /opt/ros/jazzy/lib/libfastcdr.so.2.2.5 is found before the vulcanexus
#      2.3.5 version → undefined reference errors. Fixed by prepending
#      /opt/vulcanexus/jazzy/lib to LIBRARY_PATH (the linker's search env var,
#      distinct from LD_LIBRARY_PATH which is only for the runtime loader).
RUN source /opt/ros/jazzy/setup.bash && \
    LIBRARY_PATH=/opt/vulcanexus/jazzy/lib${LIBRARY_PATH:+:$LIBRARY_PATH} \
    colcon build \
      --paths src/colcon_pkgs/* \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCOMPILE_EXAMPLES=ON \
        -Dfastdds_DIR=/opt/vulcanexus/jazzy/share/fastdds/cmake \
        -Dfastcdr_DIR=/opt/vulcanexus/jazzy/share/fastcdr/cmake \
      --event-handlers console_cohesion+ \
      --parallel-workers $(nproc)

# Runtime — use Vulcanexus base for matching Fast-DDS libraries
FROM eprosima/vulcanexus:jazzy-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    libjsoncpp25 libyaml-cpp0.8 libcurl4t64 \
    && rm -rf /var/lib/apt/lists/*

# Copy ONLY enabler packages (Fast-DDS/Fast-CDR come from Vulcanexus system)
COPY --from=builder /ws/install/ /opt/ddsenabler/

# Enabler libs + Vulcanexus system Fast-DDS / Fast-CDR.
# $LD_LIBRARY_PATH expands to empty here (vulcanexus:jazzy-base sets the
# lib path via ldconfig, not as an env var), so list every required path
# explicitly rather than relying on the append.
ENV LD_LIBRARY_PATH=/opt/vulcanexus/jazzy/lib:\
/opt/ddsenabler/ddsenabler/lib:\
/opt/ddsenabler/ddsenabler_participants/lib:\
/opt/ddsenabler/ddsenabler_yaml/lib:\
/opt/ddsenabler/ddspipe_core/lib:\
/opt/ddsenabler/ddspipe_participants/lib:\
/opt/ddsenabler/ddspipe_yaml/lib:\
/opt/ddsenabler/cpp_utils/lib:\
/opt/ddsenabler/cmake_utils/lib

# Symlink the executable
RUN mkdir -p /usr/local/bin && \
    ln -sf $(find /opt/ddsenabler -name 'ddsenabler_example' -type f -executable | head -1) /usr/local/bin/ddsenabler

ENV PATH=/usr/local/bin:$PATH

ENTRYPOINT ["ddsenabler"]
CMD ["--config", "/config/dds-enabler.json"]
