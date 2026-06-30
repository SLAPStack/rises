# ==============================
# Multi-stage Central Bridge Container (Rosbag Mode)
# ==============================
# Centralized container for rosbag-based simulations
# Launches: centralized translator + rosbag playback
# 
# Build: docker build -f central.dockerfile -t rises:central .

# ============================================================
# Stage 1: Base ROS2 build
# ============================================================
FROM eprosima/vulcanexus:jazzy-base AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Build / Compiler Options
ARG CC=clang
ARG CXX=clang++
ARG LINKER=lld
ARG CXX_STD=17
ARG BUILD_TYPE=Release
ARG CMAKE_GENERATOR=Ninja
ARG OPT_LEVEL=-O3
ARG LTO=ON
ARG LTO_FLAG="-flto=thin"
ARG BUILD_JOBS=0
ARG TARGET_ARCH="-march=native"

# Environment
ENV LANG=en_US.UTF-8 \
  LC_ALL=en_US.UTF-8 \
  CC=${CC} \
  CXX=${CXX} \
  LINKER=${LINKER} \
  CXX_STD=${CXX_STD} \
  OPT_LEVEL=${OPT_LEVEL} \
  LTO=${LTO} \
  LTO_FLAG=${LTO_FLAG} \
  BUILD_TYPE=${BUILD_TYPE} \
  CMAKE_GENERATOR=${CMAKE_GENERATOR} \
  BUILD_JOBS=${BUILD_JOBS} \
  TARGET_ARCH=${TARGET_ARCH}

# System dependencies
# The eprosima/vulcanexus base image ships an apt source for repo.vulcanexus.org
# whose 'noble' suite currently returns 404, which aborts `apt-get update`.
# Vulcanexus is already installed in the base image and none of the packages below
# come from that repo, so drop the dead source before updating.
RUN (find /etc/apt -type f \( -name '*.list' -o -name '*.sources' \) \
      -exec grep -iq vulcanexus {} \; -delete 2>/dev/null || true); \
  apt-get update && apt-get install -y --no-install-recommends \
  locales \
  curl \
  gnupg2 \
  lsb-release \
  software-properties-common \
  build-essential \
  cmake \
  clang \
  lld \
  ninja-build \
  git \
  python3-pip \
  nlohmann-json3-dev \
  libpaho-mqtt-dev \
  libpaho-mqttpp-dev \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-rviz2 \
  libx11-xcb1 \
  libxrender1 \
  libxcb1 \
  libxcb-icccm4 \
  libxcb-image0 \
  libxcb-keysyms1 \
  libxcb-randr0 \
  libxcb-render0 \
  libxcb-shm0 \
  libxcb-xfixes0 \
  libxcb-xinerama0 \
  libxkbcommon-x11-0 \
  libgl1 \
  libgl1-mesa-dri \
  libegl1 \
  mesa-utils \
  qtbase5-dev \
  qtbase5-dev-tools \
  x11-apps \
  openbox \
  dos2unix \
  python3-colcon-common-extensions \
  python3-rosdep \
  clangd \
  gdb liburcu-dev liburcu8 \
  libboost-all-dev \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

# ROS tools (colcon already installed via apt)
RUN (rosdep init 2>/dev/null || true) && rosdep update

WORKDIR /workspace
RUN mkdir -p src

# Copy packages
COPY src/ src/

# Copy resources
COPY resources/ /workspace/resources/

# Build workspace with dynamic BUILD_JOBS
RUN source /opt/ros/jazzy/setup.bash && \
  apt-get update && \
  rosdep install --from-paths src --ignore-src -r -y && \
  rm -rf /var/lib/apt/lists/* && \
  BUILD_JOBS=$(if [ -z "${BUILD_JOBS}" ] || [ "${BUILD_JOBS}" -le 0 ]; then nproc; else echo "${BUILD_JOBS}"; fi) && \
  # Stage 1: Build interface packages first (dependencies for everything else)
  colcon build --symlink-install --event-handlers console_direct+ --parallel-workers $BUILD_JOBS \
    --packages-select rises_interfaces && \
  source install/setup.bash && \
  # Stage 2: Build remaining packages (explicit flags; presets are dev-only clang/ninja)
  colcon build --symlink-install --event-handlers console_direct+ --parallel-workers $BUILD_JOBS \
    --packages-skip rises_interfaces \
    --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DLASER_COUNT=1 -DUSE_SIMD=OFF -DBUILD_TESTING=OFF

# Source ROS setup in shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
  echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Set environment defaults
ENV QT_X11_NO_MITSHM=1
ENV PATH="/usr/bin:${PATH}"

COPY entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# ============================================================
# Stage 2: Central-specific configuration
# ============================================================
FROM base AS central

# Copy central entrypoint
COPY central_entrypoint.sh /central_entrypoint.sh
RUN dos2unix /central_entrypoint.sh && chmod +x /central_entrypoint.sh

# Copy RViz configuration
COPY resources/rises.rviz /root/.rviz/rises.rviz

# Expose common ROS ports (DDS discovery)
EXPOSE 11311 7400-7500

ENTRYPOINT ["/central_entrypoint.sh"]

# ============================================================
# Stage 3: Unity-specific configuration
# ============================================================
FROM base AS unity

# Copy unity entrypoint
COPY unity_entrypoint.sh /unity_entrypoint.sh
RUN dos2unix /unity_entrypoint.sh && chmod +x /unity_entrypoint.sh

# Copy RViz configuration
COPY resources/rises.rviz /root/.rviz/rises.rviz

# Expose ROS-TCP-Endpoint port + DDS discovery
EXPOSE 10000 11311 7400-7500

ENTRYPOINT ["/unity_entrypoint.sh"]
