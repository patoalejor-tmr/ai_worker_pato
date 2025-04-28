ARG TARGETARCH

# Base images for different architectures
FROM ros:jazzy-ros-base AS base-arm64
FROM robotis/ros:jazzy-base-torch2.7.0 AS base-amd64

# Select the appropriate base image based on TARGETARCH
FROM base-${TARGETARCH} AS base

#################################
#   Librealsense Builder Stage  #
#################################
FROM ubuntu:24.04 AS librealsense-builder

ARG LIBRS_VERSION=2.56.3
# Make sure that we have a version number of librealsense as argument
RUN test -n "$LIBRS_VERSION"

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Builder dependencies installation
RUN apt-get update || true \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    python3-pip \
    libpython3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION -o librealsense.tar.gz
RUN tar -zxf librealsense.tar.gz \
    && rm librealsense.tar.gz
RUN ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense

# Build and install
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
    -DBUILD_GRAPHICAL_EXAMPLES=TRUE \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    -DFORCE_RSUSB_BACKEND=TRUE \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && make -j$(($(nproc)-1)) all \
    && make install

######################################
#   librealsense Base Image Stage    #
######################################
FROM base AS librealsense

# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
# COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install realsense-viewer dependencies
RUN apt-get update && apt-get install -y \
    libusb-1.0-0-dev \
    libglfw3-dev \
    && rm -rf /var/lib/apt/lists/*

# Define build argument for Gazebo support
ARG WITH_GAZEBO=true

# Install common system dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pal-statistics \
    vim \
    nano \
    && apt-get install -y ros-${ROS_DISTRO}-moveit --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo dependencies conditionally
RUN if [ "$WITH_GAZEBO" = "true" ]; then \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-gz* \
    && rm -rf /var/lib/apt/lists/*; \
    fi

ENV COLCON_WS=/root/colcon_ws
WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b feature-ai-worker https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b feature-refactoring https://github.com/ROBOTIS-GIT/physical_ai_tools.git --recursive && \
    git clone -b feature-hardware-revision-3 https://github.com/ROBOTIS-GIT/ai_worker.git

# Clone gz_ros2_control only if WITH_GAZEBO is true
RUN if [ "$WITH_GAZEBO" = "true" ]; then \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ros-controls/gz_ros2_control; \
    fi

# Install Python packages and ffmpeg only for amd64
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    pip install torch cmake datasets deepdiff diffusers draccus einops flask gdown gymnasium h5py huggingface-hub imageio jsonlines numba omegaconf opencv-python-headless packaging av pymunk pynput pyzmq rerun-sdk termcolor torchvision wandb zarr && \
    cd ${COLCON_WS}/src/physical_ai_tools/lerobot && \
    pip install -e . --ignore-installed packaging --no-deps && \
    apt-get update && \
    apt-get install -y ffmpeg && \
    rm -rf /var/lib/apt/lists/*; \
    fi

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
