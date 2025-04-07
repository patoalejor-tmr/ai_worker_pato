# Base image
ARG BASE_IMAGE=ros:jazzy-ros-base
#################################
#   Librealsense Builder Stage  #
#################################
FROM ubuntu:22.04 AS librealsense-builder

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
FROM ${BASE_IMAGE} AS librealsense

# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Install dep packages
RUN apt-get update || true \
    && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_HOME=/root/colcon_ws

# Update and install prerequisites
RUN apt-get update || true && apt-get install -y \
    apt-transport-https \
    curl \
    git \
    build-essential \
    cmake \
    python3-rosdep \
    libglfw3 libglfw3-dev \
    vim \
    git \
    python3-pip \
    libboost-all-dev \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-pal-statistics \
    ros-${ROS_DISTRO}-moveit --no-install-recommends

# Setup and initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Setup colcon workspace
RUN mkdir -p $COLCON_HOME/src \
    && cd $COLCON_HOME/src \
    && git clone https://github.com/IntelRealSense/realsense-ros.git -b r/4.56.3 \
    && cd $COLCON_HOME \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y \
    && . /ros_entrypoint.sh \
    && colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' --symlink-install

# Add colcon build alias and environment setups to bashrc
RUN echo "alias cb='colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' --symlink-install'" >> /root/.bashrc \
    && echo ". /ros_entrypoint.sh" >> /root/.bashrc \
    && echo ". $COLCON_HOME/install/setup.bash" >> /root/.bashrc \
    && echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "set +e" >> /root/.bashrc

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
