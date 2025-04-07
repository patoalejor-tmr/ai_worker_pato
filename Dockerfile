# Base ROS2 Jazzy image
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    vim \
    git \
    libboost-all-dev \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pal-statistics \
    ros-${ROS_DISTRO}-moveit --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Optional: setup bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc

CMD ["bash"]
