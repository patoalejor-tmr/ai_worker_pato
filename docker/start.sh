#!/bin/bash

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

COLCON_WS=/root/ros2_ws

if [ -f ${COLCON_WS}/install/setup.bash ]; then
    source ${COLCON_WS}/install/setup.bash
fi

# Run the physical_ai_server
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
