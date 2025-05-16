#!/usr/bin/env python3
#
# Launch all main components: robot, ZED camera, and RealSense multi-camera with delays

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'follower.launch.py'))
    )
    camera_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'camera_zed.launch.py')),
        launch_arguments={'camera_model': 'zedm'}.items()
    )
    camera_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'camera_realsense.launch.py'))
    )

    return LaunchDescription([
        follower,
        TimerAction(period=10.0, actions=[camera_zed]),
        TimerAction(period=20.0, actions=[camera_realsense]),
    ])
