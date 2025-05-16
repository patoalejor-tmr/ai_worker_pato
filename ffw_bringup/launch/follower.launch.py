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

    hardware_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'hardware_follower_teleop_with_rh.launch.py'))
    )
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'zed_camera.launch.py')),
        launch_arguments={'camera_model': 'zedm'}.items()
    )
    rs_multi_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'rs_multi_camera.launch.py'))
    )

    return LaunchDescription([
        hardware_follower,
        TimerAction(period=10.0, actions=[zed_camera]),
        TimerAction(period=20.0, actions=[rs_multi_camera]),
    ])
