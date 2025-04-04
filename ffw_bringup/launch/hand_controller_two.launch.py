#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ffw_bringup',
            executable='hand_controller_right',
            name='right_hand_controller',
            output='screen',
            parameters=[
                {'serial_port': '/dev/right_hand'},
                {'hand_id': 1}
            ]
        ),
        Node(
            package='ffw_bringup',
            executable='hand_controller_left',
            name='left_hand_controller',
            output='screen',
            parameters=[
                {'serial_port': '/dev/left_hand'},
                {'hand_id': 2}
            ]
        )
    ])
