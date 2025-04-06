#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():
    # Step 1: Start follower launch file
    start_follower = ExecuteProcess(
        cmd=["ros2", "launch", "ffw_bringup", "hardware_follower_teleop_without_hand.launch.py"],
        output="screen"
    )

    # Step 2: Run the initialization script for the follower
    init_follower = ExecuteProcess(
        cmd=["ros2", "run", "ffw_bringup", "init_position_for_follower_teleop.py"],
        output="screen",
        shell=True
    )

    # Step 3: Start leader launch file
    start_leader = ExecuteProcess(
        cmd=["ros2", "launch", "ffw_bringup", "hardware_leader_without_hand.launch.py"],
        output="screen",
        shell=True
    )

    disable_leader_torque = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "service", "call",
                "/leader/dynamixel_hardware_interface/set_dxl_torque",
                "std_srvs/srv/SetBool",
                "data:\ false"
            ],
            output="screen",
            shell=True
        )]
    )


    # ✅ Step 5: Run keyboard control node
    run_keyboard_control = ExecuteProcess(
        cmd=["ros2", "run", "ffw_teleop", "keyboard_control_standalone.py"],
        output="screen",
        shell=True
    )

    return LaunchDescription([
        LogInfo(msg="🚀 Starting hardware_follower.launch.py..."),
        start_follower,

        # Step 2: Ensure init_follower starts after follower is launched
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_follower,
                on_start=[
                    LogInfo(msg="✅ hardware_follower.launch.py has fully started. Running init_position_for_follower.py..."),
                    init_follower
                ]
            )
        ),

        # Step 3: Start leader after follower initialization completes
        RegisterEventHandler(
            OnProcessExit(
                target_action=init_follower,
                on_exit=[
                    LogInfo(msg="✅ init_position_for_follower.py has fully executed and exited. Starting hardware_leader.launch.py..."),
                    start_leader
                ]
            )
        ),

        # Step 4: Disable torque and run keyboard control after leader launch starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_leader,
                on_start=[
                    LogInfo(msg="🔧 Disabling torque on leader..."),
                    disable_leader_torque,
                    LogInfo(msg="⌨️ Starting keyboard_control_standalone.py..."),
                    run_keyboard_control
                ]
            )
        ),
    ])
