#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler, Shutdown, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ffw_bringup_path = os.path.join(
        get_package_share_directory('ffw_bringup')
    )

    # 👉 상태 저장 변수
    left_done_flag = {'value': False}
    right_done_flag = {'value': False}

    # 1️⃣ hardware bringup
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ffw_bringup_path, 'launch', 'hardware_leader_change.launch.py')
        )
    )

    # 2️⃣ 토크 disable (3초 뒤)
    disable_torque = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/leader/dynamixel_hardware_interface/set_dxl_torque',
                    'std_srvs/srv/SetBool',
                    '{"data": false}'
                ],
                output='screen'
            )
        ]
    )

    # 3️⃣ 캘리브레이터 노드 (5초 뒤)
    calibrator_left = Node(
        package='ffw_bringup',
        executable='hand_calibrator_left.py',
        name='hand_calibrator_left',
        output='screen',
    )
    calibrator_right = Node(
        package='ffw_bringup',
        executable='hand_calibrator_right.py',
        name='hand_calibrator_right',
        output='screen',
    )

    run_calibrators = TimerAction(
        period=5.0,
        actions=[calibrator_left, calibrator_right]
    )

    # 4️⃣ 캘리브레이터 종료 감시 → 둘 다 끝나면 shutdown
    def check_and_shutdown(context, flag_dict, other_flag, shutdown_msg):
        flag_dict['value'] = True
        if other_flag['value']:
            return [LogInfo(msg=shutdown_msg), Shutdown()]
        return []

    left_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrator_left,
            on_exit=lambda context: check_and_shutdown(context, left_done_flag, right_done_flag, "✅ Both hands calibrated. Shutting down.")
        )
    )
    right_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrator_right,
            on_exit=lambda context: check_and_shutdown(context, right_done_flag, left_done_flag, "✅ Both hands calibrated. Shutting down.")
        )
    )

    return LaunchDescription([
        hardware_launch,
        disable_torque,
        run_calibrators,
        left_exit_handler,
        right_exit_handler
    ])
