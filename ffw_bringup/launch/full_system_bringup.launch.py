from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
import os

def generate_launch_description():
    bringup_dir = os.path.join(
        os.getenv('COLCON_PREFIX_PATH', '/opt/ros/humble').split(':')[0],
        'share/ffw_bringup'
    )

    teleop_launch = os.path.join(bringup_dir, 'launch', 'hardware_follower_teleop_with_hand.launch.py')
    leader_launch = os.path.join(bringup_dir, 'launch', 'hardware_leader_change.launch.py')
    hand_ctrl_launch = os.path.join(bringup_dir, 'launch', 'hand_controller_two.launch.py')

    return LaunchDescription([
        # Step 1: follower bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
            launch_arguments={}
        ),

        # Step 2: Init follower pose
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ffw_bringup', 'init_position_for_follower_teleop.py'],
                    output='screen'
                )
            ]
        ),

        # Step 3: leader bringup (wait a bit for follower init to complete)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(leader_launch),
                    launch_arguments={}
                )
            ]
        ),

        # Step 4: Leader torque OFF
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/leader/dynamixel_hardware_interface/set_dxl_torque',
                        'std_srvs/srv/SetBool',
                        'data:\ false'
                    ],
                    output='screen'
                )
            ]
        ),

        # Step 5: Hand controller launch
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(hand_ctrl_launch),
                    launch_arguments={}
                )
            ]
        ),

        # Step 6: GUI Keyboard Teleop
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ffw_teleop', 'keyboard_control_standalone.py'],
                    output='screen'
                )
            ]
        ),
    ])
