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

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveToHomeSmooth(Node):
    def __init__(self):
        super().__init__('move_to_home_smooth')

        self.joint_publishers = {
            'arm_l_controller': self.create_publisher(
                JointTrajectory, '/leader/joint_trajectory_left/joint_trajectory', 10),
            'arm_r_controller': self.create_publisher(
                JointTrajectory, '/leader/joint_trajectory_right/joint_trajectory', 10),
            'neck_controller': self.create_publisher(
                JointTrajectory, '/neck_controller/joint_trajectory', 10),
            'body_controller': self.create_publisher(
                JointTrajectory, '/body_controller/joint_trajectory', 10)
        }

        self.joint_groups = {
            'arm_l_controller': [
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
                'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'l_rh_r1_joint'],
            'arm_r_controller': [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
                'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'r_rh_r1_joint'],
            'neck_controller': ['neck_joint1', 'neck_joint2'],
            'body_controller': ['linear_joint']
        }

        self.target_positions = {
            'arm_l_joint1': 0.0, 'arm_l_joint2': 0.0, 'arm_l_joint3': 0.0,
            'arm_l_joint4': 0.0, 'arm_l_joint5': 0.0, 'arm_l_joint6': 0.0,
            'arm_l_joint7': 0.0, 'arm_r_joint1': 0.0, 'arm_r_joint2': 0.0,
            'arm_r_joint3': 0.0, 'arm_r_joint4': 0.0, 'arm_r_joint5': 0.0,
            'arm_r_joint6': 0.0, 'arm_r_joint7': 0.0, 'neck_joint1': 0.0,
            'neck_joint2': 0.0, 'linear_joint': 0.0, 'l_rh_r1_joint': 0.0,
            'r_rh_r1_joint': 0.0
        }

        self.current_positions = {}
        self.epsilon = 0.05
        self.reached_target = False
        self.num_points = 5000  # Number of points for smooth trajectory
        self.duration = 3.0  # Duration of the movement in seconds

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.timer = self.create_timer(1.0, self.move_all_to_home_smooth)

    def create_smooth_trajectory(self, controller_name, start_positions):
        joint_names = self.joint_groups[controller_name]
        end_positions = [self.target_positions[joint] for joint in joint_names]

        traj = JointTrajectory()
        traj.joint_names = joint_names
        times = np.linspace(0, self.duration, self.num_points)

        for i in range(self.num_points):
            point = JointTrajectoryPoint()
            t_norm = times[i] / self.duration

            positions = []
            velocities = []
            accelerations = []

            for j in range(len(joint_names)):
                start_pos = start_positions[j]
                end_pos = end_positions[j]

                # Quintic polynomial interpolation for smooth motion
                coeffs = [
                    start_pos,
                    0.0,
                    0.0,
                    10 * end_pos - 10 * start_pos - 4 *
                    self.duration * 0.0 - 6 * self.duration * 0.0,
                    -15 * end_pos + 15 * start_pos + 7 *
                    self.duration * 0.0 + 8 * self.duration * 0.0,
                    6 * end_pos - 6 * start_pos - 3 *
                    self.duration * 0.0 - 3 * self.duration * 0.0,
                ]

                position = (
                    coeffs[0] + coeffs[1] * t_norm + coeffs[2] * t_norm**2 +
                    coeffs[3] * t_norm**3 + coeffs[4] * t_norm**4 + coeffs[5] * t_norm**5
                )
                velocity = (
                    coeffs[1] / self.duration + 2 * coeffs[2] * t_norm / self.duration +
                    3 * coeffs[3] * t_norm**2 / self.duration +
                    4 * coeffs[4] * t_norm**3 / self.duration +
                    5 * coeffs[5] * t_norm**4 / self.duration
                )
                acceleration = (
                    2 * coeffs[2] / (self.duration**2) +
                    6 * coeffs[3] * t_norm / (self.duration**2) +
                    12 * coeffs[4] * t_norm**2 / (self.duration**2) +
                    20 * coeffs[5] * t_norm**3 / (self.duration**2)
                )
                positions.append(position)
                velocities.append(velocity)
                accelerations.append(acceleration)

            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            point.time_from_start.sec = int(times[i])
            point.time_from_start.nanosec = int((times[i] % 1) * 1e9)
            traj.points.append(point)
        return traj

    def move_all_to_home_smooth(self):
        if not self.current_positions:
            self.get_logger().warn('Current joint positions not yet received.')
            return

        for controller_name, joint_names in self.joint_groups.items():
            start_positions = [self.current_positions.get(joint, 0.0) for joint in joint_names]
            traj = self.create_smooth_trajectory(controller_name, start_positions)
            self.joint_publishers[controller_name].publish(traj)
            self.get_logger().info(f'Sending smooth command to {controller_name}')

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.target_positions:
                self.current_positions[name] = msg.position[i]

        if self.current_positions and all(
            abs(self.current_positions.get(j, 0.0) - self.target_positions[j]) < self.epsilon
            for j in self.target_positions
        ):
            if not self.reached_target:
                self.get_logger().info(
                    'All joints reached target positions smoothly. Shutting down node.')
                self.reached_target = True
                self.shutdown_node()

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToHomeSmooth()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
