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
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class MoveToHome(Node):
    def __init__(self):
        super().__init__('move_to_home')

        self.joint_publishers = {
            'arm_l_controller': self.create_publisher(
                JointTrajectory, '/arm_l_controller/joint_trajectory', 10),
            'arm_r_controller': self.create_publisher(
                JointTrajectory, '/arm_r_controller/joint_trajectory', 10),
            'neck_controller': self.create_publisher(
                JointTrajectory, '/neck_controller/joint_trajectory', 10),
            'body_controller': self.create_publisher(
                JointTrajectory, '/body_controller/joint_trajectory', 10)
        }

        self.joint_groups = {
            'arm_l_controller': [
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
                'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7'],
            'arm_r_controller': [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
                'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7'],
            'neck_controller': ['neck_joint1', 'neck_joint2'],
            'body_controller': ['linear_joint']
        }

        self.target_positions = {
            'arm_l_joint1': 0.0, 'arm_l_joint2': 0.0, 'arm_l_joint3': 0.0,
            'arm_l_joint4': 0.0, 'arm_l_joint5': 0.0, 'arm_l_joint6': 0.0,
            'arm_l_joint7': 0.0, 'arm_r_joint1': 0.0, 'arm_r_joint2': 0.0,
            'arm_r_joint3': 0.0, 'arm_r_joint4': 0.0, 'arm_r_joint5': 0.0,
            'arm_r_joint6': 0.0, 'arm_r_joint7': 0.0, 'neck_joint1': 0.0,
            'neck_joint2': 0.0, 'linear_joint': 0.0
        }

        self.epsilon = 0.05
        self.reached_target = False

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.timer = self.create_timer(1.0, self.move_all_to_home)

    def move_all_to_home(self):
        for controller_name, joint_names in self.joint_groups.items():
            traj = JointTrajectory()
            traj.joint_names = joint_names

            point = JointTrajectoryPoint()
            point.positions = [self.target_positions[j] for j in joint_names]
            point.time_from_start.sec = 3

            traj.points.append(point)

            self.joint_publishers[controller_name].publish(traj)
            self.get_logger().info(f'Sending command to {controller_name}')

    def joint_state_callback(self, msg):
        if not set(self.target_positions.keys()).issubset(set(msg.name)):
            return

        current_positions = {
            name: msg.position[idx] for idx, name in enumerate(msg.name)
            if name in self.target_positions
        }

        if all(abs(current_positions[j] - self.target_positions[j]) < self.epsilon
               for j in self.target_positions):
            if not self.reached_target:
                self.get_logger().info(
                    'All joints reached target positions. Shutting down node.')
                self.reached_target = True
                self.shutdown_node()

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToHome()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
