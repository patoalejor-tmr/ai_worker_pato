#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import time

class HandCalibrator(Node):
    def __init__(self):
        super().__init__('hand_calibrator_right')
        self.sub = self.create_subscription(JointTrajectory, '/leader/joint_trajectory_right_hand/joint_trajectory', self.callback, 10)

        self.target_order = [
            'right_little_1_joint',
            'right_ring_1_joint',
            'right_middle_1_joint',
            'right_index_1_joint',
            'right_thumb_2_joint',
            'right_thumb_1_joint',
        ]

        self.joint_min = None
        self.joint_max = None
        self.sample_count = 0
        self.max_samples = 1000
        self.done = False

        install_path = get_package_share_directory('ffw_bringup')
        src_bringup_path = install_path.replace('/install/ffw_bringup/share/ffw_bringup', '/src/ffw/ffw_bringup')
        self.output_file = os.path.join(src_bringup_path, 'config', 'hand_joint_range_right.yaml')
        time.sleep(0.5)  # 잠시 대기
        self.get_logger().info("📡 오른손 calibration 시작 중... (움직이세요!)")

    def callback(self, msg: JointTrajectory):
        if self.done or not msg.points:
            return

        joint_dict = dict(zip(msg.joint_names, msg.points[0].positions))
        ordered_pos = [joint_dict.get(name, 0.0) for name in self.target_order]

        if self.joint_min is None:
            self.joint_min = list(ordered_pos)
            self.joint_max = list(ordered_pos)
        else:
            self.joint_min = [min(a, b) for a, b in zip(self.joint_min, ordered_pos)]
            self.joint_max = [max(a, b) for a, b in zip(self.joint_max, ordered_pos)]

        self.sample_count += 1
        if self.sample_count >= self.max_samples:
            self.save()
            self.done = True
            self.get_logger().info("🎯 오른손 샘플링 완료. 노드를 종료합니다.")

    def save(self):
        data = {'min': self.joint_min, 'max': self.joint_max}
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f"✅ Calibration 완료! 저장됨: {self.output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = HandCalibrator()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
