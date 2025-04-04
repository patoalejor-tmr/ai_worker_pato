#!/usr/bin/env python3

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32MultiArray
from ffw_hand_library.library import InspireHand
import rclpy
from rclpy.node import Node
import serial
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class LeaderFollowerHand(Node):
    def __init__(self):
        super().__init__('leader_follower_left_hand')

        # === 파라미터 ===
        self.declare_parameter('serial_port', '/dev/left_hand')
        self.declare_parameter('hand_id', 2)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.hand_id = self.get_parameter('hand_id').get_parameter_value().integer_value

        # === Inspire Hand 연결 ===
        self.hand = InspireHand(self.serial_port, self.hand_id)
        self.get_logger().info(f"✅ InspireHand 연결 완료: {self.serial_port}, ID {self.hand_id}")

        # === Subscriber ===
        self.sub = self.create_subscription(JointTrajectory, '/leader/joint_trajectory_left_hand/joint_trajectory', self.leader_callback, 10)

        # === Publisher ===
        self.hand_pub = self.create_publisher(Int32MultiArray, '/follower/left_hand_angles', 10)

        # === 설정 로드 ===
        self.joint_range = self.load_joint_range()
        self.joint_map = {
            'left_little_1_joint': 0,
            'left_ring_1_joint': 1,
            'left_middle_1_joint': 2,
            'left_index_1_joint': 3,
            'left_thumb_2_joint': 4,
            'left_thumb_1_joint': 5,
        }

    def get_src_config_path(self, filename):
        install_path = get_package_share_directory('ffw_bringup')
        src_bringup_path = install_path.replace(
            '/install/ffw_bringup/share/ffw_bringup',
            '/src/ffw/ffw_bringup'
        )
        return os.path.join(src_bringup_path, 'config', filename)

    def load_joint_range(self):
        self.output_file = self.get_src_config_path('hand_joint_range_left.yaml')

        if not os.path.exists(self.output_file):
            self.get_logger().warn("⚠️ joint_range YAML 파일 없음! 기본값 사용")
            return {'min': [0.0] * 6, 'max': [6.28] * 6}

        with open(self.output_file, 'r') as f:
            data = yaml.safe_load(f)
        self.get_logger().info("📥 joint_range.yaml 불러옴")
        return data

    def scale(self, val, index):
        rad_min = self.joint_range['min'][index]
        rad_max = self.joint_range['max'][index]

        if rad_max - rad_min == 0:
            self.get_logger().warn(f"⚠️ Index {index}에서 min과 max가 같음: {rad_min}. 기본값 0 반환")
            return 0

        val = max(min(val, rad_max), rad_min)
        norm = (val - rad_min) / (rad_max - rad_min)
        scaled = int(norm * 1000)
        return 1000 - scaled  # 뒤집기

    def leader_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("⚠️ Trajectory 메시지에 points가 없음")
            return

        joint_names = msg.joint_names
        positions = msg.points[0].positions
        name_to_position = dict(zip(joint_names, positions))

        scaled = [0] * 6
        for joint_name, target_index in self.joint_map.items():
            val = name_to_position.get(joint_name, 0.0)
            scaled[target_index] = self.scale(val, target_index)

        self.get_logger().info(f"[Leader ➝ Follower LEFT] Scaled: {scaled}")
        self.hand.setangle(*scaled)

        angles = self.hand.get_actangle()
        self.hand_pub.publish(Int32MultiArray(data=angles))

    def destroy_node(self):
        self.hand.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
