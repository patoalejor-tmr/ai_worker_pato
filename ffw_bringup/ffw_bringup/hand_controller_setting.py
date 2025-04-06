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

import rclpy
from rclpy.node import Node
import time
from ffw_hand_library.library import InspireHand

Speed = 500         # 0~1000
Power = 500         # 0~1000
start_angle = 1000  # 0~1000 (open state)


class HandControllerSetting(Node):
    def __init__(self):
        super().__init__('hand_controller_setting')

        # Initialize both hands
        self.hands = {
            'right': InspireHand('/dev/right_hand', 1),
            'left': InspireHand('/dev/left_hand', 2)
        }

        time.sleep(0.5)  # Wait for stabilization

        for side, hand in self.hands.items():
            self.get_logger().info(f'{side.upper()} hand configuration started')
            self.apply_settings(hand)
            self.get_logger().info(f'{side.upper()} hand configuration completed\n')

    def apply_settings(self, hand):
        # Clear errors
        hand.set_clear_error()
        time.sleep(0.1)

        # Force sensor calibration (uncomment if needed)
        hand.gesture_force_clb()
        self.get_logger().info('Force sensor calibration completed')

        # Set default speed
        hand.setdefaultspeed(Speed, Speed, Speed, Speed, Speed, Speed)
        hand.setspeed(Speed, Speed, Speed, Speed, Speed, Speed)

        # Set default power
        hand.setdefaultpower(Power, Power, Power, Power, Power, Power)
        hand.setpower(Power, Power, Power, Power, Power, Power)

        # Set initial angle (open state)
        hand.setangle(start_angle, start_angle, start_angle, start_angle, start_angle, start_angle)

        # Save to flash
        hand.set_save_flash()


def main(args=None):
    rclpy.init(args=args)
    node = HandControllerSetting()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
