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
start_angle = 1000  # 0~1000 (펼친 상태)


class HandControllerSetting(Node):
    def __init__(self):
        super().__init__('hand_controller_setting')

        # 👉 양손 초기화
        self.hands = {
            'right': InspireHand('/dev/right_hand', 1),
            'left': InspireHand('/dev/left_hand', 2)
        }

        time.sleep(0.5)  # 안정화 대기

        for side, hand in self.hands.items():
            self.get_logger().info(f"🔧 {side.upper()} 손 설정 시작")
            self.apply_settings(hand)
            self.get_logger().info(f"✅ {side.upper()} 손 설정 완료\n")

    def apply_settings(self, hand):
        # 에러 초기화
        hand.set_clear_error()
        time.sleep(0.1)

        # 포스 센서 보정 (원할 경우 주석 해제)
        hand.gesture_force_clb()
        self.get_logger().info("🧪 포스 센서 보정 완료")

        # 기본 속도 설정
        hand.setdefaultspeed(Speed, Speed, Speed, Speed, Speed, Speed)
        hand.setspeed(Speed, Speed, Speed, Speed, Speed, Speed)

        # 기본 힘 설정
        hand.setdefaultpower(Power, Power, Power, Power, Power, Power)
        hand.setpower(Power, Power, Power, Power, Power, Power)

        # 초기 각도 설정 (펼친 상태)
        hand.setangle(start_angle, start_angle, start_angle, start_angle, start_angle, start_angle)

        # Flash 저장
        hand.set_save_flash()


def main(args=None):
    rclpy.init(args=args)
    node = HandControllerSetting()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
