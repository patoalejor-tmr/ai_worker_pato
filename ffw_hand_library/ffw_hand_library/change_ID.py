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
# Authors: Wonho Yun

from library import InspireHand
import time

CURRENT_ID = 1      # Currently set ID
NEW_ID = 2          # New ID to be set
PORT = '/dev/left_hand'  # Serial port path (modify as needed)

def change_hand_id(current_id, new_id, port):
    print(f"Attempting to change ID {current_id} ➜ {new_id}...")

    # Create object with the current ID
    hand = InspireHand(port=port, hand_id=current_id)

    # 1. Send ID change command (RAM)
    data = [new_id] + [-1] * 5  # Only the first value is the new ID
    res = hand.set_6val(addr=0x03E8, values=data, label="set_ID_RAM")
    if not res:
        print("Failed to change ID (RAM)")
        return

    print("ID changed (RAM applied). Current ID is now", new_id)

    # 2. Wait for stabilization
    print("⏳ Waiting for stabilization...")
    time.sleep(0.2)

    # 3. Save to Flash (reconnect with the new ID)
    hand = InspireHand(port=port, hand_id=new_id)
    res = hand.set_save_flash()
    if res:
        print("Flash save completed — ID permanently changed!")
    else:
        print("Flash save failed — ID may revert after reboot.")

if __name__ == '__main__':
    change_hand_id(CURRENT_ID, NEW_ID, PORT)
