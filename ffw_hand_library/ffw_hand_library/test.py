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


right = InspireHand('/dev/right_hand', 1)
right.setangle(0, 0, 0, 0, 1000, 1000)
# right.setangle(1000, 1000, 1000, 1000, 1000, 1000)

left = InspireHand('/dev/left_hand', 2)
left.setangle(0, 0, 0, 0, 1000, 1000)
# left.setangle(1000, 1000, 1000, 1000, 1000, 1000)
