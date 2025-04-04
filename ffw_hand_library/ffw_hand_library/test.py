from library import InspireHand

right = InspireHand('/dev/right_hand', 1)
right.setangle(0, 0, 0, 0, 1000, 1000)
# right.setangle(1000, 1000, 1000, 1000, 1000, 1000)

left = InspireHand('/dev/left_hand', 2)
left.setangle(0, 0, 0, 0, 1000, 1000)
# left.setangle(1000, 1000, 1000, 1000, 1000, 1000)
