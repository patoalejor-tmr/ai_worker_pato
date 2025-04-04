cmake_minimum_required(VERSION 3.8)
project(ffw_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/keyboard_control_standalone.py
  scripts/keyboard_control_teleop_with_hand.py
  scripts/keyboard_control_only_left.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
