cmake_minimum_required(VERSION 3.16)
project(ffw_robot_manager)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  std_srvs
  dynamixel_hardware_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(ffw_robot_manager SHARED
  src/ffw_robot_manager.cpp
)
target_compile_features(ffw_robot_manager PUBLIC cxx_std_17)
target_include_directories(ffw_robot_manager PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ffw_robot_manager>
)

ament_target_dependencies(ffw_robot_manager
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface ffw_robot_manager_plugin.xml)

install(
  TARGETS ffw_robot_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/ffw_robot_manager
)
install(
  TARGETS
    ffw_robot_manager
  EXPORT export_ffw_robot_manager
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_ffw_robot_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
