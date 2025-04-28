################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.16)
project(ffw_fsr_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

################################################################################
# Build
################################################################################
generate_parameter_library(fsr_controller_parameters
  src/fsr_controller_parameters.yaml
)

add_library(fsr_controller SHARED
  src/fsr_controller.cpp
)
target_compile_features(fsr_controller PUBLIC cxx_std_17)
target_include_directories(fsr_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/fsr_controller>
)
target_link_libraries(fsr_controller PUBLIC
  fsr_controller_parameters
)
ament_target_dependencies(fsr_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fsr_controller PRIVATE "fsr_controller_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface fsr_plugin.xml)

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/fsr_controller
)
install(TARGETS
    fsr_controller
    fsr_controller_parameters
  EXPORT export_fsr_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

################################################################################
# Export information
################################################################################
ament_export_targets(export_fsr_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

################################################################################
# Macro for ament package
################################################################################
ament_package() 