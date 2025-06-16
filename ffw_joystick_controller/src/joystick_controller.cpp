// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <joystick_controller/joystick_controller.hpp>

#include <chrono>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/helpers.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joystick_controller
{

JoystickController::JoystickController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
JoystickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
JoystickController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & sensorxel_joy_name : sensorxel_joy_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(sensorxel_joy_name + "/" + interface_type);
    }
  }

  return config;
}

void JoystickController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Store current joint states
  current_joint_states_ = *msg;

  // 센서별로 last_active_positions_ 초기화
  if (!has_joint_states_) {
    for (const auto& sensor_name : sensorxel_joy_names_) {
      const auto& controlled_joints = sensor_controlled_joints_[sensor_name];
      auto& last_active_positions = sensor_last_active_positions_[sensor_name];
      last_active_positions.resize(controlled_joints.size());
      for (size_t i = 0; i < controlled_joints.size(); ++i) {
        const auto & joint_name = controlled_joints[i];
        auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_name);
        if (it != current_joint_states_.name.end()) {
          size_t index = std::distance(current_joint_states_.name.begin(), it);
          last_active_positions[i] = current_joint_states_.position[index];
        }
      }
    }
  }

  has_joint_states_ = true;
}
controller_interface::return_type JoystickController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Starting joystick controller update");

  // Iterate for each sensor
  for (size_t sensor_idx = 0; sensor_idx < sensorxel_joy_names_.size(); ++sensor_idx) {
    const auto& sensor_name = sensorxel_joy_names_[sensor_idx];
    RCLCPP_DEBUG(get_node()->get_logger(), "Processing sensor: %s", sensor_name.c_str());
    
    const auto& controlled_joints = sensor_controlled_joints_[sensor_name];
    for (const auto& joint_name : controlled_joints) {
      RCLCPP_WARN(get_node()->get_logger(), "controlled joint: %s", joint_name.c_str());
    }
    const auto& reverse_interfaces = sensor_reverse_interfaces_[sensor_name];
    auto& last_active_positions = sensor_last_active_positions_[sensor_name];
    auto joint_trajectory_publisher = sensor_joint_trajectory_publisher_[sensor_name];

    bool any_sensorxel_joy_active = false;

    // Read and normalize values for each interface type
    std::vector<double> normalized_values(state_interface_types_.size(), 0.0);
    for (size_t j = 0; j < state_interface_types_.size(); ++j) {
      if (j >= joint_state_interface_.size() || sensor_idx >= joint_state_interface_[j].size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid interface access: j=%zu, i=%zu", j, sensor_idx);
        continue;
      }
      auto opt_value = joint_state_interface_[j][sensor_idx].get().get_optional();
      if (!opt_value.has_value()) {
        RCLCPP_ERROR(get_node()->get_logger(), "No value for state interface [%zu][%zu]", j, sensor_idx);
        continue;
      }
      double raw_adc = opt_value.value();
      RCLCPP_DEBUG(get_node()->get_logger(), "Raw ADC value for sensor %s interface %zu: %f", 
                   sensor_name.c_str(), j, raw_adc);

      // Normalize ADC value to [-1.0, 1.0] range
      double normalized_value;
      if (raw_adc < params_.joystick_calibration_center) {
        normalized_value = -(params_.joystick_calibration_center - raw_adc) /
          (params_.joystick_calibration_center - params_.joystick_calibration_min);
      } else {
        normalized_value = (raw_adc - params_.joystick_calibration_center) /
          (params_.joystick_calibration_max - params_.joystick_calibration_center);
      }
      RCLCPP_DEBUG(get_node()->get_logger(), "Normalized value before deadzone: %f", normalized_value);

      // Apply deadzone
      if (std::abs(normalized_value) < params_.deadzone) {
        normalized_value = 0.0;
        RCLCPP_DEBUG(get_node()->get_logger(), "Value in deadzone, set to 0.0");
      } else {
        any_sensorxel_joy_active = true;
        if (normalized_value > 0) {
          normalized_value = (normalized_value - params_.deadzone) / (1.0 - params_.deadzone);
        } else {
          normalized_value = (normalized_value + params_.deadzone) / (1.0 - params_.deadzone);
        }
        RCLCPP_DEBUG(get_node()->get_logger(), "Value after deadzone adjustment: %f", normalized_value);
      }

      // Check if this interface should be reversed
      const auto & interface_name = state_interface_types_[j];
      if (std::find(reverse_interfaces.begin(), reverse_interfaces.end(), interface_name) != reverse_interfaces.end()) {
        normalized_value = -normalized_value;
        RCLCPP_DEBUG(get_node()->get_logger(), "Interface reversed, new value: %f", normalized_value);
      }

      normalized_values[j] = normalized_value;
    }
    //make dummy joint states
    current_joint_states_ = sensor_msgs::msg::JointState();
    current_joint_states_.name = controlled_joints;
    current_joint_states_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_joint_states_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_joint_states_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_joint_states_.header.stamp = rclcpp::Time(0);
    current_joint_states_.header.frame_id = "base_link";
    // Handle transition from active to inactive (entering deadzone)

    RCLCPP_WARN(get_node()->get_logger(), "was_active_: %d", was_active_);
    RCLCPP_WARN(get_node()->get_logger(), "any_sensorxel_joy_active: %d", any_sensorxel_joy_active);
    RCLCPP_WARN(get_node()->get_logger(), "current_joint_states_.name.empty(): %d", current_joint_states_.name.empty());
    RCLCPP_WARN(get_node()->get_logger(), "controlled_joints.empty(): %d", controlled_joints.empty());


    if (was_active_ && !any_sensorxel_joy_active && !current_joint_states_.name.empty() && !controlled_joints.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "Joystick entering deadzone for sensor %s", sensor_name.c_str());
      for (size_t i = 0; i < controlled_joints.size(); ++i) {
        const auto & joint_name = controlled_joints[i];
        auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_name);
        RCLCPP_WARN(get_node()->get_logger(), "it: %d", it != current_joint_states_.name.end());
        if (it != current_joint_states_.name.end()) {
          size_t index = std::distance(current_joint_states_.name.begin(), it);
          last_active_positions[i] = current_joint_states_.position[index];
          RCLCPP_DEBUG(get_node()->get_logger(), "Stored last position for joint %s: %f", 
                       joint_name.c_str(), last_active_positions[i]);
        }
      }
    }

    RCLCPP_WARN(get_node()->get_logger(), "debug flag");


    // Publish trajectory message in both active and inactive states
    if (!current_joint_states_.name.empty() && !controlled_joints.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "debug flag 2");
      auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
      trajectory_msg.header.stamp = rclcpp::Time(0);
      trajectory_msg.joint_names = controlled_joints;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start = rclcpp::Duration(0, 0);

      RCLCPP_WARN(get_node()->get_logger(), "any_sensorxel_joy_active: %d", any_sensorxel_joy_active);
      if (any_sensorxel_joy_active) {
        RCLCPP_WARN(get_node()->get_logger(), "Joystick active for sensor %s", sensor_name.c_str());
        for (size_t i = 0; i < controlled_joints.size(); ++i) {
          const auto & joint_name = controlled_joints[i];
          RCLCPP_WARN(get_node()->get_logger(), "joint_name: %s", joint_name.c_str());
          auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_name);
          if (it != current_joint_states_.name.end()) {
            size_t index = std::distance(current_joint_states_.name.begin(), it);
            double current_position = current_joint_states_.position[index];
            RCLCPP_WARN(get_node()->get_logger(), "current_position: %f", current_position);
            // X for first joint, Y for second joint, etc.
            double sensorxel_joy_value = (state_interface_types_.size() > 1 && i == 1) ?
              normalized_values[1] : normalized_values[0];
            RCLCPP_WARN(get_node()->get_logger(), "sensorxel_joy_value: %f", sensorxel_joy_value);

            double new_position = current_position + sensorxel_joy_value * params_.jog_scale;
            RCLCPP_WARN(get_node()->get_logger(), "new_position: %f", new_position);
            point.positions.push_back(new_position);
            RCLCPP_WARN(get_node()->get_logger(), "point.positions: %f", point.positions[i]);
            // last_active_positions[sensor_idx] = new_position;
            RCLCPP_WARN(get_node()->get_logger(), "Joint %s: current=%f, new=%f",
                         joint_name.c_str(), current_position, new_position);
          }
        }
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Using last active positions for sensor %s", sensor_name.c_str());
        point.positions = last_active_positions;
      }

      point.velocities.resize(point.positions.size(), 0.0);
      point.accelerations.resize(point.positions.size(), 0.0);

      trajectory_msg.points.push_back(point);
      if (joint_trajectory_publisher) {
        RCLCPP_INFO(get_node()->get_logger(), "Publishing joint trajectory to topic: %s", 
                    sensor_joint_trajectory_topic_[sensor_name].c_str());
        joint_trajectory_publisher->publish(trajectory_msg);
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Joint trajectory publisher not found for sensor: %s", 
                    sensor_name.c_str());
      }
    }
    was_active_ = any_sensorxel_joy_active;

    // sensorxel_joy_values_[sensor_idx] = normalized_values;
  }

  // Optionally publish sensorxel_joy values (flattened)
  auto sensorxel_joy_msg = std_msgs::msg::Float64MultiArray();
  for (const auto & sensorxel_joy_value : sensorxel_joy_values_) {
    sensorxel_joy_msg.data.insert(
      sensorxel_joy_msg.data.end(),
      sensorxel_joy_value.begin(),
      sensorxel_joy_value.end());
  }

  if (sensorxel_joy_publisher_.count("common") > 0) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Publishing joystick values to common topic");
    sensorxel_joy_publisher_["common"]->publish(sensorxel_joy_msg);
  }

  // was_active_ = any_sensorxel_joy_active;
  RCLCPP_DEBUG(get_node()->get_logger(), "Joystick controller update completed");

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn JoystickController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // Get sensorxel_joy sensor names from parameters
  sensorxel_joy_names_ = params_.joystick_sensors;
  n_sensorxel_joys_ = sensorxel_joy_names_.size();

  if (sensorxel_joy_names_.empty()) {
    RCLCPP_WARN(logger, "'joystick_sensors' parameter is empty.");
  }

  // Initialize the sensorxel_joy values vector with the correct size
  sensorxel_joy_values_.resize(n_sensorxel_joys_);
  for (auto & vec : sensorxel_joy_values_) {
    vec.resize(state_interface_types_.size(), 0.0);
  }
  // 센서별 파라미터 읽기
  for (const auto& sensor_name : sensorxel_joy_names_) {
    // controlled_joints
    std::string joints_param = sensor_name + "_controlled_joints";
    if (get_node()->has_parameter(joints_param)) {
      sensor_controlled_joints_[sensor_name] = get_node()->get_parameter(joints_param).as_string_array();
    }
    // reverse_interfaces
    std::string reverse_param = sensor_name + "_reverse_interfaces";
    if (get_node()->has_parameter(reverse_param)) {
      sensor_reverse_interfaces_[sensor_name] = get_node()->get_parameter(reverse_param).as_string_array();
    }
    // joint_trajectory_topic
    std::string topic_param = sensor_name + "_joint_trajectory_topic";
    if (get_node()->has_parameter(topic_param)) {
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s, value: %s", topic_param.c_str(), get_node()->get_parameter(topic_param).as_string().c_str());
      sensor_joint_trajectory_topic_[sensor_name] = get_node()->get_parameter(topic_param).as_string();
    }else{
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s not found", topic_param.c_str());
    }
  }

  // Create publisher for sensorxel_joy values (공통 토픽)
  sensorxel_joy_publisher_["common"] = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/sensorxel_joy_values", rclcpp::SystemDefaultsQoS());

  // Create publisher for joint trajectory
  for (const auto& sensor_name : sensorxel_joy_names_) {
    RCLCPP_WARN(get_node()->get_logger(), "Creating joint trajectory publisher for sensor: %s, topic: %s", sensor_name.c_str(), sensor_joint_trajectory_topic_[sensor_name].c_str());
    // rclcpp::sleep_for(std::chrono::seconds(1));
    sensor_joint_trajectory_publisher_[sensor_name] = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      sensor_joint_trajectory_topic_[sensor_name], rclcpp::SystemDefaultsQoS());
  }

  // Create subscriber for joint states
  RCLCPP_WARN(get_node()->get_logger(), "Creating joint states subscriber for topic: %s", params_.joint_states_topic.c_str());
  // rclcpp::sleep_for(std::chrono::seconds(5));
  joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_states_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&JoystickController::joint_states_callback, this, std::placeholders::_1));


  RCLCPP_INFO(get_node()->get_logger(), "JoystickController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // Initialize state interface vector
  joint_state_interface_.resize(state_interface_types_.size());

  // Order all sensorxel_joy sensors in the storage
  for (size_t i = 0; i < state_interface_types_.size(); ++i) {
    const auto & interface = state_interface_types_[i];
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    ordered_interfaces;
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, sensorxel_joy_names_, interface, ordered_interfaces))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.",
        n_sensorxel_joys_, interface.c_str(), ordered_interfaces.size());
      return CallbackReturn::ERROR;
    }
    joint_state_interface_[i] = ordered_interfaces;
  }

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "JoystickController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
}  // namespace joystick_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joystick_controller::JoystickController, controller_interface::ControllerInterface)
