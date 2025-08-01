// Copyright 2021 ros2_control development team
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

#include "joint_trajectory_command_broadcaster/joint_trajectory_command_broadcaster.hpp"

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <algorithm>
#include <iterator>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace joint_trajectory_command_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_POSITION;

JointTrajectoryCommandBroadcaster::JointTrajectoryCommandBroadcaster() {}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointTrajectoryCommandBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration JointTrajectoryCommandBroadcaster::
state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (use_all_available_interfaces()) {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  } else {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : params_.joints) {
      for (const auto & interface : params_.interfaces) {
        state_interfaces_config.names.push_back(joint + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (use_all_available_interfaces()) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'joints' or 'interfaces' parameter is empty. "
      "All available state interfaces will be considered.");
    params_.joints.clear();
    params_.interfaces.clear();
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing trajectory states for the defined 'joints' and 'interfaces' parameters.");
  }

  // Map interface if needed
  map_interface_to_joint_state_.clear();
  map_interface_to_joint_state_[HW_IF_POSITION] = params_.map_interface_to_joint_state.position;

  try {
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "";
    // Create publisher for JointTrajectory
    joint_trajectory_publisher_ =
      get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      topic_name_prefix + "joint_trajectory", rclcpp::SystemDefaultsQoS());

    realtime_joint_trajectory_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(
      joint_trajectory_publisher_);

    // Create subscriber for follower joint states
    joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.follower_joint_states_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTrajectoryCommandBroadcaster::joint_states_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Subscribed to follower joint states topic: %s",
      params_.follower_joint_states_topic.c_str());
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  const std::string & urdf = get_robot_description();
  is_model_loaded_ = !urdf.empty() && model_.initString(urdf);
  if (!is_model_loaded_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to parse robot description. Will proceed without URDF-based filtering.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  // Check offsets
  const size_t num_joints = joint_names_.size();
  if (params_.offsets.empty()) {
    // If no offsets provided, use zeros
    joint_offsets_.assign(num_joints, 0.0);
  } else if (params_.offsets.size() != num_joints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "The number of provided offsets (%zu) does not match the number of joints (%zu).",
      params_.offsets.size(), num_joints);
    return CallbackReturn::ERROR;
  } else {
    // Use provided offsets
    joint_offsets_ = params_.offsets;
  }

  // No need to init JointState or DynamicJointState messages, only JointTrajectory
  // will be published. We'll construct it on-the-fly in update()

  if (
    !use_all_available_interfaces() &&
    state_interfaces_.size() != (params_.joints.size() * params_.interfaces.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exist. "
      "Check ControllerManager output for more detailed information.");
  }

  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();

  return CallbackReturn::SUCCESS;
}

template<typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  for (const auto & key_item : map) {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend()) {
      return true;
    }
  }
  return false;
}

bool JointTrajectoryCommandBroadcaster::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty()) {
    return false;
  }

  // Initialize mapping
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++) {
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0) {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }

  // Filter out joints without position interface (since we want positions)
  for (const auto & name_ifv : name_if_value_mapping_) {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION})) {
      if (
        !params_.use_urdf_to_filter || !params_.joints.empty() || !is_model_loaded_ ||
        model_.getJoint(name_ifv.first))
      {
        joint_names_.push_back(name_ifv.first);
      }
    }
  }

  // Add extra joints if needed
  rclcpp::Parameter extra_joints;
  if (get_node()->get_parameter("extra_joints", extra_joints)) {
    const std::vector<std::string> & extra_joints_names = extra_joints.as_string_array();
    for (const auto & extra_joint_name : extra_joints_names) {
      if (name_if_value_mapping_.count(extra_joint_name) == 0) {
        name_if_value_mapping_[extra_joint_name] = {
          {HW_IF_POSITION, 0.0}};
        joint_names_.push_back(extra_joint_name);
      }
    }
  }

  return true;
}

bool JointTrajectoryCommandBroadcaster::use_all_available_interfaces() const
{
  return params_.joints.empty() || params_.interfaces.empty();
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

void JointTrajectoryCommandBroadcaster::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update follower joint positions
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->position.size()) {
      follower_joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  // Debug logging (only log occasionally to avoid spam)
  static int callback_count = 0;
  if (++callback_count % 100 == 0) {
    RCLCPP_DEBUG(get_node()->get_logger(),
      "Received follower joint states for %zu joints", msg->name.size());
  }
}

double JointTrajectoryCommandBroadcaster::calculate_mean_error() const
{
  // Check if we have received any follower joint states
  if (follower_joint_positions_.empty()) {
    return std::numeric_limits<double>::max();  // Return max error if no follower data
  }

  double total_error = 0.0;
  int valid_joints = 0;

  // Calculate mean error across all joints
  for (const auto & joint_name : joint_names_) {
    auto follower_it = follower_joint_positions_.find(joint_name);
    if (follower_it == follower_joint_positions_.end()) {
      continue;  // Skip joints not available in follower
    }

    double leader_pos = get_value(name_if_value_mapping_, joint_name, HW_IF_POSITION);
    if (std::isnan(leader_pos)) {
      continue;  // Skip joints without valid leader position
    }

    // Apply reverse and offset to leader position for comparison
    if (std::find(params_.reverse_joints.begin(), params_.reverse_joints.end(), joint_name) !=
      params_.reverse_joints.end())
    {
      leader_pos = -leader_pos;
    }

    // Find the offset for this joint
    auto joint_it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
    if (joint_it != joint_names_.end()) {
      size_t joint_index = std::distance(joint_names_.begin(), joint_it);
      if (joint_index < joint_offsets_.size()) {
        leader_pos += joint_offsets_[joint_index];
      }
    }

    total_error += std::abs(leader_pos - follower_it->second);
    valid_joints++;
  }

  return valid_joints > 0 ? total_error / valid_joints : std::numeric_limits<double>::max();
}

bool JointTrajectoryCommandBroadcaster::check_joints_synced() const
{
  double mean_error = calculate_mean_error();
  return mean_error <= params_.sync_threshold;
}

controller_interface::return_type JointTrajectoryCommandBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Update stored values
  for (const auto & state_interface : state_interfaces_) {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    auto value = state_interface.get_optional();
    if (value) {
      name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] = *value;
    }
  }

  // Calculate mean error and check if joints are synced
  double mean_error = calculate_mean_error();
  bool current_synced = check_joints_synced();

  // Update sync status and handle first publish
  if (first_publish_) {
    joints_synced_ = false;
    first_publish_ = false;
    RCLCPP_INFO(get_node()->get_logger(),
        "First publish - using adaptive time_from_start based on error");
  } else {
    // Once synced, stay synced permanently
    if (!joints_synced_ && current_synced) {
      joints_synced_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
          "Joints synced for the first time - switching to immediate time_from_start permanently");
    }
  }

  // Publish JointTrajectory message with current positions
  if (realtime_joint_trajectory_publisher_ && realtime_joint_trajectory_publisher_->trylock()) {
    auto & traj_msg = realtime_joint_trajectory_publisher_->msg_;
    traj_msg.header.stamp = rclcpp::Time(0, 0);
    traj_msg.joint_names = joint_names_;

    const size_t num_joints = joint_names_.size();
    traj_msg.points.clear();
    traj_msg.points.resize(1);
    traj_msg.points[0].positions.resize(num_joints, kUninitializedValue);

    for (size_t i = 0; i < num_joints; ++i) {
      double pos_value =
        get_value(name_if_value_mapping_, joint_names_[i], HW_IF_POSITION);

      // Check if the current joint is in the reverse_joints parameter
      if (
        std::find(
          params_.reverse_joints.begin(),
          params_.reverse_joints.end(),
          joint_names_[i]) != params_.reverse_joints.end())
      {
        pos_value = -pos_value;
      }

      // Apply offset
      pos_value += joint_offsets_[i];

      traj_msg.points[0].positions[i] = pos_value;
    }

    // Set time_from_start based on sync status and mean error
    if (joints_synced_) {
      traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);  // immediate when synced
    } else {
      // Adaptive timing based on mean error using parameters
      if(mean_error < params_.min_error) {
        mean_error = 0;
      }

      double error_ratio = std::min(mean_error / params_.max_error, 1.0);
      // Corrected logic: small error -> small delay, large error -> large delay
      double adaptive_delay = params_.min_delay + (params_.max_delay - params_.min_delay) *
        error_ratio;

      // Convert to nanoseconds
      int32_t delay_ns = static_cast<int32_t>(adaptive_delay * 1e9);
      traj_msg.points[0].time_from_start = rclcpp::Duration(0, delay_ns);
    }

    realtime_joint_trajectory_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_trajectory_command_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_command_broadcaster::JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
