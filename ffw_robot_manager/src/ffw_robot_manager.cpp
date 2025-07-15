#include "ffw_robot_manager/ffw_robot_manager.hpp"

#include <algorithm>
#include <string>
#include <vector>
#include <unordered_set>

namespace ffw_robot_manager
{
FfwRobotManager::FfwRobotManager() {}

controller_interface::CallbackReturn FfwRobotManager::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FfwRobotManager::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FfwRobotManager::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn FfwRobotManager::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  gpio_names_.clear();
  gpio_interface_indices_.clear();
  torque_disabled_ = false;

  // Create service client for Dynamixel torque control
  torque_client_ = get_node()->create_client<std_srvs::srv::SetBool>("dynamixel_hardware_interface/set_dxl_torque");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FfwRobotManager::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Discover all GPIO devices and their error interfaces
  gpio_names_.clear();
  gpio_interface_indices_.clear();
  std::unordered_set<std::string> found_gpios;
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    const auto & si = state_interfaces_[i];
    const std::string & prefix = si.get_prefix_name();
    const std::string & interface = si.get_interface_name();
    // Heuristic: GPIOs are named like dxl1, dxl2, ...
    if (prefix.rfind("dxl", 0) == 0) {
      found_gpios.insert(prefix);
      if (interface == "Error Code" || interface == "Hardware Error Status") {
        gpio_interface_indices_[prefix][interface] = i;
      }
    }
  }
  gpio_names_.assign(found_gpios.begin(), found_gpios.end());
  std::sort(gpio_names_.begin(), gpio_names_.end());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FfwRobotManager::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  gpio_names_.clear();
  gpio_interface_indices_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FfwRobotManager::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & gpio : gpio_names_)
  {
    bool has_error = false;
    std::string error_details;

    // Check Error Code
    auto error_code_it = gpio_interface_indices_[gpio].find("Error Code");
    if (error_code_it != gpio_interface_indices_[gpio].end())
    {
      auto opt = state_interfaces_[error_code_it->second].get_optional();
      if (opt.has_value() && opt.value() != 0) {
        has_error = true;
        auto error_info = dynamixel_hardware_interface::get_error_code_info(static_cast<int>(opt.value()));
        if (error_info) {
          error_details += "Error Code: " + std::string(error_info->label) + " (" + error_info->description + ")";
        } else {
          error_details += "Error Code: Unknown error (" + std::to_string(static_cast<int>(opt.value())) + ")";
        }
      }
    }

    // Check Hardware Error Status
    auto hw_error_it = gpio_interface_indices_[gpio].find("Hardware Error Status");
    if (hw_error_it != gpio_interface_indices_[gpio].end())
    {
      auto opt = state_interfaces_[hw_error_it->second].get_optional();
      if (opt.has_value() && opt.value() != 0) {
        has_error = true;
        if (!error_details.empty()) error_details += "; ";
        error_details += "Hardware Error Status: ";

        int status_value = static_cast<int>(opt.value());
        bool first_bit = true;
        for (int bit = 0; bit < 8; ++bit) {
          if (status_value & (1 << bit)) {
            auto bit_info = dynamixel_hardware_interface::get_hardware_error_status_bit_info(bit);
            if (bit_info) {
              if (!first_bit) error_details += ", ";
              error_details += bit_info->label;
              first_bit = false;
            }
          }
        }
      }
    }

    // Log and disable torque if there are errors
    if (has_error) {
      // RCLCPP_WARN(get_node()->get_logger(), "GPIO '%s' has errors: %s", gpio.c_str(), error_details.c_str());

      // Disable torque for all Dynamixels if not already disabled
      if (!torque_disabled_) {
        disable_all_torque();
      }
    }
  }
  return controller_interface::return_type::OK;
}

void FfwRobotManager::disable_all_torque()
{
  if (!torque_client_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Torque service client not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false; // Disable torque

  auto future = torque_client_->async_send_request(request);

  // Wait for response with timeout
  if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_WARN(get_node()->get_logger(), "Successfully disabled torque for all Dynamixels due to errors");
      torque_disabled_ = true;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to disable torque for all Dynamixels: %s", response->message.c_str());
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Timeout while trying to disable torque for all Dynamixels");
  }
}

} // namespace ffw_robot_manager

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ffw_robot_manager::FfwRobotManager, controller_interface::ControllerInterface)