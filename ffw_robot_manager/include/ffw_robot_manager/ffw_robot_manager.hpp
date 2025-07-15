#ifndef FFW_ROBOT_MANAGER__FFW_ROBOT_MANAGER_HPP_
#define FFW_ROBOT_MANAGER__FFW_ROBOT_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <unordered_set>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "dynamixel_hardware_interface/dynamixel_hardware_interface.hpp"

namespace ffw_robot_manager
{

class FfwRobotManager : public controller_interface::ControllerInterface
{
public:
  FfwRobotManager();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> gpio_names_;
  std::unordered_map<std::string, std::unordered_map<std::string, size_t>> gpio_interface_indices_;

  // Service client for disabling Dynamixel torque
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_client_;

  // Track if torque has been disabled due to errors
  bool torque_disabled_ = false;

  // Disable torque for all Dynamixels
  void disable_all_torque();
};
}  // namespace ffw_robot_manager

#endif  // FFW_ROBOT_MANAGER__FFW_ROBOT_MANAGER_HPP_