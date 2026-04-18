#include "autolife_hardware/autolife_system.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autolife_hardware
{

hardware_interface::CallbackReturn AutolifeSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  position_states_.assign(joint_names_.size(), 0.0);
  velocity_states_.assign(joint_names_.size(), 0.0);
  position_commands_.assign(joint_names_.size(), 0.0);
  velocity_commands_.assign(joint_names_.size(), 0.0);

  base_ip_ = info_.hardware_parameters["base_ip"];
  base_port_ = std::stoi(info_.hardware_parameters["base_port"]);
  arm_ip_ = info_.hardware_parameters["arm_ip"];
  use_mock_ = (info_.hardware_parameters["use_mock"] == "true");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AutolifeSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AutolifeSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  position_commands_ = position_states_;
  velocity_commands_.assign(joint_names_.size(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AutolifeSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AutolifeSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]);
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AutolifeSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type AutolifeSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (use_mock_) {
    position_states_ = position_commands_;
    velocity_states_ = velocity_commands_;
  } else {
    // TODO: 从真实硬件读取 joint position / velocity
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AutolifeSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (use_mock_) {
    return hardware_interface::return_type::OK;
  }

  // TODO: 把 position_commands_ / velocity_commands_ 发给真实硬件
  return hardware_interface::return_type::OK;
}

}  // namespace autolife_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  autolife_hardware::AutolifeSystem,
  hardware_interface::SystemInterface)
