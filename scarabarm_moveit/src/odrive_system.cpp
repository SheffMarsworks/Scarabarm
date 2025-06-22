#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <string>

namespace odrive_control {

class OdriveSystem : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    size_t num_joints = info_.joints.size();
    hw_positions_.resize(num_joints, 0.0);
    hw_velocities_.resize(num_joints, 0.0);
    hw_commands_.resize(num_joints, 0.0);
    hw_vel_cmds_.resize(num_joints, 0.0);
    joint_names_.reserve(num_joints);

    for (const auto & joint : info_.joints) {
      joint_names_.push_back(joint.name);
    }

    RCLCPP_INFO(rclcpp::get_logger("OdriveSystem"), "Initialized %zu joints.", num_joints);
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      state_interfaces.emplace_back(joint_names_[i], "position", &hw_positions_[i]);
      state_interfaces.emplace_back(joint_names_[i], "velocity", &hw_velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      command_interfaces.emplace_back(joint_names_[i], "position", &hw_commands_[i]);
      command_interfaces.emplace_back(joint_names_[i], "velocity", &hw_vel_cmds_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override {
    // No-op for now
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
    // In real implementation, convert hw_commands_ to CAN messages
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string &joint_name = joint_names_[i];
      double cmd_pos = hw_commands_[i];
      double cmd_vel = hw_vel_cmds_[i];

      // Example: log values instead of actual send
      RCLCPP_INFO(rclcpp::get_logger("OdriveSystem"),
                  "Joint: %s | Pos: %.3f | Vel: %.3f",
                  joint_name.c_str(), cmd_pos, cmd_vel);

      // TODO: Call external Python node via topic/service if needed
      // Or use a C++ SocketCAN wrapper to send the CAN frame here
    }

    return hardware_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_vel_cmds_;
};

} // namespace odrive_control

PLUGINLIB_EXPORT_CLASS(odrive_control::OdriveSystem, hardware_interface::SystemInterface)
