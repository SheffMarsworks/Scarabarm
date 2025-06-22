#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_control {

class OdriveSystem : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    return {};
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    return {};
  }

  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
    return hardware_interface::return_type::OK;
  }
};

} // namespace odrive_control

PLUGINLIB_EXPORT_CLASS(odrive_control::OdriveSystem, hardware_interface::SystemInterface)
