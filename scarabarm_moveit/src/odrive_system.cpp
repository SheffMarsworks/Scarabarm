#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <string>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

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
    gear_ratios_.resize(num_joints, 1.0);
    joint_names_.reserve(num_joints);

    for (size_t i = 0; i < num_joints; ++i) {
      const auto &joint = info_.joints[i];
      joint_names_.push_back(joint.name);

      if (joint.name == "joint_2" || joint.name == "joint_3" || joint.name == "joint_4")
        gear_ratios_[i] = 64.0;
      else if (joint.name == "joint_5" || joint.name == "joint_6")
        gear_ratios_[i] = 10.0;
      else
        gear_ratios_[i] = 1.0;  // default or gripper
    }

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveSystem"), "Failed to open CAN socket");
      return CallbackReturn::ERROR;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveSystem"), "CAN ioctl failed");
      return CallbackReturn::ERROR;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveSystem"), "CAN bind failed");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("OdriveSystem"), "Initialized %zu joints and CAN socket.", num_joints);
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
    // TODO: Optionally read encoder feedback from CAN bus
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string &joint = joint_names_[i];
      double cmd_turns = hw_commands_[i];
      double ratio = gear_ratios_[i];
      int axis_id = get_axis_id(joint);
      if (axis_id < 0) continue;

      double motor_turns = cmd_turns * ratio;
      if (!send_odrive_position_command(axis_id, motor_turns)) {
        RCLCPP_ERROR(rclcpp::get_logger("OdriveSystem"),
                     "Failed to send pos to %s (axis %d)", joint.c_str(), axis_id);
      } else {
        RCLCPP_INFO_THROTTLE(
          rclcpp::get_logger("OdriveSystem"), logger_clock_, 1000,
          "Sent %.3f motor turns to %s (axis %d)", motor_turns, joint.c_str(), axis_id);
      }
    }

    return hardware_interface::return_type::OK;
  }

private:
  int socket_ = -1;
  std::vector<std::string> joint_names_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_vel_cmds_;
  std::vector<double> gear_ratios_;
  rclcpp::Clock logger_clock_{RCL_STEADY_TIME};

  int get_axis_id(const std::string& joint) {
    if (joint == "joint_2") return 2;
    if (joint == "joint_3") return 3;
    if (joint == "joint_4") return 4;
    if (joint == "joint_5") return 5;
    if (joint == "joint_6") return 6;
    return -1;
  }

  bool send_odrive_position_command(uint8_t axis_id, double turns) {
    const int32_t counts_per_rev = 8192;
    int32_t encoder_counts = static_cast<int32_t>(turns * counts_per_rev);

    struct can_frame frame{};
    frame.can_id = 0x00C | (axis_id << 5);  // Position command ID
    frame.can_dlc = 8;
    std::memcpy(frame.data, &encoder_counts, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);  // velocity_ff + current_ff

    return ::write(socket_, &frame, sizeof(frame)) == sizeof(frame);
  }
};

}  // namespace odrive_control

PLUGINLIB_EXPORT_CLASS(odrive_control::OdriveSystem, hardware_interface::SystemInterface)
