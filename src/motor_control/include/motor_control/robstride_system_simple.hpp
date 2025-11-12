#ifndef _ROBSTRIDE_SYSTEM_H_
#define _ROBSTRIDE_SYSTEM_H_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "can_msgs/msg/frame.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "motor_control/robstride.hpp"   // Your driver header

namespace motor_control {

/**
 * RobstrideSystemHardwareSimple
 * - Only supports **position** command interface (for JointGroupPositionController)
 * - Maps joint <-> motor using optional joint_signs (±1) and gear_ratios (>0)
 * - Uses PosPP (profile position) mode on activation
 */
class RobstrideSystemHardwareSimple : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobstrideSystemHardwareSimple)
  RobstrideSystemHardwareSimple() = default;
  ~RobstrideSystemHardwareSimple() override = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void rx_from_can_(const can_msgs::msg::Frame::SharedPtr msg);
  void set_run_mode_position_pp_();
  void stop_all_();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_from_can_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_to_can_;

  std::vector<std::shared_ptr<RobStrite_Motor>> motors_;

  // Parameters
  std::string can_rx_topic_{"/from_can_bus"};
  std::string can_tx_topic_{"/to_can_bus"};
  std::vector<uint8_t> motor_ids_;
  uint32_t can_timeout_counts_{0};

  // PosPP profile
  double vel_max_{1.0};   // rad/s
  double acc_set_{1.0};   // rad/s^2

  // Joints
  std::vector<std::string> joint_names_;
  std::vector<double> pos_rad_, vel_rad_s_;
  std::vector<double> cmd_pos_rad_;

  // Mapping helpers (optional)
  std::vector<double> joint_signs_;   // +1 or -1
  std::vector<double> gear_ratios_;   // motor = joint * gear

  std::mutex mtx_;

  bool pp_setup_done_{false};
  bool first_write_after_activate_{false};
};

} // namespace motor_control

#endif
