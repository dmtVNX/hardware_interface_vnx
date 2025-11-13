#ifndef MOTOR_CONTROL_HW_INTERFACE__ROBSTRIDE_SYSTEM_HPP_
#define MOTOR_CONTROL_HW_INTERFACE__ROBSTRIDE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

//  Include motor_control
#include "motor_control/quadruped_controller.hpp"

namespace motor_control_hw_interface
{

class RobstrideSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobstrideSystem)

  //  Lifecycle callbacks (correct signatures for ros2_control)
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  //  read/write with correct signatures
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  //  Motor controller instance
  std::shared_ptr<QuadrupedController> quadruped_controller_;
  std::shared_ptr<rclcpp::Node> node_;

  //  Joint states (position, velocity, effort)
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;

  //  Motor ID mapping
  std::vector<uint8_t> motor_ids_;
  
  //  Control parameters
  double kp_;
  double kd_;
};

}  // namespace motor_control_hw_interface

#endif  // MOTOR_CONTROL_HW_INTERFACE__ROBSTRIDE_SYSTEM_HPP_