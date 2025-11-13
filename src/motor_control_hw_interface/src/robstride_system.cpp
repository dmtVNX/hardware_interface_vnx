#include "motor_control_hw_interface/robstride_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motor_control_hw_interface
{

hardware_interface::CallbackReturn RobstrideSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Initializing hardware interface...");

  //  Resize vectors for 12 joints
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  //  Parse motor IDs from hardware parameters
  motor_ids_.clear();
  for (const auto & joint : info_.joints) {
    // Expected parameter: motor_id = "0x01", "0x02", etc.
    auto it = joint.parameters.find("motor_id");
    if (it != joint.parameters.end()) {
      uint8_t id = std::stoi(it->second, nullptr, 16);  // Parse hex string
      motor_ids_.push_back(id);
      RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), 
                  "Joint '%s' mapped to motor ID 0x%02X", 
                  joint.name.c_str(), id);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("RobstrideSystem"), 
                   "Joint '%s' missing motor_id parameter", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  //  Parse control gains (optional parameters)
  kp_ = 50.0;  // Default Kp
  kd_ = 5.0;   // Default Kd
  
  for (const auto & [key, value] : info_.hardware_parameters) {
    if (key == "kp") {
      kp_ = std::stod(value);
      RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Kp set to %.2f", kp_);
    } else if (key == "kd") {
      kd_ = std::stod(value);
      RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Kd set to %.2f", kd_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Hardware interface initialized");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Configuring hardware interface...");

  //  Create ROS2 node for motor controller
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=robstride_hw_node"});
  node_ = std::make_shared<rclcpp::Node>("robstride_hw_node", options);

  //  Initialize QuadrupedController
  try {
    quadruped_controller_ = std::make_shared<QuadrupedController>(node_);
    quadruped_controller_->initialize();
    
    RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Waiting for CAN bridges...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    //  Enable all motors
    quadruped_controller_->initializeAllMotors();
    
    RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "All 12 motors initialized");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RobstrideSystem"), 
                 "Failed to initialize motors: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  //  Initialize states to zero
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    hw_states_positions_[i] = 0.0;
    hw_states_velocities_[i] = 0.0;
    hw_states_efforts_[i] = 0.0;
    hw_commands_positions_[i] = 0.0;
    hw_commands_velocities_[i] = 0.0;
    hw_commands_efforts_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Hardware configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobstrideSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), 
              "Exported %zu state interfaces", state_interfaces.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobstrideSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), 
              "Exported %zu command interfaces", command_interfaces.size());
  return command_interfaces;
}

hardware_interface::CallbackReturn RobstrideSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Activating hardware interface...");

  //  Re-enable motors if needed
  try {
    for (uint8_t id : motor_ids_) {
      auto& motor = quadruped_controller_->getMotor(id);
      motor.Enable_Motor();
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      rclcpp::spin_some(node_);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RobstrideSystem"), "Activation failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Deactivating hardware interface...");

  //  Safely disable motors
  if (quadruped_controller_) {
    quadruped_controller_->shutdown();
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystem"), "Hardware deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobstrideSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //  Spin ROS2 callbacks to receive CAN feedback
  rclcpp::spin_some(node_);

  //  Read motor states
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    try {
      auto& motor = quadruped_controller_->getMotor(motor_ids_[i]);
      
      hw_states_positions_[i] = motor.Pos_Info.Angle;
      hw_states_velocities_[i] = motor.Pos_Info.Speed;
      hw_states_efforts_[i] = motor.Pos_Info.Torque;
      
    } catch (const std::exception& e) {
      // Don't spam logs
      static auto last_log = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
        RCLCPP_WARN(rclcpp::get_logger("RobstrideSystem"), 
                    "Failed to read motor 0x%02X: %s", motor_ids_[i], e.what());
        last_log = now;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //  Write commands to motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    try {
      auto& motor = quadruped_controller_->getMotor(motor_ids_[i]);

      //  Check for NaN commands (no command received)
      if (std::isnan(hw_commands_positions_[i]) && 
          std::isnan(hw_commands_velocities_[i]) && 
          std::isnan(hw_commands_efforts_[i])) {
        continue;  // Skip if no command
      }

      //  Use position control mode (MIT mode)
      float pos = std::isnan(hw_commands_positions_[i]) ? 
                  hw_states_positions_[i] : hw_commands_positions_[i];
      float vel = std::isnan(hw_commands_velocities_[i]) ? 
                  0.0f : hw_commands_velocities_[i];
      float torque = std::isnan(hw_commands_efforts_[i]) ? 
                     0.0f : hw_commands_efforts_[i];
      
      motor.RobStrite_Motor_move_control(
        torque,  // Feedforward torque
        pos,     // Target position
        vel,     // Feedforward velocity
        kp_,     // Kp gain
        kd_      // Kd gain
      );

      rclcpp::spin_some(node_);  // Process CAN send

    } catch (const std::exception& e) {
      static auto last_log = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
        RCLCPP_WARN(rclcpp::get_logger("RobstrideSystem"), 
                    "Failed to write motor 0x%02X: %s", motor_ids_[i], e.what());
        last_log = now;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace motor_control_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  motor_control_hw_interface::RobstrideSystem, hardware_interface::SystemInterface)