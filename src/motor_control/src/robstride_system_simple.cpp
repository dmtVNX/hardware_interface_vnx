#include "motor_control/robstride_system_simple.hpp"
#include <algorithm>
#include <chrono>

namespace motor_control {

hardware_interface::CallbackReturn RobstrideSystemHardwareSimple::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Collect joint names and ensure exactly ONE command interface per joint
  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(rclcpp::get_logger("RobstrideHW.Simple"),
                   "Joint '%s' must expose exactly ONE command interface: 'position'.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Parse parameters
  const auto & p = info_.hardware_parameters;
  if (p.count("can_rx_topic")) can_rx_topic_ = p.at("can_rx_topic");
  if (p.count("can_tx_topic")) can_tx_topic_ = p.at("can_tx_topic");
  if (p.count("can_timeout_counts")) can_timeout_counts_ = static_cast<uint32_t>(std::stoul(p.at("can_timeout_counts")));
  if (p.count("vel_max")) vel_max_ = std::stod(p.at("vel_max"));
  if (p.count("acc_set")) acc_set_ = std::stod(p.at("acc_set"));

  // Required: motor_ids (comma separated; hex like 0x10 is allowed)
  if (!p.count("motor_ids")) {
    RCLCPP_ERROR(rclcpp::get_logger("RobstrideHW.Simple"), "Missing required param 'motor_ids'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  {
    motor_ids_.clear();
    std::string s = p.at("motor_ids");
    size_t start = 0;
    while (start < s.size()) {
      size_t comma = s.find(',', start);
      std::string token = s.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
      // trim
      token.erase(0, token.find_first_not_of(" \t\n\r"));
      token.erase(token.find_last_not_of(" \t\n\r") + 1);
      if (token.empty()) break;
      if (token.rfind("0x", 0) == 0 || token.rfind("0X", 0) == 0)
        motor_ids_.push_back(static_cast<uint8_t>(std::stoul(token, nullptr, 16)));
      else
        motor_ids_.push_back(static_cast<uint8_t>(std::stoul(token, nullptr, 10)));
      if (comma == std::string::npos) break;
      start = comma + 1;
    }
  }
  if (motor_ids_.size() != joint_names_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("RobstrideHW.Simple"),
                 "motor_ids count (%zu) must match joints count (%zu)",
                 motor_ids_.size(), joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional mapping
  joint_signs_.assign(joint_names_.size(), 1.0);
  if (p.count("joint_signs")) {
    joint_signs_.clear();
    std::string s = p.at("joint_signs"); size_t start = 0;
    while (start < s.size()) {
      size_t comma = s.find(',', start);
      std::string token = s.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
      token.erase(0, token.find_first_not_of(" \t\n\r"));
      token.erase(token.find_last_not_of(" \t\n\r") + 1);
      if (!token.empty()) joint_signs_.push_back(std::stod(token));
      if (comma == std::string::npos) break;
      start = comma + 1;
    }
  }
  if (joint_signs_.size() != joint_names_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("RobstrideHW.Simple"),
                 "joint_signs count (%zu) must match joints count (%zu)", joint_signs_.size(), joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  gear_ratios_.assign(joint_names_.size(), 1.0);
  if (p.count("gear_ratios")) {
    gear_ratios_.clear();
    std::string s = p.at("gear_ratios"); size_t start = 0;
    while (start < s.size()) {
      size_t comma = s.find(',', start);
      std::string token = s.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
      token.erase(0, token.find_first_not_of(" \t\n\r"));
      token.erase(token.find_last_not_of(" \t\n\r") + 1);
      if (!token.empty()) gear_ratios_.push_back(std::stod(token));
      if (comma == std::string::npos) break;
      start = comma + 1;
    }
  }
  if (gear_ratios_.size() != joint_names_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("RobstrideHW.Simple"),
                 "gear_ratios count (%zu) must match joints count (%zu)", gear_ratios_.size(), joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Buffers
  size_t n = joint_names_.size();
  pos_rad_.assign(n, 0.0);
  vel_rad_s_.assign(n, 0.0);
  cmd_pos_rad_.assign(n, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobstrideSystemHardwareSimple::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_rad_[i]);
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_rad_s_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> RobstrideSystemHardwareSimple::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_pos_rad_[i]);
  }
  return ci;
}

hardware_interface::CallbackReturn RobstrideSystemHardwareSimple::on_configure(const rclcpp_lifecycle::State &) {
  node_ = std::make_shared<rclcpp::Node>("robstride_hw_simple");
  pub_to_can_ = node_->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, rclcpp::QoS(200));
  sub_from_can_ = node_->create_subscription<can_msgs::msg::Frame>(
      can_rx_topic_, rclcpp::QoS(200),
      std::bind(&RobstrideSystemHardwareSimple::rx_from_can_, this, std::placeholders::_1));

  // Build drivers
  motors_.clear();
  for (auto id : motor_ids_) {
    motors_.push_back(std::make_shared<RobStrite_Motor>(id, node_, can_tx_topic_));
  }

  // Optional watchdog
  if (can_timeout_counts_ > 0) {
    for (auto & m : motors_) {
      m->Set_RobStrite_Motor_parameter(0x7028, static_cast<float>(can_timeout_counts_), Set_parameter);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Configured Robstride HW (simple, %zu joints; POSITION mode)", joint_names_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RobstrideSystemHardwareSimple::set_run_mode_position_pp_() {
  RCLCPP_INFO(node_->get_logger(), "Setting motors to position PP mode");
  for (auto & m : motors_) {
    m->Disenable_Motor(1);
    m->Set_RobStrite_Motor_parameter(0x7005, PosPP_control_mode, Set_mode);
    m->Enable_Motor();
  }
}

hardware_interface::CallbackReturn RobstrideSystemHardwareSimple::on_activate(const rclcpp_lifecycle::State &) {
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  set_run_mode_position_pp_();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardwareSimple::on_deactivate(const rclcpp_lifecycle::State &) {
  stop_all_();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobstrideSystemHardwareSimple::read(const rclcpp::Time &, const rclcpp::Duration &) {
  std::scoped_lock<std::mutex> lk(mtx_);
  for (size_t i = 0; i < motors_.size(); ++i) {
    double mpos = motors_[i]->Pos_Info.Angle;
    double mvel = motors_[i]->Pos_Info.Speed;
    pos_rad_[i] = joint_signs_[i] * (mpos / gear_ratios_[i]);
    vel_rad_s_[i] = joint_signs_[i] * (mvel / gear_ratios_[i]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystemHardwareSimple::write(const rclcpp::Time &, const rclcpp::Duration &) {
  for (size_t i = 0; i < motors_.size(); ++i) {
    double tgt_motor = (cmd_pos_rad_[i] * joint_signs_[i]) * gear_ratios_[i];
    RCLCPP_INFO(node_->get_logger(), "Write to the motor: tgt_motor = %f", tgt_motor);
    motors_[i]->RobStrite_Motor_PosPP_control(
      static_cast<float>(vel_max_), static_cast<float>(acc_set_), static_cast<float>(tgt_motor));
  }
  return hardware_interface::return_type::OK;
}

void RobstrideSystemHardwareSimple::rx_from_can_(const can_msgs::msg::Frame::SharedPtr msg) {
  std::scoped_lock<std::mutex> lk(mtx_);
  for (auto & m : motors_) {
    m->RobStrite_Motor_Analysis(const_cast<uint8_t*>(msg->data.data()), msg->id);
  }
}

void RobstrideSystemHardwareSimple::stop_all_() {
  for (auto & m : motors_) {
    // In PP mode, send current position as target and disable
    m->RobStrite_Motor_PosPP_control(0.0f, 0.0f, static_cast<float>(m->Pos_Info.Angle));
    m->Disenable_Motor(1);
  }
}

} // namespace motor_control

PLUGINLIB_EXPORT_CLASS(motor_control::RobstrideSystemHardwareSimple, hardware_interface::SystemInterface)
