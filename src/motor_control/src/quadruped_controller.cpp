#include "motor_control/quadruped_controller.hpp"
#include <iostream>
#include <iomanip>
#include <stdexcept>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define CYAN    "\033[36m"
#define BLUE    "\033[34m"

QuadrupedController::QuadrupedController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    setupCANBuses();
    buildMotorMap();
}

void QuadrupedController::setupCANBuses()
{
    // CAN0 - Front Left Leg (FL)
    can_buses_.push_back({
        "can0",
        "/to_can_bus_0",
        "/from_can_bus_0",
        {0x01, 0x02, 0x03},
        LegID::FRONT_LEFT,
        "FL"
    });
    
    // CAN1 - Front Right Leg (FR)
    can_buses_.push_back({
        "can1",
        "/to_can_bus_1",
        "/from_can_bus_1",
        {0x04, 0x05, 0x06},
        LegID::FRONT_RIGHT,
        "FR"
    });
    
    // CAN2 - Rear Left Leg (RL)
    can_buses_.push_back({
        "can2",
        "/to_can_bus_2",
        "/from_can_bus_2",
        {0x07, 0x08, 0x09},
        LegID::REAR_LEFT,
        "RL"
    });
    
    // CAN3 - Rear Right Leg (RR)
    can_buses_.push_back({
        "can3",
        "/to_can_bus_3",
        "/from_can_bus_3",
        {0x0A, 0x0B, 0x0C},
        LegID::REAR_RIGHT,
        "RR"
    });
}

void QuadrupedController::buildMotorMap()
{
    const char* joint_names[] = {"hip", "thigh", "calf"};
    
    for (size_t bus_idx = 0; bus_idx < can_buses_.size(); bus_idx++) {
        const auto& bus = can_buses_[bus_idx];
        
        for (size_t motor_idx = 0; motor_idx < bus.motor_ids.size(); motor_idx++) {
            uint8_t id = bus.motor_ids[motor_idx];
            
            MotorInfo info;
            info.id = id;
            info.name = bus.leg_name + std::string("_") + joint_names[motor_idx];
            info.leg = bus.leg;
            info.bus_index = bus_idx;
            
            motor_map_[id] = info;
        }
    }
}

void QuadrupedController::initialize()
{
    RCLCPP_INFO(node_->get_logger(), "Initializing Quadruped Controller");
    RCLCPP_INFO(node_->get_logger(), "   12 motors on 4 CAN buses");
    
    for (size_t i = 0; i < can_buses_.size(); i++) {
        const auto& bus = can_buses_[i];
        
        RCLCPP_INFO(node_->get_logger(), "Setting up %s (%s leg):",
                    bus.can_interface.c_str(), bus.leg_name.c_str());
        RCLCPP_INFO(node_->get_logger(), "  RX topic: %s", bus.topic_rx.c_str());
        RCLCPP_INFO(node_->get_logger(), "  TX topic: %s", bus.topic_tx.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Motors: 0x%02X, 0x%02X, 0x%02X",
                    bus.motor_ids[0], bus.motor_ids[1], bus.motor_ids[2]);
        
        auto controller = std::make_shared<MotorControlSet>(
            node_,
            bus.topic_rx,
            bus.topic_tx,
            bus.motor_ids
        );
        
        controllers_.push_back(controller);
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(node_->get_logger(), "✅ All 4 CAN buses initialized");
}

int QuadrupedController::getCANBusIndex(uint8_t motor_id)
{
    auto it = motor_map_.find(motor_id);
    if (it == motor_map_.end()) {
        throw std::runtime_error("Invalid motor ID: 0x" + 
            std::to_string(motor_id));
    }
    return it->second.bus_index;
}

RobStrite_Motor& QuadrupedController::getMotor(uint8_t id)
{
    int bus_idx = getCANBusIndex(id);
    return controllers_[bus_idx]->getMotor(id);
}

std::vector<RobStrite_Motor*> QuadrupedController::getLegMotors(LegID leg)
{
    std::vector<RobStrite_Motor*> motors;
    int leg_idx = static_cast<int>(leg);
    
    for (uint8_t id : can_buses_[leg_idx].motor_ids) {
        motors.push_back(&getMotor(id));
    }
    
    return motors;
}

void QuadrupedController::initializeAllMotors()
{
    std::cout << YELLOW << "Initializing all 12 motors...\n" << RESET;
    
    for (const auto& [motor_id, info] : motor_map_) {
        auto& motor = getMotor(motor_id);
        
        std::cout << "  Initializing " << info.name 
                  << " (0x" << std::hex << (int)motor_id << std::dec << ")...\n";
        
        motor.Disenable_Motor(1);
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        motor.Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        motor.Enable_Motor();
        rclcpp::spin_some(node_);
        
        std::cout << GREEN << "    ✅ " << info.name << " enabled\n" << RESET;
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    
    std::cout << GREEN << "\n✅ All 12 motors initialized!\n" << RESET;
}

void QuadrupedController::emergencyShutdown()
{
    std::cout << RED << "\n⚠️  EMERGENCY SHUTDOWN - Disabling all 12 motors...\n" << RESET;
    
    for (const auto& [motor_id, info] : motor_map_) {
        try {
            auto& motor = getMotor(motor_id);
            motor.Disenable_Motor(0);
            rclcpp::spin_some(node_);
            std::cout << "  " << info.name << " emergency stopped\n";
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        } catch (...) {}
    }
    
    std::cout << GREEN << "✅ Emergency shutdown complete\n" << RESET;
}

void QuadrupedController::shutdown()
{
    std::cout << YELLOW << "\n🛑 Shutting down all 12 motors safely...\n" << RESET;
    
    for (const auto& [motor_id, info] : motor_map_) {
        auto& motor = getMotor(motor_id);
        motor.Disenable_Motor(0);
        rclcpp::spin_some(node_);
        std::cout << "  " << info.name << " disabled\n";
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << GREEN << "✅ All motors safely disabled\n" << RESET;
}

void QuadrupedController::displayAllStatus()
{
    std::cout << "\n" << CYAN << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              QUADRUPED - ALL MOTORS STATUS (12 motors)            ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n" << RESET;
    
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    
    for (size_t leg_idx = 0; leg_idx < 4; leg_idx++) {
        std::cout << "\n" << YELLOW << ">>> " << can_buses_[leg_idx].leg_name 
                  << " Leg (CAN" << leg_idx << ") <<<\n" << RESET;
        displayLegStatus(static_cast<LegID>(leg_idx));
    }
}

void QuadrupedController::displayLegStatus(LegID leg)
{
    auto motors = getLegMotors(leg);
    int leg_idx = static_cast<int>(leg);
    
    std::cout << std::setw(12) << "Motor"
              << std::setw(12) << "Pos(rad)"
              << std::setw(12) << "Vel(r/s)"
              << std::setw(12) << "Tor(Nm)"
              << std::setw(10) << "Temp(°C)\n";
    std::cout << std::string(58, '-') << "\n";
    
    for (size_t i = 0; i < motors.size(); i++) {
        uint8_t id = can_buses_[leg_idx].motor_ids[i];
        auto it = motor_map_.find(id);
        auto* motor = motors[i];
        
        std::cout << "  " << std::setw(10) << std::left << it->second.name << std::right
                  << std::setw(12) << std::fixed << std::setprecision(3) << motor->Pos_Info.Angle
                  << std::setw(12) << std::setprecision(3) << motor->Pos_Info.Speed
                  << std::setw(12) << std::setprecision(3) << motor->Pos_Info.Torque
                  << std::setw(10) << std::setprecision(1) << motor->Pos_Info.Temp << "\n";
    }
}