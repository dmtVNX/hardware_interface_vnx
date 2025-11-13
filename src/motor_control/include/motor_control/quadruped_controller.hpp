#ifndef QUADRUPED_CONTROLLER_HPP
#define QUADRUPED_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "motor_control/motor_config.hpp"
#include <map>
#include <string>
#include <vector>

enum class LegID {
    FRONT_LEFT = 0,   // FL
    FRONT_RIGHT = 1,  // FR
    REAR_LEFT = 2,    // RL
    REAR_RIGHT = 3    // RR
};

struct CANBusInfo {
    std::string can_interface;  // "can0", "can1", "can2", "can3"
    std::string topic_tx;       // "/to_can_bus_0"
    std::string topic_rx;       // "/from_can_bus_0"
    std::vector<uint8_t> motor_ids; // Motor IDs on this CAN bus
    LegID leg; // FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
    std::string leg_name;
};

struct MotorInfo {
    uint8_t id; //0x01, 0x02, ...
    std::string name; // "FL_hip", "FR_thigh", "RL_knee", ...
    LegID leg; // FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
    int bus_index; 
};

class QuadrupedController {
public:
    QuadrupedController(rclcpp::Node::SharedPtr node);
    
    void initialize();
    void initializeAllMotors();
    void emergencyShutdown();
    void shutdown();
    
    RobStrite_Motor& getMotor(uint8_t id);
    std::vector<RobStrite_Motor*> getLegMotors(LegID leg);
    
    void displayAllStatus();
    void displayLegStatus(LegID leg);
    
    const std::vector<CANBusInfo>& getCANBuses() const { return can_buses_; }
    const std::map<uint8_t, MotorInfo>& getMotorMap() const { return motor_map_; }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::vector<std::shared_ptr<MotorControlSet>> controllers_;
    std::vector<CANBusInfo> can_buses_;
    std::map<uint8_t, MotorInfo> motor_map_;
    
    void setupCANBuses();
    void buildMotorMap();
    int getCANBusIndex(uint8_t motor_id);
};

#endif // QUADRUPED_CONTROLLER_HPP