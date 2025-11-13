#include <rclcpp/rclcpp.hpp>
#include "motor_control/quadruped_controller.hpp"
#include <iostream>
#include <string>
#include <limits>
#include <iomanip>
#include <signal.h>
#include <atomic>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"

std::atomic<bool> g_shutdown_requested{false};

class QuadrupedTerminal;
static QuadrupedTerminal* g_node_ptr = nullptr;

class QuadrupedTerminal : public rclcpp::Node
{
private:
    std::shared_ptr<QuadrupedController> controller_;

public:
    QuadrupedTerminal() : Node("quadruped_terminal")
    {
        RCLCPP_INFO(this->get_logger(), "🦾 Starting Quadruped Control Terminal");
        g_node_ptr = this;
    }

    void initialize()
    {
        controller_ = std::make_shared<QuadrupedController>(shared_from_this());
        controller_->initialize();
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        controller_->initializeAllMotors();
        
        std::cout << GREEN << "\n✅ All 12 motors ready!\n" << RESET;
    }

    void emergencyShutdown()
    {
        if (controller_) {
            controller_->emergencyShutdown();
        }
    }

    void shutdown()
    {
        if (controller_) {
            controller_->shutdown();
        }
    }

    ~QuadrupedTerminal()
    {
        RCLCPP_INFO(this->get_logger(), "Destructor called");
        g_node_ptr = nullptr;
    }

    void clearScreen() { system("clear"); }
    
    void clearInputBuffer()
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    void printHeader()
    {
        std::cout << BOLD << CYAN << "\n";
        std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║      🦾 QUADRUPED ROBOT CONTROL - VNX 2025 🦾                  ║\n";
        std::cout << "║            Testing 12 Motors on 4 CAN Buses                    ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
        std::cout << RESET << "\n";
    }

    int getIntInput(const std::string& prompt, int min_val, int max_val)
    {
        int value;
        while (true)
        {
            std::cout << prompt;
            
            if (std::cin.eof() || g_shutdown_requested) {
                throw std::runtime_error("Input interrupted");
            }
            
            if (std::cin >> value && value >= min_val && value <= max_val)
            {
                clearInputBuffer();
                return value;
            }
            else
            {
                std::cout << RED << "❌ Invalid! Enter " << min_val << "-" << max_val << "\n" << RESET;
                clearInputBuffer();
            }
        }
    }

    float getFloatInput(const std::string& prompt, float min_val, float max_val)
    {
        float value;
        while (true)
        {
            std::cout << prompt;
            
            if (std::cin.eof() || g_shutdown_requested) {
                throw std::runtime_error("Input interrupted");
            }
            
            if (std::cin >> value && value >= min_val && value <= max_val)
            {
                clearInputBuffer();
                return value;
            }
            else
            {
                std::cout << RED << "❌ Invalid! Enter " << min_val << "-" << max_val << "\n" << RESET;
                clearInputBuffer();
            }
        }
    }

    int selectMainMenu()
    {
        std::cout << "\n" << BOLD << YELLOW << "═══ MAIN MENU ═══\n" << RESET;
        std::cout << GREEN << "1." << RESET << " Control ALL motors (12 motors)\n";
        std::cout << GREEN << "2." << RESET << " Control by LEG (FL/FR/RL/RR)\n";
        std::cout << GREEN << "3." << RESET << " Control SINGLE motor (by ID)\n";
        std::cout << BLUE << "4." << RESET << " Display All Motors Status\n";
        std::cout << BLUE << "5." << RESET << " Display Leg Status\n";
        std::cout << RED << "0." << RESET << " Exit\n\n";
        
        return getIntInput(CYAN "➜ Choice: " RESET, 0, 5);
    }

    int selectControlMode()
    {
        std::cout << "\n" << BOLD << YELLOW << "═══ CONTROL MODE ═══\n" << RESET;
        std::cout << GREEN << "1." << RESET << " Motion Control (MIT)\n";
        std::cout << GREEN << "2." << RESET << " Speed Control\n";
        std::cout << GREEN << "3." << RESET << " Position Profile\n";
        std::cout << GREEN << "4." << RESET << " Current Control\n";
        
        return getIntInput(CYAN "➜ Mode: " RESET, 1, 4);
    }

    LegID selectLeg()
    {
        std::cout << "\n" << BOLD << YELLOW << "═══ SELECT LEG ═══\n" << RESET;
        const auto& buses = controller_->getCANBuses();
        for (size_t i = 0; i < 4; i++) {
            std::cout << GREEN << i << "." << RESET << " " << buses[i].leg_name 
                      << " (0x" << std::hex << (int)buses[i].motor_ids[0] 
                      << ", 0x" << (int)buses[i].motor_ids[1]
                      << ", 0x" << (int)buses[i].motor_ids[2] 
                      << std::dec << ")\n";
        }
        
        return static_cast<LegID>(getIntInput(CYAN "➜ Leg: " RESET, 0, 3));
    }

    uint8_t selectMotor()
    {
        std::cout << "\n" << BOLD << YELLOW << "═══ SELECT MOTOR ═══\n" << RESET;
        const auto& motor_map = controller_->getMotorMap();
        for (const auto& [id, info] : motor_map) {
            std::cout << "  0x" << std::hex << std::setw(2) << std::setfill('0') 
                      << (int)id << std::dec << std::setfill(' ') 
                      << " - " << info.name << "\n";
        }
        
        int id = getIntInput(CYAN "➜ Motor ID (1-12): " RESET, 1, 12);
        return static_cast<uint8_t>(id);
    }

    void switchModeAll(int mode)
    {
        std::cout << BLUE << "\n🔄 Switching all 12 motors...\n" << RESET;
        
        const auto& motor_map = controller_->getMotorMap();
        for (const auto& [id, info] : motor_map) {
            if (g_shutdown_requested) return;
            
            auto& motor = controller_->getMotor(id);
            
            motor.Disenable_Motor(1);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            motor.Set_RobStrite_Motor_parameter(0X7005, mode, Set_mode);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            motor.Enable_Motor();
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            std::cout << "  " << info.name << " OK\n";
        }
        
        std::cout << GREEN << "✅ Mode switched\n" << RESET;
    }

    void switchModeLeg(LegID leg, int mode)
    {
        std::cout << BLUE << "\n🔄 Switching leg motors...\n" << RESET;
        
        auto motors = controller_->getLegMotors(leg);
        const auto& buses = controller_->getCANBuses();
        const auto& motor_map = controller_->getMotorMap();
        int leg_idx = static_cast<int>(leg);
        
        for (size_t i = 0; i < motors.size(); i++) {
            if (g_shutdown_requested) return;
            
            uint8_t id = buses[leg_idx].motor_ids[i];
            auto* motor = motors[i];
            auto it = motor_map.find(id);
            
            motor->Disenable_Motor(1);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            motor->Set_RobStrite_Motor_parameter(0X7005, mode, Set_mode);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            motor->Enable_Motor();
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            std::cout << "  " << it->second.name << " OK\n";
        }
        
        std::cout << GREEN << "✅ Mode switched\n" << RESET;
    }

    void switchModeSingle(uint8_t motor_id, int mode)
    {
        auto& motor = controller_->getMotor(motor_id);
        
        motor.Disenable_Motor(1);
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        motor.Set_RobStrite_Motor_parameter(0X7005, mode, Set_mode);
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        motor.Enable_Motor();
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << GREEN << "✅ Mode switched\n" << RESET;
    }

    void controlAll()
    {
        int mode = selectControlMode();
        switchModeAll(mode);
        
        if (mode == 1) {  // Motion Control
            float t = getFloatInput(YELLOW "Torque [-60→60]: " RESET, -60.0f, 60.0f);
            float a = getFloatInput(YELLOW "Angle [-12.57→12.57]: " RESET, -12.57f, 12.57f);
            float v = getFloatInput(YELLOW "Speed [-20→20]: " RESET, -20.0f, 20.0f);
            float kp = getFloatInput(YELLOW "Kp [0→500]: " RESET, 0.0f, 500.0f);
            float kd = getFloatInput(YELLOW "Kd [0→100]: " RESET, 0.0f, 100.0f);
            
            const auto& motor_map = controller_->getMotorMap();
            for (const auto& [id, info] : motor_map) {
                controller_->getMotor(id).RobStrite_Motor_move_control(t, a, v, kp, kd);
                rclcpp::spin_some(shared_from_this());
                std::cout << "  " << info.name << " commanded\n";
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else if (mode == 2) {  // Speed Control
            float cl = getFloatInput(YELLOW "Current limit [0→43]: " RESET, 0.0f, 43.0f);
            float ac = getFloatInput(YELLOW "Acceleration [0→50]: " RESET, 0.0f, 50.0f);
            float sp = getFloatInput(YELLOW "Speed [-20→20]: " RESET, -20.0f, 20.0f);
            
            const auto& motor_map = controller_->getMotorMap();
            for (const auto& [id, info] : motor_map) {
                controller_->getMotor(id).RobStrite_Motor_Speed_control(cl, ac, sp);
                rclcpp::spin_some(shared_from_this());
                std::cout << "  " << info.name << " commanded\n";
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else if (mode == 3) {  // Position Profile
            float sp = getFloatInput(YELLOW "Speed [0→20]: " RESET, 0.0f, 20.0f);
            float ac = getFloatInput(YELLOW "Acceleration [0→50]: " RESET, 0.0f, 50.0f);
            float ps = getFloatInput(YELLOW "Position [-12.57→12.57]: " RESET, -12.57f, 12.57f);
            
            const auto& motor_map = controller_->getMotorMap();
            for (const auto& [id, info] : motor_map) {
                controller_->getMotor(id).RobStrite_Motor_PosPP_control(sp, ac, ps);
                rclcpp::spin_some(shared_from_this());
                std::cout << "  " << info.name << " commanded\n";
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else if (mode == 4) {  // Current Control
            float iq = getFloatInput(YELLOW "Iq [-43→43]: " RESET, -43.0f, 43.0f);
            float id = getFloatInput(YELLOW "Id [-43→43]: " RESET, -43.0f, 43.0f);
            
            const auto& motor_map = controller_->getMotorMap();
            for (const auto& [mid, info] : motor_map) {
                controller_->getMotor(mid).RobStrite_Motor_Current_control(iq, id);
                rclcpp::spin_some(shared_from_this());
                std::cout << "  " << info.name << " commanded\n";
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        controller_->displayAllStatus();
    }

    void controlLeg()
    {
        LegID leg = selectLeg();
        int mode = selectControlMode();
        switchModeLeg(leg, mode);
        
        if (mode == 1) { // Motion Control
            float t = getFloatInput(YELLOW "Torque [-60→60]: " RESET, -60.0f, 60.0f);
            float a = getFloatInput(YELLOW "Angle [-12.57→12.57]: " RESET, -12.57f, 12.57f);
            float v = getFloatInput(YELLOW "Speed [-20→20]: " RESET, -20.0f, 20.0f);
            float kp = getFloatInput(YELLOW "Kp [0→500]: " RESET, 0.0f, 500.0f);
            float kd = getFloatInput(YELLOW "Kd [0→100]: " RESET, 0.0f, 100.0f);
            
            auto motors = controller_->getLegMotors(leg);
            for (auto* motor : motors) {
                motor->RobStrite_Motor_move_control(t, a, v, kp, kd);
                rclcpp::spin_some(shared_from_this());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else if (mode == 2) {  // Speed Control
            float cl = getFloatInput(YELLOW "Current limit [0→43]: " RESET, 0.0f, 43.0f);
            float ac = getFloatInput(YELLOW "Acceleration [0→50]: " RESET, 0.0f, 50.0f);
            float sp = getFloatInput(YELLOW "Speed [-20→20]: " RESET, -20.0f, 20.0f);
            
            auto motors = controller_->getLegMotors(leg);
            for (auto* motor : motors) {
                motor->RobStrite_Motor_Speed_control(cl, ac, sp);
                rclcpp::spin_some(shared_from_this());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else if (mode == 3) {  // Position Profile
            float sp = getFloatInput(YELLOW "Speed [0→20]: " RESET, 0.0f, 20.0f);
            float ac = getFloatInput(YELLOW "Acceleration [0→50]: " RESET, 0.0f, 50.0f);
            float ps = getFloatInput(YELLOW "Position [-12.57→12.57]: " RESET, -12.57f, 12.57f);
            
            auto motors = controller_->getLegMotors(leg);
            for (auto* motor : motors) {
                motor->RobStrite_Motor_PosPP_control(sp, ac, ps);
                rclcpp::spin_some(shared_from_this());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else if (mode == 4) {  // Current Control
            float iq = getFloatInput(YELLOW "Iq [-43→43]: " RESET, -43.0f, 43.0f);
            float id = getFloatInput(YELLOW "Id [-43→43]: " RESET, -43.0f, 43.0f);
            
            auto motors = controller_->getLegMotors(leg);
            for (auto* motor : motors) {
                motor->RobStrite_Motor_Current_control(iq, id);
                rclcpp::spin_some(shared_from_this());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        controller_->displayLegStatus(leg);
    }

    void controlSingle()
    {
        uint8_t id = selectMotor();
        int mode = selectControlMode();
        switchModeSingle(id, mode);
        
        if (mode == 1) { // Motion Control
            float t = getFloatInput(YELLOW "Torque [-60→60]: " RESET, -60.0f, 60.0f);
            float a = getFloatInput(YELLOW "Angle [-12.57→12.57]: " RESET, -12.57f, 12.57f);
            float v = getFloatInput(YELLOW "Speed [-20→20]: " RESET, -20.0f, 20.0f);
            float kp = getFloatInput(YELLOW "Kp [0→500]: " RESET, 0.0f, 500.0f);
            float kd = getFloatInput(YELLOW "Kd [0→100]: " RESET, 0.0f, 100.0f);
            
            controller_->getMotor(id).RobStrite_Motor_move_control(t, a, v, kp, kd);
            rclcpp::spin_some(shared_from_this());
        }
        else if (mode == 2) {  // Speed Control
            float cl = getFloatInput(YELLOW "Current limit [0→43]: " RESET, 0.0f, 43.0f);
            float ac = getFloatInput(YELLOW "Acceleration [0→50]: " RESET, 0.0f, 50.0f);
            float sp = getFloatInput(YELLOW "Speed [-20→20]: " RESET, -20.0f, 20.0f);
            
            controller_->getMotor(id).RobStrite_Motor_Speed_control(cl, ac, sp);
            rclcpp::spin_some(shared_from_this());
        }
        else if (mode == 3) {  // Position Profile
            float sp = getFloatInput(YELLOW "Speed [0→20]: " RESET, 0.0f, 20.0f);
            float ac = getFloatInput(YELLOW "Acceleration [0→50]: " RESET, 0.0f, 50.0f);
            float ps = getFloatInput(YELLOW "Position [-12.57→12.57]: " RESET, -12.57f, 12.57f);
            
            controller_->getMotor(id).RobStrite_Motor_PosPP_control(sp, ac, ps);
            rclcpp::spin_some(shared_from_this());
        }
        else if (mode == 4) {  // Current Control
            float iq = getFloatInput(YELLOW "Iq [-43→43]: " RESET, -43.0f, 43.0f);
            float id = getFloatInput(YELLOW "Id [-43→43]: " RESET, -43.0f, 43.0f);
            
            controller_->getMotor(id).RobStrite_Motor_Current_control(iq, id);
            rclcpp::spin_some(shared_from_this());
        }
        
        std::cout << GREEN << "✅ Command sent\n" << RESET;
    }

    void run()
    {
        while (rclcpp::ok() && !g_shutdown_requested)
        {
            clearScreen();
            printHeader();
            
            std::cout << BOLD << CYAN << "CAN Buses:\n" << RESET;
            const auto& buses = controller_->getCANBuses();
            for (const auto& bus : buses) {
                std::cout << "  " << bus.can_interface << " (" << bus.leg_name << "): 0x" 
                          << std::hex << (int)bus.motor_ids[0] << ", 0x" 
                          << (int)bus.motor_ids[1] << ", 0x" 
                          << (int)bus.motor_ids[2] << std::dec << "\n";
            }
            
            int choice;
            try {
                choice = selectMainMenu();
            } catch (const std::exception& e) {
                break;
            }
            
            if (choice == 0) {
                std::cout << RED << "\n👋 Exiting...\n" << RESET;
                break;
            }
            
            try {
                switch (choice) {
                    case 1: controlAll(); break;
                    case 2: controlLeg(); break;
                    case 3: controlSingle(); break;
                    case 4: controller_->displayAllStatus(); break;
                    case 5: controller_->displayLegStatus(selectLeg()); break;
                }
            }
            catch (const std::exception& e) {
                if (g_shutdown_requested) break;
                std::cout << RED << "\n❌ Error: " << e.what() << "\n" << RESET;
            }
            
            if (!g_shutdown_requested) {
                std::cout << "\nPress Enter...";
                std::cin.get();
            }
        }
    }
};

void signalHandler(int signum)
{
    std::cout << "\n" << YELLOW << "⚠️  Shutdown (" << signum << ")\n" << RESET;
    g_shutdown_requested = true;
    
    if (g_node_ptr) {
        try {
            g_node_ptr->emergencyShutdown();
        } catch (...) {}
    }
    
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<QuadrupedTerminal>();
        node->initialize();
        
        std::cout << GREEN << "\n✅ System ready!\n" << RESET;
        std::cout << YELLOW << "Press Enter to start..." << RESET;
        std::cin.get();
        
        node->run();
        
        if (!g_shutdown_requested) {
            node->shutdown();
        }
    }
    catch (const std::exception& e) {
        std::cerr << RED << "❌ Error: " << e.what() << RESET << "\n";
    }
    
    g_node_ptr = nullptr;
    rclcpp::shutdown();
    return 0;
}