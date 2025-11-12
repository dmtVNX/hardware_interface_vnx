#include <rclcpp/rclcpp.hpp>
#include "motor_control/motor_config.hpp"
#include <iostream>
#include <string>
#include <limits>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <signal.h>
#include <atomic>

// ANSI Color codes
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"

std::atomic<bool> g_shutdown_requested{false};

// Forward declaration
class MultiMotorControlTerminal;
static MultiMotorControlTerminal* g_node_ptr = nullptr;

class MultiMotorControlTerminal : public rclcpp::Node
{
private:
    std::shared_ptr<MotorControlSet> motor_controller_;
    std::vector<uint8_t> motor_ids_ = {0x01, 0x02, 0x03};

public:
    MultiMotorControlTerminal() : Node("multi_motor_control_terminal")
    {
        RCLCPP_INFO(this->get_logger(), "🚀 Starting Multi-Motor Control Terminal");
        g_node_ptr = this; 
    }

    void initialize()
    {
        // Initialize motor controller
        motor_controller_ = std::make_shared<MotorControlSet>(
            shared_from_this(),  
            "/from_can_bus",
            "/to_can_bus",
            motor_ids_
        );
        
        RCLCPP_INFO(this->get_logger(), "✅ Motor controller initialized");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        
        // Initialize all motors
        initializeMotors();
        
        std::cout << GREEN << "\n✅ All 3 motors ready for testing\n" << RESET;
    }

    void emergencyShutdown()
    {
        std::cout << RED << "\n⚠️  EMERGENCY SHUTDOWN - Disabling all motors...\n" << RESET;
        if (motor_controller_) {
            for(size_t i = 0; i < motor_ids_.size(); i++) {
                try {
                    motor_controller_->getMotor(motor_ids_[i]).Disenable_Motor(0);
                    rclcpp::spin_some(shared_from_this());
                    std::cout << "  Motor 0x" << std::hex << (int)motor_ids_[i] 
                              << std::dec << " emergency stopped\n";
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                } catch (...) {
                    // Ignore errors during emergency shutdown
                }
            }
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            std::cout << GREEN << "✅ Emergency shutdown complete\n" << RESET;
        }
    }

    void shutdown()
    {
        std::cout << YELLOW << "\n🛑 Shutting down all motors safely...\n" << RESET;
        if (motor_controller_) {
            for(size_t i = 0; i < motor_ids_.size(); i++) {
                motor_controller_->getMotor(motor_ids_[i]).Disenable_Motor(0);
                rclcpp::spin_some(shared_from_this());
                std::cout << "  Motor 0x" << std::hex << (int)motor_ids_[i] 
                          << std::dec << " disabled\n";
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            rclcpp::sleep_for(std::chrono::milliseconds(300));
            std::cout << GREEN << "✅ All motors safely disabled\n" << RESET;
        }
    }

    ~MultiMotorControlTerminal()
    {
        RCLCPP_INFO(this->get_logger(), "Destructor called");
        g_node_ptr = nullptr; 
    }

    void initializeMotors()
    {
        std::cout << YELLOW << "Initializing motors...\n" << RESET;
        
        for(size_t i = 0; i < motor_ids_.size(); i++) {
            auto& motor = motor_controller_->getMotor(motor_ids_[i]);
            
            motor.Disenable_Motor(1);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            
            motor.Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            
            motor.Enable_Motor();
            rclcpp::spin_some(shared_from_this());
            RCLCPP_INFO(this->get_logger(), "✅ Motor 0x%02X enabled", motor_ids_[i]);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
    }

    void clearScreen()
    {
        #ifdef _WIN32
            system("cls");
        #else
            system("clear");
        #endif
    }

    void clearInputBuffer()
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    void printHeader()
    {
        std::cout << BOLD << CYAN << "\n";
        std::cout << "╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║       🎮 MULTI-MOTOR CONTROL TERMINAL - VNX 2025 🎮        ║\n";
        std::cout << "║              Testing 3 Motors Simultaneously               ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n";
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
            
            if (std::cin >> value)
            {
                if (value >= min_val && value <= max_val)
                {
                    clearInputBuffer();
                    return value;
                }
                else
                {
                    std::cout << RED << "❌ Value must be between " << min_val 
                              << " and " << max_val << RESET << "\n";
                }
            }
            else
            {
                std::cout << RED << "❌ Invalid input! Please enter a number.\n" << RESET;
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
            
            if (std::cin >> value)
            {
                if (value >= min_val && value <= max_val)
                {
                    clearInputBuffer();
                    return value;
                }
                else
                {
                    std::cout << RED << "❌ Value must be between " << min_val 
                              << " and " << max_val << RESET << "\n";
                }
            }
            else
            {
                std::cout << RED << "❌ Invalid input! Please enter a number.\n" << RESET;
                clearInputBuffer();
            }
        }
    }

    int selectMode()
    {
        std::cout << "\n" << BOLD << YELLOW << "═══ SELECT CONTROL MODE ═══\n" << RESET;
        std::cout << GREEN << "1." << RESET << " Motion Control (MIT mode)\n";
        std::cout << GREEN << "2." << RESET << " Speed Control\n";
        std::cout << GREEN << "3." << RESET << " Position Profile Control\n";
        std::cout << GREEN << "4." << RESET << " Current Control\n";
        std::cout << BLUE << "5." << RESET << " Display Motors Status\n";
        std::cout << RED << "0." << RESET << " Exit program\n\n";
        
        return getIntInput(CYAN "➜ Enter choice: " RESET, 0, 5);
    }

    void switchModeAllMotors(int mode, const std::string& mode_name)
    {
        std::cout << BLUE << "\n🔄 Switching all motors to " << mode_name << "...\n" << RESET;
        
        for(size_t i = 0; i < motor_ids_.size(); i++) {
            if (g_shutdown_requested) return; 
            
            auto& motor = motor_controller_->getMotor(motor_ids_[i]);
            
            motor.Disenable_Motor(1);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            
            motor.Set_RobStrite_Motor_parameter(0X7005, mode, Set_mode);
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            
            motor.Enable_Motor();
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << GREEN << "✅ Mode switched successfully\n" << RESET;
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }

    void motionControlMode()
    {
        std::cout << "\n" << BOLD << MAGENTA << "═══ MOTION CONTROL MODE (MIT) ═══\n" << RESET;
        
        float torque = getFloatInput(YELLOW "Torque (Nm) [-60 → 60]: " RESET, -60.0f, 60.0f);
        float angle = getFloatInput(YELLOW "Angle (rad) [-12.57 → 12.57]: " RESET, -12.57f, 12.57f);
        float speed = getFloatInput(YELLOW "Speed (rad/s) [-20 → 20]: " RESET, -20.0f, 20.0f);
        float kp = getFloatInput(YELLOW "Kp [0 → 500]: " RESET, 0.0f, 500.0f);
        float kd = getFloatInput(YELLOW "Kd [0 → 100]: " RESET, 0.0f, 100.0f);
        
        switchModeAllMotors(move_control_mode, "Motion Control");
        
        std::cout << CYAN << "\n📍 Sending command to all motors...\n" << RESET;
        for(size_t i = 0; i < motor_ids_.size(); i++) {
            motor_controller_->getMotor(motor_ids_[i]).RobStrite_Motor_move_control(torque, angle, speed, kp, kd);
            rclcpp::spin_some(shared_from_this());
            std::cout << "  Motor 0x" << std::hex << (int)motor_ids_[i] << std::dec << " commanded\n";
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        
        monitorMotors(3);
    }

    void speedControlMode()
    {
        std::cout << "\n" << BOLD << MAGENTA << "═══ SPEED CONTROL MODE ═══\n" << RESET;
        
        float current_limit = getFloatInput(YELLOW "Current limit (A) [0 → 43]: " RESET, 0.0f, 43.0f);
        float accel = getFloatInput(YELLOW "Acceleration (rad/s²) [0 → 50]: " RESET, 0.0f, 50.0f);
        float speed = getFloatInput(YELLOW "Target speed (rad/s) [-20 → 20]: " RESET, -20.0f, 20.0f);
        
        switchModeAllMotors(Speed_control_mode, "Speed Control");
        
        std::cout << CYAN << "\n🏃 Sending command to all motors...\n" << RESET;
        for(size_t i = 0; i < motor_ids_.size(); i++) {
            motor_controller_->getMotor(motor_ids_[i]).RobStrite_Motor_Speed_control(current_limit, accel, speed);
            rclcpp::spin_some(shared_from_this());
            std::cout << "  Motor 0x" << std::hex << (int)motor_ids_[i] << std::dec << " commanded\n";
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        
        monitorMotors(3);
    }

    void positionControlMode()
    {
        std::cout << "\n" << BOLD << MAGENTA << "═══ POSITION PROFILE MODE ═══\n" << RESET;
        
        float speed = getFloatInput(YELLOW "Speed limit (rad/s) [0 → 20]: " RESET, 0.0f, 20.0f);
        float accel = getFloatInput(YELLOW "Acceleration (rad/s²) [0 → 50]: " RESET, 0.0f, 50.0f);
        float position = getFloatInput(YELLOW "Target position (rad) [-12.57 → 12.57]: " RESET, -12.57f, 12.57f);
        
        switchModeAllMotors(PosPP_control_mode, "Position Profile");
        
        std::cout << CYAN << "\n📍 Sending command to all motors...\n" << RESET;
        for(size_t i = 0; i < motor_ids_.size(); i++) {
            motor_controller_->getMotor(motor_ids_[i]).RobStrite_Motor_PosPP_control(speed, accel, position);
            rclcpp::spin_some(shared_from_this());
            std::cout << "  Motor 0x" << std::hex << (int)motor_ids_[i] << std::dec << " commanded\n";
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        
        monitorMotors(5);
    }

    void currentControlMode()
    {
        std::cout << "\n" << BOLD << MAGENTA << "═══ CURRENT CONTROL MODE ═══\n" << RESET;
        
        float iq = getFloatInput(YELLOW "Iq current (A) [-43 → 43]: " RESET, -43.0f, 43.0f);
        float id = getFloatInput(YELLOW "Id current (A) [-43 → 43]: " RESET, -43.0f, 43.0f);
        
        switchModeAllMotors(Elect_control_mode, "Current Control");
        
        std::cout << CYAN << "\n⚡ Sending command to all motors...\n" << RESET;
        for(size_t i = 0; i < motor_ids_.size(); i++) {
            motor_controller_->getMotor(motor_ids_[i]).RobStrite_Motor_Current_control(iq, id);
            rclcpp::spin_some(shared_from_this());
            std::cout << "  Motor 0x" << std::hex << (int)motor_ids_[i] << std::dec << " commanded\n";
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        
        monitorMotors(3);
    }

    void monitorMotors(int seconds)
    {
        std::cout << YELLOW << "\n⏳ Monitoring for " << seconds << " seconds...\n" << RESET;
        
        for(int s = 0; s < seconds * 10; s++) {
            if (g_shutdown_requested) break;
            
            rclcpp::spin_some(shared_from_this());
            
            if(s % 10 == 0) {
                std::cout << "\n--- Motors Status (t=" << (s/10) << "s) ---\n";
                for(size_t i = 0; i < motor_ids_.size(); i++) {
                    auto& motor = motor_controller_->getMotor(motor_ids_[i]);
                    std::cout << "Motor 0x" << std::hex << (int)motor_ids_[i] << std::dec << ": "
                              << "Pos=" << std::fixed << std::setprecision(3) << motor.Pos_Info.Angle << " rad, "
                              << "Vel=" << std::setprecision(3) << motor.Pos_Info.Speed << " rad/s, "
                              << "Tor=" << std::setprecision(3) << motor.Pos_Info.Torque << " Nm, "
                              << "T=" << std::setprecision(1) << motor.Pos_Info.Temp << "°C\n";
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void displayAllMotorsStatus()
    {
        std::cout << "\n" << BOLD << CYAN << "╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                   ALL MOTORS STATUS                        ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n" << RESET;
        
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "\n";
        std::cout << BOLD << std::setw(10) << "Motor" 
                  << std::setw(15) << "Position" 
                  << std::setw(15) << "Velocity" 
                  << std::setw(15) << "Torque" 
                  << std::setw(12) << "Temp" 
                  << std::setw(10) << "Pattern" << RESET << "\n";
        std::cout << std::string(77, '-') << "\n";
        
        for(size_t i = 0; i < motor_ids_.size(); i++) {
            auto& motor = motor_controller_->getMotor(motor_ids_[i]);
            std::cout << GREEN << "  0x" << std::hex << std::setw(2) << std::setfill('0') 
                      << (int)motor_ids_[i] << std::dec << std::setfill(' ') << RESET
                      << std::setw(15) << std::fixed << std::setprecision(3) << motor.Pos_Info.Angle
                      << std::setw(15) << std::setprecision(3) << motor.Pos_Info.Speed
                      << std::setw(15) << std::setprecision(3) << motor.Pos_Info.Torque
                      << std::setw(12) << std::setprecision(1) << motor.Pos_Info.Temp
                      << std::setw(10) << (int)motor.Pos_Info.pattern << "\n";
        }
        std::cout << std::string(77, '-') << "\n";
    }

    void run()
    {
        while (rclcpp::ok() && !g_shutdown_requested)
        {
            clearScreen();
            printHeader();
            
            std::cout << BOLD << CYAN << "Connected Motors: " << RESET;
            for(size_t i = 0; i < motor_ids_.size(); i++) {
                std::cout << GREEN << "Motor " << (i+1) << " (0x" 
                          << std::hex << std::setfill('0') << std::setw(2) 
                          << (int)motor_ids_[i] << std::dec << std::setfill(' ') << ")  " << RESET;
            }
            std::cout << "\n";
            
            int choice;
            try {
                choice = selectMode();
            } catch (const std::exception& e) {
                break;
            }
            
            if (choice == 0) {
                std::cout << RED << "\n👋 Exiting program...\n" << RESET;
                break;
            }
            
            try {
                switch (choice) {
                    case 1: motionControlMode(); break;
                    case 2: speedControlMode(); break;
                    case 3: positionControlMode(); break;
                    case 4: currentControlMode(); break;
                    case 5: displayAllMotorsStatus(); break;
                    default: std::cout << RED << "Invalid choice!\n" << RESET; break;
                }
            }
            catch (const std::exception& e) {
                if (g_shutdown_requested) break;
                std::cout << RED << "\n❌ Error: " << e.what() << "\n" << RESET;
            }
            
            if (!g_shutdown_requested) {
                std::cout << "\nPress Enter to continue...";
                std::cin.get();
            }
        }
    }
};

void signalHandler(int signum)
{
    std::cout << "\n" << YELLOW << "⚠️  Shutdown signal received (" << signum << ")\n" << RESET;
    g_shutdown_requested = true;
    
    if (g_node_ptr) {
        try {
            g_node_ptr->emergencyShutdown();
        } catch (...) {
            std::cerr << RED << "Error during emergency shutdown\n" << RESET;
        }
    }
    
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<MultiMotorControlTerminal>();
        
        node->initialize();
        
        std::cout << GREEN << "\n✅ System initialized successfully!\n" << RESET;
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