#include "motor_control/motor_config.hpp"
#include <stdexcept>

MotorControlSet::MotorControlSet(const std::shared_ptr<rclcpp::Node>& node_handle, 
						const std::string &can_rx, 
						const std::string &can_tx,
						const std::vector<uint8_t>& motor_ids)
                    :   l1_joint(motor_ids[0], node_handle, "robstride_cmd"),
                        l2_joint(motor_ids[1], node_handle, "robstride_cmd"),
                        l3_joint(motor_ids[2], node_handle, "robstride_cmd")
{   
    can_receive = node_handle->create_subscription<can_msgs::msg::Frame>(
                    can_rx, 100, 
                    std::bind(&MotorControlSet::can1_rx_Callback, this, std::placeholders::_1));
    can_send = node_handle->create_publisher<can_msgs::msg::Frame>(can_tx, 100);
    command_bridge = node_handle->create_subscription<can_msgs::msg::Frame>(
                    "robstride_cmd", 100, 
                    std::bind(&MotorControlSet::command_Callback, this, std::placeholders::_1));
    robstride_state_pub = node_handle->create_publisher<motor_feedback_msg::msg::MotorFeedback>("/robstride_state", 100);

    joint_feedback.pos.resize(Joint_Num);
    joint_feedback.vel.resize(Joint_Num);
    joint_feedback.tor.resize(Joint_Num);
    joint_feedback.temp.resize(Joint_Num);
    joint_feedback.error_code.resize(Joint_Num);
    joint_feedback.pattern.resize(Joint_Num);
}

void MotorControlSet::shutdownCallback()
{
    l1_joint.Disenable_Motor(0);
    l2_joint.Disenable_Motor(0);
    l3_joint.Disenable_Motor(0);
}

MotorControlSet::~MotorControlSet()
{   
    shutdownCallback();
    // std::cout << "All joint motor have been disabled." << std::endl;
}

void MotorControlSet::can1_rx_Callback(const can_msgs::msg::Frame::SharedPtr msg)
{   
    uint8_t motor_id = uint8_t((msg->id & 0xFF00) >> 8);
   
    if (motor_id == l1_joint.getCANID()) {
        l1_joint.RobStrite_Motor_Analysis(msg->data.data(), msg->id);
        joint_feedback.pos[0] = l1_joint.Pos_Info.Angle;
        joint_feedback.vel[0] = l1_joint.Pos_Info.Speed;
        joint_feedback.tor[0] = l1_joint.Pos_Info.Torque;
        joint_feedback.temp[0] = l1_joint.Pos_Info.Temp;
        joint_feedback.error_code[0] = l1_joint.error_code;
        joint_feedback.pattern[0] = l1_joint.Pos_Info.pattern;
    }
    else if (motor_id == l2_joint.getCANID()) {
        l2_joint.RobStrite_Motor_Analysis(msg->data.data(), msg->id);
        joint_feedback.pos[1] = l2_joint.Pos_Info.Angle;
        joint_feedback.vel[1] = l2_joint.Pos_Info.Speed;
        joint_feedback.tor[1] = l2_joint.Pos_Info.Torque;
        joint_feedback.temp[1] = l2_joint.Pos_Info.Temp;
        joint_feedback.error_code[1] = l2_joint.error_code;
        joint_feedback.pattern[1] = l2_joint.Pos_Info.pattern;
    }
    else if (motor_id == l3_joint.getCANID()) {
        l3_joint.RobStrite_Motor_Analysis(msg->data.data(), msg->id);
        joint_feedback.pos[2] = l3_joint.Pos_Info.Angle;
        joint_feedback.vel[2] = l3_joint.Pos_Info.Speed;
        joint_feedback.tor[2] = l3_joint.Pos_Info.Torque;
        joint_feedback.temp[2] = l3_joint.Pos_Info.Temp;
        joint_feedback.error_code[2] = l3_joint.error_code;
        joint_feedback.pattern[2] = l3_joint.Pos_Info.pattern;
    }

    robstride_state_pub->publish(joint_feedback);
}

RobStrite_Motor& MotorControlSet::getMotor(uint8_t ID)
{
    if (ID == l1_joint.getCANID()) return l1_joint;
    else if (ID == l2_joint.getCANID()) return l2_joint;
    else if (ID == l3_joint.getCANID()) return l3_joint;
    else {
        throw std::runtime_error("Invalid motor ID: 0x" + std::to_string(ID));
    }
}

void MotorControlSet::command_Callback(const can_msgs::msg::Frame::SharedPtr msg){
    can_send->publish(*msg);
}
