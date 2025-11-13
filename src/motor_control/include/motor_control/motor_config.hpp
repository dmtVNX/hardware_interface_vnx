// MotorControlSet — ROS 2 wrapper that groups multiple Robstride motors,
// subscribes to frames coming from the CAN bus, publishes frames to the bus,
// and exposes a structured MotorFeedback message for higher layers.
//
// Data flow:
//   HW → SocketCAN → ros2_socketcan (/from_can_bus) → MotorControlSet::can1_rx_Callback()
//   MotorControlSet (decodes, updates state) → publish /robstride_state (MotorFeedback)
//
//   App/CLI → RobStrite_Motor APIs → publish /to_can_bus (raw CAN frames) → ros2_socketcan → HW
//
// Notes:
// - One MotorControlSet typically represents one robot leg (2–3 joints).
// - Each RobStrite_Motor instance represents one actuator (one joint).

#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "stdint.h"
#include "can_msgs/msg/frame.hpp"
#include "motor_feedback_msg/msg/motor_feedback.hpp"
#include "motor_control/robstride.hpp"

// Number of joints/actuator managed by this set
#define Joint_Num 3

// Normalization ranges
#define P_MIN -12.57f
#define P_MAX  12.57f
#define V_MIN -20.0f
#define V_MAX  20.0f
#define KP_MIN  0.0f
#define KP_MAX  5000.0f
#define KD_MIN  0.0f
#define KD_MAX  100.0f
#define T_MIN  -60.0f
#define T_MAX   60.0f

// Raw motor state snapshot (per joint), as parsed from feedback frames.
// Fields are in SI units (rad, rad/s, Nm) and degrees C for temperatures.
typedef struct
{
  uint16_t state; // optional raw state bits / vendor-specific state word
  float pos;      // mechanical angle at load end (rad)
  float vel;      // mechanical speed (rad/s)
  float tor;      // torque (Nm)
  float Kp;       // controller Kp (if mapped in your feedback)
  float Kd;       // controller Kd (if mapped)
  float Tmos;     // MOS temperature (degC)
  float Tcoil;    // Stator/coil temperature (degC)
} motor_state_t;

// Desired command cache (per joint). Not directly published on the bus;
// used to store "intent" values that higher layers want to apply.
typedef struct
{
  float pos_d;  // desired position (rad)
  float vel_d;  // desired velocity (rad/s)
  float tor_d;  // desired torque (Nm)
  float Kp;     // desired Kp
  float Kd;     // desired Kd
} motor_cmd_t;

//------------------------------------------------------------------------------
// MotorControlSet: manages a small group of RobStrite_Motor instances, and
// bridges raw CAN frames <-> structured MotorFeedback.
//
// Topics (typical):
//   - Sub:  /from_can_bus  (can_msgs/Frame)  ← ros2_socketcan receiver
//   - Pub:  /to_can_bus    (can_msgs/Frame)  → ros2_socketcan sender
//   - Pub:  /robstride_state (motor_feedback_msg/MotorFeedback)  ← decoded joint state
//
// Optional:
//   - Sub:  robstride_cmd (can_msgs/Frame) bridge “intent” via raw frames internally.
//------------------------------------------------------------------------------
class MotorControlSet
{
public:
  // Construct with a node handle and topic names for RX/TX, plus the list of CAN motor IDs.
  // node_handle : shared ROS 2 node
  // can_rx      : topic name for frames FROM the CAN bus (e.g., "/from_can_bus")
  // can_tx      : topic name for frames TO the CAN bus (e.g., "/to_can_bus")
  // motor_ids   : list of Robstride CAN IDs to manage
  MotorControlSet(const std::shared_ptr<rclcpp::Node>& node_handle,
                  const std::string &can_rx,
                  const std::string &can_tx,
                  const std::vector<uint8_t>& motor_ids);
  ~MotorControlSet();

  // Hook to gracefully stop on shutdown (e.g., send Stop, release resources).
  void shutdownCallback();

  // ---------------- ROS I/O ----------------

  // Subscriber for frames coming FROM the CAN bus via ros2_socketcan.
  // Callback: can1_rx_Callback()
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_receive;

  // subscriber that receives raw “command frames” from an internal topic
  // and this callback forwards to /to_can_bus).
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr command_bridge;

  // Publisher to send raw frames TO the CAN bus (ros2_socketcan sender subscribes this).
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_send;

  // Publisher for structured, decoded state for higher-level consumers (planner, GUI, logging).
  rclcpp::Publisher<motor_feedback_msg::msg::MotorFeedback>::SharedPtr robstride_state_pub;

  // Callback for /from_can_bus:
  // - Route frame to the correct RobStrite_Motor by CAN ID
  // - Decode payload → update internal arrays and joint_feedback
  // - Publish updated /robstride_state
  void can1_rx_Callback(const can_msgs::msg::Frame::SharedPtr msg);

  // Callback for internal command bridge (optional):
  // - Accept raw can_msgs/Frame from a local topic (e.g., "robstride_cmd")
  // - Forward to can_send → /to_can_bus
  void command_Callback(const can_msgs::msg::Frame::SharedPtr msg);

  // Per-joint state/cache arrays (size = Joint_Num)
  motor_state_t motor_state[Joint_Num];
  motor_cmd_t   motor_cmd[Joint_Num];

  // Aggregate feedback message published to /robstride_state.
  // Fill its arrays in the RX callback using data parsed from RobStrite_Motor instances.
  motor_feedback_msg::msg::MotorFeedback joint_feedback;

  // Lookup a motor by can_id
  RobStrite_Motor& getMotor(uint8_t ID);

private:
  // Each RobStrite_Motor knows how to encode/decode its own frames.
  RobStrite_Motor l1_joint;  // Motor 1
  RobStrite_Motor l2_joint;  // Motor 2
  RobStrite_Motor l3_joint;  // Motor 3

  void buildMotorMaps();

};

#endif
