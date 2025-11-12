#ifndef _ROBSTRIDE_H_
#define _ROBSTRIDE_H_

#include "rclcpp/rclcpp.hpp"
#include "stdint.h"
#include "can_msgs/msg/frame.hpp"

/**
 * RobStrite_Motor — Low-level driver for a single Robstride actuator over CAN.
 * - Builds 29-bit extended CAN IDs as: (comm_type << 24) | (master_id << 8) | (motor_can_id)
 * - Encodes control commands per motor mode; decodes feedback into structured fields.
 * - Publishes CAN frames to a ROS 2 topic (e.g., "/to_can_bus") via ros2_socketcan bridge.
 */

// ==========================
// Protocol normalization ranges (MIT-style scaling; used when packing 16-bit fields)
// P = position (rad), V = velocity (rad/s), KP/KD = controller gains, T = torque (Nm).
// These are device-specific bounds from the manual; values are clamped before quantization.
// ==========================
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

// Speed-mode float parameter guards (current limit A, speed rad/s, Iq lower bound)
#define SC_MAX   43.0f
#define SC_MIN    0.0f
#define SV_MAX   20.0f
#define SV_MIN  -20.0f
#define SCIQ_MIN -43.0f

// ==========================
// Write-mode tags used by Set_RobStrite_Motor_parameter()
// 'Set_mode' tells function to write a "mode selector" (e.g., run_mode at index 0x7005)
// 'Set_parameter' tells function to write a general parameter (iq_ref, spd_ref, etc).
// -> These 'char' tags are *internal* function hints; they are NOT sent on the CAN wire.
// ==========================
#define Set_mode       'j' // Select & write control mode (e.g., run_mode = SPEED)
#define Set_parameter  'p' // Write a regular parameter (e.g., spd_ref, limit_cur)

// ==========================
// High-level logical control modes (local constants, used by your code)
// Map to run_mode (index 0x7005) via Set_RobStrite_Motor_parameter(0x7005, <mode>, Set_mode)
// 0: Motion (MIT-like 5-tuple), 1: PP (Profile Position), 2: Velocity, 3: Current, 4: Zero, 5: CSP
// ==========================
#define move_control_mode    0 // Motion control mode (Torque, Angle, Speed, Kp, Kd)
#define PosPP_control_mode   1 // Position mode (Profile-Position)
#define Speed_control_mode   2 // Velocity (Profile-Velocity)
#define Elect_control_mode   3 // Current/Iq mode
#define Set_Zero_mode        4 // Zero/mechanical zero
#define PosCSP_control_mode  5 // Position CSP (cyclic synchronous)

// ==========================
// Communication types (5-bit "type" field → bits 28..24 of 29-bit CAN ID)
// CAUTION: Some values below come from manual; ensure no duplicates.
// - 0x12 appears twice in original code (Control_Mode vs SetSingleParameter). The effective semantics used
//   by this driver match "Single parameter write" (type 0x12).
// ==========================
#define Communication_Type_Get_ID            0x00  // Query device ID & 64-bit MCU UID
#define Communication_Type_MotionControl     0x01  // Motion-control command (5-tuple) to host
#define Communication_Type_MotorRequest      0x02  // Motor feedback/status to host
#define Communication_Type_MotorEnable       0x03  // Enable motor (enter run state)
#define Communication_Type_MotorStop         0x04  // Stop motor (and optionally clear fault bits)
#define Communication_Type_SetPosZero        0x06  // Set mechanical zero
#define Communication_Type_Can_ID            0x07  // Change motor CAN_ID
#define Communication_Type_GetSingleParameter 0x11  // Read single parameter by index
#define Communication_Type_SetSingleParameter 0x12  // Write single parameter by index (persist if saved)
#define Communication_Type_ErrorFeedback     0x15  // Fault/warning feedback frame

// --------------------------
// Parameter descriptor (index + value). Used to stage read/write operations.
// --------------------------
class data_read_write_one
{
  public:
    uint16_t index; // Parameter index (e.g., 0x7005 run_mode, 0x700A spd_ref, ...)
    float    data;  // Value to write/read (float unless index specifies uint8/uint16 elsewhere)
};

// Default writable/readable index list per manual (0x7005.., 0x7018.., etc.)
static const uint16_t Index_List[] = {
  0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014,
  0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D
};

/**
 * data_read_write — Logical parameter bundle mapping common indices to meaningful names.
 * Most fields are float per manual; some are read-only (feedback/monitoring).
 * Write path uses Communication_Type_SetSingleParameter (0x12).
 */
class data_read_write
{
  public:
    // Writable set (comm type 0x12)
    data_read_write_one run_mode;       // 0: motion, 1: position (PP), 2: speed, 3: current, 4: zero (uint8)
    data_read_write_one iq_ref;         // Iq command in current mode (A) [-43..43]
    data_read_write_one spd_ref;        // Speed command in speed mode (rad/s) [-20..20]
    data_read_write_one imit_torque;    // Torque limit (Nm) [0..60]
    data_read_write_one cur_kp;         // Current loop Kp (default ~0.125)
    data_read_write_one cur_ki;         // Current loop Ki (default ~0.0158)
    data_read_write_one cur_filt_gain;  // Current loop filter gain [0..1.0] (default 0.1)
    data_read_write_one loc_ref;        // Position command in position mode (rad)
    data_read_write_one limit_spd;      // Position mode speed limit (rad/s)
    data_read_write_one limit_cur;      // Current limit for speed/position modes (A)

    // Read-only monitoring (updated from feedback frames)
    data_read_write_one mechPos;        // Mechanical angle at load end (rad)
    data_read_write_one iqf;            // Filtered Iq (A)
    data_read_write_one mechVel;        // Mechanical speed (rad/s)
    data_read_write_one VBUS;           // Bus voltage (V)
    data_read_write_one rotation;       // Rotation counter (int16 → stored in .data as float)

    data_read_write(const uint16_t *index_list = Index_List);
};

// --------------------------
// Decoded motor feedback snapshot (published via MotorFeedback)
// pattern: 0 reset, 1 calibration, 2 run (RUN)
// --------------------------
typedef struct
{
  float Angle;    // rad
  float Speed;    // rad/s
  float Torque;   // Nm
  float Temp;     // degC
  int   pattern;  // state pattern: 0=Reset, 1=Calibration, 2=Run
} Motor_Pos_RobStrite_Info;

// --------------------------
// Command set cache (what we intend to send / last set)
// Used to keep the most recent "intent" values for this motor.
// --------------------------
typedef struct
{
  int   set_motor_mode; // Mirror of run_mode (0..5)
  float set_current;    // Generic "current" depending on mode
  float set_speed;      // rad/s
  float set_Torque;     // Nm
  float set_angle;      // rad
  float set_limit_cur;  // A (current limit)
  float set_Kp;
  float set_Ki;
  float set_Kd;
  float set_iq;         // A (current-mode Iq)
  float set_id;         // A (current-mode Id)
  float set_acc;        // rad/s^2
} Motor_Set;

/**
 * RobStrite_Motor — Single-motor driver.
 *
 * Members:
 *  - CAN_ID:         device CAN ID (default 0x7F). Used in 29-bit extended ID composition.
 *  - Master_CAN_ID:  host/master CAN ID (set to 0x1F in init).
 *  - command_pub:    ROS 2 publisher of can_msgs::msg::Frame (→ /to_can_bus).
 *  - Pos_Info:       latest decoded feedback (angle/speed/torque/temp/pattern).
 *  - drw:            parameter map (run_mode, spd_ref, iq_ref, limits, etc.).
 *  - error_code:     last fault code seen (if any).
 *
 * Key API:
 *  - Enable_Motor() / Disenable_Motor(clear_error):
 *      Build & send type 0x03 / 0x04 frames. Clear error if requested.
 *  - Set_RobStrite_Motor_parameter(index, value, Value_mode):
 *      Writes single parameter (type 0x12). For mode switch, use index=0x7005 and Value_mode=Set_mode.
 *  - RobStrite_Motor_*_control(...):
 *      Send control command according to active mode.
 *  - RobStrite_Motor_Analysis(data, id):
 *      Parse a received CAN frame payload & ID; update Pos_Info/drw/error_code.
 *
 * Usage pattern (per manual):
 *   Disenable_Motor() →
 *   Set_RobStrite_Motor_parameter(0x7005, <mode>, Set_mode) →
 *   Enable_Motor() → (wait for RUN) →
 *   RobStrite_Motor_<Mode>_control(...)
 */
class RobStrite_Motor
{
private:
  uint8_t  CAN_ID;             // Device CAN ID (0..127). Default 0x7F. Must match hardware.
  uint16_t Master_CAN_ID;      // Host/master CAN ID (set to 0x1F during init).
  float   (*Motor_Offset_MotoFunc)(float Motor_Tar); // Optional offset/transform hook for angle commands.

  Motor_Set                Motor_Set_All; // Last commanded values for diagnostics/telemetry.

public:
  // ROS 2 publisher to send raw CAN frames to ros2_socketcan (→ /to_can_bus)
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr command_pub;

  float  output = 0.0f;                 // Optional computed output
  int    Can_Motor = 0;                 // Optional tag/index
  Motor_Pos_RobStrite_Info Pos_Info{};  // Latest decoded feedback snapshot
  data_read_write          drw;         // Parameter map (writable + read-only mirrors)
  uint8_t                  error_code = 0; // Last error code (if decoded)

  // --- Constructors: attach to a ROS node and declare which topic to publish CAN frames to.
  RobStrite_Motor(uint8_t CAN_Id,
                  const std::shared_ptr<rclcpp::Node>& node_handle,
                  const std::string &command_tx);

  RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar),
                  uint8_t CAN_Id,
                  const std::shared_ptr<rclcpp::Node>& node_handle,
                  const std::string &command_tx);

  // --- Device discovery / parameter R/W ---
  void RobStrite_Get_CAN_ID();                                      // Send type 0x00 to query IDs
  void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode); // type 0x12 write
  void Get_RobStrite_Motor_parameter(uint16_t Index);               // type 0x11 read

  // --- Feedback decode (called from MotorControlSet RX callback) ---
  void RobStrite_Motor_Analysis(uint8_t *DataFrame, uint32_t ID_ExtId);

  // --- Control commands (match the current run_mode) ---
  // Motion control (5-tuple). Typical for MIT-like mode.
  void RobStrite_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd);

  // Profile Position (PP): Speed/Acc limits + target Angle
  void RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle);

  // Velocity: Current limit, Accel (rad/s^2), Target speed (rad/s)
  void RobStrite_Motor_Speed_control(float Current_Limits, float Accel, float Speed);

  // Current (Iq/Id): raw current commands (A)
  void RobStrite_Motor_Current_control(float IqCommand, float IdCommand);

  // Position CSP: target speed & angle for cyclic synchronous position
  void RobStrite_Motor_PosCSP_control(float Speed, float Angle);

  // Set mechanical zero (type 0x06). Do NOT call while in PP mode per manual.
  void RobStrite_Motor_Set_Zero_control();

  // --- State transitions ---
  void Enable_Motor();                          // type 0x03 — put motor into RUN
  void Disenable_Motor(uint8_t clear_error);    // type 0x04 — stop; clear_error!=0 clears faults in payload
  void Set_CAN_ID(uint8_t Set_CAN_ID);          // type 0x07 — change device CAN ID (takes effect immediately)
  void Set_ZeroPos();                           // Convenience wrapper for Set_Zero control
};

#endif
