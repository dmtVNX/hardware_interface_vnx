# Motor Control Package - ROS2 Humble

Control package for RobStride motors via SocketCAN.
Supports both **3-motor test** and **12-motor quadruped** configurations.

---

## 📦 Installation

### Step 1: Install dependencies
```bash
sudo apt update
sudo apt install can-utils
sudo apt install ros-humble-can-msgs
sudo apt install ros-humble-ros2-socketcan
sudo apt install tmux
```

### Step 2: Build package
```bash
cd ~/ros2_ws
colcon build --packages-select motor_control
source install/setup.bash
```

### Step 3: Make scripts executable
```bash
cd ~/ros2_ws/src/motor_control/scripts
chmod +x *.sh
```

---

## 🚀 Usage

### **Option 1: Test 3 Motors (Single CAN bus)**

#### Terminal 1: Start CAN bridge
```bash
# For SLCAN firmware:
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up type can bitrate 1000000

# For gs_usb (Candlelight):
sudo ip link set can0 up type can bitrate 1000000

# Start ROS2 bridge
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

#### Terminal 2: Run motor control
```bash
source ~/ros2_ws/install/setup.bash
ros2 run motor_control motor_control_node
```

---

### **Option 2: Test 12 Motors (4 CAN buses) - QUADRUPED**

#### Step 1: Setup 4 CAN interfaces
```bash
cd ~/ros2_ws/src/motor_control/scripts
sudo ./setup_4can.sh
```

**Verify:**
```bash
ip link show | grep can
# Should show can0, can1, can2, can3 in UP state
```

#### Step 2: Start 4 CAN bridges
```bash
./start_can_bridges.sh
```

This creates **8 topics**:
- `/from_can_bus_0` ↔ `/to_can_bus_0` (can0: FL Leg)
- `/from_can_bus_1` ↔ `/to_can_bus_1` (can1: FR Leg)
- `/from_can_bus_2` ↔ `/to_can_bus_2` (can2: RL Leg)
- `/from_can_bus_3` ↔ `/to_can_bus_3` (can3: RR Leg)

**View bridges:**
```bash
tmux attach -t can_bridges  # Ctrl+B then D to detach
```

**Verify topics:**
```bash
ros2 topic list | grep can
```

#### Step 3: Test motors (optional)
```bash
./test_all_motors.sh
```

#### Step 4: Run quadruped control
**In NEW terminal:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run motor_control quadruped_control_node
```

---

## 🗺️ Motor Mapping (Quadruped)

| Motor ID | Joint     | Leg | CAN Bus | RX Topic         | TX Topic        |
|----------|-----------|-----|---------|------------------|-----------------|
| 0x01     | FL_hip    | FL  | can0    | /from_can_bus_0  | /to_can_bus_0   |
| 0x02     | FL_thigh  | FL  | can0    | /from_can_bus_0  | /to_can_bus_0   |
| 0x03     | FL_calf   | FL  | can0    | /from_can_bus_0  | /to_can_bus_0   |
| 0x04     | FR_hip    | FR  | can1    | /from_can_bus_1  | /to_can_bus_1   |
| 0x05     | FR_thigh  | FR  | can1    | /from_can_bus_1  | /to_can_bus_1   |
| 0x06     | FR_calf   | FR  | can1    | /from_can_bus_1  | /to_can_bus_1   |
| 0x07     | RL_hip    | RL  | can2    | /from_can_bus_2  | /to_can_bus_2   |
| 0x08     | RL_thigh  | RL  | can2    | /from_can_bus_2  | /to_can_bus_2   |
| 0x09     | RL_calf   | RL  | can2    | /from_can_bus_2  | /to_can_bus_2   |
| 0x0A     | RR_hip    | RR  | can3    | /from_can_bus_3  | /to_can_bus_3   |
| 0x0B     | RR_thigh  | RR  | can3    | /from_can_bus_3  | /to_can_bus_3   |
| 0x0C     | RR_calf   | RR  | can3    | /from_can_bus_3  | /to_can_bus_3   |

**Legend:** FL=Front Left, FR=Front Right, RL=Rear Left, RR=Rear Right

---

## 🧪 Monitoring & Debug

### Monitor CAN traffic (4 buses)
```bash
./scripts/monitor_can.sh
```

### Monitor single bus
```bash
candump can0 -c -t A
```

### Check ROS2 topics
```bash
ros2 topic list
ros2 topic echo /from_can_bus_0
ros2 topic hz /from_can_bus_0
```

### Check CAN interface status
```bash
ip -details link show can0
```

---

## 🐛 Troubleshooting

### CAN interface not found
```bash
# Check devices
lsusb
ls /dev/ttyACM*

# For gs_usb firmware
dmesg | grep can
sudo modprobe -r gs_usb
sudo modprobe gs_usb

# Re-run setup
sudo ./scripts/setup_4can.sh
```

### Bridges not starting
```bash
# Kill existing
tmux kill-session -t can_bridges

# Check package installed
ros2 pkg list | grep socketcan

# Restart
./scripts/start_can_bridges.sh
```

### Motor not responding
```bash
# Test CAN communication
cansend can0 01#0001000000000000
candump can0

# Check:
# - Motor power supply
# - CAN termination (120Ω resistors)
# - CAN wiring (CANH, CANL, GND)
# - Motor ID matches configuration
```

### Frame drops (gs_usb)
- Increase delays in code (200ms recommended)
- Reduce TX queue load
- Check USB cable quality
- Use USB 2.0 ports (not USB 3.0)

---

## 📚 Control Modes

1. **Motion Control (MIT)** - Position/velocity/torque with PD gains
   - Torque: -60 to 60 Nm
   - Angle: -12.57 to 12.57 rad (±720°)
   - Speed: -20 to 20 rad/s
   - Kp: 0-500, Kd: 0-100

2. **Speed Control** - Velocity with current limit
   - Current limit: 0-43 A
   - Acceleration: 0-50 rad/s²
   - Speed: -20 to 20 rad/s

3. **Position Profile** - Trajectory planning
   - Speed limit: 0-20 rad/s
   - Acceleration: 0-50 rad/s²
   - Position: -12.57 to 12.57 rad

4. **Current Control** - Direct current (iq/id)
   - Iq: -43 to 43 A
   - Id: -43 to 43 A

---

## ⚠️ Safety

- **ALWAYS** test individual motors before full system
- Keep emergency stop accessible
- Monitor motor temperatures (<80°C)
- Start with low torque/speed values
- Ensure robot is secured when testing
- Use appropriate PD gains to prevent oscillation

---

## 📁 Package Structure

```
motor_control/
├── include/motor_control/
│   ├── motor_config.hpp
│   ├── robstride.hpp
│   ├── robstride_system_simple.hpp
│   └── quadruped_controller.hpp
├── src/
│   ├── motor_config.cpp
│   ├── robstride.cpp
│   ├── robstride_system_simple.cpp
│   ├── motor_control_node.cpp (3 motors)
│   ├── quadruped_controller.cpp
│   └── quadruped_control_node.cpp (12 motors)
├── scripts/
│   ├── setup_4can.sh
│   ├── start_can_bridges.sh
│   ├── test_all_motors.sh
│   └── monitor_can.sh
├── launch/
│   ├── motor_with_can.launch.py
│   └── bringup_hw_two_joint_simple.launch.py
└── config/
    └── ...
```

---

## 🎯 Quick Start Commands

```bash
# Build
cd ~/ros2_ws && colcon build --packages-select motor_control && source install/setup.bash

# Setup CAN (one-time)
cd ~/ros2_ws/src/motor_control/scripts && sudo ./setup_4can.sh

# Start bridges (tmux session)
./start_can_bridges.sh

# Test motors
./test_all_motors.sh

# Run control (new terminal)
ros2 run motor_control quadruped_control_node

# Monitor (new terminal)
./monitor_can.sh
```

---