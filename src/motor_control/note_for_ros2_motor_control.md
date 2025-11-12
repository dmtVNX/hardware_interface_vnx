#### Step 1: Install `can_utils`
```bash
sudo apt update
sudo apt install can-utils
sudo apt install ros-humble-can-msgs
sudo apt install ros-humble-ros2-socketcan
```

#### Step 2: Check result has something like `CANable`
```bash
lsusb
```

#### Step 3: Verify Kernel module
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan
```

check if modules are loaded
```bash
lsmod | grep can
```

#### Step 4: Set up `dmesg_restrict` = 0
```bash
cat /proc/sys/kernel/dmesg_restrict
sudo sysctl -w kernel.dmesg_restrict=0
```

Show any USB-to-serial devices that have been detected
```bash
dmesg | grep tty
```

Check your ACM
```bash
ls /dev/ttyACM*
```

#### Step 5: Run `can0`
```bash
sudo slcand -o -c -s8 /dev/ttyACMx can0
```

> **Noted:** Replace `x` by your actual ACM* device. `-s6:` 500k bps, `-s8:` 1M bps.

Check for `can0`
```bash
ip link
```

Bring `can0` up and build bitrate for `can0`
```bash
sudo ip link set can0 up type can bitrate 1000000
```

View can message
```bash
candump can0
```

#### Step 6: Run launch file

Run example
```bash
ros2 launch motor_control motor_with_can.launch.py 
```

Run motor_control
```bash
ros2 launch motor_control bringup_hw_two_joint_simple.launch.py can_iface:=can0
```