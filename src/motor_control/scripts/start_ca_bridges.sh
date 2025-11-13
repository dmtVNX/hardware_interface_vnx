#!/bin/bash
# filepath: /home/dell/ros2_ws/src/motor_control/scripts/start_can_bridges.sh

##############################################################################
# Start 4 SocketCAN bridges with separate topics in tmux
##############################################################################

if ! command -v tmux &> /dev/null; then
    echo "Installing tmux..."
    sudo apt-get update && sudo apt-get install -y tmux
fi

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

tmux kill-session -t can_bridges 2>/dev/null

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     Starting 4 SocketCAN Bridges in tmux                   ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Create new session with 4 panes (2x2)
tmux new-session -d -s can_bridges -n bridges

# Split into 4 panes
tmux split-window -h -t can_bridges
tmux split-window -v -t can_bridges:0.0
tmux split-window -v -t can_bridges:0.1

# Start bridge in each pane
# Pane 0 - can0 (FL Leg)
tmux select-pane -t can_bridges:0.0
tmux send-keys -t can_bridges:0.0 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t can_bridges:0.0 "source ~/ros2_ws/install/setup.bash" C-m
tmux send-keys -t can_bridges:0.0 "echo '=== CAN0 Bridge (FL Leg: 0x01-03) ==='" C-m
tmux send-keys -t can_bridges:0.0 "ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0 from_can_bus_topic:=/from_can_bus_0 to_can_bus_topic:=/to_can_bus_0" C-m

# Pane 1 - can1 (FR Leg)
tmux select-pane -t can_bridges:0.1
tmux send-keys -t can_bridges:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t can_bridges:0.1 "source ~/ros2_ws/install/setup.bash" C-m
tmux send-keys -t can_bridges:0.1 "echo '=== CAN1 Bridge (FR Leg: 0x04-06) ==='" C-m
tmux send-keys -t can_bridges:0.1 "ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can1 from_can_bus_topic:=/from_can_bus_1 to_can_bus_topic:=/to_can_bus_1" C-m

# Pane 2 - can2 (RL Leg)
tmux select-pane -t can_bridges:0.2
tmux send-keys -t can_bridges:0.2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t can_bridges:0.2 "source ~/ros2_ws/install/setup.bash" C-m
tmux send-keys -t can_bridges:0.2 "echo '=== CAN2 Bridge (RL Leg: 0x07-09) ==='" C-m
tmux send-keys -t can_bridges:0.2 "ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can2 from_can_bus_topic:=/from_can_bus_2 to_can_bus_topic:=/to_can_bus_2" C-m

# Pane 3 - can3 (RR Leg)
tmux select-pane -t can_bridges:0.3
tmux send-keys -t can_bridges:0.3 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t can_bridges:0.3 "source ~/ros2_ws/install/setup.bash" C-m
tmux send-keys -t can_bridges:0.3 "echo '=== CAN3 Bridge (RR Leg: 0x0A-0C) ==='" C-m
tmux send-keys -t can_bridges:0.3 "ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can3 from_can_bus_topic:=/from_can_bus_3 to_can_bus_topic:=/to_can_bus_3" C-m

sleep 2

echo "✅ CAN bridges started in tmux session 'can_bridges'"
echo ""
echo "Topics created:"
echo "  can0: /from_can_bus_0 ↔ /to_can_bus_0 (FL: 0x01,02,03)"
echo "  can1: /from_can_bus_1 ↔ /to_can_bus_1 (FR: 0x04,05,06)"
echo "  can2: /from_can_bus_2 ↔ /to_can_bus_2 (RL: 0x07,08,09)"
echo "  can3: /from_can_bus_3 ↔ /to_can_bus_3 (RR: 0x0A,0B,0C)"
echo ""
echo "Commands:"
echo "  View bridges: tmux attach -t can_bridges"
echo "  Detach:       Ctrl+B then D"
echo "  Kill all:     tmux kill-session -t can_bridges"
echo ""
echo "Verify topics:"
echo "  ros2 topic list | grep can"
echo ""
echo "Next: ros2 run motor_control quadruped_control_node"