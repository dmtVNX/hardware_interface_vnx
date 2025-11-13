#!/bin/bash
# filepath: /home/dell/ros2_ws/src/motor_control/scripts/monitor_can.sh

##############################################################################
# Monitor all 4 CAN buses in split terminal
##############################################################################

if ! command -v tmux &> /dev/null; then
    echo "Installing tmux..."
    sudo apt-get update && sudo apt-get install -y tmux
fi

tmux kill-session -t can_monitor 2>/dev/null

tmux new-session -d -s can_monitor

# Split into 4 panes (2x2)
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Monitor each CAN bus
tmux select-pane -t 0
tmux send-keys "clear && echo '=== CAN0 (FL Leg: 0x01-03) ===' && candump can0 -c -t A" C-m

tmux select-pane -t 1
tmux send-keys "clear && echo '=== CAN1 (FR Leg: 0x04-06) ===' && candump can1 -c -t A" C-m

tmux select-pane -t 2
tmux send-keys "clear && echo '=== CAN2 (RL Leg: 0x07-09) ===' && candump can2 -c -t A" C-m

tmux select-pane -t 3
tmux send-keys "clear && echo '=== CAN3 (RR Leg: 0x0A-0C) ===' && candump can3 -c -t A" C-m

echo "Monitor started in tmux session 'can_monitor'"
echo "Detach: Ctrl+B then D"
echo "Kill: tmux kill-session -t can_monitor"
echo ""

tmux attach-session -t can_monitor