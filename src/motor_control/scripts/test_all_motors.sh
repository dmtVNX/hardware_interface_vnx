#!/bin/bash
# filepath: /home/dell/ros2_ws/src/motor_control/scripts/test_all_motors.sh

##############################################################################
# Test all 12 motors connectivity
##############################################################################

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║            Testing 12 Motors Connectivity                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}\n"

test_motor() {
    local motor_id=$1
    local can_bus=$2
    local motor_name=$3
    
    echo -ne "Testing ${motor_name} (0x${motor_id} on ${can_bus})... "
    
    # Send enable command
    cansend ${can_bus} ${motor_id}#0001000000000000 2>/dev/null
    sleep 0.3
    
    # Check response (timeout 1s)
    if timeout 1 candump ${can_bus} -n 1 2>/dev/null | grep -q ${motor_id}; then
        echo -e "${GREEN}✅ OK${NC}"
        return 0
    else
        echo -e "${RED}❌ NO RESPONSE${NC}"
        return 1
    fi
}

# Test counters
total=12
passed=0

echo -e "${YELLOW}=== Front Left Leg (can0) ===${NC}"
test_motor "01" "can0" "FL_hip   " && ((passed++))
test_motor "02" "can0" "FL_thigh " && ((passed++))
test_motor "03" "can0" "FL_calf  " && ((passed++))

echo -e "\n${YELLOW}=== Front Right Leg (can1) ===${NC}"
test_motor "04" "can1" "FR_hip   " && ((passed++))
test_motor "05" "can1" "FR_thigh " && ((passed++))
test_motor "06" "can1" "FR_calf  " && ((passed++))

echo -e "\n${YELLOW}=== Rear Left Leg (can2) ===${NC}"
test_motor "07" "can2" "RL_hip   " && ((passed++))
test_motor "08" "can2" "RL_thigh " && ((passed++))
test_motor "09" "can2" "RL_calf  " && ((passed++))

echo -e "\n${YELLOW}=== Rear Right Leg (can3) ===${NC}"
test_motor "0A" "can3" "RR_hip   " && ((passed++))
test_motor "0B" "can3" "RR_thigh " && ((passed++))
test_motor "0C" "can3" "RR_calf  " && ((passed++))

echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                     TEST SUMMARY                           ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"

if [ $passed -eq $total ]; then
    echo -e "${GREEN}✅ All ${total} motors responding!${NC}"
    exit 0
else
    echo -e "${RED}⚠️  ${passed}/${total} motors responding${NC}"
    echo -e "${YELLOW}Failed motors: $((total - passed))${NC}"
    exit 1
fi