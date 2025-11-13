#!/bin/bash
# filepath: /home/dell/ros2_ws/src/motor_control/scripts/setup_4can.sh

##############################################################################
# Setup 4 CAN interfaces for Quadruped Robot
# Supports both SLCAN and gs_usb (Candlelight) firmware
##############################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║        Quadruped Robot - 4 CAN Buses Setup                 ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}❌ Please run as root: sudo $0${NC}"
    exit 1
fi

BITRATE=1000000

# Check firmware type
check_firmware() {
    if ls /dev/ttyACM* >/dev/null 2>&1; then
        echo -e "${YELLOW}Found SLCAN devices${NC}"
        return 0
    elif lsusb | grep -i "CANable\|gs_usb" >/dev/null; then
        echo -e "${YELLOW}Found gs_usb (Candlelight) devices${NC}"
        return 1
    else
        echo -e "${RED}❌ No CAN devices found!${NC}"
        echo "Please connect USB-CAN adapters"
        exit 1
    fi
}

# Setup CAN interface
setup_can() {
    local can_name=$1
    
    echo -e "${YELLOW}Setting up ${can_name}...${NC}"
    
    # Bring down if exists
    ip link set ${can_name} down 2>/dev/null || true
    
    # Configure
    ip link set ${can_name} type can bitrate ${BITRATE} restart-ms 100
    ip link set ${can_name} txqueuelen 100
    
    # Bring up
    ip link set ${can_name} up
    
    # Verify
    if ip link show ${can_name} 2>/dev/null | grep -q "UP"; then
        echo -e "${GREEN}✅ ${can_name} UP (${BITRATE} bps)${NC}"
        return 0
    else
        echo -e "${RED}❌ Failed: ${can_name}${NC}"
        return 1
    fi
}

# Setup SLCAN
setup_slcan() {
    local acm_dev=$1
    local can_name=$2
    
    echo -e "${YELLOW}Setting up SLCAN ${acm_dev} -> ${can_name}...${NC}"
    
    # Kill existing slcand
    killall slcand 2>/dev/null || true
    
    # Start slcand (-s8 = 1Mbps, -o = open, -c = close on exit)
    slcand -o -c -s8 ${acm_dev} ${can_name}
    
    sleep 0.5
    
    setup_can ${can_name}
}

# Main setup
echo -e "${YELLOW}Detecting firmware type...${NC}\n"

if check_firmware; then
    # SLCAN firmware
    ACM_DEVS=($(ls /dev/ttyACM* 2>/dev/null | head -4))
    
    if [ ${#ACM_DEVS[@]} -lt 4 ]; then
        echo -e "${RED}❌ Found only ${#ACM_DEVS[@]} ACM devices, need 4${NC}"
        echo "Available devices: ${ACM_DEVS[@]}"
        exit 1
    fi
    
    echo -e "${BLUE}Setting up SLCAN mode...${NC}\n"
    
    setup_slcan ${ACM_DEVS[0]} can0
    setup_slcan ${ACM_DEVS[1]} can1
    setup_slcan ${ACM_DEVS[2]} can2
    setup_slcan ${ACM_DEVS[3]} can3
    
else
    # gs_usb firmware (Candlelight)
    echo -e "${BLUE}Setting up gs_usb mode...${NC}\n"
    
    # Load driver
    modprobe gs_usb 2>/dev/null || true
    
    sleep 1
    
    # Setup interfaces
    setup_can can0
    setup_can can1
    setup_can can2
    setup_can can3
fi

# Summary
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                    ✅ SETUP COMPLETE                        ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}\n"

echo -e "${BLUE}CAN Bus Mapping:${NC}"
echo "  can0 → FL Leg (Motors 0x01, 0x02, 0x03)"
echo "  can1 → FR Leg (Motors 0x04, 0x05, 0x06)"
echo "  can2 → RL Leg (Motors 0x07, 0x08, 0x09)"
echo "  can3 → RR Leg (Motors 0x0A, 0x0B, 0x0C)"
echo ""

echo -e "${YELLOW}Status:${NC}"
for i in {0..3}; do
    echo -e "\n${BLUE}=== can${i} ===${NC}"
    ip -details link show can${i} 2>/dev/null || echo "  ❌ Not found"
done

echo ""
echo -e "${GREEN}Next step:${NC}"
echo -e "  ${YELLOW}./scripts/start_can_bridges.sh${NC}"