#!/bin/bash
# Simple CAN setup script

set -e  # gặp lỗi sẽ dừng ngay
set -o pipefail

echo ">>> Load kernel modules..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan

echo ">>> Cho phép đọc dmesg..."
sudo sysctl -w kernel.dmesg_restrict=0

echo ">>> Kiểm tra thiết bị tty..."
dmesg | grep tty | tail -n 5
ls /dev/ttyACM* || { echo "Không tìm thấy /dev/ttyACM*" ; exit 1; }

echo ">>> Khởi tạo slcand..."
sudo slcand -o -c -s8 /dev/ttyACM0 can0

echo ">>> Bật can0..."
sudo ip link set can0 up type can bitrate 1000000

echo ">>> Trạng thái can0:"
ip -details -statistics link show can0
