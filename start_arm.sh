#!/bin/bash
# Soft arm startup script
# =======================
# 1. Move all 9 arm motors to home position
# 2. Set USB latency timer for maximum throughput
# 3. Start dynamixel node: motors 11–19 in torque mode

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [1/3] Moving arm to home position ==="
python3 "$SCRIPT_DIR/VMC/MultiGoHome.py"

echo "=== [2/3] Setting USB latency timer ==="
sudo bash -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'

echo "=== [3/3] Building and launching dynamixel node (torque mode) ==="
(cd "$SCRIPT_DIR/ros2_dynamixel" && colcon build --quiet && source install/setup.bash && \
 ros2 run dynamixel_interface dynamixel_node \
     --ros-args -p baudrate:=1000000 \
     -p motor_ids:=[11,12,13,14,15,16,17,18,19] \
     -p control_mode:=torque) &
NODE_PID=$!

echo ""
echo "Arm is ready."
echo "  Node PID:     $NODE_PID"
echo "  Torque motors: 11-13 (seg 1), 14-16 (seg 2), 17-19 (seg 3)"
echo ""
echo "  TERMINAL 2 — torque back-end:"
echo "    cd VMC && python3 GUI_MultiVMC_Torque.py"
echo "  TERMINAL 3 — GUI:"
echo "    cd VMC && python3 GUI_MultiVMC.py"
echo ""
echo "Press Ctrl+C to stop."
trap "kill \$NODE_PID 2>/dev/null; echo 'Node stopped.'" INT TERM
wait
