echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
ros2 run dynamixel_interface dynamixel_node --ros-args \
-p baudrate:=1000000 \
-p motor_ids:=[11,12,13,14,15,16,17,18,19] \
-p control_mode:=torque
