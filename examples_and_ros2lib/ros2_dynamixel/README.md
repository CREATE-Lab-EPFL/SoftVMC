# ros2_dynamixel

ROS2 driver for Dynamixel servo motors. Publishes joint positions/velocities and subscribes to torque commands. Built for **ROS2 Humble** and the **XL330** motor series.

## Prerequisites

- ROS2 Humble (sourced: `source /opt/ros/humble/setup.bash`)
- Dynamixel SDK for ROS2: `sudo apt install ros-humble-dynamixel-sdk`
- USB-to-serial adapter connected to `/dev/ttyUSB0`

## Motor torque constant kT

**Check this before running.** kT converts torque commands [N·m] to motor current [mA]:

```
I [mA] = τ [N·m] / kT
```

kT is defined in [dynamixel_node.cpp](src/dynamixel_interface/src/dynamixel_node.cpp):

```cpp
const double KT = 0.000345;  // [N·m/mA]  — XL330 series @ 6V
```

This value varies by motor series and supply voltage. Verify it against the Dynamixel e-Manual for your specific motor before running.

## Build

```bash
cd ros2_dynamixel
colcon build
source install/setup.bash
```

## Run

**Reduce USB latency first** (do this once per boot):
```bash
sudo echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

**Single finger** (default: 2 motors, 1 Mbps):
```bash
ros2 run dynamixel_interface dynamixel_node
```

**Full ADAPT hand** (15 motors, 2 Mbps):
```bash
ros2 run dynamixel_interface dynamixel_node \
  --ros-args -p baudrate:=2000000 \
             -p motor_ids:=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]
```

**Position control mode** (default is torque):
```bash
ros2 run dynamixel_interface dynamixel_node --ros-args -p control_mode:=position
```

## ROS2 topics

| Topic | Type | Direction | Units |
|-------|------|-----------|-------|
| `/goal_torque` | `Float64MultiArray` | subscribe | N·m |
| `/joint_positions` | `Float64MultiArray` | publish | degrees |
| `/joint_velocities` | `Float64MultiArray` | publish | deg/s |

Control loop runs at **1 kHz** via synchronous SyncRead/SyncWrite over Dynamixel Protocol 2.0.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `motor_ids` | `[0, 1]` | List of Dynamixel IDs |
| `baudrate` | `1000000` | Serial baud rate |
| `control_mode` | `"torque"` | `"torque"` or `"position"` |
