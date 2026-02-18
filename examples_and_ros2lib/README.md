# examples_and_ros2lib

Three components for running Virtual Model Control on soft tendon-driven robots:

```
examples_and_ros2lib/
├── ros2_dynamixel/       # ROS2 driver: Dynamixel motors <-> ROS2 topics
├── jacobians_example/    # Symbolic Jacobians for single/multi-section soft structures
└── fingerVMCexamples/    # VMC implementation for a single tendon-driven finger
```

## How the components connect

```
                    ┌─────────────────────┐
                    │   ros2_dynamixel     │  C++ node @ 1 kHz
                    │  /joint_positions    │──────────────────┐
                    │  /joint_velocities   │                  │
                    │  /goal_torque  <─────│──────────┐       │
                    └─────────────────────┘          │       │
                                                      │       │
                    ┌─────────────────────┐           │       │
                    │  jacobians_example   │           │       ▼
                    │  q2coordinates()     │    ┌──────────────────────┐
                    │  q2tendon()          │───>│  fingerVMCexamples   │
                    │  tendon2pose()       │    │  FK + VMC controller │
                    └─────────────────────┘    │  springs / dampers   │
                                               └──────────────────────┘
```

**Data flow at runtime:**
1. `ros2_dynamixel` reads motor positions/velocities and publishes them
2. `fingerVMCexamples` reads the state, evaluates Jacobians, computes virtual forces
3. Torques go back to `ros2_dynamixel` which commands the motors

## Quick start

**1. Build and source the ROS2 driver** (once):
```bash
cd ros2_dynamixel && colcon build && source install/setup.bash
```

**2. Reduce USB latency** (once per boot):
```bash
sudo echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

**3. Start the motor node** (terminal 1):
```bash
ros2 run dynamixel_interface dynamixel_node
```

**4. Run a VMC example** (terminal 2, from SoftVMC root):
```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)/examples_and_ros2lib/fingerVMCexamples"
python3 examples_and_ros2lib/fingerVMCexamples/FingerReferenceCodes/FingerVMCPressing.py
```

## Component READMEs

- [ros2_dynamixel/README.md](ros2_dynamixel/README.md) — installation, kT, build, topics
- [jacobians_example/README.md](jacobians_example/README.md) — kinematic model, Jacobian API, usage
- [fingerVMCexamples/README.md](fingerVMCexamples/README.md) — VMC structure, setup, examples
