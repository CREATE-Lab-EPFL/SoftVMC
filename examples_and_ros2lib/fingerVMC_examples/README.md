# fingerVMCexamples

Virtual Model Control (VMC) for a single tendon-driven finger. Implements forward kinematics, symbolic Jacobians, and virtual spring/damper control in both task space and joint space.

## Hardware

- **3 DOF**: MCP (40 mm), PIP (30 mm), DIP (17.5 mm)
- **2 motors**: one for MCP, one for PIP/DIP (DIP mimics PIP via cable routing)
- **Communication**: via `ros2_dynamixel` (see `../ros2_dynamixel/`)

## Structure

```
fingerVMCexamples/
├── KinematicsFinger/
│   ├── FK_Finger.py          # Forward kinematics: FK(q_motor, link, r_local) -> position [m]
│   ├── JacobiansFinger.py    # Symbolic Jacobians: q2joint, q2MCP, q2PIP, q2DIP
│   └── HessiansFinger.py     # Hessians (2nd-order terms)
├── VMCFinger/
│   ├── FingerController.py   # ROS2 node base: subscribes positions/velocities, publishes torques
│   ├── FingerVMCTaskSpace.py # VMC in Cartesian space (springs/dampers at MCP and fingertip)
│   ├── FingerVMCFingerSpace.py # VMC directly on joint angles
│   └── FingerGravFricLim.py  # Gravity + Stribeck friction + joint limit compensation
└── FingerReferenceCodes/
    ├── FingerVMCPressing.py          # Pressing task: VMC + full compensation
    ├── FingerVMCVariableStiffness.py # Variable stiffness modulation
    ├── constant_stiffness_modulation.py
    └── FingerTipForce/               # Tip-force sensing examples
```

## How VMC works (task space)

1. Compute current fingertip position via FK
2. A virtual spring pulls the tip toward a target: **F = K · (target − current)**
3. A virtual damper opposes velocity: **F = −D · v**
4. Map virtual forces to motor torques via Jacobian transpose: **τ = Jᵀ · F**
5. Add gravity/friction compensation and send to `/goal_torque`

```
motor angles (q) ──FK──> tip position (x)
                 ──J──>  tip velocity (ẋ = J·q̇)
virtual forces (F) ──Jᵀ──> motor torques (τ)
```

## Setup

**1. Start the Dynamixel node** (in a separate terminal, from `ros2_dynamixel/`):
```bash
source install/setup.bash
ros2 run dynamixel_interface dynamixel_node
```

**2. Set PYTHONPATH** so both `VMC_utils` (at SoftVMC root) and `KinematicsFinger` (here) are importable:
```bash
# From the SoftVMC root:
export PYTHONPATH="${PYTHONPATH}:$(pwd)/examples_and_ros2lib/fingerVMCexamples"
```

**3. Run an example**:
```bash
python3 examples_and_ros2lib/fingerVMCexamples/FingerReferenceCodes/FingerVMCPressing.py
```

## Dependencies

Python: `numpy`, `sympy`, `rclpy`, `std_msgs`

All units are SI internally: lengths [m], angles [rad], forces [N], torques [N·m]. ROS2 topics use degrees (converted on receive/send).
