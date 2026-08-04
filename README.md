# SoftVMC

Virtual Model Control (VMC) of a 3-segment soft continuum manipulator (Helyx).  Kinematics are modelled with **Piecewise Constant Curvature (PCC)**; control is achieved by attaching virtual mechanical elements (springs, dampers) to points along the arm body and computing the resulting motor torques via the Jacobian transpose.

> **Philosophy:** the robot receives *suggestions* rather than rigid commands, exploiting its inherent compliance to interact safely with the environment and adapt its posture through contact.

---

## Repository Structure

```
SoftVMC/
├── Kinematics/          FK and analytical Jacobians (PCC, 3 segments, 9 motors)
├── VMC/                 Core VMC controller + ROS 2 nodes (GUI, torque, go-home)
├── VMC_utils/           Shared building blocks (virtual models, gravity comp)
│
├── LinearSpring/        Experiments: k-saturation, disturbance rejection
├── TanhSpring/          Experiments: force-limited spring (sequential jump tests)
├── ConstrainedSpring/   Experiments: surface-sliding (constrained/cart spring)
├── GaussianElement/     Experiments: obstacle avoidance (fixed & moving Gaussian)
├── ContactAndShaping/   Experiments: contact, whole-body shaping, surface tracking
├── ButtonPressing/      Experiments: avoid / exploit / minimize (new, in progress)
│
├── SingleVMC/           1-segment VMC (reference / simulation only)
├── ros2_dynamixel/      ROS 2 Dynamixel driver package
├── LoadCell/            Load cell interface utilities
│
├── plot_config.mplstyle  Matplotlib style (LaTeX, NPG colors, 18 pt)
├── plot_config.py        Plotting helpers (COLORS, FIG_W_*, set_font_size)
└── start_arm.sh          Full startup script (home → latency → dynamixel_node)
```

---

## Setup

**Conda environment:** `finger` (Python 3.10, ROS 2 Humble)

```bash
conda activate finger
```

**ROS 2 driver** (build once):

```bash
cd ros2_dynamixel
colcon build
source install/setup.bash
```

---

## Quick Start

```bash
# 1. Start the arm (homes motors + starts ROS 2 driver)
bash start_arm.sh

# 2. From an experiment directory — pick up its VMC config automatically
cd LinearSpring
python3 ../VMC/GUI_MultiVMC_Torque.py &   # torque computation node
python3 ../VMC/GUI_MultiVMC.py            # visualization + control GUI
```

---

## Kinematics

Each segment is parametrized by `(Dx, Dy, Dl)` — lateral curvatures and elongation.  Nine Dynamixel motors (IDs 11–19, three per segment) control tendon lengths.  The mapping `q_motor ↔ PCC ↔ Cartesian pose` is implemented in `Kinematics/`.

See [Kinematics/KINEMATICS_DOCUMENTATION.md](Kinematics/KINEMATICS_DOCUMENTATION.md) for the full derivation.

---

## Virtual Element Library

| Element | Force law | Use case |
|---|---|---|
| `LinearSpring` | f = k Δx | Position control, stiffness characterization |
| `LinearDamper` | f = c ẋ | Velocity damping |
| `TanhSpring` | f = f_max tanh(k Δx / f_max) | Bounded interaction force |
| `ConstrainedSpring` | linear spring on normal direction | Sliding contact |
| `GaussianSpring` | f ∝ exp(−‖Δx‖²/2σ²) | Obstacle avoidance |
| `DeadzoneLimitSpring` | zero inside deadzone | Soft limits |

All elements are in `VMC_utils/VirtualModels.py`.  See [VMC/VMC_DOCUMENTATION.md](VMC/VMC_DOCUMENTATION.md) for the node architecture.

---

## Plotting Style

All notebooks use the shared NPG color palette and LaTeX rendering.  Import at the top of each notebook:

```python
import sys; sys.path.insert(0, '..')
from plot_config import COLORS, FIG_W_SINGLE, FIG_W_DOUBLE, set_font_size
set_font_size(18)
```

---

## Notes

- **ROS 2 version:** ROS 2 Humble (check `ros2_dynamixel` build if switching to Jazzy).
- **Motor kT:** updated for the current Dynamixel series — verify `MOTOR_RADIUS` in `Kinematics/JacobiansMulti.py` if motors are replaced.
- **Latency timer:** set to 1 ms for real-time performance (`start_arm.sh` does this automatically).
