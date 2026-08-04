# VMC Documentation — Virtual Model Control for the Soft Continuum Arm

## Overview

Virtual Model Control (VMC) maps forces from virtual mechanical elements (springs, dampers) attached to points along the arm to motor torques via the Jacobian transpose:

```
τ = Σᵢ Jᵢ(q)ᵀ · fᵢ(xᵢ, ẋᵢ)
```

where `xᵢ = FK(q, sᵢ)` is the Cartesian position of the attachment point at arc fraction `sᵢ ∈ [0,1]`.

---

## Node Architecture

```
GUI_MultiVMC.py          — tkinter GUI + ROS 2 publisher
        │ publishes /VC_Param, /VC_ref_pos
        ▼
GUI_MultiVMC_Torque.py   — torque computation node
        │ subscribes /joint_positions, /joint_velocities, /VC_Param, /VC_ref_pos
        │ computes τ = Σ Jᵢᵀ fᵢ  via  VMC_Multi_Modular
        │ publishes /goal_torque
        ▼
dynamixel_node           — hardware interface (ros2_dynamixel package)
```

A config file `GUI_MultiVMC_Config.py` (one per experiment, placed in the experiment directory) defines `VIRTUAL_COMPONENTS`, `DEFAULT_START`, limits, and GUI parameters.

---

## Virtual Components

Each `VirtualComponent` (defined in `MultiVMCTaskSpace.py`) has:

| Field | Type | Description |
|---|---|---|
| `spring` | VirtualModel | Force law as function of displacement |
| `damper` | VirtualModel | Force law as function of velocity |
| `s_param` | float ∈ [0,1] | Arc fraction of attachment point |
| `name` | str | Label for logging |

Virtual model classes live in `VMC_utils/VirtualModels.py`:

| Class | Force law | Key params |
|---|---|---|
| `LinearSpring` | f = k·Δx | k [N/m] |
| `LinearDamper` | f = c·ẋ | c [N·m·s/rad] |
| `TanhSpring` | f = f_max · tanh(k·Δx / f_max) | k, f_max |
| `TanhDamper` | f = c_max · tanh(c·ẋ / c_max) | c, c_max |
| `ConstrainedLinearSpring` | linear spring projected onto a plane | k, normal |
| `GaussianSpring` | f = k · exp(−‖Δx‖²/2σ²) · Δx | k, σ |
| `DeadzoneLimitSpring` | zero inside deadzone, linear outside | k, deadzone |

---

## Running an Experiment

**Prerequisites:** ROS 2 Humble, conda env `finger`, `ros2_dynamixel` built.

```bash
# Terminal 1 — hardware driver
bash start_arm.sh            # homes arm, sets latency, starts dynamixel_node

# Terminal 2 — torque back-end (from repo root)
cd VMC
python3 GUI_MultiVMC_Torque.py   # or GUI_MultiVMC_Noisy_Torque.py

# Terminal 3 — GUI (from the experiment directory, so it picks up the right config)
cd LinearSpring               # or TanhSpring, ConstrainedSpring, etc.
python3 ../VMC/GUI_MultiVMC.py
```

On exit the torque node saves a timestamped `.npz` + `.png` snapshot in the working directory.

---

## File Reference

| File | Role |
|---|---|
| `MultiVMCTaskSpace.py` | `VirtualComponent` dataclass; `VMC_Multi_Modular` controller |
| `MultiController.py` | PD motor-space controller, `CONTROL_FREQUENCY` constant |
| `MultiGravityComp.py` | Gravity compensation (optional, wraps `VMC_utils/GravityCompensation.py`) |
| `MultiGoHome.py` | Position-mode homing routine (run before torque experiments) |
| `GUI_MultiVMC.py` | tkinter GUI + ROS 2 bridge |
| `GUI_MultiVMC_Torque.py` | Main torque computation node |
| `GUI_MultiVMC_Noisy_Torque.py` | Torque node with configurable sensor noise injection |
| `MultiVMC_MovingGoal_Pub.py` | Publishes a moving reference position (sinusoidal sweep) |
| `MultiVMC_publish_0.py` | Publishes zero reference (park position) |
| `read_npz.py` | Quick CLI viewer for saved `.npz` experiment files |

---

## Torque Limit

All motor commands are clamped to `±TAU_LIMIT` (set per-experiment in `GUI_MultiVMC_Config.py`) before being sent to the hardware. The default is 1.5 N·m per motor.

---

## Saving and Loading Data

The torque node saves on shutdown:

```
<timestamp>_<experiment>.npz
    q          (N, 9)   motor positions [rad]
    dq         (N, 9)   motor velocities [rad/s]
    tau        (N, 9)   commanded torques [N·m]
    pos_ee     (N, 3)   end-effector position [m]
    ref_pos    (N, 3M)  reference positions (M VCs) [m]
    time       (N,)     timestamps [s]
```

Load with:
```python
data = np.load('outputs/<subfolder>/file.npz')
q, tau, t = data['q'], data['tau'], data['time']
```
