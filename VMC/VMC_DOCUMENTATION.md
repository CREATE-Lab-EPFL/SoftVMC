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

Virtual model classes live in `VMC_utils/VirtualModels.py`. Constructor keyword
names below are exact (many differ from the `k`/`c` shorthand used in comments
elsewhere in the codebase):

| Class | Constructor | Description |
|---|---|---|
| `LinearSpring` | `stiffness` | f = stiffness · Δx |
| `TanhSpring` | `stiffness, max_force` | f = max_force · tanh(stiffness·Δx / max_force) |
| `ConstrainedLinearSpring` | `stiffness, n` | linear spring projected onto plane with normal `n` |
| `ConstrainedTanhSpring` | `stiffness, max_force, n` | tanh spring projected onto plane with normal `n` |
| `GaussianSpring` | `strength, sigma` | f = strength · exp(−‖Δx‖²/2σ²) · Δx |
| `ConstrainedGaussianSpring` | `strength, sigma, n` | Gaussian spring projected onto plane with normal `n` |
| `SigmoidSpring` | `kmin, kmax, threshold, alpha` | stiffness sigmoid-blends from kmin to kmax around `threshold` |
| `PolynomialSpring` | `stiffness, order, dist_norm` | f = stiffness · (Δx / dist_norm)^order |
| `DeadzoneLimitSpring` | `stiffness, angle_lower, angle_upper` | zero inside [angle_lower, angle_upper], linear outside |
| `LinearDamper` | `damping` | f = damping · ẋ |
| `TanhDamper` | `damping, max_force` | f = max_force · tanh(damping·ẋ / max_force) |
| `ConstrainedLinearDamper` | `damping, n` | linear damper projected onto plane with normal `n` |
| `ConstrainedTanhDamper` | `damping, max_force, n` | tanh damper projected onto plane with normal `n` |

---

## Running an Experiment

**Prerequisites:** ROS 2 Humble, conda env `finger`, `ros2_dynamixel` built.

`GUI_MultiVMC.py` and `GUI_MultiVMC_Torque.py` both do
`from GUI_MultiVMC_Config import (...)`, which Python resolves against the
**caller's current working directory** (both scripts prepend `os.getcwd()` to
`sys.path` for exactly this reason) — so **both** terminals below must be
launched from inside the *same* experiment directory, not from `VMC/`:

```bash
# Terminal 1 — hardware driver
bash start_arm.sh            # homes arm, sets latency, starts dynamixel_node

# Terminal 2 and 3 — from the SAME experiment directory:
cd LinearSpring               # or TanhSpring, ConstrainedSpring, etc.
python3 ../VMC/GUI_MultiVMC_Torque.py   # or GUI_MultiVMC_Noisy_Torque.py
python3 ../VMC/GUI_MultiVMC.py
```

If an experiment directory holds more than one named config (e.g.
`GaussianElement/GUI_MultiVMC_Config_fixed.py` and `_moving.py`), name the one
you want with `--config`:

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _fixed
python3 ../VMC/GUI_MultiVMC.py        --config _fixed
```

`_fixed` and `fixed` both work, and a name that does not exist lists what does.
Every command in a run must name the same variant — for `ButtonPressing/` the
driver checks this over `/vmc_config` and aborts on a mismatch. Copying a
variant over `GUI_MultiVMC_Config.py` still works, since that plain name is
what `--config` falls back to. See the experiment's own README.

On exit, the torque node saves a timestamped `.npz` + `.png` snapshot into an
`experiments/` subfolder of **the directory you launched it from** (e.g.
`LinearSpring/experiments/z_<DD_HHMM>.npz`) — move/rename it into the matching
`outputs/<name>/` subfolder before re-running the plotting notebook.

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
| `MultiVMC_MovingGoal_Pub.py` | Ramps `/VC_ref_pos` step-by-step from `DEFAULT_START` to `DEFAULT_GOALS` (and back, if `LOOP_BACK=True`) |
| `MultiVMC_publish_0.py` | Publishes zero reference (park position) |
| `read_npz.py` | Quick CLI viewer for saved `.npz` experiment files |

---

## Torque Limit

All motor commands are clamped to `±TAU_LIMIT` (set per-experiment in `GUI_MultiVMC_Config.py`) before being sent to the hardware. The default is 2.1 N·m per motor.

---

## Saving and Loading Data

The torque node saves on shutdown, gated by two flags in `GUI_MultiVMC_Config.py`:
`SAVE_PLOT`/`SAVE_DATA` (whether this config saves at all) and `COLLECTED_DATA`
(`True` = real data already collected for this config -- skip saving even if
`SAVE_PLOT` is `True`, so a rerun/demo/sanity-check can never overwrite it).
When a save is skipped this way, the node prints `[INFO] COLLECTED_DATA=True --
not saving` instead. Key names as written by `np.savez_compressed` in
`GUI_MultiVMC_Torque.py`:

```
z_<DD_HHMM>.npz
    t             (n,)          [s]     timestamps
    tau           (n, 9)        [N·m]   commanded motor torques
    tau_grav      (n, 9)        [N·m]   gravity-compensation torque component
    goal          (n, N_VC, 3)  [m]     reference position per virtual component
    comp_pos      (n, N_VC, 3)  [m]     measured (FK) position per virtual component
    force         (n,)          [g]     measured shear force, RAW load-cell units --
                                          multiply by 9.80665e-3 for N (load cell if wired)
    f_virtual     (n, 3)        [N]     task-space virtual force
    k_params      (n, N_VC)             spring parameter per component over time
    q_motor       (n, 9)        [rad]   motor positions
    q_motor_dot   (n, 9)        [rad/s] motor velocities
    comp_names    (N_VC,)               virtual component names
    comp_s_params (N_VC,)               arc-length fraction per component
    control_freq  (1,)          [Hz]    control loop frequency
    tau_limit     (1,)          [N·m]   torque clamp used during the run
```

`N_VC` is the number of virtual components in that experiment's config (`N_VC`
in `GUI_MultiVMC_Config.py`) — not fixed across experiments.

Load with:
```python
data = np.load('outputs/<subfolder>/file.npz')
q_motor, tau, t = data['q_motor'], data['tau'], data['t']
```
