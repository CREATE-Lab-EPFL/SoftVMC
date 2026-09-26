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
GUI_MultiVMC_Torque.py   — torque computation node (330 Hz, `CONTROL_FREQUENCY`)
        │ subscribes /joint_positions, /joint_velocities, /VC_Param, /VC_ref_pos,
        │            /force_shear (load cell, optional), /vmc_mark, /vmc_end_run
        │ computes τ = Σ Jᵢᵀ fᵢ  via  VMC_Multi_Modular
        │ publishes /goal_torque, /virtual_forces, /vmc_config
        ▼
dynamixel_node           — hardware interface (ros2_dynamixel package)
```

`/vmc_mark` (a `String` label) and `/vmc_end_run` (a `Bool`) do the same jobs as a Ctrl+C mark and a Ctrl+C end-of-run, from a script or from the GUI's close button; nothing publishes them unless a script such as `ButtonPressing/castle_demo.py`, `replay_goal.py` or a config with `END_RUN_ON_CLOSE` does. `/vmc_config` carries the torque node's config module name once, at start-up (see [Running an Experiment](#running-an-experiment) for what does *not* consume it).

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
| `TanhSpring` | `stiffness, max_force` | f = max_force · tanh(stiffness·Δx / max_force), per component |
| `ConstrainedLinearSpring` | `stiffness, n` | linear spring whose force acts **only along `n`** (P = n·nᵀ, f = P K P Δx); motion in the plane normal to `n` is free |
| `ConstrainedTanhSpring` | `stiffness, max_force, n` | the tanh spring, projected the same way |
| `GaussianSpring` | `strength, sigma` | f = strength · exp(−‖Δx‖²/2σ²) · Δx, Δx = point − centre, **not normalised**. `strength` is in **N/m** (the constructor docstring says "force amplitude (N)"; it is wrong). Peak `strength·σ·e^(−1/2)` at ‖Δx‖ = σ |
| `ConstrainedGaussianSpring` | `strength, sigma, n` | the Gaussian spring, with Δx projected onto `n` (force along `n` only) |
| `SigmoidSpring` | `kmin, kmax, threshold, alpha` | stiffness sigmoid-blends from kmin to kmax around `threshold` (per component, or on the norm with `element_wise=False`) |
| `PolynomialSpring` | `stiffness, order, dist_norm` | f = stiffness · (\|Δx\| / dist_norm)^order · Δx (stiffness grows with distance) |
| `DeadzoneLimitSpring` | `stiffness, angle_lower, angle_upper` | zero inside [angle_lower, angle_upper], linear outside; acts on one joint angle, not a task-space position |
| `LinearDamper` | `damping` | f = −damping · ẋ |
| `TanhDamper` | `damping, max_force` | f = −max_force · tanh(damping·ẋ / max_force) |
| `ConstrainedLinearDamper` | `damping, n` | linear damper acting only along `n`: f = −P D P ẋ |
| `ConstrainedTanhDamper` | `damping, max_force, n` | the tanh damper, projected the same way |

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
python3 ../VMC/GUI_MultiVMC_Torque.py
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
Every command in a run must name the same variant, and **nothing checks that
for you**: the torque node publishes its config module name on `/vmc_config`
once at start-up, but no node in this repo subscribes to it, so a mismatch
between terminals is silent. Compare the `[INFO] config: …` line each process
logs. (The GUI *does* refuse to start without `--run` when the config sets
`REQUIRE_RUN_NAME`, since it applies the run's strengths.) Copying a variant over
`GUI_MultiVMC_Config.py` still works, since that plain name is what `--config`
falls back to. See the experiment's own README.

On exit, the torque node saves a timestamped `.npz` + `.png` snapshot into an
`experiments/` subfolder of **the directory you launched it from** (e.g.
`LinearSpring/experiments/z_<DD_HHMM>.npz`) — move/rename it into the matching
`outputs/<name>/` subfolder before re-running the plotting notebook. With
`--run NAME` it writes `outputs/<OUTPUT_FOLDER>/NAME.{npz,png}` directly instead,
overwriting a previous run of that name.

---

## File Reference

| File | Role |
|---|---|
| `MultiVMCTaskSpace.py` | `VirtualComponent` dataclass; `VMC_Multi_Modular` controller |
| `MultiController.py` | PD motor-space controller, `CONTROL_FREQUENCY` constant |
| `MultiGravityComp.py` | Gravity compensation (`MultiGravComp`, 40 g lumped mass per segment; wraps `VMC_utils/GravityCompensation.py`). Always used by the torque node, oriented by `GRAVITY_DIR` |
| `MultiGoHome.py` | Position-mode homing routine (run before torque experiments); `--read` prints the 9 motors' absolute positions |
| `MultiCheckBus.py` | Read-only Dynamixel bus health check (no ROS, no motion) — run it when the driver logs `SyncRead failed` or homing times out |
| `GUI_MultiVMC.py` | tkinter GUI + ROS 2 bridge |
| `GUI_MultiVMC_Torque.py` | Main torque computation node |
| `GUI_MultiVMC_Config.py` | The template/default config (`TAU_LIMIT = 0.25`); every experiment folder has its own |
| `MultiVMC_MovingGoal_Pub.py` | Ramps `/VC_ref_pos` step-by-step from `DEFAULT_START` to `DEFAULT_GOALS` (and back, if `LOOP_BACK=True`) |
| `MultiVMC_publish_0.py` | Continuously publishes `DEFAULT_START` (10 Hz) so the arm holds its home pose while you test external perturbations |
| `read_npz.py` | Quick CLI viewer for saved `.npz` experiment files: re-plots one, crops a time window, or writes a cropped `.npz` |
| `ros2_node_graphs/control_architecture.dot` | Graphviz source of the node/topic diagram |

---

## Torque Limit

All motor commands are clamped to `±TAU_LIMIT` (set per-experiment in `GUI_MultiVMC_Config.py`) before being sent to the hardware. There is **no default in the node**: every config must define it, and the values in use are 0.15 N·m (`GaussianElement`), 0.25 (most, and the `VMC/` template), 0.35 (`ButtonPressing` `_person`, `_card_castle`) and 0.5 (`ConnectivityTest`). The pre-clamp torque is logged as `tau_raw`, the clamped one as `tau`.

---

## Saving and Loading Data

The torque node saves on shutdown, gated by two flags in `GUI_MultiVMC_Config.py`:
`SAVE_PLOT`/`SAVE_DATA` (whether this config saves at all) and `COLLECTED_DATA`
(`True` = real data already collected for this config -- skip saving even if
`SAVE_PLOT` is `True`, so a rerun/demo/sanity-check can never overwrite it).
When a save is skipped this way, the node prints `[INFO] COLLECTED_DATA=True --
not saving` instead. Key names as written by `np.savez_compressed` in
`GUI_MultiVMC_Torque.py` (the arrays are logged at control rate, 330 Hz):

```
<name>.npz  (or z_<DD_HHMM>.npz)
    t              (n,)          [s]     seconds from the first /VC_ref_pos (t < 0 = before it)
    t_wall         (n,)          [s]     unix time
    tau            (n, 9)        [N·m]   commanded motor torques, AFTER the clamp
    tau_raw        (n, 9)        [N·m]   the same, BEFORE the clamp (clipping = |tau_raw| > tau_limit)
    tau_vmc        (n, 9)        [N·m]   virtual-model term, before the clamp
    tau_grav       (n, 9)        [N·m]   gravity-compensation term
    tau_pretension (n, 9)        [N·m]   pretension term (PRETENSION_WEIGHTS · q_motor)
    tau_comp       (n, N_VC, 9)  [N·m]   each element's share of tau_vmc
    goal           (n, N_VC, 3)  [m]     reference position per virtual component, as applied
    comp_pos       (n, N_VC, 3)  [m]     measured (FK) position per virtual component
    comp_vel       (n, N_VC, 3)  [m/s]   task-space velocity per component
    f_comp         (n, N_VC, 3)  [N]     each element's force separately
    f_virtual      (n, 3)        [N]     their sum, the task-space virtual force
    force          (n,)          [g]     /force_shear, RAW load-cell grams, sign flipped by
                                           the publisher -- multiply by 9.80665e-3 for N;
                                           identically 0 if no load cell is publishing
    force_age      (n,)          [s]     staleness of that reading
    k_params       (n, N_VC)             spring parameter per component over time
    c_params       (n, N_VC)             damper parameter per component over time
    q_motor        (n, 9)        [rad]   motor positions
    q_motor_dot    (n, 9)        [rad/s] motor velocities
    marks_t / marks_wall / marks_label (m,)   marks made during the run (Ctrl+C or /vmc_mark)
  metadata:
    comp_names, comp_s_params, comp_spec (JSON of every element), spring_param_names,
    damper_param_names, control_freq [Hz], tau_limit [N·m], gravity_dir, pretension_weights,
    pretension_enabled, config_name, config_source (the whole config file), run_name,
    ramp_components, ramp_strength, ramp_s, t_reference_ok
    condition_targets, condition_strength   -- old names of ramp_components / ramp_strength,
                                               still written alongside them
```

`N_VC` is the number of virtual components in that experiment's config (`N_VC`
in `GUI_MultiVMC_Config.py`) — not fixed across experiments.

**Older runs carry fewer fields.** The repo holds four generations:

| fields | where | what is missing |
|---|---|---|
| 12 | 5 runs: `LinearSpring/k_saturation` (3), `GaussianElement/fixed_gauss` `fixed_gauss1_no_q`, `fixed_gauss2_no_q` | the 12 listed as `t`, `tau`, `tau_grav`, `goal`, `comp_pos`, `force`, `f_virtual`, `k_params`, `comp_names`, `comp_s_params`, `control_freq`, `tau_limit` — no `q_motor` / `q_motor_dot` |
| 14 | 39 runs: `ConstrainedSpring`, `TanhSpring`, `LinearSpring/disturbance_rejection`, all of `ContactAndShaping` except `bending_*`, `GaussianElement`'s `fixed_gauss3–5`, `k2_` and `k20_moving_gauss` | the 12 above plus `q_motor`, `q_motor_dot`; still no `tau_raw`, `f_comp`, `comp_spec`, `config_source`, `gravity_dir`, marks, … — so the config that made the run is **not stored** |
| 39 | the 20 `ButtonPressing/button_pressing` runs | everything in the list above except `ramp_components`, `ramp_strength` (they carry the `condition_*` names) |
| 41 | the 11 recorded since: `ButtonPressing` `person_avoiding` and `card_castle`, `ContactAndShaping` `bending_1–6`, `GaussianElement` `k4_moving_gauss` | nothing |

Code that reads a run should test for a field before using it (the
`ButtonPressing` notebooks do: `d.get('tau_raw', d['tau'])`). The logged
stiffness values of the oldest runs are not always the number in their file
name — see the `LinearSpring/` and `GaussianElement/` READMEs.

Load with:
```python
data = np.load('outputs/<subfolder>/file.npz')
q_motor, tau, t = data['q_motor'], data['tau'], data['t']
```
