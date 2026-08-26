# ButtonPressing

A linear spring reaching for a button, plus two repulsive Gaussians pushing the
arm's body off an obstacle. Sweeping the Gaussian strength trades reach against
clearance. Driven live from the GUI.

The two variants carry **identical values** — same target, stiffness, torque
limit, pretension, ramp — and differ only in what the obstacle is and how many
strengths are swept:

| | `_button` | `_person` |
|---|---|---|
| obstacle | a fixture | a person's arm |
| load cell | yes (`/force_shear`) | no |
| runs | `no_gauss`, `weak`, `medium`, `strong` | `no_gauss`, `medium`, `strong` |
| runs land in | `outputs/button_pressing/` | `outputs/person_avoiding/` |

`GUI_MultiVMC_Config_person.py` is generated from the button one, so nothing can
drift between them: edit a value in one and copy it across. `GAUSS_CONDITIONS`
is the **one** field that is deliberately different.

## Elements

| Component | Element | s | Goal |
|---|---|---|---|
| `lin_tip` | `LinearSpring(30)` | 1.0 | starts `[0, 0, 0.38]`; drag x/y onto the button |
| `gauss_seg1` | `GaussianSpring(k, 0.06)` | 1/3 | starts `[-0.05, 0, 0.2175]` |
| `gauss_seg2` | `GaussianSpring(k, 0.06)` | 2/3 | **shares `gauss_seg1`'s goal** |

`tau = sum_i J_i^T F_i`. Both z values are held — z is the reach **out along the
arm's axis**, not height (see [Gravity and axes](#gravity-and-axes)): the
button's distance from the base, and the midpoint between the tips of sections 1
and 2 (`rest_pose(0.5)`, which also puts both attachment points equidistant from
the centre).

Only x/y is placed by hand, and x/y is (across, **up**). The order matters and
is the one to keep: let the Gaussians ramp in and push the arm off the obstacle
first, wait for it to settle, and only then drag `lin_tip` out to the button.
Moving both at once is a different experiment. Aim the tip a little past the
surface, or the steady-state force decays to zero on arrival.

The Gaussian centre is placed by hand too. Its `GAUSS_GOAL_XY = (-0.05, 0.0)`
default is a **starting point, not a placement** — it sits 5 cm off the arm's own
rest axis with zero y offset, which leaves the direction the arm escapes in
barely determined. Drag it onto the obstacle.

⚠ Dragging is what makes the conditions hard to compare: `weak` and `medium`
landed 7.4 cm apart in x and 5.4 cm apart in y, pressing a different spot on the
cell at a different angle, which moves the shear reading for reasons that have
nothing to do with the Gaussian. Get the tip to the same place each run.

The Gaussian centre is still placed by hand. Its `GAUSS_GOAL_XY = (-0.05, 0.0)`
default is a **starting point, not a placement** — it sits 5 cm off the arm's own
rest axis with zero y offset, which leaves the direction the arm escapes in
barely determined. Drag it onto the obstacle.

`K_REACH = 30` N/m, `C_DAMP = 1.0` N·s/m, `TAU_LIMIT = 0.25` N·m,
`PRETENSION_WEIGHTS = 0.010` N·m/rad. Springs and dampers are live sliders in
the GUI.

## Run names

`_button` — four:

| `--run` | strength | | `--run` | strength |
|---|---|---|---|---|
| `no_gauss` | 0 | | `medium` | 1000 |
| `weak` | 500 | | `strong` | 2000 |

`_person` — three. Every run needs someone holding an arm still in front of a
moving robot, so the sweep drops `weak`. `medium` and `strong` are the button's
own two, unchanged, so the fixture and the person can be compared at an
identical field:

| `--run` | strength |
|---|---|
| `no_gauss` | 0 |
| `medium` | 1000 |
| `strong` | 2000 |

The name sets the strength: `--run` applies `GAUSS_CONDITIONS` to both
Gaussians, in the torque node and on the GUI sliders. It is **ramped in from 0
over `GAUSS_RAMP_S` = 5 s** (which happens as soon as terminal 3 is up, so it
lands at `t < 0` in the log, before the GUI connects) rather than applied at once — the obstacle centre is
already placed, so a field switched on at full strength would shove the arm off
it in a single tick. Touching **Apply Parameters** cancels the ramp and hands
control to the sliders; `k_params` records what was actually applied.

A name this config has no condition for is **refused**, in a second, before
anything is built — so a `weak` typed at `_person`, or the retired `mid_1`
anywhere, stops with the valid names listed instead of quietly recording at
strength 0 under a name that reads like a condition:

```
[ERROR] --run 'mid_1' is not a condition of GUI_MultiVMC_Config_button.
        this config knows: no_gauss (0), weak (500), medium (1000), strong (2000)
```

Drop `--run` altogether for a timestamped scratch run in `experiments/`. For
`_person`, record `strong` first.

⚠ These strengths sweep how far the arm is pushed off the obstacle, not how
hard it presses. Measured from a 1000 N/m run: at the straight rest
pose the Gaussian torque is 0.418 N·m per 1000 N/m of strength against a tip
torque of 0.346 N·m, so the field already outweighs the reach at ≈ 830 N/m and
the arm settles 23 cm off the button with **zero** contact force. Modulating a
*press* is a much smaller quantity — the tip spring working on its overshoot
past the surface, 0.15–1.5 N — and needs strengths of order 12–124 depending on
how far past the surface the goal is aimed. So check the shear reading is still
above zero at `strong`; if it is not, the sweep is showing on/off rather than
modulation, and the fix is a larger overshoot (or a smaller `SIGMA`: repulsion
is `k · exp(−d²/2σ²) · d`, and with the obstacle 1.5σ away the field is already
down to 34% before `k` multiplies).

## Running

Two ways: the **sweep** records every condition unattended and is what the
reported data should come from; the **manual** route is four terminals and a
mouse, for exploring.

### The sweep

```bash
conda activate finger
cd ButtonPressing
python3 ../LoadCell/arduino_force.py     # its own terminal, `_button` only

python3 run_sweep.py                     # _button: no_gauss, weak, medium, strong
python3 run_sweep.py --config _person    # _person: no_gauss, medium, strong
python3 run_sweep.py --only weak strong
python3 run_sweep.py --dry-run           # print the plan, touch no hardware
```

**Do not run `start_arm.sh`** — the sweep owns `dynamixel_node`, because the
driver cannot change operating mode while running and holds `/dev/ttyUSB0`
exclusively. Per condition it does:

| step | set by | |
|---|---|---|
| | | launch the driver (torque mode), wait for `/joint_positions` |
| | | launch the torque node with `--run <condition>` |
| | | publish `/VC_ref_pos` from the script — **no GUI** — tip goal at the straight-arm tip `[0, 0, 0.435]`, so the linear spring pulls **nothing** |
| settle | `--settle-gauss` 15 s | the Gaussians ramp in and push the arm off the obstacle, alone |
| move | `--ramp-speed` 2 cm/s | tip goal walks to `[-0.30, 0, 0.38]`, 30.5 cm, 15 s |
| settle | `--settle-arrive` 15 s | the arm follows it and stops |
| | | Ctrl+C #1 → marks *"target reached"* |
| window | `--window` 20 s | **the measurement window** |
| | | Ctrl+C #2 → writes `outputs/<folder>/<condition>.{npz,png}` |
| | | kill the driver to free the bus |
| home | `--home-velocity` 10 | extended position mode, ~2.3 rpm, back to `HOME_POSITION`, **verified tick for tick** |
| settle | `--settle-home` 5 s | hand back in current mode, next condition |

≈118 s per condition: **7.8 min** for the button's four, **5.9 min** for the
person's three. The homing leg is the longest single item (~30 s) because the
arm ends a run about 1.15 turns wound.

Every condition therefore starts from the same pose and drives to the same
target, which is the point — the only difference between them is the Gaussian
strength. Recorded by hand they were not comparable: `weak` and `medium` landed
7.4 cm apart in x and 5.4 cm apart in y, pressing a different spot on the cell
at a different angle.

Any failure — homing lands a turn out, a motor does not arrive, the torque node
dies — **aborts the sweep**, leaves the motors holding, does not restart the
driver, and names the conditions already recorded. Nothing is recorded from a
zero that was not verified.

### By hand

Four terminals, each starting from:

```bash
conda activate finger
cd ButtonPressing
```

**Terminal 1** — hardware

```bash
bash ../start_arm.sh
```

**Terminal 2** — load cell (`_button` only, skip for `_person`)

```bash
python3 ../LoadCell/arduino_force.py
```

**Terminal 3** — torque node

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _button --run medium
```

**Terminal 4** — GUI

```bash
python3 ../VMC/GUI_MultiVMC.py --config _button --run medium
```

Terminals 3 and 4 must name the same variant and the same run. Terminal 3 spends
its first half-minute in sympy building the Jacobians, then logs what it loaded
— check the config, the run and the gravity line before moving anything:

```
[INFO] building Jacobians (sympy -- takes a while, Ctrl+C to abort)
[INFO] condition medium: gauss_seg1, gauss_seg2 -> strength 1000.0, ramped in over 5 s
[INFO] Jacobians ready in 31 s
[INFO] config: GUI_MultiVMC_Config_button
[INFO] run: medium -- saving to ~/github/SoftVMC/ButtonPressing/outputs/button_pressing/medium.{png,npz}, overwriting any previous medium
[INFO] gravity compensation: gravity pulls along -y in the robot frame
[INFO] Ctrl+C in THIS terminal: mark 'Target reached?' -> end the run, write medium.npz
[INFO] checkpointing every 30 s to medium.autosave.npz
[INFO] VMCTorqueNode started — 3 virtual component(s), control @ 330 Hz
```

A Ctrl+C during that build just aborts — nothing has been recorded yet, so it
exits with one line rather than a sympy traceback.

The torque node drives the arm from its **first tick**, before the GUI is up: the
goal is already `DEFAULT_START`, so the arm takes up the 5.5 cm to it (≈1.65 N at
`K_REACH = 30`) and the run's Gaussian ramps in, all of it logged at `t < 0`.
Untick **Publisher Active** in the GUI to move goals without the arm following.

`t = 0` in the log is the GUI's **first** `/VC_ref_pos`, and the GUI publishes
only when a goal is clicked or dragged — so `t = 0` is the moment you first
touch the canvas, not when the GUI opened. Everything before it, including the
take-up and the Gaussian ramp, is at `t < 0`; `t_reference_ok` is False for a
run where nothing was ever dragged.

**Ctrl+C in terminal 3 does two things, in order** (`MARK_PROMPTS` in the
config; terminal 3 prints the protocol when it starts, and what to press next
after each one):

| press | |
|---|---|
| 1st | marks *"Target reached?"* — press it once the tip is on the button |
| 2nd | ends the run and writes it — **once**, then wait |

The mark is what tells the analysis where the measurement window starts.
Without one it has to be guessed as the longest stretch you did not touch
anything, which cannot tell "on the button" from "let go of the mouse".

For the other variant, swap `_button` for `_person` in terminals 3 and 4, and
skip terminal 2.

## Gravity and axes

The arm is mounted **on its side**, so both configs set `GRAVITY_DIR = "-y"`.
Override for one run with `--gravity-dir -z`. Wrong value = compensation pushing
along an axis gravity no longer acts on.

That mounting is also what the robot axes mean physically, and it is easy to get
backwards:

| axis | physically | in the GUI |
|---|---|---|
| `x` | horizontal, across | canvas X |
| `y` | **vertical — up is +y** | canvas Y |
| `z` | out along the arm's axis | the locked slider |

So the drag canvas is (across, **up**), and a "z" in this folder is always a
distance out from the base, never a height.

## Data

`--run NAME` writes `outputs/<folder>/NAME.{npz,png}` on exit, **overwriting**
that run. Without it, output is a timestamped `experiments/z_<stamp>` pair for
`collect_run.py` to move. Needs `COLLECTED_DATA = False`; set it `True` once the
reported runs are recorded and nothing can overwrite them.

The full log is checkpointed to `NAME.autosave.npz` every `AUTOSAVE_S` = 30 s
while the run goes, with a status line printed, so a crash or a `kill -9` now
costs the last few seconds rather than the run. That file is deleted once the
real one is written, so **a leftover `.autosave.npz` means the run did not exit
cleanly** — rename it over `NAME.npz` to recover it. A `NAME.npz.part.npz` is a
write that was interrupted: truncated, unreadable, delete it. Writing through
that temporary is why an interrupted save can no longer destroy a good `NAME.npz`.

The final save still happens on exit, so Ctrl+C once (after the mark) and wait.
The `.npz` goes first, so an interrupt costs the figure rather than the run.
Launch T3 from this folder; paths are relative to it.

| field | shape | |
|---|---|---|
| `t` | (n,) | s, 0 at the first reference message. **`t < 0` is the stretch before the GUI connected**, when the arm was already being driven towards `DEFAULT_START` and the Gaussian ramp ran (kept: last `PRE_REFERENCE_S` = 60 s) |
| `t_wall` | (n,) | unix time, to line the run up with other logs |
| `goal` | (n, 3, 3) | reference per element [m] — where the goals were |
| `comp_pos` / `comp_vel` | (n, 3, 3) | FK position [m] and task-space velocity [m/s] per element |
| `f_comp` | (n, 3, 3) | **each element's force separately** [N]; `f_virtual` (n, 3) is still their sum |
| `tau_comp` | (n, 3, 9) | each element's share of the torque [N·m] |
| `tau_vmc` / `tau_grav` / `tau_pretension` | (n, 9) | the three terms, before the limiter |
| `tau_raw` / `tau` | (n, 9) | asked for vs sent — clipping is `abs(tau_raw) > tau_limit` |
| `q_motor` / `q_motor_dot` | (n, 9) | [rad], [rad/s] |
| `k_params` / `c_params` | (n, 3) | spring and damper parameter per element over time |
| `force` / `force_age` | (n,) | load-cell shear, raw **grams** (×`9.80665e-3` for N), and how stale that reading was [s] |
| `marks_t` / `marks_label` / `marks_wall` | (m,) | where Ctrl+C was pressed, on the same clock as `t` |
| `comp_spec` | (1,) | every spring/damper type and value, as JSON |
| `config_source` | (1,) | the whole config file, so a run stays readable after the numbers move on |
| `comp_names`, `comp_s_params`, `spring_param_names`, `damper_param_names`, `control_freq`, `tau_limit`, `gravity_dir`, `config_name`, `run_name`, `condition_targets`, `condition_strength`, `ramp_s`, `pretension_weights`, `pretension_enabled`, `t_reference_ok` | | metadata (`t_reference_ok` is False if the GUI never connected, so `t = 0` is the first sample rather than the first reference) |

No single projection is the geometry. Because y is vertical, an **x–z plot is a
plan view from above**, not a side view, and the arm has been recorded **lifted
16 cm** by the +y component of the Gaussian — invisible from above. Read
clearance and reach error in 3-D: an x–z view of one run shows the arm grazing
the obstacle at 0.8 cm where the true distance is 12.2 cm.

## Files

| File | |
|---|---|
| `GUI_MultiVMC_Config_button.py` / `_person.py` | the elements, their values, the conditions |
| `plot_button_and_person.ipynb` | analysis — discovers `outputs/`, takes positions from the data and the measurement window from the Ctrl+C mark, falling back to the longest untouched stretch for runs without one |
| `run_sweep.py` | the unattended sweep: every condition, same start pose, same target, homed and verified between |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` (only the by-hand route makes those; the sweep always names its runs) |

The scripted state machine that used to drive this folder is in the git history.
