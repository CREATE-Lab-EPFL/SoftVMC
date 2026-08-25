# ButtonPressing

A linear spring reaching for a button, plus two repulsive Gaussians pushing the
arm's body off an obstacle. Sweeping the Gaussian strength trades reach against
clearance. Driven live from the GUI.

The two variants carry **identical values** — same target, stiffness,
conditions, torque limit, pretension, ramp — and differ only in what the
obstacle is:

| | `_button` | `_person` |
|---|---|---|
| obstacle | a fixture | a person's arm |
| load cell | yes (`/force_shear`) | no |
| runs land in | `outputs/button_pressing/` | `outputs/person_avoiding/` |

`GUI_MultiVMC_Config_person.py` is generated from the button one, so nothing can
drift between them: edit a value in one and copy it across.

## Elements

| Component | Element | s | Goal |
|---|---|---|---|
| `lin_tip` | `LinearSpring(30)` | 1.0 | starts `[0, 0, 0.38]`; drag x/y onto the button |
| `gauss_seg1` | `GaussianSpring(k, 0.06)` | 1/3 | starts `[-0.05, 0, 0.2175]` |
| `gauss_seg2` | `GaussianSpring(k, 0.06)` | 2/3 | **shares `gauss_seg1`'s goal** |

`tau = sum_i J_i^T F_i`. Both z values are held — the button's height, and the
midpoint between the tips of sections 1 and 2 (`rest_pose(0.5)`, which also puts
both attachment points equidistant from the centre). Only x/y is placed by hand,
and only for the tip: the obstacle is already where it belongs. Aim the tip a
little past the surface, or the steady-state force decays to zero on arrival.

`K_REACH = 30` N/m, `C_DAMP = 1.0` N·s/m, `TAU_LIMIT = 0.25` N·m,
`PRETENSION_WEIGHTS = 0.010` N·m/rad. Springs and dampers are live sliders in
the GUI.

## Run names

| `--run` | strength | | `--run` | strength |
|---|---|---|---|---|
| `no_gauss` | 0 | | `mid_2` | 2000 |
| `mid_1` | 1000 | | `strong` | 5000 |

The name sets the strength: `--run` applies `GAUSS_CONDITIONS` to both
Gaussians, in the torque node and on the GUI sliders. It is **ramped in from 0
over `GAUSS_RAMP_S` = 5 s** rather than applied at once — the obstacle centre is
already placed, so a field switched on at full strength would shove the arm off
it in a single tick. Touching **Apply Parameters** cancels the ramp and hands
control to the sliders; `k_params` records what was actually applied.

Any other name is a scratch run at the config's own strength. For `_person`,
record `strong` first.

⚠ At `strong = 5000` the Gaussian torque is roughly **1.75 N·m, about 7×
`TAU_LIMIT`** — the controller is hard-clipped and the arm's behaviour is set by
the limiter rather than by the virtual model. If lower strengths did nothing,
`SIGMA` is the cheaper knob: repulsion is `k · exp(−d²/2σ²) · d`, and with the
obstacle 1.5σ away the field is already down to 34% before `k` multiplies.

## Running

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
python3 ../VMC/GUI_MultiVMC_Torque.py --config _button --run mid_1
```

**Terminal 4** — GUI

```bash
python3 ../VMC/GUI_MultiVMC.py --config _button --run mid_1
```

Terminals 3 and 4 must name the same variant and the same run. Terminal 3 logs
what it loaded — check the config, the run and the gravity line before moving
anything:

```
[INFO] config: GUI_MultiVMC_Config_button
[INFO] run: mid_1 -- saving to outputs/button_pressing/mid_1.{png,npz}, overwriting any previous mid_1
[INFO] gravity compensation: gravity pulls along -y in the robot frame
```

The GUI publishes from the start, so the arm takes up the 5.5 cm to its goal
as soon as the torque node is up (≈1.65 N at `K_REACH = 30`), with the run's
Gaussian strength already applied. Drag `lin_tip` to the button, then leave it alone — the longest
stretch you don't touch is the measurement window. **Ctrl+C terminal 3 once**
to write the run. Untick **Publisher Active** to move goals without the arm
following.

For the other variant, swap `_button` for `_person` in terminals 3 and 4, and
skip terminal 2.

## Gravity

The arm is mounted **on its side**, so both configs set `GRAVITY_DIR = "-y"`.
Override for one run with `--gravity-dir -z`. Wrong value = compensation pushing
along an axis gravity no longer acts on.

## Data

`--run NAME` writes `outputs/<folder>/NAME.{npz,png}` on exit, **overwriting**
that run. Without it, output is a timestamped `experiments/z_<stamp>` pair for
`collect_run.py` to move. Needs `COLLECTED_DATA = False`; set it `True` once the
reported runs are recorded and nothing can overwrite them.

Saving happens **only on exit**, so Ctrl+C once and wait — a second Ctrl+C
truncates the write, and `kill -9` loses everything. The `.npz` goes first, so
an interrupt costs the figure rather than the run. Launch T3 from this folder;
paths are relative to it.

| field | shape | |
|---|---|---|
| `t` | (n,) | s, from the first reference message |
| `goal` | (n, 3, 3) | reference per component [m] — where the goals were |
| `comp_pos` | (n, 3, 3) | actual FK position per component [m] |
| `q_motor` / `q_motor_dot` | (n, 9) | [rad], [rad/s] |
| `tau` / `tau_grav` | (n, 9) | commanded torque and its gravity share [N·m] |
| `force` | (n,) | load-cell shear, raw **grams** — ×`9.80665e-3` for N |
| `f_virtual` | (n, 3) | task-space virtual force [N] |
| `k_params` | (n, 3) | spring parameter per component over time |
| `comp_names`, `comp_s_params`, `control_freq`, `tau_limit`, `gravity_dir` | | metadata |

## Files

| File | |
|---|---|
| `GUI_MultiVMC_Config_button.py` / `_person.py` | the elements, their values, the conditions |
| `plot_button_and_person.ipynb` | analysis — discovers `outputs/`, takes positions and the steady window from the data |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` |

The scripted state machine that used to drive this folder is in the git history.
