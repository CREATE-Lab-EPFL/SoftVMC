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
over `GAUSS_RAMP_S` = 5 s** (which happens as soon as terminal 3 is up, so it
lands at `t < 0` in the log, before the GUI connects) rather than applied at once — the obstacle centre is
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
Gaussian strength already applied. Drag `lin_tip` to the button, then leave it
alone. Untick **Publisher Active** to move goals without the arm following.

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

## Gravity

The arm is mounted **on its side**, so both configs set `GRAVITY_DIR = "-y"`.
Override for one run with `--gravity-dir -z`. Wrong value = compensation pushing
along an axis gravity no longer acts on.

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
| `comp_names`, `comp_s_params`, `control_freq`, `tau_limit`, `gravity_dir`, `config_name`, `run_name`, `condition_strength`, `ramp_s`, `pretension_weights` | | metadata |

The projections are not the geometry: the tip goal is dragged in x-y at a locked
z and gravity pulls along -y, so a recorded pose leaves the y = 0 plane by up to
16 cm. Read clearance and reach error in 3-D — an x-z view of one run shows the
arm grazing the obstacle at 0.8 cm where the real distance is 12.2 cm.

## Files

| File | |
|---|---|
| `GUI_MultiVMC_Config_button.py` / `_person.py` | the elements, their values, the conditions |
| `plot_button_and_person.ipynb` | analysis — discovers `outputs/`, takes positions from the data and the measurement window from the Ctrl+C mark |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` |

The scripted state machine that used to drive this folder is in the git history.
