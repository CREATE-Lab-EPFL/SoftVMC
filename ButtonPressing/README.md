# ButtonPressing

A linear spring reaching for a button, plus two repulsive Gaussians pushing the
arm's body off an obstacle. Sweeping the Gaussian strength trades reach against
clearance. Driven live from the GUI.

| | `_button` | `_person` |
|---|---|---|
| obstacle | a fixture | a person's arm |
| `TAU_LIMIT` | 0.25 N·m | 0.15 N·m |
| load cell | yes (`/force_shear`) | no |
| runs land in | `outputs/button_pressing/` | `outputs/person_avoiding/` |

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

`K_REACH = 30` N/m, `C_DAMP = 1.0` N·s/m. Both are live sliders in the GUI.

## Run names

| `--run` | strength | | `--run` | strength |
|---|---|---|---|---|
| `no_gauss` | 0 N/m | | `mid_2` | 10 N/m |
| `mid_1` | 5 N/m | | `strong` | 20 N/m |

The name sets the strength: `--run` applies `GAUSS_CONDITIONS` to both
Gaussians, in the torque node and on the GUI sliders. The sliders stay live and
`k_params` records what was actually applied. Reach and repulsion balance near
20 N/m, so `strong` sits about there — but that moves with the obstacle, so read
it off the torque plot. Any other name is a scratch run at the config's own
strength. For `_person`, record `strong` first.

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
