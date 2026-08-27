# ButtonPressing

A linear spring reaching for a button, plus two repulsive Gaussians pushing the
arm's body off an obstacle. Sweeping the Gaussian strength trades reach against
clearance. Scripted end to end by `run_sweep.py` — no GUI, nothing placed by hand.

| | `_button` | `_person` |
|---|---|---|
| obstacle | a fixture | a person's arm |
| load cell | yes (`/force_shear`) | no |
| runs land in | `outputs/button_pressing/` | `outputs/person_avoiding/` |

`_person.py` is generated from `_button.py`: edit a value in one and copy it
across. `GAUSS_CONDITIONS` is the one field deliberately different.

## Elements

| Component | Element | s | Goal |
|---|---|---|---|
| `lin_tip` | `LinearSpring(30)` | 1.0 | starts `[0, 0, 0.435]`, walked to `--target-x/y` at `TIP_GOAL_Z` |
| `gauss_seg1` | `GaussianSpring(k, 0.06)` | 1/3 | `GAUSS_GOAL_XY` + `GAUSS_GOAL_Z` = `[-0.05, 0, 0.2175]` |
| `gauss_seg2` | `GaussianSpring(k, 0.06)` | 2/3 | **shares `gauss_seg1`'s goal** |

`tau = sum_i J_i^T F_i`. Both z are locked; z is the reach out along the arm's
axis, **not height** (see [Gravity and axes](#gravity-and-axes)).

`K_REACH = 30` N/m, `C_DAMP = 1.0` N·s/m, `SIGMA = 0.06` m, `TAU_LIMIT = 0.25`
N·m, `PRETENSION_WEIGHTS = 0.010` N·m/rad. All from the config — the sweep runs
no GUI, so nothing is adjustable mid-run.

**The target is `--target-x/y`, default `(-0.40, 0)`** — and it is deliberately
**beyond the arm's reach**: 55.2 cm from the base against a 43.5 cm straight
length. The tip stops at full extension either way, so the target sets how hard
it pulls there (5.94 N, against 2.94 N at `-0.30`), not where it ends up.

**Set `GAUSS_GOAL_XY` to where the obstacle actually is.** The `(-0.05, 0.0)`
default sits 5 cm off the arm's rest axis with zero y offset, which leaves the
direction the arm escapes in barely determined.

## Conditions

| `--run` | strength | `_button` | `_person` |
|---|---|---|---|
| `no_gauss` | 0 | ✓ | ✓ |
| `weak` | 500 | ✓ | — |
| `medium` | 1000 | ✓ | ✓ |
| `strong` | 2000 | ✓ | ✓ |

`medium` and `strong` are shared, so the fixture and the person are comparable at
an identical field. `_person` drops `weak`: every run costs someone holding an
arm still in front of a moving robot.

The strength is **ramped in from 0 over `GAUSS_RAMP_S` = 5 s**, starting as soon
as the torque node is up — before the sweep publishes, so it lands at `t < 0`.
`k_params` records what was actually applied. A name the config has no condition
for is refused in a second, before anything is built:

```
[ERROR] --run 'mid_1' is not a condition of GUI_MultiVMC_Config_button.
        this config knows: no_gauss (0), weak (500), medium (1000), strong (2000)
```

**Where the crossover sits.** At the pose the arm actually reaches (measured from
`no_gauss`), the Gaussian is worth 0.248 N·m per 1000 N/m of strength, against a
tip torque of 0.174 N·m at `--target-x -0.40`. So the field overtakes the reach
at **≈ 700 N/m**, which puts the crossover between `weak` and `medium`: one
condition where the tip wins, two where the field does.

This is why the target matters. At `-0.30` the tip torque is only 0.087 N·m and
the crossover falls to ≈ 350 — below every configured strength, so all three
conditions sit on the same side and the sweep shows no transition.

⚠ These strengths still sweep how far the arm is pushed off, not how hard it
presses. Check the shear is above zero at `strong`; if it is not, the sweep shows
on/off rather than modulation, and the fix is a target further out (more pull at
full extension) or a smaller `SIGMA`.

## Running

Two terminals, each starting from:

```bash
conda activate finger
cd ButtonPressing
```

**Terminal 1** — load cell (`_button` only, skip for `_person`)

```bash
python3 ../LoadCell/arduino_force.py
```

**Terminal 2** — the sweep. One of these, not all.

```bash
python3 run_sweep.py
```

```bash
python3 run_sweep.py --config _person
```

```bash
python3 run_sweep.py --only weak strong
```

```bash
python3 run_sweep.py --dry-run
```

**Do not run `start_arm.sh`** — the sweep owns `dynamixel_node`, which cannot
change operating mode while running and holds `/dev/ttyUSB0` exclusively.

| step | set by | |
|---|---|---|
| | | driver up (torque mode), torque node up with `--run <condition>` |
| | | publish `/VC_ref_pos` from the script; tip goal at the rest tip `[0, 0, 0.435]`, so the linear spring pulls **nothing** |
| settle | `--settle-gauss` 15 s | the Gaussians ramp in and push the arm off, alone |
| move | `--ramp-speed` 2 cm/s | tip goal walks to `--target-x/y` at `TIP_GOAL_Z` — 40.4 cm, 20 s |
| settle | `--settle-arrive` 15 s | the arm follows and stops |
| window | `--window` 20 s | between the two Ctrl+Cs — **the measurement window** |
| | | writes `outputs/<folder>/<condition>.{npz,png}`, kills the driver |
| home | `--home-velocity` 10 | extended position mode, ~2.3 rpm, back to `HOME_POSITION`, **verified tick for tick** |
| settle | `--settle-home` 5 s | hand back in current mode, next condition |

≈123 s per condition: **8.2 min** for the button's four, **6.1 min** for the
person's three. Homing is the longest leg (~30 s) — the arm ends a run about 1.15
turns wound. Every condition therefore starts from the same pose and drives to
the same target, which is the point; dragged by hand, `weak` and `medium` landed
7.4 cm apart in x and 5.4 cm apart in y.

**Any failure aborts the sweep** — homing a turn out, a motor not arriving, the
torque node dying. The motors are left holding, the driver is not restarted, and
the conditions already recorded are named. Nothing is recorded from an unverified
zero.

### The mark

Ctrl+C to the torque node does two things in order (`MARK_PROMPTS`), and the
sweep sends both, `--window` apart: the **first marks** *"Target reached?"*, the
**second ends and writes** the run. The mark is what tells the analysis where the
measurement window starts; without one it falls back to the longest stretch
nothing was touched.

`t = 0` is the first `/VC_ref_pos`. Before it the torque node is already driving
at `DEFAULT_START`, so the take-up and the start of the ramp sit at `t < 0`
(kept: last `PRE_REFERENCE_S` = 60 s). A Ctrl+C during the sympy build aborts
cleanly rather than dumping a traceback.

## Gravity and axes

The arm is mounted **on its side**: both configs set `GRAVITY_DIR = "-y"`
(override per run with `--gravity-dir`). That fixes what the axes mean, and it is
easy to get backwards:

| axis | physically | set by |
|---|---|---|
| `x` | horizontal, across | `--target-x`, `GAUSS_GOAL_XY[0]` |
| `y` | **vertical — up is +y** | `--target-y`, `GAUSS_GOAL_XY[1]` |
| `z` | out along the arm's axis | `TIP_GOAL_Z`, `GAUSS_GOAL_Z` (locked) |

⚠ No single projection is the geometry. Since y is vertical, an **x–z plot is a
plan view from above**, and the arm has been recorded **lifted 16 cm** by the +y
component of the Gaussian — invisible from above. Read clearance and reach error
in 3-D: an x–z view of one run shows the arm grazing the obstacle at 0.8 cm where
the true distance is 12.2 cm.

## Data

`--run NAME` writes `outputs/<folder>/NAME.{npz,png}` on exit, **overwriting**
that run; needs `COLLECTED_DATA = False`. The log is checkpointed to
`NAME.autosave.npz` every `AUTOSAVE_S` = 30 s, so a crash costs seconds rather
than the run:

- a leftover **`.autosave.npz`** means the run did not exit cleanly — rename it
  over `NAME.npz` to recover it;
- a **`.npz.part.npz`** is an interrupted write: truncated, unreadable, delete
  it. Writing through that temporary is why an interrupted save can no longer
  destroy a good `NAME.npz`.

| field | shape | |
|---|---|---|
| `t` / `t_wall` | (n,) | s from the first reference (`t < 0` before it); unix time |
| `goal` | (n, 3, 3) | reference per element [m] |
| `comp_pos` / `comp_vel` | (n, 3, 3) | FK position [m] and task-space velocity [m/s] |
| `f_comp` | (n, 3, 3) | **each element's force separately** [N]; `f_virtual` (n, 3) is their sum |
| `tau_comp` | (n, 3, 9) | each element's share of the torque [N·m] |
| `tau_vmc` / `tau_grav` / `tau_pretension` | (n, 9) | the three terms, before the limiter |
| `tau_raw` / `tau` | (n, 9) | asked for vs sent — clipping is `abs(tau_raw) > tau_limit` |
| `q_motor` / `q_motor_dot` | (n, 9) | [rad], [rad/s] |
| `k_params` / `c_params` | (n, 3) | spring and damper parameter per element over time |
| `force` / `force_age` | (n,) | load-cell shear, raw **grams** (×`9.80665e-3` for N), and its staleness [s] |
| `marks_t` / `marks_label` / `marks_wall` | (m,) | where Ctrl+C was pressed, on `t`'s clock |
| `comp_spec` / `config_source` | (1,) | every element's type and values as JSON; the whole config file |
| metadata | | `comp_names`, `comp_s_params`, `spring_param_names`, `damper_param_names`, `control_freq`, `tau_limit`, `gravity_dir`, `config_name`, `run_name`, `condition_targets`, `condition_strength`, `ramp_s`, `pretension_weights`, `pretension_enabled`, `t_reference_ok` |

## Files

| File | |
|---|---|
| `GUI_MultiVMC_Config_button.py` / `_person.py` | the elements, their values, the conditions |
| `run_sweep.py` | the unattended sweep: every condition, same start pose, same target, homed and verified between |
| `plot_button_and_person.ipynb` | analysis — discovers `outputs/`, takes positions from the data and the window from the mark |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` |
