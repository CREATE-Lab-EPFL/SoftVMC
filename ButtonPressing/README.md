# ButtonPressing

A linear spring reaching for a button, plus two repulsive Gaussians pushing the
arm's body off an obstacle. `tau = sum_i J_i^T F_i`.

Two experiments, and since the person one changed they no longer share a
protocol:

| | `_button` | `_person` |
|---|---|---|
| obstacle | a fixture | a person's arm |
| driven by | `run_sweep.py`, unattended, **no GUI** | the **GUI**, by hand |
| Gaussian strength | swept: 0 / 200 / 600 / 1000 | **fixed at 500**, every run |
| what varies | the strength; goals scripted, identical every run | where the two goals are put |
| load cell | yes (`/force_shear`) | no |
| runs land in | `outputs/button_pressing/` | `outputs/person_avoiding/` |
| what a run is | one condition | one whole session, start to finish |

`_button` is the quantitative one and is **recorded**: 4 conditions × 5 repeats,
`COLLECTED_DATA = True` so nothing can overwrite them. `_person` is qualitative
— the record is what was done at the GUI, not a point on a curve.

## Elements

| Component | Element | s | Goal |
|---|---|---|---|
| `lin_tip` | `LinearSpring(30)` | 1.0 | the button, at `TIP_GOAL_Z` |
| `gauss_seg1` | `GaussianSpring(k, 0.06)` | 1/3 | the obstacle, at `GAUSS_GOAL_Z` |
| `gauss_seg2` | `GaussianSpring(k, 0.06)` | 2/3 | **shares `gauss_seg1`'s goal** |

Common to both: `K_REACH = 30` N/m, `C_DAMP = 1.0` N·s/m, `SIGMA = 0.06` m,
`TAU_LIMIT = 0.25` N·m, `PRETENSION_WEIGHTS = 0.010` N·m/rad, `TIP_GOAL_Z =
0.38`, `GAUSS_GOAL_Z = 0.2175` (the midpoint between the tips of sections 1
and 2).

Both z are locked in both variants; z is the reach out along the arm's axis,
**not height** (see [Gravity and axes](#gravity-and-axes)).

The strength is **ramped in from 0 over `RAMP_S` = 5 s** (`RAMP_COMPONENTS`),
starting as soon as the torque node is up — before anything publishes, so it
lands at `t < 0`. Switched on at once instead, a placed field shoves the arm
off the obstacle within a single tick. `k_params` records what was actually
applied, tick by tick.

⚠ **`GAUSS_GOAL_XY` has to sit at more negative x than the body ever reaches**,
or the repulsion points *at* the button instead of away from it. At the old
`-0.05` it did exactly that: the body had already curved past the centre, the
field **assisted** the reach, and shear rose with strength while reach error
fell — monotonic, but not avoidance. `-0.15` is where it is opposed.

---

## `_button` — the scripted sweep

Every condition starts from the same arm pose and drives to the same target, so
the only thing that differs between them is the Gaussian strength. Dragged by
hand, `weak` and `medium` landed 7.4 cm apart in x and 5.4 cm apart in y.

| `--run` | k [N/m] |
|---|---|
| `no_gauss` | 0 |
| `weak` | 200 |
| `medium` | 600 |
| `strong` | 1000 |

**The target is `--target-x/y`, default `(-0.50, 0)`** — beyond the arm's 43.5
cm reach on purpose, so the tip stops at full extension and the target sets how
hard it pulls there rather than where it ends up: 8.97 N, asking 0.189 N·m.

⚠ **The tip spring alone reaches `TAU_LIMIT` at x = −0.597**, before the
Gaussian adds anything — past that the arm is driven by the limiter rather than
by the virtual model. `tau_raw` records what was asked; the run figure shades
where it bit.

Each condition is recorded 5 times (`SWEEP_REPEATS`, `--repeats N` overrides)
as `<condition>_r<N>.npz` — a separate recording each, the condition still
resolvable from the name, so the analysis groups them back automatically. Every
other experiment in this repo is n = 1, so this is the only folder with error
bars.

A name the config has no condition for is refused in a second, before anything
is built:

```
[ERROR] --run 'mid_1' is not a condition of GUI_MultiVMC_Config_button.
        this config knows: no_gauss (0), weak (200), medium (600), strong (1000)
```

### Running it

Two terminals, each starting from:

```bash
conda activate finger
cd ButtonPressing
```

**Terminal 1** — load cell

```bash
python3 ../LoadCell/arduino_force.py
```

**Terminal 2** — the sweep. One of these, not all.

```bash
python3 run_sweep.py
```

```bash
python3 run_sweep.py --only weak strong
```

```bash
python3 run_sweep.py --repeats 1
```

```bash
python3 run_sweep.py --dry-run
```

**Do not run `start_arm.sh`** — the sweep owns `dynamixel_node`, which cannot
change operating mode while running and holds `/dev/ttyUSB0` exclusively.

| step | set by | |
|---|---|---|
| | | driver up (torque mode), torque node up with `--run <condition>_r<N>` |
| | | publish `/VC_ref_pos` from the script; tip goal at the rest tip `[0, 0, 0.435]`, so the linear spring pulls **nothing** |
| settle | `--settle-gauss` 15 s | the Gaussians ramp in and push the arm off, alone |
| move | `--ramp-speed` 2 cm/s | tip goal walks to `--target-x/y` at `TIP_GOAL_Z`; `--dry-run` prints the distance and time |
| settle | `--settle-arrive` 15 s | the arm follows and stops |
| window | `--window` 20 s | between the two Ctrl+Cs — **the measurement window** |
| | | writes `outputs/button_pressing/<condition>.{npz,png}`, kills the driver |
| home | `--home-velocity` 80 | extended position mode, ~18.3 rpm, back to `HOME_POSITION`, **verified tick for tick** |
| settle | `--settle-home` 5 s | hand back in current mode, next condition |

≈101 s per recording at the default target: **33.9 min** for the full 4 × 5,
6.9 min at `--repeats 1` — `--dry-run` prints the exact figure, since the ramp
is a third of it and scales with the target.

**Any failure aborts the sweep** — homing a turn out, a motor not arriving, the
torque node dying. The motors are left holding, the driver is not restarted,
and the conditions already recorded are named. Nothing is recorded from an
unverified zero.

To record again, set `COLLECTED_DATA = False` in
`GUI_MultiVMC_Config_button.py`; it is `True` so that the 20 recorded runs
cannot be overwritten.

---

## `_person` — driven from the GUI

The strength is not the variable here. Both Gaussians are held at **500 N/m for
the whole run**, and the experiment is what a person does with two points:

1. **the tip goal** — `lin_tip`'s x and y, dragged onto the button;
2. **the obstacle** — one centre shared by both Gaussians, dragged onto the
   person's arm. `GOAL_LINK` moves both together; there is no way to separate
   them.

Everything else is held. All three z are locked, and `LOCK_PARAMS = True` makes
the stiffnesses read-only — the GUI shows them but cannot edit them and never
publishes `/VC_Param`, so a run cannot quietly become a different experiment
halfway through. One run is one continuous recording of the session.

### Running it

Three terminals, each starting from:

```bash
conda activate finger
cd ButtonPressing
```

**Terminal 1** — the arm: homes, then the driver in torque mode

```bash
bash ../start_arm.sh
```

**Terminal 2** — the controller and the log. `<name>` is yours to choose
(`reach_1`, `side_on`, …); it names `outputs/person_avoiding/<name>.{npz,png}`
and **overwrites** a previous run of that name.

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _person --run <name>
```

**Terminal 3** — the GUI

```bash
python3 ../VMC/GUI_MultiVMC.py --config _person
```

No load cell: `PLOT_FORCE = False` and nothing publishes `/force_shear`.

### What to do, in order

Terminal 2 is already driving the arm before the GUI is up: the tip goal starts
straight ahead at `TIP_GOAL_Z`, so the arm takes up 5.5 cm of slack, and the
Gaussians ramp 0 → 500 over 5 s around `GAUSS_GOAL_XY = (-0.15, 0)`. Let that
settle before touching anything.

1. **Place the obstacle first, with nobody in reach.** Pick `gauss_seg1` under
   *Active VC* and click where the arm will be — both Gaussians jump there
   together, and the body pushes off it. Confirm the arm has moved off before
   the person steps in.
2. **The person takes position** at the centre you just placed. The field is
   already at full strength; it is not switched on around them.
3. **Pick `lin_tip`** and drag its goal onto the button. Drag, do not click:
   a click jumps the goal, a drag is speed-capped by *Drag speed* (default 5
   cm/s, top of cell ②) and the arm follows at that rate. Slower is better with
   someone standing there.
4. **Ctrl+C once in terminal 2** when the tip is on the button. That is the
   mark — it is what tells the analysis where the measurement window starts.
   Hold still for a few seconds after it.
5. Move things and mark again as you like; each Ctrl+C beyond the first ends
   the run, so mark once per run unless `MARK_PROMPTS` says otherwise.
6. **Home & Quit** (top of cell ②) → confirm. The arm goes back to
   `HOME_POSITION` in extended position control at ~9.2 rpm — half the speed
   the button sweep homes at, because that one runs with nobody near the arm —
   and the GUI closes when it arrives.
7. **Ctrl+C in terminal 2** to write the run.

⚠ **Save before homing if you want the homing in the log.** *Home & Quit* stops
`dynamixel_node` to take the serial bus — that is the only way to reach the
servos in position control — so the session ends there and nothing restarts the
driver. The torque node is left alone and still holds the log, but with the
driver gone it is recording an arm that is no longer listening to it.

If a motor does not arrive, or arrives a whole turn out, the GUI says so and
stays open: the motors are left holding in position mode and are **not** handed
back to current mode, because in current mode the arm goes limp and whatever
pose it is in becomes the model's zero the moment the driver next starts. A
delta that is a multiple of 4096 is a lost turn count, not a motor that failed
to move — power-cycle, re-place the arm by hand, and re-capture with
`python3 ../VMC/MultiGoHome.py --read`.

---

## The mark

Ctrl+C in the torque node's terminal consumes `MARK_PROMPTS` one press at a
time, and the press after the last one ends the run and writes it. With
`("Target reached?",)` that is: **first press marks, second press ends**.

The mark is what tells the analysis where the measurement window starts;
without one it falls back to the longest stretch nothing was touched, which
cannot tell "on the button" from "let go of the mouse".

`t = 0` is the first `/VC_ref_pos`. Before it the torque node is already driving
at `DEFAULT_START`, so the take-up and the ramp sit at `t < 0` (kept: last
`PRE_REFERENCE_S` = 60 s). A Ctrl+C during the sympy build aborts cleanly
rather than dumping a traceback.

## Gravity and axes

The arm is mounted **on its side**: both configs set `GRAVITY_DIR = "-y"`
(override per run with `--gravity-dir`). That fixes what the axes mean, and it
is easy to get backwards:

| axis | physically | set by |
|---|---|---|
| `x` | horizontal, across | the GUI's x, `GAUSS_GOAL_XY[0]`, `--target-x` |
| `y` | **vertical — up is +y** | the GUI's y, `GAUSS_GOAL_XY[1]`, `--target-y` |
| `z` | out along the arm's axis | `TIP_GOAL_Z`, `GAUSS_GOAL_Z` (locked) |

⚠ No single projection is the geometry. Since y is vertical, an **x–z plot is a
plan view from above**, and the arm has been recorded **lifted 16 cm** by the
+y component of the Gaussian — invisible from above. Read clearance and reach
error in 3-D: an x–z view of one run shows the arm grazing the obstacle at
0.8 cm where the true distance is 12.2 cm.

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
| `goal` | (n, 3, 3) | reference per element [m] — for `_person`, everything the hand did |
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
| metadata | | `comp_names`, `comp_s_params`, `spring_param_names`, `damper_param_names`, `control_freq`, `tau_limit`, `gravity_dir`, `config_name`, `run_name`, `ramp_components`, `ramp_strength`, `ramp_s`, `pretension_weights`, `pretension_enabled`, `t_reference_ok` |

Everything positional is taken from the run itself, never from the config — for
`_person` the goals were dragged live, so where the button and the obstacle
actually were is recorded in `goal` and nowhere else.

## Files

| File | |
|---|---|
| `GUI_MultiVMC_Config_button.py` | `_button`: the elements, their values, the four conditions |
| `GUI_MultiVMC_Config_person.py` | `_person`: the same values at a fixed strength, plus the GUI's locks |
| `run_sweep.py` | `_button` only: every condition × `SWEEP_REPEATS`, same start pose, same target, homed and verified between |
| `plot_button_and_person.ipynb` | analysis — discovers `outputs/`, takes positions from the data and the window from the mark |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` |
