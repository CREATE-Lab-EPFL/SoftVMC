# ButtonPressing

A linear spring reaches the tip for a button; two repulsive Gaussians push the
arm's *body* off an obstacle. `tau = sum_i J_i^T F_i`. Turning the Gaussians up
buys clearance and costs reach — that trade-off is the experiment.

Three variants, two protocols:

| | `_button` | `_person` | `_card_castle` |
|---|---|---|---|
| obstacle | a fixture at x = −0.15 | a person's arm, held still at x = −0.25 | a house of cards at x = −0.25 |
| driven by | `run_sweep.py`, unattended, no GUI | the GUI, by hand | the GUI, by hand |
| Gaussian strength | swept 0 / 200 / 600 / 1000 | 2 cases: 0 and 1000 | 2 cases: 0 and 1000 |
| goals | scripted, identical every run | placed by hand | placed by hand |
| `TAU_LIMIT` | 0.25 N·m | **0.35 N·m** | **0.35 N·m** |
| load cell | yes | no | no |
| files | `outputs/button_pressing/`, 4 × 5 repeats | `outputs/person_avoiding/`, 2 | `outputs/card_castle/`, 2 |
| status | **recorded, locked** (`COLLECTED_DATA = True`) | to record | to record |

`_person` and `_card_castle` are the same experiment with a different obstacle —
every value in their configs matches, so they are directly comparable, and 1000
is the button's `strong` as well. Keep them in step.

---

# What to run

Every terminal starts from:

```bash
conda activate finger
cd ButtonPressing
```

## `_button` — the scripted sweep

Already recorded. To record again, first set `COLLECTED_DATA = False` in
`GUI_MultiVMC_Config_button.py` — it is `True` so the 20 runs cannot be
overwritten. **Do not run `start_arm.sh`**: the sweep owns `dynamixel_node`.

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

Then walk away: ≈101 s per recording, **33.9 min** for the full 4 × 5 (6.9 min
at `--repeats 1`). Per recording it drives the tip to `--target-x/y` at 2 cm/s,
holds a 20 s measurement window, writes
`outputs/button_pressing/<level>_r<N>.{npz,png}`, kills the driver, homes at
~18.3 rpm **verified tick for tick**, and goes again. Any failure aborts:
motors left holding, driver not restarted, runs already recorded named.
`--dry-run` prints the exact plan and timings.

## `_person` and `_card_castle` — the pair, from the GUI

| case | k [N/m] | |
|---|---|---|
| `no_gauss` | 0 | no field — the body goes wherever reaching takes it |
| `gauss` | 1000 | the field on |

Same two cases for both `_person` and `_card_castle`.

Two runs, nothing in between: the comparison is the point, not a curve. Three
terminals, **run the whole thing twice**, once per case. The tip should reach
the button both times; what changes is how far the *body* stays off the
obstacle.

Both variants take the same commands — swap `_person` for `_card_castle` and
the runs land in `outputs/card_castle/` instead. **With the card castle, record
`gauss` first**: `no_gauss` is expected to knock the castle over, so rebuild it
between the two rather than the other way round.

**Terminal 1** — homes the arm, then the driver in torque mode

```bash
bash ../start_arm.sh
```

**Terminal 2** — the controller and the log. Pick the case; `--run` is
required and only these two names are accepted.

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _person --run gauss
```

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _person --run no_gauss
```

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _card_castle --run gauss
```

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _card_castle --run no_gauss
```

**Terminal 3** — the GUI. **`--run` must be the same case as terminal 2**: the
GUI applies all three stiffnesses at once, so one told the wrong case would
apply the wrong field the first time you press *Apply Parameters*. It refuses
to start without it.

```bash
python3 ../VMC/GUI_MultiVMC.py --config _person --run gauss
```

```bash
python3 ../VMC/GUI_MultiVMC.py --config _person --run no_gauss
```

```bash
python3 ../VMC/GUI_MultiVMC.py --config _card_castle --run gauss
```

```bash
python3 ../VMC/GUI_MultiVMC.py --config _card_castle --run no_gauss
```

### Then, in order

Terminal 2 already drives the arm before the GUI is up. The tip goal starts *at*
the resting tip, so the linear spring pulls nothing until you drag it — only the
Gaussians moving, ramping 0 → k over 5 s. **Let it settle.**

1. **Place the obstacle, with nobody in reach.** Pick `gauss_seg1` under
   *Active VC*, click where the obstacle will be — `(-0.25, 0)` is where it
   starts. Both Gaussians jump there together. Check the body has pushed off it.
2. **Put the obstacle there** — the person takes position and holds it for the
   whole run, or the castle goes up. The field is already at strength; it is
   not switched on around them.
3. **Pick `lin_tip`, drag** its goal onto the button. Drag, don't click — a
   click jumps the goal, a drag is capped by *Drag speed* (cell ②) and the arm
   follows at that rate. Slower is better with someone standing there.
4. **Ctrl+C once in terminal 2** when the tip is on the button. That is the
   mark, and the analysis takes its measurement window from it. Hold still a
   few seconds after.
5. **Home & Quit** (cell ②) → confirm. It ends the whole session in one go:
   terminal 2 saves and exits → `dynamixel_node` stops (closing terminal 1) →
   the arm homes at ~9.2 rpm → terminal 3 closes.
6. Repeat from `start_arm.sh` for the other case.

⚠ Homing hands the arm back in **current mode**, so it goes limp at
`HOME_POSITION`. Check nothing fragile is under where it comes to rest.

The run is saved **first**, before anything is stopped — it is the only step
that cannot be redone. Once a run is ending, **Ctrl+C is ignored**: pressing it
again used to land inside the save and cost the run. Closing the window with
its X saves too, but leaves the arm and driver alone.

If a motor misses home, or lands a whole turn out, the GUI says so and stays
open — motors keep holding in position mode and are **not** handed back to
current mode, because in current mode the arm goes limp and whatever pose it is
in becomes the model's zero next start-up. A delta that is a multiple of 4096
is a lost turn count, not a failed move: power-cycle, re-place by hand, and
re-capture with `python3 ../VMC/MultiGoHome.py --read`.

---

# The model

| Component | Element | s | Goal |
|---|---|---|---|
| `lin_tip` | `LinearSpring(k)` | 1.0 | the button, at `TIP_GOAL_Z` |
| `gauss_seg1` | `GaussianSpring(k, 0.06)` | 1/3 | the obstacle, at `GAUSS_GOAL_Z` |
| `gauss_seg2` | `GaussianSpring(k, 0.06)` | 2/3 | **shares `gauss_seg1`'s goal** |

Shared by all three: `SIGMA = 0.06` m, `PRETENSION_WEIGHTS = 0.010` N·m/rad,
`GAUSS_GOAL_Z = 0.2175` (midpoint between the tips of sections 1 and 2).

Four things differ between the sweep and the GUI pair:

| | `_button` | `_person`, `_card_castle` |
|---|---|---|
| `K_REACH` | 30 N/m | 35 N/m |
| `C_DAMP` | 1.0 N·s/m | 3.0 N·s/m |
| `TAU_LIMIT` | 0.25 N·m | 0.35 N·m |
| `TIP_GOAL_Z` | 0.38 | **0.435** — the straight arm's own tip |

Every goal z is locked in all three variants, so a goal only ever moves in x
and y. For the GUI pair that plane is the tip's own: the tip goal starts exactly
where the resting tip is, so the linear spring pulls **nothing** until it is
dragged, and dragging it steers the tip across that one plane instead of also
pulling the tip in along the arm's axis. As the body bends the tip falls short
of z = 0.435, and the spring pulls it back out — that residual is the price of
keeping the goal in a fixed plane.

The GUI pair pulls harder and settles harder, against a heavier field: 1000 N/m
acting on a body that has to come to rest next to a person or a stack of cards,
where ringing is the thing to avoid. The stiffer tip spring keeps the tip
arriving at the button while the body is being pushed off, and the higher torque
limit leaves room for both. Damping costs nothing in steady state — a damper
only pushes back while the arm is moving.

**Strengths are ramped 0 → k over `RAMP_S` = 5 s** at start-up, before anything
publishes, so it lands at `t < 0`. Switched on at once, a placed field shoves
the arm off the obstacle in one tick. `k_params` records what was applied, tick
by tick. A `--run` the config has no level for is refused in a second, before
anything is built.

⚠ **`GAUSS_GOAL_XY` must sit at more negative x than the body ever reaches**, or
the repulsion points *at* the button. At an early `-0.05` the body had already
curved past the centre, the field **assisted** the reach, and shear rose with
strength while reach error fell — monotonic, but not avoidance. The button uses
`-0.15`, the person `-0.25`; both oppose.

⚠ **The tip spring alone hits `TAU_LIMIT` at x = −0.597** (button), before the
Gaussian adds anything — past that the limiter, not the virtual model, drives
the arm. `tau_raw` records what was asked; the run figure shades where it bit.
The limiter warns once every 2 s (`CLIP_WARN_S`), not once per tick.

## `_button` only

`--target-x/y` defaults to `(-0.50, 0)` — beyond the arm's 43.5 cm reach on
purpose, so the tip stops at full extension and the target sets how *hard* it
pulls there: 8.97 N, asking 0.189 N·m. Each level is recorded `SWEEP_REPEATS`
= 5 times as `<level>_r<N>`; the level is still resolvable from the name, so
the analysis groups them automatically. This is the only folder with error
bars.

## `_person` and `_card_castle` only

Strengths are **field strengths, not forces**: a Gaussian peaks at
`k·exp(−½)·σ = k·0.0364` N when its attachment point is σ from the centre, so
`gauss` peaks at 36 N and falls away fast either side of it: 16 N at 2σ
(12 cm), 3.3 N at 2.8σ (17 cm) — a tenth of the peak, and by then weaker than
the reach. **Placement matters more than strength.** What the motors get is
clipped at `TAU_LIMIT`, which is 0.35 here against the button's 0.25, so the
limiter has less say in the pose.

With the obstacle at −0.25 it is **`gauss_seg2` (s = 2/3) that does the work**:
the body curves to about −17 cm, putting it ~8 cm = 1.4σ from the centre, near
the peak. `gauss_seg1` sits around −6 cm, 19 cm away, and contributes ~1 N. If
the arm barely reacts, the centre is too far from where the body actually
passes — move it, or raise `SIGMA`, before reaching for a bigger `k`.

The GUI can change: both goals' x/y, the tip stiffness, both Gaussian strengths
(cell ①, `Apply Parameters`), and a **`Gaussians active` tickbox**. Every z is
locked. `XY_RANGE = ±2.00` m is both the plot extent and the clamp on a dragged
goal — far past the arm's 43.5 cm reach on purpose, since a goal outside it sets
how *hard* the tip pulls rather than where it ends up.

Two costs come with that. **Precision:** the plot spans 4 m across the same
pixels, so drop *Drag speed* (`DRAG_SPEED_MAX = 100` cm/s) before placing the
obstacle, where a couple of centimetres matter against `SIGMA = 6` cm.
**Saturation:** past roughly |x| = 0.9 m the tip spring alone asks more than
`TAU_LIMIT`, and the limiter rather than the virtual model is driving the arm —
`tau_raw` records what was asked, and the run figure shades where it bit.

The tickbox owns *whether* the field is on, the entries own *how strong* when it
is — so it returns to whatever was last applied, and a strength typed while it
is off is remembered for the next tick. Either way the strength is **walked
over `RAMP_S` in both directions**. Untouched, it publishes nothing, so it
cannot cancel the start-up ramp. `k_params` records every walk, and the
analysis segments on it.

---

# Gravity and axes

The arm is mounted **on its side** — both configs set `GRAVITY_DIR = "-y"`
(override per run with `--gravity-dir`):

| axis | physically | set by |
|---|---|---|
| `x` | horizontal, across | the GUI's x (±2.00, `_button` ±0.70), `GAUSS_GOAL_XY[0]`, `--target-x` |
| `y` | **vertical — up is +y** | the GUI's y (±2.00, `_button` ±0.70), `GAUSS_GOAL_XY[1]`, `--target-y` |
| `z` | out along the arm's axis | `TIP_GOAL_Z`, `GAUSS_GOAL_Z` (locked — goals move in x, y only) |

⚠ No single projection is the geometry. Since y is vertical an **x–z plot is a
plan view from above**, and the arm has been recorded **lifted 16 cm** by the
+y component of the Gaussian — invisible from above. Read clearance and reach
error in 3-D: an x–z view of one run shows the arm grazing the obstacle at
0.8 cm where the true distance is 12.2 cm.

# Data

`--run NAME` writes `outputs/<folder>/NAME.{npz,png}` on exit, **overwriting**
that run; needs `COLLECTED_DATA = False`. Checkpointed to `NAME.autosave.npz`
every 30 s, so a crash costs seconds:

- a leftover **`.autosave.npz`** means the run did not exit cleanly — rename it
  over `NAME.npz` to recover it;
- a **`.npz.part.npz`** is an interrupted write: truncated, delete it. Writing
  through that temporary is why an interrupted save cannot destroy a good file.

| field | shape | |
|---|---|---|
| `t` / `t_wall` | (n,) | s from the first reference (`t < 0` before it); unix time |
| `goal` | (n, 3, 3) | reference per element [m] — for `_person`, everything the hand did |
| `comp_pos` / `comp_vel` | (n, 3, 3) | FK position [m] and task-space velocity [m/s] |
| `f_comp` | (n, 3, 3) | **each element's force separately** [N]; `f_virtual` (n, 3) is their sum |
| `tau_comp` | (n, 3, 9) | each element's share of the torque [N·m] |
| `tau_vmc` / `tau_grav` / `tau_pretension` | (n, 9) | the three terms, before the limiter |
| `tau_raw` / `tau` | (n, 9) | asked vs sent — clipping is `abs(tau_raw) > tau_limit` |
| `q_motor` / `q_motor_dot` | (n, 9) | [rad], [rad/s] |
| `k_params` / `c_params` | (n, 3) | spring and damper parameter per element over time — start-up ramp and every toggle included |
| `force` / `force_age` | (n,) | load-cell shear, raw **grams** (×`9.80665e-3` for N), and its staleness [s] |
| `marks_t` / `marks_label` / `marks_wall` | (m,) | where Ctrl+C was pressed, on `t`'s clock |
| `comp_spec` / `config_source` | (1,) | every element's type and values as JSON; the whole config file |
| metadata | | `comp_names`, `comp_s_params`, `spring_param_names`, `damper_param_names`, `control_freq`, `tau_limit`, `gravity_dir`, `config_name`, `run_name`, `ramp_components`, `ramp_strength`, `ramp_s`, `pretension_weights`, `pretension_enabled`, `t_reference_ok` |

Everything positional is taken from the run, never the config — for `_person`
the goals were dragged live, so where the button and the obstacle actually were
is in `goal` and nowhere else. The same for strength, read from `k_params` over
the window.

# Files

| File | |
|---|---|
| `GUI_MultiVMC_Config_button.py` | `_button`: the elements, the four swept levels |
| `GUI_MultiVMC_Config_person.py` | `_person`: the two cases, `TAU_LIMIT = 0.35`, what the GUI may touch |
| `GUI_MultiVMC_Config_card_castle.py` | `_card_castle`: `_person`'s values, a different obstacle |
| `run_sweep.py` | `_button` only: every level × `SWEEP_REPEATS`, homed and verified between |
| `plot_experiments.ipynb` | analysis for all three — discovers `outputs/`, positions from the data, window from the mark |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` |
| `media/<variant>/` | photos of each set-up, alongside `outputs/<variant>/` |
