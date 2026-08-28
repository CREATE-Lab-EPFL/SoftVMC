# ButtonPressing

A linear spring reaches the tip for a button; two repulsive Gaussians push the
arm's *body* off an obstacle. `tau = sum_i J_i^T F_i`. Turning the Gaussians up
buys clearance and costs reach — that trade-off is the experiment.

Three variants, two protocols:

| | `_button` | `_person` | `_card_castle` |
|---|---|---|---|
| obstacle | a fixture at x = −0.15 | a person's arm, held still at x = −0.25 | a house of cards at x = −0.25 |
| driven by | `run_sweep.py`, unattended, no GUI | the GUI, by hand | the GUI, by hand |
| Gaussian strength | swept 0 / 200 / 600 / 1000 | 2 cases: 0 and 500 | 2 cases: 0 and 500 |
| goals | scripted, identical every run | placed by hand | placed by hand |
| `TAU_LIMIT` | 0.25 N·m | **0.35 N·m** | **0.35 N·m** |
| load cell | yes | no | no |
| files | `outputs/button_pressing/`, 4 × 5 repeats | `outputs/person_avoiding/`, 2 | `outputs/card_castle/`, 2 |
| status | **recorded, locked** (`COLLECTED_DATA = True`) | to record | to record |

`_person` and `_card_castle` are the same experiment with a different obstacle —
every value in their configs matches, so they are directly comparable. Keep
them in step.

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
| `gauss` | 500 | the field on |

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

### Replaying `no_gauss` into `gauss`

Hand-driving both cases makes them differ in the field **and** in wherever the
mouse went — and the second difference is usually the bigger one. `replay_goal.py`
removes it: record `no_gauss` from the GUI, then publish that same goal
trajectory back into `gauss`, sample for sample.

**It replays only the reference** — the goals on `/VC_ref_pos`. Torque is still
computed live by the torque node from the arm's actual state and this run's
field. Replaying the torque would replay the first run's motion verbatim and the
Gaussians could not change anything, which is the opposite of the experiment.
The two runs share an *input* and differ in a *controller*.

#### Run 1 — record `no_gauss` from the GUI

Exactly the normal procedure below, with `--run no_gauss`. Drive the tip to the
button by hand, mark it with Ctrl+C, and end with *Home & Quit*. This run is the
source, so drive it the way you want the pair to look.

```bash
bash ../start_arm.sh
```

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _person --run no_gauss
```

```bash
python3 ../VMC/GUI_MultiVMC.py --config _person --run no_gauss
```

#### Run 2 — replay it into `gauss`

Same terminals 1 and 2, with `--run gauss`. **Terminal 3 runs the replay script
instead of the GUI** — this is the step that is easy to get wrong: launching
`GUI_MultiVMC.py` here just gives you an empty GUI waiting to be dragged, and
nothing replays.

```bash
bash ../start_arm.sh
```

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _person --run gauss
```

```bash
python3 replay_goal.py --config _person --from no_gauss --run gauss
```

Then **do nothing**. It settles 8 s while the field ramps in, replays the
trajectory at 100 Hz, re-emits the marks at their recorded times, ends the run
so the torque node writes `gauss.npz`, and finishes the session exactly as
*Home & Quit* does — stop `dynamixel_node` (closing terminal 1), home the arm,
hand back in current mode. `--no-home` leaves the arm up instead.

Check the plan first without touching anything:

```bash
python3 replay_goal.py --config _person --dry-run
```

For the card castle, swap the config — and remember `no_gauss` knocks the castle
over, so record `gauss` first there and replay *it* into `no_gauss`:

```bash
python3 replay_goal.py --config _card_castle --from gauss --run no_gauss
```

| flag | | |
|---|---|---|
| `--from` | `no_gauss` | the run whose trajectory is replayed |
| `--run` | `gauss` | must match the torque node's `--run` |
| `--settle` | 8 s | wait before starting, so the field has ramped in |
| `--rate` | 100 Hz | publish rate |
| `--speed` | 1.0 | replay faster or slower than recorded |
| `--no-home` | | skip the homing at the end |

Goals are matched by component **name**, so a source recorded with a different
set of elements still replays: `gauss_seg3` takes `gauss_seg1`'s goal, since
they share one.

⚠ A click in the GUI *jumps* the goal, so a source can contain steps of a few
centimetres in one frame. A step is reproduced within one publish interval
(10 ms at the default rate); `--speed` above 1 widens that, so leave it at 1 for
real runs.

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

`_button` has three elements, the GUI pair four:

| Component | Element | s | Goal | in |
|---|---|---|---|---|
| `lin_tip` | `LinearSpring(k)` | 1.0 | the button | all |
| `gauss_seg1` | `GaussianSpring(k, 0.06)` | 1/3 | the obstacle | all |
| `gauss_seg2` | `GaussianSpring(k, 0.06)` | 2/3 | **shares `seg1`'s goal** | all |
| `gauss_seg3` | `GaussianSpring(k, 0.06)` | 1.0 | **shares it too** | GUI pair |

The GUI pair puts a Gaussian at the **end of every section**, so each part of
the body that can swing into the obstacle has its own repulsion. `gauss_seg3`
sits at s = 1.0, the same point `lin_tip` pulls: the tip is both drawn to the
target and pushed off the obstacle.

Shared by all three: `SIGMA = 0.06` m, `PRETENSION_WEIGHTS = 0.010` N·m/rad.

Four things differ between the sweep and the GUI pair:

| | `_button` | `_person`, `_card_castle` |
|---|---|---|
| `K_REACH` | 30 N/m | 35 N/m |
| `C_DAMP` | 1.0 N·s/m | 3.0 N·s/m |
| `TAU_LIMIT` | 0.25 N·m | 0.35 N·m |
| elements | 3 | 4 |
| goal z | fixed constants | **each follows its own attachment point** |

In all three variants a goal is only ever moved in x and y by hand. The
difference is what its z is.

`_button` holds every goal z at a constant. In the GUI pair **no goal z is a
reference at all**: every one of them is set to the z of the point it acts on,
each control tick (`GOAL_Z_TRACK`). Each goal therefore sits in the same x–y
plane as its own attachment point, no element has a z error, and nothing pulls
along the arm's axis. The whole model works laterally — which is what an
obstacle standing beside the arm calls for. The tip is steered across its plane;
each section end is pushed off the obstacle in its own. How far the arm extends
is left to gravity and the pretension.

`GOAL_Z_LOCK` is only where each goal starts, before the first FK is available —
that point on the straight arm. The arm does start straight, so tracking has
nothing to correct on the first tick either. Thereafter a goal is one control
period behind its point: measured on a moving arm at **0.09 mm worst case**
(~0.003 N), zero at the median, and the residual correlates with per-tick motion
at 1.000 — pure lag, no offset. `goal` in the log records the z applied, not the
one the GUI published, and those z sliders read *follows the tip*.

The GUI pair pulls harder and settles harder, against a heavier field: 500 N/m
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
`gauss` peaks at 18 N and falls away fast either side of it: 8.1 N at 2σ
(12 cm), 1.7 N at 2.8σ (17 cm) — a tenth of the peak, and by then weaker than
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
locked. `XY_RANGE = ±1.00` m is both the plot extent and the clamp on a dragged
goal — past the arm's 43.5 cm reach on purpose, since a goal outside it sets how
*hard* the tip pulls rather than where it ends up. It also sits close to where
the tip spring alone starts asking more than `TAU_LIMIT` (about |x| = 0.9 m),
past which the limiter rather than the virtual model drives the arm, so most of
the range is usable. `tau_raw` records what was asked, and the run figure shades
where clipping bit.

Placing the obstacle still wants care — a couple of centimetres matter against
`SIGMA = 6` cm — so drop *Drag speed* (`DRAG_SPEED_MAX = 100` cm/s) for that.

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
| `x` | horizontal, across | the GUI's x (±1.00, `_button` ±0.70), `GAUSS_GOAL_XY[0]`, `--target-x` |
| `y` | **vertical — up is +y** | the GUI's y (±1.00, `_button` ±0.70), `GAUSS_GOAL_XY[1]`, `--target-y` |
| `z` | out along the arm's axis | never placed by hand — locked (`_button`) or tracking (GUI pair) |

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
| `goal` | (n, N, 3) | reference per element [m] — for `_person`, everything the hand did |
| `comp_pos` / `comp_vel` | (n, N, 3) | FK position [m] and task-space velocity [m/s] |
| `f_comp` | (n, N, 3) | **each element's force separately** [N]; `f_virtual` (n, 3) is their sum. `N` = 3 for `_button`, 4 for the GUI pair |
| `tau_comp` | (n, N, 9) | each element's share of the torque [N·m] |
| `tau_vmc` / `tau_grav` / `tau_pretension` | (n, 9) | the three terms, before the limiter |
| `tau_raw` / `tau` | (n, 9) | asked vs sent — clipping is `abs(tau_raw) > tau_limit` |
| `q_motor` / `q_motor_dot` | (n, 9) | [rad], [rad/s] |
| `k_params` / `c_params` | (n, N) | spring and damper parameter per element over time — start-up ramp and every toggle included |
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
| `replay_goal.py` | replays a recorded goal trajectory into the other case, so the pair differs only in the field |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` |
| `media/<variant>/` | photos of each set-up, alongside `outputs/<variant>/` |
