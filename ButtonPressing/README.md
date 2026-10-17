# ButtonPressing — Regulating and avoiding contact

A linear spring pulls the tip towards a target, and repulsive Gaussians push
the arm off an obstacle. Turning the Gaussians up buys clearance and costs
reach. On the button, that trade-off sets how hard the body presses (contact
regulation). Next to a card castle, or an obstacle on the way to a person, it
keeps the arm clear (contact avoidance).

Three variants:

| | `_button` | `_person` | `_card_castle` |
|---|---|---|---|
| what it shows | reach traded for clearance, swept | a teleoperated reach to a person (handover), replayed with body Gaussians so the arm clears an obstacle | the tip deflected round a card castle on its way to a target beyond it (designed as a standoff, see [the standoff demo](#_card_castle--the-standoff-demo-no-gui)) |
| elements | 3: tip + 2 body Gaussians | 4: tip + 3 body Gaussians | **2, both on the tip** |
| obstacle | a fixture at x = −0.15 | an obstacle at x = −0.25 | the cards at (0, +0.25), target beyond at (0, +0.50) |
| driven by | `run_sweep.py`, no GUI | the GUI by hand (`no_gauss`), then replayed into `gauss` by `replay_goal.py` | `castle_demo.py`, **no GUI** |
| Gaussian strength | swept 0 / 200 / 600 / 1000 | 2 cases: 0 and 500 | 2 cases: 0 and 1500 |
| `TAU_LIMIT` | 0.25 N·m | **0.35 N·m** | **0.35 N·m** |
| load cell | yes | no | no |
| files | `outputs/button_pressing/`, 4 × 5 repeats | `outputs/person_avoiding/`, 2 | `outputs/card_castle/`, 2 |
| status | **recorded, locked** (`COLLECTED_DATA = True`) | **recorded, locked** (`COLLECTED_DATA = True`) | **recorded, locked** (`COLLECTED_DATA = True`) |
| recorded result | clearance **1.7 → 6.1 → 10.5 → 12.1 cm** as strength goes 0 → 200 → 600 → 1000 N/m; reach error 23.1 → 35.8 cm | clearance **8.8 → 13.5 cm** (3-D; 7.3 → 12.3 cm lateral) at 0 → 500 N/m | `no_gauss` goes through the cards; `gauss` **slides round them** and ends 15.0 cm from the centre, beside it and past it |

Numbers are from the run logs, see [Recorded results](#recorded-results).

`_person` keeps the body-shaping model: Gaussians spread along the arm, goals
placed by hand and then replayed. `_card_castle` is a different demo: both elements sit on the
tip but pull and push towards *different* points, and the whole run is scripted.

---

# What to run

Every terminal starts from:

```bash
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

## `_person` — the pair, from the GUI

| case | k [N/m] | |
|---|---|---|
| `no_gauss` | 0 | no field — the body goes wherever reaching takes it |
| `gauss` | 500 | the field on |

`_card_castle` uses the same two case names (0 and 1500 N/m) but is scripted —
see [the standoff demo](#_card_castle--the-standoff-demo-no-gui) below.

Already recorded. To record again, first set `COLLECTED_DATA = False` in
`GUI_MultiVMC_Config_person.py`, which is `True` so the two runs cannot be
overwritten.

Two runs, nothing in between: the comparison is the point, not a curve. Three
terminals, **run the whole thing twice**, once per case. The tip is pulled
towards the same goal both times; what changes is how far the *body* stays off
the obstacle. In the recorded pair that goal sits 0.81 m from the base —
beyond the arm's 43.5 cm reach, as `XY_RANGE` allows on purpose — so the tip
ends at full stretch rather than on the goal, and the two tips finish 6.2 cm
apart (see [Recorded results](#recorded-results)).

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

**Terminal 3** — the GUI. **`--run` must be the same case as terminal 2**: the
GUI applies all four stiffnesses (tip + three Gaussians) at once, so one told
the wrong case would apply the wrong field the first time you press *Apply
Parameters*. It refuses to start without it.

```bash
python3 ../VMC/GUI_MultiVMC.py --config _person --run gauss
```

```bash
python3 ../VMC/GUI_MultiVMC.py --config _person --run no_gauss
```

`_card_castle` has no GUI: its terminals 2 and 3 are in
[the standoff demo](#_card_castle--the-standoff-demo-no-gui).

### Replaying `no_gauss` into `gauss`

Hand-driving both cases makes them differ in the field **and** in wherever the
mouse went — and the second difference is usually the bigger one. `replay_goal.py`
removes it: record `no_gauss` from the GUI, then publish that same goal
trajectory back into `gauss`, sample for sample. The recorded pair was made
this way: `gauss`'s goals match `no_gauss`'s, shifted by the 8 s settle, to
within 0.7 mm on average.

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

The card castle does not need this: `castle_demo.py` already drives both cases
on the same clock. It only makes sense there if a castle run was ever driven by
hand — and since `no_gauss` knocks the castle over, that would mean recording
`gauss` first and replaying *it* into `no_gauss`:

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
   follows at that rate. Slower is better with someone standing there. (The
   recorded pair put it at (−0.773, +0.248) m — out of reach, see above.)
4. **Ctrl+C once in terminal 2** when the tip is on the button. That is the
   mark, and the analysis takes its measurement window from it. Hold still a
   few seconds after.

   ⚠ **The recorded `person_avoiding/` runs carry no mark** (`marks_t` is
   empty in both), so the notebook *infers* the window — the longest stretch
   in which no goal or parameter changed: 26.2–50.5 s for `no_gauss`, 34.3–58.5 s
   for `gauss`, each ending with its run. Mark the source run, and
   `replay_goal.py` re-emits the mark in the replayed run at the same point in
   the trajectory; a source with no mark gives no mark in either.
5. **Home & Quit** (cell ②) → confirm. It ends the whole session in one go:
   terminal 2 saves and exits → `dynamixel_node` stops (closing terminal 1) →
   the arm homes at ~9.2 rpm → terminal 3 closes.
6. Repeat from `start_arm.sh` for the other case.

⚠ Homing hands the arm back in **current mode**, so it goes limp at
`HOME_POSITION`. Check nothing fragile is under where it comes to rest.

The run is saved **first**, before anything is stopped — it is the only step
that cannot be redone. Once a run is ending, **Ctrl+C is ignored**, so a second
press cannot interrupt the save. Closing the window with
its X saves too, but leaves the arm and driver alone.

If a motor misses home, or lands a whole turn out, the GUI says so and stays
open — motors keep holding in position mode and are **not** handed back to
current mode, because in current mode the arm goes limp and whatever pose it is
in becomes the model's zero next start-up. A delta that is a multiple of 4096
is a lost turn count, not a failed move: power-cycle, re-place by hand, and
re-capture with `python3 ../VMC/MultiGoHome.py --read`.

---

## `_card_castle` — the standoff demo, no GUI

Two elements, **both on the tip**, with **two different targets** — and the
castle sits between the tip and where it is being sent:

| | | aimed at | |
|---|---|---|---|
| `lin_tip` | `LinearSpring(35)` | `TARGET_XY = (0, +0.50)` | pulls the tip **past** the cards |
| `gauss_tip` | `GaussianSpring(k, 0.06)` | `CASTLE_XY = (0, +0.25)` | pushes it **away** from them |

So the tip is asked to cross the castle and the field is the only thing in the
way. Along the path (x = 0) the two forces cancel at one point, and that is
what the demo was designed around. What the two runs did:

| `--run` | k [N/m] | predicted along the path | recorded |
|---|---|---|---|
| `no_gauss` | 0 | nothing stops the tip — it drives for the target **through** the cards | the tip ends at (0.8, 30.6) cm, 5.6 cm **past** the castle centre (closest approach 0.7 cm at t = 17.5 s). the tip is then 42 cm from the base, near full stretch: the target at y = 50 cm is out of reach |
| `gauss` | 1500 | the tip stalls **14.0 cm short** of them, at (0, 0.110) | the tip gets to (5.5, 11.5) cm at t = 19 s — the predicted stall's height, but already 5.5 cm off the axis — then **slides sideways round the castle** and ends at (13.1, 32.3) cm: 15.0 cm from its centre, but beside it and *past* it (the castle is at y = 25 cm) |

### The predicted stall is a saddle

The balance is stable **along the path**: nearer, the push wins and the tip is
driven back; further out, the pull wins and it is drawn in. Across the path it
is not. Linearised about the stall point, the net virtual force changes with
tip position as

| direction | dF/dx | |
|---|---|---|
| along the path (y) | −470 N/m | restoring |
| across it (x) | **+62.4 N/m** | pushes the tip further sideways |

The Gaussian pushes radially, so a sideways offset is pushed out at
`k·e^(−r²/2σ²)` = 97.4 N/m there, while the tip spring pulls it back at only
35 N/m. In closed form the sideways stiffness is `K_REACH · (d_target / d_castle − 1)`,
with both distances measured from the tip: **positive — unstable — whenever the
target lies beyond the castle**, for any `k` or `SIGMA`. Retuning strength or
width cannot fix it. What can hold the tip on the axis is only what else acts
sideways — the arm's own stiffness and friction — and in the recorded run
that was not enough.

To make the stall hold, something has to resist sideways motion by more than
62.4 N/m at that point (plus margin) — for instance a third element on the tip,
an x-only `ConstrainedLinearSpring` (`n = (1, 0, 0)`) centred on x = 0. **Not
tried on the arm.** `castle_demo.py`'s solver only projects the force onto the
path, so its "stable" means stable along the path and it cannot see this.

⚠ **The barrier is finite, not a wall.** There is a second, *unstable*
equilibrium 0.6 cm from the castle: past it the Gaussian falls off faster than
the linear pull grows, and the tip would pop through to the far side.
Approaching from the start it never gets there — but raise `K_REACH` (to about
200 N/m at these values), or push `TARGET_XY` far enough out, and the two
equilibria merge and the barrier disappears. `castle_demo.py --dry-run` solves
for both along the path and prints them, so a change can be checked **before**
the cards are built:

```
[demo] expected: the tip stalls 14.0 cm short of the castle
[demo]     stable   at (+0.000, +0.110),  14.0 cm from the cards
[demo]     unstable at (+0.000, +0.244),   0.6 cm from the cards
```

The stall point moves out only slowly with strength — the balance is logarithmic
in the ratio: 10.1 cm at 500 N/m, 14.0 cm at 1500, 19.0 cm at 12000 — whereas it
is proportional to `SIGMA` (6.2 / 14.0 / 22.0 cm at σ = 3 / 6 / 9 cm). `SIGMA`
and `TARGET_XY` are the better handles if 14.0 cm is the wrong distance.

Already recorded. To record again, first set `COLLECTED_DATA = False` in
`GUI_MultiVMC_Config_card_castle.py`, which is `True` so the two runs cannot be
overwritten. Running a case then **overwrites** `outputs/card_castle/<case>.npz`.
Record `gauss` first, then rebuild the castle for `no_gauss`.

### Running it

**Terminal 1** — the arm

```bash
bash ../start_arm.sh
```

**Terminal 2** — the controller and the log

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _card_castle --run gauss
```

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _card_castle --run no_gauss
```

**Terminal 3** — the demo, *instead of* the GUI. Same `--run` as terminal 2.

```bash
python3 castle_demo.py --run gauss
```

```bash
python3 castle_demo.py --run no_gauss
```

```bash
python3 castle_demo.py --run gauss --dry-run
```

Then **do nothing** — it runs itself, ≈38 s (the recorded runs: mark at
28.0 s, end at 38.1 s):

1. the field comes up 0 → k over `RAMP_S` = 5 s, with the tip goal still sitting
   on the tip so the linear spring pulls nothing while it does;
2. `DEMO_SETTLE_S` = 5 s — the arm settles with the field at full strength;
3. the tip goal walks the 50 cm to `TARGET_XY` at `DEMO_SPEED` = 5 cm/s (10 s),
   straight past the castle;
4. `DEMO_ARRIVE_S` = 8 s — the arm follows, meant to stall short of the cards
   (`no_gauss`: driving through them; recorded `gauss`: sliding round them);
5. the window is marked and held for `DEMO_WINDOW_S` = 10 s, and the run is
   written;
6. the driver is stopped (closing terminal 1) and the arm homed, as *Home &
   Quit* does. `--no-home` skips it.

`--settle`, `--speed`, `--arrive`, `--window` override the config per run.
`--dry-run` prints the plan, the total time and the predicted standoff without
publishing anything.

⚠ The castle goal never moves — only the tip goal walks, to its own separate
target. And homing hands the arm back in current mode, so it goes **limp** at
`HOME_POSITION`: check the cards are not under where it comes to rest.

---

# The model

Each variant has its own element set:

| Component | Element | s | `_button` | `_person` | `_card_castle` |
|---|---|---|---|---|---|
| `lin_tip` | `LinearSpring(k)` | 1.0 | ✓ | ✓ | ✓ |
| `gauss_seg1` | `GaussianSpring(k, 0.06)` | 1/3 | ✓ | ✓ | |
| `gauss_seg2` | `GaussianSpring(k, 0.06)` | 2/3 | ✓ | ✓ | |
| `gauss_seg3` | `GaussianSpring(k, 0.06)` | 1.0 | | ✓ | |
| `gauss_tip` | `GaussianSpring(k, 0.06)` | 1.0 | | | ✓ |

`_person` puts a Gaussian at the **end of every section**, so each part of the
body that can swing into the obstacle has its own repulsion; the three share one
goal. `_card_castle` keeps only the tip: one spring pulling it towards a target
beyond the cards and one pushing it off the cards themselves.

Shared by all three: `SIGMA = 0.06` m, `PRETENSION_WEIGHTS = 0.010` N·m/rad.

Four things differ between the sweep and the other two:

| | `_button` | `_person`, `_card_castle` |
|---|---|---|
| `K_REACH` | 30 N/m | 35 N/m |
| `C_DAMP` | 1.0 N·s/m | 3.0 N·s/m |
| `TAU_LIMIT` | 0.25 N·m | 0.35 N·m |
| goal z | fixed constants | **each follows its own attachment point** |

In all three a goal is only ever moved in x and y. The difference is what its
z is.

`_button` holds every goal z at a constant. In the other two **no goal z is a
reference at all**: every one of them is set to the z of the point it acts on,
each control tick (`GOAL_Z_TRACK`). Each goal therefore sits in the same x–y
plane as its own attachment point, no element has a z error, and nothing pulls
along the arm's axis. The whole model works laterally — which is what an
obstacle standing beside the arm calls for. How far the arm extends is left to
gravity and the pretension.

`GOAL_Z_LOCK` is only where each goal starts, before the first FK is available —
that point on the straight arm. The arm does start straight, so tracking has
nothing to correct on the first tick either. Thereafter a goal is one control
period behind its point: measured on a moving arm at **0.09 mm worst case**
(~0.003 N), zero at the median, and the residual correlates with per-tick motion
at 1.000 — pure lag, no offset. `goal` in the log records the z applied, not the
one the GUI published, and the GUI's z sliders read *follows the tip*.

Both pull harder and settle harder than the sweep, against a heavier field —
500 N/m next to the person, 1500 N/m against the cards — where ringing is the
thing to avoid. The stiffer tip spring keeps the tip
arriving at the button while the body is being pushed off, and the higher torque
limit leaves room for both. Damping costs nothing in steady state — a damper
only pushes back while the arm is moving.

**Strengths are ramped 0 → k over `RAMP_S`** at start-up — 5 s in all three
variants (for `_card_castle` bringing the field up slowly is also step 1 of the
demo) — before anything publishes, so it lands at `t < 0`. Switched on at once,
a placed field shoves the arm off the obstacle in one tick. `k_params` records
what was applied, tick by tick. A `--run` the config has no level for is
refused in a second, before anything is built.

⚠ **`GAUSS_GOAL_XY` must sit at more negative x than the points the field
pushes ever reach**, or the repulsion points *at* the button. At an early
`-0.05` the body had already curved past the centre, the field **assisted** the
reach, and shear rose with strength while reach error fell — monotonic, but not
avoidance. The button uses `-0.15`, the person `-0.25`; both oppose. In
`_button` those points are the two Gaussians' attachments: `gauss_seg2` sits at
x = −11.5 → −5.2 cm across the three fields, on the centre (−15.5 cm) with the
field off. The **tip carries no Gaussian there** and does go past the centre
(−27.6 cm with the field off). `_card_castle` is the exception and is meant to
be so: there the Gaussian centre IS the target, and the standoff is the result.

⚠ **`tau_raw` records what was asked, and the limiter never bit in any recorded
run** — the run figure shades where it would. Peaks of `|tau_raw|` over the
whole run: 0.049 N·m for `_button` (limit 0.25), 0.095 N·m for `_person` and
0.061 N·m for `_card_castle` (limit 0.35). If you move a target or raise a
strength, check `tau_raw` rather than trusting an extrapolation. The limiter warns once every 2 s (`CLIP_WARN_S`), not once per
tick.

## `_button` only

`--target-x/y` defaults to `(-0.50, 0)` — beyond the arm's 43.5 cm reach on
purpose, so the tip ends near full stretch (42.6–43.0 cm from the base, of 43.5)
and the target sets how *hard* it pulls there. The tip spring pulls 6.9 N with
the field off, rising to 10.7 N at 1000 N/m as the tip is held further from the
goal (30 N/m × 23 → 36 cm).
Each level is recorded `SWEEP_REPEATS` = 5 times as `<level>_r<N>`; the level is
still resolvable from the name, so the analysis groups them automatically. This
is the only folder with error bars.

## `_person` only

Strengths are **field strengths, not forces**: a Gaussian peaks at
`k·exp(−½)·σ = k·0.0364` N when its attachment point is σ from the centre, so
`gauss` peaks at 18 N and falls away fast either side of it: 8.1 N at 2σ
(12 cm), 1.7 N at 2.8σ (17 cm) — a tenth of the peak, and by then weaker than
the reach. **Placement matters more than strength.** What the motors get is
clipped at `TAU_LIMIT`, which is 0.35 here against the button's 0.25, so the
limiter has less say in the pose.

With the obstacle at −0.25 it is **`gauss_seg2` (s = 2/3) that does the work**.
In the recorded `gauss` run it sits at x = −18.4 cm, 12.4 cm = 2.1σ from the
centre (laterally, x–y) and pushes with 7.4 N; with the field off it passes
7.5 cm = 1.25σ away, near the peak — the field moves it out. `gauss_seg1` sits
at −6.1 cm, 19.2 cm away, and contributes 0.6 N; the tip's `gauss_seg3` is
17.8 cm away and contributes 1.1 N. If the arm barely reacts, the centre is too
far from where the body actually passes — move it, or raise `SIGMA`, before
reaching for a bigger `k`.

The GUI can change: both goals' x/y, the tip stiffness, all three Gaussian
strengths (cell ①, `Apply Parameters`), and a **`Gaussians active` tickbox**.
Every z is locked (tracking, see below). `XY_RANGE = ±1.00` m is both the plot
extent and the clamp on a dragged goal — past the arm's 43.5 cm reach on
purpose, since a goal outside it sets how *hard* the tip pulls rather than where
it ends up: the recorded goal, 0.81 m out, pulls the tip with about 16 N and
peaked at 0.095 N·m against the 0.35 limit, so goals out to at least that
distance are fine; beyond it, check `tau_raw`, which records what was asked
(the run figure shades where clipping bit).

Placing the obstacle still wants care — a couple of centimetres matter against
`SIGMA = 6` cm — so drop *Drag speed* (`DRAG_SPEED_MAX = 100` cm/s) for that.

The tickbox owns *whether* the field is on, the entries own *how strong* when it
is — so it returns to whatever was last applied, and a strength typed while it
is off is remembered for the next tick. Either way the strength is **walked
over `RAMP_S` in both directions**. Untouched, it publishes nothing, so it
cannot cancel the start-up ramp. `k_params` records every walk, and the
analysis segments on it.

---

# Recorded results

Read off the run logs with the notebooks' own loaders (`summary()` and the hold
window). **Clearance** is the smallest 3-D distance from the FK backbone, at its
mean pose over the hold window, to the obstacle centre. **Reach error** is the
distance from the tip goal to the tip. The hold window starts 1 s after the mark
and ends with the run.

## `_button` — 4 levels × 5 repeats

Mean ± sample SD (n = 5). The error bars in the figures use `np.std`, the
population SD (ddof = 0), which is 0.894× the value listed here.

| level | k [N/m] | clearance [cm] | reach error [cm] | tip x [cm] | shear force [N] | tip-spring pull [N] |
|---|---|---|---|---|---|---|
| `no_gauss` | 0 | 1.75 ± 0.10 | 23.1 ± 0.06 | −27.6 | 0.62 ± 0.04 | 6.92 |
| `weak` | 200 | 6.09 ± 0.04 | 27.6 ± 0.04 | −22.5 | 0.22 ± 0.02 | 8.29 |
| `medium` | 600 | 10.5 ± 0.02 | 33.5 ± 0.02 | −16.6 | 0.077 ± 0.025 | 10.0 |
| `strong` | 1000 | 12.1 ± 0.01 | 35.8 ± 0.05 | −14.3 | 0.004 ± 0.007 | 10.7 |

Goal `(-0.50, 0, 0.38)`, obstacle `(-0.15, 0, 0.2175)`. Mark at 55.2 s, hold
56.2–75.2 s, never clipped. Clearance rises monotonically with strength and so
does reach error, while the load-cell shear force falls from 0.62 N to ≈ 0 as
the tip is held off the button — that is the trade-off, in numbers.

## `_person` — one run per case

| | `no_gauss` | `gauss` |
|---|---|---|
| k [N/m] | 0 | 500 |
| obstacle, tip goal (xy) [cm] | (−25.0, 0.0), (−77.3, +24.8) — 81 cm from the base | same |
| clearance, 3-D / lateral x–y / x–z [cm] | 8.8 / 7.3 / 5.3 | **13.5 / 12.3 / 6.4** |
| tip at the hold (x, y, z) [cm] | (−34.7, 10.9, 18.9) | (−31.9, 16.4, 19.9) |
| body lift, max \|y\| [cm] | 10.9 | 16.4 |
| pull on the tip, \|F\| [N] | 15.7 | 16.2 |
| Gaussian force seg1 / seg2 / seg3 [N] | 0 / 0 / 0 | 0.57 / **7.39** / 1.08 |
| peak \|tau_raw\| [N·m], limit 0.35 | 0.072 | 0.095 |
| window (inferred — no marks) | 26.2–50.5 s | 34.3–58.5 s |

The two tips finish 6.2 cm apart, both near full stretch: the goal is out of
reach, so this pair shows what the field does to the *body* — not that the tip
lands on the same spot.

## `_card_castle` — one run per case

| | `no_gauss` | `gauss` |
|---|---|---|
| k [N/m] | 0 | 1500 |
| tip at the hold (x, y, z) [cm] | (0.8, 30.6, 28.3) | (13.1, 32.3, 21.8) |
| distance to the castle centre (0, 25) [cm] | 5.6 | 15.0 |
| closest approach [cm] | 0.7 (t = 17.5 s) | 14.6 (t = 19 s, tip at (5.5, 11.5)) |
| \|F\| tip spring / Gaussian [N] | 6.8 / 0 | 7.7 / 9.8 |
| peak \|tau_raw\| [N·m], limit 0.35 | 0.049 | 0.061 |
| mark, end of run | 28.0 s, 38.1 s | 28.0 s, 38.1 s |

`gauss` did not stall short of the castle: see
[why](#the-predicted-stall-is-a-saddle).

---

# Gravity and axes

All three variants were recorded with the arm mounted **on its side**, and all
three configs set `GRAVITY_DIR = "-y"` (override per run with `--gravity-dir`):

| axis | physically | set by |
|---|---|---|
| `x` | horizontal, across | the GUI's x (±1.00, `_button` ±0.70), `GAUSS_GOAL_XY[0]`, `--target-x` |
| `y` | **vertical — up is +y** | the GUI's y (±1.00, `_button` ±0.70), `GAUSS_GOAL_XY[1]`, `--target-y` |
| `z` | out along the arm's axis | never placed by hand — locked (`_button`) or tracking (`_person`, `_card_castle`) |

So an **x–z plot is a plan view from above**, and an **x–y plot looks along the
arm's axis** (a front view). The *side* view — where the arm's droop under
gravity shows — is **y–z**, which the notebooks do not plot.

⚠ No single projection is the geometry, and the arm leaves the y = 0 plane by
different amounts in different variants. `_person`'s `gauss` run is **lifted
16.4 cm** by the +y component of the Gaussians — invisible from above: in x–z the
backbone is 6.4 cm from the obstacle, in 3-D 13.5 cm (12.3 cm laterally). The
`_button` sweep, whose goal z is locked, stays within 1.8 cm of y = 0, so there
x–z and 3-D agree to 0.15 cm (1.60 vs 1.75 cm with the field off). Read clearance
and reach error in 3-D — or, for `_person` and `_card_castle`, laterally, as the
next point explains.

⚠ **In `_person` and `_card_castle` an obstacle's or target's z is not a
coordinate.** `GOAL_Z_TRACK` sets every goal's z to its own attachment point's z
each tick (in the recorded `person_avoiding/gauss` run `gauss_seg1`'s goal z
differs from its attachment z by ≤ 0.07 mm), so the recorded `goal` z is just the
arm's own z. The field is a cylinder along z; the clearance that matters is the lateral x–y distance. A
marker for the obstacle or the target drawn at that z in an x–z view marks
nothing physical — in the castle runs the tip, the castle and the target all
share one z.

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
| `f_comp` | (n, N, 3) | **each element's force separately** [N]; `f_virtual` (n, 3) is their sum. `N` = 3 for `_button`, 4 for `_person`, 2 for `_card_castle` |
| `tau_comp` | (n, N, 9) | each element's share of the torque [N·m] |
| `tau_vmc` / `tau_grav` / `tau_pretension` | (n, 9) | the three terms, before the limiter |
| `tau_raw` / `tau` | (n, 9) | asked vs sent — clipping is `abs(tau_raw) > tau_limit` |
| `q_motor` / `q_motor_dot` | (n, 9) | [rad], [rad/s] |
| `k_params` / `c_params` | (n, N) | spring and damper parameter per element over time — start-up ramp and every toggle included |
| `force` / `force_age` | (n,) | load-cell shear, raw **grams** (×`9.80665e-3` for N), and its staleness [s] |
| `marks_t` / `marks_label` / `marks_wall` | (m,) | where the mark was made, on `t`'s clock: Ctrl+C in the torque node, or `/vmc_mark` from `castle_demo.py` / `replay_goal.py`. Empty (m = 0) for both `person_avoiding` runs |
| `comp_spec` / `config_source` | (1,) | every element's type and values as JSON; the whole config file |
| metadata | | `comp_names`, `comp_s_params`, `spring_param_names`, `damper_param_names`, `control_freq`, `tau_limit`, `gravity_dir`, `config_name`, `run_name`, `ramp_components`, `ramp_strength`, `ramp_s`, `pretension_weights`, `pretension_enabled`, `t_reference_ok`. The 20 `button_pressing` runs predate `ramp_components` / `ramp_strength` and carry them under their old names `condition_targets` / `condition_strength` (still written alongside, so a re-recorded run is field-identical) |

Everything positional is taken from the run, never the config — for `_person`
the goals were dragged live, so where the button and the obstacle actually were
is in `goal` and nowhere else. The same for strength, read from `k_params` over
the window.

# Files

| File | |
|---|---|
| `GUI_MultiVMC_Config_button.py` | `_button`: the elements, the four swept levels |
| `GUI_MultiVMC_Config_person.py` | `_person`: the two cases, `TAU_LIMIT = 0.35`, what the GUI may touch |
| `GUI_MultiVMC_Config_card_castle.py` | `_card_castle`: two tip elements with separate targets, and the demo's timings |
| `castle_demo.py` | `_card_castle` only: the whole scripted demo, no GUI |
| `run_sweep.py` | `_button` only: every level × `SWEEP_REPEATS`, homed and verified between |
| `plot_button.ipynb` / `plot_person.ipynb` / `plot_card_castle.ipynb` | analysis, one notebook per variant — discovers `outputs/`, positions from the data, window from the mark. See [Plotting](#plotting) |
| `replay_goal.py` | replays a recorded goal trajectory into the other case, so the pair differs only in the field |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` |
| `sensor/fdc1004_single_taxel/` | Arduino sketch for a single FDC1004 capacitive taxel. Nothing in this repo reads it, and no recorded run uses it |
| `media/<variant>/` | photos of each set-up, alongside `outputs/<variant>/`. Only `media/button/` (`photo_1..6.jpg`) is populated; `person/` and `card_castle/` hold a `.gitkeep` |

# Plotting

Each notebook writes its figures as PDF **and** SVG next to the runs
(`outputs/<variant>/`):

| notebook | writes |
|---|---|
| `plot_button.ipynb` | `backbone` (all four overlaid), `backbone_<level>` (×4), `force_vs_time` (time axis to 65 s, `FORCE_TIME_MAX_S`), `force_vs_strength`; and, **only with `PLOT_SHEAR = True`** (off by default), `shear_<level>_{full,zoom}` (×4 levels) |
| `plot_person.ipynb` | `backbone` (X–Y above X–Z, sharing X), `tip_trajectory` (X–Y); each with its legend as a figure of its own, `backbone_legend` / `tip_trajectory_legend` |
| `plot_card_castle.ipynb` | the same four figures |

To know before you rely on them:

- **Requirements.** LaTeX (the shared style sets `text.usetex`) and a matplotlib
  that is **not 3.11.0**, which drops the minus sign from usetex PDFs (see the
  [top-level README](../README.md#reproducing-the-figures)). The committed
  figures were made with 3.11.2.
- **Palette.** These notebooks pin their own colours instead of the repo's pastel
  default: NPG in `plot_button`, Dark2 (`#1B9E77`, `#D95F02`) in `plot_person` and
  `plot_card_castle`. So `no_gauss` is cyan in the button figures and green in
  the other two.
- **Views.** The geometry figures are X–Z (button, at **equal scale**: 1 cm in Z
  is the same length as 1 cm in X, so the σ circle is a circle and the panel
  takes the shape of its data — the combined figure is tall because the target
  sits at X = −50 cm) and X–Y over X–Z (person, castle; equal scale on every
  axis of both panels too). In all of them only the **width** is set (in the
  notebook cell, saved at exactly that width) and the height follows from the
  data, so a tall scene gives a tall figure — the castle's X–Y over X–Z is about
  2.8 times taller than wide (X capped at 25 cm, 28 pt text), because the target sits 50 cm up. The person and
  castle figures carry no legend: it is a separate `*_legend` figure, sized to
  the legend itself (the castle's draws its coloured lines vertical). See [Gravity and axes](#gravity-and-axes) for what those are, and for
  why the obstacle and target markers in the X–Z panels of `_person` and
  `_card_castle` sit at meaningless z.
- **Helper code** (`hold_window`, `load_run`, `backbone`, …, about 130 lines) is
  copied verbatim into all three notebooks. Change one and change the others.
