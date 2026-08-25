# ButtonPressing

Two headless runs of the same mechanism: a linear reaching spring at the tip
plus two Gaussian repulsive springs on the body. **The tip always reaches for
the same button.** What changes between the two runs is what the body has to
get around — a fixture in one case, a person's arm in the other. Sweeping the
Gaussian strength trades press force against clearance.

Nothing is dragged in the GUI: every value a run depends on lives in its config
file, and a run is identified by that config plus one condition name.

## Mechanism

| Component | Element | s | Goal |
|---|---|---|---|
| `lin_tip` | `LinearSpring(K_REACH)` | 1.0 | `PRESS_TARGET`, ramped from rest during the run |
| `gauss_seg1` | `GaussianSpring(strength, SIGMA)` | 1/3 | `OBSTACLE_POS` |
| `gauss_seg2` | `GaussianSpring(strength, SIGMA)` | 2/3 | `OBSTACLE_POS` |

Torques sum as `tau = sum_i J_i^T F_i`. The two Gaussians share one strength and
act on the body, pushing the arm's midsection off the obstacle while `lin_tip`
still pulls the tip in; that competition is what modulates the contact force.

`PRESS_TARGET` sits `PRESS_DEPTH` **past** the contact surface. Aimed exactly at
the surface, the steady-state spring error — and the contact force with it —
would decay to zero once the tip arrives.

## Parameters

| | `_button` | `_person` |
|---|---|---|
| target (`BUTTON_POS`) | `[-0.20, 0, 0.38]` | `[-0.20, 0, 0.38]` — same button |
| obstacle | `OBSTACLE_POS` `[-0.05, 0, 0.20]` | `PERSON_POS` `[-0.05, 0, 0.20]` — **placeholder, measure it** |
| `K_REACH` / `PRESS_DEPTH` | 25 N/m / 0.02 m → 0.50 N | 15 N/m / 0.02 m → 0.30 N |
| `SIGMA` / `C_DAMP` | 0.06 m / 1.0 N·s/m | 0.06 m / 1.0 N·s/m |
| `TAU_LIMIT` | 0.25 N·m | 0.15 N·m |
| conditions | 0 / 60 / 120 / 200 | 0 / 40 / 80 |
| load cell | yes (`/force_shear`) | no |

The person config is deliberately softer — lower `K_REACH`, lower `TAU_LIMIT` —
because that one runs near a human. Everything else defaults to the button
config's values.

Strengths come from the torque balance at the rest pose: `|tau_gauss|` matches
`|tau_reach|` at ≈48 N/m (button) and ≈29 N/m (person), so each set straddles
the crossover (button ratios 0 / 1.24 / 2.49 / 4.14, person 0 / 1.38 / 2.76).
`--dry-run` recomputes all of this offline after any edit.

Two caveats on those numbers. They are evaluated at the straight-arm rest pose,
so they size the conditions rather than predict them. And they drift one way:
reaching bends the body *toward* the obstacle, so the distance falls toward
`SIGMA` and the repulsion grows — the Gaussians bite somewhat harder in practice.

## Gravity direction

The arm is **rotated onto its side** for both experiments, so gravity pulls
along **-y** in the robot frame, not the upright **-z**. Both configs set
`GRAVITY_DIR = "-y"` and `GUI_MultiVMC_Torque.py` orients its gravity
compensation from it; the node logs the direction on startup, so confirm that
line before recording. Goals and obstacle positions are unaffected — they are
already expressed in the robot frame.

## Sequence

Identical for every condition, only the strength differs:

`RAMP_GAUSS` (`GAUSS_RAMP_S`, strength 0 → condition, tip at rest) → `APPROACH`
(tip reference to `PRESS_TARGET` at `APPROACH_V`) → `HOLD` (`HOLD_TIME`) →
**`CONFIRM`** → `RETRACT` (back home) → `RELEASE` (strength back to 0).

`CONFIRM` is the stop at the end: the driver keeps holding the press and prints

```
  hold complete -- did it touch? press ENTER to return home
```

Press ENTER and the tip reference walks back to the straight-arm rest pose. The
wait is polled rather than blocking, so the reference keeps publishing at
`CTRL_HZ` the whole time and the arm holds its press steadily instead of going
limp. Set `CONFIRM_BEFORE_RETURN = False` to retract straight out of `HOLD`; if
stdin is not a terminal the driver falls through immediately, so scripted runs
behave as before. Returning *home* here means the reference pose — the motors'
own homing routine is still `python3 ../VMC/MultiGoHome.py`.

The timing is all config (`CTRL_HZ`, `APPROACH_V`, `GAUSS_RAMP_S`, `HOLD_TIME`,
`CONFIRM_BEFORE_RETURN`), editable in the same file as the geometry. Defaults:
20 Hz, **2 cm/s**, 5 s ramp, 15 s hold — the 22.7 cm approach takes ≈11.4 s.
That is the *reference* speed; the arm follows it compliantly, so the hardware
lags behind and moves slower still.

## Configuration

The config file is the single source of truth for a run — geometry, elements,
conditions, sequence timing, torque limit, gravity direction, save flags. The
driver, the runners and the offline check all import it, so they cannot
disagree. Two deliberate exceptions:

- **Robot constants** are imported from where they are defined, never copied
  into a config: section length from `Kinematics/JacobiansMulti.py`
  (`REST_LENGTH`), lumped segment mass from `VMC/MultiGravityComp.py`
  (`SEGMENT_MASS`). Copying them into each config is how they drift apart.
- **Plumbing** stays in `_reach_avoid_driver.py`: `POS_TOL` (convergence
  epsilon) and `FLUSH_TICKS` (shutdown guard) describe the code, not the
  experiment.

Boilerplate every config used to repeat — the rest-pose helper and the GUI
parameter introspection — now lives in `VMC_utils/ConfigHelpers.py`. These two
configs are headless, so they carry no GUI slider metadata at all.

## Files

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config_button.py` | config, load cell in the loop |
| `GUI_MultiVMC_Config_person.py` | config, softer reach and lower torque limit |
| `_reach_avoid_driver.py` | shared state machine; not run directly |
| `_geometry_check.py` | offline force/torque check behind `--dry-run` |
| `button_pressing.py` | the runner — takes the variant as its first argument (`_button`, `_person`) |
| `person_avoiding.py` | alias for `button_pressing.py _person` |
| `collect_run.py` | moves the newest auto-saved run into `outputs/` under its condition name |
| `plot_button_and_person.ipynb` | analysis for both |

## Running an experiment

Both commands that matter name the variant — `_button` or `_person` — so
nothing is copied into place and there is no editing between runs. The two must
agree: `button_pressing.py` aborts if the torque node is running a different
config, rather than quietly using one config's strengths against another's
torque limit.

### Pre-flight — once, before the first condition

```bash
conda activate finger
cd ButtonPressing

python3 button_pressing.py _button --dry-run    # offline, no hardware, ~30 s
```

Check, in this order:

1. **The arm is on its side.** Both configs assume `GRAVITY_DIR = "-y"`. If the
   arm is mounted upright, set `"-z"` — see [Gravity direction](#gravity-direction).
2. **The dry-run numbers are sane** — press target, obstacle distances, the
   crossover, and `worst nominal |tau|` under `TAU_LIMIT`. Re-run it after
   *any* config edit; every number comes from the config.
3. **`COLLECTED_DATA = False`**, or the torque node will refuse to save.
4. **Person runs only:** `PERSON_POS` is measured, not the placeholder. Someone
   is on the e-stop. Record `--condition strong` first and confirm the arm
   stops clear before trying anything weaker.

### One condition, start to finish

Four terminals, all from `ButtonPressing/`:

```bash
# Terminal 1 — hardware
bash ../start_arm.sh

# Terminal 2 — load cell (button config only)
python3 ../LoadCell/arduino_force.py

# Terminal 3 — torque node
python3 ../VMC/GUI_MultiVMC_Torque.py --config _button

# Terminal 4 — the run
python3 button_pressing.py _button --condition no_gauss
```

Swap both `_button` for `_person` to run the other variant (and skip terminal
2 — no load cell there).

On startup, terminal 3 logs what it loaded:

```
[INFO] config: GUI_MultiVMC_Config_button
[INFO] gravity compensation: gravity pulls along -y in the robot frame
```

Confirm the config is the variant you meant and the gravity direction matches
how the arm is actually mounted before going further.

Then terminal 4 walks the sequence, roughly 45 s of motion plus however long
you take to answer:

| | what you should see |
|---|---|
| `RAMP_GAUSS` | 5 s, arm still, repulsion fading in |
| `APPROACH` | ≈11 s, tip creeping to the target at 2 cm/s |
| `HOLD` | 15 s steady — this is the measurement window |
| `CONFIRM` | `hold complete -- did it touch? press ENTER to return home` |
| `RETRACT` → `RELEASE` | back to rest, repulsion fading out |

At `CONFIRM` the arm keeps holding its press. Look at the contact, then press
ENTER to send it home. The run ends with:

```
run complete -- Ctrl+C the torque node to save, then run collect_run.py
```

### Saving and repeating

The torque node saves **once, on exit**, so every condition needs its own run
of the node:

```bash
# Terminal 3: Ctrl+C   -> writes experiments/z_<stamp>.{npz,png}
python3 collect_run.py button_pressing no_gauss   # -> outputs/button_pressing/no_gauss.*
# restart terminal 3, then run the next condition
```

Work through every condition in `GAUSS_CONDITIONS` — `no_gauss`, `mid_1`,
`mid_2`, `strong` for the button; `strong` first, then `mid`, `no_gauss` for
the person. `collect_run.py` refuses to overwrite an existing file without
`--force`, so a forgotten Ctrl+C shows up as an error rather than a lost run.

`python3 person_avoiding.py …` still works as an alias for
`button_pressing.py _person`.

### After the last condition

Open `plot_button_and_person.ipynb` and run it — it reads straight from
`outputs/` and writes the figures back there as PDF + SVG. Then set
`COLLECTED_DATA = True` in the config you just recorded, so a later rerun
cannot overwrite the data.

### Tuning outside a condition

`--gauss-strength 90` runs an arbitrary strength without touching the config
(it labels the run `tuning`), and `--hold-time` overrides `HOLD_TIME` for one
run. Both are for exploration — reported runs should come from a named
condition.

### If something looks wrong

| symptom | cause |
|---|---|
| `no subscriber on /VC_ref_pos -- aborting` | terminal 3 is not running, or was started from another directory |
| `torque node is running …_person, this run needs …_button` | the two terminals name different variants; restart terminal 3 with the right `--config` |
| `no GUI_MultiVMC_Config_x.py in …` | typo in the variant name; the error lists what is available |
| `Torque limit exceeded — clipping` spam | strength too high for `TAU_LIMIT`; check the dry-run's `worst nominal \|tau\|` |
| arm sags or drifts to one side | `GRAVITY_DIR` does not match the real mounting |
| `no z_*.npz in .../experiments` | `COLLECTED_DATA = True`, or the torque node was launched from elsewhere, or killed rather than Ctrl+C'd |

## Data

`GUI_MultiVMC_Torque.py` auto-saves to `experiments/z_<stamp>.{npz,png}`;
`collect_run.py` moves that pair into `outputs/button_pressing/<condition>.{npz,png}`
or `outputs/person_avoiding/<condition>.{npz,png}`, which is where the notebook
looks. The `force` field is raw load cell **grams** — multiply by `9.80665e-3`
for newtons.

The notebook finds each run's steady-state window on its own, from the logged
tip goal and `k_params`, and reads the Gaussian attachment points from the
run's own `comp_s_params`, so nothing about the analysis is hardcoded. It
writes `force_vs_time`, `force_vs_strength` and `backbone` for the button runs,
and `clearance` for the person runs — closest approach of the whole arm to the
person, alongside the tip's remaining reach error (PDF + SVG).

Both configs ship `COLLECTED_DATA = False` so runs actually save. Flip both to
`True` once the reported runs are collected, or a later rerun will overwrite them.
