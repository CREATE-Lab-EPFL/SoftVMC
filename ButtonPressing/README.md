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
| `button_pressing.py` | runner for the button config |
| `person_avoiding.py` | runner for the person config |
| `collect_run.py` | moves the newest auto-saved run into `outputs/` under its condition name |
| `plot_button_and_person.ipynb` | analysis for both |

## Running

`GUI_MultiVMC_Torque.py` always imports a module literally named
`GUI_MultiVMC_Config` from the current directory, so copy the variant into place
first. The runners refuse to start against the wrong one.

```bash
conda activate finger
cd ButtonPressing
cp GUI_MultiVMC_Config_button.py GUI_MultiVMC_Config.py

python3 button_pressing.py --dry-run          # offline, no hardware

# Terminal 1
bash ../start_arm.sh
# Terminal 2  (button only)
python3 ../LoadCell/arduino_force.py
# Terminal 3
python3 ../VMC/GUI_MultiVMC_Torque.py
# Terminal 4
python3 button_pressing.py --condition no_gauss
```

The torque node saves once, on exit, so each condition needs its own run:
Ctrl+C the torque node, collect the file, restart it for the next condition.

```bash
python3 collect_run.py button_pressing no_gauss
```

For the person config, `cp GUI_MultiVMC_Config_person.py GUI_MultiVMC_Config.py`,
drop the load-cell terminal, and record `--condition strong` first — confirm the
arm stops clear before running anything lower. `PERSON_POS` — where the
person's arm is — is a placeholder until measured, and someone should be on the
e-stop for every run.

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
