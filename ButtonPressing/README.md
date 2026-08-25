# ButtonPressing

Two headless runs of the same mechanism: a linear reaching spring at the tip
plus two Gaussian repulsive springs on the body. Sweeping the Gaussian strength
modulates how hard the tip presses on what it reaches for — a load cell in one
case, a person's arm in the other.

Nothing is dragged in the GUI: every position and strength is hardcoded, and a
run is identified by its config file plus one condition name.

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
| target | `[0.05, 0, 0.38]` | `[0.10, 0, 0.35]` — **placeholder, measure it** |
| `OBSTACLE_POS` | `[0.07, 0, 0.22]` | `[0.07, 0, 0.22]` |
| `K_REACH` / `PRESS_DEPTH` | 25 N/m / 0.05 m → 1.25 N | 15 N/m / 0.03 m → 0.45 N |
| `SIGMA` / `C_DAMP` | 0.06 m / 1.0 N·s/m | 0.06 m / 1.0 N·s/m |
| `TAU_LIMIT` | 0.25 N·m | 0.15 N·m |
| conditions | 0 / 60 / 120 / 200 | 0 / 40 / 80 |
| load cell | yes (`/force_shear`) | no |

Strengths come from the torque balance at the rest pose: `|tau_gauss|` matches
`|tau_reach|` at ≈70 N/m (button) and ≈28 N/m (person), so each set straddles
the crossover. `--dry-run` reprints all of this offline after any edit.

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

`RAMP_GAUSS` (5 s, strength 0 → condition, tip at rest) → `APPROACH` (tip
reference to `PRESS_TARGET` at 4 cm/s) → `HOLD` (15 s) → `RETRACT` → `RELEASE`
(strength back to 0).

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
arm stops clear before running anything lower. `PERSON_TARGET` is a placeholder
until measured, and someone should be on the e-stop for every run.

## Data

`GUI_MultiVMC_Torque.py` auto-saves to `experiments/z_<stamp>.{npz,png}`;
`collect_run.py` moves that pair into `outputs/button_pressing/<condition>.{npz,png}`
or `outputs/person_avoiding/<condition>.{npz,png}`, which is where the notebook
looks. The `force` field is raw load cell **grams** — multiply by `9.80665e-3`
for newtons.

The notebook finds each run's steady-state window on its own, from the logged
tip goal and `k_params`, so no timing is hardcoded in the analysis. It writes
`force_vs_time`, `force_vs_strength` and `backbone` for the button runs, and
`standoff` for the person runs (PDF + SVG).

Both configs ship `COLLECTED_DATA = False` so runs actually save. Flip both to
`True` once the reported runs are collected, or a later rerun will overwrite them.
