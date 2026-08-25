# ButtonPressing

A linear reaching spring at the tip plus two repulsive Gaussian springs on the
body, driven **live from the GUI**. The tip reaches for a button; the Gaussians
push the arm's midsection off whatever is in the way. Trading those two against
each other is the experiment.

Two variants, differing only in what the obstacle is and how much authority the
arm is allowed:

| | `_button` | `_person` |
|---|---|---|
| obstacle | a fixture | a person's arm |
| `TAU_LIMIT` | 0.25 N·m | 0.15 N·m |
| load cell | yes (`/force_shear`) | no |
| runs land in | `outputs/button_pressing/` | `outputs/person_avoiding/` |

Both goals are **already placed when the GUI opens** — nothing has to be
dragged to start a run:

| | starts at | fixed by |
|---|---|---|
| **button** (`lin_tip`) | x 0, y 0, **z 0.38 m** | z held at the button's height; set x/y by hand |
| **obstacle** (both Gaussians) | **x −0.05, y 0, z 0.2175 m** | one shared centre; z held at the midpoint between the tips of sections 1 and 2 |

Drag either only to move it, in x/y — the Z sliders are disabled and read
`(fixed)`. Whatever you end up with is recorded in the run's own data.

## Mechanism

| Component | Element | s | Its goal |
|---|---|---|---|
| `lin_tip` | `LinearSpring(K_REACH)` | 1.0 | drag x/y onto the button — z held at `TIP_GOAL_Z` = 0.38 m |
| `gauss_seg1` | `GaussianSpring(0, SIGMA)` | 1/3 | drag x/y onto the obstacle — z held at `GAUSS_GOAL_Z` = 0.2175 m |
| `gauss_seg2` | `GaussianSpring(0, SIGMA)` | 2/3 | **shares `gauss_seg1`'s goal** — drag either, both move |

Torques superpose as `tau = sum_i J_i^T F_i`. The tip spring pulls the arm
toward the button while the two Gaussians push its body away from the obstacle;
both act on the same 9 motors, and how that contest resolves sets how hard the
tip ends up pressing.

Both Gaussians start at whatever strength the run name stands for (see
[Run names](#run-names)), and share it. Because the GUI comes up with
publishing off, that strength does not act until you have placed the centre and
switched publishing on — which matters, since a Gaussian switched on at full
strength peaks immediately if its centre is already within `SIGMA` of the arm.

`GAUSS_GOAL_Z` is derived, not typed: z is linear in s on the straight arm, so
the midpoint between the tips of sections 1 (s=1/3) and 2 (s=2/3) is exactly
`rest_pose(0.5)`. Change `REST_LENGTH` and it follows. It also puts both
attachment points the same distance from the centre — 0.088 m at the rest pose,
for the pre-placed x — so the two Gaussians push equally hard to begin with.

Aim the tip goal a little *past* the contact surface. Parked exactly on it, the
steady-state spring error — and the contact force with it — decays to zero once
the tip arrives.

## Parameters

Set in the config; the spring and damper values are also live sliders in the GUI.

| | `_button` | `_person` |
|---|---|---|
| `K_REACH` | 10 N/m | 10 N/m |
| `K_GAUSS` (start) | 0 N/m | 0 N/m |
| `SIGMA` / `C_DAMP` | 0.06 m / 1.0 N·s/m | 0.06 m / 1.0 N·s/m |
| `TAU_LIMIT` | 0.25 N·m | 0.15 N·m |

`K_REACH` was 25 N/m and made the tip oscillate; 10 is the current value. The
person variant may never be stiffer than the button one — it keeps the lower
`TAU_LIMIT` as its extra margin.

## Run names

Nothing is recorded here yet. Use these, so the config, this README and the
notebook agree on what a run is called:

| `--run` | Gaussian strength | |
|---|---|---|
| `no_gauss` | 0 N/m | baseline — reach only |
| `mid_1` | 5 N/m | |
| `mid_2` | 10 N/m | |
| `strong` | 20 N/m | around the reach/repulsion balance |

**The name sets the strength.** `GAUSS_CONDITIONS` in the config maps each name
to a value, and `--run` applies it to both Gaussians — in the torque node and on
the GUI sliders alike, so the two agree from the first tick and there is no
slider work to set up a named run:

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _button --run mid_1
python3 ../VMC/GUI_MultiVMC.py        --config _button --run mid_1
```

The sliders stay live — move off the condition whenever you want, and
`k_params` in the saved data records what was actually applied, tick by tick.

Both configs use the same four. For `_person`, **record `strong` first** and
confirm the arm stops clear before running anything weaker.

For scale: at `K_REACH = 10` the reach and repulsion torques crossed at
**19.3 N/m** for the geometry these were sized against, so `strong` sits about
at that balance and the rest below it. With the obstacle now placed by hand
that crossover moves with it — read it off the live torque plot rather than
trusting the number.

Anything not in the table is a scratch name with no strength attached:
`--run tuning` starts from the config's own value and overwrites itself every
time, which is what you want while feeling out gains.

## Gravity direction

The arm is **rotated onto its side** for both experiments, so gravity pulls
along **-y** in the robot frame, not the upright **-z**. Both configs set
`GRAVITY_DIR = "-y"`, and the torque node logs the direction on startup —
confirm that line matches the real mounting before recording. Override for one
run with `--gravity-dir -z`.

## Running

```bash
conda activate finger
cd ButtonPressing

# Terminal 1 — hardware
bash ../start_arm.sh

# Terminal 2 — load cell (_button only)
python3 ../LoadCell/arduino_force.py

# Terminal 3 — torque node, naming the run
python3 ../VMC/GUI_MultiVMC_Torque.py --config _button --run mid_1

# Terminal 4 — the GUI
python3 ../VMC/GUI_MultiVMC.py --config _button
```

Both terminals must name the same variant; the torque node logs what it loaded:

```
[INFO] config: GUI_MultiVMC_Config_button
[INFO] run: mid_1 -- saving to .../outputs/button_pressing/mid_1.{png,npz}, overwriting any previous mid_1
[INFO] gravity compensation: gravity pulls along -y in the robot frame
```

The GUI comes up with **publishing off** (`PUBLISH_ON_START = False`), so
nothing moves while you set up. Then:

1. **The obstacle is already at x = −0.05, y = 0** (`GAUSS_GOAL_XY`), shared
   by both Gaussians — nothing to do unless you want it elsewhere, in which
   case drag either one and the other follows. x/y only; the Z slider is
   disabled and reads `(fixed)`.
2. **Place the button.** Drag `lin_tip` to it — x/y only, z fixed at 0.38 m.
   This is the one placement a run actually needs.
3. **Switch publishing on.** *Now* the arm moves, once, to the goals you set.
   The tip goal starts 5.5 cm below the resting tip, so expect it to take up
   that slack immediately (≈0.55 N at `K_REACH = 10`).
4. **Check the repulsion.** Both `strength` sliders already sit at the run's
   condition. Move them if you want to explore — they publish live, and what
   was applied is recorded in `k_params`.
5. **Leave it alone.** The longest stretch where you touch nothing is what the
   notebook reads as the measurement window.
6. **Ctrl+C terminal 3 — once.** That writes the run.

Swap `_button` for `_person` to run the other variant, and drop terminal 2.

If the tip rings instead of settling, lower its `stiffness` slider until it
does, and write the value that works into `K_REACH`. Damping (`C_DAMP`) is the
other lever.

## Data

`--run NAME` writes straight to `outputs/<folder>/NAME.{npz,png}` and
**overwrites** a previous run of that name, so repeating a run replaces it.
Without `--run` the node falls back to a timestamped `experiments/z_<stamp>`
pair, which `collect_run.py <folder> <name>` can move afterwards.

Saving needs `SAVE_PLOT`/`SAVE_DATA` `True` and `COLLECTED_DATA` `False` — that
is how both configs ship. Set `COLLECTED_DATA = True` once the reported runs
are recorded and nothing will overwrite them again.

It happens **on exit only**, from the node's `finally` block, so:

- **Ctrl+C once, then wait.** A second Ctrl+C during the save truncates the
  file — that is exactly how `experiments/z_25_1434.png` ended up a broken PNG
  with no data beside it. The `.npz` is now written *before* the plot, so an
  interrupt costs the figure rather than the run, and both writes report
  failure loudly instead of silently.
- **`kill -9` loses everything.**
- The node writes relative to its launch directory, so start it from
  `ButtonPressing/`.

Every control tick is logged, whoever publishes the reference:

| field | shape | what |
|---|---|---|
| `t` | (n,) | seconds since the first reference message |
| `goal` | (n, N_VC, 3) | reference trajectory per component [m] — *where you dragged things* |
| `comp_pos` | (n, N_VC, 3) | actual FK position of each component [m] |
| `q_motor` / `q_motor_dot` | (n, 9) | joint positions [rad], velocities [rad/s] |
| `tau` | (n, 9) | commanded torque, post-clip [N·m] |
| `tau_grav` | (n, 9) | the gravity-compensation share of it [N·m] |
| `force` | (n,) | load-cell shear, raw **grams** — ×`9.80665e-3` for N |
| `f_virtual` | (n, 3) | task-space virtual force [N] |
| `k_params` | (n, N_VC) | each component's spring parameter over time |
| `comp_names`, `comp_s_params`, `control_freq`, `tau_limit`, `gravity_dir` | — | metadata |

## Analysis

`plot_button_and_person.ipynb` discovers whatever is in `outputs/`, so runs do
not have to be named in advance. Nothing about the analysis is hardcoded: the
measurement window is the longest stretch where the tip goal and the Gaussian
strength both hold still, the attachment points come from `comp_s_params`, and
the button and obstacle positions are read back out of the logged reference —
the only place a GUI-dragged position exists. Figures are written next to the
runs as PDF + SVG.

## Files

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config_button.py` | the three elements and their values; load cell in the loop |
| `GUI_MultiVMC_Config_person.py` | the same, lower torque limit, no load cell |
| `collect_run.py` | moves an unnamed `experiments/z_*` pair into `outputs/` — unnecessary when using `--run` |
| `plot_button_and_person.ipynb` | analysis for both |

The scripted state machine that used to drive this folder — `button_pressing.py`,
`person_avoiding.py`, `_reach_avoid_driver.py`, `_geometry_check.py` — was
removed when the positions moved into the GUI. It is in the git history if the
timed sequence is ever wanted back.
