# ButtonPressing Experiments

Covers both remaining planned experiments. Neither has been run yet — no data exists in this folder.
Structurally, Fig 9 and Fig 10 are **the same experiment** — drive the tip to a
target with a bounded press spring, while a Gaussian repels it away — just
pointed at a different target (button vs. person) with different framing
(quantitative sweep vs. qualitative demo). One notebook (`plot_button_and_person.ipynb`)
covers both.

## The mechanism (shared by both figures)

Two virtual components summed at the arm tip (`s_param=1.0`), both targeting
the same point, `tau = J1^T@F1 + J2^T@F2`:

| Component | Element | Role |
|---|---|---|
| `press` | `TanhSpring(stiffness, max_force)` | Drives the tip into the target. Saturates at `max_force`, so that value *is* the calibrated force cap — bounded/safe regardless of the component below. |
| `gauss_repel` | `GaussianSpring(strength, sigma)` | Repels the tip away from the target. Sweeping `strength` from 0 (full contact) up to a value that fully cancels the approach (no contact) trades off against `press`. |

Only `press.stiffness` and `gauss_repel.strength` are live-tunable — those are
the *first* constructor argument of each spring, and `GUI_MultiVMC_Torque.py`'s
`/VC_Param` handler only `setattr()`s that one attribute (see
`GUI_MultiVMC_Config.py`'s header note). `press.max_force` and `gauss_repel.sigma`
are fixed by the constructor calls in `GUI_MultiVMC_Config.py` — change those by
editing the file and restarting the torque node, not via any slider or script flag.

## Fig 9 — Exploiting Contact for Button Pressing

Sweeping `gauss_repel`'s strength (0 → high) against a fixed `press` produces
the run family: **no-contact → middle → middle → full-contact**. Contact force
is read from the load cell's **shear** channel (`/force_shear`) — the arm is
mounted horizontally against the load-cell structure, so shear, not normal
force, is the meaningful signal here. The visible "button" itself is cosmetic
(for photos) — the actual measurement structure is the external load cell the
arm moves onto and presses against.

## Fig 10 — Demo with Person

Qualitative photo/video demo only (paper's softer "C. Demo" section, medical
framing) — arm avoids or gently touches a person. No quantitative logging is
required for the figure itself, but `GUI_MultiVMC_Torque.py` still auto-saves
an `.npz` per run (harmless — the notebook's Fig 10 section treats it as
optional supporting data, not the main deliverable). Conservative `TAU_LIMIT`
applies as the safety bound regardless.

## Files

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config: `press` + `gauss_repel` components described above |
| `_press_repel_driver.py` | Shared headless ROS 2 driver (approach → ramp gauss strength → hold state machine) used by both scripts below. Not run directly. |
| `button_pressing.py` | Fig 9 entry point — hardcoded target (`button_pos`), `--gauss-strength` required per run. |
| `person_avoiding.py` | Fig 10 entry point — hardcoded/placeholder `PERSON_TARGET` (override with `--target X Y Z`), `--gauss-strength` required per run, extra safety notes in its docstring. |
| `plot_button_and_person.ipynb` | Single analysis notebook for both figures — see below. Loading/plotting code is smoke-tested against synthetic fixtures (correct shapes, renders cleanly) but **has not seen real data yet**. |

## Notebook

`plot_button_and_person.ipynb` makes exactly two figures for Fig 9 (Fig 10 stays photo/video-only, no plot):

| Figure | Shows |
|---|---|
| `fig9_force_vs_time.pdf` | Contact force (N) vs. time, all 4 sweep conditions overlaid — the paper's required quantitative result. |
| `fig9_pose.pdf` | FK backbone (X-Z) at steady state per condition, plus the button position — shows spatially how far the tip is held back as `gauss_repel` strength increases. |

## Running

Two equally-feasible paths — pick whichever suits a given session:

**A — GUI (live-tuning, drag goals by hand):**
```bash
conda activate finger
cd ButtonPressing

# Terminal 1 -- hardware
bash ../start_arm.sh

# Terminal 2 -- torque node
python3 ../VMC/GUI_MultiVMC_Torque.py &

# Terminal 3 -- GUI
python3 ../VMC/GUI_MultiVMC.py
```
Use this first, to find safe/working `press.stiffness` and `gauss_repel.strength`
values live while watching `/force_shear` — both spring parameters start
conservative/untested (`press` stiffness=20, max_force=1.0; `gauss_repel`
strength=0.0). Keep both components' goal markers coincident (drag one, drag
the other to match) since the headless scripts below assume that.

**B — headless scripts (hardcoded target + hardcoded gauss strength per run):**
```bash
conda activate finger
cd ButtonPressing

# Terminal 1 -- hardware
bash ../start_arm.sh

# Terminal 2 -- torque node
python3 ../VMC/GUI_MultiVMC_Torque.py &

# Terminal 3 -- replaces GUI_MultiVMC.py
python3 button_pressing.py --gauss-strength 0        # full contact
python3 button_pressing.py --gauss-strength 1500     # no contact -- UNVERIFIED, tune live (path A) first
# ...and similarly for person_avoiding.py once PERSON_TARGET is measured.
```
Once path A has found real numbers, lock them into the commands above (or
into `--press-stiffness`) for the actual recorded/reported runs — this is
more reproducible than a live drag for the runs that go in the paper.
`GUI_MultiVMC_Torque.py` only auto-saves once, on exit, so each of the 4 sweep
conditions needs its own process run (restart the torque node between
conditions) rather than one continuous session — same constraint under both
paths A and B.

## Data

Saved `.npz` files go to `outputs/button_pressing/` and `outputs/person_avoiding/`
(both created, currently empty). Rename each auto-saved timestamped file to
something descriptive right after saving — e.g. `no_contact.npz` /
`middle_1.npz` / `middle_2.npz` / `full_contact.npz` — so
`plot_button_and_person.ipynb` can find them by name. The `force` field is the
raw load-cell reading in **grams**; multiply by `9.80665e-3` to get Newtons
(see `LinearSpring/plot_k_saturation.ipynb` for the verified precedent —
`GUI_MultiVMC_Torque.py`'s own inline comment was wrong about this until it
was corrected alongside this experiment).

`GUI_MultiVMC_Config.py` has `COLLECTED_DATA = False` — the one exception in
this repo, since real data hasn't been recorded here yet (see root
`README.md`'s Notes for what the flag does). Once the actual reported runs
are saved and renamed, flip it to `True` so a later demo/rerun can't silently
overwrite them.
