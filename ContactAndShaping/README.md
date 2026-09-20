# ContactAndShaping Experiments

Experiments involving physical contact and body shaping — multiple virtual elements acting simultaneously on different points along the arm.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/sponge/` | End-effector pressed against a sponge; sweep contact stiffness k. Only raw `.npz` + auto-generated `GUI_MultiVMC_Torque.py` per-run snapshot PNGs exist here — **no analysis notebook in the repo processes this data into a comparison figure**. |
| `outputs/surface_tracking/` | Arm slides along a rigid surface with a constrained spring |
| `outputs/body_shaping/` | Whole-body shaping / bending: a Gaussian on segment 1 (`gauss_seg1`) and a linear spring on the tip (`lin_seg3`), goals and strengths tuned live from the GUI. **One config for every case** (`GUI_MultiVMC_Config_bending.py`); a case is just the name you give `--run`. Only each run's settled final pose is analyzed (`plot_full_body_shaping.ipynb`), not a time series. See [Whole-body shaping](#whole-body-shaping-recording-a-case). `old/whole_body_shaping_bad.*` is an earlier discarded run from before this config existed. |
| `outputs/three_spring_vs_one/` | Comparison: 1 end-effector spring vs. 3 distributed springs for posture |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config_bending.py` | Whole-body shaping / bending, **all cases in one file** (`gauss_seg1` on segment 1 + `lin_seg3` on the tip, everything set in the GUI) (`--config _bending`) |
| `GUI_MultiVMC_Config_surface.py` | VMC config for surface-tracking experiment (`--config _surface`) |
| `GUI_MultiVMC_Config_three_springs.py` | VMC config for three-spring comparison (`--config _three_springs`) |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_three_spring_vs_one.ipynb` | Posture error comparison: 1 vs. 3 springs |
| `plot_surface_tracking.ipynb` | XY/X-position trajectory comparisons, soft-vs-stiff energy comparison, and per-k Y-position tracking across the `outputs/surface_tracking/` k-sweep |
| `plot_full_body_shaping.ipynb` | Side-view (X-Z) backbone of **every** run in `outputs/body_shaping/`: `all_backbones.{pdf,svg}` with all runs overlaid, plus one `<run>_backbone.{pdf,svg}` per run, all on the same axes. Drawn from the motor encoders (FK on `q_motor`), not the virtual model's internal reference; each run's K and A are read from the run itself |

## Whole-body shaping: recording a case

One config (`_bending`), so the only thing that changes between cases is the
name after `--run`.

**Terminal 1** — the arm (blocks; leave it running):

```bash
cd ~/github/SoftVMC
bash start_arm.sh
```

**Terminal 2** — the torque node; `<name>` is the case, and is also the name
of the saved file (`outputs/body_shaping/<name>.npz`):

```bash
cd ~/github/SoftVMC/ContactAndShaping
python3 ../VMC/GUI_MultiVMC_Torque.py --config _bending --run <name>
```

**Terminal 3** — the GUI, once Terminal 2 is up:

```bash
cd ~/github/SoftVMC/ContactAndShaping
python3 ../VMC/GUI_MultiVMC.py --config _bending
```

In the GUI:

1. Drag each goal in X/Y and set its **Z** with the vertical slider (0 – 0.45 m;
   the arm is 0.435 m long straight, and both recorded runs so far left the tip
   goal's Z at 0.435, i.e. fully extended — lowering it is what bends the arm
   more).
2. Set the tip spring's stiffness **K** and the Gaussian's strength **A** with
   the sliders. The Gaussian starts at A = 0: move its goal away from the arm
   first, then raise A (it pushes hardest when its attachment point is one
   sigma from the goal).
3. Wait until the arm has settled — the notebook takes the mean of the last 3 s.

To finish: press **Ctrl+C once** in Terminal 2 and wait for it to write
`<name>.npz` (do not press it again). Then stop the GUI and Terminal 1. Check
the file exists:

```bash
ls ~/github/SoftVMC/ContactAndShaping/outputs/body_shaping/
```

To plot: open `plot_full_body_shaping.ipynb` and run all cells. It finds every
run in the folder, prints each run's K, A and goals, and saves the overlay and
one figure per run.

Names used so far:

| `<name>` | What it is |
|---|---|
| `linear_left` | Tip goal dragged to the left (−X) |
| `linear_right` | Tip goal dragged to the right (+X) |
| `bending` | More bending: tip goal also pulled down in Z |
| `left`, `right` | The first two recordings, made before the configs were merged. Both K = 50 N/m, A = 2000 N; tip goal (−16.7, 1.6, 43.5) cm and (34.3, −3.9, 43.5) cm |

Watch out for:

- **Re-using a name overwrites that run.** This config ships
  `COLLECTED_DATA = False` on purpose (it is what lets a run save at all, and
  every case is a new name), so nothing protects an existing file — `ls` first.
- **Forgetting `--run`** sends the run to `experiments/z_<stamp>/` instead, and
  the notebook will not see it.
- The arm is mounted on its side, so this config sets `GRAVITY_DIR = "-y"`
  itself; no `--gravity-dir` flag is needed.

## Run names

What is recorded here, and the name `--run` needs to write each one:

| run | `--run` names |
|---|---|
| `outputs/surface_tracking/` (`--config _surface`) | `k130` · `k20` · `k20_new` · `k25` · `k25_old` · `k30` · `k30_new` · `k5` · `k70` · `k70_new` · `k8` |
| `outputs/three_spring_vs_one/` (`--config _three_springs`) | `filtered` · `raw` · `raw2` · `raw_with_gauss` |
| `outputs/body_shaping/` (`--config _bending`) | any name you choose — see [Whole-body shaping](#whole-body-shaping-recording-a-case) |

`outputs/sponge/` holds earlier data that no current config points at, so
`--run` cannot route there; it is kept as a record. In `outputs/body_shaping/`,
`old/whole_body_shaping_bad.*` and the top-level `whole_body_shaping.*` predate
the current config and are likewise kept records (the notebook skips the latter).

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config <variant> --run <name>
```

That writes `outputs/<folder>/<name>.{npz,png}` on Ctrl+C, overwriting the run
of that name — the folder comes from `OUTPUT_FOLDER` in the config. `_surface`
and `_three_springs` ship `COLLECTED_DATA = True`, so nothing is written at
all until you set it `False`; that flag is what keeps their recorded data
safe. `_bending` ships `COLLECTED_DATA = False`, see above. Names starting `old_` are discarded runs.

## Gravity direction

`_surface` and `_three_springs` don't set `GRAVITY_DIR`, so the torque node
compensates for an **upright** arm (`-z`) — which is how those runs were
recorded. The arm is currently mounted **on its side** for `ButtonPressing/`.
To reproduce that data, remount it upright; if you run it on the side anyway,
pass `--gravity-dir -y` or the compensation pushes along an axis gravity no
longer acts on, and expect the numbers to differ from the recorded runs
either way, since the arm's own weight loads it differently.

`_bending` is the exception: it sets `GRAVITY_DIR = "-y"` directly,
since it was written for the arm's current side-mounted state — no
`--gravity-dir` flag needed for it.

## Running the other experiments

`GUI_MultiVMC.py`/`GUI_MultiVMC_Torque.py` always import a module literally named
`GUI_MultiVMC_Config` from the current directory. This folder holds several
named variants (no single unambiguous default), so pass `--config` to pick one
rather than copying it into place. Body shaping (`_bending`) is in
[its own section](#whole-body-shaping-recording-a-case); the other two:

```bash
conda activate finger
cd ContactAndShaping

# Surface tracking -- every command names the same variant:
python3 ../VMC/GUI_MultiVMC_Torque.py --config _surface &
python3 ../VMC/GUI_MultiVMC.py --config _surface

# Three-spring vs. one comparison (separate run):
python3 ../VMC/GUI_MultiVMC_Torque.py --config _three_springs &
python3 ../VMC/GUI_MultiVMC.py --config _three_springs
```
