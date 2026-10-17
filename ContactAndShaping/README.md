# ContactAndShaping — Full-body shaping and surface following

Several virtual elements acting at once on different points along the arm: a
tip spring and a body Gaussian that shape the whole arm's configuration, a
compliant tip spring that lets the arm slide along a surface, and distributed
springs compared with a single tip spring.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/sponge/` | End-effector pressed against a sponge; sweep contact stiffness k. Only the raw `.npz` and the torque node's per-run snapshot PNGs are here; no notebook in the repo turns this data into a figure. |
| `outputs/surface_tracking/` | Surface following: a linear tip spring pulls the arm towards a goal with a slanted rigid surface in the way, and the arm slides along the surface until it clears its edge. One run per stiffness (`k5` … `k130`). `k8.xlsx`, `k70.xlsx` and `k_compare.xlsx` are tip-marker trajectories from video tracking, hand-corrected; `plot_surface_tracking.ipynb` reads `k_compare.xlsx` |
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
| `plot_three_spring_vs_one.ipynb` | `filtered.{pdf,svg}`: backbone posture (X–Z) 5 s before and after the two body springs are switched on next to the tip spring |
| `plot_surface_tracking.ipynb` | `energy_comparison` (actuator work, K = 8 vs. 70 N/m, from the run logs), and `tracking_x`, `tracking_y` and `k_compare_xy` (tip position from video tracking, `k_compare.xlsx`) in `outputs/surface_tracking/` |
| `plot_full_body_shaping.ipynb` | X–Z backbone of **every** run in `outputs/body_shaping/` (with `GRAVITY_DIR = "-y"` in `_bending`, the X–Z plane is the plan view from above): `all_backbones.{pdf,svg}` with all runs overlaid, plus one `<run>_backbone.{pdf,svg}` per run, all on the same axes. Drawn from the motor encoders (FK on `q_motor`), not the virtual model's internal reference; each run's K and A are read from the run itself |

## Whole-body shaping: recording a case

One config (`_bending`), so the only thing that changes between cases is the
name after `--run`.

**Terminal 1** — the arm (blocks; leave it running):

```bash
bash start_arm.sh        # from the repository root
```

**Terminal 2** — the torque node; `<name>` is the case, and is also the name
of the saved file (`outputs/body_shaping/<name>.npz`):

```bash
cd ContactAndShaping
python3 ../VMC/GUI_MultiVMC_Torque.py --config _bending --run <name>
```

**Terminal 3** — the GUI, once Terminal 2 is up:

```bash
cd ContactAndShaping
python3 ../VMC/GUI_MultiVMC.py --config _bending
```

In the GUI:

1. Drag each goal in X/Y and set its **Z** with the vertical slider (0 – 0.45 m;
   the arm is 0.435 m long straight, and `bending_1`, `_2` and `_6` left the tip
   goal's Z at 0.435, i.e. fully extended — lowering it is what bends the arm
   more, as in `bending_3` and `_4` at Z = 0).
2. Set the tip spring's stiffness **K** and the Gaussian's strength **A** with
   the sliders. The Gaussian starts at A = 0: move its goal away from the arm
   first, then raise A (it pushes hardest when its attachment point is one
   sigma from the goal).
3. Wait until the arm has settled — the notebook takes the mean of the last 3 s.

To finish: press **Ctrl+C once** in Terminal 2 and wait for it to write
`<name>.npz` (do not press it again). Then stop the GUI and Terminal 1. Check
the file exists:

```bash
ls outputs/body_shaping/
```

To plot: open `plot_full_body_shaping.ipynb` and run all cells. It finds every
run in the folder, prints each run's K, A and goals, and saves the overlay and
one figure per run.

Names used so far:

| `<name>` | What it is |
|---|---|
| `bending_1` | Tip goal dragged to the right (+X). K = 50 N/m, A = 2000 N/m; tip goal (34.3, −3.9, 43.5) cm, Gaussian goal (4.5, −0.3, 14.5) cm |
| `bending_2` | Tip goal dragged to the left (−X). K = 50 N/m, A = 2000 N/m; tip goal (−16.7, 1.6, 43.5) cm, Gaussian goal (−6.7, 1.9, 14.5) cm |
| `bending_3` | K = 50 N/m, A = 1500 N/m; tip goal (36.7, −3.4, **0.0**) cm, Gaussian goal (7.1, −0.5, 8.9) cm |
| `bending_4` | K = 50 N/m, A = 1500 N/m; tip goal (38.0, −7.4, **0.0**) cm, Gaussian goal (−3.7, 0.5, 19.2) cm |
| `bending_5` | K = **100** N/m, A = 1500 N/m; tip goal (4.5, −0.6, 41.7) cm, Gaussian goal (0.0, 0.0, 14.5) cm |
| `bending_6` | K = 100 N/m, A = 1800 N/m; tip goal (3.3, 0.2, 43.5) cm, Gaussian goal (1.5, 0.1, 14.5) cm |

Read from each run's last 3 s (goals, `k_params`), which is what the notebook
uses; `bending_1..6` all have a photo, `media/full_body_shaping/bending_<n>.jpg`.
`bending_1` and `_2` left the tip goal fully extended (Z = 43.5 cm); `_3` and
`_4` put it at Z = 0, and `_5` and `_6` aim it almost straight ahead (x ≈ 3–5 cm). **A is a stiffness in
N/m**, not a force (see [the Gaussian's force law](../GaussianElement/README.md)).
Use the same name for a case's data
(`outputs/body_shaping/<name>.npz`), its photo
(`media/full_body_shaping/<name>.jpg`) and its plots (`<name>_backbone.*`).

Watch out for:

- **Re-using a name overwrites that run.** This config ships
  `COLLECTED_DATA = False` on purpose (it is what lets a run save at all, and
  every case is a new name), so nothing protects an existing file — `ls` first.
- **Forgetting `--run`** sends the run to `experiments/z_<stamp>/` instead, and
  the notebook will not see it.
- This config sets `GRAVITY_DIR = "-y"` itself (the arm mounted on its side), so
  no `--gravity-dir` flag is needed.

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

`_surface` and `_three_springs` were recorded with the arm **upright**. They
don't set `GRAVITY_DIR`, so the torque node compensates for an upright arm
(`-z`). To reproduce that data, mount the arm upright. On an arm mounted on
its side, pass `--gravity-dir -y`. Otherwise the compensation pushes along an
axis gravity no longer acts on. Expect the numbers to differ from the recorded
runs either way, since the arm's own weight loads it differently.

`_bending` is the exception: it was recorded with the arm on its side and sets
`GRAVITY_DIR = "-y"` directly, so no `--gravity-dir` flag is needed.

## Running the other experiments

`GUI_MultiVMC.py`/`GUI_MultiVMC_Torque.py` always import a module literally named
`GUI_MultiVMC_Config` from the current directory. This folder holds several
named variants (no single unambiguous default), so pass `--config` to pick one
rather than copying it into place. Body shaping (`_bending`) is in
[its own section](#whole-body-shaping-recording-a-case); the other two:

```bash
cd ContactAndShaping

# Surface tracking -- every command names the same variant:
python3 ../VMC/GUI_MultiVMC_Torque.py --config _surface &
python3 ../VMC/GUI_MultiVMC.py --config _surface

# Three-spring vs. one comparison (separate run):
python3 ../VMC/GUI_MultiVMC_Torque.py --config _three_springs &
python3 ../VMC/GUI_MultiVMC.py --config _three_springs
```
